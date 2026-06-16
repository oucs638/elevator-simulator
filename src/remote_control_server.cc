// Implements the newline-delimited TCP protocol for remote elevator control.

#include "remote_control_server.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <optional>
#include <sstream>
#include <vector>

namespace elevator_simulator {
namespace {

// Normalizes a remote command line before server-side parsing.
std::string Trim(const std::string& text) {
  const auto begin = text.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }

  const auto end = text.find_last_not_of(" \t\r\n");
  return text.substr(begin, end - begin + 1);
}

// Sends the entire server response over a connected client socket.
bool SendAll(int socket_fd, const std::string& text) {
  const char* data = text.c_str();
  std::size_t remaining = text.size();

  // send() may accept only part of the response, so retry the remaining bytes.
  while (remaining > 0) {
    const ssize_t sent = send(socket_fd, data, remaining, 0);
    if (sent <= 0) {
      return false;
    }

    data += sent;
    remaining -= static_cast<std::size_t>(sent);
  }

  return true;
}

// Converts internal elevator stages into stable remote protocol text.
std::string StageText(ElevatorStage stage) {
  switch (stage) {
    case ElevatorStage::kIdle:
      return "idle";
    case ElevatorStage::kToPickup:
      return "to-pickup";
    case ElevatorStage::kBoarding:
      return "boarding";
    case ElevatorStage::kToDestination:
      return "to-destination";
    case ElevatorStage::kArrived:
      return "arrived";
    case ElevatorStage::kStopped:
      return "stopped";
  }
  return "unknown";
}

// Converts signed movement direction into stable remote protocol text.
std::string DirectionText(int direction) {
  if (direction > 0) {
    return "up";
  }
  if (direction < 0) {
    return "down";
  }
  return "stopped";
}

// Formats active or queued requests for compact remote status output.
std::string RequestText(const ElevatorRequest& request) {
  if (request.type == ElevatorRequestType::kDirectSend) {
    return "send->" + std::to_string(request.destination);
  }
  return std::to_string(request.current) + "->" +
         std::to_string(request.destination);
}

// Serializes elevator snapshots into the remote dashboard status protocol.
std::string StatusText(const ElevatorSystem& system) {
  std::ostringstream stream;
  stream << "Elevator bank status\n";

  for (const auto& snapshot : system.Snapshots()) {
    // Emit one pipe-delimited row per elevator.
    stream << "E" << snapshot.id << " | floor " << snapshot.current_floor
           << " | direction " << DirectionText(snapshot.direction)
           << " | stage " << StageText(snapshot.stage);

    // Target is omitted when the elevator has no active travel target.
    if (snapshot.target_floor.has_value()) {
      stream << " | target " << *snapshot.target_floor;
    }

    // Active request is explicit so the client can show "--" when idle.
    if (snapshot.active_request.has_value()) {
      stream << " | active " << RequestText(*snapshot.active_request);
    } else {
      stream << " | active --";
    }

    // Queue entries share one comma-separated field in the status row.
    stream << " | queue ";
    if (snapshot.queued_requests.empty()) {
      stream << "--";
    } else {
      for (std::size_t index = 0; index < snapshot.queued_requests.size();
           ++index) {
        if (index > 0) {
          stream << ",";
        }
        stream << RequestText(snapshot.queued_requests[index]);
      }
    }
    stream << "\n";
  }

  return stream.str();
}

// Executes a remote passenger call and formats the activity event response.
std::string CallText(ElevatorSystem* system, int current, int destination) {
  const auto result = system->DispatchNearest(current, destination);
  if (!result.accepted) {
    return result.message + "\n";
  }

  // EVENT rows are rendered as Recent Activity by the remote dashboard.
  std::ostringstream stream;
  stream << "EVENT|" << result.elevator_id << "|Call " << current << " -> "
         << destination << " assigned to E" << result.elevator_id << " | ETA "
         << result.estimated_wait_seconds << "s\n";
  return stream.str();
}

// Parses numeric command forms and rejects nonnumeric trailing tokens.
std::optional<std::vector<int>> ParseIntegers(const std::string& command) {
  std::istringstream stream(command);
  std::vector<int> values;
  int value = 0;

  while (stream >> value) {
    values.push_back(value);
  }

  // If extraction stopped before EOF, a nonnumeric token was encountered.
  if (!stream.eof()) {
    return std::nullopt;
  }
  return values;
}

// Formats a dispatch result as a remote activity event or error line.
std::string SendText(const DispatchResult& result) {
  if (!result.accepted) {
    return result.message + "\n";
  }

  return "EVENT|" + std::to_string(result.elevator_id) + "|" + result.message +
         " | ETA " + std::to_string(result.estimated_wait_seconds) + "s\n";
}

}  // namespace

// Creates a stopped server bound to the shared elevator system.
RemoteControlServer::RemoteControlServer(ElevatorSystem& system)
    : system_(system), running_(false), server_fd_(-1), port_(0) {}

// Ensures server sockets and worker threads stop before destruction.
RemoteControlServer::~RemoteControlServer() { Stop(); }

// Starts the remote TCP server and launches the accept loop thread.
bool RemoteControlServer::Start(int port, std::string* error) {
  if (error == nullptr) {
    return false;
  }

  // Reject duplicate starts so only one accept thread owns the server socket.
  if (running_) {
    *error = "Remote control server is already running.";
    return false;
  }

  // Use an explicit TCP port that clients can connect to directly.
  if (port <= 0 || port > 65535) {
    *error = "Port must be between 1 and 65535.";
    return false;
  }

  // Convert broken-pipe writes into send() errors instead of process exits.
  std::signal(SIGPIPE, SIG_IGN);

  // Create an IPv4 TCP listening socket.
  const int socket_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_fd < 0) {
    *error =
        "Failed to create server socket: " + std::string(std::strerror(errno));
    return false;
  }

  // Allow quick restarts on the same port after the simulator exits.
  int reuse = 1;
  setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

  // Bind the server to all local IPv4 interfaces on the requested port.
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_ANY);
  address.sin_port = htons(static_cast<uint16_t>(port));

  if (bind(socket_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) <
      0) {
    *error = "Failed to bind port " + std::to_string(port) + ": " +
             std::strerror(errno);
    close(socket_fd);
    return false;
  }

  // Start accepting queued client connections.
  if (listen(socket_fd, 8) < 0) {
    *error = "Failed to listen: " + std::string(std::strerror(errno));
    close(socket_fd);
    return false;
  }

  // Publish server state only after socket setup succeeds.
  server_fd_ = socket_fd;
  port_ = port;
  running_ = true;
  accept_thread_ = std::thread(&RemoteControlServer::AcceptLoop, this);
  return true;
}

// Stops the remote server, wakes blocked sockets, and joins owned threads.
void RemoteControlServer::Stop() {
  if (!running_.exchange(false)) {
    return;
  }

  // Closing the listening socket releases AcceptLoop() before it is joined.
  if (server_fd_ >= 0) {
    shutdown(server_fd_, SHUT_RDWR);
    close(server_fd_);
    server_fd_ = -1;
  }

  if (accept_thread_.joinable()) {
    accept_thread_.join();
  }

  {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    for (const int client_fd : client_fds_) {
      shutdown(client_fd, SHUT_RDWR);
    }
  }

  for (auto& thread : client_threads_) {
    if (thread.joinable()) {
      thread.join();
    }
  }

  client_threads_.clear();
  port_ = 0;
}

// Returns whether the remote server is currently accepting clients.
bool RemoteControlServer::IsRunning() const { return running_; }

// Returns the configured listening port, or 0 before startup.
int RemoteControlServer::Port() const { return port_; }

// Executes one remote command line and returns the server response text.
std::string RemoteControlServer::ExecuteCommand(
    ElevatorSystem* system, const std::string& raw_command) {
  if (system == nullptr) {
    return "No elevator system is available.\n";
  }

  // Normalize socket input before matching command names.
  const std::string command = Trim(raw_command);
  if (command.empty()) {
    return "";
  }

  if (command == "help") {
    return "Use call <from> <to>, send <elevator> <floor>, or quit.\n";
  }

  if (command == "status") {
    return StatusText(*system);
  }

  if (command.rfind("call ", 0) == 0) {
    // call expects exactly two numeric arguments: caller floor and destination.
    const auto values = ParseIntegers(command.substr(5));
    if (!values.has_value() || values->size() != 2) {
      return "Use: call <current floor> <destination floor>\n";
    }

    return CallText(system, (*values)[0], (*values)[1]);
  }

  if (command.rfind("send ", 0) == 0) {
    // send expects exactly two numeric arguments: elevator ID and target floor.
    const auto values = ParseIntegers(command.substr(5));
    if (!values.has_value() || values->size() != 2) {
      return "Use: send <elevator> <floor>\n";
    }

    return SendText(system->SendElevator((*values)[0], (*values)[1]));
  }

  return "Unknown command. Use call <from> <to>, send <elevator> <floor>, or "
         "quit.\n";
}

// Accepts remote clients and starts one handler thread per connection.
void RemoteControlServer::AcceptLoop() {
  while (running_) {
    sockaddr_in client_address{};
    socklen_t client_length = sizeof(client_address);

    // accept() returns a connected socket distinct from the listening socket.
    const int client_fd =
        accept(server_fd_, reinterpret_cast<sockaddr*>(&client_address),
               &client_length);

    if (client_fd < 0) {
      // Stop() closes server_fd_, which wakes accept() with an error.
      if (!running_) {
        break;
      }
      continue;
    }

    // If shutdown raced with accept(), close the new client immediately.
    if (!running_) {
      close(client_fd);
      break;
    }

    {
      // Track clients so Stop() can wake handlers blocked in recv().
      std::lock_guard<std::mutex> lock(clients_mutex_);
      client_fds_.insert(client_fd);
    }

    // Handle each client independently so accept() can keep listening.
    client_threads_.emplace_back(&RemoteControlServer::HandleClient, this,
                                 client_fd);
  }
}

// Handles one connected remote client until it disconnects or quits.
void RemoteControlServer::HandleClient(int client_fd) {
  SendAll(client_fd, "Connected to the live Elevator Simulator.\n");

  std::string pending;
  char buffer[1024];

  while (running_) {
    const ssize_t received = recv(client_fd, buffer, sizeof(buffer), 0);
    if (received <= 0) {
      break;
    }

    // TCP can split or combine commands, so buffer until newline appears.
    pending.append(buffer, static_cast<std::size_t>(received));

    std::size_t newline = pending.find('\n');
    while (newline != std::string::npos) {
      const std::string line = pending.substr(0, newline);
      pending.erase(0, newline + 1);

      // quit is a connection command, not an elevator-system command.
      if (Trim(line) == "quit") {
        SendAll(client_fd, "bye\n");
        shutdown(client_fd, SHUT_RDWR);
        close(client_fd);

        // Stop() also tracks clients, so remove this closed descriptor.
        std::lock_guard<std::mutex> lock(clients_mutex_);
        client_fds_.erase(client_fd);
        return;
      }

      // Delegate command parsing and elevator operations to the command layer.
      const std::string response = ExecuteCommand(&system_, line);
      if (!response.empty() && !SendAll(client_fd, response)) {
        break;
      }

      newline = pending.find('\n');
    }
  }

  shutdown(client_fd, SHUT_RDWR);
  close(client_fd);

  // Remove this client from the shared descriptor set during cleanup.
  std::lock_guard<std::mutex> lock(clients_mutex_);
  client_fds_.erase(client_fd);
}

}  // namespace elevator_simulator
