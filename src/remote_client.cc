// Implements the ncurses dashboard used by the central-control client.

#include <arpa/inet.h>
#include <ncurses.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <charconv>
#include <chrono>
#include <climits>
#include <csignal>
#include <deque>
#include <iostream>
#include <map>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <vector>

namespace {

constexpr int kDefaultPort = 5050;
constexpr int kActivityLimit = 12;

// Fixed x positions for the remote dashboard status table.
constexpr std::array<int, 7> kStatusColumns{2, 11, 18, 30, 47, 57, 70};

// Column labels paired with kStatusColumns by index.
constexpr std::array<std::string_view, 7> kStatusHeaders{
    "Elevator", "Floor", "Direction", "Stage", "Target", "Active", "Queue",
};

// Stores one formatted elevator row for the remote dashboard.
struct CarStatus {
  int id = 0;
  std::string floor = "--";
  std::string direction = "--";
  std::string stage = "--";
  std::string target = "--";
  std::string active = "--";
  std::string queue = "--";
};

// Tracks one remote activity entry and the metadata needed to expire it.
struct ActivityMessage {
  std::string text;
  int elevator_id = 0;
  bool seen_active = false;
  std::chrono::steady_clock::time_point created =
      std::chrono::steady_clock::now();
};

// Shared remote-dashboard state protected between receiver and UI threads.
struct DashboardState {
  std::mutex mutex;
  std::map<int, CarStatus> cars;
  std::deque<ActivityMessage> activity;
  bool connected = true;
};

// Parses the optional remote port and falls back to the simulator default.
int ParsePort(int argc, char* argv[]) {
  if (argc < 3) {
    return kDefaultPort;
  }

  std::istringstream stream(argv[2]);
  int port = 0;
  // Accept only explicit TCP port numbers in the range [1, 65535].
  if (!(stream >> port) || port <= 0 || port > 65535) {
    return kDefaultPort;
  }
  return port;
}

// Opens a TCP client socket to host:port and returns -1 on failure.
int ConnectToServer(const std::string& host, int port) {
  // Zero-initialize unused addrinfo fields before setting lookup hints.
  addrinfo hints{};

  // Allow either IPv4 or IPv6 addresses returned by name resolution.
  hints.ai_family = AF_UNSPEC;

  // Request a TCP stream socket for ordered command and status messages.
  hints.ai_socktype = SOCK_STREAM;

  // getaddrinfo() writes a linked list of candidate addresses into result.
  addrinfo* result = nullptr;
  const std::string port_text = std::to_string(port);

  // Resolve host and port into socket-ready address candidates.
  if (getaddrinfo(host.c_str(), port_text.c_str(), &hints, &result) != 0) {
    return -1;
  }

  int socket_fd = -1;

  // Try every resolved candidate until one socket connects successfully.
  for (addrinfo* item = result; item != nullptr; item = item->ai_next) {
    // Create a socket matching this candidate's address family and protocol.
    socket_fd = socket(item->ai_family, item->ai_socktype, item->ai_protocol);
    if (socket_fd < 0) {
      continue;
    }

    // connect() returns 0 when the TCP connection is established.
    if (connect(socket_fd, item->ai_addr, item->ai_addrlen) == 0) {
      break;
    }

    // Close failed sockets immediately to avoid file descriptor leaks.
    close(socket_fd);
    socket_fd = -1;
  }

  // Release the linked list allocated by getaddrinfo().
  freeaddrinfo(result);
  return socket_fd;
}

// Sends the entire text buffer over a connected socket.
bool SendAll(int socket_fd, const std::string& text) {
  const char* data = text.c_str();
  std::size_t remaining = text.size();

  // TCP send() can write only part of the buffer, so retry the remainder.
  while (remaining > 0) {
    const ssize_t sent = send(socket_fd, data, remaining, 0);
    if (sent <= 0) {
      return false;
    }

    // Advance past the bytes already accepted by the socket.
    data += sent;
    remaining -= static_cast<std::size_t>(sent);
  }

  return true;
}

// Splits protocol text using a fixed, non-empty delimiter.
std::vector<std::string> Split(const std::string& text,
                               const std::string& delimiter) {
  std::vector<std::string> parts;
  std::size_t begin = 0;
  std::size_t end = text.find(delimiter);

  // Repeatedly copy the text before each delimiter.
  while (end != std::string::npos) {
    parts.push_back(text.substr(begin, end - begin));
    begin = end + delimiter.size();
    end = text.find(delimiter, begin);
  }

  // Add the final segment after the last delimiter.
  parts.push_back(text.substr(begin));
  return parts;
}

// Parses an integer only when the entire text is valid.
std::optional<int> ParseInteger(std::string_view text) {
  int value = 0;
  const char* begin = text.data();
  const char* end = begin + text.size();

  const auto result = std::from_chars(begin, end, value);
  if (result.ec != std::errc() || result.ptr != end) {
    return std::nullopt;
  }

  return value;
}

std::string DisplayDirection(const std::string& direction) {
  if (direction == "up") {
    return "Up";
  }
  if (direction == "down") {
    return "Down";
  }
  if (direction == "stopped") {
    return "Stopped";
  }
  return direction;
}

// Converts protocol direction text into remote-dashboard labels.
std::string DisplayStage(const std::string& stage) {
  if (stage == "idle") {
    return "Idle";
  }
  if (stage == "to-pickup") {
    return "Picking up";
  }
  if (stage == "boarding") {
    return "Boarding";
  }
  if (stage == "to-destination") {
    return "In service";
  }
  if (stage == "arrived") {
    return "Arrived";
  }
  return stage;
}

// Adds a bounded recent-activity entry to the remote dashboard.
void AddActivity(DashboardState* state, const std::string& text,
                 int elevator_id = 0) {
  if (text.empty()) {
    return;
  }

  std::lock_guard<std::mutex> lock(state->mutex);

  // New entries appear first; elevator_id links trip messages to car state.
  state->activity.push_front({text, elevator_id});

  // Keep the activity panel bounded by removing the oldest entries.
  while (static_cast<int>(state->activity.size()) > kActivityLimit) {
    state->activity.pop_back();
  }
}

// Updates activity lifetimes using the latest car status.
// The caller must already hold state->mutex.
void UpdateActivityStateLocked(DashboardState* state) {
  const auto now = std::chrono::steady_clock::now();

  // Mark car-linked activity after status polling observes the trip start.
  for (auto& activity : state->activity) {
    if (activity.elevator_id == 0) {
      continue;
    }

    const auto car = state->cars.find(activity.elevator_id);
    if (car == state->cars.end()) {
      continue;
    }

    const bool idle = car->second.stage == "Idle" && car->second.queue == "--";
    if (!idle) {
      activity.seen_active = true;
    }
  }

  // Remove expired informational messages and completed car-linked messages.
  state->activity.erase(
      std::remove_if(state->activity.begin(), state->activity.end(),
                     [state, now](const ActivityMessage& activity) {
                       const auto age = now - activity.created;
                       if (activity.elevator_id == 0) {
                         return age > std::chrono::seconds(5);
                       }

                       const auto car = state->cars.find(activity.elevator_id);
                       if (car == state->cars.end()) {
                         return false;
                       }

                       // Keep trip messages until the car returns to idle.
                       const bool idle = car->second.stage == "Idle" &&
                                         car->second.queue == "--";
                       return idle && (activity.seen_active ||
                                       age > std::chrono::seconds(3));
                     }),
      state->activity.end());
}

// Parses one server status line into the remote dashboard car table.
bool UpdateCarStatus(DashboardState* state, const std::string& line) {
  // Status lines use the server's stable, pipe-delimited wire format.
  if (line.size() < 2 || line[0] != 'E' ||
      line.find(" | floor ") == std::string::npos) {
    return false;
  }

  const auto parts = Split(line, " | ");
  if (parts.empty()) {
    return false;
  }

  CarStatus car;

  // Parse the leading token, such as "E3", into a numeric elevator ID.
  const auto car_id = ParseInteger(std::string_view(parts[0]).substr(1));
  if (!car_id.has_value()) {
    return false;
  }
  car.id = *car_id;

  // Map recognized wire-format fields into display-ready table columns.
  for (std::size_t index = 1; index < parts.size(); ++index) {
    const std::string& part = parts[index];
    if (part.rfind("floor ", 0) == 0) {
      car.floor = part.substr(6);
    } else if (part.rfind("direction ", 0) == 0) {
      car.direction = DisplayDirection(part.substr(10));
    } else if (part.rfind("stage ", 0) == 0) {
      car.stage = DisplayStage(part.substr(6));
    } else if (part.rfind("target ", 0) == 0) {
      car.target = part.substr(7);
    } else if (part.rfind("active ", 0) == 0) {
      car.active = part.substr(7);
    } else if (part.rfind("queue ", 0) == 0) {
      car.queue = part.substr(6);
    }
  }

  std::lock_guard<std::mutex> lock(state->mutex);

  // Store the latest row, then let activity cleanup observe the new state.
  state->cars[car.id] = car;
  UpdateActivityStateLocked(state);
  return true;
}

// Receives server messages and updates the remote dashboard state.
void ReceiveLoop(int socket_fd, DashboardState* state,
                 std::atomic<bool>* running) {
  std::string pending;
  char buffer[1024];

  while (*running) {
    // Read the next chunk from the TCP stream.
    const ssize_t received = recv(socket_fd, buffer, sizeof(buffer), 0);
    if (received <= 0) {
      break;
    }

    // TCP can split or combine messages, so keep bytes until a full line
    // exists.
    pending.append(buffer, static_cast<std::size_t>(received));
    std::size_t newline = pending.find('\n');

    while (newline != std::string::npos) {
      const std::string line = pending.substr(0, newline);
      pending.erase(0, newline + 1);

      // EVENT lines update Recent Activity instead of the status table.
      if (line.rfind("EVENT|", 0) == 0) {
        const auto parts = Split(line, "|");
        if (parts.size() >= 3) {
          std::string message = parts[2];

          // Rejoin extra fields so messages may safely contain pipe text.
          for (std::size_t index = 3; index < parts.size(); ++index) {
            message += " | " + parts[index];
          }

          // Link the activity to an elevator when the event ID is valid.
          const auto elevator_id = ParseInteger(parts[1]);
          if (elevator_id.has_value()) {
            AddActivity(state, message, *elevator_id);
          } else {
            AddActivity(state, message);
          }
        }
      } else if (line != "Elevator bank status" &&
                 line != "Connected to the live Elevator Simulator." &&
                 !UpdateCarStatus(state, line)) {
        // Non-status lines are displayed as general activity messages.
        AddActivity(state, line);
      }
      newline = pending.find('\n');
    }
  }

  {
    std::lock_guard<std::mutex> lock(state->mutex);

    // Let the UI show the disconnected state after recv() ends.
    state->connected = false;
    state->activity.push_front({"Connection closed."});
  }

  // Signal the main UI loop to stop after the receiver exits.
  *running = false;
}

// Truncates text so ncurses output stays within a fixed column width.
std::string TrimToWidth(std::string_view text, int width) {
  // No visible space means no output; fitting text is returned unchanged.
  if (width <= 0 || static_cast<int>(text.size()) <= width) {
    return width <= 0 ? "" : std::string(text);
  }

  // Use ellipsis only when the column is wide enough to contain it.
  return width <= 3
             ? std::string(text.substr(0, static_cast<std::size_t>(width)))
             : std::string(
                   text.substr(0, static_cast<std::size_t>(width - 3))) +
                   "...";
}

// Prints bounded text at a fixed ncurses row and column.
void PrintAt(int y, int x, int width, std::string_view text) {
  if (width > 0) {
    mvaddnstr(y, x, TrimToWidth(text, width).c_str(), width);
  }
}

// Renders the remote dashboard from the latest receiver-thread state.
void DrawDashboard(DashboardState* state, const std::string& host, int port,
                   const std::string& input) {
  int max_y = 0;
  int max_x = 0;
  getmaxyx(stdscr, max_y, max_x);
  erase();

  // Avoid drawing the table when the terminal cannot fit the layout.
  if (max_y < 20 || max_x < 82) {
    PrintAt(1, 2, max_x - 4,
            "Please enlarge the terminal to at least 82 x 20.");
    refresh();
    return;
  }

  std::map<int, CarStatus> cars;
  std::deque<ActivityMessage> activity;
  bool connected = false;
  {
    // Copy shared state quickly so ncurses rendering does not hold the lock.
    std::lock_guard<std::mutex> lock(state->mutex);
    cars = state->cars;
    activity = state->activity;
    connected = state->connected;
  }

  // Draw the title bar.
  attron(A_REVERSE | A_BOLD);
  mvhline(0, 0, ' ', max_x);
  PrintAt(0, 2, max_x - 4, "Elevator Remote Control");
  attroff(A_REVERSE | A_BOLD);

  // Show the current socket target and connection state.
  const std::string connection =
      std::string(connected ? "CONNECTED" : "DISCONNECTED") + "  " + host +
      ":" + std::to_string(port);
  PrintAt(1, 2, max_x - 4, connection);

  attron(A_BOLD);
  PrintAt(3, 2, max_x - 4, "Live Elevator Bank");
  attroff(A_BOLD);

  // Draw the status table header using fixed column positions.
  attron(A_REVERSE);
  mvhline(4, 1, ' ', max_x - 2);
  for (std::size_t index = 0; index < kStatusHeaders.size(); ++index) {
    const int end = index + 1 < kStatusColumns.size()
                        ? kStatusColumns[index + 1] - 1
                        : max_x - 2;
    PrintAt(4, kStatusColumns[index], end - kStatusColumns[index],
            kStatusHeaders[index]);
  }
  attroff(A_REVERSE);

  // Render one row per assignment elevator, using placeholders until updated.
  int row = 5;
  for (int id = 1; id <= 4; ++id) {
    const auto found = cars.find(id);
    const CarStatus car = found == cars.end() ? CarStatus{id} : found->second;
    PrintAt(row, kStatusColumns[0], kStatusColumns[1] - kStatusColumns[0] - 1,
            "E" + std::to_string(id));
    PrintAt(row, kStatusColumns[1], kStatusColumns[2] - kStatusColumns[1] - 1,
            car.floor);
    PrintAt(row, kStatusColumns[2], kStatusColumns[3] - kStatusColumns[2] - 1,
            car.direction);
    PrintAt(row, kStatusColumns[3], kStatusColumns[4] - kStatusColumns[3] - 1,
            car.stage);
    PrintAt(row, kStatusColumns[4], kStatusColumns[5] - kStatusColumns[4] - 1,
            car.target);
    PrintAt(row, kStatusColumns[5], kStatusColumns[6] - kStatusColumns[5] - 1,
            car.active);
    PrintAt(row, kStatusColumns[6], max_x - kStatusColumns[6] - 2, car.queue);
    ++row;
  }

  // Draw recent activity below the status table.
  mvhline(row++, 1, '-', max_x - 2);
  attron(A_BOLD);
  PrintAt(row++, 2, max_x - 4, "Recent Activity");
  attroff(A_BOLD);

  const int instruction_y = max_y - 3;

  // Stop before the command area so activity cannot overlap the prompt.
  for (const auto& message : activity) {
    if (row >= instruction_y - 1) {
      break;
    }
    PrintAt(row++, 2, max_x - 4, message.text);
  }

  // Draw command help and the editable input line at the bottom.
  mvhline(instruction_y - 1, 1, '-', max_x - 2);
  PrintAt(instruction_y, 2, max_x - 4,
          "Commands: call <from> <to>  |  send <elevator> <floor>  |  quit");

  const std::string prompt = "Remote > " + input;
  attron(A_BOLD);
  PrintAt(max_y - 2, 2, max_x - 4, prompt);
  attroff(A_BOLD);

  // Keep the cursor after typed input without moving past the screen edge.
  move(max_y - 2, std::min(max_x - 2, 2 + static_cast<int>(prompt.size())));
  refresh();
}

}  // namespace

// Runs the remote ncurses dashboard and connects it to the simulator server.
int main(int argc, char* argv[]) {
  // Prevent broken socket writes from terminating the process with SIGPIPE.
  std::signal(SIGPIPE, SIG_IGN);

  // Use command-line host and port, falling back to the local simulator.
  const std::string host = argc >= 2 ? argv[1] : "127.0.0.1";
  const int port = ParsePort(argc, argv);
  const int socket_fd = ConnectToServer(host, port);

  if (socket_fd < 0) {
    std::cerr << "Failed to connect to " << host << ":" << port << "\n";
    return 1;
  }

  DashboardState state;
  std::atomic<bool> running{true};

  // Receive socket messages on a background thread so rendering stays live.
  std::thread receiver(ReceiveLoop, socket_fd, &state, &running);

  // Initialize ncurses after connecting so connection errors print normally.
  initscr();
  cbreak();
  noecho();
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  curs_set(1);

  std::string input;
  auto next_status_update = std::chrono::steady_clock::now();

  while (running) {
    const auto now = std::chrono::steady_clock::now();

    // Poll status independently of keyboard input to keep the dashboard live.
    if (now >= next_status_update) {
      if (!SendAll(socket_fd, "status\n")) {
        running = false;
        break;
      }
      next_status_update = now + std::chrono::milliseconds(400);
    }

    // Drain all currently available key presses from non-blocking getch().
    int ch = getch();
    while (ch != ERR) {
      if (ch == '\n' || ch == '\r') {
        if (!input.empty()) {
          // Send completed commands to the simulator as newline-delimited text.
          if (!SendAll(socket_fd, input + "\n")) {
            running = false;
            break;
          }

          // A quit command closes only this remote client.
          if (input == "quit") {
            running = false;
            break;
          }
          input.clear();
        }
      } else if (ch == KEY_BACKSPACE || ch == 127 || ch == 8) {
        if (!input.empty()) {
          input.pop_back();
        }
      } else if (ch >= 0 && ch <= UCHAR_MAX &&
                 std::isprint(static_cast<unsigned char>(ch))) {
        input.push_back(static_cast<char>(ch));
      }
      ch = getch();
    }

    // Render from the latest receiver-thread state and current input buffer.
    DrawDashboard(&state, host, port, input);

    // Limit the render loop so non-blocking input does not spin the CPU.
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  // Restore the terminal before closing network resources.
  endwin();

  // Wake any blocking recv() call, then close the socket descriptor.
  shutdown(socket_fd, SHUT_RDWR);
  close(socket_fd);

  // Wait for the receiver thread to finish before exiting main().
  if (receiver.joinable()) {
    receiver.join();
  }
  return 0;
}
