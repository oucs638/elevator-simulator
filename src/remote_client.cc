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
constexpr std::array<int, 7> kStatusColumns{2, 11, 18, 30, 47, 57, 70};
constexpr std::array<std::string_view, 7> kStatusHeaders{
    "Elevator", "Floor", "Direction", "Stage", "Target", "Active", "Queue",
};

struct CarStatus {
  int id = 0;
  std::string floor = "--";
  std::string direction = "--";
  std::string stage = "--";
  std::string target = "--";
  std::string active = "--";
  std::string queue = "--";
};

struct ActivityMessage {
  std::string text;
  int elevator_id = 0;
  bool seen_active = false;
  std::chrono::steady_clock::time_point created =
      std::chrono::steady_clock::now();
};

struct DashboardState {
  std::mutex mutex;
  std::map<int, CarStatus> cars;
  std::deque<ActivityMessage> activity;
  bool connected = true;
};

int ParsePort(int argc, char* argv[]) {
  if (argc < 3) {
    return kDefaultPort;
  }

  std::istringstream stream(argv[2]);
  int port = 0;
  if (!(stream >> port) || port <= 0 || port > 65535) {
    return kDefaultPort;
  }
  return port;
}

int ConnectToServer(const std::string& host, int port) {
  // Try every resolved address so both IPv4 and IPv6 host names work.
  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;

  addrinfo* result = nullptr;
  const std::string port_text = std::to_string(port);
  if (getaddrinfo(host.c_str(), port_text.c_str(), &hints, &result) != 0) {
    return -1;
  }

  int socket_fd = -1;
  for (addrinfo* item = result; item != nullptr; item = item->ai_next) {
    socket_fd = socket(item->ai_family, item->ai_socktype, item->ai_protocol);
    if (socket_fd < 0) {
      continue;
    }

    if (connect(socket_fd, item->ai_addr, item->ai_addrlen) == 0) {
      break;
    }

    close(socket_fd);
    socket_fd = -1;
  }

  freeaddrinfo(result);
  return socket_fd;
}

bool SendAll(int socket_fd, const std::string& text) {
  const char* data = text.c_str();
  std::size_t remaining = text.size();

  // A successful TCP send may write only part of the command.
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

std::vector<std::string> Split(const std::string& text,
                               const std::string& delimiter) {
  std::vector<std::string> parts;
  std::size_t begin = 0;
  std::size_t end = text.find(delimiter);

  while (end != std::string::npos) {
    parts.push_back(text.substr(begin, end - begin));
    begin = end + delimiter.size();
    end = text.find(delimiter, begin);
  }
  parts.push_back(text.substr(begin));
  return parts;
}

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

void AddActivity(DashboardState* state, const std::string& text,
                 int elevator_id = 0) {
  if (text.empty()) {
    return;
  }

  std::lock_guard<std::mutex> lock(state->mutex);
  state->activity.push_front({text, elevator_id});
  while (static_cast<int>(state->activity.size()) > kActivityLimit) {
    state->activity.pop_back();
  }
}

void UpdateActivityStateLocked(DashboardState* state) {
  // The caller holds state->mutex while status and activity are reconciled.
  const auto now = std::chrono::steady_clock::now();

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

  // Keep an elevator activity visible until its trip finishes. Informational
  // messages without an elevator identifier use a short fixed lifetime.
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

                       const bool idle = car->second.stage == "Idle" &&
                                         car->second.queue == "--";
                       return idle && (activity.seen_active ||
                                       age > std::chrono::seconds(3));
                     }),
      state->activity.end());
}

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
  const auto car_id = ParseInteger(std::string_view(parts[0]).substr(1));
  if (!car_id.has_value()) {
    return false;
  }
  car.id = *car_id;

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
  state->cars[car.id] = car;
  UpdateActivityStateLocked(state);
  return true;
}

void ReceiveLoop(int socket_fd, DashboardState* state,
                 std::atomic<bool>* running) {
  std::string pending;
  char buffer[1024];

  while (*running) {
    const ssize_t received = recv(socket_fd, buffer, sizeof(buffer), 0);
    if (received <= 0) {
      break;
    }

    // Preserve a partial final line because TCP does not retain message
    // boundaries between recv() calls.
    pending.append(buffer, static_cast<std::size_t>(received));
    std::size_t newline = pending.find('\n');

    while (newline != std::string::npos) {
      const std::string line = pending.substr(0, newline);
      pending.erase(0, newline + 1);

      if (line.rfind("EVENT|", 0) == 0) {
        const auto parts = Split(line, "|");
        if (parts.size() >= 3) {
          std::string message = parts[2];
          for (std::size_t index = 3; index < parts.size(); ++index) {
            message += " | " + parts[index];
          }
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
        AddActivity(state, line);
      }
      newline = pending.find('\n');
    }
  }

  {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->connected = false;
    state->activity.push_front({"Connection closed."});
  }
  *running = false;
}

std::string TrimToWidth(std::string_view text, int width) {
  if (width <= 0 || static_cast<int>(text.size()) <= width) {
    return width <= 0 ? "" : std::string(text);
  }
  return width <= 3
             ? std::string(text.substr(0, static_cast<std::size_t>(width)))
             : std::string(
                   text.substr(0, static_cast<std::size_t>(width - 3))) +
                   "...";
}

void PrintAt(int y, int x, int width, std::string_view text) {
  if (width > 0) {
    mvaddnstr(y, x, TrimToWidth(text, width).c_str(), width);
  }
}

void DrawDashboard(DashboardState* state, const std::string& host, int port,
                   const std::string& input) {
  int max_y = 0;
  int max_x = 0;
  getmaxyx(stdscr, max_y, max_x);
  erase();

  if (max_y < 20 || max_x < 82) {
    PrintAt(1, 2, max_x - 4,
            "Please enlarge the terminal to at least 82 x 20.");
    refresh();
    return;
  }

  // Draw from a snapshot so network updates never block ncurses rendering.
  std::map<int, CarStatus> cars;
  std::deque<ActivityMessage> activity;
  bool connected = false;
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    cars = state->cars;
    activity = state->activity;
    connected = state->connected;
  }

  attron(A_REVERSE | A_BOLD);
  mvhline(0, 0, ' ', max_x);
  PrintAt(0, 2, max_x - 4, "Elevator Remote Control");
  attroff(A_REVERSE | A_BOLD);

  const std::string connection =
      std::string(connected ? "CONNECTED" : "DISCONNECTED") + "  " + host +
      ":" + std::to_string(port);
  PrintAt(1, 2, max_x - 4, connection);

  attron(A_BOLD);
  PrintAt(3, 2, max_x - 4, "Live Elevator Bank");
  attroff(A_BOLD);

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

  mvhline(row++, 1, '-', max_x - 2);
  attron(A_BOLD);
  PrintAt(row++, 2, max_x - 4, "Recent Activity");
  attroff(A_BOLD);

  const int instruction_y = max_y - 3;
  for (const auto& message : activity) {
    if (row >= instruction_y - 1) {
      break;
    }
    PrintAt(row++, 2, max_x - 4, message.text);
  }

  mvhline(instruction_y - 1, 1, '-', max_x - 2);
  PrintAt(instruction_y, 2, max_x - 4,
          "Commands: call <from> <to>  |  send <elevator> <floor>  |  quit");

  const std::string prompt = "Remote > " + input;
  attron(A_BOLD);
  PrintAt(max_y - 2, 2, max_x - 4, prompt);
  attroff(A_BOLD);
  move(max_y - 2, std::min(max_x - 2, 2 + static_cast<int>(prompt.size())));
  refresh();
}

}  // namespace

int main(int argc, char* argv[]) {
  std::signal(SIGPIPE, SIG_IGN);

  const std::string host = argc >= 2 ? argv[1] : "127.0.0.1";
  const int port = ParsePort(argc, argv);
  const int socket_fd = ConnectToServer(host, port);

  if (socket_fd < 0) {
    std::cerr << "Failed to connect to " << host << ":" << port << "\n";
    return 1;
  }

  DashboardState state;
  std::atomic<bool> running{true};
  std::thread receiver(ReceiveLoop, socket_fd, &state, &running);

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
    // Poll independently of operator input to keep the dashboard live.
    if (now >= next_status_update) {
      if (!SendAll(socket_fd, "status\n")) {
        running = false;
        break;
      }
      next_status_update = now + std::chrono::milliseconds(400);
    }

    int ch = getch();
    while (ch != ERR) {
      if (ch == '\n' || ch == '\r') {
        if (!input.empty()) {
          if (!SendAll(socket_fd, input + "\n")) {
            running = false;
            break;
          }
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

    DrawDashboard(&state, host, port, input);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  endwin();
  shutdown(socket_fd, SHUT_RDWR);
  close(socket_fd);
  if (receiver.joinable()) {
    receiver.join();
  }
  return 0;
}
