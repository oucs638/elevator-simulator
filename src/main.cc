// Implements the primary ncurses simulator and starts remote-control service.

#include <ncurses.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <climits>
#include <deque>
#include <iostream>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "elevator.h"
#include "elevator_system.h"
#include "remote_control_server.h"

using elevator_simulator::DispatchResult;
using elevator_simulator::Elevator;
using elevator_simulator::ElevatorRequest;
using elevator_simulator::ElevatorRequestType;
using elevator_simulator::ElevatorSnapshot;
using elevator_simulator::ElevatorStage;
using elevator_simulator::ElevatorSystem;
using elevator_simulator::RemoteControlServer;

namespace {

constexpr int kMessageLimit = 6;
constexpr int kDefaultRemotePort = 5050;

// Fixed labels for the main UI elevator status table.
constexpr std::array<std::string_view, 7> kStatusHeaders{
    "Elevator", "Floor", "Direction", "Stage", "Target", "Active", "Queue",
};

// Tracks one main-UI activity line and its related elevator, if any.
struct ActivityMessage {
  std::string text;
  int elevator_id;
};

// Truncates text so ncurses output stays within a fixed column width.
std::string TrimToWidth(std::string_view text, int width) {
  if (width <= 0) {
    return "";
  }

  if (static_cast<int>(text.size()) <= width) {
    return std::string(text);
  }

  // Use raw truncation when the column is too narrow for an ellipsis.
  if (width <= 3) {
    return std::string(text.substr(0, static_cast<std::size_t>(width)));
  }

  return std::string(text.substr(0, static_cast<std::size_t>(width - 3))) +
         "...";
}

// Prints bounded text at a fixed ncurses row and column.
void PrintAt(int y, int x, int width, std::string_view text) {
  if (width <= 0) {
    return;
  }

  mvaddnstr(y, x, TrimToWidth(text, width).c_str(), width);
}

// Prints text centered between two ncurses column boundaries.
void PrintCentered(int y, int left, int right, std::string_view text) {
  const int available_width = right - left - 1;
  if (available_width <= 0) {
    return;
  }

  const std::string visible_text = TrimToWidth(text, available_width);
  const int x =
      left + 1 +
      std::max(0,
               (available_width - static_cast<int>(visible_text.size())) / 2);
  mvaddnstr(y, x, visible_text.c_str(), available_width);
}

// Draws the main floor grid and places each elevator on its current floor.
void DrawBuildingView(const std::vector<ElevatorSnapshot>& snapshots, int top,
                      int left, int width) {
  constexpr int kHeight = 13;
  constexpr int kFloorColumnSpan = 7;

  const int right = left + width - 1;
  const int bottom = top + kHeight - 1;
  const int elevator_count = static_cast<int>(snapshots.size());
  const int remaining_span = right - left - kFloorColumnSpan;

  std::vector<int> boundaries;
  boundaries.reserve(snapshots.size() + 2);
  boundaries.push_back(left);
  boundaries.push_back(left + kFloorColumnSpan);

  // Split remaining width evenly across the elevator columns.
  for (int index = 1; index <= elevator_count; ++index) {
    boundaries.push_back(left + kFloorColumnSpan +
                         (remaining_span * index) / elevator_count);
  }

  // Draw top and bottom borders.
  for (int x = left + 1; x < right; ++x) {
    mvaddch(top, x, '-');
    mvaddch(bottom, x, '-');
  }

  // Draw vertical borders at each computed column boundary.
  for (const int boundary : boundaries) {
    for (int y = top + 1; y < bottom; ++y) {
      mvaddch(y, boundary, '|');
    }
    mvaddch(top, boundary, '+');
    mvaddch(bottom, boundary, '+');
  }

  // Draw column headers.
  PrintCentered(top + 1, boundaries[0], boundaries[1], "Floor");
  for (int index = 0; index < elevator_count; ++index) {
    PrintCentered(top + 1, boundaries[index + 1], boundaries[index + 2],
                  "Elevator " + std::to_string(snapshots[index].id));
  }

  // Draw floors from top to bottom so higher floors appear higher on screen.
  for (int floor = Elevator::kMaxFloor; floor >= Elevator::kMinFloor; --floor) {
    const int row = top + 2 + (Elevator::kMaxFloor - floor);
    PrintCentered(row, boundaries[0], boundaries[1],
                  std::to_string(floor) + "F");

    // Place each elevator marker on the row matching its current floor.
    for (int index = 0; index < elevator_count; ++index) {
      const auto& snapshot = snapshots[index];
      if (floor != snapshot.current_floor) {
        continue;
      }

      attron(A_BOLD | A_REVERSE);
      PrintCentered(row, boundaries[index + 1], boundaries[index + 2],
                    "[ E" + std::to_string(snapshot.id) + " ]");
      attroff(A_BOLD | A_REVERSE);
    }
  }
}

// Converts signed movement direction into main-UI display text.
std::string DirectionText(int direction) {
  if (direction > 0) {
    return "Up";
  }
  if (direction < 0) {
    return "Down";
  }
  return "Stopped";
}

// Converts elevator stages into main-UI display labels.
std::string StageText(ElevatorStage stage) {
  switch (stage) {
    case ElevatorStage::kIdle:
      return "Idle";
    case ElevatorStage::kToPickup:
      return "Picking up";
    case ElevatorStage::kBoarding:
      return "Boarding";
    case ElevatorStage::kToDestination:
      return "In service";
    case ElevatorStage::kArrived:
      return "Arrived";
    case ElevatorStage::kStopped:
      return "Stopped";
  }
  return "--";
}

// Formats one request for compact main-UI status output.
std::string RequestText(const ElevatorRequest& request) {
  if (request.type == ElevatorRequestType::kDirectSend) {
    return "send->" + std::to_string(request.destination);
  }

  return std::to_string(request.current) + "->" +
         std::to_string(request.destination);
}

// Returns the active request text or a placeholder when the car is idle.
std::string ActiveText(const ElevatorSnapshot& snapshot) {
  if (!snapshot.active_request.has_value()) {
    return "--";
  }

  return RequestText(*snapshot.active_request);
}

// Joins queued requests into one compact status-table field.
std::string QueueText(const ElevatorSnapshot& snapshot) {
  if (snapshot.queued_requests.empty()) {
    return "--";
  }

  std::ostringstream stream;
  for (std::size_t index = 0; index < snapshot.queued_requests.size();
       ++index) {
    if (index > 0) {
      stream << ",";
    }
    stream << RequestText(snapshot.queued_requests[index]);
  }

  return stream.str();
}

// Draws the main elevator status table and returns the divider row.
int DrawStatusView(const std::vector<ElevatorSnapshot>& snapshots, int top,
                   int left, int width) {
  const int right = left + width - 1;
  const std::array<int, 7> columns{
      left + 1,  left + 10, left + 17, left + 29,
      left + 46, left + 56, left + 69,
  };

  // Draw a highlighted header row using fixed status-table columns.
  attron(A_REVERSE | A_BOLD);
  mvhline(top, left, ' ', width);
  for (std::size_t index = 0; index < kStatusHeaders.size(); ++index) {
    const int end =
        index + 1 < columns.size() ? columns[index + 1] - 1 : right - 1;
    PrintAt(top, columns[index], end - columns[index], kStatusHeaders[index]);
  }
  attroff(A_REVERSE | A_BOLD);

  // Render one snapshot per row without reading shared elevator state directly.
  int row = top + 1;
  for (const auto& snapshot : snapshots) {
    PrintAt(row, columns[0], columns[1] - columns[0] - 1,
            "E" + std::to_string(snapshot.id));
    PrintAt(row, columns[1], columns[2] - columns[1] - 1,
            std::to_string(snapshot.current_floor));
    PrintAt(row, columns[2], columns[3] - columns[2] - 1,
            DirectionText(snapshot.direction));
    PrintAt(row, columns[3], columns[4] - columns[3] - 1,
            StageText(snapshot.stage));
    PrintAt(row, columns[4], columns[5] - columns[4] - 1,
            snapshot.target_floor.has_value()
                ? std::to_string(*snapshot.target_floor)
                : "--");
    PrintAt(row, columns[5], columns[6] - columns[5] - 1, ActiveText(snapshot));
    PrintAt(row, columns[6], right - columns[6] - 1, QueueText(snapshot));
    ++row;
  }

  // Draw the divider below the table and report its row to the caller.
  for (int x = left; x <= right; ++x) {
    mvaddch(row, x, '-');
  }

  return row;
}

// Adds a dispatch result to the main activity panel.
void HandleDispatchResult(const DispatchResult& result,
                          std::deque<ActivityMessage>* messages) {
  // Link successful messages to an elevator so completion can remove them.
  messages->push_front(
      {result.message, result.accepted ? result.elevator_id : 0});
}

// Parses the main UI command line and submits auto or manual dispatch.
void ParseCommand(const std::string& command, ElevatorSystem* system,
                  std::deque<ActivityMessage>* messages) {
  std::istringstream stream(command);
  std::vector<int> values;
  int value = 0;

  // The main UI accepts numeric commands only.
  while (stream >> value) {
    values.push_back(value);
  }

  // Accept exactly two numbers for auto mode or three for manual mode.
  if (!stream.eof() || (values.size() != 2 && values.size() != 3)) {
    messages->push_front({
        "Use auto: <current> <destination> or manual: <elevator> <current> "
        "<destination>.",
        0,
    });
    return;
  }

  // Two values mean auto dispatch: current floor and destination.
  if (values.size() == 2) {
    HandleDispatchResult(system->DispatchNearest(values[0], values[1]),
                         messages);
    return;
  }

  // Three values mean manual dispatch: elevator ID, current, and destination.
  HandleDispatchResult(system->SubmitManual(values[0], values[1], values[2]),
                       messages);
}

// Removes activity entries whose related elevator has finished all work.
void RemoveCompletedActivities(std::deque<ActivityMessage>* messages,
                               const std::vector<ElevatorSnapshot>& snapshots) {
  messages->erase(
      std::remove_if(messages->begin(), messages->end(),
                     [&snapshots](const ActivityMessage& message) {
                       // General messages are not tied to elevator completion.
                       if (message.elevator_id == 0) {
                         return false;
                       }

                       const auto elevator = std::find_if(
                           snapshots.begin(), snapshots.end(),
                           [&message](const ElevatorSnapshot& snapshot) {
                             return snapshot.id == message.elevator_id;
                           });

                       // Remove only after active and queued work are finished.
                       return elevator != snapshots.end() && !elevator->busy &&
                              elevator->queued_requests.empty();
                     }),
      messages->end());
}

}  // namespace

// Runs the main ncurses simulator and shared remote-control server.
int main() {
  // Keep assignment-required elevator variable names visible in main().
  Elevator elevator1(1);
  Elevator elevator2(2);
  Elevator elevator3(3);
  Elevator elevator4(4);

  std::vector<Elevator*> elevators{&elevator1, &elevator2, &elevator3,
                                   &elevator4};

  // Local and remote interfaces share one system and one elevator bank.
  ElevatorSystem system(elevators);
  RemoteControlServer remote_server(system);

  // Start each elevator's worker before accepting local or remote commands.
  for (auto* elevator : elevators) {
    elevator->Start();
  }

  // Start the remote server so the second terminal can attach with make remote.
  std::string remote_error;
  if (!remote_server.Start(kDefaultRemotePort, &remote_error)) {
    std::cerr << remote_error << "\n";
    return 1;
  }

  // Initialize ncurses in nonblocking input mode for continuous rendering.
  initscr();
  cbreak();
  noecho();
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  curs_set(1);

  std::string input;
  std::deque<ActivityMessage> messages;

  bool running = true;

  while (running) {
    // Nonblocking input keeps rendering independent from elevator movement.
    int ch = getch();
    while (ch != ERR) {
      if (ch == '\n' || ch == '\r') {
        if (!input.empty()) {
          if (input == "quit") {
            running = false;
            break;
          } else {
            // Submit the completed local command and record the result.
            ParseCommand(input, &system, &messages);

            // Keep the activity panel bounded by removing oldest messages.
            while (static_cast<int>(messages.size()) > kMessageLimit) {
              messages.pop_back();
            }
            input.clear();
          }
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

    erase();

    int max_y = 0;
    int max_x = 0;
    getmaxyx(stdscr, max_y, max_x);

    // Avoid drawing overlapping panels when the terminal is too small.
    if (max_y < 27 || max_x < 82) {
      PrintAt(1, 2, max_x - 4,
              "Please enlarge the terminal to at least 82 x 27.");
      refresh();
      std::this_thread::sleep_for(std::chrono::milliseconds(80));
      continue;
    }

    PrintAt(0, 2, max_x - 4, "Elevator Simulator");

    // Render from snapshots so UI never reads elevator internals directly.
    const auto snapshots = system.Snapshots();
    RemoveCompletedActivities(&messages, snapshots);
    DrawBuildingView(snapshots, 3, 1, max_x - 2);
    const int status_bottom = DrawStatusView(snapshots, 17, 1, max_x - 2);

    // Draw the activity panel below the status table.
    int message_y = status_bottom + 1;
    attron(A_BOLD);
    PrintAt(message_y++, 2, max_x - 4, "Activity");
    attroff(A_BOLD);

    const int prompt_y = max_y - 2;
    const int available_message_rows = prompt_y - message_y - 1;
    int rendered_messages = 0;

    // Stop before the command area so activity cannot overlap the prompt.
    for (const auto& message : messages) {
      if (rendered_messages >= available_message_rows) {
        break;
      }
      PrintAt(message_y++, 2, max_x - 4, message.text);
      ++rendered_messages;
    }

    const int instruction_y = prompt_y - 1;
    mvhline(instruction_y - 1, 1, '-', max_x - 2);

    // Keep command help directly above the editable input line.
    PrintAt(instruction_y, 2, max_x - 4,
            "Commands: <from> <to>  |  <elevator> <from> <to>  |  quit");

    const std::string prompt = "Command > " + input;
    attron(A_BOLD);
    PrintAt(prompt_y, 2, max_x - 4, prompt);
    attroff(A_BOLD);

    // Place the cursor after the current input without exceeding screen width.
    move(prompt_y, std::min(max_x - 2, 2 + static_cast<int>(prompt.size())));

    refresh();

    // Limit redraw frequency so the nonblocking loop does not spin the CPU.
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
  }

  // Restore the terminal before stopping background server and workers.
  endwin();

  remote_server.Stop();
  for (auto* elevator : elevators) {
    elevator->Stop();
  }

  return 0;
}