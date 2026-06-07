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

using elevator_simulator::DispatchResult;
using elevator_simulator::Elevator;
using elevator_simulator::ElevatorRequest;
using elevator_simulator::ElevatorRequestType;
using elevator_simulator::ElevatorSnapshot;
using elevator_simulator::ElevatorStage;
using elevator_simulator::ElevatorSystem;

namespace {

constexpr int kMessageLimit = 6;

constexpr std::array<std::string_view, 7> kStatusHeaders{
    "Elevator", "Floor", "Direction", "Stage", "Target", "Active", "Queue",
};

struct ActivityMessage {
  std::string text;
  int elevator_id;
};

// Truncates text without allowing a UI field to overwrite adjacent content.
std::string TrimToWidth(std::string_view text, int width) {
  if (width <= 0) {
    return "";
  }

  if (static_cast<int>(text.size()) <= width) {
    return std::string(text);
  }

  if (width <= 3) {
    return std::string(text.substr(0, static_cast<std::size_t>(width)));
  }

  return std::string(text.substr(0, static_cast<std::size_t>(width - 3))) +
         "...";
}

void PrintAt(int y, int x, int width, std::string_view text) {
  if (width <= 0) {
    return;
  }

  mvaddnstr(y, x, TrimToWidth(text, width).c_str(), width);
}

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

// Draws the ten-floor building view and current position of each elevator.
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

  for (int index = 1; index <= elevator_count; ++index) {
    boundaries.push_back(left + kFloorColumnSpan +
                         (remaining_span * index) / elevator_count);
  }

  for (int x = left + 1; x < right; ++x) {
    mvaddch(top, x, '-');
    mvaddch(bottom, x, '-');
  }

  for (const int boundary : boundaries) {
    for (int y = top + 1; y < bottom; ++y) {
      mvaddch(y, boundary, '|');
    }
    mvaddch(top, boundary, '+');
    mvaddch(bottom, boundary, '+');
  }

  PrintCentered(top + 1, boundaries[0], boundaries[1], "Floor");
  for (int index = 0; index < elevator_count; ++index) {
    PrintCentered(top + 1, boundaries[index + 1], boundaries[index + 2],
                  "Elevator " + std::to_string(snapshots[index].id));
  }

  for (int floor = Elevator::kMaxFloor; floor >= Elevator::kMinFloor; --floor) {
    const int row = top + 2 + (Elevator::kMaxFloor - floor);
    PrintCentered(row, boundaries[0], boundaries[1],
                  std::to_string(floor) + "F");

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

std::string DirectionText(int direction) {
  if (direction > 0) {
    return "Up";
  }
  if (direction < 0) {
    return "Down";
  }
  return "Stopped";
}

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

std::string RequestText(const ElevatorRequest& request) {
  if (request.type == ElevatorRequestType::kDirectSend) {
    return "send->" + std::to_string(request.destination);
  }
  return std::to_string(request.current) + "->" +
         std::to_string(request.destination);
}

std::string ActiveText(const ElevatorSnapshot& snapshot) {
  if (!snapshot.active_request.has_value()) {
    return "--";
  }
  return RequestText(*snapshot.active_request);
}

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

// Draws live elevator state using the same columns as remote control.
int DrawStatusView(const std::vector<ElevatorSnapshot>& snapshots, int top,
                   int left, int width) {
  const int right = left + width - 1;
  const std::array<int, 7> columns{
      left + 1,  left + 10, left + 17, left + 29,
      left + 46, left + 56, left + 69,
  };

  attron(A_REVERSE | A_BOLD);
  mvhline(top, left, ' ', width);
  for (std::size_t index = 0; index < kStatusHeaders.size(); ++index) {
    const int end =
        index + 1 < columns.size() ? columns[index + 1] - 1 : right - 1;
    PrintAt(top, columns[index], end - columns[index], kStatusHeaders[index]);
  }
  attroff(A_REVERSE | A_BOLD);

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

  for (int x = left; x <= right; ++x) {
    mvaddch(row, x, '-');
  }

  return row;
}

}  // namespace

int main() {
  Elevator elevator1(1);
  Elevator elevator2(2);
  Elevator elevator3(3);
  Elevator elevator4(4);

  std::vector<Elevator*> elevators{&elevator1, &elevator2, &elevator3,
                                   &elevator4};

  ElevatorSystem system(elevators);

  for (auto* elevator : elevators) {
    elevator->Start();
  }

  initscr();
  cbreak();
  noecho();
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  curs_set(1);

  bool running = true;

  while (running) {
    int ch = getch();

    if (ch == 'q') {
      running = false;
    }

    erase();

    int max_y = 0;
    int max_x = 0;
    getmaxyx(stdscr, max_y, max_x);

    if (max_y < 27 || max_x < 82) {
      PrintAt(1, 2, max_x - 4,
              "Please enlarge the terminal to at least 82 x 27.");
      refresh();
      std::this_thread::sleep_for(std::chrono::milliseconds(80));
      continue;
    }

    PrintAt(0, 2, max_x - 4, "Elevator Simulator");

    const auto snapshots = system.Snapshots();

    DrawBuildingView(snapshots, 3, 1, max_x - 2);
    DrawStatusView(snapshots, 17, 1, max_x - 2);

    refresh();
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
  }

  endwin();

  for (auto* elevator : elevators) {
    elevator->Stop();
  }

  return 0;
}
