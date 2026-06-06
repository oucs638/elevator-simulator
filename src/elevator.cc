// Implemnts one elevator's movement state machine and request worker.

#include "elevator.h"

#include <algorithm>
#include <sstream>

namespace elevator_simulator {

Elevator::Elevator(int id, int start_floor) { status_ = "Idle"; }

void Elevator::display_floor() {}

void Elevator::move(int current, int floor) {}

ElevatorSnapshot Elevator::Snapshot() const {
  int direction = 0;
  return ElevatorSnapshot{
      id_,
      current_floor,
      busy_,
      active_request_,
      std::vector<ElevatorRequest>(queue_.begin(), queue_.end()),
      target_floor_,
      direction,
      stage_,
      status_,
      display_text_,
  };
}
}  // namespace elevator_simulator
