// Implements route-aware request planning and elevator-bank coordination.

#include "elevator_system.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <thread>
#include <utility>

namespace elevator_simulator {

namespace {

int DirectionOf(int from, int to) {
  if (to > from) {
    return 1;
  }
  if (to < from) {
    return -1;
  }
  return 0;
}

std::string RequestText(int current, int destination) {
  std::ostringstream stream;
  stream << "floor " << current << " -> " << destination;
  return stream.str();
}

std::string DirectionText(int direction) {
  if (direction > 0) {
    return "up";
  }
  if (direction < 0) {
    return "down";
  }
  return "stopped";
}

void AddTravel(int* total, int* position, int destination) {
  *total += std::abs(*position - destination);
  *position = destination;
}

void AddServiceStop(int* total) {
  // One simulated second represents pickup or drop-off door service.
  ++*total;
}

}  // namespace

ElevatorSystem::ElevatorSystem(std::vector<Elevator*> elevators)
    : elevators_(std::move(elevators)) {}

DispatchPlan ElevatorSystem::PlanDispatch(int current, int destination) const {
  std::lock_guard<std::mutex> lock(dispatch_mutex_);
  return BuildDispatchPlan(current, destination);
}

DispatchResult ElevatorSystem::DispatchNearest(int current, int destination) {
  std::lock_guard<std::mutex> lock(dispatch_mutex_);
  const auto plan = BuildDispatchPlan(current, destination);
  if (!plan.valid || plan.candidates.empty()) {
    return {false, 0, 0, "", plan.message};
  }

  const auto& selected = plan.candidates.front();
  Elevator* elevator = FindElevator(selected.elevator_id);
  if (elevator == nullptr) {
    return {false, 0, 0, "", "No elevator is available."};
  }

  elevator->SubmitRequest(current, destination);
  return {true, selected.elevator_id, selected.estimated_wait_seconds,
          selected.reason,
          "Auto dispatched elevator " + std::to_string(selected.elevator_id) +
              ": " + RequestText(current, destination)};
}

DispatchResult ElevatorSystem::SubmitManual(int elevator_id, int current,
                                            int destination) {
  std::lock_guard<std::mutex> lock(dispatch_mutex_);

  if (!IsValidFloor(current) || !IsValidFloor(destination)) {
    return {false, 0, 0, "", "Floors must be from 1 to 10."};
  }

  if (current == destination) {
    return {false, 0, 0, "", "Current floor and desired floor are the same."};
  }

  Elevator* elevator = FindElevator(elevator_id);
  if (elevator == nullptr) {
    return {false, 0, 0, "",
            "Elevator must be between 1 and " +
                std::to_string(ElevatorCount()) + "."};
  }

  const auto candiate =
      BuildCandidate(elevator->Snapshot(), current, destination);
  elevator->SubmitRequest(current, destination);
  return {true, elevator_id, candiate.estimated_wait_seconds, candiate.reason,
          "Manually queued elevator " + std::to_string(elevator_id) + ": " +
              RequestText(current, destination)};
}

DispatchResult ElevatorSystem::SendElevator(int elevator_id, int floor) {
  std::lock_guard<std::mutex> lock(dispatch_mutex_);

  if (!IsValidFloor(floor)) {
    return {false, 0, 0, "", "Floor must be from 1 to 10."};
  }

  Elevator* elevator = FindElevator(elevator_id);
  if (elevator == nullptr) {
    return {false, 0, 0, "",
            "Elevator must be between 1 and " +
                std::to_string(ElevatorCount()) + "."};
  }

  const auto snapshot = elevator->Snapshot();
  elevator->SubmitDirectRequest(floor);
  return {true, elevator_id, EstimateWaitSeconds(snapshot, floor),
          "central control direct send",
          "Sending elevator " + std::to_string(elevator_id) + " to floor " +
              std::to_string(floor)};
}

std::vector<ElevatorSnapshot> ElevatorSystem::Snapshots() const {
  std::vector<ElevatorSnapshot> result;
  result.reserve(elevators_.size());

  for (const auto* elevator : elevators_) {
    if (elevator != nullptr) {
      result.push_back(elevator->Snapshot());
    }
  }

  return result;
}

bool ElevatorSystem::WaitUntilAllIdle(std::chrono::milliseconds timeout) const {
  const auto deadline = std::chrono::steady_clock::now() + timeout;

  while (std::chrono::steady_clock::now() < deadline) {
    bool all_idle = true;
    for (const auto& snapshot : Snapshots()) {
      if (snapshot.busy || !snapshot.queued_requests.empty()) {
        all_idle = false;
        break;
      }
    }

    if (all_idle) {
      return true;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  return false;
}

int ElevatorSystem::ElevatorCount() const {
  return static_cast<int>(elevators_.size());
}

bool ElevatorSystem::IsValidFloor(int floor) const {
  return floor >= Elevator::kMinFloor && floor <= Elevator::kMaxFloor;
}

Elevator* ElevatorSystem::FindElevator(int elevator_id) const {
  for (auto* elevator : elevators_) {
    if (elevator != nullptr && elevator->Snapshot().id == elevator_id) {
      return elevator;
    }
  }

  return nullptr;
}

DispatchPlan ElevatorSystem::BuildDispatchPlan(int current,
                                               int destination) const {
  if (!IsValidFloor(current) || !IsValidFloor(destination)) {
    return {false, 0, {}, "Floors must be from 1 to 10."};
  }

  if (current == destination) {
    return {false, 0, {}, "Current floor and desired floor are the same."};
  }

  std::vector<DispatchCandidate> candidates;
  candidates.reserve(elevators_.size());
  for (const auto& snapshot : Snapshots()) {
    candidates.push_back(BuildCandidate(snapshot, current, destination));
  }

  // Stable tie-breakers keep dispatch decisions predictable and explainable.
  std::sort(candidates.begin(), candidates.end(),
            [](const auto& left, const auto& right) {
              if (left.estimated_wait_seconds != right.estimated_wait_seconds) {
                return left.estimated_wait_seconds <
                       right.estimated_wait_seconds;
              }
              if (left.same_direction != right.same_direction) {
                return left.same_direction;
              }
              if (left.queued_trips != right.queued_trips) {
                return left.queued_trips < right.queued_trips;
              }
              return left.elevator_id < right.elevator_id;
            });

  if (candidates.empty()) {
    return {false, 0, {}, "No elevator is available."};
  }

  return {true, candidates.front().elevator_id, candidates,
          "Dispatch plan ready."};
}

DispatchCandidate ElevatorSystem::BuildCandidate(
    const ElevatorSnapshot& snapshot, int current, int destination) const {
  const int passenger_direction = DirectionOf(current, destination);
  const bool same_direction =
      snapshot.direction != 0 && snapshot.direction == passenger_direction;
  const int estimated_wait = EstimateWaitSeconds(snapshot, current);

  std::ostringstream reason;
  if (!snapshot.busy && snapshot.queued_requests.empty()) {
    reason << "idle, " << std::abs(snapshot.current_floor - current)
           << " floor(s) from caller";
  } else {
    reason << "route-aware wait includes current trip";
    if (!snapshot.queued_requests.empty()) {
      reason << " and " << snapshot.queued_requests.size() << " queued trip(s)";
    }
    reason << "; moving " << DirectionText(snapshot.direction);
    if (same_direction) {
      reason << " in passenger direction";
    }
  }

  return {
      snapshot.id,
      estimated_wait,
      static_cast<int>(snapshot.queued_requests.size()),
      !snapshot.busy && snapshot.queued_requests.empty(),
      same_direction,
      DirectionText(snapshot.direction),
      reason.str(),
  };
}

int ElevatorSystem::EstimateWaitSeconds(const ElevatorSnapshot& snapshot,
                                        int current) const {
  int total = 0;
  int position = snapshot.current_floor;

  // Finish the active journey before simulating requests already in the queue.
  if (snapshot.active_request.has_value()) {
    const auto active = *snapshot.active_request;
    if (snapshot.stage == ElevatorStage::kToPickup) {
      AddTravel(&total, &position, active.current);
      AddServiceStop(&total);
      AddTravel(&total, &position, active.destination);
      AddServiceStop(&total);
    } else if (snapshot.stage == ElevatorStage::kBoarding) {
      position = active.current;
      AddTravel(&total, &position, active.destination);
      AddServiceStop(&total);
    } else if (snapshot.stage == ElevatorStage::kToDestination) {
      AddTravel(&total, &position, active.destination);
      AddServiceStop(&total);
    } else if (snapshot.stage == ElevatorStage::kArrived) {
      position = active.destination;
    }
  }

  // The new passenger is appended after every request already assigned to the
  // car.
  for (const auto& queued : snapshot.queued_requests) {
    if (queued.type == ElevatorRequestType::kPassengerTrip) {
      AddTravel(&total, &position, queued.current);
      AddServiceStop(&total);
    }
    AddTravel(&total, &position, queued.destination);
    AddServiceStop(&total);
  }

  AddTravel(&total, &position, current);
  return total;
}

}  // namespace elevator_simulator
