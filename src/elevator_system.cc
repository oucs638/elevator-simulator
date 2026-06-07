// Implements route-aware request planning and elevator-bank coordination.

#include "elevator_system.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <thread>
#include <utility>

namespace elevator_simulator {

namespace {

std::string RequestText(int current, int destination) {
  std::ostringstream stream;
  stream << "floor " << current << " -> " << destination;
  return stream.str();
}

}  // namespace

ElevatorSystem::ElevatorSystem(std::vector<Elevator*> elevators)
    : elevators_(std::move(elevators)) {}

DispatchPlan ElevatorSystem::PlanDispatch(int current, int destination) const {
  std::lock_guard<std::mutex> lock(dispatch_mutex_);

  if (!IsValidFloor(current) || !IsValidFloor(destination)) {
    return {false, 0, {}, "Floors must be from 1 to 10."};
  }

  if (current == destination) {
    return {false, 0, {}, "Current floor and desired floor are the same."};
  }

  std::vector<DispatchCandidate> candidates;
  for (const auto& snapshot : Snapshots()) {
    const int wait = std::abs(snapshot.current_floor - current);
    candidates.push_back({
        snapshot.id,
        wait,
        static_cast<int>(snapshot.queued_requests.size()),
        !snapshot.busy && snapshot.queued_requests.empty(),
        false,
        "stopped",
        "basic nearest-floor estimate",
    });
  }

  std::sort(candidates.begin(), candidates.end(),
            [](const auto& left, const auto& right) {
              if (left.estimated_wait_seconds != right.estimated_wait_seconds) {
                return left.estimated_wait_seconds <
                       right.estimated_wait_seconds;
              }
              return left.elevator_id < right.elevator_id;
            });

  if (candidates.empty()) {
    return {false, 0, {}, "No elevator is available."};
  }

  return {true, candidates.front().elevator_id, candidates,
          "Dispatch plan ready."};
}

DispatchResult ElevatorSystem::DispatchNearest(int current, int destination) {
  std::lock_guard<std::mutex> lock(dispatch_mutex_);
  const auto plan = PlanDispatch(current, destination);
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

  elevator->SubmitRequest(current, destination);
  return {true, elevator_id, 0, "manual dispatch",
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

  elevator->SubmitDirectRequest(floor);
  return {true, elevator_id, 0, "central control direct send",
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

}  // namespace elevator_simulator
