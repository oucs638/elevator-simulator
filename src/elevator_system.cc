// Implements route-aware request planning and elevator-bank coordination.

#include "elevator_system.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <thread>
#include <utility>

namespace elevator_simulator {

namespace {

// Returns the signed movement direction between two floors.
int DirectionOf(int from, int to) {
  if (to > from) {
    return 1;
  }
  if (to < from) {
    return -1;
  }
  return 0;
}

// Formats a passenger trip for status messages and activity logs.
std::string RequestText(int current, int destination) {
  std::ostringstream stream;
  stream << "floor " << current << " -> " << destination;
  return stream.str();
}

// Converts a signed direction value into status text.
std::string DirectionText(int direction) {
  if (direction > 0) {
    return "up";
  }
  if (direction < 0) {
    return "down";
  }
  return "stopped";
}

// Adds the travel distance and advances the simulated route position.
void AddTravel(int* total, int* position, int destination) {
  *total += std::abs(*position - destination);
  *position = destination;
}

// Adds one simulated second for pickup or drop-off door service.
void AddServiceStop(int* total) { ++*total; }

}  // namespace

// Stores non-owning elevator pointers used for dispatch decisions.
ElevatorSystem::ElevatorSystem(std::vector<Elevator*> elevators)
    : elevators_(std::move(elevators)) {}

// Computes a dispatch plan while serializing concurrent dispatch decisions.
DispatchPlan ElevatorSystem::PlanDispatch(int current, int destination) const {
  std::lock_guard<std::mutex> lock(dispatch_mutex_);
  return BuildDispatchPlan(current, destination);
}

// Selects the best elevator and submits the passenger request atomically.
DispatchResult ElevatorSystem::DispatchNearest(int current, int destination) {
  std::lock_guard<std::mutex> lock(dispatch_mutex_);

  // Build a route-aware plan before mutating any elevator queue.
  const auto plan = BuildDispatchPlan(current, destination);
  if (!plan.valid || plan.candidates.empty()) {
    return {false, 0, 0, "", plan.message};
  }

  // Use the top-ranked candidate produced by the dispatch planner.
  const auto& selected = plan.candidates.front();

  // Resolve the selected ID back to the live elevator instance.
  Elevator* elevator = FindElevator(selected.elevator_id);
  if (elevator == nullptr) {
    return {false, 0, 0, "", "No elevator is available."};
  }

  // Queue the passenger trip on the selected elevator worker.
  elevator->SubmitRequest(current, destination);

  // Return metadata used by the UI and remote dashboard.
  return {true, selected.elevator_id, selected.estimated_wait_seconds,
          selected.reason,
          "Auto dispatched elevator " + std::to_string(selected.elevator_id) +
              ": " + RequestText(current, destination)};
}

// Queues a passenger trip on a user-selected elevator.
DispatchResult ElevatorSystem::SubmitManual(int elevator_id, int current,
                                            int destination) {
  std::lock_guard<std::mutex> lock(dispatch_mutex_);

  // Reject requests outside the supported building range.
  if (!IsValidFloor(current) || !IsValidFloor(destination)) {
    return {false, 0, 0, "", "Floors must be from 1 to 10."};
  }

  // Reject trips that do not require elevator movement.
  if (current == destination) {
    return {false, 0, 0, "", "Current floor and desired floor are the same."};
  }

  // Resolve the user-selected ID to a live elevator instance.
  Elevator* elevator = FindElevator(elevator_id);
  if (elevator == nullptr) {
    return {false, 0, 0, "",
            "Elevator must be between 1 and " +
                std::to_string(ElevatorCount()) + "."};
  }

  // Estimate wait time for the selected elevator before queueing the trip.
  const auto candidate =
      BuildCandidate(elevator->Snapshot(), current, destination);

  // Queue the passenger trip on the selected elevator worker.
  elevator->SubmitRequest(current, destination);

  // Return metadata used by the UI and remote dashboard.
  return {true, elevator_id, candidate.estimated_wait_seconds, candidate.reason,
          "Manually queued elevator " + std::to_string(elevator_id) + ": " +
              RequestText(current, destination)};
}

// Queues a central-control command that sends one elevator to a floor.
DispatchResult ElevatorSystem::SendElevator(int elevator_id, int floor) {
  std::lock_guard<std::mutex> lock(dispatch_mutex_);

  // Reject direct sends outside the supported building range.
  if (!IsValidFloor(floor)) {
    return {false, 0, 0, "", "Floor must be from 1 to 10."};
  }

  // Resolve the requested elevator ID before queueing the command.
  Elevator* elevator = FindElevator(elevator_id);
  if (elevator == nullptr) {
    return {false, 0, 0, "",
            "Elevator must be between 1 and " +
                std::to_string(ElevatorCount()) + "."};
  }

  // Capture the pre-command state for a stable wait-time estimate.
  const auto snapshot = elevator->Snapshot();

  // Queue a direct move without a passenger pickup stage.
  elevator->SubmitDirectRequest(floor);

  // Return metadata used by the UI and remote dashboard.
  return {true, elevator_id, EstimateWaitSeconds(snapshot, floor),
          "central control direct send",
          "Sending elevator " + std::to_string(elevator_id) + " to floor " +
              std::to_string(floor)};
}

// Returns per-elevator state copies without exposing mutable elevator data.
std::vector<ElevatorSnapshot> ElevatorSystem::Snapshots() const {
  std::vector<ElevatorSnapshot> result;

  // Reserve the upper bound because null elevator pointers are skipped below.
  result.reserve(elevators_.size());

  for (const auto* elevator : elevators_) {
    if (elevator != nullptr) {
      // Each elevator synchronizes its own state before returning the copy.
      result.push_back(elevator->Snapshot());
    }
  }

  return result;
}

// Waits for asynchronous elevator workers to finish all active and queued work.
// Primarily used by tests as a deterministic synchronization point.
bool ElevatorSystem::WaitUntilAllIdle(std::chrono::milliseconds timeout) const {
  const auto deadline = std::chrono::steady_clock::now() + timeout;

  // Poll snapshots until either all elevators are idle or the timeout expires.
  while (std::chrono::steady_clock::now() < deadline) {
    bool all_idle = true;

    // A bank is idle only when every elevator has no active or pending request.
    for (const auto& snapshot : Snapshots()) {
      if (snapshot.busy || !snapshot.queued_requests.empty()) {
        all_idle = false;
        break;
      }
    }

    if (all_idle) {
      return true;
    }

    // Avoid busy-waiting while keeping tests responsive.
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  return false;
}

// Returns the number of configured elevator slots.
int ElevatorSystem::ElevatorCount() const {
  return static_cast<int>(elevators_.size());
}

// Returns whether a floor is within the building range supported by Elevator.
bool ElevatorSystem::IsValidFloor(int floor) const {
  return floor >= Elevator::kMinFloor && floor <= Elevator::kMaxFloor;
}

// Resolves a user-facing elevator ID to a live non-owning elevator pointer.
Elevator* ElevatorSystem::FindElevator(int elevator_id) const {
  for (auto* elevator : elevators_) {
    if (elevator != nullptr && elevator->Snapshot().id == elevator_id) {
      return elevator;
    }
  }

  return nullptr;
}

// Builds a ranked dispatch plan without submitting the passenger request.
DispatchPlan ElevatorSystem::BuildDispatchPlan(int current,
                                               int destination) const {
  // Reject requests outside the supported building range.
  if (!IsValidFloor(current) || !IsValidFloor(destination)) {
    return {false, 0, {}, "Floors must be from 1 to 10."};
  }

  // Reject trips that do not require elevator movement.
  if (current == destination) {
    return {false, 0, {}, "Current floor and desired floor are the same."};
  }

  std::vector<DispatchCandidate> candidates;
  // Reserve the upper bound; Snapshots() skips null elevator pointers.
  candidates.reserve(elevators_.size());

  // Convert each live elevator snapshot into a scored dispatch candidate.
  for (const auto& snapshot : Snapshots()) {
    candidates.push_back(BuildCandidate(snapshot, current, destination));
  }

  // Rank by wait time, route fit, queue load, then ID for deterministic output.
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

  // No candidate means no configured live elevator can serve the request.
  if (candidates.empty()) {
    return {false, 0, {}, "No elevator is available."};
  }

  // The first ranked candidate is the selected elevator.
  return {true, candidates.front().elevator_id, candidates,
          "Dispatch plan ready."};
}

// Scores one elevator snapshot for a passenger request.
DispatchCandidate ElevatorSystem::BuildCandidate(
    const ElevatorSnapshot& snapshot, int current, int destination) const {
  // Compare the elevator route with the passenger's requested direction.
  const int passenger_direction = DirectionOf(current, destination);
  const bool same_direction =
      snapshot.direction != 0 && snapshot.direction == passenger_direction;

  // Estimate how long this elevator needs to reach the caller.
  const int estimated_wait = EstimateWaitSeconds(snapshot, current);

  // Build a short explanation for UI and remote dispatch output.
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

  // Return the fields used for ranking and dispatch explanation.
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

// Estimates when this elevator can reach a new caller after its existing route.
int ElevatorSystem::EstimateWaitSeconds(const ElevatorSnapshot& snapshot,
                                        int current) const {
  int total = 0;
  int position = snapshot.current_floor;

  // Start with the active request because it is already being served.
  if (snapshot.active_request.has_value()) {
    const auto active = *snapshot.active_request;
    if (snapshot.stage == ElevatorStage::kToPickup) {
      // The car must still pick up this passenger, then complete the trip.
      AddTravel(&total, &position, active.current);
      AddServiceStop(&total);
      AddTravel(&total, &position, active.destination);
      AddServiceStop(&total);
    } else if (snapshot.stage == ElevatorStage::kBoarding) {
      // Pickup travel is complete; only the destination leg remains.
      position = active.current;
      AddTravel(&total, &position, active.destination);
      AddServiceStop(&total);
    } else if (snapshot.stage == ElevatorStage::kToDestination) {
      // The passenger is already onboard, so continue to the destination.
      AddTravel(&total, &position, active.destination);
      AddServiceStop(&total);
    } else if (snapshot.stage == ElevatorStage::kArrived) {
      // Travel is complete; future work starts from the arrival floor.
      position = active.destination;
    }
  }

  // Replay queued requests in FIFO order after the active request finishes.
  for (const auto& queued : snapshot.queued_requests) {
    if (queued.type == ElevatorRequestType::kPassengerTrip) {
      // Passenger trips require a pickup stop before the destination.
      AddTravel(&total, &position, queued.current);
      AddServiceStop(&total);
    }
    // Both passenger trips and direct sends end with destination service.
    AddTravel(&total, &position, queued.destination);
    AddServiceStop(&total);
  }

  // Finally, measure how long it takes to reach the new caller floor.
  AddTravel(&total, &position, current);
  return total;
}

}  // namespace elevator_simulator
