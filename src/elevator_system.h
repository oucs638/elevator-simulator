// Defines route-aware dispatching for a shared bank of elevators.

#ifndef ELEVATOR_SIMULATOR_SRC_ELEVATOR_SYSTEM_H_
#define ELEVATOR_SIMULATOR_SRC_ELEVATOR_SYSTEM_H_

#include <chrono>
#include <mutex>
#include <string>
#include <vector>

#include "elevator.h"

namespace elevator_simulator {

// Stores one elevator's rank and estimated wait in a dispatch plan.
struct DispatchCandidate {
  int elevator_id;
  int estimated_wait_seconds;
  int queued_trips;
  bool idle;
  bool same_direction;
  std::string direction;
  std::string reason;
};

// Stores the ordered candidates considered for an automatic dispatch.
struct DispatchPlan {
  bool valid;
  int selected_elevator_id;
  std::vector<DispatchCandidate> candidates;
  std::string message;
};

// Reports whether a request was accepted and which elevator received it.
struct DispatchResult {
  bool accepted;
  int elevator_id;
  int estimated_wait_seconds;
  std::string reason;
  std::string message;
};

// Coordinates passenger requests and direct sends across multiple elevators.
class ElevatorSystem {
 public:
  explicit ElevatorSystem(std::vector<Elevator*> elevators);

  // Ranks elevators for a passenger journey without submitting the request.
  DispatchPlan PlanDispatch(int current, int destination) const;

  // Selects and queues the elevator with the lowest estimated passenger wait.
  DispatchResult DispatchNearest(int current, int destination);

  // Queues a passenger journey on a user-selected elevator.
  DispatchResult SubmitManual(int elevator_id, int current, int destination);

  // Queues a direct central-control movement on a selected elevator.
  DispatchResult SendElevator(int elevator_id, int floor);

  // Returns snapshots for every elevator in the bank.
  std::vector<ElevatorSnapshot> Snapshots() const;

  // Waits until every elevator has completed active and queued requests.
  bool WaitUntilAllIdle(std::chrono::milliseconds timeout) const;

  // Returns the number of elevators managed by the system.
  int ElevatorCount() const;

 private:
  std::vector<Elevator*> elevators_;
  mutable std::mutex dispatch_mutex_;

  bool IsValidFloor(int floor) const;
  Elevator* FindElevator(int elevator_id) const;

  // Builds and sorts dispatch candidates while dispatch_mutex_ is held.
  DispatchPlan BuildDispatchPlan(int current, int destination) const;

  // Creates the dispatch metadata for one elevator.
  DispatchCandidate BuildCandidate(const ElevatorSnapshot& snapshot,
                                   int current, int destination) const;
  // Simulates pending route segments to estimate passenger waiting time.
  int EstimateWaitSeconds(const ElevatorSnapshot& snapshot, int current) const;
};

}  // namespace elevator_simulator

#endif  // ELEVATOR_SIMULATOR_SRC_ELEVATOR_SYSTEM_H_
