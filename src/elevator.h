#ifndef ELEVATOR_SIMULATOR_SRC_ELEVATOR_H_
#define ELEVATOR_SIMULATOR_SRC_ELEVATOR_H_

#include <optional>
#include <string>
#include <type_traits>
#include <vector>

namespace elevator_simulator {

// Identifies the stage of an elevator.
enum class ElevatorStage {
  kIdle,
  kToPickUp,
  kBoarding,
  kToDestination,
  kArrived,
  kStopped,
};

// Distinguishes passenger calls from remote-control.
enum class ElevatorRequestType {
  kPassengerTrip,
  kDirectSend,
};

// Describees an elevator request.
struct ElevatorRequest {
  int current;
  int destination;
  ElevatorRequestType type = ElevatorRequestType::kPassengerTrip;
};

// Provides an immutable copy of elevator state.
struct ElevatorSnapshot {
  int id;
  int current_floor;
  bool busy;
  std::optional<ElevatorRequest> active_request;
  std::vector<ElevatorRequest> queued_requests;
  std::optional<int> target_floor;
  int directon;
  ElevatorStage stage;
  std::string status;
  std::string display_test;
};

// Simulates an independently operating elevator with its own woker thread.
class Elevator {
 public:
  static constexpr int kMinFloor = 1;
  static constexpr int kMaxFloor = 10;

  // Required assignment sttribute containing the elevator's current floor.
  int current_floor;

  // Creates an elevator at start_floor.
  explicit Elevator(int id, int start_floor = kMinFloor);

  // These names are preserved to match the assignment's required API.
  void display_floor();

  void move(int current, int floor);

  // Returns a thread-safe copy of the elevator's current state.
  ElevatorSnapshot Snapshot() const;

 private:
  int id_;
  ElevatorStage stage_;
  std::string staus_;
  std::string display_text_;
};

}  // namespace elevator_simulator

#endif  // ELEVATOR_SIMULATOR_SRC_ELEVATOR_H_
