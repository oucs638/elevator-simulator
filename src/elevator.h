// Defines the state and behavior of one elevator car.

#ifndef ELEVATOR_SIMULATOR_SRC_ELEVATOR_H_
#define ELEVATOR_SIMULATOR_SRC_ELEVATOR_H_

#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace elevator_simulator {

// Identifies the stage of an elevator.
enum class ElevatorStage {
  kIdle,
  kToPickup,
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
  int direction;
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
  explicit Elevator(
      int id, int start_floor = kMinFloor,
      std::chrono::milliseconds floor_delay = std::chrono::milliseconds(1000));
  ~Elevator();

  // Disable copy constructor and copy assignment operator.
  Elevator(const Elevator&) = delete;
  Elevator& operator=(const Elevator&) = delete;

  // Starts the worker thread that processes queued requests.
  void Start();

  // Stops movement, clears queued requests, and joins the worker thread.
  void Stop();

  // Queues a passenger pickup and destination journey.
  void SubmitRequest(int current, int floor);

  // Queues a central-control command that moves directly to a floor.
  void SubmitDirectRequest(int floor);

  // These names are preserved to match the assignment's required API.
  void display_floor();

  void move(int current, int floor);

  // Returns a thread-safe copy of the elevator's current state.
  ElevatorSnapshot Snapshot() const;

  // Waits until no active or queued request remains, or until timeout.
  bool WaitUntilIdle(std::chrono::milliseconds timeout) const;

 private:
  int id_;
  std::chrono::milliseconds floor_delay_;
  std::chrono::milliseconds door_delay_;

  // Thread synchronization mechanism, mutable to allow locking in const member
  // functions.
  mutable std::mutex mutex_;
  mutable std::condition_variable idle_cv_;

  std::condition_variable queue_cv_;
  std::deque<ElevatorRequest> queue_;
  std::optional<ElevatorRequest> active_request_;
  std::thread worker_;
  bool running_;
  bool worker_started_;
  bool busy_;
  std::optional<int> target_floor_;
  ElevatorStage stage_;
  std::string status_;
  std::string display_text_;

  // Processes queued requests in first-in, first-out order.
  void WorkerLoop();

  // Executes a direct central-control movement without a pickup stop.
  void MoveDirect(int floor);

  // Travels one floor at a time while publishing the requested stage.
  bool TravelTo(int target, ElevatorStage stage, const std::string& status);

  // Waits for a duration while allowing Stop() to interrupt immediately.
  bool SleepInterruptibly(std::chrono::milliseconds duration) const;
  void SetStatus(ElevatorStage stage, const std::string& status);
  void FinishMove(const std::string& status);
  void DisplayFloorLocked();
};

}  // namespace elevator_simulator

#endif  // ELEVATOR_SIMULATOR_SRC_ELEVATOR_H_
