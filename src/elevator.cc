// Implemnts one elevator's movement state machine and request worker.

#include "elevator.h"

#include <algorithm>
#include <sstream>

namespace elevator_simulator {

// Explicit constructor that initializes configuration parameters,
// enforces floor boundaries, and prepares runtime state control flags.
Elevator::Elevator(int id, int start_floor,
                   std::chrono::milliseconds floor_delay)
    : current_floor(std::clamp(start_floor, kMinFloor, kMaxFloor)),
      id_(id),
      floor_delay_(floor_delay),
      door_delay_(floor_delay.count() == 0 ? std::chrono::milliseconds(0)
                                           : std::chrono::milliseconds(400)),
      running_(true),
      worker_started_(false),
      busy_(false),
      stage_(ElevatorStage::kIdle) {
  status_ = "Idle";
  // Safely refresh the initial UI text layout before any worker thread runs.
  DisplayFloorLocked();
}

// Destructor that automatically ensures the background thread is synchronously
// terminated and all shared resource lifecycles are properly aligned.
Elevator::~Elevator() {
  // Trigger graceful shutdown and join the worker thread before memory
  // deallocation.
  Stop();
}

// Spins up the background worker thread to start processing elevator requests.
// Guarantees safe initialization and prevents accidental double-activation.
void Elevator::Start() {
  // Lock must be held to check flags and construct the thread atomically.
  std::lock_guard<std::mutex> lock(mutex_);
  if (worker_started_) {
    return;
  }

  // Set control flags prior to thread birth to prevent initialization race
  // conditions.
  running_ = true;
  worker_started_ = true;

  // Spawn the background universe, binding the member function to the current
  // object instance.
  worker_ = std::thread(&Elevator::WorkerLoop, this);
}

// Stops the elevator operation and shuts down the background thread safely.
// Blocks the caller until the worker thread has completely terminated.
void Elevator::Stop() {
  {
    // Modify running state and purge queue under lock protection.
    std::lock_guard<std::mutex> lock(mutex_);
    running_ = false;
    queue_.clear();
  }  // Scope ends here to release the lock early and prevent deadlocks.

  // Wake up all threads sleeping on condition variables to let them exit.
  queue_cv_.notify_all();
  idle_cv_.notify_all();

  // Synchronously join the worker thread to prevent dangling references during
  // destruction.
  if (worker_.joinable()) {
    worker_.join();
  }
}

// Appends a passenger trip request to the dispatch queue.
// Signals the background worker thread to process the new task.
void Elevator::SubmitRequest(int current, int floor) {
  {
    // Package and enqueue the request under lock protection.
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push_back({current, floor, ElevatorRequestType::kPassengerTrip});
    if (!busy_) {
      status_ = "Request queued";
    }
  }  // Lock is released here before signaling to maximize concurrency.

  // Wake up a single sleeping worker thread to prevent thundering herd.
  queue_cv_.notify_one();
}

// Appends a direct send command from remote control to the dispatch queue.
// Assumes the current elevator floor as the starting point.
void Elevator::SubmitDirectRequest(int floor) {
  {
    // Package and enqueue the command under lock protection.
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push_back({current_floor, floor, ElevatorRequestType::kDirectSend});
    if (!busy_) {
      status_ = "Remote send queued";
    }
  }  // Lock is released here before signaling to maximize concurrency.

  // Wake up a single sleeping worker thread to prevent thundering herd.
  queue_cv_.notify_one();
}

// Explicitly refreshes and displays the current floor under lock protection.
void Elevator::display_floor() {
  std::lock_guard<std::mutex> lock(mutex_);
  DisplayFloorLocked();
}

// Executes a full passenger trip sequence synchronously on the calling thread.
// Handles pickup, passenger boarding delays, and transit to final destination.
void Elevator::move(int current, int floor) {
  {
    // Initialize initial state machine variables and text markers under lock.
    std::lock_guard<std::mutex> lock(mutex_);
    busy_ = true;
    active_request_ = ElevatorRequest{current, floor};
    target_floor_ = current;
    stage_ = ElevatorStage::kToPickup;
    status_ = "Going to passenger floor " + std::to_string(current);
    DisplayFloorLocked();
  }  // Lock released early to allow non-blocking physics movement.

  // Phase 1: Transit to the passenger's floor.
  if (!TravelTo(current, ElevatorStage::kToPickup,
                "Going to passenger floor " + std::to_string(current))) {
    FinishMove("Stopped");
    return;
  }

  // Phase 2: Open doors and wait for the passenger to board.
  SetStatus(ElevatorStage::kBoarding,
            "Passenger boarding at floor " + std::to_string(current));
  display_floor();
  if (!SleepInterruptibly(door_delay_)) {
    FinishMove("Stopped");
    return;
  }

  // Phase 3: Transit to the final destination floor.
  if (!TravelTo(floor, ElevatorStage::kToDestination,
                "Going to destination floor " + std::to_string(floor))) {
    FinishMove("Stopped");
    return;
  }

  // Phase 4: Arrive and open doors before entering idle shutdown.
  SetStatus(ElevatorStage::kArrived,
            "Arrived at floor " + std::to_string(floor));
  display_floor();
  SleepInterruptibly(door_delay_);
  FinishMove("Idle");
}

// Executes a direct remote dispatch sequence synchronously on the calling
// thread. Transitions the elevator straight to the destination without any
// mid-way pickup.
void Elevator::MoveDirect(int floor) {
  {
    // Initialize direct send metadata and state machine flags under lock.
    std::lock_guard<std::mutex> lock(mutex_);
    busy_ = true;
    active_request_ =
        ElevatorRequest{current_floor, floor, ElevatorRequestType::kDirectSend};
    target_floor_ = floor;
    stage_ = ElevatorStage::kToDestination;
    status_ =
        "Remote control sending elevator to floor " + std::to_string(floor);
    DisplayFloorLocked();
  }  // Lock released early to unblock state monitoring during transit.

  // Phase 1: Transit straight to the specified floor.
  if (!TravelTo(floor, ElevatorStage::kToDestination,
                "Remote control sending elevator to floor " +
                    std::to_string(floor))) {
    FinishMove("Stopped");
    return;
  }

  // Phase 2: Open doors upon arrival before releasing the elevator back to
  // Idle.
  SetStatus(ElevatorStage::kArrived,
            "Remote send arrived at floor " + std::to_string(floor));
  display_floor();
  SleepInterruptibly(door_delay_);
  FinishMove("Idle");
}

// Generates a thread-safe, immutable snapshot of the current elevator state.
// Used primarily by UIs or dispatchers to query metrics without blocking
// workers.
ElevatorSnapshot Elevator::Snapshot() const {
  // Lock is required to maintain a consistent state view during copy
  // initialization.
  std::lock_guard<std::mutex> lock(mutex_);

  // Evaluate the movement direction safely by unpacking the optional target
  // floor.
  int direction = 0;
  if (target_floor_.has_value()) {
    direction = (*target_floor_ > current_floor)
                    ? 1
                    : (*target_floor_ < current_floor ? -1 : 0);
  }

  // Construct and return the snapshot struct by value, copying the underlying
  // deque into a vector.
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

// Blocks the caller until the elevator becomes idle or the timeout expires.
// Returns true if the elevator successfully reached the idle state, false on
// timeout.
bool Elevator::WaitUntilIdle(std::chrono::milliseconds timeout) const {
  // Lock is required by the condition variable to safely evaluate the idle
  // predicate.
  std::unique_lock<std::mutex> lock(mutex_);
  return idle_cv_.wait_for(lock, timeout,
                           [this] { return !busy_ && queue_.empty(); });
}

// Continuous background loop that consumes and executes elevator requests.
// Sleeps efficiently when idle and terminates cleanly upon shutdown.
void Elevator::WorkerLoop() {
  while (true) {
    ElevatorRequest request{};

    {
      // Wait passively until a new request is queued or shutdown is initiated.
      // Lambda predicate protects against spurious wakeups.
      std::unique_lock<std::mutex> lock(mutex_);
      queue_cv_.wait(lock, [this] { return !running_ || !queue_.empty(); });

      // Gracefully exit the thread if the system is shutting down and no tasks
      // remain.
      if (!running_ && queue_.empty()) {
        return;
      }

      // Fetch the next task and update the state machine within the critical
      // section.
      request = queue_.front();
      queue_.pop_front();
      busy_ = true;
      active_request_ = request;

      // Determine targets and update stage tags using strong-typed enums.
      target_floor_ = request.type == ElevatorRequestType::kDirectSend
                          ? request.destination
                          : request.current;
      stage_ = request.type == ElevatorRequestType::kDirectSend
                   ? ElevatorStage::kToDestination
                   : ElevatorStage::kToPickup;
      status_ = request.type == ElevatorRequestType::kDirectSend
                    ? "Remote send accepted"
                    : "Request accepted";
      DisplayFloorLocked();
    }  // Lock is released automatically here to unblock producers during
       // physical movement.

    // Execute time-consuming physical motion outside the critical section to
    // maximize throughput.
    if (request.type == ElevatorRequestType::kDirectSend) {
      MoveDirect(request.destination);
    } else {
      move(request.current, request.destination);
    }
  }
}

// Moves the elevator floor-by-floor toward the target destination.
// Returns true upon successful arrival, or false if interrupted by shutdown.
bool Elevator::TravelTo(int target, ElevatorStage stage,
                        const std::string& status) {
  while (true) {
    {
      // Phase 1: Update target parameters and state tags under lock.
      std::lock_guard<std::mutex> lock(mutex_);
      if (!running_) {
        return false;
      }

      target_floor_ = target;
      stage_ = stage;
      status_ = status;
      DisplayFloorLocked();

      // Stop processing if the elevator has already arrived at the target.
      if (current_floor == target) {
        return true;
      }
    }  // Lock released to allow concurrent status queries during physical
       // delay.

    // Simulate physical transit delay. Returns false if interrupted by
    // shutdown.
    if (!SleepInterruptibly(floor_delay_)) {
      return false;
    }

    {
      // Phase 2: Re-acquire lock to increment/decrement the physical floor
      // safely.
      std::lock_guard<std::mutex> lock(mutex_);
      if (!running_) {
        return false;
      }

      current_floor += (target > current_floor) ? 1 : -1;
      DisplayFloorLocked();
    }
  }
}

// Performs a simulation delay that can be immediately interrupted by a shutdown
// signal. Returns true if the sleep completed fully, or false if aborted early
// by shutdown.
bool Elevator::SleepInterruptibly(std::chrono::milliseconds duration) const {
  // Lock is mandatory for the condition variable to safely check the running
  // flag.
  std::unique_lock<std::mutex> lock(mutex_);
  return !idle_cv_.wait_for(lock, duration, [this] { return !running_; });
}

// Updates the elevator's status and stages, while resetting the target floor.
// This enforces an immediate state change and halts any further movement.
void Elevator::SetStatus(ElevatorStage stage, const std::string& status) {
  // Lock is required to atomically modify the state machine and refresh the
  // display.
  std::lock_guard<std::mutex> lock(mutex_);

  // Set target to current floor to effectively cancel any active movement
  // requests.
  target_floor_ = current_floor;
  stage_ = stage;
  status_ = status;
  DisplayFloorLocked();
}

// Cleans up the elevator state upon completing a movement and signals waiting
// threads. Transitions the state machine to idle or stopped and clears active
// requests.
void Elevator::FinishMove(const std::string& status) {
  {
    // Reset runtime flags and clear optional fields under lock protection.
    std::lock_guard<std::mutex> lock(mutex_);
    busy_ = false;
    active_request_.reset();  // Clear optional request snapshot.
    target_floor_.reset();    // Reset optional target floor to nullopt.

    // Determine the next stage and format the final text status.
    stage_ = status == "Idle" ? ElevatorStage::kIdle : ElevatorStage::kStopped;
    status_ = status + " at floor " + std::to_string(current_floor);
    DisplayFloorLocked();
  }  // Lock is released here to prevent lock contention during thread
     // notification.

  // Notify all threads waiting for the elevator to become idle (e.g.,
  // WaitUntilIdle()).
  idle_cv_.notify_all();
}

// Formats the current floor display text using a type-safe string stream.
// REQUIRES: The caller must hold `mutex_` before invoking this function.
void Elevator::DisplayFloorLocked() {
  std::ostringstream stream;
  stream << "Elevator " << id_ << ": floor " << current_floor;

  // Update the internal display text buffer.
  display_text_ = stream.str();
}

}  // namespace elevator_simulator
