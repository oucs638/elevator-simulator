// Implemnts one elevator's movement state machine and request worker.

#include "elevator.h"

#include <algorithm>
#include <sstream>

namespace elevator_simulator {

// Initializes an elevator in a valid idle state without starting its worker.
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
  // Keep status fields consistent with the clamped starting floor.
  status_ = "Idle";
  DisplayFloorLocked();
}

// Ensures the worker thread is stopped before destroying elevator state.
Elevator::~Elevator() { Stop(); }

// Starts the elevator worker thread if it has not been started.
void Elevator::Start() {
  std::lock_guard<std::mutex> lock(mutex_);

  // Starting twice would create multiple consumers for the same request queue.
  if (worker_started_) {
    return;
  }

  // Mark the worker as active before exposing the thread to the scheduler.
  running_ = true;
  worker_started_ = true;

  // Build a background thread.
  worker_ = std::thread(&Elevator::WorkerLoop, this);
}

// Stops the worker thread and cancels pending elevator requests.
void Elevator::Stop() {
  {
    // Limit the lock scope so join() never waits while holding mutex_.
    std::lock_guard<std::mutex> lock(mutex_);

    // Signal the worker loop to exit and discard work that has not started.
    running_ = false;
    queue_.clear();
  }

  // Wake workers and waiters so they can observe the stopped state.
  queue_cv_.notify_all();
  idle_cv_.notify_all();

  // Join after releasing the mutex because the worker may lock it while
  // exiting.
  if (worker_.joinable()) {
    worker_.join();
  }
}

// Queues a passenger trip for the elevator worker.
void Elevator::SubmitRequest(int current, int floor) {
  {
    // Limit the lock scope so the worker can acquire mutex_ after notification.
    std::lock_guard<std::mutex> lock(mutex_);

    // Passenger trips include both pickup and destination service.
    queue_.push_back({current, floor, ElevatorRequestType::kPassengerTrip});

    // Do not overwrite active movement status while the car is already busy.
    if (!busy_) {
      status_ = "Request queued";
    }
  }

  // Wake the worker if it is waiting for new queued work.
  queue_cv_.notify_one();
}

// Queues a central-control direct send for the elevator worker.
void Elevator::SubmitDirectRequest(int floor) {
  {
    // Limit the lock scope so the worker can acquire mutex_ after notification.
    std::lock_guard<std::mutex> lock(mutex_);

    // Direct sends skip pickup service and move straight to the target floor.
    queue_.push_back({current_floor, floor, ElevatorRequestType::kDirectSend});

    // Do not overwrite active movement status while the car is already busy.
    if (!busy_) {
      status_ = "Remote send queued";
    }
  }

  // Wake the worker if it is waiting for new queued work.
  queue_cv_.notify_one();
}

// Locks shared elevator state before updating the assignment-facing display.
void Elevator::display_floor() {
  std::lock_guard<std::mutex> lock(mutex_);
  DisplayFloorLocked();
}

// Runs a complete passenger trip from pickup to destination.
void Elevator::move(int current, int floor) {
  {
    // Publish the active pickup target without holding the lock during travel.
    std::lock_guard<std::mutex> lock(mutex_);
    busy_ = true;
    active_request_ = ElevatorRequest{current, floor};
    target_floor_ = current;
    stage_ = ElevatorStage::kToPickup;
    status_ = "Going to passenger floor " + std::to_string(current);
    DisplayFloorLocked();
  }

  // Travel to the caller first; Stop() can interrupt between floors.
  if (!TravelTo(current, ElevatorStage::kToPickup,
                "Going to passenger floor " + std::to_string(current))) {
    FinishMove("Stopped");
    return;
  }

  // Simulate door service while the passenger boards at the caller floor.
  SetStatus(ElevatorStage::kBoarding,
            "Passenger boarding at floor " + std::to_string(current));
  display_floor();
  if (!SleepInterruptibly(door_delay_)) {
    FinishMove("Stopped");
    return;
  }

  // Carry the passenger to the requested destination floor.
  if (!TravelTo(floor, ElevatorStage::kToDestination,
                "Going to destination floor " + std::to_string(floor))) {
    FinishMove("Stopped");
    return;
  }

  // Show arrival briefly, then clear active state back to idle.
  SetStatus(ElevatorStage::kArrived,
            "Arrived at floor " + std::to_string(floor));
  display_floor();
  SleepInterruptibly(door_delay_);
  FinishMove("Idle");
}

// Runs a remote-control direct send without passenger pickup or boarding.
void Elevator::MoveDirect(int floor) {
  {
    // Publish the direct-send target without holding the lock during travel.
    std::lock_guard<std::mutex> lock(mutex_);
    busy_ = true;
    active_request_ =
        ElevatorRequest{current_floor, floor, ElevatorRequestType::kDirectSend};
    target_floor_ = floor;
    stage_ = ElevatorStage::kToDestination;
    status_ =
        "Remote control sending elevator to floor " + std::to_string(floor);
    DisplayFloorLocked();
  }

  // Move straight to the requested floor; Stop() can interrupt between floors.
  if (!TravelTo(floor, ElevatorStage::kToDestination,
                "Remote control sending elevator to floor " +
                    std::to_string(floor))) {
    FinishMove("Stopped");
    return;
  }

  // Show direct-send arrival briefly, then clear active state back to idle.
  SetStatus(ElevatorStage::kArrived,
            "Remote send arrived at floor " + std::to_string(floor));
  display_floor();
  SleepInterruptibly(door_delay_);
  FinishMove("Idle");
}

// Returns a locked copy of elevator state for UI, dispatching, and tests.
ElevatorSnapshot Elevator::Snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);

  // Derive display and dispatch direction from the active target floor.
  int direction = 0;
  if (target_floor_.has_value()) {
    direction = (*target_floor_ > current_floor)
                    ? 1
                    : (*target_floor_ < current_floor ? -1 : 0);
  }

  // Copy queued work so callers cannot mutate queue_ directly.
  const std::vector<ElevatorRequest> queued_requests(queue_.begin(),
                                                     queue_.end());

  return ElevatorSnapshot{
      id_,           current_floor, busy_,  active_request_, queued_requests,
      target_floor_, direction,     stage_, status_,         display_text_,
  };
}

// Waits until this elevator has no active or queued work, or times out.
bool Elevator::WaitUntilIdle(std::chrono::milliseconds timeout) const {
  // condition_variable requires unique_lock so it can unlock while waiting.
  std::unique_lock<std::mutex> lock(mutex_);
  // The predicate handles spurious wakeups and confirms the car is fully idle.
  return idle_cv_.wait_for(lock, timeout,
                           [this] { return !busy_ && queue_.empty(); });
}

// Consumes queued requests on the elevator worker thread.
void Elevator::WorkerLoop() {
  while (true) {
    ElevatorRequest request{};

    {
      std::unique_lock<std::mutex> lock(mutex_);

      // Wait until work arrives or Stop() asks the worker to exit.
      queue_cv_.wait(lock, [this] { return !running_ || !queue_.empty(); });

      // Exit only after Stop() is requested and no queued work remains.
      if (!running_ && queue_.empty()) {
        return;
      }

      // Remove one FIFO request and publish it as the active request.
      request = queue_.front();
      queue_.pop_front();
      busy_ = true;
      active_request_ = request;

      // Passenger trips target pickup first; direct sends target destination.
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
    }

    // Execute long-running movement outside the lock to keep state observable.
    if (request.type == ElevatorRequestType::kDirectSend) {
      MoveDirect(request.destination);
    } else {
      move(request.current, request.destination);
    }
  }
}

// Moves one floor at a time toward a target and returns false if stopped.
bool Elevator::TravelTo(int target, ElevatorStage stage,
                        const std::string& status) {
  while (true) {
    {
      // Publish the current travel target and status before each floor step.
      std::lock_guard<std::mutex> lock(mutex_);
      if (!running_) {
        return false;
      }

      target_floor_ = target;
      stage_ = stage;
      status_ = status;
      DisplayFloorLocked();

      if (current_floor == target) {
        return true;
      }
    }

    // Wait outside the lock so snapshots and Stop() remain responsive.
    if (!SleepInterruptibly(floor_delay_)) {
      return false;
    }

    {
      // Advance exactly one floor after the simulated travel delay.
      std::lock_guard<std::mutex> lock(mutex_);
      if (!running_) {
        return false;
      }

      current_floor += (target > current_floor) ? 1 : -1;
      DisplayFloorLocked();
    }
  }
}

// Sleeps for a delay but returns false if Stop() interrupts the wait.
bool Elevator::SleepInterruptibly(std::chrono::milliseconds duration) const {
  // wait_for needs unique_lock so it can release mutex_ while sleeping.
  std::unique_lock<std::mutex> lock(mutex_);

  // wait_for returns true when stopped, so invert it for "completed normally".
  return !idle_cv_.wait_for(lock, duration, [this] { return !running_; });
}

// Publishes a non-travel stage such as boarding or arrival.
void Elevator::SetStatus(ElevatorStage stage, const std::string& status) {
  std::lock_guard<std::mutex> lock(mutex_);

  // Non-travel stages occur at the current floor, so direction becomes stopped.
  target_floor_ = current_floor;
  stage_ = stage;
  status_ = status;
  DisplayFloorLocked();
}

// Clears active movement state and notifies waiters that the car may be idle.
void Elevator::FinishMove(const std::string& status) {
  {
    // Limit the lock scope so waiters can acquire mutex_ after notification.
    std::lock_guard<std::mutex> lock(mutex_);
    busy_ = false;

    // Clear the active request now that movement has finished.
    active_request_.reset();

    // Clear the active target so snapshots report no travel direction.
    target_floor_.reset();

    stage_ = status == "Idle" ? ElevatorStage::kIdle : ElevatorStage::kStopped;
    status_ = status + " at floor " + std::to_string(current_floor);
    DisplayFloorLocked();
  }

  // Wake WaitUntilIdle() and interruptible sleeps waiting on elevator state.
  idle_cv_.notify_all();
}

// Refreshes display_text_ from the current floor while mutex_ is held.
void Elevator::DisplayFloorLocked() {
  std::ostringstream stream;
  stream << "Elevator " << id_ << ": floor " << current_floor;

  display_text_ = stream.str();
}

}  // namespace elevator_simulator
