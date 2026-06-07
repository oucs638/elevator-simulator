// Unit test for basic elevator movement and snapshots.

#include <chrono>
#include <functional>
#include <initializer_list>
#include <iostream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "../src/elevator.h"

namespace {

using elevator_simulator::Elevator;
using elevator_simulator::ElevatorRequestType;
using elevator_simulator::ElevatorStage;
using std::chrono_literals::operator""ms;

std::vector<std::string>* g_current_failures = nullptr;

// RAII manager that automates the initialization and cleanup of multiple
// elevators. Typically scoped within a single unit test to tightly control
// worker thread lifecycles.
class RunningElevators {
 public:
  // Starts all provided elevators sequentially upon constructor initialization.
  explicit RunningElevators(std::initializer_list<Elevator*> elevators)
      : elevators_(elevators) {
    for (auto* elevator : elevators_) {
      elevator->Start();
    }
  }

  // Synchronously stops and joins all managed elevator threads during
  // destruction.
  ~RunningElevators() {
    for (auto* elevator : elevators_) {
      elevator->Stop();
    }
  }

  // Disable copying to enforce exclusive ownership over the managed elevator
  // cluster.
  RunningElevators(const RunningElevators&) = delete;
  RunningElevators& operator=(const RunningElevators&) = delete;

 private:
  std::vector<Elevator*> elevators_;
};

// Evaluates a test condition without aborting execution on failure.
// Logs the failure message and returns the condition state to the caller.
bool Expect(bool condition, const std::string& message) {
  if (!condition) {
    // NOTE: g_current_failures must be properly synchronized or guarded by a
    // mutex if invoked concurrently across multiple elevator worker threads.
    g_current_failures->push_back(message);
  }
  return condition;
}

// Polls a generic condition lambda continuously until it evaluates to true or
// times out. Utilizes a Monotonic steady clock to guarantee accurate timeout
// enforcement.
template <typename Predicate>
bool WaitForCondition(Predicate predicate, std::chrono::milliseconds timeout) {
  // Use steady_clock to prevent distortions from system time adjustments (NTP
  // sync).
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    // Yield the CPU briefly to mitigate busy-waiting overhead during polling.
    std::this_thread::sleep_for(10ms);
  }
  // Perform a last-minute check right after timeout to avoid race condition
  // false-negatives.
  return predicate();
}

// Specialized helper that blocks the caller until the elevator reaches the
// expected stage. Conveniently wraps the generic generic polling mechanism with
// thread-safe snapshot inspection.
bool WaitForStage(const Elevator& elevator, ElevatorStage stage,
                  std::chrono::milliseconds timeout) {
  // Pass the elevator by reference to prevent copying, and sample the state via
  // thread-safe snapshots.
  return WaitForCondition(
      [&elevator, stage] { return elevator.Snapshot().stage == stage; },
      timeout);
}

// Test case to verify that the constructor enforces boundary constraints via
// std::clamp. Validates that invalid extreme input floors are automatically
// adjusted to valid limits.
void ConstructorClampsStartingFloor() {
  // Initialize test elevators with 0ms delay to achieve maximum execution
  // speed.
  Elevator below_minimum(1, -5, 0ms);
  Elevator above_maximum(2, 50, 0ms);

  // Assert that illegal starting floors are seamlessly normalized by the
  // defensive constructor.
  Expect(below_minimum.Snapshot().current_floor == Elevator::kMinFloor,
         "A starting floor below the building must clamp to kMinFloor.");
  Expect(above_maximum.Snapshot().current_floor == Elevator::kMaxFloor,
         "A starting floor above the building must clamp to kMaxFloor.");
}

// Test case to verify the complete lifecycle of a standard passenger move
// command. Validates that all post-movement runtime flags and optional objects
// are properly reset.
void DirectMoveArrivesAtDestination() {
  // Initialize with 0ms to bypass transit delays for high-speed automated
  // testing.
  Elevator elevator(1, 1, 0ms);

  // Issue a passenger trip command.
  elevator.move(3, 8);

  // NOTE: In a strictly concurrent environment, a WaitForStage() barrier should
  // be invoked here to prevent race condition false-negatives before capturing
  // the snapshot.
  const auto snapshot = elevator.Snapshot();

  // Assert that the state machine reaches full convergence post-execution.
  Expect(snapshot.current_floor == 8,
         "A direct move must end at the destination.");
  Expect(!snapshot.busy, "The elevator must be idle after a direct move.");
  Expect(snapshot.stage == ElevatorStage::kIdle,
         "The completed move must return to Idle.");
  Expect(!snapshot.active_request.has_value(),
         "The completed move must clear its active request.");
}

// Test case to verify that Snapshot() correctly reports dynamic states during
// active transit. Validates physical directions, target floors, and active
// optional requests concurrently.
void SnapshotReportsDirectionTargetAndStage() {
  // Inject a 100ms floor delay to stretch the movement window for mid-transit
  // sampling.
  Elevator elevator(1, 1, 100ms);
  RunningElevators running(
      {&elevator});  // Automatically starts the background worker.

  // Enqueue a request and block the test thread until the elevator begins
  // moving.
  elevator.SubmitRequest(1, 5);
  Expect(WaitForStage(elevator, ElevatorStage::kToDestination, 1000ms),
         "The elevator must begin its destination stage.");

  // Capture a snapshot mid-transit, guaranteed to be thread-safe due to the
  // sync barrier above.
  const auto snapshot = elevator.Snapshot();

  // Assert dynamic metrics calculated by the running state machine.
  Expect(snapshot.direction == 1,
         "An upward trip must report an upward direction.");
  Expect(snapshot.target_floor == 5,
         "The active target must be the destination floor.");

  // Defensively verify the optional payload has value before accessing its
  // underlying fields.
  if (Expect(snapshot.active_request.has_value(),
             "A moving elevator must expose its active request.")) {
    Expect(snapshot.active_request->destination == 5,
           "The active request must retain its destination.");
  }
}  // `running` goes out of scope here, triggering RAII destructor to safely
   // stop the elevator thread.

// Test case to verify that WaitUntilIdle correctly enforces its timeout
// deadline. Ensures the function aborts early and returns false when the
// elevator remains active.
void WaitUntilIdleTimesOutWhileMoving() {
  // Inject a large floor delay (300ms) to ensure long transit duration
  // (approx. 2.7s total).
  Elevator elevator(1, 1, 300ms);
  RunningElevators running(
      {&elevator});  // Spawns the background worker universe.

  // Dispatch a heavy long-distance task (1 to 10).
  elevator.SubmitRequest(1, 10);
  Expect(WaitForStage(elevator, ElevatorStage::kToDestination, 1000ms),
         "The elevator must begin moving before the timeout check.");

  // Request idle state with a tight 50ms timeout. Since 50ms << 2700ms transit
  // time, the condition variable must time out and return false, which we
  // invert via '!'.
  Expect(!elevator.WaitUntilIdle(50ms),
         "WaitUntilIdle must time out while the elevator is moving.");
}  // RAII cleanup: `running` destruction safely joins the moving elevator
   // worker thread.

// Test case to verify that Stop() immediately interrupts an active transit,
// flushes all pending requests, and synchronizes termination cleanly.
void StopInterruptsActiveTripAndClearsQueue() {
  // Inject a large floor delay (500ms) to create a wide window for mid-transit
  // interruption.
  Elevator elevator(1, 1, 500ms);
  elevator.Start();

  // Enqueue multiple requests: one active trip and one queued background task.
  elevator.SubmitRequest(1, 10);
  elevator.SubmitDirectRequest(3);

  // Synchronously block until the elevator worker thread is actively moving.
  Expect(WaitForStage(elevator, ElevatorStage::kToDestination, 1000ms),
         "The elevator must begin its active trip before stop is tested.");

  // Benchmark the execution time of Stop() to verify non-blocking interruptible
  // sleep.
  const auto started = std::chrono::steady_clock::now();
  elevator.Stop();
  const auto elapsed = std::chrono::steady_clock::now() - started;
  const auto snapshot = elevator.Snapshot();

  // Assert prompt interruption, complete queue purge, and state convergence to
  // Stopped.
  Expect(elapsed < 500ms, "stop must interrupt movement promptly.");
  Expect(!snapshot.busy, "A stopped elevator must not remain busy.");
  Expect(snapshot.queued_requests.empty(), "stop must clear queued requests.");
  Expect(snapshot.stage == ElevatorStage::kStopped,
         "An interrupted trip must report Stopped.");
  Expect(snapshot.current_floor < 10,
         "The interrupted elevator must not reach its destination.");
}

// Test case to verify that enqueued elevator requests are processed in
// First-In, First-Out order. Captures a dynamic mid-transit snapshot to inspect
// the boundary between active and queued states.
void QueuedRequestsRunInFifoOrder() {
  // Inject a 40ms floor delay to expand the execution window for precise
  // concurrent sampling.
  Elevator elevator(1, 1, 40ms);
  RunningElevators running({&elevator});  // Spawns the worker thread.

  // Enqueue two distinct requests sequentially.
  elevator.SubmitRequest(1, 3);
  elevator.SubmitDirectRequest(7);

  // Synchronously block until the first task becomes active while the second is
  // still stuck in the queue.
  if (!Expect(WaitForCondition(
                  [&elevator] {
                    const auto snapshot = elevator.Snapshot();
                    return snapshot.active_request.has_value() &&
                           snapshot.active_request->destination == 3 &&
                           snapshot.queued_requests.size() == 1;
                  },
                  1000ms),
              "The first request must become active while the second remains "
              "queued.")) {
    return;  // Early return to avoid out-of-bounds access on failed vector
             // states.
  }

  // Sample the mid-transit state to thoroughly inspect the properties of the
  // queued command.
  const auto snapshot = elevator.Snapshot();
  if (!Expect(!snapshot.queued_requests.empty(),
              "The second request must remain queued.")) {
    return;
  }

  // Assert the secondary task retains its unmutated metadata within the queue.
  const auto queued = snapshot.queued_requests.front();
  Expect(queued.type == ElevatorRequestType::kDirectSend,
         "The second request must remain direct-send.");
  Expect(queued.destination == 7,
         "The second request must retain its destination.");

  // Allow both tasks to flush fully and assert final spatial convergence at the
  // last destination.
  Expect(elevator.WaitUntilIdle(3000ms), "Both queued requests must finish.");
  Expect(elevator.Snapshot().current_floor == 7,
         "FIFO processing must finish at the second destination.");
}

// Test case to benchmark and verify true concurrency across multiple
// independent instances. Ensures that multiple elevators operate in parallel
// without suffering from bad lock contention.
void TwoElevatorsMoveConcurrently() {
  // Initialize two elevators with contrasting routes to observe simultaneous
  // multi-threaded transit.
  Elevator elevator_1(1, 1, 20ms);
  Elevator elevator_2(2, 10, 20ms);
  RunningElevators running(
      {&elevator_1,
       &elevator_2});  // Concurrently spawns both background threads.

  // Benchmark start point utilizing the monotonic steady clock.
  const auto started = std::chrono::steady_clock::now();
  elevator_1.SubmitRequest(1, 10);
  elevator_2.SubmitRequest(10, 1);

  // Block the test thread until both asynchronous trips converge safely.
  Expect(elevator_1.WaitUntilIdle(3000ms),
         "Elevator 1 must complete its concurrent trip.");
  Expect(elevator_2.WaitUntilIdle(3000ms),
         "Elevator 2 must complete its concurrent trip.");
  const auto elapsed = std::chrono::steady_clock::now() - started;

  // Assert that execution duration confirms overlapping timelines instead of
  // sequential execution.
  Expect(elapsed < 1800ms,
         "Both elevator trips must overlap instead of running sequentially.");
  Expect(elevator_1.Snapshot().current_floor == 10,
         "Elevator 1 must reach floor 10.");
  Expect(elevator_2.Snapshot().current_floor == 1,
         "Elevator 2 must reach floor 1.");
}

using TestFunction = std::function<void()>;

int RunTests(const std::vector<std::pair<std::string, TestFunction>>& tests) {
  int failures = 0;

  for (const auto& [name, test] : tests) {
    test();
    std::cout << "[ PASS ] " << name << "\n";
  }

  std::cout << tests.size() - failures << "/" << tests.size()
            << " tests passed.\n";
  return failures == 0 ? 0 : 1;
}

}  // namespace
int main() {
  return RunTests({
      {"ConstructorClampsStartingFloor", ConstructorClampsStartingFloor},
      {"DirectMoveArrivesAtDestination", DirectMoveArrivesAtDestination},
      {"SnapshotReportsDirectionTargetAndStage",
       SnapshotReportsDirectionTargetAndStage},
      {"WaitUntilIdleTimesOutWhileMoving", WaitUntilIdleTimesOutWhileMoving},
      {"StopInterruptsActiveTripAndClearsQueue",
       StopInterruptsActiveTripAndClearsQueue},
      {"QueuedRequestsRunInFifoOrder", QueuedRequestsRunInFifoOrder},
      {"TwoElevatorsMoveConcurrently", TwoElevatorsMoveConcurrently},
  });
}
