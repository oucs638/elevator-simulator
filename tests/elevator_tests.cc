// Unit tests for elevator movement, dispatching, concurrency, and remote
// commands.

#include <chrono>
#include <functional>
#include <initializer_list>
#include <iostream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "elevator.h"
#include "elevator_system.h"
#include "remote_control_server.h"

namespace {

using elevator_simulator::Elevator;
using elevator_simulator::ElevatorRequestType;
using elevator_simulator::ElevatorStage;
using elevator_simulator::ElevatorSystem;
using elevator_simulator::RemoteControlServer;
using std::chrono_literals::operator""ms;

// Points to the failure list for the currently running test case.
std::vector<std::string>* g_current_failures = nullptr;

// Starts and stops a collection of elevators for the lifetime of a test.
class RunningElevators {
 public:
  // Starts each elevator so asynchronous requests can be processed.
  explicit RunningElevators(std::initializer_list<Elevator*> elevators)
      : elevators_(elevators) {
    for (auto* elevator : elevators_) {
      elevator->Start();
    }
  }

  // Stops all elevators before the test scope exits.
  ~RunningElevators() {
    for (auto* elevator : elevators_) {
      elevator->Stop();
    }
  }

  RunningElevators(const RunningElevators&) = delete;
  RunningElevators& operator=(const RunningElevators&) = delete;

 private:
  std::vector<Elevator*> elevators_;
};

// Records a descriptive failure while allowing the remaining tests to run.
bool Expect(bool condition, const std::string& message) {
  if (!condition) {
    g_current_failures->push_back(message);
  }
  return condition;
}

// Waits for an asynchronous elevator state without using fixed sleeps.
template <typename Predicate>
bool WaitForCondition(Predicate predicate, std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(10ms);
  }
  return predicate();
}

// Waits until one elevator reports a specific movement stage.
bool WaitForStage(const Elevator& elevator, ElevatorStage stage,
                  std::chrono::milliseconds timeout) {
  return WaitForCondition(
      [&elevator, stage] { return elevator.Snapshot().stage == stage; },
      timeout);
}

// Verifies that constructor clamps invalid starting floors into range.
void ConstructorClampsStartingFloor() {
  Elevator below_minimum(1, -5, 0ms);
  Elevator above_maximum(2, 50, 0ms);

  Expect(below_minimum.Snapshot().current_floor == Elevator::kMinFloor,
         "A starting floor below the building must clamp to kMinFloor.");
  Expect(above_maximum.Snapshot().current_floor == Elevator::kMaxFloor,
         "A starting floor above the building must clamp to kMaxFloor.");
}

// Verifies that a passenger move ends at the requested destination.
void DirectMoveArrivesAtDestination() {
  Elevator elevator(1, 1, 0ms);

  elevator.move(3, 8);
  const auto snapshot = elevator.Snapshot();

  Expect(snapshot.current_floor == 8,
         "A direct move must end at the destination.");
  Expect(!snapshot.busy, "The elevator must be idle after a direct move.");
  Expect(snapshot.stage == ElevatorStage::kIdle,
         "The completed move must return to Idle.");
  Expect(!snapshot.active_request.has_value(),
         "The completed move must clear its active request.");
}

// Verifies that snapshots expose direction, target, and active request state.
void SnapshotReportsDirectionTargetAndStage() {
  Elevator elevator(1, 1, 100ms);
  RunningElevators running({&elevator});

  elevator.SubmitRequest(1, 5);
  Expect(WaitForStage(elevator, ElevatorStage::kToDestination, 1000ms),
         "The elevator must begin its destination stage.");

  const auto snapshot = elevator.Snapshot();
  Expect(snapshot.direction == 1,
         "An upward trip must report an upward direction.");
  Expect(snapshot.target_floor == 5,
         "The active target must be the destination floor.");
  if (Expect(snapshot.active_request.has_value(),
             "A moving elevator must expose its active request.")) {
    Expect(snapshot.active_request->destination == 5,
           "The active request must retain its destination.");
  }
}

// Verifies that waiting for idle can time out while the car is still moving.
void WaitUntilIdleTimesOutWhileMoving() {
  Elevator elevator(1, 1, 300ms);
  RunningElevators running({&elevator});

  elevator.SubmitRequest(1, 10);
  Expect(WaitForStage(elevator, ElevatorStage::kToDestination, 1000ms),
         "The elevator must begin moving before the timeout check.");
  Expect(!elevator.WaitUntilIdle(50ms),
         "WaitUntilIdle must time out while the elevator is moving.");
}

// Verifies that Stop() interrupts movement and clears queued work.
void StopInterruptsActiveTripAndClearsQueue() {
  Elevator elevator(1, 1, 500ms);
  elevator.Start();
  elevator.SubmitRequest(1, 10);
  elevator.SubmitDirectRequest(3);

  Expect(WaitForStage(elevator, ElevatorStage::kToDestination, 1000ms),
         "The elevator must begin its active trip before stop is tested.");

  const auto started = std::chrono::steady_clock::now();
  elevator.Stop();
  const auto elapsed = std::chrono::steady_clock::now() - started;
  const auto snapshot = elevator.Snapshot();

  Expect(elapsed < 500ms, "stop must interrupt movement promptly.");
  Expect(!snapshot.busy, "A stopped elevator must not remain busy.");
  Expect(snapshot.queued_requests.empty(), "stop must clear queued requests.");
  Expect(snapshot.stage == ElevatorStage::kStopped,
         "An interrupted trip must report Stopped.");
  Expect(snapshot.current_floor < 10,
         "The interrupted elevator must not reach its destination.");
}

// Verifies that queued requests are processed in FIFO order.
void QueuedRequestsRunInFifoOrder() {
  Elevator elevator(1, 1, 40ms);
  RunningElevators running({&elevator});

  elevator.SubmitRequest(1, 3);
  elevator.SubmitDirectRequest(7);

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
    return;
  }

  const auto snapshot = elevator.Snapshot();
  if (!Expect(!snapshot.queued_requests.empty(),
              "The second request must remain queued.")) {
    return;
  }
  const auto queued = snapshot.queued_requests.front();
  Expect(queued.type == ElevatorRequestType::kDirectSend,
         "The second request must remain direct-send.");
  Expect(queued.destination == 7,
         "The second request must retain its destination.");
  Expect(elevator.WaitUntilIdle(3000ms), "Both queued requests must finish.");
  Expect(elevator.Snapshot().current_floor == 7,
         "FIFO processing must finish at the second destination.");
}

// Verifies that independent elevator workers can move concurrently.
void TwoElevatorsMoveConcurrently() {
  Elevator elevator1(1, 1, 20ms);
  Elevator elevator2(2, 10, 20ms);
  RunningElevators running({&elevator1, &elevator2});

  const auto started = std::chrono::steady_clock::now();
  elevator1.SubmitRequest(1, 10);
  elevator2.SubmitRequest(10, 1);

  Expect(elevator1.WaitUntilIdle(3000ms),
         "Elevator 1 must complete its concurrent trip.");
  Expect(elevator2.WaitUntilIdle(3000ms),
         "Elevator 2 must complete its concurrent trip.");
  const auto elapsed = std::chrono::steady_clock::now() - started;

  // Sequential execution takes about two seconds because each trip includes
  // door delays.
  Expect(elapsed < 1800ms,
         "Both elevator trips must overlap instead of running sequentially.");
  Expect(elevator1.Snapshot().current_floor == 10,
         "Elevator 1 must reach floor 10.");
  Expect(elevator2.Snapshot().current_floor == 1,
         "Elevator 2 must reach floor 1.");
}

// Verifies that auto dispatch selects the nearest suitable elevator.
void NearestElevatorIsAutoDispatched() {
  Elevator elevator1(1, 1, 0ms);
  Elevator elevator2(2, 5, 0ms);
  Elevator elevator3(3, 9, 0ms);
  RunningElevators running({&elevator1, &elevator2, &elevator3});
  ElevatorSystem system({&elevator1, &elevator2, &elevator3});

  const auto result = system.DispatchNearest(8, 2);

  Expect(result.accepted, "A valid passenger request must be accepted.");
  Expect(result.elevator_id == 3,
         "The closest idle elevator must be selected.");
  Expect(system.WaitUntilAllIdle(1000ms), "The dispatched trip must complete.");
  Expect(elevator3.Snapshot().current_floor == 2,
         "The selected elevator must reach the destination.");
}

// Verifies that invalid passenger, manual, and direct-send requests fail.
void DispatchRejectsInvalidPassengerRequests() {
  Elevator elevator(1, 1, 0ms);
  ElevatorSystem system({&elevator});

  Expect(!system.DispatchNearest(0, 5).accepted,
         "A floor below kMinFloor must be rejected.");
  Expect(!system.DispatchNearest(5, 11).accepted,
         "A floor above kMaxFloor must be rejected.");
  Expect(!system.DispatchNearest(5, 5).accepted,
         "A zero-distance passenger trip must be rejected.");
  Expect(!system.SubmitManual(9, 1, 5).accepted,
         "An unknown manual elevator ID must be rejected.");
  Expect(!system.SendElevator(1, 0).accepted,
         "A direct send outside the building must be rejected.");
  Expect(!system.SendElevator(9, 5).accepted,
         "A direct send to an unknown elevator must be rejected.");
}

// Verifies that an empty elevator bank cannot produce a valid dispatch plan.
void EmptySystemCannotCreateDispatchPlan() {
  ElevatorSystem system({});

  const auto plan = system.PlanDispatch(3, 8);

  Expect(!plan.valid,
         "An empty elevator bank must not create a valid dispatch plan.");
  Expect(plan.candidates.empty(),
         "An empty elevator bank must not return candidates.");
}

// Verifies that manual dispatch queues work on the requested elevator only.
void ManualDispatchUsesRequestedElevator() {
  Elevator elevator1(1, 1, 0ms);
  Elevator elevator2(2, 5, 0ms);
  RunningElevators running({&elevator1, &elevator2});
  ElevatorSystem system({&elevator1, &elevator2});

  const auto result = system.SubmitManual(2, 5, 9);

  Expect(result.accepted, "A valid manual request must be accepted.");
  Expect(result.elevator_id == 2,
         "Manual dispatch must retain the requested elevator ID.");
  Expect(system.WaitUntilAllIdle(1000ms), "The manual trip must complete.");
  Expect(elevator1.Snapshot().current_floor == 1,
         "Manual dispatch must not move another elevator.");
  Expect(elevator2.Snapshot().current_floor == 9,
         "The requested elevator must reach the destination.");
}

// Verifies that dispatch scoring accounts for active and queued route work.
void DispatchUsesQueuedRouteInsteadOfCurrentPositionOnly() {
  Elevator busy_elevator(1, 5, 100ms);
  Elevator available_elevator(2, 1, 0ms);
  RunningElevators running({&busy_elevator, &available_elevator});
  busy_elevator.SubmitRequest(5, 10);

  Expect(WaitForStage(busy_elevator, ElevatorStage::kToDestination, 1000ms),
         "The busy elevator must begin its existing trip.");

  ElevatorSystem system({&busy_elevator, &available_elevator});
  const auto plan = system.PlanDispatch(5, 1);

  Expect(plan.valid, "A valid passenger request must produce a dispatch plan.");
  Expect(plan.selected_elevator_id == 2,
         "The dispatcher must select the lower-wait elevator.");
  if (Expect(plan.candidates.size() >= 2,
             "The plan must include both elevators.")) {
    Expect(plan.candidates.front().elevator_id == 2,
           "The best candidate must rank first.");
    Expect(plan.candidates.front().estimated_wait_seconds <
               plan.candidates.back().estimated_wait_seconds,
           "The available elevator must have a shorter estimated wait.");
  }
}

// Verifies that direction match breaks equal-wait dispatch ties.
void DispatchUsesDirectionAsTieBreaker() {
  Elevator moving_up(1, 1, 1000ms);
  Elevator idle_above_caller(2, 8, 0ms);
  RunningElevators running({&moving_up, &idle_above_caller});
  moving_up.SubmitRequest(1, 3);

  Expect(WaitForStage(moving_up, ElevatorStage::kToDestination, 1000ms),
         "The first elevator must begin moving upward.");

  ElevatorSystem system({&moving_up, &idle_above_caller});
  const auto plan = system.PlanDispatch(4, 10);

  Expect(plan.valid, "A valid passenger request must produce a plan.");
  if (Expect(plan.candidates.size() >= 2,
             "The plan must include both elevators.")) {
    Expect(plan.candidates[0].estimated_wait_seconds ==
               plan.candidates[1].estimated_wait_seconds,
           "The test setup must produce equal estimated waits.");
    Expect(plan.candidates[0].same_direction,
           "The first candidate must match passenger direction.");
  }
  Expect(plan.selected_elevator_id == 1,
         "Direction must break an equal-wait tie.");
}

// Verifies that remote call commands return trackable activity events.
void RemoteCallReturnsTrackableEvent() {
  Elevator elevator1(1, 1, 0ms);
  Elevator elevator2(2, 5, 0ms);
  Elevator elevator3(3, 9, 0ms);
  RunningElevators running({&elevator1, &elevator2, &elevator3});
  ElevatorSystem system({&elevator1, &elevator2, &elevator3});

  const std::string response =
      RemoteControlServer::ExecuteCommand(&system, "call 8 2");

  Expect(response.find("EVENT|3|") != std::string::npos,
         "Remote call must identify its elevator.");
  Expect(response.find("Call 8 -> 2 assigned to E3") != std::string::npos,
         "Remote call must describe its assignment.");
}

// Verifies that remote send commands move only the selected elevator.
void RemoteSendMovesOnlySelectedElevator() {
  Elevator elevator1(1, 1, 0ms);
  Elevator elevator2(2, 5, 0ms);
  RunningElevators running({&elevator1, &elevator2});
  ElevatorSystem system({&elevator1, &elevator2});

  const std::string response =
      RemoteControlServer::ExecuteCommand(&system, "send 2 9");

  Expect(response.find("Sending elevator 2 to floor 9") != std::string::npos,
         "Remote send must describe the selected elevator and floor.");
  Expect(elevator2.WaitUntilIdle(1000ms),
         "The direct-send trip must complete.");
  Expect(elevator2.Snapshot().current_floor == 9,
         "The selected elevator must reach floor 9.");
  Expect(elevator1.Snapshot().current_floor == 1,
         "Remote send must not move another elevator.");
}

// Verifies remote help, status, validation, and unknown-command responses.
void RemoteCommandsValidateSyntaxAndReportStatus() {
  Elevator elevator(1, 4, 0ms);
  ElevatorSystem system({&elevator});

  const std::string help = RemoteControlServer::ExecuteCommand(&system, "help");
  const std::string status =
      RemoteControlServer::ExecuteCommand(&system, "status");
  const std::string malformed_call =
      RemoteControlServer::ExecuteCommand(&system, "call 3");
  const std::string invalid_floor =
      RemoteControlServer::ExecuteCommand(&system, "call 0 5");
  const std::string unknown =
      RemoteControlServer::ExecuteCommand(&system, "open doors");

  Expect(help.find("call <from> <to>") != std::string::npos,
         "Remote help must document call.");
  Expect(status.find("E1 | floor 4") != std::string::npos,
         "Remote status must report live floors.");
  Expect(malformed_call.find("Use: call") != std::string::npos,
         "Malformed calls must return usage guidance.");
  Expect(invalid_floor.find("Floors must be from 1 to 10") != std::string::npos,
         "Out-of-range calls must return a validation error.");
  Expect(unknown.find("Unknown command") != std::string::npos,
         "Unknown remote commands must return a clear error.");
}

// Stores one named no-argument test case.
using TestFunction = std::function<void()>;

// Runs all tests, prints individual results, and returns a process status.
int RunTests(const std::vector<std::pair<std::string, TestFunction>>& tests) {
  int failures = 0;

  for (const auto& [name, test] : tests) {
    std::vector<std::string> failure_messages;
    g_current_failures = &failure_messages;
    test();
    g_current_failures = nullptr;

    if (failure_messages.empty()) {
      std::cout << "[ PASS ] " << name << "\n";
      continue;
    }

    ++failures;
    for (const auto& message : failure_messages) {
      std::cerr << "[ FAIL ] " << name << ": " << message << "\n";
    }
  }

  std::cout << tests.size() - failures << "/" << tests.size()
            << " tests passed.\n";
  return failures == 0 ? 0 : 1;
}

}  // namespace

// Registers the test suite and returns the runner's exit code.
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
      {"NearestElevatorIsAutoDispatched", NearestElevatorIsAutoDispatched},
      {"DispatchRejectsInvalidPassengerRequests",
       DispatchRejectsInvalidPassengerRequests},
      {"EmptySystemCannotCreateDispatchPlan",
       EmptySystemCannotCreateDispatchPlan},
      {"ManualDispatchUsesRequestedElevator",
       ManualDispatchUsesRequestedElevator},
      {"DispatchUsesQueuedRouteInsteadOfCurrentPositionOnly",
       DispatchUsesQueuedRouteInsteadOfCurrentPositionOnly},
      {"DispatchUsesDirectionAsTieBreaker", DispatchUsesDirectionAsTieBreaker},
      {"RemoteCallReturnsTrackableEvent", RemoteCallReturnsTrackableEvent},
      {"RemoteSendMovesOnlySelectedElevator",
       RemoteSendMovesOnlySelectedElevator},
      {"RemoteCommandsValidateSyntaxAndReportStatus",
       RemoteCommandsValidateSyntaxAndReportStatus},
  });
}
