// Unit test for basic elevator movement and snapshots.

#include <functional>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "../src/elevator.h"

namespace {

using elevator_simulator::Elevator;
using elevator_simulator::ElevatorStage;

// Records a descriptive failure while allowing the remaining tests to run.
bool Expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << message << "\n";
  }
  return condition;
}

void ConstructorClampsStartingFloor() {
  Elevator below_minimum(1, -5);
  Elevator above_maximum(2, 50);

  Expect(below_minimum.Snapshot().current_floor == Elevator::kMinFloor,
         "Starting floor below range must clamp to kMinFloor.");
  Expect(above_maximum.Snapshot().current_floor == Elevator::kMaxFloor,
         "Starting floor above range must clamp to kMaxFloor.");
}

void DirectMoveArrivesAtDestination() {
  Elevator elevator(1, 1);

  elevator.move(3, 8);
  const auto snapshot = elevator.Snapshot();

  Expect(snapshot.current_floor == 8,
         "Elevator must end at the destination floor.");
  Expect(!snapshot.busy, "Elevator must not be busy after move.");
  Expect(snapshot.stage == ElevatorStage::kIdle,
         "Elevator must return to idle after move.");
  Expect(!snapshot.active_request.has_value(),
         "Elevator must clear active request after move.");
}

using TestFunction = std::function<void()>;

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
  });
}
