CXX ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -pedantic -pthread
CPPFLAGS ?= -I src
LDLIBS ?= -lncurses
sdk_root ?= $(shell xcrun --show-sdk-path 2>/dev/null)

ifneq ($(sdk_root),)
CPPFLAGS += -isysroot $(sdk_root) -isystem $(sdk_root)/usr/include/c++/v1
endif


test_sources := tests/elevator_tests.cc src/elevator.cc src/elevator_system.cc

simulator_binary := elevator_simulator
simulator_headers := src/elevator.h src/elevator_system.h src/remote_control_server.h
simulator_sources := src/main.cc src/elevator.cc src/elevator_system.cc src/remote_control_server.cc
test_binary := elevator_tests
test_sources := tests/elevator_tests.cc src/elevator.cc src/elevator_system.cc src/remote_control_server.cc

.PHONY: all run test clean

all: $(simulator_binary)

$(simulator_binary): $(simulator_sources) $(simulator_headers)
	@$(CXX) $(CXXFLAGS) $(CPPFLAGS) -o $@ $(simulator_sources) $(LDLIBS)
	@echo "Built $(simulator_binary)"

run: $(simulator_binary)
	@./$(simulator_binary)

$(test_binary): $(test_sources) $(simulator_headers)
	@$(CXX) $(CXXFLAGS) $(CPPFLAGS) -o $@ $(test_sources)
	@echo "Built $(test_binary)"

test: $(test_binary)
	@./$(test_binary)

clean:
	@rm -f $(simulator_binary) $(test_binary)
	@echo "Cleaned build artifacts"
