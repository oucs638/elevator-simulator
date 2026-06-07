CXX ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -pedantic -pthread
CPPFLAGS ?= -I src
LDLIBS ?= -lncurses
sdk_root ?= $(shell xcrun --show-sdk-path 2>/dev/null)

ifneq ($(sdk_root),)
CPPFLAGS += -isysroot $(sdk_root) -isystem $(sdk_root)/usr/include/c++/v1
endif

simulator_binary := elevator_simulator
simulator_headers := src/elevator.h src/elevator_system.h src/remote_control_server.h
simulator_sources := src/main.cc src/elevator.cc src/elevator_system.cc src/remote_control_server.cc
test_binary := elevator_tests
test_sources := tests/elevator_tests.cc src/elevator.cc src/elevator_system.cc src/remote_control_server.cc
remote_client_binary := elevator_client
remote_client_sources := src/remote_client.cc

.PHONY: all run remote test clean

all: $(simulator_binary) $(remote_client_binary)

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

$(remote_client_binary): $(remote_client_sources)
	@$(CXX) $(CXXFLAGS) $(CPPFLAGS) -o $@ $(remote_client_sources) $(LDLIBS)
	@echo "Built $(remote_client_binary)"

remote: $(remote_client_binary)
	@./$(remote_client_binary) 127.0.0.1 5050

clean:
	@rm -f $(simulator_binary) $(test_binary) $(remote_client_binary)
	@echo "Cleaned build artifacts"
