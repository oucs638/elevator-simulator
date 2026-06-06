CXX ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -pedantic -pthread
CPPFLAGS ?= -I src
LDLIBS ?= -lncurses

simulator_binary := elevator_simulator
simulator_headers := src/elevator.h
simulator_sources := src/main.cc src/elevator.cc

.PHONY: all run clean

all: $(simulator_binary): $(simulator_sources) $(simulator_headers)

$(simulator_binary): $(simulator_sources)
  @$(CXX) $(CXXFLAGS) $(CPPFLAGS) -o $@ $(simulator_sources) $(LDLIBS)
  @echo "Built $(simulator_binary)"

run: $(simulator_binary)
  @./$(simulator_binar)

clean:
  @rm -f $(simulator_binary)
