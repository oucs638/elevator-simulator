# Elevator Simulator

A concurrent elevator-bank simulator written in C++17. It provides an ncurses
main interface and a remote-control dashboard connected through TCP. Four
independent elevator workers support automatic route-aware dispatching, manual
dispatching, simultaneous movement, and live status updates.

## Installation Instructions

The required compiler, build tools, libraries, and runtime capabilities are
listed in [`requirement.txt`](requirement.txt). These are system-level C++
requirements, not Python packages, so `requirement.txt` is provided as a
dependency reference and should not be installed with `pip`.

On macOS, install the Apple Command Line Tools:

```sh
xcode-select --install
```

On Ubuntu or Debian, install the compiler, Make, and ncurses development
library:

```sh
sudo apt update
sudo apt install build-essential libncurses-dev
```

Build the simulator and remote-control dashboard:

```sh
make
```

## Usage

Start the main simulator:

```sh
make run
```

Enter one of the following commands in the main simulator:

```text
<from> <to>                 Automatically select an elevator
<elevator> <from> <to>      Manually select an elevator
quit                        Close the simulator
```

The main simulator automatically starts the remote-control server on TCP port
`5050`. While it is running, open another terminal and start the remote
dashboard:

```sh
make remote
```

Enter one of the following commands in the remote dashboard:

```text
call <from> <to>            Create an automatically dispatched passenger call
send <elevator> <floor>     Send a specific elevator directly to a floor
quit                        Close the remote dashboard
```

Run the unit tests:

```sh
make test
```

Use a terminal of at least `82 x 27` for the main simulator and `82 x 20` for
the remote dashboard.

