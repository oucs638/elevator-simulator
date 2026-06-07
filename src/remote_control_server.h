// Defines the TCP server that exposes live elevator monitoring and control.

#ifndef ELEVATOR_SIMULATOR_SRC_REMOTE_CONTROL_SERVER_H_
#define ELEVATOR_SIMULATOR_SRC_REMOTE_CONTROL_SERVER_H_

#include <atomic>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "elevator_system.h"

namespace elevator_simulator {

// Serves remote commands against the same ElevatorSystem used by the main UI.
class RemoteControlServer {
 public:
  explicit RemoteControlServer(ElevatorSystem& system);
  ~RemoteControlServer();

  RemoteControlServer(const RemoteControlServer&) = delete;
  RemoteControlServer& operator=(const RemoteControlServer&) = delete;

  // Starts listening on port and writes a failure description to error.
  bool Start(int port, std::string* error);

  // Stops accepting clients and joins all server-owned threads.
  void Stop();

  // Returns whether the server is currently accepting clients.
  bool IsRunning() const;

  // Returns the active listening port, or zero when stopped.
  int Port() const;

  // Parses and executes one command without requiring a network connection.
  static std::string ExecuteCommand(ElevatorSystem* system,
                                    const std::string& raw_command);

 private:
  ElevatorSystem& system_;
  std::atomic<bool> running_;
  int server_fd_;
  int port_;
  std::thread accept_thread_;
  std::mutex clients_mutex_;
  std::set<int> client_fds_;
  std::vector<std::thread> client_threads_;

  // Accepts clients and assigns each connection to its own thread.
  void AcceptLoop();

  // Reads newline-delimited commands from one connected client.
  void HandleClient(int client_fd);
};

}  // namespace elevator_simulator

#endif  // ELEVATOR_SIMULATOR_SRC_REMOTE_CONTROL_SERVER_H_
