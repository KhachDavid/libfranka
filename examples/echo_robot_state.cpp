// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

// UDP state publisher for live plotting (auto-detects endpoint; see monitoring_tee.h)
#include "monitoring_tee.h"

/**
 * @example echo_robot_state.cpp
 * An example showing how to continuously read the robot state.
 */

int main(int argc, char** argv) {
  if (argc < 2 || argc > 3) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname> [--src=real|sim]" << std::endl;
    return -1;
  }

  try {
    const std::string host = argv[1];
    franka::Robot robot(host);
    // Monitoring publisher (uses ~/.config/franka/mon_endpoint by default)
    std::string src_label = (host == std::string("127.0.0.1")) ? std::string("sim") : std::string("real");
    if (argc == 3) {
      std::string a2 = argv[2];
      if (a2.rfind("--src=", 0) == 0) src_label = a2.substr(6);
    }
    franka_monitor::StatePublisher monitor(src_label);

    size_t count = 0;
    robot.read([&count, &monitor](const franka::RobotState& robot_state) {
      // Printing to std::cout adds a delay. This is acceptable for a read loop such as this, but
      // should not be done in a control loop.
      if (monitor.enabled()) monitor.publish(robot_state);
      std::cout << robot_state << std::endl;
      return count++ < 100;
    });

    std::cout << "Done." << std::endl;
  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
