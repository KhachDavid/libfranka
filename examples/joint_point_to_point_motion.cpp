// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"
// UDP state publisher for live plotting (auto-detects endpoint; see monitoring_tee.h)
#include "monitoring_tee.h"

/**
 * @example joint_point_to_point_motion.cpp
 * An example that moves the robot to a target position by commanding joint positions.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

int main(int argc, char** argv) {
  if (argc != 10 && argc != 11) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname> "
              << "<joint0> <joint1> <joint2> <joint3> <joint4> <joint5> <joint6> "
              << "<speed-factor> [--src=real|sim]" << std::endl
              << "joint0 to joint6 are joint angles in [rad]." << std::endl
              << "speed-factor must be between zero and one." << std::endl;
    return -1;
  }
  try {
    const std::string host = argv[1];
    franka::Robot robot(host);
    setDefaultBehavior(robot);

    // Monitoring publisher (uses ~/.config/franka/mon_endpoint by default)
    std::string src_label = (host == std::string("127.0.0.1")) ? std::string("sim") : std::string("real");
    if (argc == 11) {
      std::string a = argv[10];
      if (a.rfind("--src=", 0) == 0) src_label = a.substr(6);
    }
    franka_monitor::StatePublisher monitor(src_label);

    std::array<double, 7> q_goal;
    for (size_t i = 0; i < 7; i++) {
      q_goal[i] = std::stod(argv[i + 2]);
    }
    double speed_factor = std::stod(argv[9]);

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});

    MotionGenerator motion_generator(speed_factor, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(monitor.tee(motion_generator));
    std::cout << "Motion finished" << std::endl;
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
