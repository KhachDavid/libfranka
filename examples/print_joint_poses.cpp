// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <iostream>
#include <iterator>

#include <franka/exception.h>
#include <franka/model.h>

// UDP state publisher for live plotting (auto-detects endpoint; see monitoring_tee.h)
#include "monitoring_tee.h"

/**
 * @example print_joint_poses.cpp
 * An example showing how to use the model library that prints the transformation
 * matrix of each joint with respect to the base frame.
 */

template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}

int main(int argc, char** argv) {
  if (argc < 2 || argc > 3) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname> [--src=real|sim]" << std::endl;
    return -1;
  }

  try {
    const std::string host = argv[1];
    franka::Robot robot(host);
    std::string src_label = (host == std::string("127.0.0.1")) ? std::string("sim") : std::string("real");
    if (argc == 3) {
      std::string a2 = argv[2];
      if (a2.rfind("--src=", 0) == 0) src_label = a2.substr(6);
    }
    franka_monitor::StatePublisher monitor(src_label);
    franka::RobotState state = robot.readOnce();
    if (monitor.enabled()) monitor.publish(state);
    franka::Model model(robot.loadModel());
    for (franka::Frame frame = franka::Frame::kJoint1; frame <= franka::Frame::kEndEffector;
         frame++) {
      std::cout << model.pose(frame, state) << std::endl;
    }
  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
