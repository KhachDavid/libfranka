#include <chrono>
#include <cmath>
#include <iostream>
#include <numeric>
#include <thread>

#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/robot.h>

#include "examples_common.h"
#include "monitoring_tee.h"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

// Target joint configuration in degrees (same as original example).
constexpr double kQDeg[7] = {-11.0, 15.0, -9.0, -146.0, 4.0, 157.0, 25.0};

// Home configuration in radians (Panda default example home).
constexpr std::array<double, 7> kQHomeRad = {
    {0.0, -M_PI / 4.0, 0.0, -3.0 * M_PI / 4.0, 0.0, M_PI / 2.0, M_PI / 4.0}};

// Place location joint configuration (from current robot state)
// This represents the destination where we want to place the object
// Current joint angles: [0.181531, 0.291347, 0.118949, -2.57022, -0.182026, 2.942, 1.2214] radians
// Converted to degrees: [10.4, 16.7, 6.8, -147.3, -10.4, 168.6, 70.0]
constexpr double kQPlaceDeg[7] = {10.4, 16.7, 6.8, -147.3, -10.4, 168.6, 70.0};

// Assume a 4-inch maximum gripper opening for this example regardless of reported caps.
constexpr double kAssumedMaxGripperWidth = 0.1016;  // 4 inches in meters

std::array<double, 7> toRadians(const double q_deg[7]) {
  std::array<double, 7> q_rad{};
  for (size_t i = 0; i < 7; ++i) {
    q_rad[i] = q_deg[i] * kDegToRad;
  }
  return q_rad;
}

}  // namespace

int main(int argc, char** argv) {
  // Usage: ./pick_and_place <robot_ip> [speed_factor] [--src=real|sim]
  if (argc < 2 || argc > 4) {
    std::cerr << "Usage: ./pick_and_place <robot_ip> [speed_factor] [--src=real|sim]" << std::endl;
    return -1;
  }

  try {
    const char* host = argc > 1 ? argv[1] : "127.0.0.1";  // FCI sim server
    const bool is_sim = (std::string(host) == std::string("127.0.0.1"));
    double speed_factor = 0.2;        // safe default
    std::string src_label = is_sim ? std::string("sim") : std::string("real");
    if (argc >= 3) {
      // argv[2] may be speed or --src
      std::string a2 = argv[2];
      if (a2.rfind("--src=", 0) == 0) {
        src_label = a2.substr(6);
      } else {
        speed_factor = std::stod(a2);
      }
    }
    if (argc == 4) {
      std::string a3 = argv[3];
      if (a3.rfind("--src=", 0) == 0) src_label = a3.substr(6);
    }
    if (speed_factor < 0.0 || speed_factor > 1.0) {
      std::cerr << "[speed_factor] must be in [0.0, 1.0]." << std::endl;
      return -1;
    }

    // Connect to robot and gripper.
    franka::Robot robot(host);
    franka::Gripper gripper(host);

    // Apply standard behavior.
    setDefaultBehavior(robot);

    // Monitoring publisher (uses ~/.config/franka/mon_endpoint by default)
    franka_monitor::StatePublisher monitor(src_label);

    // Homing + open fully (near max width)
    std::cout << "=== Starting Pick and Place Operation ===" << std::endl;
    std::cout << "Homing gripper..." << std::endl;
    gripper.homing();
    franka::GripperState gs = gripper.readOnce();
    double open_width = std::max(0.0, gs.max_width - 0.001);
    std::cout << "Reported gripper max_width: " << gs.max_width
              << ", opening to: " << open_width << std::endl;
    gripper.move(open_width, 0.1);

    // Move to home pose if not already there.
    franka::RobotState start_state = robot.readOnce();
    auto max_abs_diff = [](const std::array<double,7>& a, const std::array<double,7>& b){
      double m = 0.0; for (size_t i = 0; i < 7; ++i) m = std::max(m, std::abs(a[i] - b[i])); return m; };
    if (max_abs_diff(start_state.q_d, kQHomeRad) > 1e-3) {
      std::cout << "Moving to home pose..." << std::endl;
      MotionGenerator home_generator(speed_factor, kQHomeRad);
      robot.control(monitor.tee(home_generator));
    } else {
      std::cout << "Already at home pose; skipping home move." << std::endl;
    }

    // Update collision behavior for pick operation
    robot.setCollisionBehavior(
        {{50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0}},  // lower torque thresholds
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},  // upper torque thresholds
        {{50.0, 50.0, 50.0, 50.0, 50.0, 50.0}},  // lower force thresholds
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}   // upper force thresholds
    );
    
    // Move robot to pick location
    const std::array<double, 7> q_pick = toRadians(kQDeg);
    std::cout << "Going to pick..." << std::endl;
    MotionGenerator pick_motion_generator(speed_factor, q_pick);
    robot.control(monitor.tee(pick_motion_generator));


    // Grasp
    constexpr double kObjectWidthMeters = 0.0508;  // 2 inches
    gs = gripper.readOnce();

    // Ensure we start slightly wider than the object before grasping
    double pre_open_width = std::min(gs.max_width - 0.002, kObjectWidthMeters + 0.02);
    if (gs.width < pre_open_width - 1e-4) {
      gripper.move(pre_open_width, 0.1);
    }
    std::cout << "Grasping..." << std::endl;
    bool grasp_ok = gripper.grasp(kObjectWidthMeters, /*speed=*/0.035, /*force=*/100.0,
        /*epsilon_inner=*/0.004, /*epsilon_outer=*/0.010);
    
    // Return home with object
    std::cout << "Going home with object..." << std::endl;
    MotionGenerator home_with_object_generator(speed_factor, kQHomeRad);
    robot.control(monitor.tee(home_with_object_generator));

    // Go to place
    const std::array<double, 7> q_place = toRadians(kQPlaceDeg);
    std::cout << "Going to place..." << std::endl;
    MotionGenerator place_motion_generator(speed_factor, q_place);
    robot.control(monitor.tee(place_motion_generator));

    // Release
    std::cout << "Releasing..." << std::endl;
    gripper.move(open_width, 0.1);
    
    // Return home
    std::cout << "Going home..." << std::endl;
    MotionGenerator final_home_generator(speed_factor, kQHomeRad);
    robot.control(monitor.tee(final_home_generator));

    std::cout << "Done." << std::endl;

  } catch (const franka::Exception& e) {
    std::cerr << e.what() << std::endl;
    return -1;
  }

  return 0;
}
