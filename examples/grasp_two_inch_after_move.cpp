#include <chrono>
#include <cmath>
#include <iostream>
#include <numeric>
#include <thread>

#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/robot.h>

#include "examples_common.h"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

// Target joint configuration in degrees (same as original example).
constexpr double kQDeg[7] = {-11.0, 15.0, -9.0, -146.0, 4.0, 157.0, 25.0};

// Home configuration in radians (Panda default example home).
constexpr std::array<double, 7> kQHomeRad = {
    {0.0, -M_PI / 4.0, 0.0, -3.0 * M_PI / 4.0, 0.0, M_PI / 2.0, M_PI / 4.0}};

std::array<double, 7> toRadians(const double q_deg[7]) {
  std::array<double, 7> q_rad{};
  for (size_t i = 0; i < 7; ++i) {
    q_rad[i] = q_deg[i] * kDegToRad;
  }
  return q_rad;
}

}  // namespace

int main(int argc, char** argv) {
  // Usage: ./grasp_two_inch_after_move <robot_ip> [speed_factor]
  if (argc < 2 || argc > 3) {
    std::cerr << "Usage: ./grasp_two_inch_after_move <robot_ip> [speed_factor]" << std::endl;
    return -1;
  }

  try {
    const char* host = argc > 1 ? argv[1] : "127.0.0.1";  // FCI sim server
    double speed_factor = 0.2;        // safe default
    if (argc == 2) {
      speed_factor = std::stod(argv[1]);
      if (speed_factor < 0.0 || speed_factor > 1.0) {
        std::cerr << "[speed_factor] must be in [0.0, 1.0]." << std::endl;
        return -1;
      }
    }

    // Connect to robot and gripper.
    franka::Robot robot(host);
    franka::Gripper gripper(host);

    // Apply standard behavior.
    setDefaultBehavior(robot);

    // Homing + open fully (near max width)
    std::cout << "Homing gripper..." << std::endl;
    gripper.homing();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    franka::GripperState gs = gripper.readOnce();
    double open_width = std::max(0.0, gs.max_width - 0.001);
    std::cout << "Gripper max_width: " << gs.max_width << ", opening to: " << open_width << std::endl;
    gripper.move(open_width, 0.1);

    // Move to home pose if not already there.
    franka::RobotState start_state = robot.readOnce();
    auto max_abs_diff = [](const std::array<double,7>& a, const std::array<double,7>& b){
      double m = 0.0; for (size_t i = 0; i < 7; ++i) m = std::max(m, std::abs(a[i] - b[i])); return m; };
    if (max_abs_diff(start_state.q_d, kQHomeRad) > 1e-3) {
      std::cout << "Moving to home pose..." << std::endl;
      MotionGenerator home_generator(speed_factor, kQHomeRad);
      robot.control(home_generator);
    } else {
      std::cout << "Already at home pose; skipping home move." << std::endl;
    }

    // Move robot in joint position mode to the given target configuration.
    const std::array<double, 7> q_goal = toRadians(kQDeg);
    std::cout << "Moving to target joint configuration..." << std::endl;
    MotionGenerator motion_generator(speed_factor, q_goal);
    robot.control(motion_generator);

    // Verify convergence and optionally perform settling passes.
    auto measure_and_print = [&](const char* tag) {
      franka::RobotState s = robot.readOnce();
      std::array<double, 7> q_meas = s.q;
      double max_abs_err_rad = 0.0;
      double l2_err_rad = 0.0;
      std::cout << tag << " (rad):" << std::endl;
      for (size_t i = 0; i < 7; ++i) {
        double err = q_meas[i] - q_goal[i];
        max_abs_err_rad = std::max(max_abs_err_rad, std::abs(err));
        l2_err_rad += err * err;
        std::cout << "  q" << (i + 1) << " = " << q_meas[i]
                  << "  (goal " << q_goal[i]
                  << ", err " << err << ")" << std::endl;
      }
      l2_err_rad = std::sqrt(l2_err_rad);
      std::cout << "Max |err|: " << max_abs_err_rad << " rad  (" << (max_abs_err_rad * 180.0 / M_PI)
                << " deg)\nL2 err: " << l2_err_rad << " rad" << std::endl;
      return max_abs_err_rad;
    };

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    double err = measure_and_print("Final joint positions after MotionGenerator");
    const double kMaxErrRad = 0.20;  // relaxed tolerance for sim
    int settle_attempts = 0;
    while (err > kMaxErrRad && settle_attempts < 3) {
      std::cout << "Settling pass (attempt " << (settle_attempts + 1)
                << ") to reduce residual error..." << std::endl;
      MotionGenerator settle_gen(std::min(0.15, speed_factor), q_goal);
      robot.control(settle_gen);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      err = measure_and_print("Post-settle joint positions");
      settle_attempts++;
    }
    if (err > kMaxErrRad) {
      std::cout << "Warning: joint error remains above tolerance (" << err
                << " rad). Skipping grasp to avoid misalignment." << std::endl;
      return -1;
    }

    // Grasp a 2 inch (50.8 mm) wide object (sim will just move fingers to that width).
    constexpr double kObjectWidthMeters = 0.0508;  // 2 inches
    gs = gripper.readOnce();
    double commanded_width = kObjectWidthMeters;
    if (gs.max_width <= 0.0) {
      std::cout << "Warning: gripper reports non-positive max width (" << gs.max_width
                << "). Attempting to grasp anyway with requested width." << std::endl;
    } else if (gs.max_width < kObjectWidthMeters) {
      commanded_width = std::max(0.0, gs.max_width - 0.001);
      std::cout << "Requested width (" << kObjectWidthMeters
                << ") exceeds gripper max width (" << gs.max_width
                << "). Clamping to " << commanded_width << "." << std::endl;
    }
    std::cout << "Closing gripper to width " << commanded_width << " m to grasp object..."
              << std::endl;
    bool grasp_success = gripper.grasp(commanded_width, /*speed=*/0.1, /*force=*/60.0);
    if (!grasp_success) {
      std::cout << "Grasp command reported failure." << std::endl;
      // In sim, try move() as a fallback to set width.
      gripper.move(commanded_width, 0.05);
    }

    // Wait briefly and print state.
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
    gs = gripper.readOnce();
    std::cout << "Gripper width after grasp/move: " << gs.width
              << ", is_grasped: " << (gs.is_grasped ? "true" : "false") << std::endl;

    std::cout << "Done: moved to home, then target, then attempted 2-inch grasp." << std::endl;
  } catch (const franka::Exception& e) {
    std::cerr << e.what() << std::endl;
    return -1;
  }

  return 0;
}
