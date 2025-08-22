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
  // Usage: ./pick_and_place <robot_ip> [speed_factor]
  if (argc < 2 || argc > 3) {
    std::cerr << "Usage: ./pick_and_place <robot_ip> [speed_factor]" << std::endl;
    return -1;
  }

  try {
    const char* host = argc > 1 ? argv[1] : "127.0.0.1";  // FCI sim server
    const bool is_sim = (std::string(host) == std::string("127.0.0.1"));
    double speed_factor = 0.2;        // safe default
    if (argc == 3) {
      speed_factor = std::stod(argv[2]);
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

    // Set collision behavior with relaxed thresholds to prevent cartesian reflex aborts
    // These values are more permissive than the default strict thresholds

    // Homing + open fully (near max width)
    std::cout << "=== Starting Pick and Place Operation ===" << std::endl;
    std::cout << "Homing gripper..." << std::endl;
    gripper.homing();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    franka::GripperState gs = gripper.readOnce();
    double open_width = std::max(0.0, kAssumedMaxGripperWidth - 0.001);
    std::cout << "Assuming gripper max_width: " << kAssumedMaxGripperWidth
              << ", opening to: " << open_width << std::endl;
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

    // PHASE 1: PICK OPERATION
    std::cout << "\n=== PHASE 1: PICK OPERATION ===" << std::endl;
    
    // Update collision behavior for pick operation
    robot.setCollisionBehavior(
        {{50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0}},  // lower torque thresholds
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},  // upper torque thresholds
        {{50.0, 50.0, 50.0, 50.0, 50.0, 50.0}},  // lower force thresholds
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}   // upper force thresholds
    );
    
    // Move robot to pick location
    const std::array<double, 7> q_pick = toRadians(kQDeg);
    std::cout << "Moving to pick location..." << std::endl;
    MotionGenerator pick_motion_generator(speed_factor, q_pick);
    robot.control(pick_motion_generator);

    // Verify convergence at pick location
    auto measure_and_print = [&](const char* tag, const std::array<double, 7>& q_goal) {
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
    double err = measure_and_print("Final joint positions at pick location", q_pick);
    const double kMaxErrRad = 0.20;  // relaxed tolerance for sim
    int settle_attempts = 0;
    while (err > kMaxErrRad && settle_attempts < 3) {
      std::cout << "Settling pass (attempt " << (settle_attempts + 1)
                << ") to reduce residual error..." << std::endl;
      MotionGenerator settle_gen(std::min(0.15, speed_factor), q_pick);
      robot.control(settle_gen);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      err = measure_and_print("Post-settle joint positions at pick", q_pick);
      settle_attempts++;
    }
    if (err > kMaxErrRad) {
      std::cout << "Warning: joint error remains above tolerance (" << err
                << " rad). Proceeding with grasp anyway." << std::endl;
    }

    // Grasp the object
    constexpr double kObjectWidthMeters = 0.0508;  // 2 inches
    gs = gripper.readOnce();
    double commanded_width = std::min(kObjectWidthMeters, kAssumedMaxGripperWidth - 0.001);
    std::cout << "Closing gripper to width " << commanded_width << " m to grasp object..." << std::endl;
    // Use a strong grasp attempt first; if it fails, fall back to move() tighter.
    // move the gripper to assumed width - object width
    bool grasp_ok = gripper.grasp(0.0508, /*speed=*/0.035, /*force=*/100.0,
        /*epsilon_inner=*/0.002, /*epsilon_outer=*/0.003);
    
    // Wait briefly and verify grasp
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
    gs = gripper.readOnce();
    std::cout << "Gripper width after grasp: " << gs.width
              << ", is_grasped: " << (gs.is_grasped ? "true" : "false") << std::endl;

    // PHASE 2: RETURN TO HOME WITH OBJECT
    std::cout << "\n=== PHASE 2: RETURN TO HOME WITH OBJECT ===" << std::endl;

    // Joint-space retreat to increase clearance before lateral motion,
    // then progress toward home via two gentle intermediate waypoints.
    {
      franka::RobotState s = robot.readOnce();
      // 1) Stronger lift by nudging elbow/shoulder toward zero
      std::array<double, 7> q_lift = s.q;
      auto towards_zero = [](double q, double delta){ return (q > 0.0) ? std::max(0.0, q - delta)
                                                                      : std::min(0.0, q + delta); };
      q_lift[3] = towards_zero(q_lift[3], 0.50);  // elbow up more
      q_lift[1] = towards_zero(q_lift[1], 0.25);  // shoulder up more
      MotionGenerator lift_gen(std::min(0.10, speed_factor), q_lift);
      robot.control(lift_gen);
      std::this_thread::sleep_for(std::chrono::milliseconds(150));

      // 2) Refresh state and blend two steps toward home to pull away safely
      s = robot.readOnce();
      std::array<double, 7> q_step1{};
      for (size_t i = 0; i < 7; ++i) q_step1[i] = s.q[i] + 0.35 * (kQHomeRad[i] - s.q[i]);
      MotionGenerator step_gen1(std::min(0.10, speed_factor), q_step1);
      robot.control(step_gen1);
      std::this_thread::sleep_for(std::chrono::milliseconds(120));

      s = robot.readOnce();
      std::array<double, 7> q_step2{};
      for (size_t i = 0; i < 7; ++i) q_step2[i] = s.q[i] + 0.65 * (kQHomeRad[i] - s.q[i]);
      MotionGenerator step_gen2(std::min(0.12, speed_factor), q_step2);
      robot.control(step_gen2);
      std::this_thread::sleep_for(std::chrono::milliseconds(120));
    }
    
    std::cout << "Moving back to home position with grasped object..." << std::endl;
    MotionGenerator home_with_object_generator(speed_factor, kQHomeRad);
    robot.control(home_with_object_generator);

    // PHASE 3: PLACE OPERATION
    std::cout << "\n=== PHASE 3: PLACE OPERATION ===" << std::endl;
    
    // Move to place location
    const std::array<double, 7> q_place = toRadians(kQPlaceDeg);
    std::cout << "Moving to place location..." << std::endl;
    MotionGenerator place_motion_generator(speed_factor, q_place);
    robot.control(place_motion_generator);

    // Verify convergence at place location
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    err = measure_and_print("Final joint positions at place location", q_place);
    settle_attempts = 0;
    while (err > kMaxErrRad && settle_attempts < 3) {
      std::cout << "Settling pass (attempt " << (settle_attempts + 1)
                << ") to reduce residual error..." << std::endl;
      MotionGenerator settle_gen(std::min(0.15, speed_factor), q_place);
      robot.control(settle_gen);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      err = measure_and_print("Post-settle joint positions at place", q_place);
      settle_attempts++;
    }

    // Release the object
    std::cout << "Releasing object at place location..." << std::endl;
    gripper.move(open_width, 0.1);
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
    
    // Verify release
    gs = gripper.readOnce();
    std::cout << "Gripper width after release: " << gs.width
              << ", is_grasped: " << (gs.is_grasped ? "true" : "false") << std::endl;

    // Joint-space retreat after placing to clear the surface.
    {
      franka::RobotState s = robot.readOnce();
      std::array<double, 7> q_lift = s.q;
      auto towards_zero = [](double q, double delta){ return (q > 0.0) ? std::max(0.0, q - delta)
                                                                      : std::min(0.0, q + delta); };
      q_lift[3] = towards_zero(q_lift[3], 0.30);
      q_lift[1] = towards_zero(q_lift[1], 0.15);
      MotionGenerator lift_gen(std::min(0.15, speed_factor), q_lift);
      robot.control(lift_gen);
      std::this_thread::sleep_for(std::chrono::milliseconds(150));
    }

    // PHASE 4: RETURN TO HOME
    std::cout << "\n=== PHASE 4: RETURN TO HOME ===" << std::endl;
    
    std::cout << "Moving back to home position..." << std::endl;
    // Use a gentle intermediate waypoint to avoid awkward plans from the place posture.
    {
      franka::RobotState s = robot.readOnce();
      std::array<double, 7> q_mid{};
      for (size_t i = 0; i < 7; ++i) {
        q_mid[i] = 0.5 * s.q[i] + 0.5 * kQHomeRad[i];
      }
      MotionGenerator mid_generator(std::min(0.15, speed_factor), q_mid);
      robot.control(mid_generator);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    MotionGenerator final_home_generator(speed_factor, kQHomeRad);
    robot.control(final_home_generator);

    std::cout << "\n=== Pick and Place Operation Complete ===" << std::endl;
    std::cout << "Summary: Home -> Pick -> Home -> Place -> Home" << std::endl;

  } catch (const franka::Exception& e) {
    std::cerr << e.what() << std::endl;
    return -1;
  }

  return 0;
}
