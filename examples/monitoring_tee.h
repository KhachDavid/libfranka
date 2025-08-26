// Header-only helper to publish RobotState over UDP without affecting control timing.
#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <fstream>
#include <iomanip>
#include <chrono>

#include <franka/robot_state.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace franka_monitor {

class StatePublisher {
 public:
  explicit StatePublisher(std::string src_label = std::string()) : src_(std::move(src_label)) {
    // Resolution order:
    // 1) FRANKA_MON_ENDPOINT env (e.g., udp://127.0.0.1:5601)
    // 2) ./franka_mon_endpoint (text file with endpoint)
    // 3) ~/.config/franka/mon_endpoint
    // 4) compile-time default FRANKA_MON_DEFAULT_ENDPOINT (if defined)
    std::string endpoint;
    if (const char* env = std::getenv("FRANKA_MON_ENDPOINT")) {
      endpoint = env;
    } else {
      std::ifstream f1("franka_mon_endpoint");
      if (f1.good()) {
        std::getline(f1, endpoint);
      }
      if (endpoint.empty()) {
        const char* home = std::getenv("HOME");
        if (home) {
          std::string p = std::string(home) + "/.config/franka/mon_endpoint";
          std::ifstream f2(p);
          if (f2.good()) std::getline(f2, endpoint);
        }
      }
#ifdef FRANKA_MON_DEFAULT_ENDPOINT
      if (endpoint.empty()) endpoint = FRANKA_MON_DEFAULT_ENDPOINT;
#endif
    }
    if (endpoint.empty()) return;
    // Expect format: udp://host:port
    const std::string prefix = "udp://";
    if (endpoint.rfind(prefix, 0) != 0) return;
    std::string hp = endpoint.substr(prefix.size());
    auto pos = hp.find(':');
    if (pos == std::string::npos) return;
    std::string host = hp.substr(0, pos);
    int port = std::atoi(hp.substr(pos + 1).c_str());
    if (port <= 0) return;

    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) return;
    std::memset(&dst_, 0, sizeof(dst_));
    dst_.sin_family = AF_INET;
    dst_.sin_port = htons(static_cast<uint16_t>(port));
    if (::inet_pton(AF_INET, host.c_str(), &dst_.sin_addr) != 1) {
      ::close(fd_); fd_ = -1; return;
    }
    t0_ = std::chrono::steady_clock::now();
  }

  ~StatePublisher() {
    if (fd_ >= 0) ::close(fd_);
  }

  bool enabled() const { return fd_ >= 0; }

  void publish(const franka::RobotState& s) {
    if (fd_ < 0) return;
    std::ostringstream os;
    os.setf(std::ios::fixed, std::ios::floatfield);
    os << std::setprecision(6);
    os << "{\"t\":" << s.time.toSec();
    if (!src_.empty()) {
      os << ",\"src\":\"" << src_ << "\"";
    }
    // Monotonic wall-time since process start and discrete step index
    double w = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0_).count();
    // Increment step only when joints actually move (ignore gripper-only motion)
    bool moved = false;
    if (!last_q_valid_) {
      last_q_ = s.q;
      last_q_valid_ = true;
    } else {
      double max_abs = 0.0;
      for (int i = 0; i < 7; ++i) {
        double d = std::abs(s.q[i] - last_q_[i]);
        if (d > max_abs) max_abs = d;
      }
      // Threshold ~1e-4 rad (~0.006 deg) filters encoder jitter
      if (max_abs > 1e-4) {
        moved = true;
        last_q_ = s.q;
        ++step_;
      }
    }
    os << ",\"w\":" << w << ",\"k\":" << step_ << ",\"m\":" << (moved ? 1 : 0);
    os << ",\"q\":[";
    for (int i = 0; i < 7; ++i) { if (i) os << ","; os << s.q[i]; }
    os << "],\"dq\":[";
    for (int i = 0; i < 7; ++i) { if (i) os << ","; os << s.dq[i]; }
    os << "],\"tau\":[";
    for (int i = 0; i < 7; ++i) { if (i) os << ","; os << s.tau_J[i]; }
    os << "]}";
    const std::string payload = os.str();
    ::sendto(fd_, payload.data(), payload.size(), 0, reinterpret_cast<const sockaddr*>(&dst_), sizeof(dst_));
  }

  template <typename Callback>
  auto tee(Callback user_callback) {
    return [this, user_callback](const franka::RobotState& s, franka::Duration d) mutable {
      if (enabled()) publish(s);
      return user_callback(s, d);
    };
  }

 private:
  int fd_{-1};
  sockaddr_in dst_{};
  std::string src_{};
  std::chrono::steady_clock::time_point t0_{};
  uint64_t step_{0};
  std::array<double, 7> last_q_{{0,0,0,0,0,0,0}};
  bool last_q_valid_{false};
};

}  // namespace franka_monitor


