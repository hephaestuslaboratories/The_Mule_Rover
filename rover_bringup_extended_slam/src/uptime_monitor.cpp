// uptime_monitor.cpp — Jetson boot history + on-time, for the cockpit.
//
// Answers "what has this machine been doing behind my back?":
//   * publishes /rover/uptime (std_msgs/String, JSON, 1 Hz):
//       jetson_boot / jetson_uptime_s / stack_uptime_s / last_off / boots
//   * appends one line per NEW Jetson boot to boot_history.log — an entry the
//     operator didn't cause is evidence of an unwanted POR / watchdog reset.
//   * maintains a heartbeat file (epoch, rewritten every 30 s while the stack
//     runs).  At the next boot that stale heartbeat is the best available
//     estimate of when the rover was last alive -> reported as "last_off".
//
// Boot time comes from /proc/stat's `btime` (epoch seconds the kernel booted)
// — true boot moment regardless of when this node starts.  Note the heartbeat
// only advances while the stack is up: if the stack is stopped but the Jetson
// stays on, "last_off" reads as the stack-stop time, not power-off.  Good
// enough for "a sense of what is going on"; documented so nobody debugs it.

#include <cstdio>
#include <ctime>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace
{
long long proc_btime()
{
  std::ifstream f("/proc/stat");
  std::string word;
  while (f >> word) {
    if (word == "btime") {
      long long v = 0;
      f >> v;
      return v;
    }
  }
  return 0;
}

std::string iso_local(long long epoch)
{
  if (epoch <= 0) {
    return "";
  }
  const time_t t = static_cast<time_t>(epoch);
  struct tm tmv;
  localtime_r(&t, &tmv);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tmv);
  return buf;
}
}  // namespace

class UptimeMonitor : public rclcpp::Node
{
public:
  UptimeMonitor()
  : Node("uptime_monitor")
  {
    history_file_ = this->declare_parameter<std::string>(
      "history_file", "/workspaces/isaac_ros-dev/boot_history.log");
    heartbeat_file_ = this->declare_parameter<std::string>(
      "heartbeat_file", "/workspaces/isaac_ros-dev/rover_uptime_heartbeat");

    btime_ = proc_btime();
    stack_start_ = this->get_clock()->now().seconds();

    // Previous heartbeat BEFORE we overwrite it.  Older than this boot ->
    // it is the last sign of life of the previous session.
    const long long prev_hb = read_epoch(heartbeat_file_);
    if (prev_hb > 0 && prev_hb < btime_) {
      last_off_ = prev_hb;
    }

    scan_history();
    if (!boot_recorded_) {
      std::ofstream f(history_file_, std::ios::app);
      if (f) {
        f << "BOOT " << iso_local(btime_) << " epoch=" << btime_
          << " prev_heartbeat_epoch=" << (prev_hb > 0 ? prev_hb : 0) << "\n";
        ++boots_;
      }
    }

    pub_ = this->create_publisher<std_msgs::msg::String>("/rover/uptime", 10);
    timer_ = this->create_wall_timer(1s, [this]() {this->tick();});

    RCLCPP_INFO(this->get_logger(),
      "[uptime_monitor] boot %s · boots recorded %d · last off %s",
      iso_local(btime_).c_str(), boots_,
      last_off_ > 0 ? iso_local(last_off_).c_str() : "(unknown)");
  }

private:
  static long long read_epoch(const std::string & path)
  {
    std::ifstream f(path);
    long long v = 0;
    f >> v;
    return v;
  }

  // Count BOOT lines; note whether the current btime is already recorded and,
  // if this is a node restart within the same boot, recover last_off from the
  // existing line so it survives stack restarts.
  void scan_history()
  {
    std::ifstream f(history_file_);
    std::string line;
    while (std::getline(f, line)) {
      if (line.rfind("BOOT ", 0) != 0) {
        continue;
      }
      ++boots_;
      const std::string ep_key = "epoch=";
      const size_t k = line.find(ep_key);
      if (k == std::string::npos) {
        continue;
      }
      const long long ep = std::strtoll(line.c_str() + k + ep_key.size(), nullptr, 10);
      if (ep == btime_) {
        boot_recorded_ = true;
        if (last_off_ == 0) {
          const std::string hb_key = "prev_heartbeat_epoch=";
          const size_t h = line.find(hb_key);
          if (h != std::string::npos) {
            last_off_ = std::strtoll(line.c_str() + h + hb_key.size(), nullptr, 10);
          }
        }
      }
    }
  }

  void tick()
  {
    const double now = this->get_clock()->now().seconds();
    std_msgs::msg::String msg;
    std::ostringstream j;
    j << "{\"jetson_boot\": \"" << iso_local(btime_) << "\""
      << ", \"jetson_uptime_s\": " << static_cast<long long>(now - btime_)
      << ", \"stack_uptime_s\": " << static_cast<long long>(now - stack_start_)
      << ", \"last_off\": \"" << iso_local(last_off_) << "\""
      << ", \"boots\": " << boots_ << "}";
    msg.data = j.str();
    pub_->publish(msg);

    if (++ticks_ % 30 == 0) {
      std::ofstream f(heartbeat_file_, std::ios::trunc);
      if (f) {
        f << static_cast<long long>(now) << "\n";
      }
    }
  }

  std::string history_file_;
  std::string heartbeat_file_;
  long long btime_{0};
  long long last_off_{0};
  double stack_start_{0.0};
  int boots_{0};
  bool boot_recorded_{false};
  int ticks_{0};

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UptimeMonitor>());
  rclcpp::shutdown();
  return 0;
}
