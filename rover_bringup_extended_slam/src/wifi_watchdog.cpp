// wifi_watchdog.cpp — Safety monitor that stops the rover on WiFi loss or
//                     ROS communication failure.
//
// C++ port of rover_bringup/wifi_watchdog.py.  Same parameters, same
// behaviour, same guarantees.
//
// WHY THIS EXISTS
// ───────────────
// When the operator's WiFi connection drops the Jetson may keep running an
// active Nav2 goal or hold a stale velocity command.  This node detects the
// loss and immediately floods /cmd_vel with zero-velocity commands so the
// bridge watchdog never sees a stale timeout — it sees active stop commands.
//
// It also monitors whether /cmd_vel is being published at all (heartbeat).  If
// nothing is publishing for longer than the deadman window it triggers the same
// stop flood.
//
// This is Layer 2 protection.  Layer 1 (the most critical) is the ESP32
// firmware's 500 ms serial watchdog, which is independent of the Jetson.
//
// WHAT IT MONITORS
// ────────────────
// 1. WiFi interface operstate  (/sys/class/net/<iface>/operstate)
//    — reads a single file, no subprocess, near-zero CPU cost.
//    — auto-detects the active wireless interface on startup.
// 2. /cmd_vel heartbeat
//    — if nothing published for > deadman_secs the stop flood activates,
//      regardless of WiFi state.  This catches a crashed Nav2 or teleop node.
//
// BEHAVIOUR
// ─────────
// Normal   : silent.  Publishes nothing.
// WiFi lost: publishes Twist(0,0) to /cmd_vel at stop_flood_hz.
// Restored : stops flooding.  Logs one info message.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <dirent.h>
#include <fstream>
#include <memory>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace
{
// Return the name of the first wireless interface under /sys/class/net.
// Wireless interfaces are identified by a 'wireless' directory or a 'phy80211'
// link inside their sysfs directory.  Returns "" if none found.
std::string find_wifi_iface()
{
  const char * net_root = "/sys/class/net";
  DIR * dir = opendir(net_root);
  if (dir == nullptr) {
    return "";
  }
  std::string found;
  struct dirent * ent = nullptr;
  while ((ent = readdir(dir)) != nullptr) {
    const std::string name = ent->d_name;
    if (name == "." || name == "..") {
      continue;
    }
    const std::string base = std::string(net_root) + "/" + name;
    struct stat st;
    if (stat((base + "/wireless").c_str(), &st) == 0 && S_ISDIR(st.st_mode)) {
      found = name;
      break;
    }
    if (lstat((base + "/phy80211").c_str(), &st) == 0 && S_ISLNK(st.st_mode)) {
      found = name;
      break;
    }
  }
  closedir(dir);
  return found;
}

// Read /sys/class/net/<iface>/operstate.
// Safe states: 'up', 'unknown', 'dormant' — many drivers report 'unknown' or
// 'dormant' during normal connected operation.  Only 'down' is treated as a
// genuine link loss.  Returns true (fail-safe) if the file cannot be read.
bool iface_is_up(const std::string & iface)
{
  if (iface.empty()) {
    return true;   // nothing to check — treat as safe
  }
  std::ifstream f("/sys/class/net/" + iface + "/operstate");
  if (!f.is_open()) {
    return true;   // can't read → assume ok, don't false-alarm
  }
  std::string state;
  f >> state;
  return state != "down";
}
}  // namespace

class WifiWatchdog : public rclcpp::Node
{
public:
  WifiWatchdog()
  : Node("wifi_watchdog")
  {
    // ── Parameters ───────────────────────────────────────────────────────────
    check_hz_     = this->declare_parameter<double>("check_hz", 2.0);
    flood_hz_     = this->declare_parameter<double>("stop_flood_hz", 20.0);
    deadman_secs_ = this->declare_parameter<double>("deadman_secs", 5.0);
    iface_        = this->declare_parameter<std::string>("wifi_iface", "");
    // WiFi-loss guard is TELEOP-mode protection (operator commands arrive over
    // WiFi).  During autonomous SLAM/Nav2 everything runs on-board, so a WiFi
    // blip is harmless and stopping for it just wrecks the session.  Default
    // false; launch passes teleop:=true to arm it, or toggle live:
    //   ros2 param set /wifi_watchdog enable_wifi_check true
    wifi_en_      = this->declare_parameter<bool>("enable_wifi_check", false);
    deadman_en_   = this->declare_parameter<bool>("enable_deadman", true);

    if (wifi_en_) {
      ensure_iface();
    }

    // Allow live arming/disarming without a relaunch, e.g. when the user starts
    // a teleop session mid-run.
    param_cb_handle_ = this->add_on_set_parameters_callback(
      std::bind(&WifiWatchdog::on_param_update, this, _1));

    last_cmd_t_ = now_monotonic();

    // ── Publisher / Subscriber ───────────────────────────────────────────────
    stop_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    // /cmd_vel heartbeat listener (does NOT interfere with other publishers)
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&WifiWatchdog::cb_cmd_vel, this, _1));

    // ── Timers ───────────────────────────────────────────────────────────────
    // WiFi / deadman check (low rate)
    check_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(check_hz_, 0.1)),
      std::bind(&WifiWatchdog::check_tick, this));

    // Stop-flood timer (high rate).  Created CANCELLED and only started while a
    // safety condition is actually active: at stop_flood_hz=100 a permanently
    // running timer is 100 executor wake-ups per second, ~all of which do
    // nothing.  The guard is unchanged in strength — flooding still runs at the
    // full configured rate, and the FIRST stop Twist is published the instant
    // the condition is detected rather than up to one flood period later.
    flood_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(flood_hz_, 1.0)),
      std::bind(&WifiWatchdog::flood_tick, this));
    flood_timer_->cancel();

    RCLCPP_INFO(
      this->get_logger(),
      "[WifiWatchdog] started | iface=%s wifi=%s deadman=%s(%.1fs)",
      iface_.empty() ? "none" : iface_.c_str(),
      wifi_en_ ? "True" : "False",
      deadman_en_ ? "True" : "False", deadman_secs_);
  }

private:
  static double now_monotonic()
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  void ensure_iface()
  {
    if (!iface_.empty()) {
      return;
    }
    iface_ = find_wifi_iface();
    if (!iface_.empty()) {
      RCLCPP_INFO(
        this->get_logger(),
        "[WifiWatchdog] auto-detected WiFi interface: %s", iface_.c_str());
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "[WifiWatchdog] no wireless interface found — wifi check disabled");
      wifi_en_ = false;
    }
  }

  // Live toggling of the guards (teleop starts/ends) without relaunch.  The
  // flood state self-clears on the next check_tick once a guard's reason
  // disappears, so no manual cleanup is needed here.
  rcl_interfaces::msg::SetParametersResult on_param_update(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & p : params) {
      if (p.get_name() == "enable_wifi_check") {
        wifi_en_ = p.as_bool();
        if (wifi_en_) {
          ensure_iface();
          wifi_was_up_ = true;   // fresh start, no stale alarm
          RCLCPP_INFO(this->get_logger(), "[WifiWatchdog] WiFi guard ARMED (teleop mode)");
        } else {
          RCLCPP_INFO(this->get_logger(), "[WifiWatchdog] WiFi guard DISARMED (autonomy mode)");
        }
      } else if (p.get_name() == "enable_deadman") {
        deadman_en_ = p.as_bool();
        last_cmd_t_ = now_monotonic();   // fresh grace window
        RCLCPP_INFO(
          this->get_logger(), "[WifiWatchdog] deadman %s",
          deadman_en_ ? "ENABLED" : "DISABLED");
      }
    }
    return result;
  }

  // Track the last time an EXTERNAL source published on /cmd_vel.
  //   1. Not flooding — accept every message, arm deadman on first receipt.
  //   2. Flooding, non-zero message — must be an external source (our flood is
  //      always Twist(0,0)).  Accept it so the deadman can clear when the user
  //      resumes teleop.
  //   3. Flooding, zero message — our own flood output, ignore it.
  void cb_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (!flooding_) {
      last_cmd_t_ = now_monotonic();
      cmd_ever_received_ = true;
    } else if (std::fabs(msg->linear.x) > 0.001 || std::fabs(msg->angular.z) > 0.001) {
      last_cmd_t_ = now_monotonic();
      cmd_ever_received_ = true;
    }
  }

  void check_tick()
  {
    std::vector<std::string> reasons;

    // ── WiFi check ─────────────────────────────────────────────────────────
    if (wifi_en_) {
      const bool wifi_up = iface_is_up(iface_);
      if (!wifi_up && wifi_was_up_) {
        RCLCPP_WARN(
          this->get_logger(),
          "[WifiWatchdog] !! WiFi LOST on %s — sending continuous stop commands",
          iface_.c_str());
      } else if (wifi_up && !wifi_was_up_) {
        RCLCPP_INFO(
          this->get_logger(), "[WifiWatchdog] WiFi restored on %s", iface_.c_str());
      }
      wifi_was_up_ = wifi_up;
      if (!wifi_up) {
        reasons.push_back("wifi_lost");
      }
    }

    // ── /cmd_vel deadman check ─────────────────────────────────────────────
    if (deadman_en_ && cmd_ever_received_) {
      const double silence = now_monotonic() - last_cmd_t_;
      if (silence > deadman_secs_) {
        // Only warn on the first detection in each silence window.
        if (!flooding_ || flood_reason_ != "deadman") {
          RCLCPP_WARN(
            this->get_logger(),
            "[WifiWatchdog] !! /cmd_vel silent for %.1fs — sending continuous stop commands",
            silence);
        }
        reasons.push_back("deadman");
      }
    }

    // ── Update flood state ─────────────────────────────────────────────────
    if (!reasons.empty()) {
      if (!flooding_) {
        // Enter flooding: publish the first stop NOW (do not wait for the
        // timer's first tick), then start the high-rate flood.
        stop_pub_->publish(geometry_msgs::msg::Twist());
        flood_timer_->reset();
      }
      flooding_ = true;
      flood_reason_ = reasons.front();
    } else {
      if (flooding_) {
        flood_timer_->cancel();
        RCLCPP_INFO(this->get_logger(), "[WifiWatchdog] conditions cleared — stop flood ended");
        // Reset the deadman clock so the next check gets a fresh grace window.
        // Without this the deadman would immediately re-trigger on the very
        // next tick because last_cmd_t_ is still the old timestamp.
        last_cmd_t_ = now_monotonic();
      }
      flooding_ = false;
      flood_reason_.clear();
    }
  }

  // Publish Twist(0,0) at high rate while a safety condition is active.
  void flood_tick()
  {
    if (flooding_) {
      stop_pub_->publish(geometry_msgs::msg::Twist());   // all fields zero
    }
  }

  // Parameters
  double check_hz_{2.0};
  double flood_hz_{20.0};
  double deadman_secs_{5.0};
  std::string iface_;
  bool wifi_en_{false};
  bool deadman_en_{true};

  // State
  bool wifi_was_up_{true};          // previous WiFi state
  bool flooding_{false};            // are we currently sending stop commands?
  std::string flood_reason_;
  double last_cmd_t_{0.0};
  // Deadman does not arm until at least one real /cmd_vel has arrived.  This
  // prevents false triggers on startup before the operator connects a joystick
  // or teleop node.  The WiFi check still runs from the start.
  bool cmd_ever_received_{false};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr stop_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr check_timer_;
  rclcpp::TimerBase::SharedPtr flood_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WifiWatchdog>());
  rclcpp::shutdown();
  return 0;
}
