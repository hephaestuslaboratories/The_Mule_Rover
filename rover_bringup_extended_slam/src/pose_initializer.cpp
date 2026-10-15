// pose_initializer.cpp — auto-SET-POSE from the persisted last-known pose.
//
// Counterpart to pose_tracker: at nav startup, read last_pose.json and, IF the
// saved map_name matches the map being loaded now, publish the saved pose once
// on /initialpose — the exact message RViz's "2D Pose Estimate" and the
// cockpit's SET POSE publish, so slam_toolbox re-seeds identically.
//
// Safety properties, in order of importance:
//   * Map mismatch -> do nothing.  Coordinates are only meaningful in the map
//     frame they were saved in.
//   * No file / unparseable file -> do nothing.  Manual SET POSE still works.
//   * The seed waits until something subscribes to /initialpose (slam_toolbox
//     up with the posegraph deserialized) plus settle_delay_s, so the message
//     is not published into the void during stack startup.
//   * A later manual SET POSE always overrides this — we publish once, early,
//     and exit.
//
// The rover must physically BE near the saved pose (it usually is — it powered
// down there).  If it was carried elsewhere while off, the operator overrides
// with SET POSE exactly as before this node existed.

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace
{
// Pull `"key": <number>` out of a one-line JSON we wrote ourselves
// (pose_tracker's persist()).  Not a general JSON parser and doesn't need to
// be — the format is fixed.
bool json_number(const std::string & s, const std::string & key, double & out)
{
  const std::string needle = "\"" + key + "\":";
  const size_t k = s.find(needle);
  if (k == std::string::npos) {
    return false;
  }
  out = std::strtod(s.c_str() + k + needle.size(), nullptr);
  return true;
}

bool json_string(const std::string & s, const std::string & key, std::string & out)
{
  const std::string needle = "\"" + key + "\": \"";
  const size_t k = s.find(needle);
  if (k == std::string::npos) {
    return false;
  }
  const size_t start = k + needle.size();
  const size_t end = s.find('"', start);
  if (end == std::string::npos) {
    return false;
  }
  out = s.substr(start, end - start);
  return true;
}
}  // namespace

class PoseInitializer : public rclcpp::Node
{
public:
  PoseInitializer()
  : Node("pose_initializer")
  {
    save_path_ = this->declare_parameter<std::string>(
      "save_path", "/workspaces/isaac_ros-dev/last_pose.json");
    std::string map_id = this->declare_parameter<std::string>("map_id", "");
    settle_delay_s_ = this->declare_parameter<double>("settle_delay_s", 2.0);
    // 0 = accept any age.  Non-zero: a save older than this is ignored —
    // guards against seeding from a stale file after the rover was moved.
    max_age_s_ = this->declare_parameter<double>("max_age_s", 0.0);

    const size_t slash = map_id.find_last_of('/');
    map_name_ = (slash == std::string::npos) ? map_id : map_id.substr(slash + 1);

    // Latched status for the cockpit: a late subscriber (rover_web connecting
    // after the seed fired) still receives the final state.
    status_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/rover/pose_autoinit", rclcpp::QoS(1).transient_local());

    if (!load()) {
      RCLCPP_INFO(this->get_logger(),
        "[pose_initializer] no usable saved pose — waiting for manual SET POSE");
      armed_ = false;
      publish_status();      // status_ was set by load() to the reason
      return;
    }

    status_ = "WAITING map='" + map_name_ + "'";
    publish_status();

    pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10);
    // Poll until the localizer subscribes, then settle, publish once, done.
    timer_ = this->create_wall_timer(500ms, [this]() {this->tick();});
    RCLCPP_INFO(this->get_logger(),
      "[pose_initializer] armed: map='%s' x=%.3f y=%.3f — waiting for localizer",
      map_name_.c_str(), x_, y_);
  }

private:
  void publish_status()
  {
    std_msgs::msg::String msg;
    msg.data = status_;
    status_pub_->publish(msg);
  }

  bool load()
  {
    std::ifstream f(save_path_);
    if (!f) {
      status_ = "NO_SAVED_POSE";
      return false;
    }
    std::stringstream ss;
    ss << f.rdbuf();
    const std::string j = ss.str();

    std::string saved_map;
    double stamp = 0.0;
    if (!json_string(j, "map_name", saved_map) ||
      !json_number(j, "x", x_) || !json_number(j, "y", y_) ||
      !json_number(j, "qz", qz_) || !json_number(j, "qw", qw_))
    {
      RCLCPP_WARN(this->get_logger(),
        "[pose_initializer] %s exists but is unparseable — ignoring",
        save_path_.c_str());
      status_ = "UNPARSEABLE";
      return false;
    }
    if (saved_map != map_name_) {
      RCLCPP_INFO(this->get_logger(),
        "[pose_initializer] saved pose is for map '%s', loading '%s' — ignoring",
        saved_map.c_str(), map_name_.c_str());
      status_ = "MAP_MISMATCH saved='" + saved_map + "' loading='" + map_name_ + "'";
      return false;
    }
    if (max_age_s_ > 0.0 && json_number(j, "stamp", stamp)) {
      const double age = this->get_clock()->now().seconds() - stamp;
      if (age > max_age_s_) {
        RCLCPP_WARN(this->get_logger(),
          "[pose_initializer] saved pose is %.0f s old (max %.0f) — ignoring",
          age, max_age_s_);
        status_ = "STALE age=" + std::to_string(static_cast<int>(age)) + "s";
        return false;
      }
    }
    return true;
  }

  void tick()
  {
    if (!armed_) {
      timer_->cancel();
      return;
    }
    if (pub_->get_subscription_count() == 0) {
      return;                      // slam_toolbox not up yet — keep waiting
    }
    if (!settle_started_) {
      // Localizer just appeared; give it settle_delay_s to finish
      // deserializing the posegraph before the seed lands.
      settle_started_ = true;
      settle_deadline_ = this->get_clock()->now() +
        rclcpp::Duration::from_seconds(settle_delay_s_);
      return;
    }
    if (this->get_clock()->now() < settle_deadline_) {
      return;
    }

    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "map";
    msg.pose.pose.position.x = x_;
    msg.pose.pose.position.y = y_;
    msg.pose.pose.orientation.z = qz_;
    msg.pose.pose.orientation.w = qw_;
    // Same covariance RViz uses for a manual estimate: 0.25 m² x/y, ~0.068 rad² yaw.
    msg.pose.covariance[0] = 0.25;
    msg.pose.covariance[7] = 0.25;
    msg.pose.covariance[35] = 0.0685;
    pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(),
      "[pose_initializer] SEEDED pose on /initialpose: map='%s' x=%.3f y=%.3f "
      "(override any time with SET POSE)",
      map_name_.c_str(), x_, y_);
    char buf[128];
    std::snprintf(buf, sizeof(buf), "SEEDED map='%s' x=%.2f y=%.2f",
      map_name_.c_str(), x_, y_);
    status_ = buf;
    publish_status();
    armed_ = false;
    timer_->cancel();
  }

  std::string save_path_;
  std::string map_name_;
  std::string status_;
  double settle_delay_s_{2.0};
  double max_age_s_{0.0};
  double x_{0.0}, y_{0.0}, qz_{0.0}, qw_{1.0};
  bool armed_{true};
  bool settle_started_{false};
  rclcpp::Time settle_deadline_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseInitializer>());
  rclcpp::shutdown();
  return 0;
}
