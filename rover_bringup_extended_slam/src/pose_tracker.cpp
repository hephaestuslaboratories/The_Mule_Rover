// pose_tracker.cpp — persist the rover's latest map-frame pose to the NVMe.
//
// SOURCE: TF map->base_link sampled on a timer — NOT slam_toolbox's /pose
// topic.  /pose only updates when a scan is processed, which requires motion
// beyond minimum_travel_* thresholds; a final parking rotation below the
// heading threshold never reaches /pose, so the persisted heading went stale
// by up to tens of degrees (the "right spot, wrong heading" auto-init bug).
// It also never reflected a stationary SET POSE.  The TF chain has neither
// problem: odom->base_link tracks every motion via the EKF in real time, and
// map->odom re-anchors on /initialpose immediately.  Same source SAVE PLACE
// uses.
//
// GUARD: nothing is persisted after launch until an /initialpose has been
// seen this session (auto-seed or manual SET POSE).  Before that the stack's
// pose is a guess, and a guess must never overwrite the last good save.
//
// File writes are atomic (temp + rename): a reader or power cut sees the
// complete old pose or the complete new one, never a torn file.

#include <chrono>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using std::placeholders::_1;

class PoseTracker : public rclcpp::Node
{
public:
  PoseTracker()
  : Node("pose_tracker")
  {
    save_path_ = this->declare_parameter<std::string>(
      "save_path", "/workspaces/isaac_ros-dev/last_pose.json");
    sample_rate_hz_ = this->declare_parameter<double>("sample_rate_hz", 5.0);
    log_period_s_ = this->declare_parameter<double>("log_period_s", 0.0);
    // Map identity (the `posegraph` launch arg); basename matches the
    // map_name scoping used by rover_places and pose_initializer.
    std::string map_id = this->declare_parameter<std::string>("map_id", "");
    const size_t slash = map_id.find_last_of('/');
    map_name_ = (slash == std::string::npos) ? map_id : map_id.substr(slash + 1);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    init_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10, std::bind(&PoseTracker::init_cb, this, _1));

    const double rate = (sample_rate_hz_ > 0.0) ? sample_rate_hz_ : 5.0;
    sample_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / rate),
      std::bind(&PoseTracker::sample_tick, this));

    if (log_period_s_ > 0.0) {
      log_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(log_period_s_),
        std::bind(&PoseTracker::log_tick, this));
    }

    RCLCPP_INFO(this->get_logger(),
      "[pose_tracker] tracking TF map->base_link at %.1f Hz (map '%s'); "
      "persisting after first /initialpose",
      rate, map_name_.c_str());
  }

private:
  void init_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr)
  {
    if (!have_init_) {
      have_init_ = true;
      RCLCPP_INFO(this->get_logger(),
        "[pose_tracker] /initialpose seen — persistence enabled");
    }
  }

  void sample_tick()
  {
    if (!have_init_) {
      return;
    }
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    } catch (const std::exception &) {
      return;                       // not localized right now — keep last save
    }
    x_  = tf.transform.translation.x;
    y_  = tf.transform.translation.y;
    qz_ = tf.transform.rotation.z;
    qw_ = tf.transform.rotation.w;
    have_pose_ = true;
    persist();
  }

  // Atomic write: temp file, then rename() over the real one.
  void persist()
  {
    if (save_path_.empty()) {
      return;
    }
    const std::string tmp = save_path_ + ".tmp";
    {
      std::ofstream f(tmp, std::ios::trunc);
      if (!f) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 10000,
          "[pose_tracker] cannot write %s", tmp.c_str());
        return;
      }
      f.precision(17);
      f << "{\"map_name\": \"" << map_name_ << "\", \"x\": " << x_
        << ", \"y\": " << y_
        << ", \"qz\": " << qz_ << ", \"qw\": " << qw_
        << ", \"stamp\": " << this->get_clock()->now().seconds() << "}\n";
    }
    std::rename(tmp.c_str(), save_path_.c_str());
  }

  void log_tick()
  {
    if (!have_pose_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 10000,
        "[pose_tracker] no pose persisted yet (waiting for /initialpose "
        "and TF map->base_link)");
      return;
    }
    RCLCPP_INFO(
      this->get_logger(), "[pose_tracker] x=%.3f y=%.3f qz=%.4f qw=%.4f",
      x_, y_, qz_, qw_);
  }

  std::string save_path_;
  std::string map_name_;
  double sample_rate_hz_{5.0};
  double log_period_s_{0.0};

  double x_{0.0}, y_{0.0}, qz_{0.0}, qw_{1.0};
  bool have_pose_{false};
  bool have_init_{false};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_sub_;
  rclcpp::TimerBase::SharedPtr sample_timer_;
  rclcpp::TimerBase::SharedPtr log_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseTracker>());
  rclcpp::shutdown();
  return 0;
}
