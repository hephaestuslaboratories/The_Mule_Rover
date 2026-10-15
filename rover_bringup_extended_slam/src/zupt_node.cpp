// zupt_node.cpp — Zero Velocity Update filter for EKF yaw stabilization.
//
// C++ port of rover_bringup/zupt_node.py.  Behaviour is identical:
//
// Sits between /imu/data and /imu/filtered.
// When stationary (wheel vx ≈ 0 AND measured IMU yaw rate ≈ 0):
//   → zeroes angular_velocity.z with tight covariance (suppresses gyro bias)
// When moving/turning:
//   → passes IMU data through unchanged
//
// Stationarity is determined from MEASURED signals (wheel encoder + raw IMU)
// rather than commanded velocity.  cmd_vel was the previous trigger but has
// two failure modes: (1) when the only /cmd_vel publisher goes silent, the
// cached cmd_vyaw stays frozen at the last value and ZUPT never re-engages —
// gyro bias accumulates as visible scan drift;  (2) the rover might be
// commanded to move while physically pinned against an obstacle.  Measured
// signals are robust to both.
//
// enabled=false -> pure passthrough (/imu/data copied to /imu/filtered
// untouched).  The node must keep running either way because the EKF and
// scan_deskew subscribe to /imu/filtered.  Disabled in nav mode: the
// drivetrain's low-speed turns are pulsed ("clicky"), and between pulses the
// gyro dips under vyaw_threshold long enough for ZUPT to flicker on and erase
// real rotation mid-turn — poison for Nav2.
// Live-toggleable:  ros2 param set /zupt_node enabled true|false

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

using std::placeholders::_1;

class ZUPTNode : public rclcpp::Node
{
public:
  ZUPTNode()
  : Node("zupt_node")
  {
    vx_threshold_     = this->declare_parameter<double>("vx_threshold", 0.01);
    vyaw_threshold_   = this->declare_parameter<double>("vyaw_threshold", 0.05);
    stationary_delay_ = this->declare_parameter<double>("stationary_delay", 0.2);
    zupt_covariance_  = this->declare_parameter<double>("zupt_covariance", 0.0001);
    enabled_          = this->declare_parameter<bool>("enabled", true);

    param_cb_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & p : params) {
          if (p.get_name() == "enabled") {
            enabled_ = p.as_bool();
            zupt_active_ = false;
            have_stationary_since_ = false;
            RCLCPP_INFO(
              this->get_logger(), "ZUPT %s",
              enabled_ ? "ENABLED" : "DISABLED (passthrough)");
          }
        }
        return result;
      });

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 10, std::bind(&ZUPTNode::imu_cb, this, _1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/wheel/odom_twist", 10, std::bind(&ZUPTNode::odom_cb, this, _1));

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/filtered", 10);

    RCLCPP_INFO(this->get_logger(), "ZUPT node started (measured stationarity)");
  }

private:
  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    vx_ = msg->twist.twist.linear.x;
  }

  bool is_stationary() const
  {
    return std::fabs(vx_) < vx_threshold_ &&
           std::fabs(measured_vyaw_) < vyaw_threshold_;
  }

  void imu_cb(sensor_msgs::msg::Imu::UniquePtr msg)
  {
    if (!enabled_) {
      imu_pub_->publish(std::move(msg));   // pure passthrough
      return;
    }

    const rclcpp::Time now = this->get_clock()->now();

    // Update measured yaw rate from the raw IMU before any zeroing.  This is
    // the signal we use to decide stationarity; using the *output* message
    // would be circular once we start zeroing.
    measured_vyaw_ = msg->angular_velocity.z;

    if (is_stationary()) {
      if (!have_stationary_since_) {
        stationary_since_ = now;
        have_stationary_since_ = true;
      }
      const double elapsed = (now - stationary_since_).seconds();
      if (elapsed >= stationary_delay_) {
        if (!zupt_active_) {
          RCLCPP_INFO(this->get_logger(), "ZUPT activated — suppressing gyro bias");
          zupt_active_ = true;
        }
      }
    } else {
      if (zupt_active_) {
        RCLCPP_INFO(this->get_logger(), "ZUPT deactivated — robot moving/turning");
      }
      zupt_active_ = false;
      have_stationary_since_ = false;
    }

    if (zupt_active_) {
      msg->angular_velocity.z = 0.0;
      msg->angular_velocity_covariance[8] = zupt_covariance_;
    }

    imu_pub_->publish(std::move(msg));
  }

  // Parameters
  double vx_threshold_{0.01};
  double vyaw_threshold_{0.05};
  double stationary_delay_{0.2};
  double zupt_covariance_{0.0001};
  bool   enabled_{true};

  // State
  double vx_{0.0};
  double measured_vyaw_{0.0};
  rclcpp::Time stationary_since_;
  bool have_stationary_since_{false};
  bool zupt_active_{false};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZUPTNode>());
  rclcpp::shutdown();
  return 0;
}
