// ─────────────────────────────────────────────────────────────────────────────
// tof_down_filter.cpp — turns the down-looking ToF into an obstacle Nav2 can use.
//
// WHY THIS NODE EXISTS
//   The down-looker ALWAYS has a target: the floor.  Fed straight into
//   RangeSensorLayer it would mark a permanent phantom obstacle ~0.39 m ahead
//   that nothing could ever clear.  Its raw range means nothing on its own —
//   only the DIFFERENCE from the clear-floor baseline carries information:
//
//       reading ~= baseline (0.53 m)  -> clear floor, nothing there
//       reading <  threshold (0.52 m) -> something is standing on the floor
//       reading >  baseline, or none  -> the floor fell away: a DROP
//
//   Measured on this rover: 0.53 m on clear floor, oscillating upward but
//   never below.  So the obstacle threshold sits just under the noise floor.
//   An object shortens the reading by roughly 1.5x its height
//   (height / sin(43 deg)), so 0.52 m catches anything taller than ~7 mm.
//
// WHY THE OUTPUT IS ON A DIFFERENT FRAME
//   The sensor is pitched down 43 deg, but RangeSensorLayer is 2D and would
//   place the mark at the full slant range along the sensor's heading — too
//   far away.  This node projects onto the floor plane and republishes on a
//   FLAT frame (tof_down_proj_link), where the range is a true horizontal
//   distance:  horizontal = slant * cos(tilt).
//
// WHAT IT COVERS
//   The cone reaches the floor from about 0.24 m to 0.65 m ahead of the sensor
//   (36.5 cm height, 43 deg tilt).  Closer than 0.24 m it sees nothing — no
//   sensor on this rover sees under its own bumper.  So a mark laid here must
//   SURVIVE the final approach, which is range_layer's clear_threshold's job,
//   not this node's.
//
// SMALL OBJECTS, HONESTLY
//   The sensor reports the dominant target in its cone.  At 0.53 m the cone is
//   ~26 cm wide, so a very small object competes with a lot of floor and may
//   not move the reading at all.  If that becomes the limit, the fix is a
//   narrower ROI for this sensor, not a tighter threshold here.
// ─────────────────────────────────────────────────────────────────────────────

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

class TofDownFilter : public rclcpp::Node
{
public:
  TofDownFilter()
  : Node("tof_down_filter")
  {
    in_topic_    = declare_parameter<std::string>("input_topic", "/tof/down");
    out_topic_   = declare_parameter<std::string>("output_topic", "/tof/down_obstacle");
    out_frame_   = declare_parameter<std::string>("output_frame", "tof_down_proj_link");
    tilt_deg_    = declare_parameter<double>("tilt_deg", 43.0);
    baseline_    = declare_parameter<double>("floor_baseline_m", 0.53);
    threshold_   = declare_parameter<double>("obstacle_threshold_m", 0.52);
    drop_margin_ = declare_parameter<double>("drop_margin_m", 0.08);
    streak_      = declare_parameter<int>("consecutive_readings", 3);
    fov_         = declare_parameter<double>("field_of_view", 0.4712);

    cos_tilt_ = std::cos(tilt_deg_ * M_PI / 180.0);
    // Near edge of the cone on the floor, horizontal.  A drop is marked HERE,
    // so the rover stops short of the hole rather than driving to its far rim.
    const double half_fov_deg = fov_ * 180.0 / M_PI / 2.0;
    near_edge_ = std::cos((tilt_deg_ + half_fov_deg) * M_PI / 180.0) * baseline_;

    pub_    = create_publisher<sensor_msgs::msg::Range>(out_topic_, 10);
    drop_   = create_publisher<std_msgs::msg::Bool>("/rover/tof_drop", 10);
    status_ = create_publisher<std_msgs::msg::String>("/rover/tof_status", 10);

    sub_ = create_subscription<sensor_msgs::msg::Range>(
      in_topic_, 10, [this](sensor_msgs::msg::Range::SharedPtr m) {on_range(m);});

    RCLCPP_INFO(get_logger(),
                "down filter: floor %.3f m, flag below %.3f m, tilt %.1f deg, "
                "%d consecutive readings -> %s",
                baseline_, threshold_, tilt_deg_, streak_, out_topic_.c_str());
  }

private:
  void on_range(const sensor_msgs::msg::Range::SharedPtr msg)
  {
    const double r = msg->range;
    const bool no_return = (r >= msg->max_range);
    const bool is_drop   = no_return || (r > baseline_ + drop_margin_);
    const bool is_object = (!no_return) && (r < threshold_);

    // Debounce both ways: one dropped reading over a dark tile is not a
    // staircase, and one glint is not an obstacle.
    drop_run_   = is_drop   ? drop_run_ + 1   : 0;
    object_run_ = is_object ? object_run_ + 1 : 0;

    sensor_msgs::msg::Range out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = out_frame_;
    out.radiation_type = sensor_msgs::msg::Range::INFRARED;
    out.field_of_view = static_cast<float>(fov_);
    out.min_range = 0.04f;
    // Horizontal distance to the clear-floor hit point == "nothing there".
    // Publishing exactly this is what CLEARS the cone in RangeSensorLayer.
    out.max_range = static_cast<float>(baseline_ * cos_tilt_);

    if (drop_run_ >= streak_) {
      out.range = static_cast<float>(near_edge_);
      set_drop(true);
    } else if (object_run_ >= streak_) {
      out.range = static_cast<float>(r * cos_tilt_);
      set_drop(false);
      if (++obj_log_ % 10 == 1) {
        RCLCPP_INFO(get_logger(), "object: %.3f m slant -> %.3f m ahead "
                    "(%.1f cm tall)", r, r * cos_tilt_,
                    (baseline_ - r) * std::sin(tilt_deg_ * M_PI / 180.0) * 100.0);
      }
    } else {
      out.range = out.max_range;
      set_drop(false);
      obj_log_ = 0;
    }
    pub_->publish(out);
  }

  void set_drop(bool state)
  {
    if (state == drop_state_) {return;}
    drop_state_ = state;
    std_msgs::msg::Bool b;
    b.data = state;
    drop_->publish(b);
    std_msgs::msg::String s;
    s.data = state ? "TOF_DROP_DETECTED" : "TOF_DROP_CLEARED";
    status_->publish(s);
    if (state) {
      RCLCPP_WARN(get_logger(), "DROP ahead — no floor return for %d readings",
                  streak_);
    }
  }

  std::string in_topic_, out_topic_, out_frame_;
  double tilt_deg_, baseline_, threshold_, drop_margin_, fov_;
  double cos_tilt_ = 1.0, near_edge_ = 0.25;
  int streak_;
  int drop_run_ = 0, object_run_ = 0, obj_log_ = 0;
  bool drop_state_ = false;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr drop_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TofDownFilter>());
  rclcpp::shutdown();
  return 0;
}
