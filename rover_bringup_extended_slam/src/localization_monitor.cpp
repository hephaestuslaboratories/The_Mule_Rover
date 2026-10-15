// localization_monitor.cpp — scan-vs-map agreement scorer.
//
// WHAT IT ANSWERS
//   "Is the live scan sitting ON the map walls?"  That is the question the
//   operator currently answers by eyeballing RViz, and the one nothing in the
//   stack answers automatically.  slam_toolbox in localization mode publishes
//   no match-confidence value — unlike AMCL there is no covariance to threshold
//   — so a pose that has silently slid off the map is invisible until someone
//   notices the rover is drawn somewhere it is not.  That happened on
//   2026-08-21 after the rover manoeuvred around a person standing in the
//   LiDAR plane.
//
// WHY map->odom JUMP DETECTION IS NOT ENOUGH
//   A confidently-WRONG pose produces NO corrections at all — map->odom sits
//   perfectly still while the rover drives to the wrong room.  Jump size finds
//   the moment localization slips; it goes quiet for the whole time the rover
//   is actually lost.  Scan-vs-map agreement measures the CONDITION, not the
//   transition.  Use both.
//
// THE SECOND OUTPUT — WHY THIS NODE IS ALSO THE DYNAMIC-OBSTACLE FILTER
//   A ray whose endpoint lands in map FREE SPACE is hitting something that is
//   not supposed to be there: a person, a moved chair, an open door.  Those are
//   exactly the rays that should be withheld from the scan matcher (while still
//   being given to the costmaps, which must see them to avoid them).  The
//   per-ray classification needed for that filter is the same computation as
//   the score, so it is done once here.  `drop_frac` is published so the
//   rejection threshold can be set from measured household traffic instead of
//   guessed — see REPORT-ONLY below.
//
// REPORT-ONLY BY DESIGN
//   This node publishes numbers and changes NOTHING.  It does not filter, gate,
//   or stop the rover.  Collect `drop_frac` over normal operation — people
//   walking, furniture moved, doors opened — then set the filter's safety valve
//   above the observed normal.  One person at 1 m is roughly 8% of a scan; two
//   people plus a moved chair can legitimately reach 25%.  A valve set below
//   that disables filtering in a crowded room, which is the original bug.
//
// PUBLISHES
//   /rover/localization_score   Float32  on_wall / (on_wall + in_free)
//                                        1.0 = every classified ray on a wall
//   /rover/localization_drop    Float32  in_free / total_valid
//                                        what a dynamic filter would discard
//   /rover/localization_ok      Bool     score >= score_threshold
//
// READING THE NUMBERS
//   score high  + drop low   -> localized, quiet room.  Normal.
//   score high  + drop high  -> localized, busy room.  Filter would earn its
//                               keep here; localization still fine.
//   score LOW                -> the scan does not agree with the map.  Either
//                               lost, or the room has changed beyond
//                               recognition.  Both are worth knowing.
//
// COST
//   Evaluated at eval_rate (default 2 Hz), not per scan.  ~400 rays x a 5x5
//   cell search = ~10k array reads per evaluation.  Negligible — but note
//   §7.4: this Jetson signals CPU exhaustion by starving /tf, so if /tf sags
//   below ~79 Hz after adding this, lower eval_rate before blaming anything
//   downstream.

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using std::placeholders::_1;

class LocalizationMonitor : public rclcpp::Node
{
public:
  LocalizationMonitor()
  : Node("localization_monitor")
  {
    map_topic_         = this->declare_parameter<std::string>("map_topic", "/map");
    scan_topic_        = this->declare_parameter<std::string>("scan_topic", "/scan_filtered");
    eval_rate_         = this->declare_parameter<double>("eval_rate", 2.0);
    // A ray endpoint never lands exactly on a cell centre, and the map itself
    // was built at 0.05 m.  0.10 m = a 2-cell search radius: forgiving enough
    // that a correctly-localized rover scores high, tight enough that a pose
    // error of a third of a metre still shows up.
    match_tolerance_m_ = this->declare_parameter<double>("match_tolerance_m", 0.10);
    // Long rays carry more lateral error per degree of pose error and are more
    // likely to leave the mapped area entirely.  6 m keeps the score honest.
    max_eval_range_    = this->declare_parameter<double>("max_eval_range", 6.0);
    occupied_thresh_   = this->declare_parameter<int>("occupied_thresh", 65);
    free_thresh_       = this->declare_parameter<int>("free_thresh", 25);
    score_threshold_   = this->declare_parameter<double>("score_threshold", 0.60);
    // Below this many usable rays the ratio is noise, not a measurement.
    min_valid_rays_    = this->declare_parameter<int>("min_valid_rays", 40);
    stats_period_s_    = this->declare_parameter<double>("stats_period_s", 5.0);
    transform_timeout_ = this->declare_parameter<double>("transform_timeout", 0.10);

    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // The map is latched.  slam_toolbox in localization mode may publish it
    // exactly once, at startup — a VOLATILE subscription that connects a moment
    // late would then never receive it at all.  If no map ever arrives, check
    // the publisher's durability with:  ros2 topic info /map --verbose
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(1).transient_local(),
      std::bind(&LocalizationMonitor::map_cb, this, _1));

    // BEST_EFFORT is compatible with both a reliable and a best-effort
    // publisher; the reverse is not.  Scans are cheap to miss — we sample.
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LocalizationMonitor::scan_cb, this, _1));

    score_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "/rover/localization_score", 10);
    drop_pub_  = this->create_publisher<std_msgs::msg::Float32>(
      "/rover/localization_drop", 10);
    ok_pub_    = this->create_publisher<std_msgs::msg::Bool>(
      "/rover/localization_ok", 10);

    RCLCPP_INFO(
      this->get_logger(),
      "localization_monitor started — REPORT ONLY, filters nothing. "
      "map=%s scan=%s rate=%.1f Hz tol=%.2f m",
      map_topic_.c_str(), scan_topic_.c_str(), eval_rate_, match_tolerance_m_);
  }

private:
  void map_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_ = msg;
    // Map origin may carry a yaw.  Precompute its rotation once here rather
    // than per ray.  (slam_toolbox normally emits identity, but a map that
    // came from elsewhere need not.)
    const auto & q = msg->info.origin.orientation;
    const double origin_yaw =
      std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    origin_cos_ = std::cos(-origin_yaw);
    origin_sin_ = std::sin(-origin_yaw);

    RCLCPP_INFO(
      this->get_logger(), "map received: %ux%u @ %.3f m/cell",
      msg->info.width, msg->info.height, msg->info.resolution);
  }

  // Occupancy at a world point, or -1 if outside the grid.
  int cell_at(double wx, double wy) const
  {
    const auto & info = map_->info;
    const double dx = wx - info.origin.position.x;
    const double dy = wy - info.origin.position.y;
    // Rotate into grid axes (inverse of the origin yaw).
    const double gx = dx * origin_cos_ - dy * origin_sin_;
    const double gy = dx * origin_sin_ + dy * origin_cos_;

    const int mx = static_cast<int>(std::floor(gx / info.resolution));
    const int my = static_cast<int>(std::floor(gy / info.resolution));
    if (mx < 0 || my < 0 ||
        mx >= static_cast<int>(info.width) || my >= static_cast<int>(info.height))
    {
      return -1;
    }
    return map_->data[static_cast<size_t>(my) * info.width + static_cast<size_t>(mx)];
  }

  // True if any cell within match_tolerance_m of the point is occupied.
  bool near_occupied(double wx, double wy) const
  {
    const double res = map_->info.resolution;
    const int span = std::max(0, static_cast<int>(std::round(match_tolerance_m_ / res)));
    for (int oy = -span; oy <= span; ++oy) {
      for (int ox = -span; ox <= span; ++ox) {
        const int v = cell_at(wx + ox * res, wy + oy * res);
        if (v >= occupied_thresh_) {
          return true;
        }
      }
    }
    return false;
  }

  void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (!map_) {
      return;   // nothing to compare against yet
    }

    // Rate-limit: this is a health metric, not a control signal.
    const rclcpp::Time now = this->get_clock()->now();
    if (have_last_eval_) {
      const double period = (eval_rate_ > 0.0) ? (1.0 / eval_rate_) : 0.5;
      if ((now - last_eval_).seconds() < period) {
        return;
      }
    }
    last_eval_ = now;
    have_last_eval_ = true;

    // map -> laser at the scan's own timestamp.  Using the scan frame from the
    // message means a LiDAR remount or a URDF change needs no edit here.
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(
        map_->header.frame_id, msg->header.frame_id, msg->header.stamp,
        rclcpp::Duration::from_seconds(transform_timeout_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "no %s -> %s transform: %s",
        map_->header.frame_id.c_str(), msg->header.frame_id.c_str(), ex.what());
      return;
    }

    const auto & q = tf.transform.rotation;
    const double yaw =
      std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    const double tx = tf.transform.translation.x;
    const double ty = tf.transform.translation.y;

    size_t on_wall = 0, in_free = 0, ambiguous = 0;

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const double r = msg->ranges[i];
      if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max ||
          r > max_eval_range_)
      {
        continue;
      }
      const double a  = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
      const double lx = r * std::cos(a);
      const double ly = r * std::sin(a);
      const double wx = tx + lx * cy - ly * sy;
      const double wy = ty + lx * sy + ly * cy;

      if (near_occupied(wx, wy)) {
        ++on_wall;
        continue;
      }
      const int v = cell_at(wx, wy);
      if (v >= 0 && v <= free_thresh_) {
        ++in_free;        // something stands where the map says empty
      } else {
        ++ambiguous;      // unknown space, or off-grid, or mid-probability
      }
    }

    const size_t classified = on_wall + in_free;
    const size_t total      = classified + ambiguous;
    if (classified < static_cast<size_t>(min_valid_rays_)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "only %zu classifiable rays (min %d) — no score published",
        classified, min_valid_rays_);
      return;
    }

    const double score = static_cast<double>(on_wall) / static_cast<double>(classified);
    const double drop  = (total > 0)
      ? static_cast<double>(in_free) / static_cast<double>(total)
      : 0.0;

    std_msgs::msg::Float32 sm;  sm.data = static_cast<float>(score);
    std_msgs::msg::Float32 dm;  dm.data = static_cast<float>(drop);
    std_msgs::msg::Bool     om;  om.data = (score >= score_threshold_);
    score_pub_->publish(sm);
    drop_pub_->publish(dm);
    ok_pub_->publish(om);

    // Periodic line in the launch log, in the style of scan_deskew's stats —
    // the log is where this rover gets debugged.
    if (!have_last_stats_ || (now - last_stats_).seconds() >= stats_period_s_) {
      last_stats_ = now;
      have_last_stats_ = true;
      RCLCPP_INFO(
        this->get_logger(),
        "loc  score=%.3f  drop=%.3f  on_wall=%zu  in_free=%zu  unknown=%zu%s",
        score, drop, on_wall, in_free, ambiguous,
        (score >= score_threshold_) ? "" : "   <-- BELOW THRESHOLD");
    }
  }

  // Parameters
  std::string map_topic_, scan_topic_;
  double eval_rate_{2.0};
  double match_tolerance_m_{0.10};
  double max_eval_range_{6.0};
  int    occupied_thresh_{65};
  int    free_thresh_{25};
  double score_threshold_{0.60};
  int    min_valid_rays_{40};
  double stats_period_s_{5.0};
  double transform_timeout_{0.10};

  // State
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  double origin_cos_{1.0}, origin_sin_{0.0};
  rclcpp::Time last_eval_, last_stats_;
  bool have_last_eval_{false}, have_last_stats_{false};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr score_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr drop_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ok_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationMonitor>());
  rclcpp::shutdown();
  return 0;
}
