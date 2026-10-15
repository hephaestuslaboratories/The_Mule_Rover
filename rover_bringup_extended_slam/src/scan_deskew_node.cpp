// scan_deskew_node.cpp
// ─────────────────────
// LiDAR scan motion-compensation (deskewing) using IMU angular velocity.
// C++ port of rover_bringup/scan_deskew_node.py — same algorithm, same
// parameters, same fallback behaviour.
//
// Problem this solves
// ───────────────────
// A rotating 2D LiDAR captures ~360 rays over ~100 ms.  The LaserScan message
// treats all rays as if captured at a single instant (header.stamp), but each
// ray is really taken later than the previous one.  If the sensor body is
// rotating during the scan (rover turning), the last ray is at a different
// body orientation than the first — the scan is "smeared" in the rotation
// direction.  Scan matching and costmap stamping both assume one common pose
// per scan, so a smeared scan produces fake duplicated walls, confuses
// scan-to-map registration, and leaves phantom obstacles in the costmap.
//
// The fix
// ───────
// Use the high-rate gyro (IMU angular_velocity.z) to estimate the body
// rotation between the scan's start time and each ray's capture time.  Rotate
// each ray back to a common reference (scan-start) and rebin onto the output
// angle grid.
//
// Algorithm (per incoming scan)
// ─────────────────────────────
//   1. Pull the IMU angular-velocity history covering [scan_start, scan_end].
//   2. Build a piecewise-linear cumulative yaw integral over that window.
//   3. For each ray i at time t_i = scan_start + i·time_increment, look up the
//      cumulative yaw Δψ_i at that moment.
//   4. The ray's true direction in the body-at-scan-start frame is θ_i + Δψ_i
//      (for CCW body rotation = +yaw).  Its range r_i is unchanged.
//   5. Resample: interpolate the (corrected_angle, range) pairs onto the
//      original regular angle grid.  Missing sectors at either edge become
//      +inf (no return).
//
// Translation during the scan is NOT compensated here.  At indoor ranges and
// 0.2 m/s linear velocity, translation-induced parallax is <2 cm per scan —
// much smaller than rotation smear.
//
// Fallback behaviour
// ──────────────────
// If the IMU buffer does not cover the scan window (startup, IMU dropout) the
// scan is passed through unchanged with a throttled warning.  Deskewing never
// blocks the scan stream.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();

inline double stamp_to_sec(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

// Faithful port of numpy.interp for a single query point.
//
// xp must be increasing.  Values outside [xp.front(), xp.back()] return
// `left` / `right`.  The NaN fallback ladder matters: when two adjacent fp
// values are both +inf the naive slope is inf-inf = NaN, and numpy retries
// from the right-hand node and finally returns fp[i] when fp[i] == fp[i+1].
// Without that ladder, a pair of "no return" rays would deskew into NaN.
double interp_one(
  double x, const std::vector<double> & xp, const std::vector<double> & fp,
  double left, double right)
{
  const size_t n = xp.size();
  if (n == 0) {
    return left;
  }
  if (n == 1) {
    return (x < xp[0]) ? left : ((x > xp[0]) ? right : fp[0]);
  }
  if (x < xp.front()) {
    return left;
  }
  if (x > xp.back()) {
    return right;
  }
  // Upper bound of the bracketing interval.
  size_t j = static_cast<size_t>(
    std::upper_bound(xp.begin(), xp.end(), x) - xp.begin());
  if (j == 0) {
    return fp[0];
  }
  if (j >= n) {
    return fp[n - 1];
  }
  const size_t i = j - 1;
  const double x0 = xp[i], x1 = xp[j];
  const double y0 = fp[i], y1 = fp[j];
  if (x1 == x0) {
    return y0;
  }
  const double slope = (y1 - y0) / (x1 - x0);
  double res = slope * (x - x0) + y0;
  if (std::isnan(res)) {
    res = slope * (x - x1) + y1;
    if (std::isnan(res) && y0 == y1) {
      res = y0;
    }
  }
  return res;
}

// Vector form: same result as numpy.interp over an array of query points.
void interp_many(
  const std::vector<double> & x, const std::vector<double> & xp,
  const std::vector<double> & fp, double left, double right,
  std::vector<double> & out)
{
  out.resize(x.size());
  for (size_t k = 0; k < x.size(); ++k) {
    out[k] = interp_one(x[k], xp, fp, left, right);
  }
}
}  // namespace

class ScanDeskewNode : public rclcpp::Node
{
public:
  ScanDeskewNode()
  : Node("scan_deskew")
  {
    const std::string in_topic =
      this->declare_parameter<std::string>("in_scan_topic", "/scan");
    const std::string out_topic =
      this->declare_parameter<std::string>("out_scan_topic", "/scan_deskewed");
    const std::string imu_topic =
      this->declare_parameter<std::string>("imu_topic", "/imu/filtered");

    // Max IMU samples buffered — 2 s at 100 Hz = 200 samples is plenty to
    // cover one scan window with timing jitter margin.
    buf_size_ = static_cast<size_t>(this->declare_parameter<int>("imu_buffer_size", 400));

    // If true, a scan arriving with incomplete IMU coverage is dropped.
    // If false (default), such scans are published as-is (graceful degradation
    // during IMU startup / dropout).
    drop_on_gap_ = this->declare_parameter<bool>("drop_on_imu_gap", false);

    // Deadband: if the max |Δψ| across the scan is below this threshold (rad),
    // skip the math and republish the original scan.  Avoids wasted work when
    // the rover is stationary or moving straight.
    min_deskew_ = this->declare_parameter<double>("min_yaw_deskew_rad", 0.003);

    warn_throttle_ = this->declare_parameter<double>("warn_throttle_s", 2.0);

    // ── Scan timestamp convention correction ─────────────────────────────────
    // The ROS LaserScan contract states header.stamp = time of the FIRST ray.
    // Some drivers instead stamp at the LAST ray / publish moment — off by
    // exactly one scan period (~100 ms at 10 Hz).  If your driver stamps at
    // scan-END, set stamp_offset_s to -scan_time (e.g. -0.1 for 10 Hz).
    stamp_offset_ = this->declare_parameter<double>("stamp_offset_s", 0.0);

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, 50, std::bind(&ScanDeskewNode::imu_cb, this, _1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      in_topic, 10, std::bind(&ScanDeskewNode::scan_cb, this, _1));
    pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(out_topic, 10);

    stats_timer_ = this->create_wall_timer(
      std::chrono::seconds(5), std::bind(&ScanDeskewNode::stats_cb, this));

    RCLCPP_INFO(
      this->get_logger(),
      "scan_deskew ready  %s + %s → %s  buf=%zu  drop_on_gap=%s  min_deskew=%.2f°",
      in_topic.c_str(), imu_topic.c_str(), out_topic.c_str(),
      buf_size_, drop_on_gap_ ? "True" : "False", min_deskew_ * 180.0 / M_PI);
  }

private:
  // ── IMU callback ───────────────────────────────────────────────────────────

  void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const double t = stamp_to_sec(msg->header.stamp);
    const double w = msg->angular_velocity.z;
    std::lock_guard<std::mutex> lock(buf_mutex_);
    imu_buf_.emplace_back(t, w);
    while (imu_buf_.size() > buf_size_) {
      imu_buf_.pop_front();
    }
  }

  // ── Scan callback ──────────────────────────────────────────────────────────

  void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const size_t n_rays = msg->ranges.size();
    if (n_rays < 2) {
      pub_->publish(*msg);
      ++n_passthrough_;
      return;
    }

    // Some LiDAR drivers leave scan_time = 0.  Fall back to ray count × Δt.
    double scan_time = (msg->scan_time > 1e-4)
      ? static_cast<double>(msg->scan_time)
      : static_cast<double>(n_rays - 1) * static_cast<double>(msg->time_increment);
    if (scan_time <= 1e-4) {
      // Last-ditch fallback for drivers that also leave time_increment 0.
      scan_time = 0.1;
    }

    const double t_scan_start = stamp_to_sec(msg->header.stamp) + stamp_offset_;
    const double t_scan_end = t_scan_start + scan_time;

    // Snapshot the IMU buffer under the lock; the rest runs on the copy.
    std::vector<double> imu_t, imu_w;
    {
      std::lock_guard<std::mutex> lock(buf_mutex_);
      if (imu_buf_.size() < 2) {
        passthrough(*msg, "imu buffer empty");
        return;
      }
      imu_t.reserve(imu_buf_.size() + 2);
      imu_w.reserve(imu_buf_.size() + 2);
      for (const auto & pair : imu_buf_) {
        imu_t.push_back(pair.first);
        imu_w.push_back(pair.second);
      }
    }

    // Check coverage.  Ideally the IMU buffer brackets the full scan window,
    // but the IMU pipeline typically lags the LiDAR by a few tens of ms so the
    // newest IMU sample is often behind t_scan_end.  Rather than bail out we
    // extrapolate ω at the boundaries (constant-extrapolation = hold last known
    // rate).  Body angular velocity doesn't change significantly over 20–50 ms,
    // so this introduces negligible error and keeps deskewing working across
    // the whole run.
    //
    // We only bail out if the IMU stream is STALE — latest sample more than
    // imu_max_lag seconds before the scan starts — which indicates a real
    // problem (IMU dropout, clock desync, not just pipeline delay).
    constexpr double kImuMaxLag = 0.5;   // seconds
    if (imu_t.back() < t_scan_start - kImuMaxLag) {
      if (drop_on_gap_) {
        ++n_dropped_;
        throttled_warn(
          "IMU stale — dropping scan  (lag=" +
          std::to_string(t_scan_start - imu_t.back()) + "s)");
        return;
      }
      passthrough(
        *msg, "imu stale (" + std::to_string(t_scan_start - imu_t.back()) +
        "s lag) — publishing raw");
      return;
    }

    // Extrapolate ω at the ends of the window if the buffer doesn't fully cover
    // [t_scan_start, t_scan_end].  Constant extrapolation (hold first / last ω)
    // — sufficient because the uncovered tail is typically <50 ms.
    if (imu_t.front() > t_scan_start) {
      imu_t.insert(imu_t.begin(), t_scan_start);
      imu_w.insert(imu_w.begin(), imu_w.front());
    }
    if (imu_t.back() < t_scan_end) {
      imu_t.push_back(t_scan_end);
      imu_w.push_back(imu_w.back());
    }

    // ── Build piecewise-linear cumulative yaw over the scan window ──────────
    // Include boundary points at t_scan_start and t_scan_end (values
    // interpolated from the surrounding IMU samples) so the integration covers
    // exactly the scan duration.
    const double w_start = interp_one(t_scan_start, imu_t, imu_w, imu_w.front(), imu_w.back());
    const double w_end   = interp_one(t_scan_end,   imu_t, imu_w, imu_w.front(), imu_w.back());

    std::vector<double> node_t, node_w;
    node_t.reserve(imu_t.size() + 2);
    node_w.reserve(imu_t.size() + 2);
    node_t.push_back(t_scan_start);
    node_w.push_back(w_start);
    for (size_t i = 0; i < imu_t.size(); ++i) {
      if (imu_t[i] > t_scan_start && imu_t[i] < t_scan_end) {
        node_t.push_back(imu_t[i]);
        node_w.push_back(imu_w[i]);
      }
    }
    node_t.push_back(t_scan_end);
    node_w.push_back(w_end);

    // Guard against duplicate timestamps (an IMU sample landing exactly on
    // t_scan_start/end would otherwise be included twice).  np.unique also
    // sorts; the input is already ordered, so a stable de-dup is equivalent.
    {
      std::vector<double> ut, uw;
      ut.reserve(node_t.size());
      uw.reserve(node_w.size());
      for (size_t i = 0; i < node_t.size(); ++i) {
        if (ut.empty() || node_t[i] != ut.back()) {
          ut.push_back(node_t[i]);
          uw.push_back(node_w[i]);
        }
      }
      node_t.swap(ut);
      node_w.swap(uw);
    }
    if (node_t.size() < 2) {
      passthrough(*msg, "degenerate IMU segment");
      return;
    }

    // Trapezoidal cumulative integral: Δψ(t_i) from t_scan_start.
    std::vector<double> cum_yaw(node_t.size(), 0.0);
    for (size_t i = 1; i < node_t.size(); ++i) {
      const double dt = node_t[i] - node_t[i - 1];
      const double w_avg = 0.5 * (node_w[i - 1] + node_w[i]);
      cum_yaw[i] = cum_yaw[i - 1] + dt * w_avg;
    }

    // ── Per-ray timestamps → per-ray Δψ ────────────────────────────────────
    std::vector<double> ray_t(n_rays);
    const double step = (n_rays > 1) ? scan_time / static_cast<double>(n_rays - 1) : 0.0;
    for (size_t i = 0; i < n_rays; ++i) {
      ray_t[i] = t_scan_start + static_cast<double>(i) * step;
    }
    std::vector<double> delta_yaw;
    interp_many(ray_t, node_t, cum_yaw, cum_yaw.front(), cum_yaw.back(), delta_yaw);

    // Deadband: if the rover barely turned during the scan, skip the math.
    double max_abs_dyaw = 0.0;
    for (double d : delta_yaw) {
      max_abs_dyaw = std::max(max_abs_dyaw, std::fabs(d));
    }
    if (max_abs_dyaw < min_deskew_) {
      pub_->publish(*msg);
      ++n_passthrough_;
      return;
    }

    // ── Correct each ray's angle and resample onto the output grid ──────────
    // Convention: IMU z is +CCW (ENU).  When the body rotates +CCW by Δψ during
    // the scan, a ray the driver labelled θ_i (assuming no body motion) really
    // pointed at (θ_i + Δψ_i) in the body-at-scan-start frame.  So shift ray
    // angles by +Δψ_i, then rebin.
    std::vector<double> angles(n_rays), corrected(n_rays), ranges(n_rays);
    for (size_t i = 0; i < n_rays; ++i) {
      angles[i] = static_cast<double>(msg->angle_min) +
        static_cast<double>(i) * static_cast<double>(msg->angle_increment);
      corrected[i] = angles[i] + delta_yaw[i];
      const double r = static_cast<double>(msg->ranges[i]);
      // Non-finite ranges must not pollute the interpolation.  Replace them
      // with +inf so they become legitimate "no return" entries in the
      // rebinning.  For this rover's deskew magnitudes (<5° typical) each
      // output cell pulls from ~1 input cell.
      ranges[i] = (std::isfinite(r) && r > 0.0) ? r : kInf;
    }

    // interp needs monotonically increasing x.  The corrected angles are
    // monotonic (the original grid is, and Δψ is a smooth function of ray
    // index) — but the DIRECTION depends on the sign of angle_increment.
    std::vector<double> xp, fp;
    if (msg->angle_increment > 0.0f) {
      xp = corrected;
      fp = ranges;
    } else {
      xp.assign(corrected.rbegin(), corrected.rend());
      fp.assign(ranges.rbegin(), ranges.rend());
    }

    std::vector<double> new_ranges;
    interp_many(angles, xp, fp, kInf, kInf, new_ranges);

    // Build output message (copy all metadata from the input).
    auto out = std::make_unique<sensor_msgs::msg::LaserScan>();
    out->header          = msg->header;
    out->angle_min       = msg->angle_min;
    out->angle_max       = msg->angle_max;
    out->angle_increment = msg->angle_increment;
    out->time_increment  = msg->time_increment;
    out->scan_time       = msg->scan_time;
    out->range_min       = msg->range_min;
    out->range_max       = msg->range_max;
    out->ranges.resize(n_rays);
    for (size_t i = 0; i < n_rays; ++i) {
      out->ranges[i] = static_cast<float>(new_ranges[i]);
    }
    out->intensities = msg->intensities;

    pub_->publish(std::move(out));
    ++n_deskewed_;
  }

  // ── Helpers ────────────────────────────────────────────────────────────────

  void passthrough(const sensor_msgs::msg::LaserScan & msg, const std::string & reason)
  {
    pub_->publish(msg);
    ++n_passthrough_;
    throttled_warn("passthrough — " + reason);
  }

  void throttled_warn(const std::string & text)
  {
    const double now = this->get_clock()->now().seconds();
    if (now - last_warn_time_ >= warn_throttle_) {
      RCLCPP_WARN(this->get_logger(), "%s", text.c_str());
      last_warn_time_ = now;
    }
  }

  void stats_cb()
  {
    const long total = n_deskewed_ + n_passthrough_ + n_dropped_;
    if (total == 0) {
      return;
    }
    RCLCPP_INFO(
      this->get_logger(),
      "stats  deskewed=%ld  passthrough=%ld  dropped=%ld  (last 5s)",
      n_deskewed_, n_passthrough_, n_dropped_);
    n_deskewed_ = n_passthrough_ = n_dropped_ = 0;
  }

  // Parameters
  size_t buf_size_{400};
  bool   drop_on_gap_{false};
  double min_deskew_{0.003};
  double warn_throttle_{2.0};
  double stamp_offset_{0.0};

  // IMU buffer — (timestamp_sec, omega_z_rad_per_s), kept in time order.
  std::deque<std::pair<double, double>> imu_buf_;
  std::mutex buf_mutex_;

  // Diagnostics
  double last_warn_time_{0.0};
  long n_passthrough_{0};
  long n_deskewed_{0};
  long n_dropped_{0};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanDeskewNode>());
  rclcpp::shutdown();
  return 0;
}
