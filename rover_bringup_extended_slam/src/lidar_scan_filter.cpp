// lidar_scan_filter.cpp — sanitise + spike-filter a 360° LaserScan.
//
// C++ port of rover_bringup/lidar_scan_filter.py.  The filter maths are
// unchanged — same output, ray for ray.  The Python version was itself a
// numpy rewrite of the original scalar loops, verified identical over 400
// randomised scans; this port follows the ORIGINAL scalar semantics, which
// the numpy version was proven equal to.
//
// What it does, in order:
// - Remove NaN/Inf and out-of-band ranges (→ +inf = "no return")
// - Simple median-of-3 spike filter (only where all three samples are finite)
// - Inter-scan quality gate with hysteresis (suppresses motion-blurred scans)
// - Optional stride downsampling
//
// Quality gate:
//   Two independent checks flag a scan as 'bad':
//     1. Valid-ray fraction below min_valid_frac (gross motion blur / ray loss).
//     2. Inter-scan rate-of-change: fraction of rays whose range changed by
//        more than roc_threshold metres since the previous scan exceeds
//        roc_bad_frac (vibration / scan smear without a full ray-loss event).
//
//   Hysteresis prevents toggling:
//     - Suppression begins after n_bad_to_suppress consecutive bad scans.
//     - Suppression ends only after n_good_to_resume consecutive clean scans.
//
//   Master switch: enable_quality_gate=false disables it entirely and restores
//   exactly the publish-every-scan behaviour.

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

namespace
{
constexpr float kInf = std::numeric_limits<float>::infinity();

// median of three == max(min(a,b), min(max(a,b), c))
inline float median3(float a, float b, float c)
{
  return std::max(std::min(a, b), std::min(std::max(a, b), c));
}
}  // namespace

class LidarScanFilter : public rclcpp::Node
{
public:
  LidarScanFilter()
  : Node("lidar_scan_filter")
  {
    in_topic_  = this->declare_parameter<std::string>("in_topic", "/scan");
    out_topic_ = this->declare_parameter<std::string>("out_topic", "/scan_filtered");
    min_r_     = static_cast<float>(this->declare_parameter<double>("min_range", 0.20));
    max_r_     = static_cast<float>(this->declare_parameter<double>("max_range", 25.0));
    enable_median3_ = this->declare_parameter<bool>("enable_median3", true);

    // ── Downsampling parameters ──────────────────────────────────────────────
    // Master switch: false → all rays passed through unchanged.
    // stride=2 keeps every 2nd ray → ~180 pts at 2° resolution from a 360-pt scan.
    downsample_ = this->declare_parameter<bool>("enable_downsample", false);
    // declare_parameter<int> yields int64_t — take the max in that type, then
    // narrow deliberately.
    ds_stride_ = static_cast<int>(
      std::max<int64_t>(1, this->declare_parameter<int>("downsample_stride", 2)));

    // ── Quality gate parameters ──────────────────────────────────────────────
    quality_gate_ = this->declare_parameter<bool>("enable_quality_gate", true);

    // Per-ray range-change threshold between consecutive scans (metres).
    // At angular rates below 0.15 rad/s the maximum expected wall-range change
    // is ~0.075 m at 5 m distance — so 0.30 m gives a comfortable 4× margin
    // against false positives.
    roc_thr_ = static_cast<float>(this->declare_parameter<double>("roc_threshold", 0.30));

    // If this fraction of *comparable* (both finite) rays exceeds roc_threshold,
    // the scan is flagged bad by the ROC check.
    roc_bad_frac_ = this->declare_parameter<double>("roc_bad_frac", 0.25);

    // If fewer than this fraction of rays are finite after sanitisation, the
    // scan is flagged bad by the ray-loss check.  (17-session test data showed
    // 35–57 % ray loss during turns = CRITICAL.)
    min_valid_frac_ = this->declare_parameter<double>("min_valid_frac", 0.70);

    n_bad_  = this->declare_parameter<int>("n_bad_to_suppress", 2);
    n_good_ = this->declare_parameter<int>("n_good_to_resume", 3);

    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      in_topic_, 10, std::bind(&LidarScanFilter::cb, this, _1));
    pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(out_topic_, 20);

    const std::string ds_desc =
      downsample_ ? ("ON stride=" + std::to_string(ds_stride_)) : std::string("OFF");
    RCLCPP_INFO(
      this->get_logger(),
      "Filtering %s -> %s  quality_gate=%s  downsample=%s",
      in_topic_.c_str(), out_topic_.c_str(),
      quality_gate_ ? "ON" : "OFF", ds_desc.c_str());
  }

private:
  // ───────────────────────────────────────────────────────────────────────────
  // Quality assessment
  // ───────────────────────────────────────────────────────────────────────────

  // Check 1 — valid-ray fraction: counts finite rays after sanitisation.  If
  // fewer than min_valid_frac of all rays are finite, the scan is
  // motion-blurred or sensor-faulted.
  //
  // Check 2 — inter-scan rate-of-change: for every ray finite in BOTH the
  // current and previous scan, compute |current - previous|.  If more than
  // roc_bad_frac of those comparable rays changed by more than roc_threshold
  // metres, the scan shows rapid range fluctuation indicating vibration or
  // smear.  Skipped when no previous scan exists (first scan always passes) and
  // when fewer than 10 rays are comparable (avoid dividing by near-zero).
  bool assess_quality(const std::vector<float> & ranges) const
  {
    const size_t n = ranges.size();
    if (n == 0) {
      return false;
    }

    size_t valid_count = 0;
    for (float r : ranges) {
      if (std::isfinite(r)) {
        ++valid_count;
      }
    }
    if (static_cast<double>(valid_count) / static_cast<double>(n) < min_valid_frac_) {
      return false;
    }

    if (!prev_ranges_.empty() && prev_ranges_.size() == n) {
      size_t roc_count = 0;
      size_t comparable = 0;
      for (size_t i = 0; i < n; ++i) {
        const float cur = ranges[i];
        const float prv = prev_ranges_[i];
        if (std::isfinite(cur) && std::isfinite(prv)) {
          ++comparable;
          if (std::fabs(cur - prv) > roc_thr_) {
            ++roc_count;
          }
        }
      }
      if (comparable >= 10 &&
          (static_cast<double>(roc_count) / static_cast<double>(comparable)) > roc_bad_frac_)
      {
        return false;
      }
    }

    return true;
  }

  // Entering suppression: n_bad_to_suppress consecutive bad scans.
  // Exiting  suppression: n_good_to_resume  consecutive clean scans.
  // The asymmetry (exit threshold higher than entry) prevents rapid toggling.
  bool update_hysteresis(bool scan_good)
  {
    if (scan_good) {
      bad_streak_ = 0;
      ++good_streak_;
      if (suppressed_ && good_streak_ >= n_good_) {
        suppressed_ = false;
        RCLCPP_INFO(
          this->get_logger(),
          "ScanQuality: suppression lifted — %d consecutive clean scans received  "
          "(gate event suppressed %d scans total)",
          n_good_, n_suppressed_);
        n_suppressed_ = 0;
      }
    } else {
      good_streak_ = 0;
      ++bad_streak_;
      if (!suppressed_ && bad_streak_ >= n_bad_) {
        suppressed_ = true;
        RCLCPP_WARN(
          this->get_logger(),
          "ScanQuality: suppressing %s output — %d consecutive bad scans "
          "(valid-frac or ROC check failed)",
          out_topic_.c_str(), n_bad_);
      }
    }
    return !suppressed_;
  }

  // ───────────────────────────────────────────────────────────────────────────
  // Scan callback
  // ───────────────────────────────────────────────────────────────────────────

  void cb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const size_t n = msg->ranges.size();
    std::vector<float> ranges(msg->ranges.begin(), msg->ranges.end());

    // ── Step 1: sanitise and clamp ─────────────────────────────────────────
    // NaN, ±inf, and anything outside [min_range, max_range] all become +inf,
    // i.e. "no return".  (isfinite catches NaN/inf; the comparisons are false
    // for NaN, so nothing is double-handled.)
    for (size_t i = 0; i < n; ++i) {
      const float r = ranges[i];
      if (!std::isfinite(r) || r < min_r_ || r > max_r_) {
        ranges[i] = kInf;
      }
    }

    // ── Step 2: optional median-of-3 spike filter ──────────────────────────
    // Interior rays only, and only where the ray and both of its neighbours are
    // finite.  `prev` holds the ORIGINAL value of ranges[i-1] so no corrected
    // value ever feeds forward into the next window.
    if (enable_median3_ && n >= 3) {
      float prev = ranges[0];
      for (size_t i = 1; i + 1 < n; ++i) {
        const float a = prev;
        const float b = ranges[i];
        const float c = ranges[i + 1];
        prev = ranges[i];   // capture original before potential overwrite
        if (std::isfinite(a) && std::isfinite(b) && std::isfinite(c)) {
          ranges[i] = median3(a, b, c);
        }
      }
    }

    // ── Step 3: quality gate ───────────────────────────────────────────────
    bool should_publish = true;
    if (quality_gate_) {
      const bool scan_good = assess_quality(ranges);
      should_publish = update_hysteresis(scan_good);
      if (!should_publish) {
        ++n_suppressed_;
      }
    }

    // Always update the previous-scan cache — during suppression too, and also
    // when the gate is off so it is ready if the gate is enabled later via
    // parameter reconfiguration.  Without this, the first scan after
    // suppression lifts would compare against the last *published* scan
    // (potentially many scans ago) and could falsely trigger another bad-streak.
    prev_ranges_ = ranges;

    if (!should_publish) {
      return;
    }

    // ── Step 4: optional downsampling ──────────────────────────────────────
    // Keeps every Nth ray (stride=2 → ~180 pts from a 360-pt scan).
    // angle_increment and time_increment are scaled by stride so downstream
    // consumers (SLAM, nav2) see a geometrically correct scan.
    auto out = std::make_unique<sensor_msgs::msg::LaserScan>();
    out->header          = msg->header;
    out->angle_min       = msg->angle_min;
    out->range_min       = msg->range_min;
    out->range_max       = msg->range_max;
    out->scan_time       = msg->scan_time;

    if (downsample_ && ds_stride_ > 1) {
      const size_t s = static_cast<size_t>(ds_stride_);
      out->ranges.reserve((n + s - 1) / s);
      for (size_t i = 0; i < n; i += s) {
        out->ranges.push_back(ranges[i]);
      }
      if (!msg->intensities.empty()) {
        out->intensities.reserve((msg->intensities.size() + s - 1) / s);
        for (size_t i = 0; i < msg->intensities.size(); i += s) {
          out->intensities.push_back(msg->intensities[i]);
        }
      }
      out->angle_increment = msg->angle_increment * static_cast<float>(s);
      out->time_increment  = msg->time_increment * static_cast<float>(s);
      // angle_max is recomputed from the actual number of output rays so it
      // stays consistent even if the ray count is not divisible by the stride.
      out->angle_max = out->ranges.empty()
        ? msg->angle_max
        : msg->angle_min +
          static_cast<float>(out->ranges.size() - 1) * out->angle_increment;
    } else {
      out->ranges          = std::move(ranges);
      out->intensities     = msg->intensities;
      out->angle_increment = msg->angle_increment;
      out->time_increment  = msg->time_increment;
      out->angle_max       = msg->angle_max;
      // `ranges` was moved out; prev_ranges_ already holds its own copy.
    }

    pub_->publish(std::move(out));
  }

  // Parameters
  std::string in_topic_, out_topic_;
  float  min_r_{0.20f}, max_r_{25.0f};
  bool   enable_median3_{true};
  bool   downsample_{false};
  int    ds_stride_{2};
  bool   quality_gate_{true};
  float  roc_thr_{0.30f};
  double roc_bad_frac_{0.25};
  double min_valid_frac_{0.70};
  int    n_bad_{2}, n_good_{3};

  // Quality gate state
  std::vector<float> prev_ranges_;   // sanitised ranges from the previous scan
  int  bad_streak_{0};               // consecutive bad scans seen
  int  good_streak_{0};              // consecutive clean scans seen (suppressed)
  bool suppressed_{false};           // true while output is being held back
  int  n_suppressed_{0};             // scans suppressed in this gate event

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarScanFilter>());
  rclcpp::shutdown();
  return 0;
}
