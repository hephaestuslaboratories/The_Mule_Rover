// arduino_encoder_twist_bridge.cpp
//
// C++ port of rover_bringup/arduino_encoder_twist_bridge.py.  Same serial
// protocol, same parameters, same maths, same topics.
//
// Bi-directional bridge for the differential-drive rover:
//
//   ESP32 -> ROS
//     ENC <seq> <left> <right>            (~10 Hz, encoder counts)
//     IMU <seq> <qw qx qy qz gx gy gz ax ay az>   (~100 Hz)
//   published as:
//     /wheel/odom_twist  (nav_msgs/Odometry) — twist only, pose left at zeros
//     /imu/data          (sensor_msgs/Imu)
//     /wheel/left_velocity, /wheel/right_velocity (std_msgs/Float32)
//
//   ROS -> ESP32
//     subscribes /cmd_vel (geometry_msgs/Twist)
//     writes     "CMD <v> <w>\n"  with 3 decimals
//
// The one thing to understand before editing: the serial reader runs on its own
// thread (rx_loop) and only ever writes the small scalar state below; the ROS
// timers publish from that state.  Writes to the port are serialised with
// tx_mutex_ so a CMD never interleaves with another CMD.

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <mutex>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace
{
// Map an integer baud rate to the termios constant.  Only the rates this rover
// has ever used are listed; anything else falls back to B115200.
speed_t baud_constant(int baud)
{
  switch (baud) {
    case 9600:    return B9600;
    case 19200:   return B19200;
    case 38400:   return B38400;
    case 57600:   return B57600;
    case 115200:  return B115200;
    case 230400:  return B230400;
    case 460800:  return B460800;
    case 921600:  return B921600;
    default:      return B115200;
  }
}

// Equivalent of the Python `re.findall(r'[-+]?\d+', line)` — every integer
// token in the line, in order.  Used for ENC lines.
void extract_ints(const std::string & s, std::vector<long long> & out)
{
  out.clear();
  const size_t n = s.size();
  size_t i = 0;
  while (i < n) {
    const char c = s[i];
    const bool sign = (c == '-' || c == '+');
    const size_t digit_start = sign ? i + 1 : i;
    if (digit_start < n && std::isdigit(static_cast<unsigned char>(s[digit_start]))) {
      size_t j = digit_start;
      while (j < n && std::isdigit(static_cast<unsigned char>(s[j]))) {
        ++j;
      }
      out.push_back(std::strtoll(s.substr(i, j - i).c_str(), nullptr, 10));
      i = j;
    } else {
      ++i;
    }
  }
}

// Equivalent of `re.findall(r'[-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?', line)` —
// every signed decimal token, in order.  Used for IMU lines.
void extract_floats(const std::string & s, std::vector<double> & out)
{
  out.clear();
  const size_t n = s.size();
  size_t i = 0;
  while (i < n) {
    const char c = s[i];
    const bool sign = (c == '-' || c == '+');
    size_t j = sign ? i + 1 : i;
    if (j >= n || !std::isdigit(static_cast<unsigned char>(s[j]))) {
      ++i;
      continue;
    }
    while (j < n && std::isdigit(static_cast<unsigned char>(s[j]))) {
      ++j;
    }
    if (j + 1 < n && s[j] == '.' && std::isdigit(static_cast<unsigned char>(s[j + 1]))) {
      ++j;
      while (j < n && std::isdigit(static_cast<unsigned char>(s[j]))) {
        ++j;
      }
    }
    if (j < n && (s[j] == 'e' || s[j] == 'E')) {
      size_t k = j + 1;
      if (k < n && (s[k] == '-' || s[k] == '+')) {
        ++k;
      }
      if (k < n && std::isdigit(static_cast<unsigned char>(s[k]))) {
        j = k;
        while (j < n && std::isdigit(static_cast<unsigned char>(s[j]))) {
          ++j;
        }
      }
    }
    out.push_back(std::strtod(s.substr(i, j - i).c_str(), nullptr));
    i = j;
  }
}

inline std::string strip(const std::string & s)
{
  const size_t a = s.find_first_not_of(" \t\r\n");
  if (a == std::string::npos) {
    return "";
  }
  const size_t b = s.find_last_not_of(" \t\r\n");
  return s.substr(a, b - a + 1);
}
}  // namespace

class ArduinoEncoderTwistBridge : public rclcpp::Node
{
public:
  ArduinoEncoderTwistBridge()
  : Node("arduino_encoder_twist_bridge")
  {
    // ── Parameters ───────────────────────────────────────────────────────────
    port_ = this->declare_parameter<std::string>("port", "/dev/ttyACM0");
    baud_ = this->declare_parameter<int>("baud", 115200);

    odom_topic_    = this->declare_parameter<std::string>("odom_topic", "/wheel/odom_twist");
    cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    odom_frame_    = this->declare_parameter<std::string>("odom_frame", "odom");
    base_frame_    = this->declare_parameter<std::string>("base_frame", "base_link");

    // Robot geometry.  wheel_diameter_m / wheel_base_m MUST match
    // WHEEL_DIAMETER_M / WHEEL_BASE_M in the ESP32-S3 firmware or encoder
    // odometry will systematically over/under-report.
    cpr_        = declare_num("counts_per_rev", 267.0);
    wheel_d_    = declare_num("wheel_diameter_m", 0.120);
    wheel_base_ = declare_num("wheel_base_m", 0.37);

    publish_hz_ = declare_num("publish_hz", 50.0);
    max_dt_     = declare_num("max_dt_s", 0.2);
    timeout_s_  = declare_num("timeout_s", 0.05);

    // Safety: stop if no cmd_vel arrives.  <=0 disables the watchdog.
    cmd_timeout_s_ = declare_num("cmd_timeout_s", 0.8);

    // Motor asymmetry scaling
    left_scale_  = declare_num("left_scale", 1.00);
    right_scale_ = declare_num("right_scale", 1.00);

    // Odometry covariance tuning (the EKF needs realistic values to trust it)
    odom_cov_vx_   = declare_num("odom_twist_vx_cov", 0.01);
    odom_cov_vyaw_ = declare_num("odom_twist_vyaw_cov", 0.02);

    // ── Velocity-gated vyaw covariance (skid-steer slip compensation) ───────
    // Skid-steer wheel-derived ω = (vR - vL)/wheelbase is accurate at low turn
    // rates (wheels roll cleanly) but biased at high rates (wheels scrub
    // sideways).  Instead of globally disabling wheel vyaw fusion — which
    // deprives the EKF of a useful measurement during slow/straight motion — we
    // inflate the covariance dynamically based on |cmd_w|.
    //   |cmd_w| <  vyaw_fusion_gate  → nominal cov (trusted)
    //   |cmd_w| >= vyaw_fusion_gate  → inflated cov (effectively ignored)
    vyaw_fusion_gate_       = declare_num("vyaw_fusion_gate", 0.15);
    odom_cov_vyaw_inflated_ = declare_num("vyaw_cov_inflated", 100.0);

    cmd_w_scale_ = declare_num("cmd_w_scale", 0.85);
    fwd_w_trim_  = declare_num("fwd_w_trim", -0.00);
    rev_w_trim_  = declare_num("rev_w_trim", -0.015);
    w_trim_gate_ = declare_num("w_trim_gate", 0.05);
    v_min_for_trim_ = declare_num("v_min_for_trim", 0.05);
    cmd_deadman_enabled_ = this->declare_parameter<bool>("cmd_deadman_enabled", true);

    spin_v_gate_    = declare_num("spin_v_gate", 0.05);
    spin_w_gate_    = declare_num("spin_w_gate", 0.20);
    spin_yaw_scale_ = declare_num("spin_yaw_scale", 1.0);

    yaw_sign_ = declare_num("yaw_sign", -1.0);

    // Flip the linear-velocity frame 180° so +x drives the DRIVE-WHEEL end
    // first (casters TRAIL) instead of the caster/LiDAR end (casters pushed).
    // Applied symmetrically to the outgoing CMD and the published
    // /wheel/odom_twist.linear.x so command and odometry stay in one frame
    // (else the EKF sees "commanded forward, odom says backward").  Angular is
    // untouched — yaw about z is invariant under this 180° flip.
    const bool invert_linear = this->declare_parameter<bool>("invert_linear", true);
    linear_sign_ = invert_linear ? -1.0 : 1.0;

    // ── BNO085 IMU pass-through ─────────────────────────────────────────────
    // The firmware emits "IMU seq qw qx qy qz gx gy gz ax ay az" lines at
    // ~100 Hz on the same serial port as ENC.  This bridge parses them and
    // republishes as sensor_msgs/Imu on /imu/data.
    //
    // AXIS REMAP: if the BNO085 is physically mounted with axes that don't match
    // base_link (X-forward, Y-left, Z-up), set the per-axis source + sign here
    // so output is in the rover frame.  Identity by default.
    imu_topic_name_ = this->declare_parameter<std::string>("imu_topic", "/imu/data");
    imu_frame_      = this->declare_parameter<std::string>("imu_frame", "imu_link");
    publish_imu_    = this->declare_parameter<bool>("publish_imu", true);
    axis_x_src_ = read_axis_src("axis_x_src", "x");
    axis_y_src_ = read_axis_src("axis_y_src", "y");
    axis_z_src_ = read_axis_src("axis_z_src", "z");
    axis_x_sgn_ = static_cast<double>(this->declare_parameter<int>("axis_x_sgn", 1));
    axis_y_sgn_ = static_cast<double>(this->declare_parameter<int>("axis_y_sgn", 1));
    axis_z_sgn_ = static_cast<double>(this->declare_parameter<int>("axis_z_sgn", 1));
    imu_orient_cov_ = declare_num("imu_orient_cov", 0.01);
    imu_gyro_cov_   = declare_num("imu_gyro_cov", 0.0003);
    imu_accel_cov_  = declare_num("imu_accel_cov", 0.04);

    // ── Flight-recorder diagnostics (added 2026-08-16) ──────────────────────
    // Two extra topics, nothing else changes:
    //   /rover/cmd_sent    the v,w ACTUALLY written to serial, after yaw_sign,
    //                      cmd_w_scale and the trims.  Without this you cannot
    //                      tell whether a commanded-vs-achieved discrepancy was
    //                      introduced in ROS, in this bridge, or in firmware.
    //   /rover/link_status ESP32 reboots, ENC sequence gaps, link up/down, and
    //                      the firmware diagnostic lines (IMU_STALL,
    //                      WATCHDOG_TRIGGERED, boot banner) that this bridge
    //                      used to silently discard.
    publish_diagnostics_ = this->declare_parameter<bool>("publish_diagnostics", true);
    link_timeout_s_      = declare_num("link_timeout_s", 2.0);
    // High-rate firmware debug lines (PWM/YAWDBG/CMD_ECHO) are suppressed by
    // default; they only appear when the firmware's DEBUG_SERIAL is true and
    // would otherwise flood the topic.
    publish_debug_lines_ = this->declare_parameter<bool>("publish_debug_lines", false);

    // ── Guards against a stalled reader turning into a pose teleport ─────────
    // Encoder deltas are timed by ARRIVAL, so if a backlog of lines is read in
    // one burst the elapsed time collapses while the encoder delta still covers
    // real driving — distance/tiny_dt produces hundreds of m/s.  On 2026-08-17
    // that put -1766 rad/s into the EKF and moved the rover 54.9 m in a single
    // update.  Two independent floors now stop that:
    //   min_dt_s            drop samples closer together than half the
    //                       firmware's 100 ms report period (re-baseline instead)
    //   max_wheel_speed_mps refuse any sample implying a wheel speed the
    //                       drivetrain cannot reach (firmware caps at 1.25 m/s)
    min_dt_s_          = declare_num("min_dt_s", 0.02);
    max_wheel_speed_   = declare_num("max_wheel_speed_mps", 2.0);
    // A write to the serial port should complete in microseconds.  If one takes
    // longer than this the TX path is backing up, which is what starves the
    // reader.  Logged so the condition is visible instead of inferred.
    slow_write_ms_     = declare_num("slow_write_warn_ms", 100.0);
    // Reader instrumentation.  When the data stops there are only two
    // possibilities and they need completely different repairs:
    //   READ_IDLE     the kernel has nothing for us -> the bytes are not
    //                 arriving (cable / USB / ESP32 side)
    //   SLOW_PROCESS  bytes ARE arriving but this thread is stuck handing them
    //                 to ROS -> the problem is on the Jetson, in us
    // Without this the two are indistinguishable from the outside.
    reader_warn_ms_    = declare_num("reader_warn_ms", 250.0);

    wheel_circ_ = M_PI * wheel_d_;

    // ── ROS publishers / subscribers ────────────────────────────────────────
    pub_    = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 20);
    sub_    = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, 10, std::bind(&ArduinoEncoderTwistBridge::cmd_vel_cb, this, _1));
    pub_vl_ = this->create_publisher<std_msgs::msg::Float32>("/wheel/left_velocity", 10);
    pub_vr_ = this->create_publisher<std_msgs::msg::Float32>("/wheel/right_velocity", 10);
    if (publish_imu_) {
      pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_name_, 50);
    }
    if (publish_diagnostics_) {
      pub_cmd_sent_ = this->create_publisher<geometry_msgs::msg::Twist>("/rover/cmd_sent", 20);
      // Depth 50 so a burst of firmware lines at boot is not dropped.
      pub_link_ = this->create_publisher<std_msgs::msg::String>("/rover/link_status", 50);
    }

    last_time_     = this->get_clock()->now();
    last_cmd_time_ = this->get_clock()->now();
    last_stop_send_ = this->get_clock()->now();

    // Start serial + reader thread + timers
    open_serial();
    rx_thread_ = std::thread(&ArduinoEncoderTwistBridge::rx_loop, this);

    publish_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(publish_hz_, 1.0)),
      std::bind(&ArduinoEncoderTwistBridge::publish_tick, this));
    watchdog_timer_ = this->create_wall_timer(
      50ms, std::bind(&ArduinoEncoderTwistBridge::cmd_watchdog, this));   // 20 Hz
    if (publish_diagnostics_) {
      link_timer_ = this->create_wall_timer(
        1s, std::bind(&ArduinoEncoderTwistBridge::link_tick, this));      // 1 Hz
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Serial %s@%d | SUB %s -> 'CMD v w' | PUB %s ~%.0f Hz",
      port_.c_str(), baud_, cmd_vel_topic_.c_str(), odom_topic_.c_str(), publish_hz_);
  }

  ~ArduinoEncoderTwistBridge() override
  {
    stop_.store(true);
    if (rx_thread_.joinable()) {
      rx_thread_.join();
    }
    close_serial();
  }

private:
  // ── Parameter helper ───────────────────────────────────────────────────────
  // A launch override routed through the temp YAML params file can arrive as a
  // bool: YAML 1.1 parses bare y/yes/on -> true and n/no/off -> false (the
  // "Norway problem").  Map those back to their intended letters so the remap
  // keeps working regardless of how the value survived serialization.
  // Accept EITHER an integer or a double from YAML for numeric parameters.
  //
  // rclcpp is strict where rclpy was not: declaring a double and receiving
  // `267` (an int) throws InvalidParameterTypeException from the constructor,
  // which kills the process before it ever spins — the node then does not even
  // appear in `ros2 node list`.  The Python node coerced everything with
  // float(), so rover_bringup.launch.py has always passed
  // `counts_per_rev: 267` as an int.  That mismatch took this node down on its
  // first C++ run.  Coercing here keeps the launch file untouched and makes
  // every numeric parameter immune to how it happens to be written.
  double declare_num(const std::string & name, double def)
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.dynamic_typing = true;
    const rclcpp::ParameterValue v =
      this->declare_parameter(name, rclcpp::ParameterValue(def), desc);
    switch (v.get_type()) {
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        return static_cast<double>(v.get<int64_t>());
      case rclcpp::ParameterType::PARAMETER_DOUBLE:
        return v.get<double>();
      default:
        return def;
    }
  }

  std::string read_axis_src(const std::string & name, const std::string & def)
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.dynamic_typing = true;
    const rclcpp::ParameterValue v =
      this->declare_parameter(name, rclcpp::ParameterValue(def), desc);
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      return v.get<bool>() ? "y" : "n";
    }
    if (v.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
      return def;
    }
    std::string s = v.get<std::string>();
    for (auto & c : s) {
      c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return s;
  }

  // ── Serial ─────────────────────────────────────────────────────────────────

  void open_serial()
  {
    close_serial();
    // The port is configured OUTSIDE the tx lock: the 2.5 s boot wait below
    // would otherwise block every /cmd_vel write for the whole reconnect.
    const int fd = ::open(port_.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to open serial %s: %s",
        port_.c_str(), std::strerror(errno));
      return;
    }
    struct termios tio {};
    if (tcgetattr(fd, &tio) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcgetattr failed on %s", port_.c_str());
      ::close(fd);
      return;
    }
    cfmakeraw(&tio);
    const speed_t sp = baud_constant(baud_);
    cfsetispeed(&tio, sp);
    cfsetospeed(&tio, sp);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~PARENB;
    // Blocking read with a timeout, mirroring pyserial's timeout= behaviour.
    // VTIME is in tenths of a second; clamp to at least 1 so the reader thread
    // always wakes up and can observe stop_.
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = static_cast<cc_t>(std::max(1.0, timeout_s_ * 10.0));
    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcsetattr failed on %s", port_.c_str());
      ::close(fd);
      return;
    }
    // Wait for the ESP32 to finish booting after the DTR reset, then publish
    // the port and send an explicit stop — the first command the firmware
    // receives, and what its "Jetson ready" detection keys off.
    std::this_thread::sleep_for(2500ms);

    // Flush AFTER the wait, not before.  During those 2.5 s the ESP32 is
    // already streaming ~6 KB/s into a ~4 KB kernel buffer with nobody reading
    // it, so it overflows and silently discards ~18 ENC lines — the startup
    // ENC_GAP that appeared in every single recording.  Flushing here means the
    // reader starts from a clean buffer and the gap counter only ever reports
    // real losses.
    tcflush(fd, TCIOFLUSH);
    fd_.store(fd);
    send_cmd(0.0, 0.0);
    RCLCPP_INFO(this->get_logger(), "Serial opened on %s, stop command sent.", port_.c_str());
  }

  // Only the RX thread — and the destructor, after joining it — ever closes the
  // port, so no reader is ever mid-read on a descriptor being closed.
  void close_serial()
  {
    const int fd = fd_.exchange(-1);
    if (fd >= 0) {
      ::close(fd);
    }
  }

  // Send a command in EXACTLY the format the firmware parses:  "CMD <v> <w>\n"
  // with three decimals.
  //
  // 2026-08-17 — THE FIX.  This used to call ::tcdrain(), which BLOCKS until the
  // UART has physically transmitted everything, while holding tx_mutex_.  The RX
  // thread took that same mutex on every loop iteration, so a backed-up TX
  // stopped the reader outright; the kernel's ~4 KB input buffer then overflowed
  // within a second and 47 s of sensor data was discarded.  The Python bridge
  // avoided this deliberately ("Lock only for writes. Reads run without lock..."),
  // and the port lost that property.  tcdrain is gone and the lock now covers
  // only the write syscall itself.
  void send_cmd(double v, double w)
  {
    const int fd = fd_.load();
    if (fd < 0) {
      return;
    }
    char buf[64];
    const int len = std::snprintf(buf, sizeof(buf), "CMD %.3f %.3f\n", v, w);
    if (len <= 0) {
      return;
    }

    const auto t0 = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(tx_mutex_);
      ssize_t written = 0;
      while (written < len) {
        const ssize_t k = ::write(fd, buf + written, static_cast<size_t>(len - written));
        if (k < 0) {
          if (errno == EINTR) {
            continue;
          }
          RCLCPP_WARN(
            this->get_logger(), "Failed to write CMD to serial: %s", std::strerror(errno));
          return;
        }
        written += k;
      }
    }
    // A write to a tty should take microseconds.  Anything slower means the TX
    // path is backing up — the precursor to the failure above, now visible
    // instead of inferred.
    const double ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();
    if (ms > slow_write_ms_) {
      RCLCPP_WARN(
        this->get_logger(),
        "[bridge] serial write took %.0f ms — TX path backing up", ms);
      link_event("SLOW_WRITE ms=" + std::to_string(static_cast<int>(ms)));
    }

    // Record exactly what went down the wire.  This is the ONLY place the
    // post-scale, post-trim command is visible — /cmd_vel is upstream of all of
    // it and the firmware never echoes it back at DEBUG_SERIAL=false.
    if (pub_cmd_sent_) {
      geometry_msgs::msg::Twist sent;
      sent.linear.x = v;
      sent.angular.z = w;
      pub_cmd_sent_->publish(sent);
    }
  }

  // ── Diagnostics helper ─────────────────────────────────────────────────────
  // Publishes one line to /rover/link_status.  Thread-safe: called from both the
  // RX thread and ROS timers, and rclcpp publishers are thread-safe.
  void link_event(const std::string & text)
  {
    if (!pub_link_) {
      return;
    }
    std_msgs::msg::String msg;
    msg.data = text;
    pub_link_->publish(msg);
  }

  // Republish a firmware diagnostic line and escalate the ones that matter.
  void forward_firmware_line(const std::string & line)
  {
    if (!pub_link_) {
      return;
    }
    // High-rate debug chatter only exists when the firmware's DEBUG_SERIAL is
    // true; suppressed unless explicitly asked for, so it cannot flood the bag.
    if (!publish_debug_lines_) {
      if (line.rfind("PWM", 0) == 0 || line.rfind("YAWDBG", 0) == 0 ||
          line.rfind("CMD_ECHO", 0) == 0)
      {
        return;
      }
    }

    link_event("FW " + line);

    // The three that are never routine.
    if (line.find("WATCHDOG_TRIGGERED") != std::string::npos) {
      // Mid-mission this means the JETSON stalled >500 ms without rebooting.
      // At startup it is normal (motors stay disabled until the first CMD).
      RCLCPP_WARN(
        this->get_logger(),
        "[bridge] firmware WATCHDOG_TRIGGERED — no CMD reached the ESP32 for "
        ">500 ms. Mid-mission this means the Jetson stalled.");
    } else if (line.find("IMU_STALL") != std::string::npos ||
               line.find("IMU_FAIL") != std::string::npos)
    {
      RCLCPP_ERROR(this->get_logger(), "[bridge] firmware reports: %s", line.c_str());
    } else if (line.find("VERSION READY") != std::string::npos) {
      // Boot banner.  Independent of the ENC-seq detector: catches a reboot
      // even if the encoder stream had already gone quiet.
      RCLCPP_ERROR(
        this->get_logger(),
        "[bridge] ESP32 BOOT BANNER mid-session — the MCU restarted: %s",
        line.c_str());
      link_event("ESP32_BOOT_BANNER");
    }
  }

  // 1 Hz: link liveness + a health time series for offline analysis.
  void link_tick()
  {
    const int64_t now_ns = this->get_clock()->now().nanoseconds();
    const int64_t last_ns = last_enc_rx_ns_.load();
    const bool alive =
      (last_ns != 0) && ((now_ns - last_ns) * 1e-9 < link_timeout_s_);

    if (alive != link_alive_) {
      link_alive_ = alive;
      if (alive) {
        RCLCPP_INFO(this->get_logger(), "[bridge] ENC link UP");
        link_event("LINK_UP");
      } else {
        RCLCPP_ERROR(
          this->get_logger(),
          "[bridge] ENC link DEAD — no encoder line for >%.1fs", link_timeout_s_);
        link_event("LINK_DEAD timeout_s=" + std::to_string(link_timeout_s_));
        // Tell the reader to discard whatever accumulates while we are down.
        flush_input_.store(true);
      }
    }

    // Per-second counters.  A rising gap rate is the early warning that the
    // USB link is degrading; enc/imu rates catch a silent sensor death.
    const int enc = enc_count_.exchange(0);
    const int imu = imu_count_.exchange(0);
    const int gaps = gap_count_.exchange(0);
    link_event(
      "STATS enc=" + std::to_string(enc) +
      " imu=" + std::to_string(imu) +
      " gaps=" + std::to_string(gaps) +
      " reboots=" + std::to_string(reboot_count_.load()) +
      " link=" + (alive ? "up" : "down"));
  }


  // ── ROS -> ESP32 ───────────────────────────────────────────────────────────

  void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_time_ = this->get_clock()->now();

    const double v = msg->linear.x;
    double w_cmd = msg->angular.z;

    // Record raw commanded ω for the velocity-gated vyaw covariance logic in
    // publish_tick.  Must be BEFORE trims/scale so the gate reacts to the
    // planner's actual intent, not a post-processed value.
    last_cmd_w_.store(w_cmd);

    // Trims apply to the command only.
    if (std::fabs(w_cmd) < w_trim_gate_) {
      if (v > v_min_for_trim_) {
        w_cmd += fwd_w_trim_;
      } else if (v < -v_min_for_trim_) {
        w_cmd += rev_w_trim_;
      }
    }

    w_cmd = yaw_sign_ * w_cmd;

    // Scale ONLY for the firmware's command interpretation.
    w_cmd *= cmd_w_scale_;

    send_cmd(linear_sign_ * v, w_cmd);
  }

  // If /cmd_vel goes stale, repeatedly send stop (10 Hz) until fresh commands
  // return.
  void cmd_watchdog()
  {
    if (!cmd_deadman_enabled_ || cmd_timeout_s_ <= 0.0) {
      return;
    }
    if (fd_.load() < 0) {
      return;
    }

    const rclcpp::Time now = this->get_clock()->now();
    const double age = (now - last_cmd_time_).seconds();

    if (age > cmd_timeout_s_) {
      // First detection: stop immediately.
      if (!timed_out_) {
        send_cmd(0.0, 0.0);
        last_stop_send_ = now;
        timed_out_ = true;
        return;
      }
      // While timed out: keep re-sending stop at 10 Hz.
      if ((now - last_stop_send_).seconds() >= 0.1) {
        send_cmd(0.0, 0.0);
        last_stop_send_ = now;
      }
    } else {
      timed_out_ = false;
    }
  }

  // ── ESP32 -> ROS ───────────────────────────────────────────────────────────

  // Accepts "ENC L R" or "ENC seq L R" (taking the last three integers).
  // Returns false if the line is not a usable ENC line.
  bool parse_enc_line(const std::string & line, long long & lc, long long & rc)
  {
    if (line.find("ENC") == std::string::npos) {
      return false;
    }
    extract_ints(line, int_scratch_);
    if (int_scratch_.size() == 2) {
      lc = int_scratch_[0];
      rc = int_scratch_[1];
      return true;
    }
    if (int_scratch_.size() >= 3) {
      const long long seq = int_scratch_[int_scratch_.size() - 3];
      lc = int_scratch_[int_scratch_.size() - 2];
      rc = int_scratch_[int_scratch_.size() - 1];
      // ESP32 reboot detector: seq is a monotonically increasing uint32 reset
      // to 0 by any firmware restart (brownout, USB glitch, reflash).  On a
      // reset: (a) log it loudly so silent mid-session reboots stop being a
      // mystery, (b) drop the count baseline so the huge counter discontinuity
      // cannot be interpreted as a multi-m/s velocity spike and fed to the EKF.
      if (have_enc_seq_ && seq < last_enc_seq_) {
        RCLCPP_ERROR(
          this->get_logger(),
          "[bridge] ENC seq went backwards (%lld -> %lld) — ESP32 REBOOTED "
          "mid-session (brownout/USB glitch?). Resetting encoder baseline; "
          "check dmesg for cdc_acm events.",
          last_enc_seq_, seq);
        have_last_counts_ = false;     // re-baseline, no spike
        have_twist_.store(false);
        reboot_count_.fetch_add(1);
        link_event(
          "ESP32_REBOOT prev_seq=" + std::to_string(last_enc_seq_) +
          " new_seq=" + std::to_string(seq));
      } else if (have_enc_seq_ && seq > last_enc_seq_ + 1) {
        // Gap = ENC lines that never arrived: dropped serial bytes or USB
        // congestion.  Not fatal — the counts are absolute so odometry stays
        // correct — but a rising gap rate is the early warning for a link that
        // is about to die, and it is invisible without this.
        const long long missed = seq - last_enc_seq_ - 1;
        gap_count_.fetch_add(static_cast<int>(missed));
        link_event(
          "ENC_GAP missed=" + std::to_string(missed) +
          " prev_seq=" + std::to_string(last_enc_seq_) +
          " new_seq=" + std::to_string(seq));
      }
      last_enc_seq_ = seq;
      have_enc_seq_ = true;
      return true;
    }
    return false;
  }

  // Keep the RX-thread-private timestamp and the atomic copy the publish timer
  // reads in lockstep.
  void set_last_time(const rclcpp::Time & t)
  {
    last_time_ = t;
    last_sample_ns_.store(t.nanoseconds());
  }

  double remap_axis(const std::string & src, double sgn, double vx, double vy, double vz) const
  {
    double v = 0.0;
    if (src == "x") {
      v = vx;
    } else if (src == "y") {
      v = vy;
    } else if (src == "z") {
      v = vz;
    }
    return sgn * v;
  }

  void publish_imu_line(const std::vector<double> & vals)
  {
    if (!pub_imu_) {
      return;
    }
    // vals = [seq, qw, qx, qy, qz, gx, gy, gz, ax, ay, az]
    const double q_w = vals[1], q_x = vals[2], q_y = vals[3], q_z = vals[4];
    const double g_x = vals[5], g_y = vals[6], g_z = vals[7];
    const double a_x = vals[8], a_y = vals[9], a_z = vals[10];

    auto msg = std::make_unique<sensor_msgs::msg::Imu>();
    msg->header.stamp = this->get_clock()->now();
    msg->header.frame_id = imu_frame_;

    // Quaternion: for the default identity remap we pass it through.  A full
    // rotated remap would require composing it with the chip→rover rotation;
    // downstream consumers that need it can apply their own transform.
    msg->orientation.w = q_w;
    msg->orientation.x = q_x;
    msg->orientation.y = q_y;
    msg->orientation.z = q_z;

    msg->angular_velocity.x = remap_axis(axis_x_src_, axis_x_sgn_, g_x, g_y, g_z);
    msg->angular_velocity.y = remap_axis(axis_y_src_, axis_y_sgn_, g_x, g_y, g_z);
    msg->angular_velocity.z = remap_axis(axis_z_src_, axis_z_sgn_, g_x, g_y, g_z);

    msg->linear_acceleration.x = remap_axis(axis_x_src_, axis_x_sgn_, a_x, a_y, a_z);
    msg->linear_acceleration.y = remap_axis(axis_y_src_, axis_y_sgn_, a_x, a_y, a_z);
    msg->linear_acceleration.z = remap_axis(axis_z_src_, axis_z_sgn_, a_x, a_y, a_z);

    const double oc = imu_orient_cov_, gc = imu_gyro_cov_, ac = imu_accel_cov_;
    msg->orientation_covariance         = {oc, 0.0, 0.0, 0.0, oc, 0.0, 0.0, 0.0, oc};
    msg->angular_velocity_covariance    = {gc, 0.0, 0.0, 0.0, gc, 0.0, 0.0, 0.0, gc};
    msg->linear_acceleration_covariance = {ac, 0.0, 0.0, 0.0, ac, 0.0, 0.0, 0.0, ac};

    pub_imu_->publish(std::move(msg));
  }

  // Serial reader thread.  Reassembles newline-terminated lines from whatever
  // read() hands back.
  void rx_loop()
  {
    double backoff = 0.5;   // seconds; doubles per consecutive failure, caps 16
    std::string pending;
    std::vector<char> chunk(512);
    // Each empty read costs ~VTIME (100 ms), so 10 in a row is ~1 s of silence.
    int empty_reads = 0;
    constexpr int kEmptyReadsToReport = 10;

    while (!stop_.load()) {
      // NOTE: this loop must NEVER take tx_mutex_.  Coupling the reader to the
      // writer's lock is exactly what caused the 47 s data loss on 2026-08-17.
      int fd = fd_.load();
      if (fd < 0) {
        open_serial();
        fd = fd_.load();
        if (fd < 0) {
          // Port still unavailable — wait before hammering the OS again.
          std::this_thread::sleep_for(std::chrono::duration<double>(std::min(backoff, 16.0)));
          backoff = std::min(backoff * 2.0, 16.0);
        } else {
          backoff = 0.5;    // successful open — reset backoff
        }
        continue;
      }
      backoff = 0.5;        // port healthy — keep backoff reset

      // The link was declared dead and has now come back.  Whatever is sitting
      // in the kernel buffer is stale by definition — reading it would feed the
      // EKF a burst of samples timestamped milliseconds apart that actually
      // span tens of seconds.  Discard it and re-baseline the encoder counts.
      if (flush_input_.exchange(false)) {
        ::tcflush(fd, TCIFLUSH);
        pending.clear();
        have_last_counts_ = false;
        have_twist_.store(false);
        RCLCPP_WARN(
          this->get_logger(),
          "[bridge] link recovered — flushed stale input, re-baselining encoders");
        link_event("INPUT_FLUSHED");
      }

      const ssize_t k = ::read(fd, chunk.data(), chunk.size());

      // ── Is data arriving at all? ───────────────────────────────────────────
      // VMIN=0/VTIME=1 means read() returns after ~100 ms even with nothing to
      // give, so an outage shows up as a run of empty reads rather than one
      // long block.  Counting them proves whether the kernel actually has
      // nothing for us — which separates "the bytes aren't arriving" from
      // "we're too busy to collect them".
      if (k <= 0) {
        ++empty_reads;
        if (empty_reads == kEmptyReadsToReport) {
          RCLCPP_WARN(
            this->get_logger(),
            "[bridge] no bytes from %s for ~1 s — the kernel has nothing to "
            "give us (data is NOT arriving)", port_.c_str());
          link_event("READ_IDLE reads=" + std::to_string(empty_reads));
        }
      } else if (empty_reads >= kEmptyReadsToReport) {
        RCLCPP_INFO(
          this->get_logger(), "[bridge] bytes resumed after %d empty reads",
          empty_reads);
        link_event("READ_RESUMED after_empty=" + std::to_string(empty_reads));
        empty_reads = 0;
      } else {
        empty_reads = 0;
      }

      if (k <= 0) {
        if (k < 0 && errno != EAGAIN && errno != EINTR) {
          // The port itself went away — USB re-enumeration, cable, or the MCU
          // dropping off the bus.  Distinct from "MCU rebooted but stayed
          // enumerated", which shows up as an ENC seq regression instead.
          RCLCPP_ERROR(
            this->get_logger(), "[bridge] serial read failed (%s) — reopening %s",
            std::strerror(errno), port_.c_str());
          link_event(std::string("PORT_LOST errno=") + std::strerror(errno));
          close_serial();   // the loop above will reopen it
          pending.clear();  // a partial line across a reconnect is garbage
        }
        continue;
      }
      pending.append(chunk.data(), static_cast<size_t>(k));

      // ── Are WE the bottleneck? ─────────────────────────────────────────────
      // handle_line() parses and publishes to ROS.  If a publish ever blocks
      // (slow subscriber, DDS back-pressure, a bag writer stalling on disk)
      // this is where the reader would get stuck — and while it is stuck the
      // kernel's ~4 KB input buffer overflows and data is lost.  Timing it
      // makes that visible instead of inferred.
      const auto p0 = std::chrono::steady_clock::now();
      size_t nl;
      while ((nl = pending.find('\n')) != std::string::npos) {
        std::string line = pending.substr(0, nl);
        pending.erase(0, nl + 1);
        handle_line(line);
      }
      const double proc_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - p0).count();
      if (proc_ms > reader_warn_ms_) {
        RCLCPP_WARN(
          this->get_logger(),
          "[bridge] parse/publish took %.0f ms — the READER is the bottleneck, "
          "not the link", proc_ms);
        link_event("SLOW_PROCESS ms=" + std::to_string(static_cast<int>(proc_ms)));
      }
      // Guard against a peer that never sends a newline.
      if (pending.size() > 4096) {
        pending.clear();
      }
    }
  }

  void handle_line(const std::string & raw)
  {
    const std::string line = strip(raw);
    if (line.empty()) {
      return;
    }

    // IMU lines arrive ~10x more often than ENC.  Try IMU first so the common
    // path is fast.
    if (line.rfind("IMU ", 0) == 0) {
      extract_floats(line, float_scratch_);
      // seq + 4 quat + 3 gyro + 3 accel = 11 numbers minimum
      if (float_scratch_.size() >= 11) {
        std::vector<double> vals(
          float_scratch_.end() - 11, float_scratch_.end());
        publish_imu_line(vals);
        imu_count_.fetch_add(1);
      }
      return;
    }

    long long lc = 0, rc = 0;
    if (!parse_enc_line(line, lc, rc)) {
      // Not ENC and not an IMU sample -> a firmware diagnostic line.  These
      // used to be discarded, which is why IMU_STALL and WATCHDOG_TRIGGERED
      // were invisible from ROS.  WATCHDOG_TRIGGERED arriving mid-mission is
      // the ONLY evidence that the Jetson stalled >500 ms without rebooting —
      // uptime can never reveal that.
      forward_firmware_line(line);
      return;
    }
    enc_count_.fetch_add(1);
    last_enc_rx_ns_.store(this->get_clock()->now().nanoseconds());

    const rclcpp::Time now = this->get_clock()->now();
    if (!have_last_counts_) {
      last_l_ = lc;
      last_r_ = rc;
      have_last_counts_ = true;
      set_last_time(now);
      return;
    }

    const double dt = (now - last_time_).seconds();
    if (dt <= 0.0 || dt > max_dt_ || dt < min_dt_s_) {
      // Stale, invalid, or IMPOSSIBLY CLOSE TOGETHER.  The min_dt_s floor is the
      // seatbelt: two ENC lines arriving milliseconds apart cannot represent a
      // real 100 ms firmware report interval, so they are a read backlog being
      // drained.  Dividing a real encoder delta by that tiny dt is what produced
      // -1766 rad/s.  Re-baseline instead of computing a velocity.
      last_l_ = lc;
      last_r_ = rc;
      set_last_time(now);
      return;
    }

    const double dl = static_cast<double>(lc - last_l_);
    const double dr = static_cast<double>(rc - last_r_);
    last_l_ = lc;
    last_r_ = rc;
    set_last_time(now);

    // counts -> wheel revolutions -> distance
    const double dist_l = (dl / cpr_) * wheel_circ_;
    const double dist_r = (dr / cpr_) * wheel_circ_;

    const double v_l = (dist_l / dt) * left_scale_;
    const double v_r = (dist_r / dt) * right_scale_;

    // Second seatbelt, independent of the first: refuse anything the drivetrain
    // physically cannot do (firmware's own ceiling is 1.25 m/s).  Publishing a
    // garbage twist to the EKF is unrecoverable — it teleports the pose and
    // takes localization with it — whereas dropping a sample costs 100 ms.
    if (std::fabs(v_l) > max_wheel_speed_ || std::fabs(v_r) > max_wheel_speed_) {
      RCLCPP_ERROR(
        this->get_logger(),
        "[bridge] REJECTED impossible wheel speeds (L=%.1f R=%.1f m/s, dt=%.4f s) "
        "— not publishing to the EKF; re-baselining.", v_l, v_r, dt);
      link_event(
        "VELOCITY_REJECTED vl=" + std::to_string(v_l) +
        " vr=" + std::to_string(v_r) + " dt=" + std::to_string(dt));
      have_last_counts_ = false;   // force a clean re-baseline
      // Also stop the publish timer re-emitting the last good twist under a
      // fresh timestamp — the EKF should see a gap, not stale data.
      have_twist_.store(false);
      return;
    }

    v_l_.store(v_l);
    v_r_.store(v_r);

    const double v = 0.5 * (v_r + v_l);
    double w = (v_l - v_r) / wheel_base_;

    // Spin-in-place correction (empirical)
    if (std::fabs(v) < spin_v_gate_ && std::fabs(w) > spin_w_gate_) {
      w *= spin_yaw_scale_;
    }

    v_.store(v);
    w_.store(w);
    have_twist_.store(true);
  }

  // ── Publish ────────────────────────────────────────────────────────────────

  void publish_tick()
  {
    if (!have_twist_.load()) {
      return;
    }

    // Stamp with the encoder sample time, NOT wall-clock at publish.
    // The sample time crosses threads, so it travels as an atomic nanosecond
    // count rather than an rclcpp::Time (which is not atomically readable).
    // `last_time_` is when the newest ENC line was received; using the timer's
    // now() added up to 20 ms of phantom latency on every published twist,
    // which the EKF then interpolated IMU samples to — mismatched fusion during
    // turns was a significant drift source.
    //
    // If the encoder stream has stalled (serial glitch, ESP32 reset), suppress
    // publishing rather than re-announcing stale velocities with a
    // fresh-looking stamp.  max_dt_s is the existing staleness budget.
    const rclcpp::Time sample_time(
      last_sample_ns_.load(), this->get_clock()->get_clock_type());
    const rclcpp::Time now = this->get_clock()->now();
    if ((now - sample_time).seconds() > max_dt_) {
      return;
    }

    // Skip if no new encoder sample has arrived since the last publish.
    // Firmware reports at 10 Hz → this timer (50 Hz) would otherwise emit 4
    // duplicate messages for every real update.  Duplicate timestamps are
    // rejected by robot_localization anyway; dropping them here keeps the bus
    // cleaner and makes /wheel/odom_twist's true rate legible in `ros2 topic hz`.
    const int64_t stamp_ns = sample_time.nanoseconds();
    if (stamp_ns == last_published_stamp_ns_) {
      return;
    }
    last_published_stamp_ns_ = stamp_ns;

    auto msg = std::make_unique<nav_msgs::msg::Odometry>();
    msg->header.stamp = sample_time;
    msg->header.frame_id = odom_frame_;
    msg->child_frame_id = base_frame_;

    // Pose left at zeros; the EKF publishes the fused pose.
    msg->pose.pose.orientation.w = 1.0;

    msg->twist.twist.linear.x = linear_sign_ * v_.load();
    msg->twist.twist.angular.z = w_.load();

    // ── Twist covariance ────────────────────────────────────────────────────
    // Row-major 6x6: [vx, vy, vz, vroll, vpitch, vyaw].  Only vx and vyaw are
    // used; the rest get large "unused" values.
    //
    // vyaw covariance is velocity-gated.  Skid-steer wheels don't slip at low
    // commanded ω — wheel vyaw is trustworthy and gets the nominal covariance.
    // At high commanded ω tyres scrub and the encoder differential over/under-
    // reports yaw rate by 20–40 %, so we publish an inflated covariance and the
    // EKF effectively ignores it.
    const double cov_vyaw_now =
      (std::fabs(last_cmd_w_.load()) >= vyaw_fusion_gate_)
      ? odom_cov_vyaw_inflated_
      : odom_cov_vyaw_;

    msg->twist.covariance = {
      odom_cov_vx_, 0.0, 0.0, 0.0, 0.0, 0.0,     // vx row
      0.0,          1.0, 0.0, 0.0, 0.0, 0.0,     // vy row (unused)
      0.0,          0.0, 1.0, 0.0, 0.0, 0.0,     // vz row (unused)
      0.0,          0.0, 0.0, 1.0, 0.0, 0.0,     // vroll row (unused)
      0.0,          0.0, 0.0, 0.0, 1.0, 0.0,     // vpitch row (unused)
      0.0,          0.0, 0.0, 0.0, 0.0, cov_vyaw_now,   // vyaw row (gated)
    };

    pub_->publish(std::move(msg));

    std_msgs::msg::Float32 m;
    m.data = static_cast<float>(v_l_.load());
    pub_vl_->publish(m);
    m.data = static_cast<float>(v_r_.load());
    pub_vr_->publish(m);
  }

  // ── Parameters ─────────────────────────────────────────────────────────────
  std::string port_, odom_topic_, cmd_vel_topic_, odom_frame_, base_frame_;
  int    baud_{115200};
  double cpr_{267.0}, wheel_d_{0.120}, wheel_base_{0.37}, wheel_circ_{0.0};
  double publish_hz_{50.0}, max_dt_{0.2}, timeout_s_{0.05}, cmd_timeout_s_{0.8};
  double left_scale_{1.0}, right_scale_{1.0};
  double odom_cov_vx_{0.01}, odom_cov_vyaw_{0.02};
  double vyaw_fusion_gate_{0.15}, odom_cov_vyaw_inflated_{100.0};
  double cmd_w_scale_{0.85}, fwd_w_trim_{0.0}, rev_w_trim_{-0.015};
  double w_trim_gate_{0.05}, v_min_for_trim_{0.05};
  bool   cmd_deadman_enabled_{true};
  double spin_v_gate_{0.05}, spin_w_gate_{0.20}, spin_yaw_scale_{1.0};
  double yaw_sign_{-1.0}, linear_sign_{-1.0};
  std::string imu_topic_name_, imu_frame_;
  bool   publish_imu_{true};
  bool   publish_diagnostics_{true};
  bool   publish_debug_lines_{false};
  double link_timeout_s_{2.0};
  double min_dt_s_{0.02};
  double max_wheel_speed_{2.0};
  double slow_write_ms_{100.0};
  double reader_warn_ms_{250.0};
  std::string axis_x_src_{"x"}, axis_y_src_{"y"}, axis_z_src_{"z"};
  double axis_x_sgn_{1.0}, axis_y_sgn_{1.0}, axis_z_sgn_{1.0};
  double imu_orient_cov_{0.01}, imu_gyro_cov_{0.0003}, imu_accel_cov_{0.04};

  // ── Serial state ───────────────────────────────────────────────────────────
  // Atomic so the RX thread can read it without ever touching tx_mutex_ — that
  // coupling is what let a slow write stall the reader.
  std::atomic<int> fd_{-1};
  std::mutex tx_mutex_;          // serialises writes ONLY, never the read path
  std::atomic<bool> flush_input_{false};   // set on link loss, actioned by RX
  std::thread rx_thread_;
  std::atomic<bool> stop_{false};

  // ── Shared state (written by the RX thread, read by the timers) ────────────
  std::atomic<double> v_{0.0}, w_{0.0}, v_l_{0.0}, v_r_{0.0};
  std::atomic<bool>   have_twist_{false};
  std::atomic<double> last_cmd_w_{0.0};
  std::atomic<int64_t> last_sample_ns_{0};   // stamp of the newest ENC sample

  // Link health (written by the RX thread, read by the 1 Hz link timer)
  std::atomic<int64_t> last_enc_rx_ns_{0};
  std::atomic<int> enc_count_{0};
  std::atomic<int> imu_count_{0};
  std::atomic<int> gap_count_{0};
  std::atomic<int> reboot_count_{0};
  bool link_alive_{false};   // timer-thread only

  // RX-thread-only state
  long long last_l_{0}, last_r_{0};
  bool have_last_counts_{false};
  long long last_enc_seq_{0};
  bool have_enc_seq_{false};
  std::vector<long long> int_scratch_;
  std::vector<double> float_scratch_;

  rclcpp::Time last_time_;
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_stop_send_;
  bool timed_out_{false};
  int64_t last_published_stamp_ns_{0};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_vl_, pub_vr_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_sent_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_link_;
  rclcpp::TimerBase::SharedPtr publish_timer_, watchdog_timer_, link_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArduinoEncoderTwistBridge>());
  rclcpp::shutdown();
  return 0;
}
