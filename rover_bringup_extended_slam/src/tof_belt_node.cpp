// ─────────────────────────────────────────────────────────────────────────────
// tof_belt_node.cpp — 6x VL53L1X ToF safety belt on the Jetson's I2C bus,
// behind a TCA9548A 1-to-8 mux.
//
// WHAT THIS IS FOR (SESSION_HANDOFF.md S14)
//   Obstacles BELOW the LiDAR plane.  It does NOT fix doorways — those are a
//   packing problem and the LiDAR already sees the frames perfectly.
//
// WHY THE JETSON AND NOT THE ESP32
//   Separate failure domains.  The ESP32 serial link dies for 29-47 s roughly
//   once per 45 min (S12.7) and low-obstacle detection is safety-critical.
//   The ESP32 link also drives the motors; nothing gets added to it.
//
// DESIGN RULES THIS NODE FOLLOWS
//   1. THE BELT IS ADDITIVE.  Its failure must never leave the rover worse off
//      than not having it.  A dead sensor is quarantined and the other five
//      keep publishing.  Bringup never blocks and never aborts on a missing
//      sensor: without this node the stack navigates on LiDAR alone, which is
//      exactly what it does today.
//   2. "I SEE NOTHING" IS A MEASUREMENT.  A range at max_range is what tells
//      RangeSensorLayer to CLEAR the cone.  Staying silent instead would let
//      an old mark sit there forever, because the LiDAR cannot clear what a
//      ToF cone marks (a sock on the floor gets marked and no LiDAR ray will
//      ever raytrace it away).  So a "no target" reading is published AS
//      max_range, deliberately.
//   3. NEVER PUBLISH A GUESS.  A noisy or failed reading is dropped, not
//      rounded into a distance.  A blind sensor reporting "all clear" is worse
//      than a blind sensor reporting nothing.
//   4. ALL GEOMETRY LIVES IN config/tof_belt.yaml AND rover.urdf.xacro.
//      Nothing about where a sensor is mounted is hard-coded here.
//
// SENSOR DRIVER
//   Minimal port of ST's VL53L1X ULD (the same sequence the Arduino/Adafruit
//   drivers use): write the 91-byte default config at 0x2D..0x87, set distance
//   mode + timing budget, then free-run in continuous mode.  We only ever read
//   two registers per cycle, so a channel switch plus a read is ~1 ms and all
//   six poll comfortably at 10 Hz.
//
//   Cross-check against tools/tof_probe.py --live (which uses the vetted
//   Adafruit driver) if readings ever look wrong — same sensor, same mount,
//   two independent drivers.
//
// I2C LAYOUT
//   /dev/i2c-7 (40-pin header pins 3/5 on an Orin Nano) -> TCA9548A at 0x70
//   -> one sensor per channel, every one at the factory address 0x29.  The
//   mux is the ONLY thing that tells them apart, so a channel number is a
//   sensor's identity.
// ─────────────────────────────────────────────────────────────────────────────

#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{

// ── VL53L1X registers (ST ULD names) ────────────────────────────────────────
constexpr uint8_t  SENSOR_ADDR        = 0x29;   // factory default, never changed
constexpr uint16_t SOFT_RESET                       = 0x0000;
constexpr uint16_t VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND = 0x0008;
constexpr uint16_t GPIO_HV_MUX__CTRL                = 0x0030;
constexpr uint16_t GPIO__TIO_HV_STATUS              = 0x0031;
constexpr uint16_t PHASECAL_CONFIG__TIMEOUT_MACROP  = 0x004B;
constexpr uint16_t RANGE_CONFIG__TIMEOUT_MACROP_A_HI = 0x005E;
constexpr uint16_t RANGE_CONFIG__VCSEL_PERIOD_A     = 0x0060;
constexpr uint16_t RANGE_CONFIG__TIMEOUT_MACROP_B_HI = 0x0061;
constexpr uint16_t RANGE_CONFIG__VCSEL_PERIOD_B     = 0x0063;
constexpr uint16_t RANGE_CONFIG__VALID_PHASE_HIGH   = 0x0069;
constexpr uint16_t SYSTEM__INTERMEASUREMENT_PERIOD  = 0x006C;
constexpr uint16_t SD_CONFIG__WOI_SD0               = 0x0078;
constexpr uint16_t SD_CONFIG__INITIAL_PHASE_SD0     = 0x007A;
constexpr uint16_t ROI_CONFIG__USER_ROI_CENTRE_SPAD = 0x007F;
constexpr uint16_t ROI_CONFIG__USER_ROI_XY_SIZE     = 0x0080;
constexpr uint16_t SYSTEM__INTERRUPT_CLEAR          = 0x0086;
constexpr uint16_t SYSTEM__MODE_START               = 0x0087;
constexpr uint16_t RESULT__RANGE_STATUS             = 0x0089;
constexpr uint16_t RESULT__FINAL_RANGE_MM_SD0       = 0x0096;
constexpr uint16_t RESULT__OSC_CALIBRATE_VAL        = 0x00DE;
constexpr uint16_t FIRMWARE__SYSTEM_STATUS          = 0x00E5;
constexpr uint16_t IDENTIFICATION__MODEL_ID         = 0x010F;
constexpr uint16_t ROI_CONFIG__MODE_ROI_CENTRE_SPAD = 0x013E;

constexpr uint16_t EXPECT_MODEL_ID = 0xEACC;

// The 91-byte power-on configuration, 0x2D..0x87, verbatim from ST's ULD.
// Do not "tidy" this.  Every byte is load-bearing and a single wrong one
// gives a sensor that answers on I2C and reports nonsense.
const uint8_t DEFAULT_CONFIG[] = {
  0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x02, 0x08,  // 0x2D..0x34
  0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00,  // 0x35..0x3C
  0x00, 0xFF, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00,  // 0x3D..0x44
  0x00, 0x20, 0x0B, 0x00, 0x00, 0x02, 0x0A, 0x21,  // 0x45..0x4C
  0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xC8,  // 0x4D..0x54
  0x00, 0x00, 0x38, 0xFF, 0x01, 0x00, 0x08, 0x00,  // 0x55..0x5C
  0x00, 0x01, 0xCC, 0x0F, 0x01, 0xF1, 0x0D, 0x01,  // 0x5D..0x64
  0x68, 0x00, 0x80, 0x08, 0xB8, 0x00, 0x00, 0x00,  // 0x65..0x6C
  0x00, 0x0F, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0x6D..0x74
  0x00, 0x00, 0x01, 0x0F, 0x0D, 0x0E, 0x0E, 0x00,  // 0x75..0x7C
  0x00, 0x02, 0xC7, 0xFF, 0x9B, 0x00, 0x00, 0x00,  // 0x7D..0x84
  0x01, 0x00, 0x00                                  // 0x85..0x87
};
static_assert(sizeof(DEFAULT_CONFIG) == 0x87 - 0x2D + 1, "config blob wrong size");

// Timing-budget register pairs, short distance mode (ULD table).  Short mode
// tops out near 1.3 m but is far more immune to ambient light than long mode,
// and nothing this belt cares about is past 0.6 m.
struct BudgetPair { int ms; uint16_t a; uint16_t b; };
const BudgetPair SHORT_BUDGETS[] = {
  {15, 0x001D, 0x0027}, {20, 0x0051, 0x006E}, {33, 0x00D6, 0x006E},
  {50, 0x01AE, 0x01E8}, {100, 0x02E1, 0x0388}, {200, 0x03E1, 0x0496},
  {500, 0x0591, 0x05C1},
};
const BudgetPair LONG_BUDGETS[] = {
  {20, 0x001E, 0x0022}, {33, 0x0060, 0x006E}, {50, 0x00AD, 0x00C6},
  {100, 0x01CC, 0x01EA}, {200, 0x02D9, 0x02F8}, {500, 0x048F, 0x04A4},
};

// ST range-status codes we care about, AFTER the ULD's remap to user codes.
constexpr uint8_t RS_VALID        = 0;   // trust the distance
constexpr uint8_t RS_SIGMA_FAIL   = 1;   // too noisy — drop it
constexpr uint8_t RS_SIGNAL_FAIL  = 2;   // nothing came back = nothing there
constexpr uint8_t RS_OUT_OF_BOUNDS = 4;  // target beyond the mode's range

}  // namespace


// ═════════════════════════════════════════════════════════════════ driver ════

class Vl53l1x
{
public:
  explicit Vl53l1x(int fd) : fd_(fd) {}

  // ── raw I2C (16-bit register addressing, big endian) ──────────────────────
  bool wr(uint16_t reg, const uint8_t * data, size_t n)
  {
    uint8_t buf[8];
    if (n + 2 > sizeof(buf)) {return false;}
    buf[0] = static_cast<uint8_t>(reg >> 8);
    buf[1] = static_cast<uint8_t>(reg & 0xFF);
    std::memcpy(buf + 2, data, n);
    i2c_msg msg{SENSOR_ADDR, 0, static_cast<uint16_t>(n + 2), buf};
    i2c_rdwr_ioctl_data xfer{&msg, 1};
    return ioctl(fd_, I2C_RDWR, &xfer) >= 0;
  }

  bool rd(uint16_t reg, uint8_t * data, size_t n)
  {
    uint8_t addr[2] = {static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg & 0xFF)};
    i2c_msg msgs[2] = {
      {SENSOR_ADDR, 0, 2, addr},
      {SENSOR_ADDR, I2C_M_RD, static_cast<uint16_t>(n), data},
    };
    i2c_rdwr_ioctl_data xfer{msgs, 2};
    return ioctl(fd_, I2C_RDWR, &xfer) >= 0;
  }

  bool wr_byte(uint16_t reg, uint8_t v) {return wr(reg, &v, 1);}
  bool wr_word(uint16_t reg, uint16_t v)
  {
    uint8_t b[2] = {static_cast<uint8_t>(v >> 8), static_cast<uint8_t>(v & 0xFF)};
    return wr(reg, b, 2);
  }
  bool wr_dword(uint16_t reg, uint32_t v)
  {
    uint8_t b[4] = {
      static_cast<uint8_t>(v >> 24), static_cast<uint8_t>(v >> 16),
      static_cast<uint8_t>(v >> 8), static_cast<uint8_t>(v & 0xFF)};
    return wr(reg, b, 4);
  }
  bool rd_byte(uint16_t reg, uint8_t & v) {return rd(reg, &v, 1);}
  bool rd_word(uint16_t reg, uint16_t & v)
  {
    uint8_t b[2];
    if (!rd(reg, b, 2)) {return false;}
    v = static_cast<uint16_t>((b[0] << 8) | b[1]);
    return true;
  }

  // ── identity / boot ───────────────────────────────────────────────────────
  bool check_id()
  {
    uint16_t id = 0;
    return rd_word(IDENTIFICATION__MODEL_ID, id) && id == EXPECT_MODEL_ID;
  }

  bool wait_boot(int timeout_ms)
  {
    for (int i = 0; i < timeout_ms; ++i) {
      uint8_t st = 0;
      if (rd_byte(FIRMWARE__SYSTEM_STATUS, st) && (st & 0x01)) {return true;}
      usleep(1000);
    }
    return false;
  }

  // ── ULD init: config blob, one throwaway range, VHV tweak ────────────────
  bool init()
  {
    for (uint16_t a = 0x2D; a <= 0x87; ++a) {
      if (!wr_byte(a, DEFAULT_CONFIG[a - 0x2D])) {return false;}
    }
    if (!start_ranging()) {return false;}
    // The first measurement primes the VHV calibration; the ULD throws it away.
    for (int i = 0; i < 500; ++i) {
      bool ready = false;
      if (data_ready(ready) && ready) {break;}
      usleep(1000);
    }
    clear_interrupt();
    stop_ranging();
    if (!wr_byte(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09)) {return false;}
    return wr_byte(0x000B, 0x00);   // start VHV from the previous temperature
  }

  bool set_distance_mode(int mode)   // 1 = short, 2 = long
  {
    if (mode == 1) {
      return wr_byte(PHASECAL_CONFIG__TIMEOUT_MACROP, 0x14) &&
             wr_byte(RANGE_CONFIG__VCSEL_PERIOD_A, 0x07) &&
             wr_byte(RANGE_CONFIG__VCSEL_PERIOD_B, 0x05) &&
             wr_byte(RANGE_CONFIG__VALID_PHASE_HIGH, 0x38) &&
             wr_word(SD_CONFIG__WOI_SD0, 0x0705) &&
             wr_word(SD_CONFIG__INITIAL_PHASE_SD0, 0x0606);
    }
    return wr_byte(PHASECAL_CONFIG__TIMEOUT_MACROP, 0x0A) &&
           wr_byte(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F) &&
           wr_byte(RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D) &&
           wr_byte(RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8) &&
           wr_word(SD_CONFIG__WOI_SD0, 0x0F0D) &&
           wr_word(SD_CONFIG__INITIAL_PHASE_SD0, 0x0E0E);
  }

  // MUST be called AFTER set_distance_mode — the mode overwrites these.
  bool set_timing_budget(int ms, int mode)
  {
    const BudgetPair * table = (mode == 1) ? SHORT_BUDGETS : LONG_BUDGETS;
    const size_t n = (mode == 1) ?
      sizeof(SHORT_BUDGETS) / sizeof(BudgetPair) :
      sizeof(LONG_BUDGETS) / sizeof(BudgetPair);
    for (size_t i = 0; i < n; ++i) {
      if (table[i].ms == ms) {
        return wr_word(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, table[i].a) &&
               wr_word(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, table[i].b);
      }
    }
    return false;   // unsupported budget — caller reports it
  }

  bool set_inter_measurement(int ms)
  {
    uint16_t pll = 0;
    if (!rd_word(RESULT__OSC_CALIBRATE_VAL, pll)) {return false;}
    pll &= 0x03FF;
    return wr_dword(SYSTEM__INTERMEASUREMENT_PERIOD,
                    static_cast<uint32_t>(pll * ms * 1.075f));
  }

  // Shrinking the ROI narrows the cone on one axis.  Only needed if the floor
  // test shows the belt seeing the floor; 16x16 (the default) is the full 27 deg.
  bool set_roi(int x, int y)
  {
    uint8_t centre = 0;
    if (!rd_byte(ROI_CONFIG__MODE_ROI_CENTRE_SPAD, centre)) {return false;}
    if (x > 10 || y > 10) {centre = 199;}
    return wr_byte(ROI_CONFIG__USER_ROI_CENTRE_SPAD, centre) &&
           wr_byte(ROI_CONFIG__USER_ROI_XY_SIZE,
                   static_cast<uint8_t>(((y - 1) << 4) | (x - 1)));
  }

  bool start_ranging() {return wr_byte(SYSTEM__MODE_START, 0x40);}
  bool stop_ranging()  {return wr_byte(SYSTEM__MODE_START, 0x00);}
  bool clear_interrupt() {return wr_byte(SYSTEM__INTERRUPT_CLEAR, 0x01);}

  bool data_ready(bool & ready)
  {
    uint8_t mux = 0, status = 0;
    if (!rd_byte(GPIO_HV_MUX__CTRL, mux)) {return false;}
    const uint8_t polarity = ((mux & 0x10) >> 4) ? 0 : 1;
    if (!rd_byte(GPIO__TIO_HV_STATUS, status)) {return false;}
    ready = ((status & 0x01) == polarity);
    return true;
  }

  bool read_result(uint8_t & status, uint16_t & mm)
  {
    uint8_t raw = 0;
    if (!rd_byte(RESULT__RANGE_STATUS, raw)) {return false;}
    status = remap_status(raw & 0x1F);
    return rd_word(RESULT__FINAL_RANGE_MM_SD0, mm);
  }

private:
  // ST's internal status codes are not the ones the datasheet documents; the
  // ULD remaps them before handing them to the user.  Same table here.
  static uint8_t remap_status(uint8_t raw)
  {
    switch (raw) {
      case 9:  return 0;    // valid
      case 6:  return 1;    // sigma fail
      case 4:  return 2;    // signal fail
      case 8:  return 3;    // below minimum detection threshold
      case 5:  return 4;    // out of bounds
      case 3:  return 5;    // hardware fail
      case 19: return 6;    // no wrap-around check
      case 7:  return 7;    // wrapped target
      case 12: return 9;
      case 18: return 10;
      case 22: return 11;
      case 23: return 12;
      case 13: return 13;
      default: return 255;  // unknown
    }
  }

  int fd_;
};


// ═══════════════════════════════════════════════════════════════════ node ════

class TofBeltNode : public rclcpp::Node
{
public:
  TofBeltNode()
  : Node("tof_belt")
  {
    bus_        = declare_parameter<int>("i2c_bus", 7);
    mux_addr_   = declare_parameter<int>("mux_address", 0x70);
    rate_hz_    = declare_parameter<double>("rate_hz", 10.0);
    budget_ms_  = declare_parameter<int>("timing_budget_ms", 33);
    dist_mode_  = declare_parameter<int>("distance_mode", 1);      // 1 short, 2 long
    fov_        = declare_parameter<double>("field_of_view", 0.4712);  // 27 deg
    roi_x_      = declare_parameter<int>("roi_x", 16);
    roi_y_      = declare_parameter<int>("roi_y", 16);
    max_strikes_ = declare_parameter<int>("fault_strikes", 10);
    retry_s_    = declare_parameter<double>("quarantine_retry_s", 30.0);

    status_pub_ = create_publisher<std_msgs::msg::String>("/rover/tof_status", 10);

    const auto names = declare_parameter<std::vector<std::string>>(
      "sensors", std::vector<std::string>{});
    if (names.empty()) {
      RCLCPP_ERROR(get_logger(),
                   "No 'sensors' parameter — nothing to poll.  Did config/tof_belt.yaml load?");
      return;
    }

    if (!open_bus()) {return;}

    for (const auto & name : names) {
      Sensor s;
      s.name     = name;
      s.channel  = declare_parameter<int>(name + ".channel", -1);
      s.frame_id = declare_parameter<std::string>(name + ".frame_id", name + "_link");
      s.topic    = declare_parameter<std::string>(name + ".topic", "/tof/" + name);
      s.min_range = declare_parameter<double>(name + ".min_range", 0.04);
      s.max_range = declare_parameter<double>(name + ".max_range", 0.47);
      if (s.channel < 0 || s.channel > 7) {
        RCLCPP_ERROR(get_logger(), "%s: bad or missing channel — skipping", name.c_str());
        continue;
      }
      s.pub = create_publisher<sensor_msgs::msg::Range>(s.topic, 10);
      sensors_.push_back(s);
    }

    // Rule 1: bringup never blocks.  Sensors that fail to start are reported
    // and quarantined; the rest of the belt runs.
    for (auto & s : sensors_) {
      if (!start_sensor(s)) {
        s.quarantined = true;
        report("TOF_FAULT ch=" + std::to_string(s.channel) + " name=" + s.name +
               " reason=INIT_FAILED");
      }
    }
    mux_deselect();

    const int live = count_live();
    report("TOF_READY ok=" + std::to_string(live) + "/" + std::to_string(sensors_.size()));
    RCLCPP_INFO(get_logger(), "ToF belt up: %d of %zu sensors on /dev/i2c-%d",
                live, sensors_.size(), bus_);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz_), [this]() {poll();});
    stats_timer_ = create_wall_timer(
      std::chrono::seconds(1), [this]() {publish_stats();});
  }

  ~TofBeltNode() override
  {
    if (fd_ >= 0) {
      mux_deselect();
      close(fd_);
    }
  }

private:
  struct Sensor
  {
    std::string name, frame_id, topic;
    int channel = -1;
    double min_range = 0.04, max_range = 0.47;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub;
    int strikes = 0;
    bool quarantined = false;
    rclcpp::Time quarantined_at;
    uint64_t good = 0, dropped = 0;
  };

  // ── bus / mux ─────────────────────────────────────────────────────────────
  bool open_bus()
  {
    const std::string dev = "/dev/i2c-" + std::to_string(bus_);
    fd_ = open(dev.c_str(), O_RDWR);
    if (fd_ < 0) {
      // These two failures look identical from the outside but have completely
      // different fixes, so say which one it is.
      if (errno == ENOENT) {
        RCLCPP_ERROR(get_logger(), "%s does not exist.  Inside a container this "
                     "means the device was not passed in: add "
                     "`--device %s` to the container's run arguments and "
                     "restart it.", dev.c_str(), dev.c_str());
      } else if (errno == EACCES) {
        RCLCPP_ERROR(get_logger(), "%s exists but permission was denied.  The "
                     "device is owned by root and its own group; this process "
                     "is in neither.  Add `--group-add <gid of the device's "
                     "group>` to the container's run arguments (see `ls -l %s`), "
                     "or relax the device's mode with a udev rule on the HOST.",
                     dev.c_str(), dev.c_str());
      } else {
        RCLCPP_ERROR(get_logger(), "cannot open %s: %s",
                     dev.c_str(), std::strerror(errno));
      }
      return false;
    }
    return true;
  }

  // A mux write can TIME OUT (errno 110) when the branch being selected holds
  // SDA low — that is a jammed bus, not a missing device, and it takes the
  // whole belt down until the offending branch is disconnected.  We detect it,
  // quarantine the channel, and keep the others alive.
  bool mux_select(int channel)
  {
    if (ioctl(fd_, I2C_SLAVE, mux_addr_) < 0) {return false;}
    const uint8_t mask = static_cast<uint8_t>(1 << channel);
    return write(fd_, &mask, 1) == 1;
  }

  bool mux_deselect()
  {
    if (ioctl(fd_, I2C_SLAVE, mux_addr_) < 0) {return false;}
    const uint8_t zero = 0x00;
    return write(fd_, &zero, 1) == 1;
  }

  // ── per-sensor lifecycle ──────────────────────────────────────────────────
  bool start_sensor(Sensor & s)
  {
    if (!mux_select(s.channel)) {return false;}
    Vl53l1x dev(fd_);
    if (!dev.check_id()) {return false;}
    if (!dev.wait_boot(100)) {return false;}
    if (!dev.init()) {return false;}
    if (!dev.set_distance_mode(dist_mode_)) {return false;}
    if (!dev.set_timing_budget(budget_ms_, dist_mode_)) {
      RCLCPP_ERROR(get_logger(), "%s: timing_budget_ms=%d unsupported in mode %d",
                   s.name.c_str(), budget_ms_, dist_mode_);
      return false;
    }
    // Free-running: the sensor measures on its own and we just collect the
    // latest result, so all six poll at full rate instead of serialising on
    // each other's measurement time.
    if (!dev.set_inter_measurement(budget_ms_ + 5)) {return false;}
    if (roi_x_ < 16 || roi_y_ < 16) {
      if (!dev.set_roi(roi_x_, roi_y_)) {return false;}
    }
    return dev.start_ranging();
  }

  void fault(Sensor & s, const std::string & reason)
  {
    if (++s.strikes < max_strikes_ || s.quarantined) {return;}
    s.quarantined = true;
    s.quarantined_at = now();
    report("TOF_QUARANTINED ch=" + std::to_string(s.channel) + " name=" + s.name +
           " reason=" + reason);
    RCLCPP_WARN(get_logger(), "%s (ch %d) quarantined: %s — belt continues on %d sensors",
                s.name.c_str(), s.channel, reason.c_str(), count_live());
  }

  // ── the poll loop ─────────────────────────────────────────────────────────
  void poll()
  {
    for (auto & s : sensors_) {
      if (s.quarantined) {
        // Retry occasionally: a glitch should not cost a sensor for the rest
        // of the run, but a genuinely dead one must not be retried every cycle.
        if ((now() - s.quarantined_at).seconds() < retry_s_) {continue;}
        s.quarantined_at = now();
        if (start_sensor(s)) {
          s.quarantined = false;
          s.strikes = 0;
          report("TOF_RECOVERED ch=" + std::to_string(s.channel) + " name=" + s.name);
        }
        continue;
      }

      if (!mux_select(s.channel)) {
        fault(s, "BUS_HOLD");       // errno 110 territory — see mux_select()
        mux_deselect();
        continue;
      }

      Vl53l1x dev(fd_);
      bool ready = false;
      if (!dev.data_ready(ready)) {fault(s, "READ_FAILED"); continue;}
      if (!ready) {continue;}       // not stale, just not fresh yet

      uint8_t status = 255;
      uint16_t mm = 0;
      if (!dev.read_result(status, mm)) {fault(s, "READ_FAILED"); continue;}
      dev.clear_interrupt();        // let the next measurement latch

      s.strikes = 0;
      publish(s, status, mm);
    }
    mux_deselect();
  }

  void publish(Sensor & s, uint8_t status, uint16_t mm)
  {
    const double metres = mm / 1000.0;
    double out = 0.0;

    if (status == RS_VALID) {
      if (metres < s.min_range) {
        // Closer than the sensor can honestly resolve.  Treat as a hard mark
        // at min_range rather than dropping it — something IS there.
        out = s.min_range;
      } else if (metres >= s.max_range) {
        // Beyond our cap.  The cap exists because the floor enters the cone
        // past it (see config/tof_belt.yaml), so anything out here is either
        // the floor or irrelevant: report "clear" and let RangeSensorLayer
        // wipe the cone.
        out = s.max_range;
      } else {
        out = metres;
      }
    } else if (status == RS_SIGNAL_FAIL || status == RS_OUT_OF_BOUNDS) {
      // Nothing came back.  That is a real observation of empty space, and
      // publishing it as max_range is what clears stale marks.
      out = s.max_range;
    } else {
      // Sigma fail, hardware fail, wrap-around, unknown: we do not know what
      // is out there, so we say nothing.  Never fabricate "all clear".
      ++s.dropped;
      return;
    }

    sensor_msgs::msg::Range msg;
    msg.header.stamp = now();
    msg.header.frame_id = s.frame_id;
    msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
    msg.field_of_view = static_cast<float>(fov_);
    msg.min_range = static_cast<float>(s.min_range);
    msg.max_range = static_cast<float>(s.max_range);
    msg.range = static_cast<float>(out);
    s.pub->publish(msg);
    ++s.good;
  }

  // ── health, in the style of /rover/link_status ────────────────────────────
  void report(const std::string & line)
  {
    std_msgs::msg::String msg;
    msg.data = line;
    status_pub_->publish(msg);
  }

  int count_live() const
  {
    int n = 0;
    for (const auto & s : sensors_) {if (!s.quarantined) {++n;}}
    return n;
  }

  void publish_stats()
  {
    uint64_t good = 0, dropped = 0;
    std::string faulted;
    for (auto & s : sensors_) {
      good += s.good;
      dropped += s.dropped;
      if (s.quarantined) {faulted += (faulted.empty() ? "" : ",") + s.name;}
      s.good = 0;
      s.dropped = 0;
    }
    report("STATS ok=" + std::to_string(count_live()) +
           "/" + std::to_string(sensors_.size()) +
           " reads=" + std::to_string(good) +
           " dropped=" + std::to_string(dropped) +
           (faulted.empty() ? "" : " faulted=" + faulted));
  }

  int fd_ = -1;
  int bus_, mux_addr_, budget_ms_, dist_mode_, roi_x_, roi_y_, max_strikes_;
  double rate_hz_, fov_, retry_s_;
  std::vector<Sensor> sensors_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_, stats_timer_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TofBeltNode>());
  rclcpp::shutdown();
  return 0;
}
