// enc_ros_cmd_vel_motor_control_pid_esp32s3.ino
// ESP32-S3-N16R8 rover firmware.
//
// Reads "CMD v w" from the Jetson over USB serial, runs wheel-speed PID using
// encoder feedback, drives an MDDS30 motor driver (sign-magnitude mode), and
// reports back to the Jetson:
//   * ENC <seq> <left> <right>          — encoder counts (~10 Hz)
//   * IMU <seq> <qw> <qx> <qy> <qz>      — BNO085 game rotation vector
//          <gx> <gy> <gz>                — calibrated gyro (rad/s)
//          <ax> <ay> <az>                — linear acceleration, gravity-removed (m/s²)
//                                         (~100 Hz, sent only when fresh data exists)
//
// ───────────────────────────────────────────────────────────────────────────
// COMPATIBILITY (verified against rover_bringup/arduino_encoder_twist_bridge.py)
//   * CMD format & ENC format unchanged — drop-in replacement for the Uno.
//   * IMU line is new; bridge update parses it and publishes /imu/data.
//   * counts_per_rev=267 on the bridge side STILL REQUIRES the same
//     CHANGE-on-channel-A decode used here — do NOT switch to 4x quadrature.
//
// ───────────────────────────────────────────────────────────────────────────
// PIN MAP (ESP32-S3-N16R8 GPIO numbers — matches PHYSICAL wiring, see warning)
//   Motor controller (MDDS30):
//     ⚠ DIRECTION WIRES ARE PHYSICALLY CROSSED at the MDDS30 (found by tracing
//       wires 2026-07): GPIO 4 lands on IN2, GPIO 5 lands on IN1.  The code
//       constants PIN_IN1=5 / PIN_IN2=4 MATCH THAT REALITY.  Do NOT "tidy"
//       them back to 4/5 — straight driving would still work (signs equal)
//       but spins would reverse and the PID would fight itself (the historic
//       "12–20× runaway spin" bug).
//     IN1 (LEFT  dir) -> GPIO 5  (crossed — see warning above)
//     IN2 (RIGHT dir) -> GPIO 4  (crossed — see warning above)
//     AN1 (LEFT  PWM) -> GPIO 6
//     AN2 (RIGHT PWM) -> GPIO 7
//   Left  encoder: CHA -> GPIO 8 ,  CHB -> GPIO 9
//   Right encoder: CHA -> GPIO 10,  CHB -> GPIO 11
//   BNO085 (UART mode; strap PS1=HIGH(3.3V), PS0=LOW(GND)):
//     BNO085 "SCL" pad = its UART RX -> GPIO 1  (our TX)
//     BNO085 "SDA" pad = its UART TX -> GPIO 42 (our RX)
//     RST -> GPIO 39,  VIN -> 3.3V,  GND -> GND
//     (INT / CS / DI disconnected — SPI-era pins GPIO 40/41/2 are now FREE)
//
// ───────────────────────────────────────────────────────────────────────────
// BOARD SETTINGS (Arduino IDE or arduino-cli on the Jetson HOST)
//   Board:            "ESP32S3 Dev Module"
//   Flash Size:       16MB (128Mb)
//   PSRAM:            "OPI PSRAM"
//   USB CDC On Boot:  "Enabled"          → enumerates /dev/ttyACM0 on Jetson
//   USB Mode:         "Hardware CDC and JTAG"
//   esp32 core:       >= 3.0.0 (LEDC v3 API used)
//   arduino-cli FQBN: esp32:esp32:esp32s3:CDCOnBoot=cdc,FlashSize=16M,
//                     PSRAM=opi,USBMode=hwcdc,PartitionScheme=default_8MB
//
// LIBRARIES required (Library Manager):
//   * Adafruit BNO08x       (also pulls in Adafruit BusIO)
//   * Adafruit Unified Sensor
//
// ───────────────────────────────────────────────────────────────────────────
// IMU AXIS NOTE
//   Firmware emits IMU values in the BNO085's OWN frame.  Axis reorientation
//   (if the chip is mounted with Z down, Y inverted, etc.) is handled on the
//   Jetson side in the bridge node so it is hot-tunable without reflashing.
//   The bridge defaults to identity (chip frame = base_link orientation).

#include <Arduino.h>
#include <math.h>
#include <Adafruit_BNO08x.h>

// ───────────────────────── PIN DEFINITIONS ─────────────────────────
// Motor controller pins.  ⚠ IN1/IN2 values are SWAPPED vs the MDDS30 silkscreen
// because the physical direction wires are crossed at the driver (see PIN MAP
// warning in the header).  These constants encode reality — do not "fix" them.
const int PIN_IN1 = 5;   // LEFT  direction (physically crossed wire)
const int PIN_IN2 = 4;   // RIGHT direction (physically crossed wire)
const int PIN_AN1 = 6;   // LEFT  PWM
const int PIN_AN2 = 7;   // RIGHT PWM

//motor encoder pins
const int PIN_LEFT_ENC_A  = 8;
const int PIN_LEFT_ENC_B  = 9;
const int PIN_RIGHT_ENC_A = 10;
const int PIN_RIGHT_ENC_B = 11;

// BNO085 IMU (UART mode).  The pad names are the breakout's I²C-era silkscreen:
// in UART mode the "SDA" pad is the chip's TX (→ our RX) and the "SCL" pad is
// the chip's RX (→ our TX).
const int PIN_IMU_RST = 39;
const int PIN_IMU_SDA = 42;   // our UART RX  (chip TX)
const int PIN_IMU_SCL = 1;    // our UART TX  (chip RX)

// ───────────────────────── LEDC PWM CONFIG ─────────────────────────
const int PWM_FREQ_HZ  = 20000;
const int PWM_RES_BITS = 8;

// ───────────────────────── SERIAL DEBUG VERBOSITY ──────────────────
// When false, suppress the high-rate debug lines that flood the USB-serial
// link and can delay inbound CMD parsing (risking the 500 ms watchdog):
//   * PWM       — every control cycle (~50 Hz)
//   * CMD_ECHO  — every command (10 Hz)
// ENC and IMU — the ONLY lines the Jetson bridge parses — are ALWAYS sent.
// Set true to restore full verbose telemetry for tuning.
const bool DEBUG_SERIAL = false;   // false for production — the 50 Hz PWM spam
                                   // risks CMD latency (watchdog)

// ───────────────────────── ENCODER REPORT ──────────────────────────
static uint32_t enc_seq = 0;//Encoder data current counter value
static uint32_t imu_seq = 0;//Imu data current counter value

// ───────────────────────── TURN BOOST SETTINGS ─────────────────────
unsigned long turnKickUntil = 0;
const unsigned long TURN_KICK_MS = 200;

// ───────────────────────── ROBOT PARAMETERS ────────────────────────
// 3601 Series Rhino wheels: 120 mm diameter (was 95 mm).  This DOUBLE-counts
// the wheel-circumference change AND the new chassis track width.  Both
// numbers MUST match the bridge launch params or odometry is wrong.
const float WHEEL_BASE_M     = 0.37f;   // was 0.34 — drive-axle center-to-center on new chassis
const float WHEEL_DIAMETER_M = 0.120f;  // was 0.095
const float WHEEL_CIRC       = WHEEL_DIAMETER_M * M_PI;

const float COUNTS_PER_REV = 267;   // unchanged — encoder CPR is wheel-size independent

const float MAX_PWM = 255.0f;
// Top free-run wheel speed scales linearly with wheel diameter:
//   0.99 m/s × (0.120 / 0.095) ≈ 1.25 m/s
// This is the feedforward normalizing constant; PID still rides on top.
const float MAX_SPEED_MPS = 1.25f;

// Timing
const unsigned long CONTROL_PERIOD_MS = 20;
const unsigned long CMD_TIMEOUT_MS    = 1000;
const unsigned long ENCODER_REPORT_MS = 100;

// ───────────────────────── PID GAINS ───────────────────────────────
float KP = 0.6f;
float KI = 0.40f;
float KD = 0.02f;

// ── Per-wheel straight-line calibration trim (2026-07) ──────────────────────
// Open-loop correction for a SYSTEMATIC straight-line drift (wheel-diameter /
// encoder-scale / motor mismatch). Applied to the wheel-speed SETPOINTS so the
// inner PID realizes and HOLDS it — a raw post-PID PWM offset would be erased by
// the PID integral (same lesson learned from the yaw trim).
//   * 1.00 / 1.00     = no trim (raw behaviour).
//   * Rover drifts LEFT  → right side travels more → LOWER RIGHT_WHEEL_TRIM
//                          (or raise LEFT_WHEEL_TRIM) in ~0.02 steps.
//   * Rover drifts RIGHT → do the opposite.
// Multiplies the SIGNED target, so it corrects the same way fwd and reverse.
float LEFT_WHEEL_TRIM  = 1.00f;
float RIGHT_WHEEL_TRIM = 0.969f;   // START: 3% down on the right to counter the observed LEFT drift — refine per the rule above

// ── Accel/decel ramp — motion shaping (2026-07) ──────────────────────────────
// Commands from the Jetson STEP (0.21 → 0 in one cycle).  Wheels can't do that;
// the rover skids to a stop and the two sides never decelerate identically →
// yaw twitch at every stop (and a lurch at every launch).  Fix: slew-limit the
// setpoints so the PID tracks a CONTROLLED, symmetric decel/accel on encoder
// feedback instead of chasing an impossible step.  Cost at 0.21 m/s cruise:
// ~0.26 s / ~3 cm extra travel per stop.
// SAFETY UNCHANGED: watchdog (500 ms) and CMD timeout (1 s) still hard-cut PWM
// instantly — the ramp only shapes NORMAL commanded stops/starts.
const float MAX_LIN_ACCEL = 0.8f;   // m/s² — full stop from 0.21 m/s in ~0.26 s
const float MAX_ANG_ACCEL = 3.0f;   // rad/s² — snappy but bounded turn transitions
float rampLinear  = 0.0f;           // slewed version of cmdLinear
float rampAngular = 0.0f;           // slewed version of the effective angular cmd

static inline float slewToward(float cur, float tgt, float maxStep) {
  float d = tgt - cur;
  if (d >  maxStep) return cur + maxStep;
  if (d < -maxStep) return cur - maxStep;
  return tgt;
}

// NOTE (2026-08): the IMU yaw/heading-assist outer loop that used to live here
// was REMOVED after being retired in 2026-07 (default OFF; raw wheel-PID drove
// straighter — the assist fought the wheel trims, suspect flipped yaw sign).
// The IMU still streams to the Jetson for SLAM/EKF regardless; only the
// firmware's motor-side use of it is gone.  If a heading assist is ever wanted
// again, redesign it against the git history / backup rather than resurrecting
// the old gains blind — and bench-verify the yaw sign first.

// ───────────────────────── STATE ───────────────────────────────────
volatile long leftEncoderCount  = 0;
volatile long rightEncoderCount = 0;

long prevLeftCount  = 0;
long prevRightCount = 0;

float cmdLinear  = 0.0f;
float cmdAngular = 0.0f;

float leftIntegral  = 0.0f;
float rightIntegral = 0.0f;

float leftPrevErr  = 0.0f;
float rightPrevErr = 0.0f;

unsigned long lastCmdTime     = 0;
unsigned long lastControlTime = 0;
unsigned long lastReportTime  = 0;

static float prevCmdLinRaw = 0.0f;
static float prevCmdAngRaw = 0.0f;

// ── Serial watchdog ──
unsigned long lastCmdMs  = 0;
const unsigned long WATCHDOG_MS = 500;
bool motorsEnabled = false;

// ───────────────────────── BNO085 STATE ────────────────────────────
// UART transport (Serial1).  The constructor only needs the RST pin; the UART
// pins are passed to Serial1.begin() in initIMU().
Adafruit_BNO08x bno08x(PIN_IMU_RST);
sh2_SensorValue_t sensorEvent;
volatile bool imuPresent = false;   // volatile: written by core-0 task, read by core 1
// "fresh data since last emit" flag: set true by pollIMU() (core 0) whenever a
// new sample is written; consumed (reset to false) by maybeReportIMU() (core 1)
// after each emit so we NEVER ship a repeated snapshot.  Guarded by imuMux like
// the rest of the shared snapshot.
bool   imuDirty     = false;
float  q_w = 1.0f, q_x = 0.0f, q_y = 0.0f, q_z = 0.0f;
float  g_x = 0.0f, g_y = 0.0f, g_z = 0.0f;
float  a_x = 0.0f, a_y = 0.0f, a_z = 0.0f;
unsigned long lastImuReportMs = 0;
const unsigned long IMU_REPORT_MIN_MS = 10;   // cap at ~100 Hz outbound
// Requested sample interval for BOTH BNO085 reports enabled in setReports().
// Shared constant (rather than two separate literals) so the two reports
// can't drift out of sync with each other if this ever changes.
const uint32_t IMU_REPORT_INTERVAL_US = 10000;   // 100 Hz

// ── Dual-core plumbing ──
// The BNO085 SHTP-over-UART service (getSensorEvent) can block for up to ~1 s
// when a frame is late on a marginal 3 Mbaud link.  We run it on its OWN task
// pinned to core 0 so it can never stall the core-1 control loop (encoders +
// PID).  The quaternion/gyro/accel snapshot above is shared between the two
// cores and guarded by this spinlock; keep the guarded sections tiny.
portMUX_TYPE imuMux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t imuTaskHandle = NULL;

// ───────────────────────── ENCODER ISRs ────────────────────────────
void IRAM_ATTR leftEncoderISR() {
  int a = digitalRead(PIN_LEFT_ENC_A);
  int b = digitalRead(PIN_LEFT_ENC_B);
  if (a == b) leftEncoderCount--;
  else        leftEncoderCount++;
}

void IRAM_ATTR rightEncoderISR() {
  int a = digitalRead(PIN_RIGHT_ENC_A);
  int b = digitalRead(PIN_RIGHT_ENC_B);
  if (a == b) rightEncoderCount++;
  else        rightEncoderCount--;
}

// ───────────────────────── MOTOR OUTPUT ────────────────────────────
void setMotor(int leftPWM, int rightPWM) {
  leftPWM  = constrain(leftPWM,  -255, 255);
  rightPWM = constrain(rightPWM, -255, 255);

  digitalWrite(PIN_IN1, (leftPWM >= 0) ? HIGH : LOW);
  ledcWrite(PIN_AN1, abs(leftPWM));

  digitalWrite(PIN_IN2, (rightPWM >= 0) ? HIGH : LOW);
  ledcWrite(PIN_AN2, abs(rightPWM));
}

void stopMotors() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  ledcWrite(PIN_AN1, 0);
  ledcWrite(PIN_AN2, 0);
}

// ───────────────────────── BNO085 SETUP / POLL ─────────────────────
void setReports() {
  // Game rotation vector: orientation.  The BNO085's standalone gyro reports
  // (SH2_GYROSCOPE_CALIBRATED / _UNCALIBRATED) return zeros on this chip over
  // both SPI and UART, so we derive body-frame angular velocity from the
  // quaternion delta in pollIMU() instead.  10000 µs = 100 Hz.
  // NOTE: at 100 Hz the derived gyro (omega = 2*vec(dq)/dt) is ~2x noisier per
  // sample than at 50 Hz because dt is halved — if the EKF looks jittery, raise
  // imu_gyro_cov in the bridge to match.
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, IMU_REPORT_INTERVAL_US)) {
    Serial.println("IMU_WARN: failed to enable game rotation vector");
  }
  // Linear acceleration: gravity already removed by the chip's own fusion
  // engine. Same interval as the rotation vector above so the two fields in
  // any one outbound IMU line are always from the same instant instead of
  // skewed against each other. Purely additive: the a_x/a_y/a_z fields and
  // their handler in pollIMU() already existed (previously always 0.0000,
  // unused because nothing asked the sensor for this report) — this just
  // turns the data on.
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION, IMU_REPORT_INTERVAL_US)) {
    Serial.println("IMU_WARN: failed to enable linear acceleration");
  }
}

void initIMU() {
  pinMode(PIN_IMU_RST, OUTPUT);
  digitalWrite(PIN_IMU_RST, LOW); delay(10); digitalWrite(PIN_IMU_RST, HIGH); delay(100);

  Serial1.setRxBufferSize(2048);   // headroom for 3 Mbaud SHTP frames
  Serial1.begin(3000000, SERIAL_8N1, /*RX=*/PIN_IMU_SDA, /*TX=*/PIN_IMU_SCL);

  if (!bno08x.begin_UART(&Serial1)) {
    Serial.println("IMU_FAIL: BNO085 not detected over UART");
    imuPresent = false;
    return;
  }
  imuPresent = true;
  Serial.println("IMU_OK: BNO085 detected via UART, enabling reports");
  setReports();

  // Steady-state polling MUST be non-blocking: getSensorEvent() reads from
  // Serial1 via readBytes(), which otherwise blocks up to the Stream default
  // timeout (~1 s) whenever a valid frame is late.  On a marginal 3 Mbaud link
  // that stalls the ENTIRE control loop (encoders + PID) down to ~1 Hz.  With a
  // 0 ms timeout, getSensorEvent() consumes only already-buffered bytes and
  // returns immediately; SHTP frames still assemble across successive loop()
  // passes.  Set AFTER begin_UART()/setReports() so their init handshakes,
  // which DO need blocking reads, complete normally.
  Serial1.setTimeout(0);
}

void pollIMU() {
  if (!imuPresent) return;

  // IMU stall watchdog state: the stream can die WITHOUT wasReset() firing
  // (UART desync / chip hang — seen 2026-08-01: /imu/data silent 29+ s
  // mid-nav).  If nothing decodes for 3 s, do a full re-init below: RST pin
  // toggle + UART restart + report re-enable (initIMU() does all three).
  static unsigned long lastEventMs = 0;
  if (lastEventMs == 0) lastEventMs = millis();

  // bno08x.wasReset() returns true after a sensor-side reset (rare but it
  // happens — power blip, electrical glitch).  Reports must be re-enabled.
  if (bno08x.wasReset()) {
    Serial.println("IMU_RESET: re-enabling reports");
    // setReports()'s enableReport() handshake needs BLOCKING reads to catch the
    // chip's confirmation — the same reason initIMU() defers setTimeout(0) until
    // AFTER its setReports().  In steady state the timeout is already 0, so a
    // bare setReports() here would read the handshake with a 0 ms timeout, the
    // confirmation would fail, and the report(s) could silently NOT re-enable
    // (IMU goes quiet until a full reboot).  Restore a blocking timeout for the
    // handshake, then return to non-blocking.  Safe to block here: pollIMU()
    // runs on the core-0 imuTask, isolated from the core-1 control loop.
    Serial1.setTimeout(100);
    setReports();
    Serial1.setTimeout(0);
  }

  // Drain whatever the chip has ready this tick.  CRITICAL: process each event
  // as it is decoded.  Do NOT place a time/count break BEFORE this switch — a
  // decoded quaternion would be discarded and imuDirty never set, so
  // maybeReportIMU() would emit nothing (that was the "no IMU data" bug).
  int n = 0;
  while (bno08x.getSensorEvent(&sensorEvent)) {
    switch (sensorEvent.sensorId) {
      case SH2_GAME_ROTATION_VECTOR: {
        float nq_w = sensorEvent.un.gameRotationVector.real;
        float nq_x = sensorEvent.un.gameRotationVector.i;
        float nq_y = sensorEvent.un.gameRotationVector.j;
        float nq_z = sensorEvent.un.gameRotationVector.k;

        // Derive body-frame angular velocity from the quaternion delta:
        //   omega ~= 2 * vec(qprev^-1 * qcur) / dt
        static float pw = 1, px = 0, py = 0, pz = 0;
        static unsigned long pt = 0;
        float lgx = 0, lgy = 0, lgz = 0;
        bool  gotGyro = false;
        unsigned long tn = micros();
        if (pt != 0) {
          float dt = (tn - pt) * 1e-6f;
          if (dt > 0.0005f && dt < 0.5f) {
            float rw = pw*nq_w + px*nq_x + py*nq_y + pz*nq_z;   // conj(prev) * cur
            float rx = pw*nq_x - px*nq_w - py*nq_z + pz*nq_y;
            float ry = pw*nq_y + px*nq_z - py*nq_w - pz*nq_x;
            float rz = pw*nq_z - px*nq_y + py*nq_x - pz*nq_w;
            if (rw < 0) { rx = -rx; ry = -ry; rz = -rz; }        // shortest arc
            float s = 2.0f / dt;
            lgx = rx * s;  lgy = ry * s;  lgz = rz * s;
            gotGyro = true;
          }
        }
        pt = tn;  pw = nq_w; px = nq_x; py = nq_y; pz = nq_z;

        // Publish atomically — core 1's maybeReportIMU() reads under the same lock.
        portENTER_CRITICAL(&imuMux);
        q_w = nq_w; q_x = nq_x; q_y = nq_y; q_z = nq_z;
        if (gotGyro) { g_x = lgx; g_y = lgy; g_z = lgz; }
        imuDirty = true;   // new sample → maybeReportIMU() will emit exactly once
        portEXIT_CRITICAL(&imuMux);
        break;
      }
      case SH2_LINEAR_ACCELERATION: {  // gravity-removed accel; enabled in setReports()
        float lax = sensorEvent.un.linearAcceleration.x;
        float lay = sensorEvent.un.linearAcceleration.y;
        float laz = sensorEvent.un.linearAcceleration.z;
        portENTER_CRITICAL(&imuMux);
        a_x = lax; a_y = lay; a_z = laz;
        imuDirty = true;
        portEXIT_CRITICAL(&imuMux);
        break;
      }
      default: break;
    }
    if (++n >= 64) break;   // safety bound only; loop exits naturally when drained
  }

  // ── IMU stall watchdog ──────────────────────────────────────────────────
  if (n > 0) {
    lastEventMs = millis();
  } else if (millis() - lastEventMs > 3000) {
    Serial.println("IMU_STALL: no IMU data for 3s — full re-init (RST + UART + reports)");
    initIMU();                 // safe to re-run: toggles RST, re-begins Serial1, setReports()
    lastEventMs = millis();    // retry at most every 3 s if still dead
  }
}

void maybeReportIMU() {
  if (!imuPresent) return;
  unsigned long now = millis();
  if (now - lastImuReportMs < IMU_REPORT_MIN_MS) return;

  // Copy the shared snapshot (written by the core-0 imuTask) under the lock, and
  // consume the freshness flag in the same critical section so we emit a given
  // sample exactly once — no duplicated snapshots between real sensor updates.
  float lqw, lqx, lqy, lqz, lgx, lgy, lgz, lax, lay, laz;
  bool  lDirty;
  portENTER_CRITICAL(&imuMux);
  lqw = q_w; lqx = q_x; lqy = q_y; lqz = q_z;
  lgx = g_x; lgy = g_y; lgz = g_z;
  lax = a_x; lay = a_y; laz = a_z;
  lDirty = imuDirty;
  imuDirty = false;
  portEXIT_CRITICAL(&imuMux);

  if (!lDirty) return;   // no new sensor data since last emit — skip (no duplicate)
  lastImuReportMs = now;

  // Fixed-format IMU line.  Field order matches the Jetson bridge parser.
  char out[192];
  snprintf(out, sizeof(out),
           "IMU %lu %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.4f %.4f %.4f\n",
           (unsigned long)imu_seq++,
           lqw, lqx, lqy, lqz,
           lgx, lgy, lgz,
           lax, lay, laz);
  Serial.print(out);
}

// ───────────────────────── IMU TASK (core 0) ───────────────────────
// Dedicated task, PINNED TO CORE 0.  It owns all BNO085 UART I/O: init, then a
// tight service loop.  If getSensorEvent() blocks on a late frame, only THIS
// task (core 0) waits — the control loop on core 1 keeps running at full rate.
// Servicing the UART continuously here also keeps the RX buffer drained so SHTP
// frames stay intact.  It only updates the shared snapshot; core 1 emits it.
void imuTask(void *param) {
  initIMU();                              // runs in this task's context (core 0)
  for (;;) {
    if (imuPresent) {
      pollIMU();
      vTaskDelay(1);                      // yield ~1 tick so idle/WDT get serviced
    } else {
      vTaskDelay(pdMS_TO_TICKS(500));     // no IMU present — idle, don't spin
    }
  }
}

// ───────────────────────── SERIAL PARSER ───────────────────────────
void processCommandLine(const String &line) {
  if (!line.startsWith("CMD")) return;

  int s1 = line.indexOf(' ');
  int s2 = line.indexOf(' ', s1 + 1);
  if (s1 < 0 || s2 < 0) return;

  float newLin    = line.substring(s1 + 1, s2).toFloat();
  float newAngRaw = line.substring(s2 + 1).toFloat();

  lastCmdMs       = millis();
  motorsEnabled   = true;

  bool nowTurningInPlace = (fabs(newLin) < 0.01f && fabs(newAngRaw) > 0.01f);
  bool wasTurningInPlace = (fabs(prevCmdLinRaw) < 0.01f && fabs(prevCmdAngRaw) > 0.01f);

  if (nowTurningInPlace && !wasTurningInPlace) {
    turnKickUntil = millis() + TURN_KICK_MS;
  }

  // Angular commands are used as-received: all angular calibration scaling
  // (cmd_w_scale etc.) lives in the Jetson bridge where it is hot-tunable.
  cmdLinear  = newLin;
  cmdAngular = newAngRaw;

  static float prevCmdL = 0.0f, prevCmdA = 0.0f;
  const float CMD_CHANGE_THR = 0.05f;
  if (fabs(cmdLinear - prevCmdL) > CMD_CHANGE_THR ||
      fabs(cmdAngular - prevCmdA) > CMD_CHANGE_THR) {
      leftIntegral = rightIntegral = 0.0f;
      leftPrevErr = rightPrevErr = 0.0f;
  }
  prevCmdL = cmdLinear;
  prevCmdA = cmdAngular;

  if (DEBUG_SERIAL) {
    Serial.print("CMD_ECHO ");
    Serial.print(cmdLinear, 3);
    Serial.print(" ");
    Serial.println(cmdAngular, 3);
  }

  prevCmdLinRaw = newLin;
  prevCmdAngRaw = newAngRaw;

  lastCmdTime = millis();
}

// ───────────────────────── SETUP ───────────────────────────────────
void setup() {
  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);
#endif

  // Motor pins
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  ledcAttach(PIN_AN1, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttach(PIN_AN2, PWM_FREQ_HZ, PWM_RES_BITS);
  stopMotors();

  // Encoders
  pinMode(PIN_LEFT_ENC_A,  INPUT_PULLUP);
  pinMode(PIN_LEFT_ENC_B,  INPUT_PULLUP);
  pinMode(PIN_RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(PIN_RIGHT_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_LEFT_ENC_A),  leftEncoderISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RIGHT_ENC_A), rightEncoderISR, CHANGE);

  // BNO085 on UART.  Its SHTP service can block on a late frame, so it runs in
  // a DEDICATED TASK PINNED TO CORE 0 (imuTask) — never on the core-1 control
  // loop.  Non-fatal if missing: the drivetrain runs and the Jetson EKF
  // degrades to encoder-only odometry.  initIMU() runs inside the task.
  xTaskCreatePinnedToCore(imuTask, "imuTask", 8192, NULL, 1, &imuTaskHandle, 0);

  lastCmdTime     = millis();
  lastControlTime = millis();
  lastReportTime  = millis();
  lastImuReportMs = millis();

  Serial.println("PID VERSION READY rev-I (ESP32-S3, BNO085 UART, quat-derived gyro, IMU stall watchdog, dead-code cleanup)");
  Serial.print("WHEEL_DIAMETER_M="); Serial.println(WHEEL_DIAMETER_M, 4);
  Serial.print("WHEEL_BASE_M=");     Serial.println(WHEEL_BASE_M, 4);
  Serial.print("MAX_SPEED_MPS=");    Serial.println(MAX_SPEED_MPS, 4);
  Serial.print("WATCHDOG_MS=");      Serial.println(WATCHDOG_MS);
  Serial.println("IMU: init on core-0 task (watch for IMU_OK / IMU_FAIL below)");
  Serial.println("Motors disabled until first CMD received");
}

// ───────────────────────── LOOP ────────────────────────────────────
void loop() {

  // ── Serial watchdog ──
  if (!motorsEnabled || (millis() - lastCmdMs > WATCHDOG_MS)) {
    stopMotors();
    cmdLinear     = 0.0f;
    cmdAngular    = 0.0f;
    leftIntegral  = 0.0f;
    rightIntegral = 0.0f;
    leftPrevErr   = 0.0f;
    rightPrevErr  = 0.0f;
    rampLinear    = 0.0f;   // hard stop = wheels at 0; ramp must restart from 0
    rampAngular   = 0.0f;
    motorsEnabled = false;

    static unsigned long lastWatchdogLogTime = 0;
    if ((millis() - lastWatchdogLogTime) > 500) {
      Serial.println("WATCHDOG_TRIGGERED");
      lastWatchdogLogTime = millis();
    }
  }

  // ── SERIAL READ ──
  while (Serial.available() > 0) {
    static String line = "";
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (line.length() > 0) {
        processCommandLine(line);
        line = "";
      }
    } else {
      line += c;
    }
  }

  // ── IMU REPORT ── (polling runs on the core-0 imuTask; here we just emit the
  // latest snapshot — never blocks the control loop)
  maybeReportIMU();

  unsigned long now = millis();

  // ── SAFETY TIMEOUT ──
  if (now - lastCmdTime > CMD_TIMEOUT_MS) {
    cmdLinear     = 0.0f;
    cmdAngular    = 0.0f;
    leftIntegral  = 0.0f;
    rightIntegral = 0.0f;
    leftPrevErr   = 0.0f;
    rightPrevErr  = 0.0f;
    rampLinear    = 0.0f;   // hard stop = wheels at 0; ramp must restart from 0
    rampAngular   = 0.0f;
    stopMotors();
  }

  // ── CONTROL LOOP ──
  if (now - lastControlTime >= CONTROL_PERIOD_MS) {
    float dt = (now - lastControlTime) / 1000.0f;
    lastControlTime = now;
    if (dt <= 0.0001f) dt = 0.0001f;

    float halfBase = WHEEL_BASE_M * 0.5f;
    bool turningInPlace = (fabs(cmdLinear) < 0.01f && fabs(cmdAngular) > 0.01f);

    // ── Accel/decel ramp (see MAX_LIN/ANG_ACCEL above) ────────────────────
    // Slew the setpoint inputs so stops/starts are controlled and symmetric
    // instead of step changes the wheels physically can't follow (skid → yaw
    // twitch at every stop).  Watchdog/timeout hard-stops bypass this ramp.
    rampLinear  = slewToward(rampLinear,  cmdLinear,  MAX_LIN_ACCEL * dt);
    rampAngular = slewToward(rampAngular, cmdAngular, MAX_ANG_ACCEL * dt);

    float vLeftTarget  = rampLinear - rampAngular * halfBase;
    float vRightTarget = rampLinear + rampAngular * halfBase;

    // Per-wheel straight-line calibration (see LEFT/RIGHT_WHEEL_TRIM above).
    vLeftTarget  *= LEFT_WHEEL_TRIM;
    vRightTarget *= RIGHT_WHEEL_TRIM;

    noInterrupts();
    long leftNow  = leftEncoderCount;
    long rightNow = rightEncoderCount;
    interrupts();

    long dLeft  = leftNow  - prevLeftCount;
    long dRight = rightNow - prevRightCount;

    prevLeftCount  = leftNow;
    prevRightCount = rightNow;

    float leftMps  = ((float)dLeft  / COUNTS_PER_REV) * WHEEL_CIRC / dt;
    float rightMps = ((float)dRight / COUNTS_PER_REV) * WHEEL_CIRC / dt;

    float eLeft  = vLeftTarget  - leftMps;
    float eRight = vRightTarget - rightMps;

    float leftDerivative  = (eLeft  - leftPrevErr) / dt;
    float rightDerivative = (eRight - rightPrevErr) / dt;

    leftPrevErr  = eLeft;
    rightPrevErr = eRight;

    float ffLeft  = (vLeftTarget  / MAX_SPEED_MPS) * MAX_PWM;
    float ffRight = (vRightTarget / MAX_SPEED_MPS) * MAX_PWM;

    float pidLeftOut  = KP*eLeft  + KI*leftIntegral  + KD*leftDerivative;
    float pidRightOut = KP*eRight + KI*rightIntegral + KD*rightDerivative;

    float PID_SCALE = 150.0f;

    int leftPWM  = (int)(ffLeft  + pidLeftOut  * PID_SCALE);
    int rightPWM = (int)(ffRight + pidRightOut * PID_SCALE);

    if (turningInPlace) {   // stiction-breaking feedforward: spins need extra
                            // PWM to break static friction and start moving
      const float TGT_MPS  = 0.01f;
      const int TURN_BIAS_PWM = 47;
      const int KICK_PWM      = 27;

      if (fabs(vLeftTarget) > TGT_MPS)  leftPWM  += (vLeftTarget  > 0 ? +TURN_BIAS_PWM : -TURN_BIAS_PWM);
      if (fabs(vRightTarget) > TGT_MPS) rightPWM += (vRightTarget > 0 ? +TURN_BIAS_PWM : -TURN_BIAS_PWM);

      if ((long)(millis() - turnKickUntil) < 0) {
        if (fabs(vLeftTarget) > TGT_MPS)  leftPWM  += (vLeftTarget  > 0 ? +KICK_PWM : -KICK_PWM);
        if (fabs(vRightTarget) > TGT_MPS) rightPWM += (vRightTarget > 0 ? +KICK_PWM : -KICK_PWM);
      }
    }

    leftPWM  = constrain(leftPWM,  -180, 180);
    rightPWM = constrain(rightPWM, -180, 180);

    bool leftSaturated  = abs(leftPWM)  >= 180;
    bool rightSaturated = abs(rightPWM) >= 180;

    if (!leftSaturated) leftIntegral += eLeft * dt;
    else leftIntegral *= 0.995f;

    if (!rightSaturated) rightIntegral += eRight * dt;
    else rightIntegral *= 0.995f;

    leftIntegral  = constrain(leftIntegral,  -0.5f, 0.5f);
    rightIntegral = constrain(rightIntegral, -0.5f, 0.5f);

    if (DEBUG_SERIAL) {
      Serial.print("PWM ");
      Serial.print(leftPWM);
      Serial.print(" ");
      Serial.println(rightPWM);
    }

    setMotor(leftPWM, rightPWM);
  }

  // ── ENCODER REPORT ──
  if (now - lastReportTime >= ENCODER_REPORT_MS) {
    noInterrupts();
    long left  = leftEncoderCount;
    long right = rightEncoderCount;
    interrupts();

    char out[64];
    snprintf(out, sizeof(out), "ENC %lu %ld %ld\n",
             (unsigned long)enc_seq++, left, right);
    Serial.print(out);

    lastReportTime = now;
  }
}
