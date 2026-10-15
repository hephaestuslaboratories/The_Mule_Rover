# System architecture

A file-by-file explanation of what runs on the rover, how the pieces connect,
and why the non-obvious choices were made. Read the top-level
[README](../README.md) first for the hardware and the quick start,
[INSTALL.md](INSTALL.md) for dependencies and environment, and
[CALIBRATION.md](CALIBRATION.md) for how every measured constant was obtained.

Contents

1. [Data flow](#1-data-flow)
2. [Coordinate frames](#2-coordinate-frames)
3. [The launch file](#3-the-launch-file)
4. [Nodes in `rover_bringup_extended_slam`](#4-nodes-in-rover_bringup_extended_slam)
5. [Third-party nodes the launch starts](#5-third-party-nodes-the-launch-starts)
6. [Configuration files](#6-configuration-files)
7. [ESP32-S3 motor/IMU firmware and the serial protocol](#7-esp32-s3-motorimu-firmware-and-the-serial-protocol)
8. [`rover_web`: the phone cockpit](#8-rover_web-the-phone-cockpit)
9. [Serial device naming on the Jetson](#9-serial-device-naming-on-the-jetson)
10. [Tools](#10-tools)
11. [Diagnostic topics](#11-diagnostic-topics)
12. [Safety layers](#12-safety-layers)
13. [Known drift between comments and code](#13-known-drift-between-comments-and-code)

---

## 1. Data flow

```
                 ┌───────────────────────── ESP32-S3 (firmware/) ──────────────────────────┐
 BNO085 ──UART──►│ core 0: IMU task        core 1: 50 Hz wheel PID, encoder ISRs, watchdog │
 encoders ──────►│                                                                         │
 MDDS30  ◄──PWM──│                                                                         │
                 └──────────────────────────── USB CDC 115200 ───────────────────────────────┘
                        ▲ "CMD v w"  (50 Hz)                 │ "ENC seq L R" (10 Hz)
                        │                                    │ "IMU seq qw qx qy qz gx gy gz ax ay az" (100 Hz)
                        │                                    ▼
                 ┌──────┴──────────────── arduino_encoder_twist_bridge ─────────────────────┐
                 │  /cmd_vel ──► trims, yaw_sign, cmd_w_scale ──► serial                    │
                 │  serial ──► /wheel/odom_twist  /wheel/{left,right}_velocity  /imu/data   │
                 │             /rover/cmd_sent  /rover/link_status                          │
                 └───────────────────────────────────────────────────────────────────────────┘
                                   │                              │
                                   │ /imu/data                    │ /wheel/odom_twist
                                   ▼                              │
                              zupt_node ──► /imu/filtered ────────┼──────────────┐
                                                 │                ▼              │
                                                 │      robot_localization EKF   │
                                                 │      (30 Hz) ──► /odometry/filtered,  odom→base_link TF
                                                 ▼
 RPLidar A3M1 ──► /scan ──► scan_deskew ──► /scan_deskewed ──► lidar_scan_filter ──► /scan_filtered
                                                                                          │
                      ┌───────────────────────────────────────────────────────────────────┤
                      ▼                                   ▼                               ▼
              slam_toolbox                         Nav2 costmaps                 localization_monitor
      (mapping or localization)                (obstacle_layer)                 (/rover/localization_*)
      ──► /map, map→odom TF
                                                       ▲
 6× VL53L1X ──I²C──► tof_belt_node ──► /tof/{front_left,front_right,corner_left,corner_right,rear}
                           └──► /tof/down ──► tof_down_filter ──► /tof/down_obstacle   (range_layer)

 Nav2: NavFn planner ──► SimpleSmoother ──► Regulated Pure Pursuit ──► velocity_smoother ──► /cmd_vel
 Operator: rover_web (phone browser) ──► /cmd_vel (joystick) or navigate_to_pose (saved places)
```

Everything publishing to `/cmd_vel` goes straight to the bridge. There is no
intermediate safety filter topic in this build.

## 2. Coordinate frames

Defined in [`config/rover.urdf.xacro`](../rover_bringup_extended_slam/config/rover.urdf.xacro).
All measurements are in metres relative to `base_link`, which sits at the
midpoint of the drive axle, 0.060 m above the floor (one wheel radius).

| Frame | xyz | rpy | Notes |
|---|---|---|---|
| `base_footprint` → `base_link` | 0, 0, 0.060 | 0 | floor projection |
| `left_drive_wheel` / `right_drive_wheel` | 0, ±0.185, 0 | π/2, 0, 0 | continuous joints; track 0.37 m |
| four casters | ±0.16 / −0.36, ±0.12, −0.02 | 0 | passive swivel casters, cosmetic |
| `laser_frame` | −0.335, 0.0127, 0.50 | 0, 0, −0.0202 | RPLidar A3M1; x verified to 0.4 mm, yaw provisional |
| `imu_link` | −0.0694, 0, 0.11145 | 0 | BNO085 |
| `tof_front_left_link` / `tof_front_right_link` | 0.21, ±0.10, 0.075 | 0 | |
| `tof_corner_left_link` / `tof_corner_right_link` | 0.21, ±0.1575, 0.095 | 0, 0, ±0.785 | 45° outward |
| `tof_down_link` | 0.21, 0, 0.305 | 0, 0.7505, 0 | pitched 43° down |
| `tof_down_proj_link` | 0.21, 0, 0.075 | 0 | flat frame the down filter republishes on |
| `tof_rear_link` | −0.41, 0, 0.075 | 0, 0, π | |

The chassis box is 0.62 × 0.315 m with `body_offset_x = −0.10`, so the body
spans x −0.41 to +0.21. The drive axle is 10 cm forward of the geometric
centre; the LiDAR is at the back. The Nav2 footprint
`[[0.24,0.2025],[0.24,−0.2025],[−0.44,−0.2025],[−0.44,0.2025]]` is wider than
the body because it covers the wheels (0.415 m across), and its rear corners
sweep 0.484 m from `base_link`, which is why `inflation_radius` is 0.50.

TF publishers: `robot_state_publisher` (static URDF joints), the EKF
(`odom→base_link`), slam_toolbox (`map→odom` at 50 Hz). The aggregate `/tf`
rate of about 79.6 Hz is the single best health metric for this Jetson: when
CPU is exhausted, slam_toolbox's TF publisher is the first thing to starve.

## 3. The launch file

[`launch/rover_bringup.launch.py`](../rover_bringup_extended_slam/launch/rover_bringup.launch.py)
is the only launch file. It starts the bridge immediately and everything else
5 s later (so the ESP32 has finished its boot and the IMU is streaming), then
the frontier explorer at 25 s (so Nav2 is active and a first map exists before
it runs its one and only frontier search).

### Arguments

| Argument | Default | Effect |
|---|---|---|
| `mode` | `slam` | `slam` builds a map; `nav` localises on a saved pose-graph and runs Nav2 |
| `localization` | `slam_toolbox` | nav mode backend; `amcl` is kept for comparison but drifts during in-place turns |
| `posegraph` | `""` | base path of the `.posegraph`/`.data` pair, required in nav mode |
| `map` | `""` | map YAML, only for `localization:=amcl` |
| `use_exploration` | `false` | nav mode: also run the mapping SLAM node and `explore_lite` |
| `use_pose_autoinit` | `true` | nav mode: seed `/initialpose` from the last saved pose if the map matches |
| `use_tof` | `true` | run the ToF belt nodes (publish-only; Nav2 reacts via `range_layer`) |
| `teleop` | `false` | arms the WiFi-loss stop guard in `wifi_watchdog` |
| `use_rviz` | `false` | RViz2 with `/workspaces/isaac_ros-dev/default.rviz` |
| `lidar_port` | `/dev/ttyUSB0` | the one CP2102 on the rover |
| `arduino_port` | `/dev/ttyMOTOR` | udev symlink that follows the board's serial number, see §9 |
| `map_basename` | `live_map` | basename for `/save_all_maps` |

`use_init_movement` and `init_duration_s` from the older Python package are no
longer declared.

### Which node runs in which mode

| Node | slam | nav | nav + exploration |
|---|---|---|---|
| bridge, robot_state_publisher, zupt_node, uptime_monitor, LiDAR, scan_deskew, lidar_scan_filter, wifi_watchdog, EKF | ✔ | ✔ | ✔ |
| tof_belt_node, tof_down_filter (`use_tof`) | ✔ | ✔ | ✔ |
| `async_slam_toolbox_node` (mapping) | ✔ | | ✔ |
| `localization_slam_toolbox_node` | | ✔ | |
| map_saver | ✔ | | ✔ |
| Nav2 (`navigation_launch.py`) | | ✔ | ✔ |
| localization_monitor, pose_tracker, pose_initializer | | ✔ | ✔ |
| explore_lite (at t+25 s) | | | ✔ |

`zupt_node` is launched with `enabled` = (mode == slam). In nav mode it is a
pure pass-through but it must stay in the launch: it is the only publisher of
`/imu/filtered`, which both the EKF and `scan_deskew` consume.

The launch also sets `OPENCV_OPENCL_RUNTIME=disabled` for every child so
JetPack's CUDA-enabled OpenCV cannot silently dispatch slam_toolbox's matrix
ops to the GPU.

## 4. Nodes in `rover_bringup_extended_slam`

All twelve are C++ (`ament_cmake`, Release by default, `-Wall -Wextra
-Wpedantic`). Six are ports of Python originals with identical node names,
topics and parameters; the other six were written new in C++. Each is a
single-file node under `src/`.

### 4.1 `arduino_encoder_twist_bridge` (src/arduino_encoder_twist_bridge.cpp)

The only process that talks to the motor controller. One serial port, one
reader thread, three ROS timers.

**Outbound.** Subscribes `/cmd_vel`. For each Twist it applies, in order:
a small forward/reverse yaw trim when the command is nearly straight
(`fwd_w_trim` 0.0, `rev_w_trim` −0.015, gated by `w_trim_gate` 0.05 and
`v_min_for_trim` 0.05), then `yaw_sign` (−1.0), then `cmd_w_scale` (0.85), and
`invert_linear` (negates `linear.x` so +x drives the drive-wheel end first,
casters trailing). It writes `CMD <v> <w>\n` with three decimals under a mutex
that covers only the `write()` syscall. A 20 Hz watchdog sends `CMD 0 0` when
`/cmd_vel` has been silent for `cmd_timeout_s` (0.8 s) and keeps re-sending it
at 10 Hz until commands resume.

**Inbound.** The reader thread (`rx_loop`) never takes the TX mutex. It
reassembles lines and dispatches:

- `IMU …` lines are parsed and republished on `/imu/data` as
  `sensor_msgs/Imu` with configurable per-axis remap/sign (identity on this
  rover) and fixed covariances (`imu_orient_cov` 0.01, `imu_gyro_cov` 0.0003,
  `imu_accel_cov` 0.04).
- `ENC seq L R` lines drive odometry. Counts → revolutions (`counts_per_rev`
  267) → distance (`wheel_diameter_m` 0.120) → per-wheel velocity over the
  arrival-time delta. `v = (vR+vL)/2`, `w = (vL−vR)/wheel_base_m` (0.37; the
  sign is inverted relative to textbook because the raw wheel topics are in
  the firmware's mirrored frame). A `spin_yaw_scale` (1.0, so inert) exists
  for pure spins. The result is published at the encoder sample's own
  timestamp, deduplicated so the 50 Hz publish timer emits one message per
  10 Hz encoder report, as `/wheel/odom_twist` (twist only, pose zero) plus
  `/wheel/left_velocity` and `/wheel/right_velocity`.
- Anything else is a firmware diagnostic line and is forwarded to
  `/rover/link_status` prefixed `FW `. `WATCHDOG_TRIGGERED`, `IMU_STALL`,
  `IMU_FAIL` and the boot banner are escalated to ROS warnings/errors.

**Velocity-gated yaw covariance.** Skid-steer wheels slip during turns, so the
wheel-derived yaw rate is only trustworthy when turning slowly. When the last
commanded |ω| is at or above `vyaw_fusion_gate` (0.08 rad/s) the published
vyaw covariance is inflated from 0.02 to 100.0, which makes the EKF ride on
the gyro instead. The gate was lowered from 0.15 after bags showed typical
steering (0.138 rad/s) sat inside the slipping band; the largest odometry
step fell from 0.350 m to 0.016 m.

**Guards against fabricated motion.** Encoder deltas are timed by arrival. If
a backlog is drained in a burst, elapsed time collapses while the count delta
still covers real driving, which once injected −1766 rad/s into the EKF and
moved the pose 54.9 m. Two independent floors prevent that now: samples
closer than `min_dt_s` (0.02 s) re-baseline instead of computing a velocity,
and any sample implying a wheel speed above `max_wheel_speed_mps` (2.0; the
firmware caps at 1.25) is rejected and never published. On link loss the
kernel buffer is flushed on recovery (`INPUT_FLUSHED`) so stale samples are
not replayed. The serial port is opened, left alone for 2.5 s while the ESP32
boots, then flushed, so the start-up buffer overflow no longer registers as an
encoder gap.

**Link diagnostics.** ENC sequence numbers detect reboots (`ESP32_REBOOT`,
counts re-baselined) and lost lines (`ENC_GAP`). A 1 Hz timer publishes
`STATS enc= imu= gaps= reboots= link=` and `LINK_UP`/`LINK_DEAD`. Two timers
split "data not arriving" from "we are too slow": `READ_IDLE` means the kernel
had nothing for ~1 s; `SLOW_PROCESS` means parse/publish took over 250 ms.
`SLOW_WRITE` flags a `write()` that took over 100 ms. `/rover/cmd_sent`
carries the exact post-scale `v, w` written to the wire so a discrepancy can
be attributed to ROS, the bridge, or the firmware.

All numeric parameters are declared dynamically typed and coerced, because
rclcpp (unlike rclpy) would otherwise crash on `counts_per_rev: 267` arriving
as an integer. Do not pass the `axis_*_src` letters through the launch file:
YAML 1.1 turns a bare `y` into `true`.

### 4.2 `scan_deskew` (src/scan_deskew_node.cpp)

Motion-compensates each LiDAR scan with the gyro. A 10 Hz 2D scan takes
~100 ms, during which a turning rover rotates several degrees; treating all
rays as simultaneous smears walls and leaves phantom costmap marks.

Per scan: buffer `/imu/filtered` angular_velocity.z (400 samples); build a
trapezoidal cumulative-yaw integral over the scan window, extrapolating the
rate constantly at the edges (the IMU pipeline typically lags the scan by tens
of ms); shift each ray's angle by the yaw accumulated up to its capture time;
resample the (corrected angle, range) pairs back onto the original angle grid
with a faithful port of `numpy.interp`, including its NaN ladder for pairs of
"no return" rays. Translation is not compensated (under 2 cm per scan indoors).

If the yaw change across the scan is under `min_yaw_deskew_rad` (0.003) the
scan is republished untouched. If the IMU stream is more than 0.5 s stale the
scan passes through with a throttled warning (`drop_on_imu_gap` false), so
deskewing can never block the scan stream. `stamp_offset_s` exists for drivers
that stamp at scan end. A 5 s stats line counts deskewed/passthrough/dropped.

### 4.3 `lidar_scan_filter` (src/lidar_scan_filter.cpp)

Sanitises `/scan_deskewed` into `/scan_filtered`, the topic every consumer
uses. Steps: NaN/inf and out-of-band ranges (outside 0.20–25.0 m) become +inf;
a median-of-3 spike filter where all three neighbours are finite; an optional
inter-scan quality gate with hysteresis (valid-ray fraction under 0.70 or more
than 25 % of comparable rays changing by over 0.30 m marks a scan bad; two bad
scans suppress output, three good ones restore it); optional stride
downsampling. On this rover the quality gate and downsampling are both
disabled in the launch: slam_toolbox and the costmaps do better with the full
scan, and the deskew node removed the smear the gate was designed to hide.

### 4.4 `zupt_node` (src/zupt_node.cpp)

Zero-velocity update between `/imu/data` and `/imu/filtered`. When the wheels
report |vx| under `vx_threshold` (0.03 in the launch) and the raw gyro reports
|ωz| under `vyaw_threshold` (0.05) for `stationary_delay` (0.2 s), it zeroes
angular_velocity.z and sets its covariance to `zupt_covariance` (1e-4), which
stops gyro bias from walking the EKF yaw while parked. Stationarity is judged
from measured signals, not `/cmd_vel`, because a silent command publisher left
the old trigger frozen. It is disabled (pass-through) in nav mode because the
drivetrain's pulsed low-speed turns let the gyro dip under the threshold
between pulses, which erased real rotation mid-turn. Toggle live with
`ros2 param set /zupt_node enabled false`.

### 4.5 `wifi_watchdog` (src/wifi_watchdog.cpp)

Layer-2 stop guard. Two independent checks on a 20 Hz timer: the wireless
interface's sysfs `operstate` (auto-detected; only `down` counts as lost) and a
`/cmd_vel` heartbeat (`deadman_secs` 5.0). When either trips it publishes
`Twist(0,0)` to `/cmd_vel` at `stop_flood_hz` (100) from a timer that is
otherwise cancelled, so it costs nothing while idle. The WiFi check is armed
only with `teleop:=true` (autonomy runs on-board and a WiFi blip must not stop
a mission); the deadman is launched disabled because Nav2 legitimately pauses
`/cmd_vel` for seconds during planning and recovery. Both can be toggled live
via parameters. The flood distinguishes its own zero Twists from an operator's
non-zero ones so teleop resuming clears the condition.

### 4.6 `map_saver` (src/map_saver.cpp)

Provides `/save_all_maps` (`std_srvs/Trigger`). One call writes three
representations of the live map into `map_dir`: the slam_toolbox pose-graph
(`.posegraph` + `.data`, needed for localisation mode), slam_toolbox's own
`.yaml`/`.pgm`, and `nav2_map_server map_saver_cli`'s `<name>_nav2.yaml`/`.pgm`.
Each step is retried up to `max_retries` (3) and verified by file existence
within `verify_timeout_s` (4 s). Runs on a two-thread executor with a
reentrant callback group because the service handler blocks while waiting for
other services. Result goes to `/map_saver/last_save_status`. Saving is manual
only; the old auto-save trigger was deleted with the node it depended on.

### 4.7 `localization_monitor` (src/localization_monitor.cpp)

Nav-mode-only, report-only scan-vs-map agreement score. slam_toolbox in
localisation mode publishes no confidence value, so a pose that has silently
slid is invisible until someone looks at RViz. At `eval_rate` (2 Hz) it
transforms the current `/scan_filtered` into the map frame via TF, and for each
finite ray under `max_eval_range` (6 m) classifies the endpoint: within
`match_tolerance_m` (0.10, a 2-cell search) of an occupied cell = on a wall;
in free space = something is there that the map does not know about;
otherwise ambiguous. Publishes `/rover/localization_score` = on_wall /
(on_wall + in_free), `/rover/localization_drop` = in_free / total (what a
dynamic-obstacle filter would discard), and `/rover/localization_ok` (score ≥
0.60). The drop fraction is being collected across normal household traffic
so a future dynamic-obstacle filter's safety valve can be set from data (one
person at 1 m is ~8 % of a scan).

### 4.8 `pose_tracker` (src/pose_tracker.cpp)

Nav mode. Samples TF `map→base_link` at 5 Hz and writes
`{"map_name","x","y","qz","qw","stamp"}` atomically (temp + rename) to
`last_pose.json`. TF is used instead of slam_toolbox's `/pose` because `/pose`
only updates when a scan is processed, so a final parking rotation below the
heading threshold never reached it and the saved heading was stale. Nothing is
written until an `/initialpose` has been seen this session, so a start-up
guess can never overwrite the last good save.

### 4.9 `pose_initializer` (src/pose_initializer.cpp)

The counterpart: at nav start, read `last_pose.json` and, only if its
`map_name` matches the basename of the `posegraph` argument, publish it once
on `/initialpose` (same covariance RViz uses) after the localiser has
subscribed plus `settle_delay_s` (2 s). Mismatch, missing or unparseable file
→ do nothing; a manual SET POSE always wins. Its latched status
(`NO_SAVED_POSE`, `MAP_MISMATCH`, `WAITING`, `SEEDED …`) is on
`/rover/pose_autoinit` for the cockpit.

### 4.10 `uptime_monitor` (src/uptime_monitor.cpp)

Every mode. Publishes `/rover/uptime` (JSON: Jetson boot time from
`/proc/stat` `btime`, Jetson and stack uptime, estimated last-off time, boot
count) at 1 Hz, appends one `BOOT …` line per new kernel boot to
`boot_history.log` (an entry the operator did not cause is evidence of an
unwanted reset), and rewrites a heartbeat file every 30 s so the next boot can
estimate when the previous session died.

### 4.11 `tof_belt_node` (src/tof_belt_node.cpp)

Driver for six VL53L1X time-of-flight sensors on the Jetson's `/dev/i2c-7`
behind a TCA9548A mux at 0x70, one sensor per channel, all at the factory
address 0x29. The sensor driver is a minimal port of ST's ULD (the 91-byte
default configuration blob, short distance mode, timing budget table,
free-running inter-measurement mode, status remap table) using Linux
`I2C_RDWR` ioctls directly, no external library.

Design rules, in priority order: the belt is additive (a sensor that fails
init or accumulates `fault_strikes` read failures is quarantined and retried
every `quarantine_retry_s`; the other five continue; bringup never blocks);
"I see nothing" is a measurement (SIGNAL_FAIL and out-of-bounds are published
as `max_range`, which is what lets `RangeSensorLayer` clear a cone, because no
LiDAR ray will ever raytrace away a mark on a sock); never publish a guess
(sigma-fail, hardware-fail and wrap-around readings are dropped, not rounded);
all geometry lives in `tof_belt.yaml` and the URDF. A reading below
`min_range` is published as `min_range` (something is there). Each sensor
publishes `sensor_msgs/Range` on `/tof/<name>`; health goes to
`/rover/tof_status` (`TOF_READY`, `TOF_FAULT`, `TOF_QUARANTINED`,
`TOF_RECOVERED`, 1 Hz `STATS`). A mux write that times out (errno 110) is a
jammed SDA line on that branch and is quarantined rather than taking the belt
down.

### 4.12 `tof_down_filter` (src/tof_down_filter.cpp)

The down-looking sensor always sees the floor, so fed raw into a costmap it
would mark a permanent phantom 0.39 m ahead. This node turns its slant range
into information: reading ≈ `floor_baseline_m` (0.53) is clear floor; below
`obstacle_threshold_m` (0.52) means something stands on the floor (an object
shortens the reading by about 1.5× its height, so 1 cm catches objects over
~7 mm tall); above baseline + `drop_margin_m` (0.08) or no return means the
floor fell away. Both are debounced over `consecutive_readings` (3). The
output on `/tof/down_obstacle` is republished on the flat
`tof_down_proj_link` with the range projected onto the floor
(`slant × cos 43°`) so the 2D `RangeSensorLayer` places the mark at the right
horizontal distance; a drop is marked at the near edge of the cone so the rover
stops short of the hole. `/rover/tof_drop` is a latched Bool.

## 5. Third-party nodes the launch starts

| Node | Package | Role and notable settings |
|---|---|---|
| `rplidar_node` | `rplidar_ros` | RPLidar A3M1 at 256000 baud, `scan_mode: Standard` (4 kHz, ~400 pts). `Sensitivity` mode (8–16 kHz) is untried. Several comments still say "A1M8"; that was a documentation error. |
| `robot_state_publisher` | | publishes the URDF's fixed transforms |
| `ekf_node` | `robot_localization` | see §6.2 |
| `async_slam_toolbox_node` | `slam_toolbox` | mapping; scan ingestion on its own thread so a slow Ceres solve does not back up matching |
| `localization_slam_toolbox_node` | `slam_toolbox` | nav mode; loads the serialized pose-graph, continuous scan matching; chosen over AMCL because it stays rigid during in-place rotation. Overrides: `scan_topic /scan_filtered`, looser `link_match_minimum_response_fine` 0.45 / `loop_match_minimum_response_fine` 0.40 for re-localisation, `loop_search_space_dimension` 8.0, `transform_publish_period` 0.02 |
| Nav2 `navigation_launch.py` | `nav2_bringup` | planner, smoother, controller, behavior, bt_navigator, velocity_smoother, waypoint_follower, lifecycle manager; no composition |
| `explore` | `explore_lite` (robo-friends/m-explore-ros2, built in-workspace) | frontier exploration; needs `-r __node:=explore` so the `explore:` params block applies |
| `rviz2` | | optional |

## 6. Configuration files

### 6.1 `nav2_params.yaml`

The file carries the reasoning for every value inline; this is the summary.
The stack is ROS 2 **Humble**. The file was partly written against Iron/Jazzy
parameter names, which Humble silently ignores, and two multi-month bugs came
from that. Before trusting any parameter here, run `ros2 param list /<node>`.
Iron-only names kept for a future upgrade are grouped and labelled.

**Planner.** `NavfnPlanner`, Dijkstra (`use_astar: false`), tolerance 0.5,
`allow_unknown`. NavFn never validates the start pose, which is exactly why it
keeps working at pinch points where the costmap already considers the rover in
collision. Smac Hybrid-A* and Lattice were tried and deadlock there (planner
and every recovery abort together).

**Smoother.** `SimpleSmoother`, `tolerance 1e-6`, `max_its 200`, `w_data 0.3`,
`w_smooth 0.25`. It still rejects ~⅓ of plans at the same fixed doorway
coordinates; that is geometry, not a smoother parameter.

**Controller.** `RegulatedPurePursuitController` at `controller_frequency 15`
(20 starved TF, 10 was too slow at speed). `desired_linear_vel 0.32`,
`lookahead_time 3.0` with `min/max_lookahead_dist` 0.40/0.90,
`curvature_lookahead_dist 0.40`, `regulated_linear_scaling_min_speed 0.20`
(0.15 pinned 44 % of driving time and made the inner wheel click),
`use_rotate_to_heading` with **`rotate_to_heading_min_angle 0.52`** (the
doorway fix: the 0.68 × 0.42 m footprint needs 0.77 m of width on the diagonal
but only 0.42 m square-on, and at the old 0.8 rad threshold the rover never
squared up), `allow_reversing false`, `max_allowed_time_to_collision_up_to_carrot 0.30`.
`inflation_cost_scaling_factor 2.0` and `cost_scaling_dist 0.50` must equal
the inflation layer's values: RPP inverts the inflation formula from cell cost
to estimate distance, and the Nav2 defaults (3.0/0.6) against a costmap built
with 2.0/0.50 made it over-brake ~1.5× everywhere. Goal tolerance 0.35 m /
0.13 rad; progress checker 3 cm in 20 s.

**Costmaps.** Both use the explicit `footprint` above with `footprint_padding
0.01`, a 2D `ObstacleLayer` on `/scan_filtered` (`inf_is_valid`, raytrace 6.0,
obstacle 5.5, `footprint_clearing_enabled`), a `RangeSensorLayer` for the ToF
belt placed **between** obstacle and inflation layers so ToF marks are inflated
identically to LiDAR marks (`clear_on_max_reading`, `mark_threshold 0.6`,
`clear_threshold 0.05` so a mark survives the final approach when the cone
rises above a short object, `no_readings_timeout 0` so one faulted sensor
cannot stale the whole layer), and an `InflationLayer` with
`cost_scaling_factor 2.0`, `inflation_radius 0.50`. Local: 3 × 3 m rolling at
0.05 m, update 10 Hz, publish 10 Hz, `always_send_full_costmap: true` (**must
stay**: `behavior_server` subscribes to `costmap_raw` and Humble's subscriber
only understands full messages; `false` froze its window at the start pose and
failed every recovery). Global: static layer plus the same, update 4 Hz to
match the BT replan rate, `resolution 0.025` (currently inert because the
static layer forces the map's 0.05; a 0.025 map would quadruple its cost).
`/tof/down_obstacle` feeds only the local costmap.

**behavior_server.** `global_frame: odom` is the fix for the recovery bug:
Humble has one frame parameter and it is both the pose-lookup frame and the
frame the collision checker expects the (local, odom-framed) costmap in. Set
to `map`, every Spin/BackUp tested a map-frame pose against an odom grid and
failed with "Pose Goes Off Grid". `simulate_ahead_time 0.5`,
`max/min_rotational_vel 0.30` (stock 0.10 is below the drivetrain's stiction
floor, producing "motor taps and no movement"), `rotational_acc_lim 1.5`.

**bt_navigator.** `default_nav_to_pose_bt_xml` (the Humble name;
`navigate_to_pose.bt_xml_filename` is Iron and was silently ignored, so the
custom tree and the smoother never loaded) points at the installed
`navigate_w_smoothing_and_recovery.xml`. `bt_loop_duration 100`.

**velocity_smoother.** `max_velocity [0.32, 0, 0.28]` (the linear value must
track `desired_linear_vel`; it silently clipped the controller once),
`min_velocity [−0.21, 0, −0.28]`, `max_accel/decel ±[0.8, 0, 1.5]` matching
the firmware's slew ramp so planned stops no longer overshoot, `OPEN_LOOP`.

**explore.** `costmap_topic: /map` (slam_toolbox's raw grid; the inflated Nav2
global costmap has almost no `FREE_SPACE` cells and its raytracing erases the
unknown boundary frontiers are made of), `planner_frequency 0.33`,
`progress_timeout 30`, `min_frontier_size 0.2`.

**amcl.** Retained and tuned (`DifferentialMotionModel`, 180 beams,
`update_min_d 0.05`, `update_min_a 0.1`) but not used by default.

### 6.2 `ekf_config.yaml`

`robot_localization` EKF at 30 Hz (50 Hz missed its deadline and starved
slam_toolbox; 20 Hz was the original), `two_d_mode`, world frame `odom`.
Inputs: `odom0 = /wheel/odom_twist` fusing vx and vyaw only (the bridge's
velocity-gated covariance decides when vyaw counts); `imu0 = /imu/filtered`
fusing yaw and vyaw, `imu0_relative: true` (no magnetometer, so no absolute
heading), accelerations off,
`imu0_remove_gravitational_acceleration: false` because the BNO085's linear
acceleration report is already gravity-free and removing it twice injects a
phantom 9.81 m/s². Rejection thresholds pose 3.0 / twist 1.5. A commented
`odom1` block records a retired cuVSLAM input.

### 6.3 `slam_toolbox_params.yaml`

Mapping mode (the launch overrides `mode: localization` for nav), input
`/scan_filtered`, resolution 0.05, `map_update_interval 2.0`,
`transform_publish_period 0.02` (the 50 Hz TF publisher), laser range
0.2–12 m, `minimum_travel_distance 0.05`, `minimum_travel_heading 0.087`.
`link_match_minimum_response_fine` lowered 0.65 → 0.58 so mid-turn deskewed
scans are not rejected (rejection left only the gyro to integrate pose);
`loop_match_minimum_response_fine` raised 0.45 → 0.55 for stricter loop
closure. Ceres solver, `SPARSE_NORMAL_CHOLESKY`, Levenberg-Marquardt.

### 6.4 `navigate_w_smoothing_and_recovery.xml`

The stock Humble replan-and-recover tree with two changes. First, a
`SmoothPath` step after `ComputePathToPose`, wrapped in a
`Fallback`/`AlwaysSuccess` so smoothing failure can never fail navigation
(NavFn's 45° staircase paths otherwise make RPP weave). Replanning runs in a
`RateController` at 4 Hz (1 Hz meant 25 cm of travel before the rover could
react to anything). `FollowPath` gets 3 retries with a 1 s `Wait` as its
local recovery rather than a local-costmap clear, because clearing erased the
only record of a low obstacle the ToF belt had marked. Second, the recovery
`RoundRobin` is reordered and runs 12 retries: a ±0.5 rad sweep (three Spins
ending on the original heading, which drags the ToF cones across whatever is
there) → `BackUp 0.40 m @ 0.12 m/s` → clear local then global costmap →
`Spin 1.57` → `BackUp 0.70 m` → `Wait 3`. Backing up comes before spinning
because the space just driven through is provably clear, and all speeds sit
above the stiction floor. Costmap clearing was demoted from first to third for
the same ToF reason.

### 6.5 `tof_belt.yaml`

Bus 7, mux 0x70, `rate_hz 15`, `timing_budget_ms 50` (33 returned SIGNAL_FAIL
on matte black objects), short distance mode, 27° FoV, 16 × 16 ROI,
`fault_strikes 10`, `quarantine_retry_s 30`. Per sensor: channel, frame,
topic, `min_range 0.04`, and a `max_range` cap = lens height / tan(15.5°) so
the floor stays outside the cone: front pair (ch 7, 4) 0.47 m, corners (ch 3,
0) 0.55 m, rear (ch 2) 0.47 m, down (ch 1) uncapped at 1.00 m because it goes
through `tof_down_filter`. Channels 5 and 6 are spare. The down-filter block
holds `floor_baseline_m 0.53`, `obstacle_threshold_m 0.52`, `tilt_deg 43`,
`drop_margin_m 0.08`, `consecutive_readings 3`.

### 6.6 `rover.urdf.xacro`

See §2. Sensor extrinsics are parameters at the top of the file so they can be
edited without touching the link tree. Changing `lidar_*` or any `tof_*`
mount requires only a restart (config is symlink-installed), not a rebuild.

## 7. ESP32-S3 motor/IMU firmware and the serial protocol

[`firmware/esp32s3_motor_imu/enc_ros_cmd_vel_motor_control_pid_esp32s3.ino`](../firmware/esp32s3_motor_imu/enc_ros_cmd_vel_motor_control_pid_esp32s3.ino)
(rev-I). Arduino core ≥ 3.0 for ESP32-S3-N16R8, USB CDC on boot, libraries
Adafruit BNO08x + BusIO + Unified Sensor. Flashed from the Jetson **host** with
`arduino-cli`, never from inside the container.

**Pins.** MDDS30 direction IN1 = GPIO 5 (left), IN2 = GPIO 4 (right); the
direction wires are physically crossed at the driver and the constants match
reality. Do not swap them back: straight driving would still work but spins
would reverse and the PID would fight itself (the historic 12× runaway-spin
bug). PWM AN1/AN2 = GPIO 6/7 at 20 kHz 8-bit. Encoders left A/B = 8/9, right
A/B = 10/11, `CHANGE` interrupt on channel A only (2× decode, 267 counts per
wheel revolution; switching to 4× quadrature silently breaks bridge odometry).
BNO085 in UART mode (PS1 high, PS0 low) at 3 Mbaud on GPIO 1 (TX) / 42 (RX),
reset on 39.

**Control loop (core 1, 50 Hz).** Parses `CMD v w`. Slew-limits the
setpoints (`MAX_LIN_ACCEL 0.8 m/s²`, `MAX_ANG_ACCEL 3.0 rad/s²`) so stops are
symmetric instead of skids. Converts to per-wheel targets
(`v ∓ w·0.185`), multiplies by `LEFT_WHEEL_TRIM 1.00` / `RIGHT_WHEEL_TRIM
0.969` (applied to the setpoint so the PID holds it, not as a post-PID offset
the integral would erase). Per-wheel velocity from encoder deltas, PID
(`KP 0.6, KI 0.40, KD 0.02`, output × 150) on top of a feedforward of
`target / MAX_SPEED_MPS (1.25) × 255`. For in-place spins a `TURN_BIAS_PWM 47`
stiction breaker is added, plus a 200 ms `KICK_PWM 27` at spin onset. Output
clamped to ±180 with anti-windup. The integral is reset on any command step
over 0.05.

**Safety.** Motors are disabled until the first `CMD` arrives. A 500 ms
serial watchdog (`WATCHDOG_MS`) and a 1 s `CMD_TIMEOUT_MS` both hard-stop the
motors and reset the ramp. `WATCHDOG_TRIGGERED` is printed at most every
500 ms; because the bridge's `cmd_timeout_s` is 0.8 s, this line is routine
noise on every Nav2 pause, not a fault.

**IMU task (core 0).** Owns all BNO085 UART I/O so a late SHTP frame can
never stall the control loop. Enables the game rotation vector and linear
acceleration reports at 100 Hz. The chip's native gyro reports return zeros on
this unit, so body angular velocity is derived from the quaternion delta
(`ω ≈ 2·vec(q_prev⁻¹·q_cur)/dt`). A 3 s stall watchdog performs a full re-init
(reset pin, UART, reports). The snapshot is shared under a spinlock with a
dirty flag so each sample is emitted exactly once.

**Protocol (115200 8N1 over USB CDC).**

| Direction | Line | Rate |
|---|---|---|
| Jetson → MCU | `CMD <v m/s> <w rad/s>` | 50 Hz while commanded, 10 Hz stop-flood when idle |
| MCU → Jetson | `ENC <seq> <left> <right>` absolute counts | 10 Hz |
| MCU → Jetson | `IMU <seq> <qw> <qx> <qy> <qz> <gx> <gy> <gz> <ax> <ay> <az>` | ≤100 Hz, only on fresh data |
| MCU → Jetson | `PID VERSION READY …` banner, `WHEEL_DIAMETER_M=…`, `IMU_OK`/`IMU_FAIL`/`IMU_STALL`/`IMU_RESET`, `WATCHDOG_TRIGGERED` | events |
| MCU → Jetson (debug only) | `PWM l r`, `CMD_ECHO v w` | gated by `DEBUG_SERIAL` |

Angular scaling (`cmd_w_scale`, `yaw_sign`) lives on the Jetson so it can be
tuned without reflashing. `WHEEL_DIAMETER_M`, `WHEEL_BASE_M` and
`COUNTS_PER_REV` must match the bridge's launch parameters exactly.

## 8. `rover_web`: the phone cockpit

A FastAPI + rclpy package. `ros2 run rover_web server` serves
`http://<jetson>:8080`. All state is server-side, so a phone losing WiFi does
not stop autonomy. It needed no changes for the C++ port: its only ROS
touchpoints are served by slam_toolbox, Nav2 and the EKF.

**`server.py`.** Routes `GET /` (the single-page app), `GET /api/maps`,
`GET /api/state`, WebSocket `/ws`. Every client gets a `state` and `map`
snapshot on connect; a 0.2 s broadcaster pushes `pose` at 5 Hz, `state` and
(when changed) `map` at 1 Hz, `toast`s, and `log` lines when the live log
panel is on. Client messages: `ping`, `joy {v,w}`, `joy_stop`, `estop`,
`start_slam {name}`, `start_nav {map}`, `stop_all`, `explore_start`,
`explore_stop`, `save_map`, `save_place {name}`, `goto {name}`,
`set_pose {name}`, `delete_place`, `cancel_goal`, `clear_goal`,
`set_setting`/`set_autoinit`. Mode-changing operations run under an asyncio
lock; a second request while one is running is refused with a "Busy" toast.
Taking the joystick in nav mode cancels the active goal first. Any WebSocket
disconnect calls the joystick stop. Binds `0.0.0.0` with no authentication:
anyone on the LAN can drive the rover.

**`ros_iface.py`.** The rclpy node (`rover_web`) on a daemon spin thread.
Subscribes `/map` (transient-local), `/odometry/filtered`,
`/rover/pose_autoinit`, `/rover/uptime`; publishes `/cmd_vel` and
`/initialpose`; `navigate_to_pose` action client; `/slam_toolbox/serialize_map`
service client. Joystick caps `WEB_V_MAX 0.25 m/s`, `WEB_W_MAX 0.80 rad/s`; a
20 Hz tick republishes the last joystick value while it is under
`DEADMAN_S 0.5` old, then sends a burst of five zero Twists. Renders the map
to an 8-bit PNG with a stdlib-only encoder. Reports a goal as
`failed (nav2 gone)` if Nav2 disappears for 5 s mid-goal.

**`launch_manager.py`.** Owns the ROS launch lifecycle. SLAM:
`ros2 launch rover_bringup_extended_slam rover_bringup.launch.py mode:=slam use_rviz:=false`.
NAV: `… mode:=nav localization:=slam_toolbox posegraph:=<maps_dir>/<name> use_rviz:=false use_pose_autoinit:=<bool>`,
requiring the `.posegraph`/`.data` pair. Explore from the cockpit launches
Nav2 separately then `explore_lite` with the `__node:=explore` remap (the
launch file's own `use_exploration` path is not yet wired to the button).
Children are spawned with `start_new_session=True` so a dropped SSH session
does not kill the stack; stop is SIGINT (12 s) → SIGTERM (4 s) → SIGKILL on the
process group, followed by a `pkill -9` sweep over a fixed list of node
executable names and a `ros2 daemon` restart. Writes `last_run_log.txt`
(truncated on every start; copy it before restarting after an incident) and
`rover_web_settings.json`.

**`rover_places`** (separate `ament_python` package, imported by `server.py`
for save-place and go-to). `place_store.py` keeps named poses in one JSON file
keyed by place name and map name, so the same name can exist on several maps;
it offers get, list, resolve (exact, then case-insensitive, then prefix
match), put, delete and rename. The console scripts `save`, `goto`, `list`,
`delete` and `rename` wrap it for terminal use, with `goto` sending a
`navigate_to_pose` goal and reporting progress. The CLI scopes names by the
`ROVER_MAP` environment variable or `--map`; without either it files under
`default`, which has caught the author out more than once. The cockpit passes
the loaded map's name explicitly, so it never has that problem.

**`static/app.js`.** Joystick: 61 px travel maps to ±0.25 m/s and ±0.8 rad/s,
one message per 80 ms, release or tab-hide sends `joy_stop`. Map canvas with
saved places and a heading arrow; SLAM / NAV / VOICE tabs; a STOP bar that is
never disabled while every other control is disabled during a mode change.

Pinned web dependencies that are known to work together:
`fastapi==0.115.12`, `uvicorn[standard]==0.34.0`, `websockets==12.0`
(starlette 0.46.x). Never put `from __future__ import annotations` in the
route file; FastAPI then treats the `WebSocket` argument as a query parameter
and every connection 403s.

## 9. Serial device naming on the Jetson

The motor/IMU ESP32-S3 enumerates as a generic "Espressif USB JTAG serial
debug unit", and Linux hands out `ttyACM` numbers in enumeration order. With
any second Espressif board on the bus the number can change between boots, and
the bridge would open the wrong device with nothing in any log naming the
cause. Two small files make the name follow the hardware instead:

- `tools/99-rover-serial.rules` (host udev) matches the board by its USB
  serial string, which on an ESP32-S3 is its MAC address, and creates
  `/dev/ttyMOTOR`; the CP2102 LiDAR adapter becomes `/dev/ttyLIDAR` by vendor
  and product id. The same file sets `ID_MM_DEVICE_IGNORE` for both vendors so
  ModemManager stops probing the ports with AT commands on hotplug, which it
  was observed doing at one line per second.
- `tools/link_serial_devices.sh` (run with `sudo` inside the container after
  each container start) recreates the same links from sysfs, because the
  container is started with `--device` for individual nodes and udev does not
  run inside it. Passing `--device /dev/ttyMOTOR:/dev/ttyMOTOR` to the
  container instead makes the script unnecessary.

Both files ship with a `PUT-YOUR-MOTOR-ESP32-SERIAL-HERE` placeholder; find
the real value with
`udevadm info -q property -n /dev/ttyACM0 | grep ID_SERIAL_SHORT`. The launch
file defaults `arduino_port` to `/dev/ttyMOTOR` and leaves `lidar_port` at
`/dev/ttyUSB0` because there is only ever one CP2102 on the rover.

## 10. Tools

`rover_bringup_extended_slam/tools/` is not installed by CMake; run by path.

The offline bag post-mortem script that produced the measurements quoted in
[CALIBRATION.md](CALIBRATION.md) is not part of this release.

| Tool | Where it runs | What it does |
|---|---|---|
| `record_nav.sh [all]` | container | `ros2 bag record` with the curated topic list (or `-a`). Start it only after the stack is up; rosbag2 cannot resolve a type before a publisher exists. |
| `rover_health.sh [dir]` | host, as root | boot id, nvpmodel, reset reason, `tegrastats`, filtered `dmesg -W` (USB, brownout, thermal, OOM); detects a reboot on exit. |
| `cpu_bench.sh <label> [s]` | container | A/B CPU and topic-rate capture per node for the still-unrun Python-vs-C++ comparison. |
| `lidar_cal.py --true-distance <m>` | container | Flat-wall PCA fit of the forward strip of `/scan_filtered`; prints measured wall distance and tilt against the URDF and the corrected `lidar_x`/yaw. See CALIBRATION.md. |
| `map_survey.py <map> [--places …]` | anywhere, no ROS (needs numpy) | Per-room area and dimensions from a saved map: distance-transform erosion at half the door width to separate rooms, BFS assignment, leak detection, optional place-name labelling and PNG. |
| `costmap_peek.py [--ascii]` | container | Nearest obstacle in the rover frame and an ASCII costmap, for verifying ToF marks without RViz. |
| `tof_probe.py [--watch] [--live]` | container, no ROS | Bench scan of the TCA9548A channels for the VL53L1X model ID, jammed-bus diagnosis, live distances via the Adafruit driver. Do not run it while `tof_belt_node` is up. |
| `link_serial_devices.sh` | container, sudo | Recreates `/dev/ttyMOTOR` and `/dev/ttyLIDAR` from sysfs serial numbers (§9). |
| `99-rover-serial.rules` | host | The udev rules the script mirrors, plus `ID_MM_DEVICE_IGNORE` for both USB vendors. |
| `wifi_link_logger.sh` / `.service` | host | Appends operstate, BSSID, signal, bitrate, IP and a gateway ping to `/var/log/wifi_link.log` every 2 s, fsynced, for diagnosing the "alive but unreachable" dropouts. |

`rover_bringup_extended_slam/calibration/` holds the drivetrain calibration
scripts; they are described in [CALIBRATION.md](CALIBRATION.md).

## 11. Diagnostic topics

| Topic | Publisher | Content |
|---|---|---|
| `/rover/cmd_sent` | bridge | the `v, w` actually written to serial |
| `/rover/link_status` | bridge | `STATS`, `LINK_UP/DEAD`, `ENC_GAP`, `ESP32_REBOOT`, `VELOCITY_REJECTED`, `READ_IDLE/RESUMED`, `SLOW_PROCESS`, `SLOW_WRITE`, `INPUT_FLUSHED`, `PORT_LOST`, `FW …` |
| `/rover/tof_status` | tof_belt_node, tof_down_filter | `TOF_READY/FAULT/QUARANTINED/RECOVERED`, `STATS`, `TOF_DROP_DETECTED/CLEARED` |
| `/rover/tof_drop` | tof_down_filter | latched Bool |
| `/rover/localization_score`, `_drop`, `_ok` | localization_monitor | scan-vs-map agreement |
| `/rover/pose_autoinit` | pose_initializer | latched status string |
| `/rover/uptime` | uptime_monitor | JSON boot/uptime record |
| `/map_saver/last_save_status` | map_saver | `OK:`/`FAIL:` summary |
| `/lookahead_collision_arc`, `/lookahead_point`, `/received_global_plan` | RPP | the arc actually collision-checked is the single best RViz display |
| `/plan_smoothed` | smoother | non-zero rate proves the custom BT is loaded |

## 12. Safety layers

1. **Firmware watchdog, 500 ms.** Independent of the Jetson. No `CMD` →
   motors off. Motors stay off until the first command after boot.
2. **Bridge command timeout, 0.8 s.** No `/cmd_vel` → explicit `CMD 0 0` at
   10 Hz. The bridge also refuses impossible encoder velocities so a serial
   glitch cannot teleport the pose.
3. **Cockpit joystick deadman, 0.5 s** (`ros_iface`). The phone is a
   stateless remote; manual driving never survives losing the pilot.
4. **`wifi_watchdog`.** Stop-flood on WiFi loss (teleop only) or long
   `/cmd_vel` silence.
5. **ESTOP.** The cockpit's STOP bar is never disabled; it cancels the goal,
   halts exploration and zeroes the joystick.
6. **Nav2 collision checking** on the lookahead arc and in every recovery
   behaviour, plus the ToF belt for obstacles under the LiDAR plane.

## 13. Known drift between comments and code

Noted during the documentation pass; none affects behaviour.

- `tools/tof_probe.py` `CHANNEL_MAP` labels/caps for channels 0, 1, 2 and 4
  disagree with `config/tof_belt.yaml`, which is authoritative.
- The header comment of `navigate_w_smoothing_and_recovery.xml` still
  describes the pre-ToF ladder and 1 Hz replanning.
- `config/ekf_config.yaml` says it is launched by `rover_bringup ekf.launch.py`;
  it is launched by `rover_bringup.launch.py` in this package.
- Several Nav2/SLAM comments say "A1M8"; the LiDAR is an A3M1.
- `rover_web/README.md` says the joystick is SLAM-only; it also works in NAV
  and cancels the goal.
- `rover_places/README.md` refers to a `rover_startup.sh` and a PyQt GUI that
  no longer exist; the cockpit replaced both. The CLI and store it documents
  are current.
