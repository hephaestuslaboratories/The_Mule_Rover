# Indoor SLAM rover on Jetson Orin Nano + ROS 2 Humble

A from-scratch autonomous indoor rover: skid-steer chassis, 2D LiDAR,
BNO085 IMU, a six-sensor time-of-flight safety belt, slam_toolbox for mapping
and localisation, Nav2 for planning and control, and a phone-browser cockpit.
Every layer between the motor driver and the behaviour tree was written or
tuned here, and every tuned value is backed by a recorded bag.

The project's goal was a "Pixhawk-equivalent or better" platform for mapping a
typical home (hardwood, tile, carpet) at about 0.25 m/s. It is a personal
research build, tuned to one specific rover; the numbers will not transfer to
another chassis without the calibration procedure in
[docs/CALIBRATION.md](docs/CALIBRATION.md).

- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md): what every node, config file,
  script and protocol does, and why.
- [docs/INSTALL.md](docs/INSTALL.md): what to install, the container
  question, dependencies, device permissions, firmware flashing.
- [docs/CALIBRATION.md](docs/CALIBRATION.md): how every measured constant was
  obtained and when to redo it.
- [rover_web/README.md](rover_web/README.md): installing and using the cockpit.

## Hardware

| Part | Notes |
|---|---|
| NVIDIA Jetson Orin Nano, NVMe, 25 W mode | runs the Isaac ROS dev container (ROS 2 Humble, aarch64) |
| Chassis 62 × 31.5 cm, 4.0 kg | drive axle 21 cm from the front; four swivel casters (rigid casters are disastrous for skid-steer) |
| 2 × Yellow Jacket gearmotors with hall encoders, 3601 Rhino 120 mm wheels, 0.37 m track | 267 counts per wheel revolution at 2× decode; sealed, do not oil |
| Cytron MDDS30 motor driver | sign-magnitude PWM, 3.3 V logic |
| ESP32-S3-N16R8 (motor/IMU controller) | USB CDC to the Jetson, 50 Hz wheel PID, 500 ms watchdog |
| BNO085 IMU | UART mode at 3 Mbaud; game rotation vector + linear acceleration at 100 Hz |
| Slamtec RPLidar A3M1 | 256000 baud, `Standard` scan mode (~400 points at 10 Hz), mounted at the rear, 0.56 m scan plane |
| 6 × VL53L1X ToF + TCA9548A mux | on the Jetson's `/dev/i2c-7`, belt at 13.5–15.5 cm plus one down-looker at 36.5 cm / 43° |
| Any phone or laptop browser on the same WiFi | the operator interface |

### ESP32-S3 pin map

| Function | GPIO |
|---|---|
| MDDS30 IN1 (left direction) | 5 |
| MDDS30 IN2 (right direction) | 4 |
| MDDS30 AN1 / AN2 (PWM) | 6 / 7 |
| Left encoder A / B | 8 / 9 |
| Right encoder A / B | 10 / 11 |
| BNO085 RX ← our TX | 1 |
| BNO085 TX → our RX | 42 |
| BNO085 reset | 39 |

IN1/IN2 are 5/4 and not 4/5 because the direction wires are physically
crossed at the driver; the code matches the wiring. Encoders are powered from
3.3 V. Free pins: 21, 38, 47.

## Software stack

```
ESP32-S3 firmware ──serial──► arduino_encoder_twist_bridge ──► /wheel/odom_twist, /imu/data
                                                                   │
RPLidar ──► scan_deskew ──► lidar_scan_filter ──► /scan_filtered    ▼
                                   │                 zupt_node ──► EKF (robot_localization) ──► odom→base_link
                                   ├──► slam_toolbox ──► /map, map→odom
                                   └──► Nav2 costmaps ◄── ToF belt (RangeSensorLayer)
Nav2: NavFn → SimpleSmoother → Regulated Pure Pursuit → velocity_smoother ──► /cmd_vel ──► bridge
Operator: rover_web (phone browser, :8080) ──► joystick /cmd_vel, navigate_to_pose goals, launch lifecycle
```

Third-party packages, none of which are vendored here: `slam_toolbox`,
`nav2_bringup` and friends, `robot_localization`, `rplidar_ros`,
`robot_state_publisher`, all from the Humble apt repositories, and
`explore_lite` from `robo-friends/m-explore-ros2` (not a distro package; clone
it into the workspace). The web cockpit needs `fastapi==0.115.12`,
`uvicorn[standard]==0.34.0`, `websockets==12.0`. The full list, and whether
you need NVIDIA's container, is in [docs/INSTALL.md](docs/INSTALL.md).

## Repository layout

```
rover_bringup_extended_slam/   the live ROS 2 package (ament_cmake, C++17)
  src/                         12 nodes, one file each
  launch/rover_bringup.launch.py
  config/                      nav2_params.yaml, ekf_config.yaml, slam_toolbox_params.yaml,
                               tof_belt.yaml, navigate_w_smoothing_and_recovery.xml, rover.urdf.xacro
  calibration/                 straight-line, turn and single-wheel calibration scripts
  tools/                       operations and diagnostic scripts (not installed by CMake)
rover_web/                     phone cockpit (FastAPI + rclpy)
rover_places/                  named-waypoint store and CLI (save / goto / list / delete / rename)
firmware/esp32s3_motor_imu/    the ESP32-S3 motor/IMU sketch
docs/                          INSTALL.md, ARCHITECTURE.md, CALIBRATION.md
```

The twelve nodes, in one line each (full detail in
[docs/ARCHITECTURE.md §4](docs/ARCHITECTURE.md#4-nodes-in-rover_bringup_extended_slam)):

| Node | Role |
|---|---|
| `arduino_encoder_twist_bridge` | the only process on the motor serial link: `/cmd_vel` → `CMD v w`; `ENC`/`IMU` lines → wheel odometry and `/imu/data`; link diagnostics; guards against fabricated motion |
| `scan_deskew` | rotates each LiDAR ray back to scan-start using the gyro so turns do not smear walls |
| `lidar_scan_filter` | NaN/band sanitising and median-of-3 spike removal → `/scan_filtered` |
| `zupt_node` | zero-velocity update: zeroes gyro yaw rate when both wheels and gyro say parked (mapping only) |
| `wifi_watchdog` | stop-flood on WiFi loss (teleop) or long `/cmd_vel` silence |
| `map_saver` | `/save_all_maps`: pose-graph + slam_toolbox map + Nav2 map, with retries and verification |
| `localization_monitor` | report-only scan-vs-map agreement score for nav mode |
| `pose_tracker` / `pose_initializer` | persist the last map-frame pose and re-seed `/initialpose` on the next nav start of the same map |
| `uptime_monitor` | boot history, uptime and last-seen-alive for the cockpit |
| `tof_belt_node` | VL53L1X driver over Linux I²C behind a mux; six `sensor_msgs/Range` topics; per-sensor quarantine |
| `tof_down_filter` | turns the floor-looking sensor into obstacle/drop detections on a flat frame |

## Quick start

Everything ROS runs inside the Isaac ROS container; `colcon build` runs there,
`arduino-cli` runs on the host.

### Build

First-time setup (ROS 2 Humble, apt dependencies, `explore_lite`, Python
packages, udev rule, I²C permissions) is in [docs/INSTALL.md](docs/INSTALL.md).
With that done:

```bash
cd /workspaces/isaac_ros-dev
rm -rf build/rover_bringup_extended_slam install/rover_bringup_extended_slam
colcon build --packages-select rover_bringup_extended_slam rover_places rover_web --symlink-install
source install/setup.bash
```

The purge is needed whenever files are deleted or renamed or
`CMakeLists.txt`/`package.xml` change, because colcon only adds and
overwrites. A plain `.cpp` edit needs only the `colcon build` line.
`config/` and `launch/` are symlink-installed, so edits there need only a
restart.

### Map a room

```bash
ros2 launch rover_bringup_extended_slam rover_bringup.launch.py mode:=slam
```

Drive with the cockpit joystick. Close exterior doors, drive slowly, drive
loops so loop closure has constraints, and if the map jumps in RViz stop and
restart because tracking does not recover. Save before stopping; the map only
exists in RAM:

```bash
ros2 service call /save_all_maps std_srvs/srv/Trigger
```

### Navigate on a saved map

```bash
ros2 launch rover_bringup_extended_slam rover_bringup.launch.py \
    mode:=nav posegraph:=/workspaces/isaac_ros-dev/slm_maps/<name>
```

Set the initial pose (the cockpit's SET POSE, RViz, or automatically from the
last saved pose of the same map), wait for the stack, localisation, Nav2 and
map indicators, then send goals. Add `use_exploration:=true` to run frontier
exploration on a growing map.

### The cockpit

```bash
ros2 run rover_web server          # http://<jetson-ip>:8080
```

It owns the launch lifecycle, shows the live map, saved places and goal
progress, and has a 0.5 s joystick deadman. It binds all interfaces with no
authentication; anyone on the LAN can drive the rover.

### Operating rules learned the hard way

- Run sessions under `tmux` on the host; a dropped SSH session can block every
  node on stdout.
- Edit configuration in your working copy and deploy by replacing the whole
  package folder. Jetson-side edits are silently destroyed by the next deploy.
- Before trusting any Nav2 parameter, `ros2 param list /<node>`. The stack is
  Humble; Iron/Jazzy parameter names are accepted by the YAML loader and
  silently ignored, and two multi-month bugs came from exactly that.
- Record bags only after the stack is up (`tools/record_nav.sh all`); stop
  the bag before stopping the stack.
- `/tf` at about 79.6 Hz is the health metric. When it sags, the Jetson is out
  of CPU and everything downstream is a symptom.
- Disable WiFi power save on the Jetson's wireless interface; it does not
  survive reboots or JetPack updates and causes the "alive but unreachable"
  dropouts.

## Safety

Six independent layers stop the rover: the firmware's 500 ms serial watchdog,
the bridge's 0.8 s command timeout with a 10 Hz stop flood, the cockpit's
0.5 s joystick deadman, `wifi_watchdog`, the cockpit's always-enabled STOP
bar, and Nav2's collision checking plus the ToF belt for obstacles below the
LiDAR plane. See
[docs/ARCHITECTURE.md §12](docs/ARCHITECTURE.md#12-safety-layers).

## Calibration summary

| Constant | Value | How it was found |
|---|---|---|
| wheel diameter / track / counts per rev | 0.120 m / 0.37 m / 267 | tape and encoder spec; tape-verified distance within 2 % |
| right wheel trim | 0.969 | straight-line test: IMU yaw drift per metre, forward and back |
| `cmd_w_scale` | 0.85 | drivetrain over-rotates 22 % (gyro vs wire command over 1729 samples) |
| `vyaw_fusion_gate` | 0.08 rad/s | largest odometry step 0.350 → 0.016 m |
| LiDAR x / y / yaw | −0.335 / 0.0127 / −0.0202 | flat-wall PCA fit; x verified to 0.4 mm, yaw provisional |
| footprint and `inflation_radius` | wheels-wide, 0.50 m | rear corners sweep 0.484 m; A/B recoveries 9 → 0 |
| `rotate_to_heading_min_angle` | 0.52 rad | the doorway fix: recoveries 4 → 0, time to goal −20 % |
| ToF caps and down-looker tilt | height/tan 15.5°, 43° | floor test; tilt from the sensor's own clear-floor reading |

Full procedures, maths and open items in [docs/CALIBRATION.md](docs/CALIBRATION.md).

## Status and open items

Working on hardware as of late September 2026: mapping, localisation,
navigation to saved places with zero recoveries on the reference route,
frontier exploration (manually launched), the ToF belt marking both costmaps,
and the phone cockpit.

Open: the right-turn asymmetry test (equal left and right spins through
`calibration/analyze_turn_bag.py --meas`), verification of `lidar_yaw`, a
long-corridor straight-line bias run, the Python-vs-C++ CPU A/B the port was
meant to measure, the LiDAR `Sensitivity` scan mode, two small objects the ToF
belt did not detect, and wiring the cockpit's explore button to the launch
file's `use_exploration` argument.

## Not in this repository

The original Python package this was ported from, the Smac planner
experiment, recorded bags and maps, the session notes, and an experimental
ESP-NOW handheld control panel (a second ESP32 pair that mirrored the phone
cockpit) are kept privately.

## License and attribution

Apache-2.0, see [LICENSE](LICENSE). The same identifier is declared in every
package manifest.

Third-party code and the licences it carries:

- `tof_belt_node.cpp` contains a minimal re-implementation of the VL53L1X
  initialisation sequence (the register configuration table and timing
  budget tables) derived from STMicroelectronics' Ultra Lite Driver, which is
  BSD-3-Clause.
- The recovery behaviour tree is a modified copy of Nav2's
  `navigate_w_replanning_and_recovery.xml` (Apache-2.0).
- `explore_lite` is from `robo-friends/m-explore-ros2` (BSD).
- The firmware uses the Adafruit BNO08x, BusIO and Unified Sensor Arduino
  libraries (BSD / MIT).
