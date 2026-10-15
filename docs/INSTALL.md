# Installation and environment

What has to exist on the Jetson before `colcon build` works, and the answer
to "do I need the container?".

Contents

1. [Do you need the Isaac ROS container?](#1-do-you-need-the-isaac-ros-container)
2. [Workspace layout the code expects](#2-workspace-layout-the-code-expects)
3. [ROS 2 dependencies](#3-ros-2-dependencies)
4. [Source dependencies](#4-source-dependencies)
5. [Python dependencies](#5-python-dependencies)
6. [Device access](#6-device-access)
7. [Build](#7-build)
8. [Firmware](#8-firmware)
9. [Host-side services and settings](#9-host-side-services-and-settings)
10. [First run checklist](#10-first-run-checklist)

---

## 1. Do you need the Isaac ROS container?

Short answer: **you need ROS 2 Humble on an Ubuntu 22.04 userland. The
container is one way to get that, not a requirement of this code.**

Nothing in this repository uses an Isaac ROS package or the GPU. The launch
file even sets `OPENCV_OPENCL_RUNTIME=disabled` so JetPack's CUDA OpenCV stays
out of the way. The container was used because the rover's Jetson runs a
JetPack whose host OS cannot install Humble natively, and NVIDIA's
`isaac_ros_common` dev container is the lowest-effort way to get a working
Humble aarch64 environment with the workspace bind-mounted at
`/workspaces/isaac_ros-dev`. Every absolute path in this repository follows
from that one choice (§2).

You have two options.

**Option A: the Isaac ROS dev container (what this rover runs).**

1. Install Docker and the NVIDIA container runtime per NVIDIA's JetPack
   instructions.
2. Clone `NVIDIA-ISAAC-ROS/isaac_ros_common` and use its
   `scripts/run_dev.sh` with the workspace directory set to a folder on the
   NVMe (this rover uses `/mnt/nova_ssd/workspaces/isaac_ros-dev`, mounted
   inside as `/workspaces/isaac_ros-dev`). `run_dev.sh` uses host networking
   by default, which the phone cockpit relies on.
3. Add device pass-throughs to the container's run arguments. The container
   has no udev, so names are resolved at start:

   ```
   --device /dev/ttyMOTOR:/dev/ttyMOTOR      # motor/IMU ESP32 (udev symlink, §6)
   --device /dev/ttyUSB0                     # RPLidar CP2102
   --device /dev/i2c-7                       # ToF belt (Orin Nano header pins 3/5)
   ```

   If you cannot change the run arguments, `tools/link_serial_devices.sh`
   recreates the `ttyMOTOR`/`ttyLIDAR` links inside the container from sysfs,
   and `sudo chmod 666 /dev/i2c-7` inside the container unblocks the ToF bus
   until the next restart.
4. Only `/workspaces/isaac_ros-dev` survives a container restart. Home,
   `/tmp` and anything installed with `apt` or `pip` inside the container are
   wiped. That is why Python packages go into `pydeps/` on the workspace (§5)
   and why the Isaac ROS image's own entry-point scripts are the place to add
   `apt` installs you want to persist.

**Option B: native ROS 2 Humble.**

Any Ubuntu 22.04 machine, including a Jetson on a JetPack that ships 22.04,
can install Humble from the standard apt repositories
(`ros-humble-desktop`). Then either create `/workspaces/isaac_ros-dev` and use
it as the colcon workspace, or change the paths in §2. Nothing else differs:
the same apt packages, the same source dependencies, the same build.

Native is simpler and faster to iterate on. The container is the right choice
only when the host OS forces it.

## 2. Workspace layout the code expects

```
/workspaces/isaac_ros-dev/
├── src/
│   ├── rover_bringup_extended_slam/     this repo
│   ├── rover_web/                       this repo
│   ├── rover_places/                    this repo
│   └── m-explore-ros2/                  cloned, §4
├── install/  build/  log/               colcon output
├── pydeps/                              pip --target for the cockpit, §5
├── slm_maps/                            saved maps (posegraph/.data + yaml/pgm)
├── slam_bags/                           rosbag2 recordings
├── last_pose.json                       written by pose_tracker
├── rover_places.json                    the named-place store
├── boot_history.log, rover_uptime_heartbeat
└── default.rviz                         RViz layout, optional
```

If you put the workspace somewhere else, change these:

| Where | What | How |
|---|---|---|
| `rover_bringup_extended_slam/config/nav2_params.yaml` | `default_nav_to_pose_bt_xml` (and the Iron-spelled twin below it) | absolute path of the installed `navigate_w_smoothing_and_recovery.xml` under your `install/` |
| `rover_bringup_extended_slam/launch/rover_bringup.launch.py` | `save_path`, `history_file`, `heartbeat_file`, `map_dir`, `rviz_config` | parameters passed to pose_tracker, pose_initializer, uptime_monitor, map_saver, rviz |
| `rover_bringup_extended_slam/src/{map_saver,pose_tracker,pose_initializer,uptime_monitor}.cpp` | the same values as node defaults | optional; the launch values win |
| `rover_web/rover_web/launch_manager.py` | `WORKSPACE` | everything else in that file derives from it |
| `rover_web/rover_web/server.py` | `PYDEPS` | or install the web packages normally and ignore it |
| `rover_places/rover_places/place_store.py` | `DEFAULT_STORE_PATH` | location of `rover_places.json`; or set the `ROVER_PLACES_STORE` environment variable instead of editing |

## 3. ROS 2 dependencies

Declared in the three `package.xml` files, so after sourcing Humble and
initialising rosdep once (`sudo rosdep init && rosdep update`) the standard
command installs everything apt can provide:

```bash
cd /workspaces/isaac_ros-dev
rosdep install --from-paths src --ignore-src -r -y
```

For reference, the apt packages that resolves to on Humble:

```
ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-msgs
ros-humble-slam-toolbox ros-humble-robot-localization
ros-humble-rplidar-ros ros-humble-robot-state-publisher ros-humble-xacro
ros-humble-tf2-ros ros-humble-tf2-geometry-msgs
ros-humble-rosbag2 ros-humble-rviz2          # bags and visualisation
python3-colcon-common-extensions python3-rosdep
```

`rosdep` will report `explore_lite` as unresolvable; that is expected (§4).

Versions matter in one place: this stack is tuned against **Humble**'s Nav2.
`nav2_params.yaml` carries a handful of Iron/Jazzy parameter names, grouped
and labelled, that Humble ignores. On a newer distro they would become live
and a few Humble names would stop working. Audit with
`ros2 param list /<node>` before trusting anything if you move distro.

## 4. Source dependencies

- **`explore_lite`** (frontier exploration, only used with
  `use_exploration:=true` or the cockpit's START EXPLORE):

  ```bash
  cd /workspaces/isaac_ros-dev/src
  git clone https://github.com/robo-friends/m-explore-ros2.git
  ```

  It builds with the rest of the workspace. The launch file and the cockpit
  both pass `-r __node:=explore` so the `explore:` block in `nav2_params.yaml`
  applies.

- **`rover_places`** is included in this repository; `rover_web` imports it.

## 5. Python dependencies

The cockpit needs three packages that are not in the ROS image. Inside the
container, install them onto the persistent workspace; `server.py` adds that
directory to `sys.path` at start:

```bash
pip3 install --target=/workspaces/isaac_ros-dev/pydeps \
    "fastapi==0.115.12" "uvicorn[standard]==0.34.0" "websockets==12.0"
```

Those pins are known to work together (starlette 0.46.x). Current PyPI
releases have broken the WebSocket route before. On a native install a normal
`pip3 install --user` of the same pins is fine.

Optional, for the tools:

| Tool | Needs |
|---|---|
| `calibration/*.py` | `rosbag2_py`, `rclpy` (part of the ROS install); run inside the ROS environment |
| `tools/lidar_cal.py`, `tools/costmap_peek.py` | `rclpy`, `tf2_ros`; `lidar_cal.py` also `numpy` |
| `tools/map_survey.py` | `numpy` only, no ROS |
| `tools/tof_probe.py` | `smbus2`; `--live` additionally needs `adafruit-circuitpython-vl53l1x`, `adafruit-circuitpython-tca9548a` and `adafruit-extended-bus` |

## 6. Device access

**Serial ports.** Install the udev rule on the host after putting your motor
board's serial number into it:

```bash
udevadm info -q property -n /dev/ttyACM0 | grep ID_SERIAL_SHORT   # find it
# edit tools/99-rover-serial.rules, replace PUT-YOUR-MOTOR-ESP32-SERIAL-HERE
sudo cp rover_bringup_extended_slam/tools/99-rover-serial.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/ttyMOTOR /dev/ttyLIDAR
```

The rule also tells ModemManager to ignore both USB vendors; without that it
probes the ports with AT commands on hotplug. Add your user to `dialout` (or
rely on the container running as root).

**I²C for the ToF belt.** `/dev/i2c-7` is owned by root and the `i2c` group.
Either add `--group-add <gid of i2c>` to the container, or install a udev rule
that sorts after NVIDIA's (for example `99-zz-i2c-rover.rules` containing
`KERNEL=="i2c-7", MODE:="0666"`), or `chmod 666` it inside the container
after each start. `tof_belt_node` prints which of the two failure modes it hit
(`ENOENT` = not passed in, `EACCES` = permissions). `use_tof:=false` keeps
the node off the bus entirely.

**Wiring the mux.** TCA9548A at 0x70 on Orin Nano header pins 3 (SDA) and 5
(SCL), 3.3 V, 100 kHz, one VL53L1X per channel at the factory address 0x29.
Channel-to-sensor mapping lives in `config/tof_belt.yaml`; verify yours with
`tools/tof_probe.py --watch` before trusting it.

## 7. Build

```bash
cd /workspaces/isaac_ros-dev
source /opt/ros/humble/setup.bash
colcon build --packages-select rover_bringup_extended_slam rover_places rover_web --symlink-install
source install/setup.bash
```

Rebuild rules that have cost time before:

- A `.cpp` edit needs only `colcon build --packages-select rover_bringup_extended_slam`.
- `config/` and `launch/` are symlink-installed: copy and restart, no build.
- Deleting or renaming a file, or touching `CMakeLists.txt`/`package.xml`,
  needs `rm -rf build/<pkg> install/<pkg>` first; colcon never removes.
- `rover_web` runs from `build/`, not `src/`, so editing its Python or its
  `static/` files needs a rebuild **and** a server restart.

## 8. Firmware

The ESP32-S3 sketch in `firmware/esp32s3_motor_imu/` is flashed from the
**host** with `arduino-cli` (not from inside the container, which does not
see the port the same way).

```bash
arduino-cli core update-index --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core install esp32:esp32                      # 3.0 or newer (LEDC v3 API)
arduino-cli lib install "Adafruit BNO08x" "Adafruit BusIO" "Adafruit Unified Sensor"

FQBN=esp32:esp32:esp32s3:CDCOnBoot=cdc,FlashSize=16M,PSRAM=opi,USBMode=hwcdc,PartitionScheme=default_8MB
arduino-cli compile --fqbn $FQBN firmware/esp32s3_motor_imu
arduino-cli upload  --fqbn $FQBN -p /dev/ttyMOTOR firmware/esp32s3_motor_imu
```

Stop the ROS stack first; the bridge holds the port open. After a reflash the
container may see a stale device node; if the bridge cannot open the port,
restart the container. The boot banner printed by the firmware (forwarded to
`/rover/link_status`) states `WHEEL_DIAMETER_M`, `WHEEL_BASE_M` and
`WATCHDOG_MS`; check them against the launch parameters.

Before flashing a different chassis, read
[CALIBRATION.md](CALIBRATION.md) §1: the pin map encodes physically crossed
direction wires, and the wheel constants must match the bridge exactly.

## 9. Host-side services and settings

- **WiFi power save** on the Jetson's wireless interface causes the "alive
  but unreachable" dropouts and is re-enabled by NetworkManager by default:

  ```bash
  sudo iw dev <iface> set power_save off
  sudo nmcli connection modify <profile> wifi.powersave 2
  ```

- **`tools/wifi_link_logger.sh`** and its `.service` record link state to
  `/var/log/wifi_link.log` every 2 s if you want evidence after a dropout.
- **`tools/rover_health.sh`** runs on the host as root during a session and
  captures boot id, `tegrastats` and filtered `dmesg`.
- Run every session under `tmux` on the host; a dropped SSH session can block
  the whole stack on stdout.
- If you want the cockpit to start at boot, write a systemd unit that waits
  for Docker and the container and then `docker exec`s
  `ros2 run rover_web server` with ROS sourced. (The rover has one; it is
  host-specific and not in this repository.)

## 10. First run checklist

1. `ls -l /dev/ttyMOTOR /dev/ttyUSB0 /dev/i2c-7` inside the environment.
2. `ros2 run rover_web server` and open `http://<jetson-ip>:8080`.
3. START SLAM, drive a small loop with the joystick, watch that the map
   grows and `/tf` sits near 80 Hz (`ros2 topic hz /tf --window 100`).
4. SAVE MAP, STOP, START NAV on that map, SET POSE, send a goal to a saved
   place.
5. Record a bag with `tools/record_nav.sh all`. Before tuning anything,
   check three things in it: `/tf` rate near 80 Hz, no odom→base_link step
   over 0.07 m, and map→odom corrections that stay small.

If the rover drives but turns the wrong way, or spins far faster than
commanded, stop and read [CALIBRATION.md](CALIBRATION.md) §3 before touching
any Nav2 parameter.
