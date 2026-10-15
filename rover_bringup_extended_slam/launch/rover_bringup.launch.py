#!/usr/bin/env python3
import os
import subprocess
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument,
                             IncludeLaunchDescription, TimerAction,
                             SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
THIS_DIR    = os.path.dirname(os.path.abspath(__file__))
CONFIG_DIR  = os.path.normpath(os.path.join(THIS_DIR, "..", "config"))
URDF_FILE   = os.path.join(CONFIG_DIR, "rover.urdf.xacro")
EKF_CONFIG  = os.path.join(CONFIG_DIR, "ekf_config.yaml")
NAV2_PARAMS = os.path.join(CONFIG_DIR, "nav2_params.yaml")
SLAM_PARAMS = os.path.join(CONFIG_DIR, "slam_toolbox_params.yaml")
TOF_PARAMS  = os.path.join(CONFIG_DIR, "tof_belt.yaml")
def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        "mode", default_value="slam",
        description="'slam' for mapping, 'nav' for navigation on existing map"
    )
    map_arg = DeclareLaunchArgument(
        "map", default_value="",
        description="Path to map YAML file (required in nav mode with localization=amcl)"
    )
    localization_arg = DeclareLaunchArgument(
        "localization", default_value="slam_toolbox",
        description=("Localization backend in nav mode: 'slam_toolbox' (continuous "
                     "scan matching, more rigid during rotations, preferred) or "
                     "'amcl' (particle filter, drifts during turns)")
    )
    posegraph_arg = DeclareLaunchArgument(
        "posegraph", default_value="",
        description=("Base filename (no extension) of the serialized SLAM pose-graph, "
                     "e.g. /workspaces/isaac_ros-dev/slm_maps/living_room_3. Required "
                     "when localization=slam_toolbox. Produced by the SLAM run via "
                     "`ros2 service call /slam_toolbox/serialize_map ...`.")
    )
    # STABLE udev SYMLINK, not a ttyACM number (changed 2026-09-26).
    #
    # Every Espressif board enumerates as the same kind of device, and ttyACM
    # numbers are assigned in whatever order the boards happen to come up.
    # With a second ESP32 on the bus the motor/IMU board DID swap numbers one
    # day, and a hard-coded ttyACM0 would have opened the wrong device with
    # nothing in any log naming the cause.
    #
    # tools/99-rover-serial.rules (install it in /etc/udev/rules.d/ on the
    # Jetson) matches the board by its USB serial string — an ESP32-S3 reports
    # its MAC address — and creates ttyMOTOR; the CP2102 LiDAR adapter becomes
    # ttyLIDAR.  Put your own board's serial in that file.  If the stack cannot
    # find a port, check `ls -l /dev/ttyMOTOR` before suspecting the node.
    # lidar_port is left as ttyUSB0 on purpose: there is only ever one CP2102
    # on this rover, so it has no swap hazard to fix.
    lidar_port_arg = DeclareLaunchArgument(
        "lidar_port", default_value="/dev/ttyUSB0",
        description="Serial port for A3M1 LiDAR"
    )
    arduino_port_arg = DeclareLaunchArgument(
        "arduino_port", default_value="/dev/ttyMOTOR",
        description="Serial port for the motor/IMU ESP32 (udev symlink from "
                    "tools/99-rover-serial.rules; was /dev/ttyACM0)"
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="false",
        description="Launch RViz2"
    )
    use_exploration_arg = DeclareLaunchArgument(
        "use_exploration", default_value="false",
        description="Enable autonomous exploration (nav2_explore) in nav mode"
    )
    teleop_arg = DeclareLaunchArgument(
        "teleop", default_value="false",
        description="Teleop session: arms the WiFi-loss stop guard in wifi_watchdog."
    )
    map_basename_arg = DeclareLaunchArgument(
        "map_basename", default_value="live_map",
        description="Basename (no extension) for auto-saved maps in SLAM mode."
    )
    use_pose_autoinit_arg = DeclareLaunchArgument(
        "use_pose_autoinit", default_value="true",
        description="Nav mode: auto-seed /initialpose from last_pose.json when "
                    "the saved map matches. false = manual SET POSE only."
    )
    use_tof_arg = DeclareLaunchArgument(
        "use_tof", default_value="true",
        description="Run the 6x VL53L1X ToF belt (Jetson I2C + TCA9548A). "
                    "Publishing only — whether Nav2 REACTS to it is decided by "
                    "the range layer in nav2_params.yaml, not by this argument. "
                    "false = do not touch the I2C bus at all."
    )
    mode         = LaunchConfiguration("mode")
    map_file     = LaunchConfiguration("map")
    localization = LaunchConfiguration("localization")
    posegraph    = LaunchConfiguration("posegraph")
    lidar_port   = LaunchConfiguration("lidar_port")
    arduino_port = LaunchConfiguration("arduino_port")
    use_rviz     = LaunchConfiguration("use_rviz")
    use_exploration = LaunchConfiguration("use_exploration")
    map_basename   = LaunchConfiguration("map_basename")
    teleop = LaunchConfiguration("teleop")
    use_pose_autoinit = LaunchConfiguration("use_pose_autoinit")
    use_tof = LaunchConfiguration("use_tof")
    # ── 1. Robot state publisher ──────────────────────────────────────────────
    try:
        urdf_content = subprocess.check_output(["xacro", URDF_FILE], text=True)
    except Exception:
        urdf_content = ""
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": urdf_content, "use_sim_time": False}],
        output="screen",
    )
    # ── ZUPT node (Zero Velocity Update) — critical for skid-steer drift correction
    zupt_node = Node(
        package='rover_bringup_extended_slam',
        executable='zupt_node',
        name='zupt_node',
        output='screen',
        parameters=[{
            'vx_threshold':     0.03,
            'vyaw_threshold':   0.05,
            'stationary_delay': 0.2,
            'zupt_covariance':  0.0001,
            'enabled': ParameterValue(PythonExpression(["'", mode, "' == 'slam'"]), value_type=bool),
        }],
    )

    # ── Localization health monitor (nav mode only) ───────────────────────────
    # Scores how well the live scan agrees with the loaded map — the number the
    # operator otherwise has to get by eyeballing RViz.  slam_toolbox in
    # localization mode publishes NO match-confidence value, so a pose that has
    # silently slid is invisible until someone notices the rover is drawn in the
    # wrong place.  That is exactly the 2026-08-21 failure, after the rover
    # manoeuvred around a person standing in the LiDAR plane.
    #
    # REPORT ONLY — publishes numbers, filters nothing, gates nothing.  The
    # point of this first pass is to measure `drop_frac` (rays landing where the
    # map says free space) across normal household traffic, so the eventual
    # dynamic-obstacle filter's safety valve can be set from data instead of
    # guessed.  One person at 1 m is ~8% of a scan; two people plus a moved
    # chair can legitimately reach 25%.
    #
    # nav mode only: in slam mode the map is BUILT from these same scans, so the
    # score is self-referential and always high.
    localization_monitor_node = Node(
        package='rover_bringup_extended_slam',
        executable='localization_monitor',
        name='localization_monitor',
        output='screen',
        parameters=[{
            'map_topic':         '/map',
            'scan_topic':        '/scan_filtered',
            'eval_rate':         2.0,
            'match_tolerance_m': 0.10,
            'max_eval_range':    6.0,
            'occupied_thresh':   65,
            'free_thresh':       25,
            'score_threshold':   0.60,
            'min_valid_rays':    40,
            'stats_period_s':    5.0,
            'transform_timeout': 0.10,
        }],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'nav'"])),
    )

    # ── Pose tracker — last-known pose persisted to the NVMe (nav mode only).
    # `posegraph` identifies the current map; the node strips it to a basename
    # and records it as map_name in last_pose.json.
    pose_tracker_node = Node(
        package='rover_bringup_extended_slam',
        executable='pose_tracker',
        name='pose_tracker',
        output='screen',
        parameters=[{
            'sample_rate_hz': 5.0,
            'save_path':      '/workspaces/isaac_ros-dev/last_pose.json',
            'map_id':         posegraph,
            'log_period_s':   0.0,
        }],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'nav'"])),
    )

    # ── Pose initializer — auto-SET-POSE from last_pose.json (nav mode only).
    # Seeds /initialpose with the saved pose IF its map_name matches the map
    # being loaded now.  Mismatch / missing file -> does nothing; manual SET
    # POSE always overrides.
    pose_initializer_node = Node(
        package='rover_bringup_extended_slam',
        executable='pose_initializer',
        name='pose_initializer',
        output='screen',
        parameters=[{
            'save_path':      '/workspaces/isaac_ros-dev/last_pose.json',
            'map_id':         posegraph,
            'settle_delay_s': 2.0,
            'max_age_s':      0.0,
        }],
        condition=IfCondition(PythonExpression([
            "'", mode, "' == 'nav' and '", use_pose_autoinit, "' == 'true'"
        ])),
    )

    # ── Uptime monitor — Jetson boot history + on-time on /rover/uptime.
    # Runs in every mode.  Appends each new Jetson boot to boot_history.log
    # (extra entries you didn't cause = unwanted POR / reset).
    uptime_monitor_node = Node(
        package='rover_bringup_extended_slam',
        executable='uptime_monitor',
        name='uptime_monitor',
        output='screen',
        parameters=[{
            'history_file':   '/workspaces/isaac_ros-dev/boot_history.log',
            'heartbeat_file': '/workspaces/isaac_ros-dev/rover_uptime_heartbeat',
        }],
    )

    # ── 4b. ToF safety belt ──────────────────────────────────────────────────
    # Runs in every mode.  PUBLISH-ONLY at this stage: six sensor_msgs/Range
    # topics plus /rover/tof_status.  Nothing subscribes to them yet, so this
    # cannot change how the rover drives — the costmap only reacts once a
    # RangeSensorLayer is added to nav2_params.yaml.
    #
    # It is deliberately NOT inside a mode condition: the belt is just as
    # useful while mapping as while navigating, and its data is worth having
    # in bags either way.
    #
    # If the I2C bus misbehaves in the field, `use_tof:=false` keeps the node
    # off the bus entirely.  The node itself never blocks bringup — a sensor
    # that fails to start is quarantined and reported, and the rest keep going.
    tof_belt_node = Node(
        package='rover_bringup_extended_slam',
        executable='tof_belt_node',
        name='tof_belt',
        output='screen',
        parameters=[TOF_PARAMS],
        condition=IfCondition(use_tof),
    )

    # The only consumer of /tof/down.  Publishes /tof/down_obstacle (obstacles
    # projected onto the floor plane) and /rover/tof_drop (a latched Bool when
    # the floor falls away).  Without this node the down-looker publishes into
    # the void — it is deliberately excluded from both costmaps, because raw it
    # would mark a permanent phantom obstacle ~0.39 m ahead.
    tof_down_filter_node = Node(
        package='rover_bringup_extended_slam',
        executable='tof_down_filter',
        name='tof_down_filter',
        output='screen',
        parameters=[TOF_PARAMS],
        condition=IfCondition(use_tof),
    )

    # ── 5. LiDAR ─────────────────────────────────────────────────────────────
    lidar_node = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node",
        parameters=[{
            "serial_port":      lidar_port,
            "serial_baudrate":  256000,
            "frame_id":         "laser_frame",
            "inverted":         False,
            "angle_compensate": True,
            "scan_mode":        "Standard",
        }],
        output="screen",
    )
    # ── 5a'. Scan deskew (motion compensation) ───────────────────────────────
    # Uses IMU angular velocity to back-rotate individual LiDAR rays so the
    # entire scan is referenced to a single timestamp (scan-start).  Removes
    # the rotational smear that corrupts scan matching and costmap stamping
    # during turns.  Inserted BEFORE lidar_scan_filter so downstream
    # sanitisation (median, NaN, boundary clip) operates on clean geometry.
    scan_deskew_node = Node(
        package='rover_bringup_extended_slam',
        executable='scan_deskew',
        name='scan_deskew',
        output='screen',
        parameters=[{
            'in_scan_topic':       '/scan',
            'out_scan_topic':      '/scan_deskewed',
            'imu_topic':           '/imu/filtered',
            'imu_buffer_size':     400,
            'drop_on_imu_gap':     False,   # graceful passthrough on IMU startup
            'min_yaw_deskew_rad':  0.003,   # 0.17° — skip math when stationary
            'warn_throttle_s':     2.0,
        }],
    )

    # ── 5b. LiDAR scan filter ─────────────────────────────────────────────────
    lidar_filter_node = Node(
        package='rover_bringup_extended_slam',
        executable='lidar_scan_filter',
        name='lidar_scan_filter',
        parameters=[{
            'in_topic':           '/scan_deskewed',   # was /scan — now motion-compensated
            'out_topic':          '/scan_filtered',
            # Downsampling halved scan resolution — AMCL scan-matching and the
            # nav2 costmaps work better with the full ~360 beam scan.  Leave
            # stride in place so it can be re-enabled later if CPU becomes tight.
            'enable_downsample':   False,
            'downsample_stride':   2,
            'enable_quality_gate': False,
        }],
    )
    # ── 6. Arduino bridge ─────────────────────────────────────────────────────
    arduino_bridge = Node(
        package="rover_bringup_extended_slam",
        executable="arduino_encoder_twist_bridge",
        name="arduino_encoder_twist_bridge",
        output="screen",
        parameters=[{
            "port":             arduino_port,
            "baud":             115200,
            "counts_per_rev":   267,
            # NEW CHASSIS (2026-05-22):
            #   * 3601 Series Rhino 120 mm wheels  (was 95 mm)
            #   * Drive-axle center-to-center 0.37 m  (was 0.34 m)
            # Both must match WHEEL_DIAMETER_M / WHEEL_BASE_M in the ESP32-S3
            # firmware or odometry will drift systematically.
            "wheel_diameter_m": 0.120,
            "wheel_base_m":     0.37,
            "publish_hz":       50.0,
            "cmd_timeout_s":    0.8,   # raised from 0.5 — prevents spurious motor cuts under GIL load
            "cmd_w_scale":      0.85,
            "yaw_sign":        - 1.0,
            "rev_w_trim":      - 0.015,
            "cmd_vel_topic":    "/cmd_vel",
            # Drive-wheel end = forward (casters TRAIL, not pushed).  Negates
            # linear.x on both the CMD and /wheel/odom_twist so +x drives the
            # drive-wheel end first, matching the URDF (LiDAR at negative x).
            "invert_linear":    True,
            "odom_twist_vx_cov": 0.01,    # (m/s)^2 — encoder velocity noise
            "odom_twist_vyaw_cov": 0.02,  # (rad/s)^2 — nominal yaw rate noise
            # Velocity-gated vyaw covariance.  When |cmd_w| ≥ gate, covariance
            # is inflated to effectively disable wheel-yaw fusion during turns
            # (skid-steer wheels slip → biased yaw rate).  When |cmd_w| < gate,
            # nominal covariance is used (wheels roll cleanly → trustworthy).
            # 0.15 -> 0.08 (2026-08-19).  ekf_config.yaml states the rationale as
            # "straight motion, SLAM arcs <= 0.06 rad/s" — but the gate was set at
            # 0.15, two and a half times that, so wheel yaw was still being fused
            # at nominal covariance right through the band where skid-steer wheels
            # are already slipping.  Mean commanded |angular.z| in
            # nav_20260819_103602 was 0.138 rad/s — i.e. TYPICAL steering sat
            # inside that gap, and the bags show the error it causes
            # (wheel +0.420 vs gyro +2.117 rad/s on the same instant).
            #
            # Wobble mechanism: slipping wheels under-report the turn, the EKF
            # blends that in, the estimated yaw rate comes out low, RPP commands
            # more omega to compensate, the rover actually over-turns, RPP
            # corrects back.  Oscillation.
            #
            # 0.08 keeps a small margin above the documented 0.06 slip threshold
            # so genuine straight-line and SLAM-arc motion still gets the wheel
            # yaw check, while normal nav turns ride on the gyro.
            "vyaw_fusion_gate": 0.08,     # rad/s
            "vyaw_cov_inflated": 100.0,   # (rad/s)^2 — ~5000× nominal
            # ── BNO085 IMU pass-through (2026-05-22) ─────────────────────────
            # The ESP32-S3 streams "IMU seq qw qx qy qz gx gy gz ax ay az"
            # lines on the same serial port at ~100 Hz; the bridge parses and
            # republishes as sensor_msgs/Imu on /imu/data.  Axis remap params
            # default to identity (chip +X = rover +X, labeled side up).
            # NOTE: do NOT pass axis_*_src ("x"/"y"/"z") through this launch
            # params dict.  ros2 launch serializes them to a temp YAML file
            # unquoted, and the rcl YAML parser interprets bare "y" as the
            # boolean True (YAML 1.1 treats y/n/yes/no/on/off as bools).  That
            # made declare_parameter("axis_y_src", <string>) raise
            # InvalidParameterTypeException and killed the bridge on startup.
            # The identity src values live as the node's own string defaults;
            # only the (integer-safe) sign params are overridden here.  If a
            # non-identity mounting ever needs a src remap, change the node's
            # declared default rather than passing the letter through launch.
            "publish_imu":      True,
            "imu_topic":        "/imu/data",
            "imu_frame":        "imu_link",
            "axis_x_sgn": 1,
            "axis_y_sgn": 1,
            "axis_z_sgn": 1,
            "imu_orient_cov":   0.01,
            "imu_gyro_cov":     0.0003,
            "imu_accel_cov":    0.04,
        }],
    )
    # ── 7. EKF ────────────────────────────────────────────────────────────────
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_node",
        output="screen",
        parameters=[EKF_CONFIG],
        remappings=[("odometry/filtered", "/odometry/filtered")],
    )
    # ── 8a. SLAM Toolbox ──────────────────────────────────────────────────────
    # Launch directly (not via online_async_launch.py) so that our params file
    # is the only one passed and is not overridden by slam_toolbox's temp defaults.
    # async_slam_toolbox_node: scan ingestion runs on its own thread, decoupled
    # from the Ceres solver.  A slow optimization cycle no longer blocks the
    # next scan from being matched, removing the sync-callback queue buildup
    # that manifested as "scan drift / scheduling lag" in RViz.  Same params
    # as sync variant — switching back is a one-line revert.
    slam_toolbox_launch = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[SLAM_PARAMS, {
            "use_sim_time": False,
            "minimum_laser_range": 0.2,
            "correlation_search_space_dimension": 0.5,
        }],
        # ALSO runs in nav mode when exploring (2026-08-19).  Exploration needs
        # a map that GROWS — frontiers are the boundary between known free space
        # and unknown.  The localization node below loads a finished pose-graph
        # and never extends it, so explore_lite correctly reported
        # "No frontiers found, stopping." every single time it was tried.
        # Nav2 only runs in nav mode, and exploration needs Nav2 to execute its
        # goals, so "nav mode + mapping SLAM" is the combination it requires —
        # and it was previously impossible to launch.
        condition=IfCondition(PythonExpression([
            "'", mode, "' == 'slam' or ('",
            mode, "' == 'nav' and '", use_exploration, "' == 'true')"
        ]))
    )
    # ── 8b. AMCL ─────────────────────────────────────────────────────────────
    # Particle-filter localization.  Included for compatibility but has known
    # drift problems during in-place rotations on this rover.  Use
    # localization:=slam_toolbox (default) for continuous scan matching instead.
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("nav2_bringup"), "/launch/localization_launch.py"
        ]),
        launch_arguments={
            "use_sim_time":    "false",
            "map":             map_file,
            "params_file":     NAV2_PARAMS,
            "autostart":       "true",
            "use_composition": "False",
            "use_respawn":     "False",
        }.items(),
        condition=IfCondition(PythonExpression([
            "'", mode, "' == 'nav' and '", localization, "' == 'amcl'"
        ]))
    )

    # ── 8c. slam_toolbox localization mode ──────────────────────────────────
    # Uses continuous scan matching against the saved pose-graph instead of
    # AMCL's particle filter.  Much more rigid during rotations — the scan
    # stays anchored to the map walls because every scan runs ICP-like
    # registration rather than averaging a probabilistic particle cloud.
    #
    # Requires the `.data` + `.posegraph` pair produced during the SLAM run by:
    #   ros2 service call /slam_toolbox/serialize_map \
    #       slam_toolbox/srv/SerializePoseGraph \
    #       "{filename: '/workspaces/isaac_ros-dev/slm_maps/YOUR_MAP'}"
    #
    # slam_toolbox publishes BOTH /map (occupancy grid) and the map→odom TF,
    # so no separate map_server is needed.  Nav2 costmaps subscribe to /map
    # transparently.
    slam_toolbox_localization_launch = Node(
        package="slam_toolbox",
        executable="localization_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[SLAM_PARAMS, {
            "use_sim_time":      False,
            "mode":              "localization",
            "map_file_name":     posegraph,
            "map_start_at_dock": False,
            # OVERRIDE the scan topic from the shared SLAM params.  slam_turn_gate
            # is SLAM-only — in nav mode /scan_slam has no publisher.  Point
            # slam_toolbox localization at /scan_filtered (already deskewed by
            # scan_deskew + sanitised by lidar_scan_filter).  Motion blur is
            # handled per-ray by the deskew node, so scan matching gets clean
            # geometry WITHOUT the 1.5s TF freeze that slam_turn_gate caused.
            "scan_topic":                       "/scan_filtered",
            # Looser scan-match thresholds for re-localization.  The saved
            # pose-graph was built during mapping with stricter matching; at
            # re-localization the rover's current pose may differ slightly
            # from any graph node, so tight thresholds cause silent failure.
            "link_match_minimum_response_fine": 0.45,
            "loop_match_minimum_response_fine": 0.40,
            "minimum_laser_range":              0.2,
            "correlation_search_space_dimension": 0.5,
            "correlation_search_space_resolution": 0.01,
            "loop_search_space_dimension":      8.0,
            "transform_publish_period":         0.02,
        }],
        # Stand down when exploring — the mapping node above takes over instead.
        # Both are named "slam_toolbox", so running both would be a name clash.
        condition=IfCondition(PythonExpression([
            "'", mode, "' == 'nav' and '", localization, "' == 'slam_toolbox'",
            " and '", use_exploration, "' != 'true'"
        ]))
    )
    # NAV2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("nav2_bringup"), "/launch/navigation_launch.py"
        ]),
        launch_arguments={
            "use_sim_time":    "false",
            "params_file":     NAV2_PARAMS,
            "autostart":       "true",
            "use_composition": "False",
            "use_respawn":     "False",
        }.items(),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'nav'"]))
    )

    # ── RViz ─────────────────────────────────────────────────────────────────
    # Config lives on the bind-mounted workspace so it survives container restarts.
    rviz_config = '/workspaces/isaac_ros-dev/default.rviz'
    
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    # ── WiFi + deadman safety watchdog (always on) ───────────────────────────
    wifi_watchdog_node = Node(
        package="rover_bringup_extended_slam",
        executable="wifi_watchdog",
        name="wifi_watchdog",
        output="screen",
        parameters=[{
            "check_hz":          20.0,    # WiFi poll rate
            "stop_flood_hz":     100.0,   # Twist(0,0) rate when unsafe
            # Deadman on /cmd_vel silence is for teleop, NOT nav2.  Nav2
            # pauses cmd_vel for seconds during planning/recovery behaviors;
            # a 0.5s deadman would flood zero-Twists right as nav2 is trying
            # to resume motion, producing jittery collision loops.  Keep the
            # WiFi check (catches disconnect), disable cmd_vel heartbeat.
            "deadman_secs":      5.0,    # still active, but only for long outages
            "wifi_iface":        "",     # auto-detect; or force e.g. "wlan0"
            "enable_wifi_check": ParameterValue(teleop, value_type=bool),
            "enable_deadman":    False,
        }],
    )
    # ── Map saver — provides the /save_all_maps service (saving is manual) ───
    map_saver_node = Node(
        package='rover_bringup_extended_slam',
        executable='map_saver',
        name='map_saver',
        output='screen',
        # ALSO in nav mode when exploring (2026-08-19).  Was 'slam' only, on the
        # reasonable assumption that only slam mode builds maps — but nav mode +
        # use_exploration now runs async_slam_toolbox_node and builds a map too.
        # Without this, /save_all_maps simply does not exist and the service call
        # hangs on "waiting for service to become available..." with a fully
        # explored map sitting in RAM and no way to write it out.
        # Found the hard way mid-run on 2026-08-19.
        condition=IfCondition(PythonExpression([
            "'", mode, "' == 'slam' or ('",
            mode, "' == 'nav' and '", use_exploration, "' == 'true')"
        ])),
        parameters=[{
            'map_dir':           '/workspaces/isaac_ros-dev/slm_maps',
            'map_basename':      map_basename,
            'max_retries':       3,
            'verify_timeout_s':  4.0,
            'retry_backoff_s':   0.5,
        }],
    )
    # ── 10. Autonomous exploration (nav2_explore, conditional) ─────────────────
    # Autonomously explores unmapped frontiers using frontier detection
    nav2_explore_node = Node(
        condition=IfCondition(PythonExpression(["'", mode, "' == 'nav' and '", use_exploration, "' == 'true'"])),
        package="explore_lite",
        executable="explore",
        name="explore",
        output="screen",
        parameters=[NAV2_PARAMS],
    )

    # Arduino starts first, everything else delayed 5s.  The IMU arrives via
    # the ESP32-S3 + BNO085 over the arduino bridge serial link.
    delayed_stack = TimerAction(
        period=5.0,
        actions=[
            robot_state_publisher,
            zupt_node,
            localization_monitor_node,
            pose_tracker_node,
            pose_initializer_node,
            uptime_monitor_node,
            tof_belt_node,
            tof_down_filter_node,
            lidar_node,
            scan_deskew_node,
            lidar_filter_node,
            wifi_watchdog_node,
            map_saver_node,
            ekf_node,
            slam_toolbox_launch,
            amcl_launch,
            slam_toolbox_localization_launch,
            nav2_launch,
            rviz_node,
        ]
    )

    # ── Exploration starts LAST, and late ─────────────────────────────────────
    # explore_lite runs one frontier search almost immediately, and if it finds
    # nothing it prints "No frontiers found, stopping." and shuts down FOR GOOD
    # — there is no retry.  Sharing the 5 s delay with the rest of the stack
    # meant it searched while Nav2 was still activating and slam_toolbox had not
    # published a map yet, so it saw an empty costmap, found no frontiers and
    # quit.  That is exactly what the 2026-08-19 13:13 run did: searched at
    # t+5.4 s, "Could not find nearby clear cell to start search", stopped.
    #
    # 25 s gives Nav2 time to reach active, slam_toolbox time to publish a first
    # map (map_update_interval is 2.0 s), and the costmaps time to populate.
    delayed_explore = TimerAction(
        period=25.0,
        actions=[nav2_explore_node],
    )
    return LaunchDescription([
        # Force OpenCV CPU-only for ALL nodes in this launch (SLAM Toolbox in
        # particular).  On Jetson, JetPack's OpenCV is built with CUDA so any
        # cv::Mat op silently dispatches to GPU without this.
        SetEnvironmentVariable('OPENCV_OPENCL_RUNTIME', 'disabled'),
        mode_arg,
        map_arg,
        localization_arg,
        posegraph_arg,
        lidar_port_arg,
        arduino_port_arg,
        use_rviz_arg,
        use_exploration_arg,
        map_basename_arg,
        use_pose_autoinit_arg,
        use_tof_arg,
        teleop_arg,
        arduino_bridge,
        delayed_stack,
        delayed_explore,
    ])
