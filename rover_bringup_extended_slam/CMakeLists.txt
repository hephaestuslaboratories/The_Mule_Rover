cmake_minimum_required(VERSION 3.8)
project(rover_bringup_extended_slam)

# ─────────────────────────────────────────────────────────────────────────────
# C++ port of the rover bringup nodes (was ament_python).
#
# CONTRACT — do not change without checking rover_web:
#   * package name          rover_bringup_extended_slam
#   * executable names      identical to the old console_scripts entry points
#                           (rover_web/launch_manager.py pkill -f's them)
#   * node names, topics, parameter names/defaults  identical
#   * launch file name and arguments                identical
# ─────────────────────────────────────────────────────────────────────────────

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Optimise by default — this port exists to save CPU on the Jetson.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(slam_toolbox REQUIRED)

# ── Nodes ────────────────────────────────────────────────────────────────────
add_executable(zupt_node                   src/zupt_node.cpp)
add_executable(lidar_scan_filter           src/lidar_scan_filter.cpp)
add_executable(scan_deskew                 src/scan_deskew_node.cpp)
add_executable(wifi_watchdog               src/wifi_watchdog.cpp)
add_executable(localization_monitor        src/localization_monitor.cpp)
add_executable(arduino_encoder_twist_bridge src/arduino_encoder_twist_bridge.cpp)
add_executable(pose_tracker               src/pose_tracker.cpp)
add_executable(pose_initializer           src/pose_initializer.cpp)
add_executable(uptime_monitor             src/uptime_monitor.cpp)
add_executable(tof_belt_node              src/tof_belt_node.cpp)
add_executable(tof_down_filter            src/tof_down_filter.cpp)

ament_target_dependencies(zupt_node        rclcpp sensor_msgs nav_msgs)
ament_target_dependencies(lidar_scan_filter rclcpp sensor_msgs)
ament_target_dependencies(scan_deskew      rclcpp sensor_msgs)
ament_target_dependencies(wifi_watchdog    rclcpp geometry_msgs)
ament_target_dependencies(localization_monitor
  rclcpp sensor_msgs nav_msgs std_msgs tf2 tf2_ros)
ament_target_dependencies(arduino_encoder_twist_bridge
  rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs)
ament_target_dependencies(pose_tracker     rclcpp geometry_msgs tf2 tf2_ros)
ament_target_dependencies(pose_initializer rclcpp geometry_msgs std_msgs)
ament_target_dependencies(uptime_monitor   rclcpp std_msgs)
# Linux I2C only — no external sensor library.  The VL53L1X init sequence is
# ported from ST's ULD inside the node; cross-check against tools/tof_probe.py
# --live (Adafruit driver) if readings ever look suspect.
ament_target_dependencies(tof_belt_node    rclcpp sensor_msgs std_msgs)
ament_target_dependencies(tof_down_filter  rclcpp sensor_msgs std_msgs)

# TODO(port): turn_recorder and random_patrol are not ported yet.  Both are
# opt-in and OFF by default (record_turns:=false, use_patrol:=false) and
# rover_web never passes either argument, so the default SLAM/nav stack is
# complete without them.  Re-add here when src/turn_recorder.cpp and
# src/random_patrol.cpp land.

# The bridge runs its serial reader on its own thread.
find_package(Threads REQUIRED)
target_link_libraries(arduino_encoder_twist_bridge Threads::Threads)

# ── Nodes with heavier dependencies ──────────────────────────────────────────
add_executable(map_saver src/map_saver.cpp)
ament_target_dependencies(map_saver rclcpp std_msgs std_srvs slam_toolbox)
target_link_libraries(map_saver Threads::Threads)

install(TARGETS
  zupt_node
  lidar_scan_filter
  scan_deskew
  wifi_watchdog
  localization_monitor
  arduino_encoder_twist_bridge
  pose_tracker
  pose_initializer
  uptime_monitor
  tof_belt_node
  tof_down_filter
  map_saver
  DESTINATION lib/${PROJECT_NAME}
)

# The old PyQt5 NATIVE DESKTOP app (rover_gui / rover_launcher_gui) was REMOVED
# 2026-08-16 — superseded by the rover_web browser cockpit, which is a separate
# package and is not affected by this.  With the Qt app went the last Python in
# this package, so ament_cmake_python is no longer needed either.

# ── Shared resources (paths unchanged from the Python package) ───────────────
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Copyright/licence linters are noise for this private package.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
