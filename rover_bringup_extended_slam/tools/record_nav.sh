#!/usr/bin/env bash
# record_nav.sh — start a nav-mode flight recording.
#
#   bash record_nav.sh            # curated topic list (smaller)
#   bash record_nav.sh all        # every topic (-a), safest, ~30% bigger
#
# Exists because the curated topic list is far too long to paste into an SSH
# session without characters being dropped.

set -u
WS=/workspaces/isaac_ros-dev
OUT="$WS/slam_bags/nav_$(date +%Y%m%d_%H%M%S)"

if [ "${1:-}" = "all" ]; then
  echo "recording ALL topics -> $OUT"
  exec ros2 bag record -a -o "$OUT"
fi

TOPICS=(
  # intent
  /cmd_vel /rover/cmd_sent
  /plan /received_global_plan /lookahead_point /lookahead_collision_arc
  /behavior_tree_log
  # reality
  /odometry/filtered /wheel/odom_twist
  /wheel/left_velocity /wheel/right_velocity
  /imu/data /imu/filtered
  /tf /tf_static
  # perception
  /scan /scan_filtered
  # what the planner BELIEVED was there.  bag_analyze.py needs
  # /local_costmap/costmap to measure applied inflation and path centring;
  # without it the wall-hugging half of the analysis cannot run.
  /local_costmap/costmap /global_costmap/costmap /plan_smoothed
  # health
  /rover/link_status /rosout
)

echo "recording ${#TOPICS[@]} topics -> $OUT"
exec ros2 bag record -o "$OUT" "${TOPICS[@]}"
