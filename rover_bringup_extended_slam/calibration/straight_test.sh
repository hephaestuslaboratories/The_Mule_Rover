#!/usr/bin/env bash
# straight_test.sh — straight-line calibration data acquisition
#
# Runs N forward + N backward bursts at a fixed speed while recording a rosbag
# with everything needed to derive RIGHT_WHEEL_TRIM (and check distance scale).
# Run INSIDE the Isaac ROS container, with rover_bringup already running
# (mode:=slam is fine; make sure nothing autonomous is driving: default launch
# has use_init_movement:=false, so plain bringup is safe).
#
# Usage:  bash straight_test.sh [SPEED] [DURATION_S] [RUNS_PER_DIRECTION]
# Defaults: 0.15 m/s, 5 s, 3 runs each way.
#
# NOTE: 0.10 m/s is below the comfortable stiction band (feedforward ~20 PWM vs
# ~47 PWM breakaway) — use >= 0.15 m/s for clean, repeatable runs.

set -u
SPEED="${1:-0.15}"
DUR="${2:-5}"
RUNS="${3:-3}"
RATE=10                                   # cmd publish rate (Hz)
TIMES=$(( DUR * RATE ))                   # number of messages = exact run length
BAGDIR="/workspaces/isaac_ros-dev/slam_bags"
STAMP=$(date +%Y%m%d_%H%M%S)
BAG="${BAGDIR}/straight_cal_${STAMP}"

echo "=== Straight-line calibration ==="
echo "Speed: ${SPEED} m/s | Duration: ${DUR} s | Runs: ${RUNS} fwd + ${RUNS} back"
echo "Expected travel per run: $(python3 -c "print(f'{${SPEED}*${DUR}:.2f}')") m"
echo "Bag: ${BAG}"
echo ""
echo "PRE-FLIGHT (see STRAIGHT_LINE_TEST.md):"
echo "  * >= $(python3 -c "print(f'{${SPEED}*${DUR}+0.5:.1f}')") m of clear, flat runway"
echo "  * Mark the start position with tape (measure ground-truth distance after each run)"
echo "  * IMU yaw-sign check done once (rotate CCW by hand -> yaw must INCREASE)"
read -r -p "Press ENTER to start recording..."

ros2 bag record -o "${BAG}" \
  /cmd_vel /imu/data /wheel/odom_twist \
  /wheel/left_velocity /wheel/right_velocity \
  /odometry/filtered &
RECPID=$!
trap 'kill ${RECPID} 2>/dev/null' EXIT
sleep 3   # let the recorder attach to all topics

pub_burst () {  # $1 = signed speed
  ros2 topic pub -r ${RATE} --times ${TIMES} /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: $1}, angular: {z: 0.0}}" > /dev/null
  # bridge idle stop-flood takes over as soon as pub exits; give it a beat
  sleep 3
}

for i in $(seq 1 "${RUNS}"); do
  read -r -p "FORWARD run ${i}/${RUNS} — clear runway? ENTER to go..."
  pub_burst "${SPEED}"
  echo "  -> measure & note ground-truth distance + drift direction, then reposition if needed"
done

for i in $(seq 1 "${RUNS}"); do
  read -r -p "BACKWARD run ${i}/${RUNS} — clear runway? ENTER to go..."
  pub_burst "-${SPEED}"
  echo "  -> measure & note ground-truth distance + drift direction, then reposition if needed"
done

sleep 2
kill ${RECPID} 2>/dev/null
trap - EXIT
sleep 2
echo ""
echo "=== Done. Bag saved: ${BAG} ==="
echo "Analyze with:"
echo "  python3 analyze_straight_bag.py ${BAG} --current-right-trim <value in firmware>"
