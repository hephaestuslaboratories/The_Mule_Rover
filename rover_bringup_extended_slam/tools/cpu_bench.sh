#!/usr/bin/env bash
# cpu_bench.sh — A/B CPU + health capture for one rover run.
#
#   bash cpu_bench.sh <label> [seconds]
#     e.g.  bash cpu_bench.sh cpp 90
#           bash cpu_bench.sh py  90
#
# Run it in its own terminal WHILE the rover is driving a goal.  Start it just
# before you send the goal so the window covers real motion, not idling.
#
# Writes to /workspaces/isaac_ros-dev/bench/<label>_<timestamp>/ :
#   top_raw.txt      full per-process samples (top -c, so python3 nodes are
#                    distinguishable by their script path)
#   summary.txt      averaged %CPU per node + system totals  <-- send me this
#   hz_*.txt         topic rates over the same window
#   rosout_ekf.txt   count of "Failed to meet update rate" from the EKF
#   env.txt          power mode, clocks, core count, package build type
#
# Everything here is read-only.  It changes nothing on the rover.

set -u

LABEL="${1:-run}"
DUR="${2:-90}"
WS=/workspaces/isaac_ros-dev
OUT="$WS/bench/${LABEL}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"

echo "=== cpu_bench: label=$LABEL  duration=${DUR}s ==="
echo "    output -> $OUT"
echo "    START THE GOAL NOW."
echo

# ── environment / confound record ────────────────────────────────────────────
{
  echo "label:      $LABEL"
  echo "date:       $(date -Is)"
  echo "duration_s: $DUR"
  echo "cores:      $(nproc)"
  echo "uptime:     $(uptime)"
  echo "--- nvpmodel (power mode MUST match between runs) ---"
  (nvpmodel -q 2>/dev/null || echo "nvpmodel not available inside container — check on the HOST")
  echo "--- package build type ---"
  (grep -m1 build_type "$WS/src/rover_bringup_extended_slam/package.xml" 2>/dev/null \
    || echo "package.xml not found")
  echo "--- nodes running ---"
  ros2 node list 2>/dev/null
} > "$OUT/env.txt" 2>&1

# ── topic rates (background, same window) ────────────────────────────────────
for t in /imu/data /imu/filtered /wheel/odom_twist /scan /scan_filtered /odometry/filtered /cmd_vel; do
  fname=$(echo "$t" | tr '/' '_')
  timeout "$DUR" ros2 topic hz "$t" > "$OUT/hz${fname}.txt" 2>&1 &
done

# ── EKF deadline misses over the same window ─────────────────────────────────
# /rosout carries every node's log, so this catches the EKF warning without
# needing the launch console.
( timeout "$DUR" ros2 topic echo /rosout 2>/dev/null \
    | grep -c "Failed to meet update rate" > "$OUT/rosout_ekf.txt" ) &

# ── per-process CPU sampling (the main measurement) ───────────────────────────
# -c gives full command lines so Python nodes are not all just "python3".
top -b -c -w 512 -d 1 -n "$DUR" > "$OUT/top_raw.txt" 2>&1

wait

# ── summarise ────────────────────────────────────────────────────────────────
{
  echo "================ cpu_bench summary: $LABEL ================"
  echo "duration ${DUR}s   cores $(nproc)"
  echo
  echo "--- system CPU, averaged over the window ---"
  grep '^%Cpu' "$OUT/top_raw.txt" | awk '
    { for (i=1;i<=NF;i++) {
        if ($i=="us,") us+=$(i-1);
        if ($i=="sy,") sy+=$(i-1);
        if ($i=="id,") id+=$(i-1);
      }
      n++
    }
    END { if (n>0) printf "  user %.1f%%   sys %.1f%%   idle %.1f%%   (busy %.1f%%)\n",
          us/n, sy/n, id/n, 100-(id/n) }'
  echo
  echo "--- per-node %CPU (mean of per-second samples; 100% = one full core) ---"
  printf "  %-34s %8s %8s %6s\n" NODE MEAN PEAK SAMPLES
  for pat in arduino_encoder_twist_bridge scan_deskew lidar_scan_filter zupt_node \
             wifi_watchdog map_saver ekf_node slam_toolbox rplidar_node \
             controller_server planner_server bt_navigator behavior_server \
             velocity_smoother robot_state_publisher rover_web; do
    awk -v pat="$pat" '
      $1 ~ /^[0-9]+$/ && $0 ~ pat {
        c = $9 + 0; sum += c; n++; if (c > mx) mx = c
      }
      END {
        if (n > 0) printf "  %-34s %7.1f%% %7.1f%% %6d\n", pat, sum/n, mx, n;
        else       printf "  %-34s %8s %8s %6d\n", pat, "-", "-", 0
      }' "$OUT/top_raw.txt"
  done
  echo
  echo "--- topic rates (last reported average) ---"
  for f in "$OUT"/hz_*.txt; do
    [ -e "$f" ] || continue
    printf "  %-26s %s\n" "$(basename "$f" .txt | sed 's/^hz_//;s/_/\//g')" \
      "$(grep 'average rate' "$f" | tail -1 | sed 's/^ *//' || echo 'NO DATA')"
  done
  echo
  echo "--- EKF deadline misses in window ---"
  printf "  Failed to meet update rate:  %s\n" "$(cat "$OUT/rosout_ekf.txt" 2>/dev/null || echo '?')"
} > "$OUT/summary.txt" 2>&1

cat "$OUT/summary.txt"
echo
echo "=== done.  Send me: $OUT/summary.txt ==="
