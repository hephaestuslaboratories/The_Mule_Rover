#!/usr/bin/env python3
"""
analyze_straight_bag_no_cmd.py — straight-line calibration from a bag WITHOUT /cmd_vel.

Fallback for bags recorded with only /imu/data and /wheel/odom_twist: motion
runs are detected from measured wheel speed instead of commanded speed. For
every detected run it computes yaw drift, encoder distance, and curvature
kappa = yaw_drift / signed_distance, then averages and prints the recommended
RIGHT_WHEEL_TRIM update — same math as analyze_straight_bag.py.

CAVEAT vs the /cmd_vel version: this script cannot tell a commanded turn from
unintended drift. Any segment whose |kappa| exceeds --max-kappa (default
0.3 rad/m, far above plausible trim error) is assumed to be a deliberate turn
or pivot and is excluded from the average, but you should eyeball the per-run
table: only trust the result if the listed runs really were straight drives.

ASSUMES REP-103 IMU sign (+yaw = CCW). Verify once by hand before trusting
the sign of the recommendation (rotate rover CCW -> yaw must increase).

Usage:
  python3 analyze_straight_bag_no_cmd.py <bag_dir>
        [--current-right-trim 0.95] [--wheel-base 0.37]
        [--move-thr 0.05]    # m/s odom speed that counts as "moving"
        [--gap 1.0]          # s of sub-threshold speed that splits two runs
        [--min-dist 0.30]    # m minimum |distance| for a usable run
        [--max-kappa 0.3]    # rad/m above which a run is treated as a turn
Run inside the container (needs rosbag2_py / rclpy).
"""
import argparse
import math
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

IMU_TOPIC  = "/imu/data"
ODOM_TOPIC = "/wheel/odom_twist"
SETTLE_S   = 0.5     # s — extra window after last moving sample (coast-down)


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = {IMU_TOPIC, ODOM_TOPIC}
    missing = wanted - set(type_map)
    if missing:
        sys.exit(f"ERROR: bag is missing topics: {missing}")
    msg_cls = {t: get_message(type_map[t]) for t in wanted}
    data = {t: [] for t in wanted}
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        if topic in wanted:
            data[topic].append((t_ns * 1e-9, deserialize_message(raw, msg_cls[topic])))
    for t in wanted:
        data[t].sort(key=lambda x: x[0])
    return data


def yaw_of(imu_msg):
    q = imu_msg.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def find_runs_from_odom(odom, move_thr, gap_s):
    """Group contiguous above-threshold odom speeds into (t_start, t_end, mean_v)."""
    runs, cur = [], None
    for t, m in odom:
        vx = m.twist.twist.linear.x
        if abs(vx) > move_thr:
            if cur is None:
                cur = [t, t, [vx]]
            elif t - cur[1] > gap_s:
                runs.append(cur); cur = [t, t, [vx]]
            else:
                cur[1] = t; cur[2].append(vx)
    if cur is not None:
        runs.append(cur)
    return [(a, b, sum(v) / len(v)) for a, b, v in runs]


def unwrapped_yaw_delta(imu, t0, t1):
    """Yaw at t1 minus yaw at t0, unwrapped through all samples in between."""
    samples = [(t, yaw_of(m)) for t, m in imu if t0 <= t <= t1]
    if len(samples) < 2:
        return None
    total, prev = 0.0, samples[0][1]
    for _, y in samples[1:]:
        d = y - prev
        while d > math.pi:  d -= 2.0 * math.pi
        while d < -math.pi: d += 2.0 * math.pi
        total += d
        prev = y
    return total


def integrated_distance(odom, t0, t1):
    """Signed distance from /wheel/odom_twist vx (trapezoidal)."""
    pts = [(t, m.twist.twist.linear.x) for t, m in odom if t0 <= t <= t1]
    if len(pts) < 2:
        return None
    dist = 0.0
    for (ta, va), (tb, vb) in zip(pts, pts[1:]):
        dist += 0.5 * (va + vb) * (tb - ta)
    return dist


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--current-right-trim", type=float, default=1.00)
    ap.add_argument("--wheel-base", type=float, default=0.37)
    ap.add_argument("--move-thr", type=float, default=0.05)
    ap.add_argument("--gap", type=float, default=1.0)
    ap.add_argument("--min-dist", type=float, default=0.30)
    ap.add_argument("--max-kappa", type=float, default=0.3)
    args = ap.parse_args()

    data = read_bag(args.bag)
    runs = find_runs_from_odom(data[ODOM_TOPIC], args.move_thr, args.gap)
    if not runs:
        sys.exit("No motion segments found in /wheel/odom_twist "
                 f"(threshold {args.move_thr} m/s — try lowering --move-thr).")

    print(f"\n{'run':>3} {'mean_v':>7} {'dur_s':>6} {'enc_dist':>9} "
          f"{'yaw_drift':>10} {'kappa':>9}  note")
    kappas, weights = [], []
    for i, (t0, t1, v) in enumerate(runs):
        t1e = t1 + SETTLE_S
        dyaw = unwrapped_yaw_delta(data[IMU_TOPIC], t0, t1e)
        dist = integrated_distance(data[ODOM_TOPIC], t0, t1e)
        if dyaw is None or dist is None or abs(dist) < args.min_dist:
            print(f"{i:>3} {v:>7.3f} {t1 - t0:>6.2f}   -- too short / "
                  "insufficient data, skipped --")
            continue
        kappa = dyaw / dist
        note = ""
        if abs(kappa) > args.max_kappa:
            note = "TURN? excluded"
        else:
            kappas.append(kappa); weights.append(abs(dist))
        print(f"{i:>3} {v:>7.3f} {t1 - t0:>6.2f} {dist:>8.3f}m "
              f"{math.degrees(dyaw):>8.2f}deg {kappa:>9.4f}  {note}")

    if not kappas:
        sys.exit("\nNo usable straight runs (everything excluded as turn or "
                 "too short). This bag may not contain straight-line drives.")

    k_avg = sum(k * w for k, w in zip(kappas, weights)) / sum(weights)
    eps = k_avg * args.wheel_base
    new_trim = args.current_right_trim * (1.0 - eps)

    print(f"\nused {len(kappas)}/{len(runs)} detected segments")
    print("=== RESULT (assumes verified REP-103 yaw sign: CCW = +) ===")
    print(f"weighted mean curvature : {k_avg:+.4f} rad/m "
          f"({'curves LEFT' if k_avg > 0 else 'curves RIGHT'})")
    print(f"wheel asymmetry epsilon : {eps:+.4f}  (right {'faster' if eps > 0 else 'slower'})")
    print(f"RIGHT_WHEEL_TRIM        : {args.current_right_trim:.4f} -> {new_trim:.4f}")
    print("ONLY apply this if the runs listed above were genuinely straight "
          "drives — otherwise re-record with straight_test.sh.")


if __name__ == "__main__":
    main()
