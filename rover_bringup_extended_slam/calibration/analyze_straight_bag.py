#!/usr/bin/env python3
"""
analyze_straight_bag.py — derive straight-line calibration from a test bag.

Reads a rosbag2 recorded by straight_test.sh and, for every motion burst found
in /cmd_vel, computes:
  * yaw drift (from /imu/data orientation quaternion, unwrapped)
  * encoder distance (integrated /wheel/odom_twist vx)
  * per-run curvature kappa = yaw_drift / signed_distance
Then averages curvature across runs (forward AND backward are sign-consistent:
the same physical asymmetry gives the same kappa in both directions) and prints
the recommended RIGHT_WHEEL_TRIM update.

Math (differential drive):
  omega = (vR - vL) / b            b = WHEEL_BASE_M (track width)
  with vR = v(1+e/2), vL = v(1-e/2):   omega = v*e/b
  curvature kappa = omega/v = e/b  ->  e = kappa * b
  e > 0  => right side effectively faster => rover curves LEFT (CCW, +yaw fwd)
  fix: NEW_RIGHT_TRIM = CURRENT_RIGHT_TRIM * (1 - e)

ASSUMES REP-103 IMU sign (+yaw = CCW). Verify once by hand before trusting
the sign of the recommendation (rotate rover CCW -> yaw must increase).

Usage:
  python3 analyze_straight_bag.py <bag_dir> [--current-right-trim 0.95]
                                  [--wheel-base 0.37]
                                  [--tape d1,d2,...]   # measured ground-truth
                                                       # distances (m), in run order
Run inside the container (needs rosbag2_py / rclpy).
"""
import argparse
import math
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

CMD_TOPIC  = "/cmd_vel"
IMU_TOPIC  = "/imu/data"
ODOM_TOPIC = "/wheel/odom_twist"
MOVE_THR   = 0.005   # m/s — a cmd above this counts as "moving"
GAP_S      = 1.0     # s  — gap in nonzero cmds that splits two runs
SETTLE_S   = 0.5     # s  — extra window after last cmd to capture coast-down


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = {CMD_TOPIC, IMU_TOPIC, ODOM_TOPIC}
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


def find_runs(cmds):
    """Group contiguous nonzero-vx commands into (t_start, t_end, mean_v) runs."""
    runs, cur = [], None
    for t, m in cmds:
        vx = m.linear.x
        if abs(vx) > MOVE_THR:
            if cur is None:
                cur = [t, t, [vx]]
            elif t - cur[1] > GAP_S:
                runs.append(cur); cur = [t, t, [vx]]
            else:
                cur[1] = t; cur[2].append(vx)
        # zeros don't close a run immediately; GAP_S handles separation
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
    ap.add_argument("--tape", type=str, default="",
                    help="comma-separated measured distances (m), in run order")
    args = ap.parse_args()

    data = read_bag(args.bag)
    runs = find_runs(data[CMD_TOPIC])
    if not runs:
        sys.exit("No motion bursts found in /cmd_vel.")
    tape = [float(x) for x in args.tape.split(",")] if args.tape else []

    print(f"\n{'run':>3} {'cmd_v':>7} {'dur_s':>6} {'enc_dist':>9} {'yaw_drift':>10} "
          f"{'kappa':>9} {'tape':>6} {'enc/tape':>8}")
    kappas, weights, scale_ratios = [], [], []
    for i, (t0, t1, v) in enumerate(runs):
        t1e = t1 + SETTLE_S
        dyaw = unwrapped_yaw_delta(data[IMU_TOPIC], t0, t1e)
        dist = integrated_distance(data[ODOM_TOPIC], t0, t1e)
        if dyaw is None or dist is None or abs(dist) < 0.05:
            print(f"{i:>3} {v:>7.3f}   -- insufficient data, skipped --")
            continue
        kappa = dyaw / dist                     # sign-consistent fwd & back
        kappas.append(kappa); weights.append(abs(dist))
        tp = tape[i] if i < len(tape) else None
        ratio = (abs(dist) / tp) if tp else None
        if ratio:
            scale_ratios.append(ratio)
        print(f"{i:>3} {v:>7.3f} {t1 - t0:>6.2f} {dist:>8.3f}m "
              f"{math.degrees(dyaw):>8.2f}deg {kappa:>9.4f} "
              f"{(f'{tp:.3f}' if tp else '  --'):>6} "
              f"{(f'{ratio:.4f}' if ratio else '   --'):>8}")

    if not kappas:
        sys.exit("No usable runs.")

    k_avg = sum(k * w for k, w in zip(kappas, weights)) / sum(weights)
    eps = k_avg * args.wheel_base
    new_trim = args.current_right_trim * (1.0 - eps)

    print("\n=== RESULT (assumes verified REP-103 yaw sign: CCW = +) ===")
    print(f"weighted mean curvature : {k_avg:+.4f} rad/m "
          f"({'curves LEFT' if k_avg > 0 else 'curves RIGHT'})")
    print(f"wheel asymmetry epsilon : {eps:+.4f}  (right {'faster' if eps > 0 else 'slower'})")
    print(f"RIGHT_WHEEL_TRIM        : {args.current_right_trim:.4f} -> {new_trim:.4f}")
    print("Update the constant in the firmware and reflash.")

    if scale_ratios:
        r = sum(scale_ratios) / len(scale_ratios)
        print(f"\nencoder/tape distance ratio: {r:.4f}")
        if abs(r - 1.0) > 0.02:
            print(f"  > 2% off: effective WHEEL_DIAMETER_M = 0.120 / {r:.4f} "
                  f"= {0.120 / r:.4f} m")
            print("  Update BOTH firmware WHEEL_DIAMETER_M and bridge "
                  "wheel_diameter_m or odometry lies to SLAM.")
        else:
            print("  within 2% — wheel diameter calibration is fine, leave it.")


if __name__ == "__main__":
    main()
