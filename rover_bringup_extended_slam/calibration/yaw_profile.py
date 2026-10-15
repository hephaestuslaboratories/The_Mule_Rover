#!/usr/bin/env python3
"""
yaw_profile.py — per-second yaw/distance timeline for each run in a straight-test bag.

Answers: is the yaw drift a STEADY curvature (yaw grows roughly linearly while
driving -> real wheel asymmetry, apply the trim fix) or an END-OF-RUN transient
(yaw flat during cruise, jumps in the last second(s)/settle -> stop-behavior
issue, do NOT apply the trim fix)?

Usage:
  python3 yaw_profile.py <bag_dir>
Run inside the container (needs rosbag2_py / rclpy).
"""
import math
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

CMD_TOPIC  = "/cmd_vel"
IMU_TOPIC  = "/imu/data"
ODOM_TOPIC = "/wheel/odom_twist"
MOVE_THR   = 0.005
GAP_S      = 1.0
SETTLE_S   = 1.5     # longer than analyzer's 0.5 so the full stop is visible


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
    if cur is not None:
        runs.append(cur)
    return [(a, b, sum(v) / len(v)) for a, b, v in runs]


def cum_yaw_series(imu, t0, t1):
    """[(t, cumulative unwrapped yaw)] over [t0, t1]."""
    samples = [(t, yaw_of(m)) for t, m in imu if t0 <= t <= t1]
    if len(samples) < 2:
        return []
    out = [(samples[0][0], 0.0)]
    total, prev = 0.0, samples[0][1]
    for t, y in samples[1:]:
        d = y - prev
        while d > math.pi:  d -= 2.0 * math.pi
        while d < -math.pi: d += 2.0 * math.pi
        total += d
        prev = y
        out.append((t, total))
    return out


def cum_dist_series(odom, t0, t1):
    """[(t, cumulative signed distance)] over [t0, t1]."""
    pts = [(t, m.twist.twist.linear.x) for t, m in odom if t0 <= t <= t1]
    if len(pts) < 2:
        return []
    out = [(pts[0][0], 0.0)]
    dist = 0.0
    for (ta, va), (tb, vb) in zip(pts, pts[1:]):
        dist += 0.5 * (va + vb) * (tb - ta)
        out.append((tb, dist))
    return out


def value_at(series, t):
    """Last value in series at or before t (series sorted by time)."""
    v = series[0][1]
    for ts, val in series:
        if ts > t:
            break
        v = val
    return v


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    data = read_bag(sys.argv[1])
    runs = find_runs(data[CMD_TOPIC])
    if not runs:
        sys.exit("No motion bursts found in /cmd_vel.")

    for i, (t0, t1, v) in enumerate(runs):
        t1e = t1 + SETTLE_S
        yaws  = cum_yaw_series(data[IMU_TOPIC],  t0, t1e)
        dists = cum_dist_series(data[ODOM_TOPIC], t0, t1e)
        if not yaws or not dists:
            print(f"\n=== run {i}: insufficient data ===")
            continue

        print(f"\n=== run {i}  (cmd {v:+.3f} m/s, {t1 - t0:.1f}s cmd window, "
              f"+{SETTLE_S}s settle) ===")
        print(f"{'t(s)':>6} {'dist(m)':>8} {'yaw(deg)':>9} {'dyaw/s(deg)':>12}")
        dur = t1e - t0
        prev_yaw = 0.0
        tt = 0.0
        while tt <= dur + 1e-6:
            ts = t0 + tt
            yw = math.degrees(value_at(yaws, ts))
            ds = value_at(dists, ts)
            marker = "  <- cmds end" if abs((ts - t1)) < 0.5 else ""
            print(f"{tt:>6.1f} {ds:>8.3f} {yw:>9.2f} {yw - prev_yaw:>12.2f}{marker}")
            prev_yaw = yw
            tt += 1.0

        # cruise-vs-stop split: yaw accumulated up to 1 s before cmds end,
        # versus everything after that (decel + stop + settle).
        t_split = t1 - 1.0
        yaw_cruise = math.degrees(value_at(yaws, t_split))
        yaw_total  = math.degrees(yaws[-1][1])
        dist_cruise = value_at(dists, t_split)
        k_cruise = (math.radians(yaw_cruise) / dist_cruise) if abs(dist_cruise) > 0.05 else float('nan')
        print(f"  cruise (first {t_split - t0:.1f}s): {yaw_cruise:+7.2f} deg over "
              f"{dist_cruise:.3f} m  -> kappa {k_cruise:+.4f} rad/m")
        print(f"  stop window (last 1s + settle): {yaw_total - yaw_cruise:+7.2f} deg")


if __name__ == "__main__":
    main()
