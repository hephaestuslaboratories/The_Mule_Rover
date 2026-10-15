#!/usr/bin/env python3
"""
wheel_profile.py — per-wheel velocity timeline for single-wheel isolation tests.

For each command burst in the bag, prints second-by-second measured LEFT and
RIGHT wheel velocities (converted to BODY frame: + = rover-forward), the
commanded per-wheel targets, and a verdict: did the commanded wheel move, or
its mirror?

Usage:
  python3 wheel_profile.py <bag_dir> [--wheel-base 0.37]
Needs /cmd_vel, /wheel/left_velocity, /wheel/right_velocity in the bag.
Run inside the container (needs rosbag2_py / rclpy).
"""
import argparse
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

CMD_TOPIC = "/cmd_vel"
VL_TOPIC  = "/wheel/left_velocity"
VR_TOPIC  = "/wheel/right_velocity"
GAP_S     = 1.0


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = {CMD_TOPIC, VL_TOPIC, VR_TOPIC}
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


def find_runs(cmds):
    runs, cur = [], None
    for t, m in cmds:
        v, w = m.linear.x, m.angular.z
        if abs(v) > 0.005 or abs(w) > 0.05:
            if cur is None:
                cur = [t, t, [v], [w]]
            elif t - cur[1] > GAP_S:
                runs.append(cur); cur = [t, t, [v], [w]]
            else:
                cur[1] = t; cur[2].append(v); cur[3].append(w)
    if cur is not None:
        runs.append(cur)
    return [(a, b, sum(v) / len(v), sum(w) / len(w)) for a, b, v, w in runs]


def mean_in(series, t0, t1):
    vals = [m.data for t, m in series if t0 <= t <= t1]
    return (sum(vals) / len(vals)) if vals else 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--wheel-base", type=float, default=0.37)
    args = ap.parse_args()
    hb = args.wheel_base / 2.0

    data = read_bag(args.bag)
    runs = find_runs(data[CMD_TOPIC])
    if not runs:
        sys.exit("No command bursts found.")

    for i, (t0, t1, v_cmd, w_cmd) in enumerate(runs):
        # Body-frame per-wheel targets (REP-103)
        tgt_l = v_cmd - w_cmd * hb
        tgt_r = v_cmd + w_cmd * hb
        print(f"\n=== run {i}: cmd v={v_cmd:+.3f} w={w_cmd:+.2f} "
              f"-> target L={tgt_l:+.3f} R={tgt_r:+.3f} m/s (body frame) ===")
        print(f"{'t(s)':>5} {'vL_body':>8} {'vR_body':>8}")
        tt = 0.0
        dur = t1 - t0
        while tt <= dur + 1e-6:
            # raw topics are firmware-frame: body = -raw
            vl = -mean_in(data[VL_TOPIC], t0 + tt, t0 + tt + 1.0)
            vr = -mean_in(data[VR_TOPIC], t0 + tt, t0 + tt + 1.0)
            print(f"{tt:>5.0f} {vl:>8.3f} {vr:>8.3f}")
            tt += 1.0

        vl_m = -mean_in(data[VL_TOPIC], t0 + 1.0, t1)   # skip 1s ramp
        vr_m = -mean_in(data[VR_TOPIC], t0 + 1.0, t1)
        print(f"  mean (after 1s): L={vl_m:+.3f}  R={vr_m:+.3f}")

        exp = "L" if abs(tgt_l) > abs(tgt_r) else "R"
        act = "L" if abs(vl_m) > abs(vr_m) else "R"
        ratio = min(abs(vl_m), abs(vr_m)) / max(abs(vl_m), abs(vr_m), 1e-6)
        clean = "clean single-wheel" if ratio < 0.3 else \
                "BOTH wheels active (loops fighting?)"
        verdict = "CORRECT wheel" if exp == act else "MIRRORED wheel"
        print(f"  expected mover: {exp} | actual dominant: {act} -> {verdict}; {clean}")


if __name__ == "__main__":
    main()
