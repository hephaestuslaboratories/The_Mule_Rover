#!/usr/bin/env python3
"""
analyze_turn_bag.py — turn calibration analysis (in-place spins AND arcs).

Reads a rosbag2 of turn-test bursts and, for every commanded-turn run, compares
three yaw rates over the steady-state window (after ramp-in, before cmds end):

  w_cmd   commanded /cmd_vel angular.z
  w_imu   ground truth: IMU quaternion yaw delta / time (gyro-z mean printed too)
  w_enc   raw wheel-differential: (vR - vL) / wheel_base from /wheel/*_velocity

and derives the two calibration numbers:

  execution ratio  w_imu / w_cmd   -> does the rover DO what is commanded?
                                      (firmware TURN_BIAS_PWM / FF gains / PID)
  odometry ratio   w_imu / w_enc   -> what the bridge's spin_yaw_scale SHOULD be
                                      (encoders over-report yaw when wheels scrub;
                                      also = geometric/effective wheelbase ratio)

Runs are grouped by rounded (v_cmd, w_cmd) so repeats average together, and
split into SPIN (|v| < 0.02) vs ARC groups.  For arcs the achieved turn radius
v/w is printed next to the commanded one.

Usage:
  python3 analyze_turn_bag.py <bag_dir> [--wheel-base 0.37]
                              [--ramp-skip 1.5]   # s ignored at run start
                              [--meas d1,d2,...]  # physically measured TOTAL
                                                  # rotation per run (deg,
                                                  # positive magnitudes, in
                                                  # run order — like --tape)
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
VL_TOPIC   = "/wheel/left_velocity"
VR_TOPIC   = "/wheel/right_velocity"
W_THR      = 0.05    # rad/s — a cmd above this counts as "turning"
V_THR      = 0.005   # m/s
GAP_S      = 1.0     # s — gap in active cmds that splits two runs
SPIN_V     = 0.02    # m/s — below this mean |v_cmd| a run is an in-place spin


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = {CMD_TOPIC, IMU_TOPIC, ODOM_TOPIC, VL_TOPIC, VR_TOPIC}
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


def find_turn_runs(cmds):
    """Group contiguous turn commands into (t0, t1, mean_v, mean_w) runs."""
    runs, cur = [], None
    for t, m in cmds:
        v, w = m.linear.x, m.angular.z
        active = abs(w) > W_THR or abs(v) > V_THR
        if active:
            if cur is None:
                cur = [t, t, [v], [w]]
            elif t - cur[1] > GAP_S:
                runs.append(cur); cur = [t, t, [v], [w]]
            else:
                cur[1] = t; cur[2].append(v); cur[3].append(w)
    if cur is not None:
        runs.append(cur)
    out = []
    for a, b, vs, ws in runs:
        mv, mw = sum(vs) / len(vs), sum(ws) / len(ws)
        if abs(mw) > W_THR:          # keep only runs that are actually turns
            out.append((a, b, mv, mw))
    return out


def unwrapped_yaw_delta(imu, t0, t1):
    samples = [(t, yaw_of(m)) for t, m in imu if t0 <= t <= t1]
    if len(samples) < 2:
        return None, 0.0
    total, prev = 0.0, samples[0][1]
    for _, y in samples[1:]:
        d = y - prev
        while d > math.pi:  d -= 2.0 * math.pi
        while d < -math.pi: d += 2.0 * math.pi
        total += d
        prev = y
    return total, samples[-1][0] - samples[0][0]


def mean_in_window(series, t0, t1, get):
    vals = [get(m) for t, m in series if t0 <= t <= t1]
    return (sum(vals) / len(vals)) if vals else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--wheel-base", type=float, default=0.37)
    ap.add_argument("--ramp-skip", type=float, default=1.5,
                    help="seconds ignored at run start (accel ramp)")
    ap.add_argument("--meas", type=str, default="",
                    help="comma-separated measured total rotations (deg, "
                         "positive magnitudes), in run order")
    args = ap.parse_args()
    meas = [float(x) for x in args.meas.split(",")] if args.meas else []

    data = read_bag(args.bag)
    runs = find_turn_runs(data[CMD_TOPIC])
    if not runs:
        sys.exit("No turn runs found in /cmd_vel (|w| must exceed "
                 f"{W_THR} rad/s).")

    # left/right wheel topics are std_msgs/Float32 in the firmware frame; the
    # (vR - vL)/base differential is invariant under the bridge's 180-deg
    # linear flip, so it can be used directly as body yaw rate.
    print(f"\n{'run':>3} {'type':>4} {'v_cmd':>6} {'w_cmd':>6} {'dur':>5} "
          f"{'w_imu':>7} {'gyro_z':>7} {'w_enc':>7} {'w_odom':>7} "
          f"{'imu/cmd':>8} {'imu/enc':>8} {'turn_deg':>8} "
          f"{'meas':>7} {'imu/meas':>8}")

    groups, meas_ratios = {}, []
    for i, (t0, t1, v_cmd, w_cmd) in enumerate(runs):
        ts, te = t0 + args.ramp_skip, t1
        if te - ts < 2.0:
            print(f"{i:>3}  --  run too short after ramp-skip, skipped")
            continue
        dyaw, imu_dur = unwrapped_yaw_delta(data[IMU_TOPIC], ts, te)
        dyaw_full, _ = unwrapped_yaw_delta(data[IMU_TOPIC], t0, t1 + 1.0)
        gz = mean_in_window(data[IMU_TOPIC], ts, te,
                            lambda m: m.angular_velocity.z)
        vl = mean_in_window(data[VL_TOPIC], ts, te, lambda m: m.data)
        vr = mean_in_window(data[VR_TOPIC], ts, te, lambda m: m.data)
        w_odom = mean_in_window(data[ODOM_TOPIC], ts, te,
                                lambda m: m.twist.twist.angular.z)
        if dyaw is None or imu_dur <= 0 or vl is None or vr is None:
            print(f"{i:>3}  --  insufficient data, skipped")
            continue
        w_imu = dyaw / imu_dur
        # Raw wheel topics are mirror-frame (sides keep sides, signs negated
        # vs body), so the yaw differential is (vl - vr), matching the
        # corrected bridge formula.
        w_enc = (vl - vr) / args.wheel_base
        kind = "SPIN" if abs(v_cmd) < SPIN_V else "ARC"
        r_exec = w_imu / w_cmd if abs(w_cmd) > 1e-6 else float("nan")
        r_odom = (w_imu / w_enc) if abs(w_enc) > 0.02 else float("nan")
        deg_full = math.degrees(dyaw_full) if dyaw_full is not None else 0.0
        mp = meas[i] if i < len(meas) else None
        r_meas = (abs(deg_full) / mp) if mp else None
        if r_meas:
            meas_ratios.append(r_meas)
        print(f"{i:>3} {kind:>4} {v_cmd:>6.3f} {w_cmd:>6.2f} {te - ts:>5.1f} "
              f"{w_imu:>7.3f} {gz:>7.3f} {w_enc:>7.3f} {w_odom:>7.3f} "
              f"{r_exec:>8.3f} {r_odom:>8.3f} {deg_full:>8.1f} "
              f"{(f'{mp:.0f}' if mp else '   --'):>7} "
              f"{(f'{r_meas:.4f}' if r_meas else '    --'):>8}")

        key = (kind, round(v_cmd, 2), round(w_cmd, 2))
        groups.setdefault(key, []).append((w_imu, w_enc, w_cmd, v_cmd))

    if not groups:
        sys.exit("No usable runs.")

    print("\n=== GROUP AVERAGES (repeats merged) ===")
    print(f"{'type':>4} {'v_cmd':>6} {'w_cmd':>6} {'n':>2} {'w_imu':>7} "
          f"{'imu/cmd':>8} {'imu/enc':>8} {'radius m (cmd->act)':>20}")
    spin_odom_ratios, all_exec = [], []
    for (kind, v_cmd, w_cmd), rows in sorted(groups.items()):
        w_imu = sum(r[0] for r in rows) / len(rows)
        w_enc = sum(r[1] for r in rows) / len(rows)
        r_exec = w_imu / w_cmd
        r_odom = (w_imu / w_enc) if abs(w_enc) > 0.02 else float("nan")
        all_exec.append((kind, v_cmd, w_cmd, r_exec))
        radius = ""
        if kind == "ARC" and abs(w_imu) > 1e-3:
            radius = f"{abs(v_cmd / w_cmd):>6.2f} -> {abs(v_cmd / w_imu):>6.2f}"
        if kind == "SPIN" and not math.isnan(r_odom):
            spin_odom_ratios.append(r_odom)
        print(f"{kind:>4} {v_cmd:>6.2f} {w_cmd:>6.2f} {len(rows):>2} "
              f"{w_imu:>7.3f} {r_exec:>8.3f} {r_odom:>8.3f} {radius:>20}")

    print("\n=== CALIBRATION HINTS ===")
    print("execution (imu/cmd): 1.00 = perfect. Consistent offset across rates")
    print("  -> firmware WHEEL_BASE_M effectively wrong (skid): "
          "eff_base = 0.37 * cmd/imu ratio.")
    print("  Rate-dependent (bad at low w, ok high) -> stiction/TURN_BIAS_PWM.")
    if spin_odom_ratios:
        r = sum(spin_odom_ratios) / len(spin_odom_ratios)
        print(f"odometry: suggested bridge spin_yaw_scale = {r:.3f} "
              f"(currently 0.73); implied effective wheelbase = "
              f"{args.wheel_base / r:.3f} m")
    if meas_ratios:
        r = sum(meas_ratios) / len(meas_ratios)
        print(f"IMU vs physical measurement: imu/meas = {r:.4f} "
              f"({len(meas_ratios)} runs)")
        if abs(r - 1.0) > 0.03:
            print("  >3% off: re-measure carefully before distrusting the IMU "
                  "(quaternion yaw scale errors are rare; sloppy protractor "
                  "readings are not).")
        else:
            print("  within 3% — IMU yaw is a valid ground truth for the "
                  "ratios above.")


if __name__ == "__main__":
    main()
