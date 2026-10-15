#!/usr/bin/env python3
"""lidar_cal.py — measure the LiDAR's real mount offset against a flat wall.

Park the rover square against a flat wall, run this, and it reports where the
scan says that wall is, expressed in the base_link frame:

  * perpendicular distance from base_link origin to the wall  (checks lidar_x)
  * wall tilt vs. the ideal (perpendicular to +x)             (checks lidar yaw)
  * scatter of the points about the fitted line               (data sanity)

Give it the tape-measured truth with --true-distance and it prints the exact
URDF values to use.  Read from TF, so it always compares against whatever
rover.urdf.xacro currently claims — no hard-coded assumptions.

    # rover square against wall, wall directly in FRONT
    python3 lidar_cal.py --true-distance 0.21

`--true-distance` is measured horizontally from the wall face to the DRIVE
AXLE CENTRE (base_link sits at the axle midpoint).  With the chassis front
face touching the wall that is nominally 0.21 m on this rover — but measure
it, do not assume it.

Works in any mode (SLAM or nav); it never uses the map, only /scan + TF.
"""

import argparse
import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
import tf2_ros


class LidarCal(Node):
    def __init__(self, args):
        super().__init__("lidar_cal")
        self.args = args
        self.samples = []          # (dist, angle_deg, rms, n_pts)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.mount = None          # (x, y, yaw) from TF
        self.create_subscription(LaserScan, args.topic, self.on_scan, 10)

    # ---------------------------------------------------------------- helpers

    def lookup_mount(self, laser_frame):
        tf = self.tf_buffer.lookup_transform(
            self.args.base_frame, laser_frame, rclpy.time.Time(),
            timeout=Duration(seconds=1.0))
        t, r = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2.0 * (r.w * r.z + r.x * r.y),
                         1.0 - 2.0 * (r.y * r.y + r.z * r.z))
        return t.x, t.y, yaw

    # ---------------------------------------------------------------- callback

    def on_scan(self, msg: LaserScan):
        if len(self.samples) >= self.args.samples:
            return
        try:
            if self.mount is None:
                self.mount = self.lookup_mount(msg.header.frame_id)
        except Exception as exc:                                # noqa: BLE001
            self.get_logger().warn(f"waiting for TF: {exc}")
            return

        mx, my, myaw = self.mount
        ang = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        rng = np.asarray(msg.ranges, dtype=float)

        ok = np.isfinite(rng) & (rng > max(msg.range_min, 0.02)) \
            & (rng < min(msg.range_max, self.args.max_range))
        if ok.sum() < 20:
            return
        ang, rng = ang[ok], rng[ok]

        # laser frame -> base_link (planar)
        lx, ly = rng * np.cos(ang), rng * np.sin(ang)
        c, s = math.cos(myaw), math.sin(myaw)
        bx = c * lx - s * ly + mx
        by = s * lx + c * ly + my

        # keep the wall straight ahead: forward, inside a narrow lateral window
        sel = (bx > 0.05) & (np.abs(by) <= self.args.window)
        if sel.sum() < self.args.min_points:
            return
        pts = np.column_stack((bx[sel], by[sel]))

        # PCA line fit: smallest-eigenvalue eigenvector is the wall normal
        cen = pts.mean(axis=0)
        cov = np.cov((pts - cen).T)
        evals, evecs = np.linalg.eigh(cov)
        n = evecs[:, 0]
        if float(n @ cen) < 0:
            n = -n
        dist = float(n @ cen)
        resid = pts @ n - dist
        self.samples.append((dist,
                             math.degrees(math.atan2(n[1], n[0])),
                             float(np.sqrt((resid ** 2).mean())),
                             int(sel.sum())))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--true-distance", type=float,
                    help="tape-measured wall→drive-axle distance, metres")
    ap.add_argument("--topic", default="/scan_filtered")
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--window", type=float, default=0.35,
                    help="half-width of the forward strip used, metres")
    ap.add_argument("--max-range", type=float, default=3.0)
    ap.add_argument("--min-points", type=int, default=25)
    ap.add_argument("--samples", type=int, default=30)
    args = ap.parse_args()

    rclpy.init()
    node = LidarCal(args)
    try:
        deadline = node.get_clock().now() + Duration(seconds=30)
        while (rclpy.ok() and len(node.samples) < args.samples
               and node.get_clock().now() < deadline):
            rclpy.spin_once(node, timeout_sec=0.2)

        if not node.samples:
            print("No usable scans. Is the wall directly in front, within "
                  f"{args.max_range} m, and is {args.topic} publishing?",
                  file=sys.stderr)
            return 2

        arr = np.array(node.samples)
        d, a, rms = arr[:, 0], arr[:, 1], arr[:, 2]
        mx, my, myaw = node.mount

        print(f"\nscans used: {len(arr)}   points/scan: {arr[:, 3].mean():.0f}")
        print("\nURDF currently claims (from TF base_link -> laser_frame):")
        print(f"  lidar_x = {mx:+.4f} m   lidar_y = {my:+.4f} m   "
              f"yaw = {math.degrees(myaw):+.2f}°")
        print("\nMEASURED wall, in base_link:")
        print(f"  distance      {d.mean():+.4f} m   (sd {d.std():.4f})")
        print(f"  tilt vs ideal {a.mean():+.2f}°     (sd {a.std():.2f})"
              "   [0° = wall square to the rover]")
        print(f"  fit residual  {rms.mean() * 1000:.1f} mm rms"
              "   [large = not a flat wall / clutter in the strip]")

        print("\nSUGGESTED URDF CHANGES")
        if abs(a.mean()) < 0.75:
            print(f"  yaw: leave as is — {a.mean():+.2f}° is within noise.")
        else:
            new_yaw = myaw - math.radians(a.mean())
            print(f"  yaw: set lidar joint rpy=\"0 0 {new_yaw:.4f}\"  "
                  f"({math.degrees(new_yaw):+.2f}°)")
        if args.true_distance is None:
            print("  distance: rerun with --true-distance <tape measurement> "
                  "to get lidar_x.")
        else:
            err = d.mean() - args.true_distance
            if abs(err) < 0.01:
                print(f"  lidar_x: leave as is — off by only {err * 100:+.1f} cm.")
            else:
                print(f"  scan reads {err * 100:+.1f} cm vs tape "
                      f"({'too far' if err > 0 else 'too close'})")
                print(f"  lidar_x: {mx:+.4f}  ->  {mx - err:+.4f} m")
        print("\nApply, rebuild, and rerun — values should land near zero "
              "error.\n")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
