#!/usr/bin/env python3
"""costmap_peek.py — see what the local costmap thinks, without RViz.

Answers the question RViz would answer: "is the ToF belt marking what I put in
front of the rover, and does the mark go away when I remove it?"

    # nearest obstacle, refreshing — the quick check
    python3 costmap_peek.py

    # ASCII picture of the 3x3 m window, rover at the centre facing UP
    python3 costmap_peek.py --ascii

The bearing is in the ROVER's frame: 0 deg straight ahead, +ve to the LEFT,
180 deg behind.  So "0.31 m at -45 deg" is the front-right corner — which is
exactly where tof_corner_right is pointing.

WHAT TO LOOK FOR when testing the belt:
  1. Empty floor, rover stopped   -> no obstacle within ~0.5 m
  2. Put a shoebox 30 cm ahead    -> appears at ~0.30 m, bearing near 0
  3. TAKE THE BOX AWAY            -> must clear within a second or two
Step 3 is the one that matters.  If marks linger, clear_on_max_reading is not
doing its job and the rover will slowly wall itself in.

Note the LiDAR marks this same costmap, so a box tall enough to break the
0.56 m scan plane will show up with the belt switched off too.  Test with
something LOW (a shoe, a folded towel) to prove the belt specifically.
"""

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import tf2_ros

LETHAL = 99          # >= this counts as an obstacle (253 = lethal, 99 = inscribed)


class CostmapPeek(Node):
    def __init__(self, args):
        super().__init__("costmap_peek")
        self.args = args
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(OccupancyGrid, args.topic, self.on_grid, 1)
        self.get_logger().info(f"watching {args.topic} — Ctrl-C to stop")

    def robot_in_grid(self, grid):
        """Rover pose (x, y, yaw) in the costmap's own frame."""
        tf = self.tf_buffer.lookup_transform(
            grid.header.frame_id, self.args.base_frame, rclpy.time.Time())
        t, r = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2.0 * (r.w * r.z + r.x * r.y),
                         1.0 - 2.0 * (r.y * r.y + r.z * r.z))
        return t.x, t.y, yaw

    def on_grid(self, grid):
        try:
            rx, ry, ryaw = self.robot_in_grid(grid)
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().warn(f"no TF yet: {exc}", throttle_duration_sec=5.0)
            return

        info = grid.info
        res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y

        nearest = None      # (distance, bearing_deg)
        count = 0
        for iy in range(info.height):
            row = iy * info.width
            for ix in range(info.width):
                if grid.data[row + ix] < LETHAL:
                    continue
                count += 1
                # cell centre in the costmap frame -> rover frame
                cx = ox + (ix + 0.5) * res
                cy = oy + (iy + 0.5) * res
                dx, dy = cx - rx, cy - ry
                fwd = dx * math.cos(ryaw) + dy * math.sin(ryaw)
                lft = -dx * math.sin(ryaw) + dy * math.cos(ryaw)
                d = math.hypot(fwd, lft)
                if nearest is None or d < nearest[0]:
                    nearest = (d, math.degrees(math.atan2(lft, fwd)))

        if nearest is None:
            print(f"  cells={count:<5}  nothing in the window", end="\r", flush=True)
        else:
            d, brg = nearest
            print(f"  cells={count:<5}  nearest {d:5.2f} m at {brg:+6.1f} deg "
                  f"({self.where(brg)})      ", end="\r", flush=True)

        if self.args.ascii:
            self.draw(grid, rx, ry, ryaw)

    @staticmethod
    def where(brg):
        a = abs(brg)
        if a <= 20:   return "ahead"
        if a <= 70:   return "front-left" if brg > 0 else "front-right"
        if a <= 110:  return "left" if brg > 0 else "right"
        if a <= 160:  return "rear-left" if brg > 0 else "rear-right"
        return "behind"

    def draw(self, grid, rx, ry, ryaw):
        """Top-down picture in the ROVER's frame: forward is UP."""
        info = grid.info
        res = info.resolution
        span = self.args.span
        step = self.args.step
        n = int(span / (res * step))
        lines = []
        for row in range(n, -n - 1, -1):          # forward on top
            out = []
            for col in range(-n, n + 1):          # left on the left
                fwd = row * res * step
                lft = -col * res * step
                # rover frame -> costmap frame -> cell index
                cx = rx + fwd * math.cos(ryaw) - lft * math.sin(ryaw)
                cy = ry + fwd * math.sin(ryaw) + lft * math.cos(ryaw)
                ix = int((cx - info.origin.position.x) / res)
                iy = int((cy - info.origin.position.y) / res)
                if not (0 <= ix < info.width and 0 <= iy < info.height):
                    out.append(" ")
                    continue
                v = grid.data[iy * info.width + ix]
                if row == 0 and col == 0:   out.append("R")
                elif v < 0:                 out.append(" ")   # unknown
                elif v >= 253:              out.append("#")   # lethal
                elif v >= LETHAL:           out.append("+")   # inscribed
                elif v > 0:                 out.append(":")   # inflation
                else:                       out.append(".")   # free
            lines.append("".join(out))
        print("\033[2J\033[H" + "\n".join(lines))
        print("\n  R=rover (facing UP)  #=lethal  +=inscribed  :=inflation  .=free")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", default="/local_costmap/costmap")
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--ascii", action="store_true", help="draw the window")
    ap.add_argument("--span", type=float, default=1.0,
                    help="--ascii: metres either side of the rover (default: 1.0)")
    ap.add_argument("--step", type=int, default=1,
                    help="--ascii: cells per character, 2 halves the size (default: 1)")
    args = ap.parse_args()

    rclpy.init()
    node = CostmapPeek(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
