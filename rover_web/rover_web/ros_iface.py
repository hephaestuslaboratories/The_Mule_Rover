"""ROS-side interface for the rover web cockpit.

One rclpy node spinning on a background daemon thread.  Everything the
web server needs from the ROS graph goes through this class:

  * /map                        -> latest occupancy grid, rendered to PNG
  * TF map->base_link           -> live pose + localization health
  * /odometry/filtered          -> "core stack alive" heartbeat
  * /cmd_vel                    -> web-joystick output behind a strict deadman
  * navigate_to_pose action     -> go-to-place goals + progress feedback
  * /slam_toolbox/serialize_map -> posegraph save when finishing a SLAM run

Thread model: rclpy spins on its own thread.  The asyncio web server
calls the public methods below from other threads; those methods only
copy state under a lock or fire thread-safe sends (publish /
send_goal_async).  Nothing outside the spin thread touches executors.

SAFETY: the joystick deadman lives HERE, not in the browser.  If no
joystick message arrives for DEADMAN_S the node floods a few zero
Twists and goes idle.  A dropped WebSocket, a crashed page, or a phone
leaving WiFi all stop the rover within DEADMAN_S while manually
driving.  Autonomous motion (Nav2 goals, explore) is deliberately NOT
tied to the phone connection -- see server.py.
"""

from __future__ import annotations

import json
import math
import struct
import threading
import time
import zlib
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

import tf2_ros
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String

# Web joystick hard caps -- deliberately BELOW the drivetrain maxima.
# The phone is a coarse input device; nobody needs 1.25 m/s from a thumb.
WEB_V_MAX = 0.25          # m/s
WEB_W_MAX = 0.80          # rad/s
DEADMAN_S = 0.5           # joystick silence -> stop
ZERO_BURST = 5            # zero Twists published after deadman fires


def _grid_to_png(msg: OccupancyGrid) -> bytes:
    """Render an OccupancyGrid to a grayscale PNG using only the stdlib.

    map_saver-style colors: free=254 (white-ish), occupied=0 (black),
    unknown=205 (gray).  Grid row 0 is the map's SOUTH edge but PNG row 0
    is the TOP of the image, so rows are emitted in reverse order.
    """
    w, h = msg.info.width, msg.info.height
    data = msg.data  # int8 sequence, row-major from origin (bottom-left)
    lut = bytearray(256)
    for i in range(256):
        v = i - 256 if i > 127 else i  # undo unsigned wrap of int8
        if v < 0:
            lut[i] = 205
        elif v < 50:
            lut[i] = 254
        else:
            lut[i] = 0
    rows = []
    for r in range(h - 1, -1, -1):
        base = r * w
        row = bytes(data[base:base + w])  # int8 -> unsigned bytes (wraps)
        rows.append(b"\x00" + row.translate(bytes(lut)))
    raw = b"".join(rows)

    def chunk(tag: bytes, payload: bytes) -> bytes:
        return (struct.pack(">I", len(payload)) + tag + payload
                + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF))

    ihdr = struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0)  # 8-bit grayscale
    return (b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", ihdr)
            + chunk(b"IDAT", zlib.compress(raw, 6))
            + chunk(b"IEND", b""))


class _WebNode(Node):
    def __init__(self, iface: "RosInterface"):
        super().__init__("rover_web")
        self.iface = iface

        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(OccupancyGrid, "/map", iface._on_map, map_qos)
        self.create_subscription(
            Odometry, "/odometry/filtered", iface._on_odom, 10)
        # pose_initializer publishes latched (transient_local) — match it so a
        # server that connects after the one-shot seed still gets the status.
        autoinit_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            String, "/rover/pose_autoinit", iface._on_autoinit, autoinit_qos)
        self.create_subscription(
            String, "/rover/uptime", iface._on_uptime, 10)

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.initpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_timer(0.2, iface._pose_tick)      # 5 Hz pose
        self.create_timer(0.05, iface._joy_tick)      # 20 Hz joystick/deadman
        self.create_timer(1.0, iface._status_tick)    # 1 Hz health


class RosInterface:
    def __init__(self):
        self._lock = threading.Lock()
        self._node: Optional[_WebNode] = None
        self._thread: Optional[threading.Thread] = None

        # -- shared state (guarded by _lock) --
        self._map_msg: Optional[OccupancyGrid] = None
        self._map_seq = 0
        self._map_png: Optional[bytes] = None
        self._map_png_seq = -1
        self._map_meta: Dict = {}
        self._pose: Optional[Tuple[float, float, float, float, float]] = None
        # (x, y, yaw, qz, qw)
        self._loc_ok = False
        self._last_odom = 0.0
        self._nav2_ready = False
        self._goal: Optional[Dict] = None
        self._goal_handle = None
        self._autoinit: Optional[str] = None   # /rover/pose_autoinit (latched)
        self._uptime: Optional[Dict] = None    # /rover/uptime (parsed JSON)

        # -- joystick --
        self._joy_v = 0.0
        self._joy_w = 0.0
        self._joy_stamp = 0.0
        self._joy_active = False
        self._zeros_left = 0

    # ------------------------------------------------------------ lifecycle

    def start(self) -> None:
        rclpy.init()
        self._node = _WebNode(self)
        self._thread = threading.Thread(
            target=rclpy.spin, args=(self._node,), daemon=True,
            name="rclpy-spin")
        self._thread.start()

    def shutdown(self) -> None:
        try:
            if self._node is not None:
                self._node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass

    # ------------------------------------------------------------ callbacks

    def _on_map(self, msg: OccupancyGrid) -> None:
        with self._lock:
            self._map_msg = msg
            self._map_seq += 1
            self._map_meta = {
                "w": msg.info.width,
                "h": msg.info.height,
                "res": msg.info.resolution,
                "ox": msg.info.origin.position.x,
                "oy": msg.info.origin.position.y,
            }

    def _on_odom(self, _msg: Odometry) -> None:
        with self._lock:
            self._last_odom = time.time()

    def _on_autoinit(self, msg: String) -> None:
        with self._lock:
            self._autoinit = msg.data

    def _on_uptime(self, msg: String) -> None:
        try:
            parsed = json.loads(msg.data)
        except Exception:
            return
        with self._lock:
            self._uptime = parsed

    def _pose_tick(self) -> None:
        node = self._node
        if node is None:
            return
        try:
            tf = node.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time(),
                timeout=Duration(seconds=0.05))
            t, r = tf.transform.translation, tf.transform.rotation
            yaw = 2.0 * math.atan2(r.z, r.w)
            with self._lock:
                self._pose = (t.x, t.y, yaw, r.z, r.w)
                self._loc_ok = True
        except Exception:
            with self._lock:
                self._loc_ok = False

    def _status_tick(self) -> None:
        node = self._node
        if node is None:
            return
        ready = node.nav_client.server_is_ready()
        now = time.time()
        with self._lock:
            self._nav2_ready = ready
            # If Nav2 dies (mode switch, crash) under an in-flight goal,
            # its result callback never fires — don't leave the UI stuck
            # on "driving" forever.
            if ready:
                self._nav2_down_since = None
            elif self._goal is not None and self._goal["state"] in (
                    "sent", "driving"):
                if getattr(self, "_nav2_down_since", None) is None:
                    self._nav2_down_since = now
                elif now - self._nav2_down_since > 5.0:
                    self._goal["state"] = "failed (nav2 gone)"
                    self._goal_handle = None

    def _joy_tick(self) -> None:
        node = self._node
        if node is None:
            return
        now = time.time()
        with self._lock:
            fresh = (now - self._joy_stamp) < DEADMAN_S
            v, w = self._joy_v, self._joy_w
            active = self._joy_active
            if active and not fresh:
                # Deadman fired: flood a short burst of zeros, then idle.
                self._joy_active = False
                self._zeros_left = ZERO_BURST
            zeros = self._zeros_left
            if zeros > 0:
                self._zeros_left -= 1
        msg = Twist()
        if active and fresh:
            msg.linear.x = v
            msg.angular.z = w
            node.cmd_pub.publish(msg)
        elif zeros > 0:
            node.cmd_pub.publish(msg)  # zero Twist

    # ------------------------------------------------------------ joystick

    def set_joystick(self, v: float, w: float) -> None:
        v = max(-WEB_V_MAX, min(WEB_V_MAX, float(v)))
        w = max(-WEB_W_MAX, min(WEB_W_MAX, float(w)))
        with self._lock:
            self._joy_v, self._joy_w = v, w
            self._joy_stamp = time.time()
            self._joy_active = True

    def joystick_stop(self) -> None:
        with self._lock:
            self._joy_v = self._joy_w = 0.0
            self._joy_active = False
            self._zeros_left = ZERO_BURST

    # ------------------------------------------------------------ initial pose

    def set_initial_pose(self, x: float, y: float,
                         qz: float, qw: float) -> Tuple[bool, str]:
        """Publish /initialpose — same mechanism as RViz's 2D Pose Estimate.

        slam_toolbox localization subscribes to this and re-anchors its
        scan matching at the given pose.  Only meaningful if the rover is
        PHYSICALLY at (or within ~25 cm / small angle of) that pose.
        """
        node = self._node
        if node is None:
            return False, "ROS interface not running"
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = float(qz)
        msg.pose.pose.orientation.w = float(qw)
        # RViz-style default confidence: ~0.5 m position, ~0.25 rad yaw.
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068
        node.initpose_pub.publish(msg)
        return True, "Initial pose published"

    # ------------------------------------------------------------ Nav2 goals

    def send_goal(self, name: str, x: float, y: float,
                  qz: float, qw: float) -> Tuple[bool, str]:
        node = self._node
        if node is None or not node.nav_client.server_is_ready():
            return False, "Nav2 action server not available"
        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = node.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.orientation.z = float(qz)
        ps.pose.orientation.w = float(qw)
        goal.pose = ps
        # Cancel whatever came before, unconditionally.  Nav2 normally
        # preempts, but a goal we think is dead can still be live inside the
        # behaviour tree (mid-recovery, mid-retry) — and then it resumes
        # driving to the OLD destination.  Cancelling is harmless on an
        # already-finished handle, so always do it.
        with self._lock:
            stale = self._goal_handle
            self._goal_handle = None
        if stale is not None:
            try:
                stale.cancel_goal_async()
            except Exception:
                pass
        with self._lock:
            self._goal = {"name": name, "state": "sent", "remaining": None}
        fut = node.nav_client.send_goal_async(
            goal, feedback_callback=self._goal_feedback)
        fut.add_done_callback(self._goal_response)
        return True, f"Goal sent: {name}"

    def _goal_feedback(self, msg) -> None:
        rem = getattr(msg.feedback, "distance_remaining", None)
        with self._lock:
            if self._goal is not None:
                self._goal["remaining"] = rem

    def _goal_response(self, future) -> None:
        handle = future.result()
        with self._lock:
            if not handle.accepted:
                if self._goal is not None:
                    self._goal["state"] = "rejected"
                return
            self._goal_handle = handle
            if self._goal is not None:
                self._goal["state"] = "driving"
        handle.get_result_async().add_done_callback(self._goal_result)

    def _goal_result(self, future) -> None:
        status = future.result().status
        with self._lock:
            if self._goal is not None:
                # 4 == SUCCEEDED, 5 == CANCELED, 6 == ABORTED
                self._goal["state"] = {4: "arrived", 5: "canceled"}.get(
                    status, f"failed ({status})")
            self._goal_handle = None

    def cancel_goal(self) -> None:
        with self._lock:
            handle = self._goal_handle
        if handle is not None:
            handle.cancel_goal_async()

    def clear_goal(self) -> None:
        with self._lock:
            if self._goal is not None and self._goal["state"] in (
                    "arrived", "canceled", "rejected") or (
                    self._goal is not None
                    and self._goal["state"].startswith("failed")):
                self._goal = None

    # ------------------------------------------------------------ SLAM save

    def serialize_map(self, filename: str,
                      timeout_s: float = 30.0) -> Tuple[bool, str]:
        """Call /slam_toolbox/serialize_map.  Blocking; call off-loop."""
        node = self._node
        if node is None:
            return False, "ROS interface not running"
        try:
            from slam_toolbox.srv import SerializePoseGraph
        except ImportError:
            return False, "slam_toolbox python interfaces not found"
        client = node.create_client(
            SerializePoseGraph, "/slam_toolbox/serialize_map")
        try:
            deadline = time.time() + 5.0
            while not client.service_is_ready():
                if time.time() > deadline:
                    return False, "serialize_map service not available"
                time.sleep(0.1)
            req = SerializePoseGraph.Request()
            req.filename = filename
            done = threading.Event()
            box: Dict = {}

            def _cb(fut):
                try:
                    box["resp"] = fut.result()
                except Exception as exc:  # noqa: BLE001
                    box["err"] = str(exc)
                done.set()

            client.call_async(req).add_done_callback(_cb)
            if not done.wait(timeout_s):
                return False, "serialize_map timed out"
            if "err" in box:
                return False, f"serialize_map error: {box['err']}"
            result = getattr(box["resp"], "result", 0)
            # slam_toolbox convention: result == 0 means SUCCESS.
            if result == 0:
                return True, "posegraph serialized"
            return False, f"serialize_map returned {result}"
        finally:
            node.destroy_client(client)

    # ------------------------------------------------------------ snapshots

    def get_map_png(self) -> Tuple[Optional[bytes], Dict, int]:
        """Return (png, meta, seq); renders lazily, one render per new map."""
        with self._lock:
            msg, seq = self._map_msg, self._map_seq
            if msg is None:
                return None, {}, 0
            if seq == self._map_png_seq:
                return self._map_png, dict(self._map_meta), seq
        png = _grid_to_png(msg)  # render outside the lock (can take ~10 ms)
        with self._lock:
            self._map_png, self._map_png_seq = png, seq
            return png, dict(self._map_meta), seq

    def get_pose(self) -> Optional[Dict]:
        with self._lock:
            if self._pose is None or not self._loc_ok:
                return None
            x, y, yaw, qz, qw = self._pose
            return {"x": x, "y": y, "yaw": yaw, "qz": qz, "qw": qw}

    def get_status(self) -> Dict:
        now = time.time()
        with self._lock:
            return {
                "stack": (now - self._last_odom) < 2.0,
                "loc": self._loc_ok,
                "nav2": self._nav2_ready,
                "map": self._map_msg is not None,
                "goal": dict(self._goal) if self._goal else None,
                "autoinit": self._autoinit,
                "uptime": dict(self._uptime) if self._uptime else None,
            }

    def clear_autoinit(self) -> None:
        """Forget the last autoinit status (called on stop/mode change so a
        stale SEEDED from the previous nav session can't be shown)."""
        with self._lock:
            self._autoinit = None
