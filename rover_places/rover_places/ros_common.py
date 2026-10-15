"""Shared ROS2 plumbing for the rover_places CLI tools."""

from __future__ import annotations

import os
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import tf2_ros
from geometry_msgs.msg import PoseStamped


# The map name scopes saved places.  Set it via the ROVER_MAP env var or
# the --map flag so that poses saved against one SLAM run are never
# silently reused against a different one.
DEFAULT_MAP_NAME = os.environ.get("ROVER_MAP", "default")


class PoseReader(Node):
    """Minimal node that reads the robot's current pose from tf2."""

    def __init__(
        self,
        node_name: str = "rover_places_pose_reader",
        map_frame: str = "map",
        base_frame: str = "base_link",
    ):
        super().__init__(node_name)
        self.map_frame = map_frame
        self.base_frame = base_frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def current_pose(
        self, timeout_sec: float = 5.0
    ) -> Optional[Tuple[float, float, float, float]]:
        """Return (x, y, qz, qw) in the map frame, or None on timeout.

        Spins the node while waiting so the TF listener actually receives
        transforms -- a lookup without spinning is the single most common
        reason this "mysteriously" times out.
        """
        deadline = self.get_clock().now() + Duration(seconds=timeout_sec)
        last_error = ""

        while rclpy.ok() and self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.base_frame,
                    rclpy.time.Time(),  # latest available
                    timeout=Duration(seconds=0.1),
                )
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as exc:
                last_error = str(exc)
                continue

            t = tf.transform.translation
            r = tf.transform.rotation
            return (t.x, t.y, r.z, r.w)

        self.get_logger().error(
            f"Timed out waiting for transform {self.map_frame} -> "
            f"{self.base_frame}. Last error: {last_error or 'none'}"
        )
        self.get_logger().error(
            "Check that SLAM (or AMCL) is running and publishing the map "
            "frame -- run: ros2 run tf2_tools view_frames"
        )
        return None


def make_pose_stamped(node: Node, x: float, y: float, qz: float, qw: float,
                      frame: str = "map") -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    pose.pose.orientation.z = float(qz)
    pose.pose.orientation.w = float(qw)
    return pose


def add_common_args(parser) -> None:
    parser.add_argument(
        "--map",
        default=DEFAULT_MAP_NAME,
        help=(
            "Map name these places belong to. Poses are only valid for the "
            f"map they were recorded on. Default: {DEFAULT_MAP_NAME} "
            "(override with the ROVER_MAP env var)."
        ),
    )
    parser.add_argument(
        "--store",
        default=None,
        help="Path to the place store JSON. Default: ~/.ros/rover_places.json",
    )
