"""Send the rover to a saved place, or on a tour of several.

    ros2 run rover_places goto room1
    ros2 run rover_places goto room1 room2 kitchen     # ordered tour
    ros2 run rover_places goto --pick                  # menu

Ctrl-C cancels the goal cleanly rather than leaving Nav2 driving.
"""

from __future__ import annotations

import argparse
import sys
from typing import List, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose, NavigateThroughPoses

from rover_places.place_store import Place, PlaceStore, DEFAULT_STORE_PATH
from rover_places.ros_common import add_common_args, make_pose_stamped


class Navigator(Node):
    def __init__(self):
        super().__init__("rover_places_navigator")
        self._single = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._multi = ActionClient(
            self, NavigateThroughPoses, "navigate_through_poses"
        )
        self._goal_handle = None
        self._result = None
        self._done = False
        self._last_feedback_print = -1.0

    # ------------------------------------------------------------ helpers

    def _wait_for_server(self, client: ActionClient, name: str) -> bool:
        self.get_logger().info(f"Waiting for the {name} action server...")
        if not client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                f"{name} server not available after 10 s. Is Nav2 running "
                "and fully activated? Check: ros2 action list | grep navigate"
            )
            return False
        return True

    def _feedback(self, msg) -> None:
        fb = msg.feedback
        remaining = getattr(fb, "distance_remaining", None)
        if remaining is None:
            return
        # Only print on meaningful change, to avoid spamming the console.
        if (
            self._last_feedback_print < 0
            or abs(remaining - self._last_feedback_print) > 0.25
        ):
            self._last_feedback_print = remaining
            print(f"  {remaining:5.2f} m remaining", end="\r", flush=True)

    def _goal_response(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error(
                "Goal REJECTED by Nav2. Common causes: the goal is inside "
                "an obstacle or unknown space on the costmap, or the robot "
                "is not localized."
            )
            self._done = True
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future) -> None:
        self._result = future.result()
        self._done = True

    def _run(self, client: ActionClient, goal_msg) -> bool:
        send_future = client.send_goal_async(
            goal_msg, feedback_callback=self._feedback
        )
        send_future.add_done_callback(self._goal_response)

        try:
            while rclpy.ok() and not self._done:
                rclpy.spin_once(self, timeout_sec=0.1)
        except KeyboardInterrupt:
            print()
            self.get_logger().warn("Interrupted -- cancelling goal...")
            if self._goal_handle is not None:
                cancel = self._goal_handle.cancel_goal_async()
                while rclpy.ok() and not cancel.done():
                    rclpy.spin_once(self, timeout_sec=0.1)
            self.get_logger().warn("Goal cancelled. Rover should be stopped.")
            return False

        print()
        if self._result is None:
            return False

        # status 4 == SUCCEEDED in action_msgs/GoalStatus
        succeeded = self._result.status == 4
        if succeeded:
            self.get_logger().info("Arrived.")
        else:
            self.get_logger().error(
                f"Navigation did not succeed (status {self._result.status}). "
                "Nav2's recovery behaviours may have exhausted themselves."
            )
        return succeeded

    # -------------------------------------------------------------- public

    def go_to(self, place: Place) -> bool:
        if not self._wait_for_server(self._single, "navigate_to_pose"):
            return False
        goal = NavigateToPose.Goal()
        goal.pose = make_pose_stamped(self, place.x, place.y, place.qz, place.qw)
        self.get_logger().info(
            f"Navigating to '{place.name}' "
            f"(x={place.x:.2f}, y={place.y:.2f})"
        )
        return self._run(self._single, goal)

    def tour(self, places: List[Place]) -> bool:
        if not self._wait_for_server(self._multi, "navigate_through_poses"):
            return False
        goal = NavigateThroughPoses.Goal()
        goal.poses = [
            make_pose_stamped(self, p.x, p.y, p.qz, p.qw) for p in places
        ]
        names = " -> ".join(p.name for p in places)
        self.get_logger().info(f"Touring: {names}")
        return self._run(self._multi, goal)


def _pick_interactively(store: PlaceStore, map_name: str) -> Optional[Place]:
    places = store.list(map_name)
    if not places:
        print(f"No places saved for map '{map_name}'.", file=sys.stderr)
        return None
    print(f"\nSaved places on map '{map_name}':\n")
    for i, place in enumerate(places, start=1):
        print(f"  [{i:>2}] {place.describe()}")
    print()
    try:
        choice = input("Go to which? (number, or blank to cancel): ").strip()
    except (EOFError, KeyboardInterrupt):
        print()
        return None
    if not choice:
        return None
    try:
        index = int(choice)
        if not 1 <= index <= len(places):
            raise ValueError
    except ValueError:
        print("Not a valid selection.", file=sys.stderr)
        return None
    return places[index - 1]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        prog="goto_place",
        description="Drive the rover to one or more saved places.",
    )
    parser.add_argument(
        "names",
        nargs="*",
        help="One name, or several for an ordered tour",
    )
    parser.add_argument(
        "--pick",
        action="store_true",
        help="Choose from a numbered menu instead of typing a name",
    )
    add_common_args(parser)
    args = parser.parse_args(argv)

    store = PlaceStore(args.store or DEFAULT_STORE_PATH)

    targets: List[Place] = []
    if args.pick or not args.names:
        chosen = _pick_interactively(store, args.map)
        if chosen is None:
            return 1
        targets = [chosen]
    else:
        for name in args.names:
            place = store.resolve(name, args.map)
            if place is None:
                matches = store.ambiguous_matches(name, args.map)
                if matches:
                    options = ", ".join(p.name for p in matches)
                    print(
                        f"'{name}' is ambiguous on map '{args.map}'. "
                        f"Did you mean: {options}?",
                        file=sys.stderr,
                    )
                else:
                    known = ", ".join(p.name for p in store.list(args.map))
                    print(
                        f"No place matching '{name}' on map '{args.map}'. "
                        f"Known: {known or '(none)'}",
                        file=sys.stderr,
                    )
                return 1
            targets.append(place)

    rclpy.init()
    node = Navigator()
    try:
        if len(targets) == 1:
            ok = node.go_to(targets[0])
        else:
            ok = node.tour(targets)
        return 0 if ok else 3
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
