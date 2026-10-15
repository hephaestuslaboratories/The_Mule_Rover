"""Save the rover's current pose under a name.

    ros2 run rover_places save room1
    ros2 run rover_places save kitchen --note "by the fridge"
    ros2 run rover_places save room1 --force        # overwrite
"""

from __future__ import annotations

import argparse
import sys

import rclpy

from rover_places.place_store import Place, PlaceStore, DEFAULT_STORE_PATH
from rover_places.ros_common import PoseReader, add_common_args


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        prog="save_place",
        description="Tag the rover's current position with a name.",
    )
    parser.add_argument("name", help="Label for this spot, e.g. room1")
    parser.add_argument("--note", default="", help="Optional free-text note")
    parser.add_argument(
        "--force", action="store_true", help="Overwrite an existing name"
    )
    parser.add_argument(
        "--base-frame",
        default="base_link",
        help="Robot base frame (default: base_link)",
    )
    add_common_args(parser)
    args = parser.parse_args(argv)

    store = PlaceStore(args.store or DEFAULT_STORE_PATH)

    existing = store.get(args.name, args.map)
    if existing is not None and not args.force:
        print(
            f"'{args.name}' already exists on map '{args.map}':\n"
            f"  {existing.describe()}\n"
            "Re-run with --force to overwrite it.",
            file=sys.stderr,
        )
        return 1

    rclpy.init()
    node = PoseReader(base_frame=args.base_frame)
    try:
        pose = node.current_pose()
        if pose is None:
            return 2

        x, y, qz, qw = pose
        place = Place(
            name=args.name,
            map_name=args.map,
            x=x,
            y=y,
            qz=qz,
            qw=qw,
            note=args.note,
        )
        store.put(place, overwrite=True)
        print(f"Saved: {place.describe()}")

        nearby = [
            p
            for p in store.list(args.map)
            if p.name != place.name and p.distance_to(x, y) < 0.5
        ]
        if nearby:
            names = ", ".join(p.name for p in nearby)
            print(
                f"Note: within 0.5 m of already-saved place(s): {names}. "
                "Nav2 may treat these as effectively the same goal."
            )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
