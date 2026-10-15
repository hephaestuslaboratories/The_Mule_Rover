"""List, delete, and rename saved places.  No ROS runtime needed.

    ros2 run rover_places list
    ros2 run rover_places list --all-maps
    ros2 run rover_places delete room1
    ros2 run rover_places rename room1 workshop
"""

from __future__ import annotations

import argparse
import sys

from rover_places.place_store import PlaceStore, DEFAULT_STORE_PATH
from rover_places.ros_common import add_common_args


def list_main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        prog="list_places", description="Show saved places."
    )
    parser.add_argument(
        "--all-maps", action="store_true", help="Show places from every map"
    )
    add_common_args(parser)
    args = parser.parse_args(argv)

    store = PlaceStore(args.store or DEFAULT_STORE_PATH)
    places = store.list(None if args.all_maps else args.map)

    if not places:
        scope = "any map" if args.all_maps else f"map '{args.map}'"
        print(f"No places saved for {scope}.")
        known = store.maps()
        if known and not args.all_maps:
            print(f"Maps that do have places: {', '.join(known)}")
            print("Use --map <name> or --all-maps.")
        return 0

    print(f"{len(places)} place(s):\n")
    for place in places:
        print("  " + place.describe())
    print()
    return 0


def delete_main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        prog="delete_place", description="Remove a saved place."
    )
    parser.add_argument("name")
    parser.add_argument(
        "-y", "--yes", action="store_true", help="Skip confirmation"
    )
    add_common_args(parser)
    args = parser.parse_args(argv)

    store = PlaceStore(args.store or DEFAULT_STORE_PATH)
    place = store.get(args.name, args.map)
    if place is None:
        print(
            f"No place named '{args.name}' on map '{args.map}'.",
            file=sys.stderr,
        )
        return 1

    if not args.yes:
        print(f"About to delete: {place.describe()}")
        if input("Confirm [y/N]: ").strip().lower() not in ("y", "yes"):
            print("Cancelled.")
            return 1

    store.delete(args.name, args.map)
    print(f"Deleted '{args.name}'.")
    return 0


def rename_main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        prog="rename_place", description="Rename a saved place."
    )
    parser.add_argument("old")
    parser.add_argument("new")
    add_common_args(parser)
    args = parser.parse_args(argv)

    store = PlaceStore(args.store or DEFAULT_STORE_PATH)
    if store.rename(args.old, args.new, args.map):
        print(f"Renamed '{args.old}' -> '{args.new}'.")
        return 0
    print(f"No place named '{args.old}' on map '{args.map}'.", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(list_main())
