"""Persistent named-location store for the rover.

Deliberately has NO ROS dependencies so that it can be imported by the
future voice/LLM layer, by unit tests, or by a web UI without pulling in
rclpy.  Everything ROS-specific lives in the CLI nodes.

Storage format is JSON rather than SQLite on purpose: it is human
readable, hand-editable when a pose is slightly wrong, and diffs cleanly
in git.  At the scale of "rooms in a building" the performance
difference is irrelevant.

IMPORTANT -- poses are stored in the ``map`` frame, which means they are
only meaningful for the specific map they were recorded against.  Re-run
SLAM from scratch and the origin moves; every saved pose silently
becomes wrong.  That is why every place records the map it belongs to
and why lookups are scoped by map name.
"""

from __future__ import annotations

import json
import math
import os
import tempfile
import time
from dataclasses import dataclass, asdict, field
from typing import Dict, List, Optional


# The container's home directory is EPHEMERAL -- only /workspaces/isaac_ros-dev
# is bind-mounted to disk on the Jetson.  The store must live there or every
# saved place is wiped on container restart.  Override with the
# ROVER_PLACES_STORE env var (or --store) when running outside the container.
_CONTAINER_WS = "/workspaces/isaac_ros-dev"
DEFAULT_STORE_PATH = os.environ.get(
    "ROVER_PLACES_STORE",
    os.path.join(_CONTAINER_WS, "rover_places.json")
    if os.path.isdir(_CONTAINER_WS)
    else os.path.expanduser("~/.ros/rover_places.json"),
)

# Schema version, bumped if the on-disk format ever changes so that a
# future loader can migrate rather than crash.
SCHEMA_VERSION = 1


@dataclass
class Place:
    """A single named pose in the map frame."""

    name: str
    map_name: str
    x: float
    y: float
    # Planar robot, so only the z/w components of the quaternion are
    # ever non-zero.  Stored as quaternion rather than yaw to avoid a
    # lossy round trip when handing back to Nav2.
    qz: float
    qw: float
    created_at: float = field(default_factory=time.time)
    note: str = ""

    @property
    def yaw(self) -> float:
        """Heading in radians, for display purposes."""
        return 2.0 * math.atan2(self.qz, self.qw)

    @property
    def yaw_degrees(self) -> float:
        return math.degrees(self.yaw)

    def distance_to(self, x: float, y: float) -> float:
        return math.hypot(self.x - x, self.y - y)

    def describe(self) -> str:
        return (
            f"{self.name:<16} x={self.x:>7.2f}  y={self.y:>7.2f}  "
            f"yaw={self.yaw_degrees:>7.1f}deg  map={self.map_name}"
            + (f"  # {self.note}" if self.note else "")
        )


class PlaceStore:
    """Load/save/query named places, scoped by map name."""

    def __init__(self, path: str = DEFAULT_STORE_PATH):
        self.path = os.path.expanduser(path)
        self._places: Dict[str, Place] = {}
        self.load()

    # ---------------------------------------------------------------- io

    def load(self) -> None:
        self._places = {}
        if not os.path.exists(self.path):
            return
        try:
            with open(self.path, "r") as fh:
                raw = json.load(fh)
        except (json.JSONDecodeError, OSError) as exc:
            raise RuntimeError(
                f"Could not read place store at {self.path}: {exc}. "
                "Fix or delete the file and try again."
            ) from exc

        for entry in raw.get("places", []):
            place = Place(**entry)
            self._places[self._key(place.name, place.map_name)] = place

    def save(self) -> None:
        """Write atomically so a crash mid-write cannot corrupt the store."""
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        payload = {
            "schema_version": SCHEMA_VERSION,
            "places": [asdict(p) for p in self._places.values()],
        }
        directory = os.path.dirname(self.path)
        fd, tmp_path = tempfile.mkstemp(dir=directory, suffix=".tmp")
        try:
            with os.fdopen(fd, "w") as fh:
                json.dump(payload, fh, indent=2, sort_keys=True)
                fh.flush()
                os.fsync(fh.fileno())
            os.replace(tmp_path, self.path)
        except BaseException:
            if os.path.exists(tmp_path):
                os.unlink(tmp_path)
            raise

    # ------------------------------------------------------------- query

    @staticmethod
    def _key(name: str, map_name: str) -> str:
        return f"{map_name}::{name.strip().lower()}"

    def get(self, name: str, map_name: str) -> Optional[Place]:
        return self._places.get(self._key(name, map_name))

    def list(self, map_name: Optional[str] = None) -> List[Place]:
        places = list(self._places.values())
        if map_name is not None:
            places = [p for p in places if p.map_name == map_name]
        return sorted(places, key=lambda p: p.name.lower())

    def maps(self) -> List[str]:
        return sorted({p.map_name for p in self._places.values()})

    def resolve(self, query: str, map_name: str) -> Optional[Place]:
        """Forgiving lookup: exact match, then prefix, then substring.

        This is what makes the eventual voice layer tolerable -- speech
        recognition will hand you "room one" or "the kitchen" and you do
        not want an exact-match-only lookup rejecting it.  Returns None
        when the match is ambiguous rather than guessing.
        """
        query = query.strip().lower()
        exact = self.get(query, map_name)
        if exact is not None:
            return exact

        candidates = self.list(map_name)
        prefix = [p for p in candidates if p.name.lower().startswith(query)]
        if len(prefix) == 1:
            return prefix[0]

        substring = [p for p in candidates if query in p.name.lower()]
        if len(substring) == 1:
            return substring[0]

        return None

    def ambiguous_matches(self, query: str, map_name: str) -> List[Place]:
        """Names that partially match, for a useful error message."""
        query = query.strip().lower()
        return [p for p in self.list(map_name) if query in p.name.lower()]

    # ------------------------------------------------------------ mutate

    def put(self, place: Place, overwrite: bool = False) -> None:
        key = self._key(place.name, place.map_name)
        if key in self._places and not overwrite:
            raise KeyError(
                f"'{place.name}' already exists on map '{place.map_name}'. "
                "Pass overwrite=True to replace it."
            )
        self._places[key] = place
        self.save()

    def delete(self, name: str, map_name: str) -> bool:
        key = self._key(name, map_name)
        if key not in self._places:
            return False
        del self._places[key]
        self.save()
        return True

    def rename(self, old: str, new: str, map_name: str) -> bool:
        place = self.get(old, map_name)
        if place is None:
            return False
        self.delete(old, map_name)
        place.name = new
        self.put(place, overwrite=True)
        return True
