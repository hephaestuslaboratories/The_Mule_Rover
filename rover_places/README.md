# rover_places

Name locations on a SLAM map, then send the rover to them by name.
Adapted for the rover stack: slam_toolbox localization on a saved
posegraph via `rover_bringup_extended_slam` (NOT vanilla nav2_bringup/AMCL).

Keyboard-driven now; the voice layer later replaces only the input side.
`place_store.py` has no ROS dependencies precisely so the speech layer (or
the PyQt GUI) can import it directly.

## Install (inside the container)

```bash
# package lives at /workspaces/isaac_ros-dev/src/rover_places
cd /workspaces/isaac_ros-dev
colcon build --packages-select rover_places --symlink-install
source install/setup.bash
```

`--symlink-install` means Python edits take effect without rebuilding.

## Storage & map scoping (container-specific — IMPORTANT)

* Places are stored in **`/workspaces/isaac_ros-dev/rover_places.json`**
  (bind-mounted → survives container restarts). The upstream default
  `~/.ros/` is ephemeral in the container and is only used as a fallback
  outside it. Override with `--store` or the `ROVER_PLACES_STORE` env var.
* Places are **scoped by map name** via the `ROVER_MAP` env var (or
  `--map`). `rover_startup.sh` exports `ROVER_MAP` in every worker
  terminal (edit `ROVER_MAP_NAME` at the top of that script when you save
  a new map). Poses are only valid against the map they were recorded on —
  if you ever re-map from scratch, re-tag your places.

## The workflow, in order

### 1. Launch nav mode against the saved posegraph

```bash
ros2 launch rover_bringup_extended_slam rover_bringup.launch.py \
  mode:=nav localization:=slam_toolbox \
  posegraph:=/workspaces/isaac_ros-dev/slm_maps/sls_coc_com_pas2 \
  use_rviz:=true
```

Wait until localization has converged (scan hugs the walls in RViz).

### 2. Tag locations

Get the rover to a spot (RViz "Nav2 Goal", or teleop), then in a worker
terminal:

```bash
ros2 run rover_places save kitchen
ros2 run rover_places save room1 --note "by the window"
ros2 run rover_places save dock --force        # overwrite existing
```

It reads the `map -> base_link` transform, so the saved pose is wherever
the robot physically is, **including its heading**. Park it facing the
direction you want it to end up facing.

### 3. Use them

```bash
ros2 run rover_places list
ros2 run rover_places goto kitchen
ros2 run rover_places goto --pick                  # numbered menu
ros2 run rover_places goto room1 kitchen dock      # ordered tour
```

`goto` with several names uses Nav2's `NavigateThroughPoses`, so it plans
the whole route rather than treating each leg independently.

Ctrl-C cancels the active goal properly instead of leaving Nav2 driving.

### Housekeeping

```bash
ros2 run rover_places rename room1 living_room
ros2 run rover_places delete dock
ros2 run rover_places list --all-maps
```

The JSON store is hand-editable when a pose is a little off, and worth
committing to git.

## Name resolution

Lookup is deliberately forgiving: exact match, then unique prefix, then
unique substring. `goto kit` finds `kitchen`. When a query matches several
names it refuses rather than guessing, and tells you the candidates.

That behaviour exists for the voice layer. Speech recognition will hand
you "room one" and "the kitchen", and an exact-match-only lookup would
reject both.

## Troubleshooting

**"Timed out waiting for transform map -> base_link"**
slam_toolbox localization is not up (or not yet localized). Check with
`ros2 run tf2_tools view_frames` and confirm `map` is the root of the tree.

**"Goal REJECTED by Nav2"**
Usually the goal sits in an obstacle or in unknown space on the costmap,
or the robot is not localized. Check the pose in RViz first.

**"navigate_to_pose server not available"**
Nav2 is running but not activated. `ros2 action list | grep navigate`
should show BOTH `navigate_to_pose` and `navigate_through_poses`; if not,
check the lifecycle manager.

**Saved poses are all wrong after a remap**
Expected — poses live in the `map` frame of the specific posegraph they
were saved against. Re-tag against the new map (under its new `ROVER_MAP`
name), or edit the JSON directly if the offset is a known constant.

## Where this goes next

The voice layer is a thin wrapper over the same store:

1. Wake word ("rover") -> openWakeWord.
2. Whisper transcribes the utterance.
3. Cheap intent match first: fuzzy-match the transcript against
   `store.list()` names plus a small verb list. "go to room1" resolves
   with no model call.
4. Only on no-match, fall through to an LLM for phrasing you did not
   anticipate.
5. Both paths end at the same `Navigator.go_to(place)` call used here.

Keep the wake word listener always running, including during navigation,
so "rover, stop" works as a voice e-stop. That is the single most
important command in the system.

A GUI dropdown is the same shape: import `rover_places.place_store`
directly from the PyQt launcher (no rclpy needed for listing), and shell
out to `ros2 run rover_places goto <name>` — or embed a `Navigator` — for
dispatch.
