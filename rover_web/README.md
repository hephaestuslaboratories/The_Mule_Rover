# rover_web — phone-browser cockpit for the rover

Control the rover from Safari on an iPhone (or any browser on the same
WiFi): live map view, mode switching (SLAM / NAV), teleop joystick,
frontier exploration, named-place save & go-to.

**Design rule:** all state lives on the Jetson. The phone is a stateless
remote — if it drops off WiFi mid-exploration or mid-goal, the rover
keeps doing what it was told; reopen the page and it shows exactly where
things are. The one exception is the teleop joystick: a 0.5 s deadman
stops the rover the moment input goes silent (release, page hidden,
WiFi drop). Manual driving never survives losing the pilot.

## Install (one time)

1. Copy this folder to `/workspaces/isaac_ros-dev/src/rover_web/`
   (next to `rover_bringup_extended_slam` and `rover_places`).
2. Inside the container, install the two web libraries into the
   persistent workspace (container home is ephemeral; this path is not):

```bash
pip3 install --target=/workspaces/isaac_ros-dev/pydeps fastapi "uvicorn[standard]"
```

3. Build (requires `rover_places` to be built too — places live there):

```bash
cd /workspaces/isaac_ros-dev
colcon build --packages-select rover_places rover_web --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 run rover_web server            # default port 8080
```

It prints the URL(s). On the phone (same WiFi as the Jetson):
`http://<jetson-ip>:8080`. Add to Home Screen in Safari for an
app-like fullscreen experience.

Do NOT launch the rover stack yourself — the server owns launch
lifecycle. Start `rover_web` from a plain sourced terminal and drive
everything from the phone.

## The modes

### SLAM (make a new map)
Type a map name → START SLAM. Then either:
* **Telemetry:** drag the joystick to drive and map by hand, or
* **Explore:** START EXPLORE — Nav2 + frontier exploration start on top
  of SLAM and the rover maps autonomously until no frontiers remain.

SAVE MAP serializes the posegraph + grid into `slm_maps/<name>` — the
map then appears in the NAV dropdown. STOP SLAM SESSION tears the
stack down (unsaved mapping is lost).

### NAV (use a saved map)
Pick a map → START NAV. Wait for LOC chip green. Then:
* **Save location:** drive/send the rover somewhere, type a name, SAVE.
  The box clears, a dot appears on the map. Places are scoped to the
  loaded map automatically (no ROVER_MAP export needed — the server
  handles scoping internally).
* **Go somewhere:** tap a saved place name. Progress banner shows
  distance remaining; CANCEL aborts the goal.

### VOICE
Placeholder tab — phase 3. Will need HTTPS for mic access.

### STOP button
Always visible. Cancels any Nav2 goal, halts exploration, zeroes the
joystick. It does NOT kill the launch (use the per-mode stop buttons
for that).

## Process hygiene (built in)

Mode switches stop the previous launch with a SIGINT→SIGTERM→SIGKILL
ladder on the whole process group, then `pkill -9` any known straggler
node patterns, then restart the `ros2 daemon` — so no zombie can answer
`param get` with stale values.

## Files

* `rover_web/server.py` — FastAPI + WebSocket protocol; snapshot on
  connect (reconnect resilience)
* `rover_web/ros_iface.py` — rclpy node: /map→PNG, TF pose, status,
  /cmd_vel with deadman, NavigateToPose client, serialize_map
* `rover_web/launch_manager.py` — subprocess lifecycle for mode
  switching + explore add-on + zombie cleanup
* `rover_web/static/` — the phone page (vanilla JS, no build step)

## Troubleshooting

* **Page loads but LINK chip red** — server died or phone left WiFi;
  page auto-reconnects when reachable.
* **"Missing web dependencies"** — run the pip3 --target install above.
* **Phone can't reach the URL** — confirm the container runs with host
  networking (default for run_dev.sh) and phone is on the same WiFi;
  try `curl http://localhost:8080` inside the container first.
* **Joystick does nothing** — joystick only works in SLAM mode while
  explore is NOT running (by design).
* **Save location refused** — only allowed in NAV mode: during SLAM the
  map origin can still shift (loop closures), which would silently
  corrupt saved poses.
