"""Launch-process lifecycle for the rover web cockpit.

Owns the actual `ros2 launch` / `ros2 run` subprocesses behind the
phone's mode buttons.  State lives HERE (server side), so a phone that
drops off WiFi changes nothing: explore keeps exploring, Nav2 keeps
driving.  When the phone reconnects it just reads this state back.

Zombie hygiene (hard-learned campaign rules, enforced in code):
  * every subprocess starts in its own session (process group)
  * stop = SIGINT the whole group -> wait -> SIGTERM -> wait -> SIGKILL
  * after stopping, known node name patterns are pkill -9'd if any
    survived, and the ros2 daemon is restarted so no stale discovery
    state can answer `param get` with lies
"""

from __future__ import annotations

import glob
import json
import os
import re
import signal
import subprocess
import threading
import time
from collections import deque
from typing import Callable, Dict, List, Optional, Tuple

WORKSPACE = "/workspaces/isaac_ros-dev"
MAPS_DIR = os.path.join(WORKSPACE, "slm_maps")
SETTINGS_FILE = os.path.join(WORKSPACE, "rover_web_settings.json")
RUN_LOG_FILE = os.path.join(WORKSPACE, "last_run_log.txt")
NAV2_PARAMS = os.path.join(
    WORKSPACE, "src", "rover_bringup_extended_slam",
    "config", "nav2_params.yaml")

# Process-name patterns that must be dead after a stop.  These are
# matched with pkill -9 -f as a LAST resort (the polite kill ladder
# usually gets everything).  Deliberately excludes anything that could
# match this web server's own cmdline.
STRAGGLER_PATTERNS = [
    "rover_bringup.launch.py",
    "navigation_launch.py",
    "slam_toolbox",
    "bt_navigator",
    "controller_server",
    "planner_server",
    "behavior_server",
    "smoother_server",
    "velocity_smoother",
    "waypoint_follower",
    "lifecycle_manager",
    "explore_lite",
    "rplidar",
    "arduino_encoder_twist_bridge",
    "scan_deskew",
    "zupt_node",
    "lidar_scan_filter",
    "wifi_watchdog",
    "robot_state_publisher",
    "ekf_node",
    "cmd_vel_safety_filter",
    "pose_tracker",
    "pose_initializer",
    "uptime_monitor",
    # Must die on stop or the next run cannot open the bus: a surviving
    # tof_belt_node holds /dev/i2c-* AND leaves a mux channel selected, which
    # is the state that jams the belt until the mux is power-cycled.
    "tof_belt_node",
]


class _Proc:
    """One managed subprocess with a log-tail reader thread."""

    def __init__(self, name: str, cmd: List[str],
                 log_cb: Optional[Callable[[str], None]]):
        self.name = name
        self.cmd = cmd
        self.popen = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            text=True,
            bufsize=1,
        )
        self._log_cb = log_cb
        self._reader = threading.Thread(
            target=self._read, daemon=True, name=f"log-{name}")
        self._reader.start()

    def _read(self) -> None:
        try:
            for line in self.popen.stdout:  # type: ignore[union-attr]
                line = line.rstrip("\n")
                if line and self._log_cb is not None:
                    self._log_cb(f"[{self.name}] {line}")
        except Exception:
            pass

    def alive(self) -> bool:
        return self.popen.poll() is None

    def stop(self, log: Callable[[str], None]) -> None:
        if not self.alive():
            return
        pgid = None
        try:
            pgid = os.getpgid(self.popen.pid)
        except Exception:
            pass

        def _signal_group(sig) -> None:
            try:
                if pgid is not None:
                    os.killpg(pgid, sig)
                else:
                    self.popen.send_signal(sig)
            except ProcessLookupError:
                pass

        log(f"stopping {self.name} (SIGINT)...")
        _signal_group(signal.SIGINT)
        if self._wait(12.0):
            log(f"{self.name} exited cleanly")
            return
        log(f"{self.name} ignoring SIGINT -> SIGTERM")
        _signal_group(signal.SIGTERM)
        if self._wait(4.0):
            return
        log(f"{self.name} ignoring SIGTERM -> SIGKILL")
        _signal_group(signal.SIGKILL)
        self._wait(3.0)

    def _wait(self, timeout: float) -> bool:
        try:
            self.popen.wait(timeout=timeout)
            return True
        except subprocess.TimeoutExpired:
            return False


class LaunchManager:
    """Modes: idle | slam | nav.  Explore is a sub-state of slam."""

    def __init__(self):
        self._lock = threading.RLock()
        self.mode = "idle"
        self.current_map: Optional[str] = None   # scopes rover_places names
        self.explore_running = False
        self._procs: Dict[str, _Proc] = {}
        self.log_lines: deque = deque(maxlen=400)
        self.log_cb: Optional[Callable[[str], None]] = None
        self.busy = False
        # Operator settings, persisted across server restarts and reboots.
        settings = self._load_settings()
        self.pose_autoinit = bool(settings.get("pose_autoinit", True))
        self.run_logger = bool(settings.get("run_logger", True))
        self.log_panel = bool(settings.get("log_panel", False))
        # Run logger: one file, truncated at each START, complete at STOP.
        self._run_log = None
        self._run_log_lock = threading.Lock()
        self._run_started = 0.0

    # ------------------------------------------------------------- settings

    SETTING_NAMES = ("pose_autoinit", "run_logger", "log_panel")

    @staticmethod
    def _load_settings() -> Dict:
        try:
            with open(SETTINGS_FILE, encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            return {}

    def set_setting(self, name: str, on: bool) -> bool:
        if name not in self.SETTING_NAMES:
            return False
        with self._lock:
            setattr(self, name, bool(on))
            settings = self._load_settings()
            settings[name] = bool(on)
            try:
                tmp = SETTINGS_FILE + ".tmp"
                with open(tmp, "w", encoding="utf-8") as f:
                    json.dump(settings, f)
                os.replace(tmp, SETTINGS_FILE)
            except Exception as exc:  # noqa: BLE001
                self._log(f"settings save failed: {exc}")
        return True

    def set_pose_autoinit(self, on: bool) -> None:
        self.set_setting("pose_autoinit", on)

    # ------------------------------------------------------------- run log

    def _open_run_log(self, mode: str, map_name: str) -> None:
        """Truncate last_run_log.txt and start streaming this run into it."""
        if not self.run_logger:
            return
        with self._run_log_lock:
            try:
                self._run_log = open(RUN_LOG_FILE, "w", encoding="utf-8")
                self._run_started = time.time()
                stamp = time.strftime("%Y-%m-%d %H:%M:%S")
                self._run_log.write(
                    f"==== RUN START {stamp} mode={mode} map={map_name} ====\n")
                self._run_log.flush()
            except Exception:  # noqa: BLE001
                self._run_log = None

    def _close_run_log(self) -> None:
        with self._run_log_lock:
            if self._run_log is None:
                return
            try:
                stamp = time.strftime("%Y-%m-%d %H:%M:%S")
                dur = int(time.time() - self._run_started)
                self._run_log.write(
                    f"==== RUN END {stamp} duration={dur}s ====\n")
                self._run_log.close()
            except Exception:  # noqa: BLE001
                pass
            self._run_log = None

    # ------------------------------------------------------------- logging

    def _log(self, text: str) -> None:
        self.log_lines.append(text)
        with self._run_log_lock:
            if self._run_log is not None:
                try:
                    self._run_log.write(
                        time.strftime("%H:%M:%S ") + text + "\n")
                    self._run_log.flush()
                except Exception:  # noqa: BLE001
                    self._run_log = None
        cb = self.log_cb
        if cb is not None:
            try:
                cb(text)
            except Exception:
                pass

    # ------------------------------------------------------------- catalog

    @staticmethod
    def list_maps() -> List[str]:
        names = []
        for p in sorted(glob.glob(os.path.join(MAPS_DIR, "*.posegraph"))):
            base = os.path.splitext(os.path.basename(p))[0]
            if os.path.isfile(os.path.join(MAPS_DIR, base + ".data")):
                names.append(base)
        return names

    @staticmethod
    def valid_name(name: str) -> bool:
        return bool(re.fullmatch(r"[A-Za-z0-9_\-]{1,40}", name))

    # ------------------------------------------------------------- spawn

    def _spawn(self, name: str, cmd: List[str]) -> None:
        self._log(f"starting {name}: {' '.join(cmd)}")
        self._procs[name] = _Proc(name, cmd, self._log)

    # ------------------------------------------------------------- modes

    def start_slam(self, map_name: str) -> Tuple[bool, str]:
        with self._lock:
            if not self.valid_name(map_name):
                return False, "Map name: letters/digits/_/- only (max 40)"
            if self.mode != "idle":
                self.stop_all()
            self._open_run_log("slam", map_name)
            self._spawn("slam", [
                "ros2", "launch", "rover_bringup_extended_slam",
                "rover_bringup.launch.py",
                "mode:=slam", "use_rviz:=false",
            ])
            self.mode = "slam"
            self.current_map = map_name  # the name this session will save as
            return True, f"SLAM started — will save as '{map_name}'"

    def start_nav(self, map_name: str) -> Tuple[bool, str]:
        with self._lock:
            pose_path = os.path.join(MAPS_DIR, map_name)
            if not (os.path.isfile(pose_path + ".posegraph")
                    and os.path.isfile(pose_path + ".data")):
                return False, f"Map '{map_name}' not found in slm_maps"
            if self.mode != "idle":
                self.stop_all()
            self._open_run_log("nav", map_name)
            autoinit = "true" if self.pose_autoinit else "false"
            self._spawn("nav", [
                "ros2", "launch", "rover_bringup_extended_slam",
                "rover_bringup.launch.py",
                "mode:=nav", "localization:=slam_toolbox",
                f"posegraph:={pose_path}", "use_rviz:=false",
                f"use_pose_autoinit:={autoinit}",
            ])
            self.mode = "nav"
            self.current_map = map_name
            return True, f"NAV started on map '{map_name}'"

    # ------------------------------------------------------------- explore

    def _nav2_action_ready(self) -> bool:
        try:
            res = subprocess.run(["ros2", "action", "list"],
                                 capture_output=True, text=True, timeout=15)
        except subprocess.TimeoutExpired:
            return False
        return "navigate_to_pose" in (res.stdout or "")

    def start_explore(self) -> Tuple[bool, str]:
        """Bring up Nav2, wait until it can actually accept goals, then run
        explore_lite on top of the live SLAM map.

        Three things here are load-bearing and were each a bug once:
          1. `-r __node:=explore` — the ROS 2 port of explore_lite names its
             node `explore_node` by default.  The tuning block in
             nav2_params.yaml is keyed `explore:`, so WITHOUT this remap the
             file silently does not apply, costmap_topic falls back to the
             default "costmap", and the node waits forever for a topic nobody
             publishes.  Symptom: "exploring" but the rover never moves.
          2. Sequencing — Nav2's lifecycle bringup takes 10-25 s.  Starting
             explore_lite in the same breath means it looks for an action
             server that does not exist yet.
          3. Liveness — verify the process is still alive before telling the
             UI exploration is running, instead of assuming spawn == success.
        """
        with self._lock:
            if self.mode != "slam":
                return False, "Explore only runs in SLAM mode"
            if self.explore_running:
                return False, "Explore already running"
            if not os.path.isfile(NAV2_PARAMS):
                return False, f"nav2_params.yaml not found at {NAV2_PARAMS}"
            probe = subprocess.run(["ros2", "pkg", "prefix", "explore_lite"],
                                   capture_output=True, text=True)
            if probe.returncode != 0:
                return False, ("explore_lite is not installed — check "
                               "`ros2 pkg list | grep explore`")
            self._spawn("explore_nav2", [
                "ros2", "launch", "nav2_bringup", "navigation_launch.py",
                "use_sim_time:=false", "autostart:=true",
                "use_composition:=False",
                f"params_file:={NAV2_PARAMS}",
            ])

        # Poll OUTSIDE the lock so /status keeps answering while Nav2 boots.
        self._log("explore: waiting for Nav2 to accept goals (up to 60 s)...")
        deadline = time.time() + 60.0
        while time.time() < deadline:
            if self._nav2_action_ready():
                break
            time.sleep(2.0)
        else:
            self._log("explore: Nav2 never advertised navigate_to_pose")
            with self._lock:
                proc = self._procs.pop("explore_nav2", None)
            if proc is not None:
                proc.stop(self._log)
            return False, "Nav2 did not come up — see system log"

        self._log("explore: Nav2 ready, starting frontier search")
        with self._lock:
            self._spawn("explore", [
                "ros2", "run", "explore_lite", "explore",
                "--ros-args", "-r", "__node:=explore",
                "--params-file", NAV2_PARAMS,
            ])

        time.sleep(3.0)   # long enough for an import/param error to surface
        with self._lock:
            proc = self._procs.get("explore")
            if proc is None or not proc.alive():
                self._procs.pop("explore", None)
                dead = self._procs.pop("explore_nav2", None)
                if dead is not None:
                    dead.stop(self._log)
                return False, "explore_lite exited immediately — see system log"
            self.explore_running = True
        return True, "Frontier exploration started"

    def stop_explore(self) -> Tuple[bool, str]:
        with self._lock:
            if not self.explore_running:
                return False, "Explore is not running"
            for key in ("explore", "explore_nav2"):
                proc = self._procs.pop(key, None)
                if proc is not None:
                    proc.stop(self._log)
            self.explore_running = False
            return True, "Exploration stopped (SLAM still mapping)"

    # ------------------------------------------------------------- stop all

    def stop_all(self) -> Tuple[bool, str]:
        with self._lock:
            # Children (explore) first, main launch last.
            for key in ("explore", "explore_nav2", "slam", "nav"):
                proc = self._procs.pop(key, None)
                if proc is not None:
                    proc.stop(self._log)
            self.explore_running = False
            self.mode = "idle"
            self._cleanup_stragglers()
            self._close_run_log()
            return True, "Stack stopped"

    def _cleanup_stragglers(self) -> None:
        survivors = []
        for pat in STRAGGLER_PATTERNS:
            res = subprocess.run(
                ["pgrep", "-f", pat], capture_output=True, text=True)
            if res.returncode == 0:
                survivors.append(pat)
                subprocess.run(["pkill", "-9", "-f", pat],
                               capture_output=True)
        if survivors:
            self._log(f"killed stragglers: {', '.join(survivors)}")
            time.sleep(1.0)
        # Stale-discovery hygiene: never let a zombie answer param queries.
        subprocess.run(["ros2", "daemon", "stop"], capture_output=True)
        subprocess.run(["ros2", "daemon", "start"], capture_output=True)
        self._log("ros2 daemon restarted")

    # ------------------------------------------------------------- save map

    def save_map_grid(self, map_name: str) -> Tuple[bool, str]:
        """Run map_saver_cli for the occupancy-grid pair (yaml+pgm)."""
        path = os.path.join(MAPS_DIR, map_name)
        try:
            res = subprocess.run(
                ["ros2", "run", "nav2_map_server", "map_saver_cli",
                 "-f", path],
                capture_output=True, text=True, timeout=30)
        except subprocess.TimeoutExpired:
            return False, "map_saver_cli timed out"
        if os.path.isfile(path + ".yaml"):
            return True, "grid map saved"
        tail = (res.stdout or "")[-300:]
        return False, f"map_saver_cli failed: {tail}"

    # ------------------------------------------------------------- status

    def status(self) -> Dict:
        with self._lock:
            # Keep the flag honest: if explore_lite died on its own, the UI
            # must stop claiming the rover is exploring.  Logged once, because
            # the flag flips false on the same pass.
            if self.explore_running:
                proc = self._procs.get("explore")
                if proc is not None and not proc.alive():
                    self.explore_running = False
                    self._log("explore: explore_lite exited on its own")
            return {
                "mode": self.mode,
                "map_name": self.current_map,
                "explore": self.explore_running,
                "procs": {k: p.alive() for k, p in self._procs.items()},
                "busy": self.busy,
                "pose_autoinit": self.pose_autoinit,
                "run_logger": self.run_logger,
                "log_panel": self.log_panel,
            }
