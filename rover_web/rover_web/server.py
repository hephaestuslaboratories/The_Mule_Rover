"""Rover web cockpit — FastAPI server + WebSocket protocol.

Run inside the container from a sourced terminal:

    ros2 run rover_web server            # default port 8080

Then open  http://<jetson-ip>:8080  in the phone's browser (same WiFi).

Design rule: ALL state lives server-side (LaunchManager + RosInterface).
The phone is a stateless remote.  A dropped WebSocket cancels nothing
autonomous — explore keeps exploring, Nav2 keeps driving.  On reconnect
the client receives a full snapshot and resumes where things actually
are.  The single exception is the teleop joystick, whose deadman stops
the rover when input goes silent (see ros_iface.py).

Third-party deps (fastapi/uvicorn) are vendored into the workspace so
they survive container restarts:

    pip3 install --target=/workspaces/isaac_ros-dev/pydeps \
        fastapi "uvicorn[standard]"
"""

# NOTE: no `from __future__ import annotations` here — PEP 563 string
# annotations break FastAPI's parameter inspection for routes defined
# inside create_app() (it can't resolve "WebSocket" from the closure and
# mis-classifies the socket argument as a required query parameter,
# rejecting every connection with 403). Hard-won lesson; do not re-add.
import argparse
import asyncio
import base64
import json
import os
import subprocess
import sys
import time
from typing import Dict, List, Optional, Set

PYDEPS = "/workspaces/isaac_ros-dev/pydeps"
if os.path.isdir(PYDEPS) and PYDEPS not in sys.path:
    sys.path.insert(0, PYDEPS)

from rover_web.launch_manager import MAPS_DIR, LaunchManager  # noqa: E402
from rover_web.ros_iface import RosInterface  # noqa: E402

try:
    from rover_places.place_store import Place, PlaceStore
except ImportError:  # pragma: no cover
    Place = PlaceStore = None  # type: ignore[assignment]

STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")


def _require_web_deps():
    try:
        import fastapi  # noqa: F401
        import uvicorn  # noqa: F401
    except ImportError:
        print(
            "\nMissing web dependencies (fastapi/uvicorn).\n"
            "Install ONCE into the persistent workspace (inside the "
            "container):\n\n"
            f"    pip3 install --target={PYDEPS} fastapi \"uvicorn[standard]\"\n",
            file=sys.stderr,
        )
        sys.exit(1)


def create_app(iface: RosInterface, manager: LaunchManager):
    from fastapi import FastAPI, WebSocket, WebSocketDisconnect
    from fastapi.responses import FileResponse
    from fastapi.staticfiles import StaticFiles

    app = FastAPI()
    app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")

    clients: Set[WebSocket] = set()
    log_queue: "asyncio.Queue[str]" = asyncio.Queue()
    mode_lock = asyncio.Lock()   # serializes start/stop/save operations

    # ---------------------------------------------------------- helpers

    def _store() -> Optional[PlaceStore]:
        if PlaceStore is None:
            return None
        return PlaceStore()  # constructor loads the JSON fresh

    def _places_payload() -> List[Dict]:
        store = _store()
        map_name = manager.current_map
        if store is None or not map_name:
            return []
        return [
            {"name": p.name, "x": round(p.x, 3), "y": round(p.y, 3),
             "yaw_deg": round(p.yaw_degrees, 1), "note": p.note}
            for p in store.list(map_name)
        ]

    def _status_payload() -> Dict:
        s = manager.status()
        s.update(iface.get_status())
        return s

    async def _send(ws, payload: Dict) -> None:
        await ws.send_text(json.dumps(payload))

    async def _broadcast(payload: Dict) -> None:
        text = json.dumps(payload)
        dead = []
        for ws in clients:
            try:
                await ws.send_text(text)
            except Exception:
                dead.append(ws)
        for ws in dead:
            clients.discard(ws)

    async def _push_map(ws=None) -> int:
        png, meta, seq = iface.get_map_png()
        if png is None:
            return 0
        payload = {"type": "map", "seq": seq, "meta": meta,
                   "png": base64.b64encode(png).decode("ascii")}
        if ws is not None:
            await _send(ws, payload)
        else:
            await _broadcast(payload)
        return seq

    async def _push_state(ws=None) -> None:
        payload = {"type": "state", "status": _status_payload(),
                   "maps": manager.list_maps(), "places": _places_payload()}
        if ws is not None:
            await _send(ws, payload)
        else:
            await _broadcast(payload)

    async def _toast(ws, level: str, text: str) -> None:
        await _send(ws, {"type": "toast", "level": level, "text": text})

    # ---------------------------------------------------------- lifecycle

    @app.on_event("startup")
    async def _startup() -> None:
        loop = asyncio.get_running_loop()

        def _log_cb(line: str) -> None:
            try:
                loop.call_soon_threadsafe(log_queue.put_nowait, line)
            except Exception:
                pass

        manager.log_cb = _log_cb
        asyncio.create_task(_broadcaster())

    async def _broadcaster() -> None:
        last_map_seq = -1
        last_autoinit = None
        tick = 0
        while True:
            try:
                pose = iface.get_pose()
                if pose is not None and clients:
                    await _broadcast({"type": "pose", **pose})
                # Auto pose-init result -> one toast per change, so the pilot
                # knows the pose is set (or that a manual SET POSE is needed).
                autoinit = iface.get_status().get("autoinit")
                if autoinit != last_autoinit:
                    last_autoinit = autoinit
                    if autoinit and clients:
                        if autoinit.startswith("SEEDED"):
                            await _broadcast({
                                "type": "toast", "level": "ok",
                                "text": ("Pose auto-initialized — verify the "
                                         "arrow, then send goals")})
                        elif not autoinit.startswith("WAITING"):
                            await _broadcast({
                                "type": "toast", "level": "warn",
                                "text": ("Auto pose-init skipped "
                                         f"({autoinit}) — use SET POSE")})
                if tick % 5 == 0 and clients:       # ~1 Hz
                    await _push_state()
                    png_seq = iface.get_map_png()[2]
                    if png_seq != last_map_seq:
                        last_map_seq = await _push_map()
                while not log_queue.empty():
                    line = log_queue.get_nowait()
                    if clients and manager.log_panel:
                        await _broadcast({"type": "log", "text": line})
            except Exception:
                pass
            tick += 1
            await asyncio.sleep(0.2)

    # ---------------------------------------------------------- routes

    @app.get("/")
    async def index():
        return FileResponse(os.path.join(STATIC_DIR, "index.html"))

    @app.get("/api/maps")
    async def api_maps():
        """Debug: what the map scan actually sees, straight from disk."""
        try:
            dir_exists = os.path.isdir(MAPS_DIR)
            listing = sorted(os.listdir(MAPS_DIR)) if dir_exists else []
        except Exception as exc:  # noqa: BLE001
            return {"error": str(exc)}
        return {
            "maps_dir": MAPS_DIR,
            "dir_exists": dir_exists,
            "valid_maps": manager.list_maps(),
            "raw_files": listing,
        }

    @app.get("/api/state")
    async def api_state():
        """Debug: the same payload the WebSocket broadcasts."""
        return {"status": _status_payload(), "maps": manager.list_maps(),
                "places": _places_payload()}

    @app.websocket("/ws")
    async def ws_endpoint(ws: WebSocket) -> None:
        await ws.accept()
        clients.add(ws)
        print(f"[cockpit] phone connected ({ws.client.host})", flush=True)
        try:
            # Full snapshot -> reconnect lands exactly where things are.
            await _push_state(ws)
            await _push_map(ws)
            try:
                while True:
                    raw = await ws.receive_text()
                    try:
                        msg = json.loads(raw)
                    except json.JSONDecodeError:
                        continue
                    await _handle(ws, msg)
            except WebSocketDisconnect:
                pass
        finally:
            clients.discard(ws)
            print("[cockpit] phone disconnected", flush=True)
            iface.joystick_stop()   # manual driving never survives the pilot

    # ---------------------------------------------------------- commands

    async def _handle(ws, msg: Dict) -> None:
        mtype = msg.get("type")

        if mtype == "ping":
            await _send(ws, {"type": "pong"})

        elif mtype == "joy":
            st = manager.status()
            allowed = ((st["mode"] == "slam" and not st["explore"])
                       or st["mode"] == "nav")
            if allowed:
                if st["mode"] == "nav":
                    # Human grabbed the stick: kill any Nav2 goal, ALWAYS.
                    # This used to be gated on our own tracked goal state,
                    # which meant a goal we mis-tracked as finished survived
                    # the joystick session and resumed driving to the old
                    # destination the moment the stick was released.
                    iface.cancel_goal()
                iface.set_joystick(msg.get("v", 0.0), msg.get("w", 0.0))

        elif mtype == "joy_stop":
            iface.joystick_stop()

        elif mtype == "estop":
            iface.joystick_stop()
            iface.cancel_goal()
            if manager.explore_running:
                await asyncio.to_thread(manager.stop_explore)
            await _toast(ws, "warn", "STOP: goal canceled, explore halted")
            await _push_state()

        elif mtype in ("start_slam", "start_nav", "stop_all",
                       "explore_start", "explore_stop", "save_map"):
            if mode_lock.locked():
                await _toast(ws, "warn", "Busy — previous operation running")
                return
            async with mode_lock:
                manager.busy = True
                await _push_state()
                try:
                    await _mode_op(ws, mtype, msg)
                finally:
                    manager.busy = False
                await _push_state()

        elif mtype == "save_place":
            await _save_place(ws, str(msg.get("name", "")).strip())

        elif mtype == "goto":
            await _goto(ws, str(msg.get("name", "")).strip())

        elif mtype == "set_pose":
            await _set_pose(ws, str(msg.get("name", "")).strip())

        elif mtype == "set_setting":
            name = str(msg.get("name", ""))
            on = bool(msg.get("on"))
            if manager.set_setting(name, on):
                await _toast(ws, "info",
                             f"{name.replace('_', ' ')} " + ("ON" if on else "OFF"))
                await _push_state()

        elif mtype == "set_autoinit":
            on = bool(msg.get("on"))
            manager.set_pose_autoinit(on)
            await _toast(ws, "info",
                         "Auto pose init " + ("ON — applies at next START NAV"
                                              if on else "OFF"))
            await _push_state()

        elif mtype == "cancel_goal":
            iface.cancel_goal()
            await _toast(ws, "info", "Goal cancel requested")

        elif mtype == "clear_goal":
            iface.clear_goal()

        elif mtype == "delete_place":
            store = _store()
            name = str(msg.get("name", "")).strip()
            if store and manager.current_map and store.delete(
                    name, manager.current_map):
                await _toast(ws, "info", f"Deleted '{name}'")
            await _push_state()

    async def _mode_op(ws, mtype: str, msg: Dict) -> None:
        if mtype == "start_slam":
            iface.joystick_stop()
            iface.cancel_goal()
            name = str(msg.get("name", "")).strip()
            ok, text = await asyncio.to_thread(manager.start_slam, name)
        elif mtype == "start_nav":
            # Also the map-SWITCH path: kill any active goal before the
            # old stack (and its action server) goes down under it.
            iface.joystick_stop()
            iface.cancel_goal()
            iface.clear_autoinit()   # new session -> fresh autoinit status
            ok, text = await asyncio.to_thread(
                manager.start_nav, str(msg.get("map", "")).strip())
        elif mtype == "stop_all":
            iface.joystick_stop()
            iface.cancel_goal()
            iface.clear_autoinit()   # stale SEEDED must not outlive the stack
            ok, text = await asyncio.to_thread(manager.stop_all)
        elif mtype == "explore_start":
            ok, text = await asyncio.to_thread(manager.start_explore)
        elif mtype == "explore_stop":
            ok, text = await asyncio.to_thread(manager.stop_explore)
        elif mtype == "save_map":
            ok, text = await _save_map()
        else:
            return
        await _toast(ws, "ok" if ok else "error", text)

    async def _save_map():
        st = manager.status()
        name = st["map_name"]
        if st["mode"] != "slam" or not name:
            return False, "Save map only works in SLAM mode"
        path = os.path.join(MAPS_DIR, name)
        os.makedirs(MAPS_DIR, exist_ok=True)
        ok1, t1 = await asyncio.to_thread(iface.serialize_map, path)
        if not ok1:
            return False, t1
        ok2, t2 = await asyncio.to_thread(manager.save_map_grid, name)
        if not ok2:
            return False, f"posegraph saved, but {t2}"
        return True, f"Map '{name}' saved (posegraph + grid)"

    async def _save_place(ws, name: str) -> None:
        store = _store()
        st = manager.status()
        if store is None:
            await _toast(ws, "error", "rover_places package not built")
            return
        if st["mode"] != "nav":
            await _toast(ws, "error",
                         "Save location only in NAV mode (map must be fixed)")
            return
        if not name or not manager.valid_name(name):
            await _toast(ws, "error",
                         "Name: letters/digits/_/- only (max 40)")
            return
        pose = iface.get_pose()
        if pose is None:
            await _toast(ws, "error", "Not localized — cannot save pose")
            return
        existed = store.get(name, st["map_name"]) is not None
        store.put(Place(
            name=name, map_name=st["map_name"],
            x=pose["x"], y=pose["y"], qz=pose["qz"], qw=pose["qw"],
        ), overwrite=True)
        note = " (replaced previous)" if existed else ""
        await _toast(ws, "ok", f"Saved '{name}'{note}")
        await _push_state()

    async def _set_pose(ws, name: str) -> None:
        """Tell localization the rover is PHYSICALLY at a saved place."""
        store = _store()
        st = manager.status()
        if store is None or st["mode"] != "nav":
            await _toast(ws, "error", "Set-pose only works in NAV mode")
            return
        place = store.get(name, st["map_name"])
        if place is None:
            await _toast(ws, "error", f"No place '{name}' on this map")
            return
        ok, text = iface.set_initial_pose(
            place.x, place.y, place.qz, place.qw)
        if ok:
            await _toast(ws, "ok",
                         f"Localization re-anchored at '{name}' — verify the "
                         "arrow matches reality before sending goals")
        else:
            await _toast(ws, "error", text)

    async def _goto(ws, name: str) -> None:
        store = _store()
        st = manager.status()
        if store is None or st["mode"] != "nav":
            await _toast(ws, "error", "Go-to only works in NAV mode")
            return
        place = store.get(name, st["map_name"])
        if place is None:
            await _toast(ws, "error", f"No place '{name}' on this map")
            return
        ok, text = iface.send_goal(
            place.name, place.x, place.y, place.qz, place.qw)
        await _toast(ws, "ok" if ok else "error", text)

    return app


class _WSDebug:
    """ASGI tap: prints every websocket handshake and what the app answers.

    Temporary diagnostic for the /ws 403 hunt — shows the request headers,
    each message the app sends back, and any exception the framework's
    exception middleware would otherwise swallow.
    """

    def __init__(self, app):
        self.app = app

    async def __call__(self, scope, receive, send):
        if scope["type"] != "websocket":
            await self.app(scope, receive, send)
            return
        hdrs = {k.decode(): v.decode() for k, v in scope.get("headers", [])}
        print(f"[ws-debug] handshake path={scope.get('path')!r} "
              f"headers={hdrs}", flush=True)

        async def send_tap(message):
            print(f"[ws-debug] app -> client: {message.get('type')} "
                  f"{ {k: v for k, v in message.items() if k != 'type'} }",
                  flush=True)
            await send(message)

        try:
            await self.app(scope, receive, send_tap)
        except BaseException as exc:  # noqa: BLE001
            import traceback
            print(f"[ws-debug] EXCEPTION: {exc!r}", flush=True)
            traceback.print_exc()
            raise


def _lan_addresses() -> List[str]:
    try:
        out = subprocess.run(["hostname", "-I"], capture_output=True,
                             text=True, timeout=5).stdout
        return [a for a in out.split() if not a.startswith("172.")]
    except Exception:
        return []


def main() -> int:
    _require_web_deps()
    import uvicorn

    parser = argparse.ArgumentParser(prog="rover_web")
    parser.add_argument("--port", type=int, default=8080)
    args, _ = parser.parse_known_args()

    iface = RosInterface()
    iface.start()
    manager = LaunchManager()
    # A previous server may have died without cleanup — sweep any orphaned
    # rover launches so our IDLE state matches what's actually running.
    print("Sweeping for orphaned rover processes from a previous run...")
    manager.stop_all()
    app = create_app(iface, manager)

    addrs = _lan_addresses() or ["<jetson-ip>"]
    print("\n  Rover cockpit up.  On the phone (same WiFi) open:")
    for a in addrs:
        print(f"      http://{a}:{args.port}")
    print()

    try:
        uvicorn.run(_WSDebug(app), host="0.0.0.0", port=args.port,
                    log_level="info")
    finally:
        print("\nServer exiting — stopping everything it launched...")
        iface.joystick_stop()
        try:
            manager.stop_all()
        except Exception as exc:  # noqa: BLE001
            print(f"cleanup error (run the manual pkill sweep): {exc}")
        iface.shutdown()
        print("Clean shutdown complete.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
