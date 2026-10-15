/* Rover cockpit — stateless remote for the rover_web server.
   All truth lives server-side; this page just renders snapshots and
   sends commands. Survives disconnects: on reconnect the server
   resends full state and we resume where the rover actually is. */
"use strict";

const $ = (id) => document.getElementById(id);

/* ------------------------------------------------ state */
let ws = null;
let wsOk = false;
let state = null;          // last {status, maps, places}
let mapImg = null;         // decoded map Image
let mapMeta = null;        // {w,h,res,ox,oy}
let pose = null;           // {x,y,yaw}
let places = [];
let reconnectDelay = 500;

/* ------------------------------------------------ websocket */
function connect() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  ws = new WebSocket(`${proto}://${location.host}/ws`);

  ws.onopen = () => {
    wsOk = true;
    reconnectDelay = 500;
    setChip($("conn"), true, "LINK");
  };
  ws.onclose = () => {
    wsOk = false;
    setChip($("conn"), false, "LINK");
    setTimeout(connect, reconnectDelay);
    reconnectDelay = Math.min(reconnectDelay * 2, 5000);
  };
  ws.onerror = () => ws.close();
  ws.onmessage = (ev) => {
    let msg;
    try { msg = JSON.parse(ev.data); } catch { return; }
    handle(msg);
  };
}

function send(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj));
}

setInterval(() => send({ type: "ping" }), 5000);

/* ------------------------------------------------ message handling */
function handle(msg) {
  if (msg.type === "state") {
    state = msg;
    places = msg.places || [];
    renderState();
    drawMap();
  } else if (msg.type === "map") {
    mapMeta = msg.meta;
    const img = new Image();
    img.onload = () => { mapImg = img; drawMap(); };
    img.src = "data:image/png;base64," + msg.png;
  } else if (msg.type === "pose") {
    pose = msg;
    drawMap();
  } else if (msg.type === "log") {
    appendLog(msg.text);
  } else if (msg.type === "toast") {
    toast(msg.level, msg.text);
  }
}

/* ------------------------------------------------ status UI */
function setChip(el, on, label) {
  el.className = "chip " + (on ? "on" : "off");
  if (label) el.textContent = label;
}

function renderState() {
  const s = state.status;
  setChip($("st-stack"), s.stack, "STACK");
  setChip($("st-loc"), s.loc, "LOC");
  setChip($("st-nav2"), s.nav2, "NAV2");
  setChip($("st-map"), s.map, "MAP");
  $("st-busy").classList.toggle("hidden", !s.busy);

  const badge = $("mode-badge");
  badge.textContent = s.mode.toUpperCase() + (s.explore ? " · EXPLORE" : "");

  /* goal banner */
  const g = s.goal;
  if (g) {
    const rem = g.remaining != null ? ` — ${g.remaining.toFixed(2)} m left` : "";
    $("goal-text").textContent = `→ ${g.name}: ${g.state}${rem}`;
    $("goal-banner").classList.remove("hidden");
    const finished = ["arrived", "canceled", "rejected"].includes(g.state)
      || g.state.startsWith("failed");
    $("btn-cancel-goal").classList.toggle("hidden", finished);
    if (finished) setTimeout(() => send({ type: "clear_goal" }), 4000);
  } else {
    $("goal-banner").classList.add("hidden");
  }

  /* joystick: shared control — manual driving in SLAM, and in NAV for
     reaching new spots to save (grabbing it cancels any active goal) */
  const canDrive = (s.mode === "slam" && !s.explore) || s.mode === "nav";
  $("joy-block").classList.toggle("hidden", !canDrive);

  /* SLAM tab */
  const inSlam = s.mode === "slam";
  $("slam-idle").classList.toggle("hidden", s.mode !== "idle");
  $("slam-active").classList.toggle("hidden", !inSlam);
  if (inSlam) {
    $("slam-session").textContent = s.map_name || "?";
    $("btn-explore-start").classList.toggle("hidden", s.explore);
    $("btn-explore-stop").classList.toggle("hidden", !s.explore);
  }
  if (s.mode === "nav") {
    /* if someone started NAV, SLAM tab shows nothing actionable */
    $("slam-idle").classList.add("hidden");
  }

  /* NAV tab */
  const inNav = s.mode === "nav";
  $("nav-idle").classList.toggle("hidden", s.mode !== "idle");
  $("nav-active").classList.toggle("hidden", !inNav);
  if (inNav) $("nav-session").textContent = s.map_name || "?";

  /* auto pose-init toggle (server-persisted) + live status line */
  $("chk-autoinit").checked = !!s.pose_autoinit;
  const ai = s.autoinit;
  const aiLine = $("autoinit-line");
  if (inNav && ai) {
    if (ai.startsWith("SEEDED")) {
      aiLine.textContent = "✓ pose auto-initialized (" + ai + ") — ready for goals";
    } else if (ai.startsWith("WAITING")) {
      aiLine.textContent = "… auto pose init pending (localizer starting)";
    } else {
      aiLine.textContent = "⚠ auto pose init skipped: " + ai + " — use SET POSE";
    }
    aiLine.classList.remove("hidden");
  } else if (inNav && !s.pose_autoinit) {
    aiLine.textContent = "auto pose init is OFF — use SET POSE";
    aiLine.classList.remove("hidden");
  } else {
    aiLine.classList.add("hidden");
  }

  /* settings toggles + log panel visibility */
  $("chk-runlogger").checked = !!s.run_logger;
  $("chk-logpanel").checked = !!s.log_panel;
  $("log-panel").classList.toggle("hidden", !s.log_panel);

  /* uptime footer */
  const u = s.uptime;
  $("uptime-line").textContent = u
    ? `on ${fmtDur(u.jetson_uptime_s)} · booted ${u.jetson_boot}`
      + (u.last_off ? ` · last off ~${u.last_off}` : "")
      + ` · ${u.boots} boots recorded`
    : "";

  /* dropdowns — rebuilt only when contents change, so an open iOS
     picker is never yanked away by a routine state broadcast */
  syncOptions($("nav-map-select"), state.maps || []);
  syncOptions($("nav-map-switch"), state.maps || []);
  syncOptions($("place-select"), places.map((p) => p.name));
  syncOptions($("pose-select"), places.map((p) => p.name));

  /* disable actions while busy */
  document.querySelectorAll("button.primary, button.secondary, button.danger")
    .forEach((b) => { if (b.id !== "btn-estop") b.disabled = s.busy; });
}

function fmtDur(s) {
  s = Math.max(0, Math.floor(s));
  const h = Math.floor(s / 3600), m = Math.floor((s % 3600) / 60);
  return h > 0 ? `${h}h ${m}m` : `${m}m`;
}

function syncOptions(sel, values) {
  const existing = [...sel.options].map((o) => o.value);
  if (existing.length === values.length
      && existing.every((v, i) => v === values[i])) return;
  const prev = sel.value;
  sel.innerHTML = "";
  values.forEach((v) => {
    const o = document.createElement("option");
    o.value = o.textContent = v;
    sel.appendChild(o);
  });
  if (values.includes(prev)) sel.value = prev;
}

/* ------------------------------------------------ map canvas */
function drawMap() {
  const canvas = $("map-canvas");
  const wrap = $("map-wrap");
  const dpr = window.devicePixelRatio || 1;
  const cw = wrap.clientWidth, ch = wrap.clientHeight;
  if (cw < 2 || ch < 2) return;          /* container not laid out yet */
  /* Resize on EITHER dimension.  The old check tested width only, so a tab
     switch (which changes height alone) left the canvas at its stale pixel
     height while CSS stretched it — a vertically squashed map and a rover
     marker drawn in the wrong place. */
  const pw = Math.round(cw * dpr), ph = Math.round(ch * dpr);
  if (canvas.width !== pw || canvas.height !== ph) {
    canvas.width = pw;
    canvas.height = ph;
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, cw, ch);

  if (!mapImg || !mapMeta) { $("map-empty").style.display = "flex"; return; }
  $("map-empty").style.display = "none";

  const scale = Math.min(cw / mapMeta.w, ch / mapMeta.h);
  const offX = (cw - mapMeta.w * scale) / 2;
  const offY = (ch - mapMeta.h * scale) / 2;
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(mapImg, offX, offY, mapMeta.w * scale, mapMeta.h * scale);

  /* world -> canvas (PNG row 0 = top = max Y of grid) */
  const toCanvas = (wx, wy) => {
    const gx = (wx - mapMeta.ox) / mapMeta.res;
    const gy = (wy - mapMeta.oy) / mapMeta.res;
    return [offX + gx * scale, offY + (mapMeta.h - gy) * scale];
  };

  /* saved places */
  places.forEach((p) => {
    const [cx, cy] = toCanvas(p.x, p.y);
    ctx.fillStyle = "#d29922";
    ctx.beginPath();
    ctx.arc(cx, cy, 4, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = "#d29922";
    ctx.font = "9px sans-serif";
    ctx.fillText(p.name, cx + 6, cy + 3);
  });

  /* rover — drawn big and high-contrast so it reads on a phone screen:
     white halo ring + green arrow with dark outline + heading tick */
  if (pose) {
    const [cx, cy] = toCanvas(pose.x, pose.y);
    const a = -pose.yaw; /* canvas y is flipped */
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(a);
    ctx.beginPath();                    /* halo */
    ctx.arc(0, 0, 15, 0, Math.PI * 2);
    ctx.fillStyle = "rgba(255,255,255,0.65)";
    ctx.fill();
    ctx.strokeStyle = "#3fb950";
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.beginPath();                    /* heading tick */
    ctx.moveTo(15, 0);
    ctx.lineTo(24, 0);
    ctx.lineWidth = 3;
    ctx.strokeStyle = "#3fb950";
    ctx.stroke();
    ctx.beginPath();                    /* arrow body */
    ctx.moveTo(13, 0);
    ctx.lineTo(-9, 8);
    ctx.lineTo(-9, -8);
    ctx.closePath();
    ctx.fillStyle = "#3fb950";
    ctx.strokeStyle = "#0d1117";
    ctx.lineWidth = 2;
    ctx.fill();
    ctx.stroke();
    ctx.restore();
  }
}
window.addEventListener("resize", drawMap);
window.addEventListener("orientationchange", () => setTimeout(drawMap, 250));
/* Definitive: redraw whenever the container's box changes for ANY reason —
   tab switch, expand, rotate, browser chrome hiding.  Catches everything the
   discrete event listeners above miss. */
if (window.ResizeObserver) {
  new ResizeObserver(() => drawMap()).observe($("map-wrap"));
}

/* fullscreen map toggle — the only way to read the map on a phone */
$("map-expand").onclick = (ev) => {
  ev.stopPropagation();
  const wrap = $("map-wrap");
  const big = wrap.classList.toggle("expanded");
  $("map-expand").textContent = big ? "⤡" : "⤢";
  /* wait for the layout to settle before re-measuring the canvas */
  setTimeout(drawMap, 60);
};

/* ------------------------------------------------ tabs */
document.querySelectorAll(".tab").forEach((btn) => {
  btn.onclick = () => {
    document.querySelectorAll(".tab").forEach((b) => b.classList.remove("active"));
    btn.classList.add("active");
    ["slam", "nav", "voice"].forEach((t) =>
      $("tab-" + t).classList.toggle("hidden", t !== btn.dataset.tab));
  };
});

/* ------------------------------------------------ joystick */
(function joystick() {
  const pad = $("joystick");
  const knob = $("joy-knob");
  const R = 95, KR = 34, MAX = R - KR;
  let active = false, lastSend = 0;

  function setKnob(dx, dy) {
    knob.style.left = 61 + dx + "px";
    knob.style.top = 61 + dy + "px";
  }
  function handlePoint(ev) {
    const rect = pad.getBoundingClientRect();
    let dx = ev.clientX - rect.left - R;
    let dy = ev.clientY - rect.top - R;
    const d = Math.hypot(dx, dy);
    if (d > MAX) { dx *= MAX / d; dy *= MAX / d; }
    setKnob(dx, dy);
    const now = Date.now();
    if (now - lastSend > 80) {
      lastSend = now;
      /* up = forward (+v); left = CCW (+w) */
      send({ type: "joy", v: -dy / MAX * 0.25, w: -dx / MAX * 0.8 });
    }
  }
  function release() {
    if (!active) return;
    active = false;
    setKnob(0, 0);
    send({ type: "joy_stop" });
  }
  pad.addEventListener("pointerdown", (ev) => {
    active = true;
    pad.setPointerCapture(ev.pointerId);
    handlePoint(ev);
    ev.preventDefault();
  });
  pad.addEventListener("pointermove", (ev) => { if (active) handlePoint(ev); });
  pad.addEventListener("pointerup", release);
  pad.addEventListener("pointercancel", release);
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) release();
  });
})();

/* ------------------------------------------------ buttons */
$("btn-start-slam").onclick = () => {
  const name = $("slam-name").value.trim();
  if (!name) { toast("error", "Enter a map name first"); return; }
  send({ type: "start_slam", name });
};
$("btn-save-map").onclick = () => send({ type: "save_map" });
$("btn-explore-start").onclick = () => send({ type: "explore_start" });
$("btn-explore-stop").onclick = () => send({ type: "explore_stop" });
$("btn-stop-slam").onclick = () => {
  if (confirm("Stop the SLAM session? Unsaved mapping is lost."))
    send({ type: "stop_all" });
};

$("btn-start-nav").onclick = () => {
  const map = $("nav-map-select").value;
  if (!map) { toast("error", "No map selected"); return; }
  send({ type: "start_nav", map });
};
$("btn-stop-nav").onclick = () => {
  if (confirm("Stop NAV mode?")) send({ type: "stop_all" });
};
$("btn-set-pose").onclick = () => {
  const name = $("pose-select").value;
  if (!name) { toast("error", "No place selected"); return; }
  if (confirm(`Rover is PHYSICALLY parked at '${name}' right now?`))
    send({ type: "set_pose", name });
};
$("btn-goto").onclick = () => {
  const name = $("place-select").value;
  if (!name) { toast("error", "No place selected"); return; }
  send({ type: "goto", name });
};
$("btn-del-place").onclick = () => {
  const name = $("place-select").value;
  if (!name) { toast("error", "No place selected"); return; }
  if (confirm(`Delete saved place '${name}'?`))
    send({ type: "delete_place", name });
};
$("btn-switch-map").onclick = () => {
  const map = $("nav-map-switch").value;
  if (!map) { toast("error", "No map selected"); return; }
  if (state && state.status && map === state.status.map_name) {
    toast("info", `Already on '${map}'`);
    return;
  }
  if (confirm(`Switch to map '${map}'? Localization restarts (~30 s).`))
    send({ type: "start_nav", map });
};
$("btn-save-place").onclick = () => {
  const name = $("place-name").value.trim();
  if (!name) { toast("error", "Enter a name for this spot"); return; }
  send({ type: "save_place", name });
  $("place-name").value = "";   /* box clears, ready for the next one */
};
$("btn-cancel-goal").onclick = () => send({ type: "cancel_goal" });
$("btn-estop").onclick = () => send({ type: "estop" });
$("chk-autoinit").onchange = () =>
  send({ type: "set_autoinit", on: $("chk-autoinit").checked });
$("chk-runlogger").onchange = () =>
  send({ type: "set_setting", name: "run_logger", on: $("chk-runlogger").checked });
$("chk-logpanel").onchange = () =>
  send({ type: "set_setting", name: "log_panel", on: $("chk-logpanel").checked });

/* ------------------------------------------------ log + toasts */
function appendLog(text) {
  const body = $("log-body");
  body.textContent += text + "\n";
  const lines = body.textContent.split("\n");
  if (lines.length > 300) body.textContent = lines.slice(-300).join("\n");
  body.scrollTop = body.scrollHeight;
}

function toast(level, text) {
  const div = document.createElement("div");
  div.className = "toast " + level;
  div.textContent = text;
  $("toasts").appendChild(div);
  setTimeout(() => div.remove(), 3500);
}

connect();
