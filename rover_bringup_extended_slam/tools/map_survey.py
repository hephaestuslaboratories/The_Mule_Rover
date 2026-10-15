#!/usr/bin/env python3
"""map_survey.py — measure rooms from a saved occupancy-grid map.

Whole-house breakdown: segments the map's free space into rooms (doorways are
"closed" by eroding the free space, surviving cores become room seeds, every
free cell is then assigned to its nearest seed), and reports per-room area,
dimensions, and perimeter.  Optionally labels rooms with rover_places names.

Usage (inside the container, or anywhere python3+numpy exists):

    python3 map_survey.py /workspaces/isaac_ros-dev/slm_maps/my_house
    python3 map_survey.py my_house.yaml --places /workspaces/isaac_ros-dev/rover_places.json \
        --map-name my_house --png survey.png --door-width 0.9

Accuracy: expect a few cm of local error (grid quantization + calibration) and
small global warp between distant rooms (loop-closure residuals).  Great for
"how big is the living room"; not a legal survey.

No ROS dependencies — plain python3 + numpy.
"""

import argparse
import json
import math
import os
import struct
import sys
import zlib
from collections import deque

import numpy as np

# ── map loading ──────────────────────────────────────────────────────────────

def load_yaml(path):
    """Minimal flat-YAML parse for map_saver metadata (no pyyaml needed)."""
    meta = {}
    with open(path, "r") as fh:
        for line in fh:
            line = line.split("#", 1)[0].strip()
            if ":" not in line:
                continue
            k, v = line.split(":", 1)
            meta[k.strip()] = v.strip()
    origin = meta.get("origin", "[0,0,0]").strip("[] ")
    ox, oy = [float(t) for t in origin.split(",")[:2]]
    return {
        "image": meta["image"],
        "res": float(meta["resolution"]),
        "ox": ox,
        "oy": oy,
        "negate": int(meta.get("negate", 0)),
        "occ_th": float(meta.get("occupied_thresh", 0.65)),
        "free_th": float(meta.get("free_thresh", 0.196)),
    }


def load_pgm(path):
    with open(path, "rb") as fh:
        data = fh.read()
    # header: magic, whitespace/comments, width, height, maxval
    tokens, i = [], 0
    while len(tokens) < 4:
        # skip whitespace
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b"#":                     # comment to end of line
            while i < len(data) and data[i] != 0x0A:
                i += 1
            continue
        j = i
        while j < len(data) and not data[j:j + 1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j
    magic, w, h, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    i += 1  # single whitespace after maxval
    if magic == b"P5":
        img = np.frombuffer(data[i:i + w * h], dtype=np.uint8).reshape(h, w)
    elif magic == b"P2":
        img = np.array(data[i:].split()[: w * h], dtype=np.uint8).reshape(h, w)
    else:
        sys.exit(f"Unsupported PGM magic {magic!r}")
    return img.astype(np.float32) / float(maxval)


def classify(img_norm, meta):
    """Return int8 grid: 1 free, 0 unknown, -1 occupied (map_saver convention)."""
    occ_prob = img_norm if meta["negate"] else (1.0 - img_norm)
    grid = np.zeros(img_norm.shape, dtype=np.int8)
    grid[occ_prob > meta["occ_th"]] = -1
    grid[occ_prob < meta["free_th"]] = 1
    return grid

# ── geometry helpers ─────────────────────────────────────────────────────────

N4 = ((-1, 0), (1, 0), (0, -1), (0, 1))


def bfs_distance_to_nonfree(grid):
    """Grid-steps from every free cell to the nearest non-free cell."""
    h, w = grid.shape
    dist = np.full((h, w), -1, dtype=np.int32)
    q = deque()
    nonfree = np.argwhere(grid != 1)
    for r, c in nonfree:
        dist[r, c] = 0
        q.append((int(r), int(c)))
    while q:
        r, c = q.popleft()
        d = dist[r, c] + 1
        for dr, dc in N4:
            nr, nc = r + dr, c + dc
            if 0 <= nr < h and 0 <= nc < w and dist[nr, nc] < 0:
                dist[nr, nc] = d
                q.append((nr, nc))
    return dist


def label_components(mask):
    """4-connected component labels (0 = background)."""
    h, w = mask.shape
    labels = np.zeros((h, w), dtype=np.int32)
    nxt = 0
    for r0, c0 in np.argwhere(mask):
        if labels[r0, c0]:
            continue
        nxt += 1
        q = deque([(int(r0), int(c0))])
        labels[r0, c0] = nxt
        while q:
            r, c = q.popleft()
            for dr, dc in N4:
                nr, nc = r + dr, c + dc
                if 0 <= nr < h and 0 <= nc < w and mask[nr, nc] and not labels[nr, nc]:
                    labels[nr, nc] = nxt
                    q.append((nr, nc))
    return labels, nxt


def grow_regions(grid, seeds):
    """Assign every free cell to the nearest seed label (multi-source BFS)."""
    h, w = grid.shape
    out = seeds.copy()
    q = deque(map(tuple, np.argwhere(seeds > 0)))
    while q:
        r, c = q.popleft()
        lab = out[r, c]
        for dr, dc in N4:
            nr, nc = r + dr, c + dc
            if 0 <= nr < h and 0 <= nc < w and grid[nr, nc] == 1 and out[nr, nc] == 0:
                out[nr, nc] = lab
                q.append((nr, nc))
    return out

# ── output ───────────────────────────────────────────────────────────────────

PALETTE = [  # readable on white; walls stay black
    (0x4C, 0x8F, 0xD1), (0xE0, 0xA4, 0x3B), (0x6B, 0xB8, 0x7A), (0xC9, 0x6F, 0x6F),
    (0x9A, 0x7F, 0xC9), (0x5B, 0xC0, 0xBE), (0xD1, 0x8F, 0xC0), (0xB5, 0xB0, 0x4C),
    (0x8F, 0x6A, 0x4C), (0x7F, 0x8F, 0x9F),
]


def write_png_rgb(path, rgb):
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[r].tobytes() for r in range(h))

    def chunk(tag, payload):
        return (struct.pack(">I", len(payload)) + tag + payload
                + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF))

    ihdr = struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)
    with open(path, "wb") as fh:
        fh.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr)
                 + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))

# ── main ─────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("map", help="map base path or .yaml (map_saver output)")
    ap.add_argument("--door-width", type=float, default=0.9,
                    help="max doorway width (m) to close when splitting rooms")
    ap.add_argument("--min-room", type=float, default=1.0,
                    help="minimum room area (m²); smaller fragments are merged away")
    ap.add_argument("--places", help="rover_places.json to label rooms by place names")
    ap.add_argument("--map-name", help="map name for --places scoping")
    ap.add_argument("--png", help="write a colored room map to this PNG path")
    ap.add_argument("--drop", help="comma list of room labels/names to exclude "
                                   "from the total (e.g. room_1,garage)")
    ap.add_argument("--keep", help="comma list: count ONLY these rooms")
    ap.add_argument("--include-border", action="store_true",
                    help="count regions that touch the map edge (default: "
                         "excluded — edge-touching free space is outside leakage)")
    args = ap.parse_args()

    ypath = args.map if args.map.endswith(".yaml") else args.map + ".yaml"
    meta = load_yaml(ypath)
    img_path = meta["image"]
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(os.path.abspath(ypath)), img_path)
    grid = classify(load_pgm(img_path), meta)
    res = meta["res"]
    h, w = grid.shape
    cell_area = res * res

    free_total = int((grid == 1).sum())
    print(f"map: {ypath}")
    print(f"grid: {w}×{h} @ {res:.3f} m/cell   free space total: "
          f"{free_total * cell_area:.2f} m²")

    # 1) doorway closing: erode by half the door width
    r_cells = max(1, int(round((args.door_width / 2.0) / res)))
    dist = bfs_distance_to_nonfree(grid)
    cores = (dist > r_cells)

    # 2) room seeds
    seeds, n = label_components(cores)
    if n == 0:
        sys.exit("No room cores found — try a smaller --door-width.")

    # 3) claim all free space
    rooms = grow_regions(grid, seeds)

    # 4) gather stats, merge slivers into their biggest neighbor
    def stats(lab_img, lab):
        cells = np.argwhere(lab_img == lab)
        area = len(cells) * cell_area
        rmin, cmin = cells.min(axis=0); rmax, cmax = cells.max(axis=0)
        dims = ((cmax - cmin + 1) * res, (rmax - rmin + 1) * res)
        cr, cc = cells.mean(axis=0)
        cx = meta["ox"] + (cc + 0.5) * res
        cy = meta["oy"] + (h - 1 - cr + 0.5) * res
        return area, dims, (cx, cy), cells

    # perimeter split: edges facing walls vs facing unknown.  A real room is
    # mostly wall-bounded; a region that mostly borders unknown space is map
    # leakage (scans through windows/glass), not a room.
    def boundary(lab_img, lab):
        m = (lab_img == lab)
        occ_edges = unk_edges = 0
        for dr, dc in N4:
            occ = np.roll(grid == -1, (-dr, -dc), axis=(0, 1))
            unk = np.roll(grid == 0, (-dr, -dc), axis=(0, 1))
            occ_edges += int((m & occ).sum())
            unk_edges += int((m & unk).sum())
        total_e = occ_edges + unk_edges
        encl = (occ_edges / total_e) if total_e else 0.0
        return occ_edges * res, encl

    labs = [l for l in range(1, n + 1) if (rooms == l).any()]
    small = [l for l in labs if stats(rooms, l)[0] < args.min_room]
    for l in small:
        m = (rooms == l)
        neigh = {}
        for dr, dc in N4:
            sh = np.roll(rooms, (-dr, -dc), axis=(0, 1))
            for v in np.unique(sh[m]):
                if v > 0 and v != l:
                    neigh[v] = neigh.get(v, 0) + int((sh[m] == v).sum())
        rooms[m] = max(neigh, key=neigh.get) if neigh else 0
    labs = [l for l in range(1, n + 1) if (rooms == l).any()]

    # 5) place labels
    place_of = {}
    if args.places:
        with open(args.places) as fh:
            pj = json.load(fh)
        for p in pj.get("places", []):
            if args.map_name and p.get("map_name") != args.map_name:
                continue
            col = int((p["x"] - meta["ox"]) / res)
            row = h - 1 - int((p["y"] - meta["oy"]) / res)
            if 0 <= row < h and 0 <= col < w and rooms[row, col] > 0:
                place_of.setdefault(rooms[row, col], []).append(p["name"])

    # 6) build room records + exclusion rules
    ordered = sorted(labs, key=lambda l: -stats(rooms, l)[0])
    # A region is "outside" if it reaches TWO OR MORE different map edges:
    # leakage sprawls around the building; a tightly-cropped real room may
    # touch one edge, but not wrap around to a second.
    edge_count = {}
    for edge in (rooms[0, :], rooms[-1, :], rooms[:, 0], rooms[:, -1]):
        for l in set(np.unique(edge)) - {0}:
            edge_count[l] = edge_count.get(l, 0) + 1
    border_labels = {l for l, n in edge_count.items() if n >= 2}

    recs = []
    for i, l in enumerate(ordered):
        area, (dx, dy), (cx, cy), _ = stats(rooms, l)
        perim, encl = boundary(rooms, l)
        recs.append({"lab": l, "idx": i + 1,
                     "name": ", ".join(place_of.get(l, [])) or f"room_{i + 1}",
                     "area": area, "dx": dx, "dy": dy, "cx": cx, "cy": cy,
                     "perim": perim, "encl": encl,
                     "border": l in border_labels, "excl": None})

    def matches(rec, token):
        t = token.strip().lower()
        return t and (t == f"room_{rec['idx']}" or t in rec["name"].lower())

    drops = [t for t in (args.drop or "").split(",") if t.strip()]
    keeps = [t for t in (args.keep or "").split(",") if t.strip()]
    for rec in recs:
        if rec["border"] and not args.include_border:
            rec["excl"] = "outside (touches map edge)"
        if rec["encl"] < 0.5:
            rec["excl"] = "leaky (borders unknown)"
        if any(matches(rec, t) for t in drops):
            rec["excl"] = "dropped (--drop)"
        if keeps and not any(matches(rec, t) for t in keeps):
            rec["excl"] = "not in --keep"

    # 7) report
    print(f"\nrooms found: {len(recs)}   (door-width {args.door_width} m, "
          f"min-room {args.min_room} m²)\n")
    hdr = (f"{'room':<10}{'label':<26}{'area m²':>9}{'bbox m':>14}"
           f"{'perim m':>9}{'encl%':>7}   center / status")
    print(hdr)
    print("-" * len(hdr))
    total = 0.0
    for rec in recs:
        if rec["excl"] is None:
            total += rec["area"]
        color = PALETTE[(rec["idx"] - 1) % len(PALETTE)]
        chex = "#{:02X}{:02X}{:02X}".format(*color)
        tail = (f"({rec['cx']:+.2f}, {rec['cy']:+.2f})" if rec["excl"] is None
                else f"EXCLUDED: {rec['excl']}")
        print(f"{chex:<10}{rec['name']:<26}{rec['area']:>9.2f}"
              f"{rec['dx']:>7.2f}x{rec['dy']:<6.2f}"
              f"{rec['perim']:>9.2f}{rec['encl'] * 100:>6.0f}%   {tail}")
    print("-" * len(hdr))
    print(f"{'':<10}{'TOTAL (counted rooms)':<26}{total:>9.2f}")
    print("\nExclusion rules: touches-map-edge (disable with --include-border), "
          "encl%<50 vs unknown,\n--drop/--keep lists (match room_N or any part "
          "of a label, case-insensitive).")

    # 8) optional PNG (excluded regions rendered pale grey)
    if args.png:
        rgb = np.full((h, w, 3), 205, dtype=np.uint8)      # unknown grey
        rgb[grid == -1] = (30, 30, 30)                     # walls
        rgb[(grid == 1) & (rooms == 0)] = (240, 240, 240)  # unclaimed free
        for rec in recs:
            color = ((225, 228, 231) if rec["excl"]
                     else PALETTE[(rec["idx"] - 1) % len(PALETTE)])
            rgb[rooms == rec["lab"]] = color
        write_png_rgb(args.png, rgb)
        print(f"\ncolored room map -> {args.png}")


if __name__ == "__main__":
    main()
