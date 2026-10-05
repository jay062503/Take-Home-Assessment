"""Photo tier whole-property stitch.

Each room folder is reconstructed in its own Manhattan frame (unknown 90-degree yaw and offset).
Rooms are connected through doors:
  1. door candidates per room (door/passage openings with width and outward normal)
  2. pair score = appearance (ORB matches between the two rooms' photos: a photo taken towards a
     door sees into the next room) - width mismatch penalty; optional `connections.txt` hints
  3. greedy maximum-score spanning tree; each placement snaps the two doors centre-to-centre with
     opposite normals and a wall-thickness gap, and is rejected if it overlaps a placed room
Rooms that cannot be connected are still placed (to the side) and reported as unresolved, never
silently glued.
"""
from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
from shapely.geometry import Point, Polygon

from ..geometry.structure import opening_world_center
from ..pose.rgbd_odometry import features, match
from ..types import FrameSet, Measure, Room, combine_sigma


def _R2(k: int) -> np.ndarray:
    a = k * np.pi / 2
    return np.round(np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]))


def transform_room(room: Room, k: int, t: np.ndarray) -> None:
    R = _R2(k)
    room.polygon = room.polygon @ R.T + t
    poly = Polygon(room.polygon)
    for w in room.walls:
        w.start = R @ w.start + t
        w.end = R @ w.end + t
        if k % 2:
            w.axis = 1 - w.axis
        w.coord = Measure(float(w.start[w.axis]), w.coord.sigma)
        mid = (w.start + w.end) / 2
        probe = mid.copy()
        probe[w.axis] += 0.05
        w.inward = 1 if poly.contains(Point(probe)) else -1


def transform_frames(fs: FrameSet, k: int, t: np.ndarray) -> None:
    R = np.eye(4)
    R[:2, :2] = _R2(k)
    R[:2, 3] = t
    for fr in fs.frames:
        fr.T_wc = R @ fr.T_wc


def _doors(room: Room):
    out = []
    for op in room.openings:
        if op.type not in ("door", "passage"):
            continue
        w = room.walls[op.wall_index]
        n = np.zeros(2)
        n[w.axis] = -w.inward       # outward
        out.append((op, opening_world_center(room, op), n))
    return out


def appearance_matrix(rooms_fs: List[FrameSet]) -> np.ndarray:
    feats = [[features(f.rgb) for f in fs.frames] for fs in rooms_fs]
    n = len(rooms_fs)
    S = np.zeros((n, n))
    for i in range(n):
        for j in range(i + 1, n):
            best = 0
            for fa in feats[i]:
                for fb in feats[j]:
                    best = max(best, len(match(fa[1], fb[1], ratio=0.7)))
            S[i, j] = S[j, i] = best
    return S


def read_hints(root: Path) -> Optional[set]:
    f = root / "connections.txt"
    if not f.exists():
        return None
    pairs = set()
    for line in f.read_text().splitlines():
        line = line.split("#")[0].strip()
        if not line:
            continue
        for sep in ("-", ",", "<->", ">"):
            if sep in line:
                a, b = [x.strip() for x in line.split(sep, 1)]
                pairs.add(tuple(sorted((a, b))))
                break
    return pairs


def stitch(rooms: List[Room], rooms_fs: List[FrameSet], cfg: Dict, root: Path) -> Dict:
    sc = cfg["stitch"]
    thick = float(sc["wall_thickness"])
    tol = float(sc["door_width_tol"])
    n = len(rooms)
    report = {"method": "door-to-door snapping, appearance-scored spanning tree", "edges": [], "unresolved": [],
              "overlaps": [], "hints_used": False}
    if n == 0:
        return report
    S = appearance_matrix(rooms_fs) if n > 1 else np.zeros((1, 1))
    report["appearance_matches"] = S.astype(int).tolist()
    hints = read_hints(root)
    if hints:
        report["hints_used"] = True
    names = [r.label for r in rooms]
    doors = [_doors(r) for r in rooms]

    order = sorted(range(n), key=lambda i: (-len(doors[i]), -Polygon(rooms[i].polygon).area))
    placed = {order[0]}
    used = set()
    place_sigma = {order[0]: 0.0}
    while len(placed) < n:
        cands = []
        for a in placed:
            for b in range(n):
                if b in placed:
                    continue
                if hints is not None and tuple(sorted((names[a], names[b]))) not in hints:
                    continue
                for ia, (oa, ca, na) in enumerate(doors[a]):
                    if (a, ia) in used:
                        continue
                    for ib, (ob, cb, nb) in enumerate(doors[b]):
                        if (b, ib) in used:
                            continue
                        wa, wb = oa.width.value, ob.width.value
                        dw = abs(wa - wb) / max(wa, wb)
                        if dw > tol:
                            continue
                        score = np.log1p(S[a, b]) - 4.0 * dw
                        cands.append((score, a, ia, b, ib))
        cands.sort(reverse=True)
        done = False
        for score, a, ia, b, ib in cands:
            oa, ca, na = doors[a][ia]
            ob, cb, nb = doors[b][ib]
            # rotate b so its outward normal is opposite to a's
            k = next(k for k in range(4) if np.allclose(_R2(k) @ nb, -na))
            cb_r = _R2(k) @ cb
            t = ca + na * thick - cb_r
            cand_poly = Polygon(rooms[b].polygon @ _R2(k).T + t)
            bad = False
            for p in placed:
                inter = cand_poly.buffer(-0.03).intersection(Polygon(rooms[p].polygon).buffer(-0.03)).area
                if inter > 0.05:
                    bad = True
                    break
            if bad:
                continue
            transform_room(rooms[b], k, t)
            transform_frames(rooms_fs[b], k, t)
            placed.add(b)
            used.add((a, ia))
            used.add((b, ib))
            s_place = combine_sigma(place_sigma[a], oa.center_offset.sigma, ob.center_offset.sigma,
                                    float(sc["wall_thickness_sigma"]))
            place_sigma[b] = s_place
            oa.connects_to = rooms[b].id
            ob.connects_to = rooms[a].id
            report["edges"].append({"a": rooms[a].id, "b": rooms[b].id, "via": [oa.id, ob.id],
                                    "appearance_matches": int(S[a, b]), "width_mismatch": round(abs(oa.width.value - ob.width.value), 3),
                                    "placement_sigma": round(s_place, 4)})
            done = True
            break
        if not done:
            # nothing connects: park the largest remaining room to the right, flagged
            b = max((i for i in range(n) if i not in placed), key=lambda i: Polygon(rooms[i].polygon).area)
            xmax = max(Polygon(rooms[p].polygon).bounds[2] for p in placed)
            bx0, by0, _, _ = Polygon(rooms[b].polygon).bounds
            t = np.array([xmax + 1.0 - bx0, -by0])
            transform_room(rooms[b], 0, t)
            transform_frames(rooms_fs[b], 0, t)
            placed.add(b)
            place_sigma[b] = float("inf")
            report["unresolved"].append(rooms[b].id)
    for i, r in enumerate(rooms):
        r.quality["placement_sigma"] = None if not np.isfinite(place_sigma[i]) else round(place_sigma[i], 4)
    for i in range(n):
        for j in range(i + 1, n):
            a = Polygon(rooms[i].polygon).intersection(Polygon(rooms[j].polygon)).area
            if a > 0.05:
                report["overlaps"].append({"a": rooms[i].id, "b": rooms[j].id, "area_m2": round(a, 3)})
    return report
