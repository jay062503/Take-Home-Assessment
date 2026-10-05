"""Frame-free matching of a predicted plan against ground truth.

Ground truth and predictions live in unrelated coordinate frames (a tape-measured sketch vs a
capture's arbitrary origin). Both are gravity- and Manhattan-aligned, so they differ by a
rotation k*90 deg plus a translation. Everything here estimates that transform from geometry
alone and never reads names, so a prediction cannot be flattered by labelling.
"""
from __future__ import annotations

from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy.optimize import linear_sum_assignment, minimize
from shapely.geometry import Polygon
from shapely.ops import unary_union


def rot90(k: int) -> np.ndarray:
    c, s = [(1, 0), (0, 1), (-1, 0), (0, -1)][k % 4]
    return np.array([[c, -s], [s, c]], float)


class SE2:
    def __init__(self, k: int = 0, t=(0.0, 0.0)):
        self.k, self.R, self.t = k % 4, rot90(k), np.asarray(t, float)

    def __call__(self, p) -> np.ndarray:
        return np.asarray(p, float) @ self.R.T + self.t

    def dir(self, d) -> np.ndarray:
        return np.asarray(d, float) @ self.R.T


def _poly(pts) -> Polygon:
    p = Polygon(pts)
    return p if p.is_valid else p.buffer(0)


def iou(a: Polygon, b: Polygon) -> float:
    if a.is_empty or b.is_empty:
        return 0.0
    u = a.union(b).area
    return float(a.intersection(b).area / u) if u > 0 else 0.0


def align(pred: Polygon, gt: Polygon) -> Tuple[SE2, float]:
    """Best k*90 deg + translation mapping pred onto gt, by IoU."""
    c = align_all(pred, gt)
    return c[0] if c else (SE2(), 0.0)


def align_all(pred: Polygon, gt: Polygon, tie: float = 0.03) -> List[Tuple[SE2, float]]:
    """All k*90 deg alignments whose IoU is within `tie` of the best. A rectangle is symmetric under
    180 deg, so shape alone cannot orient it; callers break the tie with openings."""
    if pred.is_empty or gt.is_empty:
        return []
    cands = []
    gc = np.array(gt.centroid.coords[0])
    for k in range(4):
        R = rot90(k)
        P = np.array(pred.exterior.coords) @ R.T
        t0 = gc - np.array(_poly(P).centroid.coords[0])

        def cost(t):
            return 1.0 - iou(_poly(P + t), gt)

        r = minimize(cost, t0, method="Nelder-Mead", options={"xatol": 1e-3, "fatol": 1e-4, "maxiter": 200})
        cands.append((SE2(k, r.x), 1.0 - float(r.fun)))
    cands.sort(key=lambda c: -c[1])
    return [c for c in cands if c[1] >= cands[0][1] - tie]


def match_rooms(pred_rooms: List[Dict], gt_rooms: List[Dict], min_iou: float = 0.3):
    """Hungarian on (1 - per-room aligned IoU). Returns [(pi, gi, SE2, iou)], unmatched pred, unmatched gt."""
    if not pred_rooms or not gt_rooms:
        return [], list(range(len(pred_rooms))), list(range(len(gt_rooms)))
    C = np.ones((len(pred_rooms), len(gt_rooms)))
    T = {}
    for i, pr in enumerate(pred_rooms):
        pp = _poly(pr["polygon"])
        for j, gr in enumerate(gt_rooms):
            gp = _poly(gr["polygon"])
            # area ratio guards against a small room "fitting inside" a large one
            ar = min(pp.area, gp.area) / max(pp.area, gp.area, 1e-9)
            if ar < 0.25:
                continue
            cands = align_all(pp, gp)
            if not cands:
                continue
            tf = min(cands, key=lambda c: _opening_cost(pr, gr, c[0]))[0] if len(cands) > 1 else cands[0][0]
            s = max(c[1] for c in cands)
            T[(i, j)] = (tf, s)
            C[i, j] = 1.0 - s
    ri, ci = linear_sum_assignment(C)
    out = [(int(i), int(j), T[(i, j)][0], T[(i, j)][1]) for i, j in zip(ri, ci)
           if (i, j) in T and T[(i, j)][1] >= min_iou]
    mp = {i for i, _, _, _ in out}
    mg = {j for _, j, _, _ in out}
    return out, [i for i in range(len(pred_rooms)) if i not in mp], [j for j in range(len(gt_rooms)) if j not in mg]


def _opening_cost(pred_room: Dict, gt_room: Dict, tf: SE2) -> float:
    pairs, ph, ms = match_openings(pred_room, gt_room, tf)
    d = [float(np.linalg.norm(tf(opening_center(pred_room, o)) - np.asarray(g["center"], float)))
         for o, g in pairs if g.get("center") is not None]
    return len(ph) + len(ms) + sum(d)


def _type_ok(p: str, g: Optional[str]) -> bool:
    return g is None or p == g or {p, g} <= {"door", "passage"}


def gt_edges(poly: Sequence) -> List[Dict]:
    P = np.asarray(poly, float)
    out = []
    for k in range(len(P)):
        a, b = P[k], P[(k + 1) % len(P)]
        L = float(np.linalg.norm(b - a))
        if L > 1e-6:
            out.append({"index": k, "start": a, "end": b, "length": L})
    return out


def _seg_cost(a0, a1, b0, b1) -> Optional[float]:
    da, db = a1 - a0, b1 - b0
    La, Lb = np.linalg.norm(da), np.linalg.norm(db)
    ua, ub = da / La, db / Lb
    if abs(float(ua @ ub)) < 0.9:
        return None
    n = np.array([-ua[1], ua[0]])
    dist = abs(float((0.5 * (b0 + b1) - a0) @ n))
    if dist > 0.35:
        return None
    s = sorted([0.0, La])
    t = sorted([float((b0 - a0) @ ua), float((b1 - a0) @ ua)])
    ov = max(0.0, min(s[1], t[1]) - max(s[0], t[0]))
    frac = ov / max(La, Lb)
    if frac < 0.4:
        return None
    return dist + (1.0 - frac)


def match_walls(pred_room: Dict, gt_room: Dict, tf: SE2):
    ge = gt_edges(gt_room["polygon"])
    pw = pred_room["walls"]
    C = np.full((len(pw), len(ge)), 9.0)
    for i, w in enumerate(pw):
        a0, a1 = tf(w["start"]), tf(w["end"])
        for j, e in enumerate(ge):
            c = _seg_cost(a0, a1, e["start"], e["end"])
            if c is not None:
                C[i, j] = c
    pairs = []
    if len(pw) and len(ge):
        ri, ci = linear_sum_assignment(C)
        pairs = [(int(i), int(j)) for i, j in zip(ri, ci) if C[i, j] < 9.0]
    mp, mg = {i for i, _ in pairs}, {j for _, j in pairs}
    return ([(pw[i], ge[j]) for i, j in pairs], [pw[i] for i in range(len(pw)) if i not in mp],
            [ge[j] for j in range(len(ge)) if j not in mg])


def opening_center(pred_room: Dict, op: Dict) -> Optional[np.ndarray]:
    w = next((w for w in pred_room["walls"] if w["id"] == op.get("wall_id")), None)
    if w is None:
        return None
    a, b = np.asarray(w["start"], float), np.asarray(w["end"], float)
    u = (b - a) / max(np.linalg.norm(b - a), 1e-9)
    return a + u * op["center_offset"]["value"]


def match_openings(pred_room: Dict, gt_room: Dict, tf: SE2, max_dist: float = 0.5):
    """Position-based when GT has centres; width-based fallback for tape GT without positions."""
    po = pred_room.get("openings", [])
    go = gt_room.get("openings", [])
    if not po or not go:
        return [], list(po), list(go)
    C = np.full((len(po), len(go)), 9.0)
    for i, o in enumerate(po):
        c = opening_center(pred_room, o)
        for j, g in enumerate(go):
            if g.get("center") is not None and c is not None:
                d = float(np.linalg.norm(tf(c) - np.asarray(g["center"], float)))
                if d <= max_dist and _type_ok(o["type"], g.get("type")):
                    C[i, j] = d
            elif g.get("center") is None:
                dw = abs(o["width"]["value"] - g["width"])
                if dw <= 0.15 and _type_ok(o["type"], g.get("type")):
                    C[i, j] = dw
    ri, ci = linear_sum_assignment(C)
    pairs = [(int(i), int(j)) for i, j in zip(ri, ci) if C[i, j] < 9.0]
    mp, mg = {i for i, _ in pairs}, {j for _, j in pairs}
    return ([(po[i], go[j]) for i, j in pairs], [po[i] for i in range(len(po)) if i not in mp],
            [go[j] for j in range(len(go)) if j not in mg])


def footprint_area(rooms: List[Dict]) -> float:
    return float(unary_union([_poly(r["polygon"]) for r in rooms]).area) if rooms else 0.0


def overlaps(rooms: List[Dict], tol_m2: float = 0.05) -> List[Tuple[str, str, float]]:
    P = [_poly(r["polygon"]) for r in rooms]
    out = []
    for i in range(len(P)):
        for j in range(i + 1, len(P)):
            a = P[i].intersection(P[j]).area
            if a > tol_m2:
                out.append((rooms[i]["id"], rooms[j]["id"], round(float(a), 3)))
    return out
