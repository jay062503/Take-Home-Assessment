"""Room structure from a gravity- and Manhattan-aligned cloud.

Pipeline (same for every tier, parameters differ):
  1. global floor / ceiling levels from oriented horizontal surfaces
  2. room masks: ceiling footprint (LiDAR/video; door headers separate rooms) or visibility
     polygon from the capture station (photo tier)
  3. rectilinear polygon per room, each wall plane refined against its own surface points
  4. per-room floor and ceiling planes -> ceiling height
  5. openings from free-space evidence: rays that cross the wall plane (through evidence) vs
     returns on the plane (hit evidence)
Every scalar carries a 1-sigma built from a statistical term (plane-fit residual) and the tier's
systematic budget; calibration multipliers are applied later in uncertainty.calibrate.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import shapely
from scipy import ndimage
from shapely.geometry import Point, Polygon

from ..types import Measure, Opening, Room, Wall, combine_sigma
from .cloud import Cloud

N_EFF_CAP = 30  # depth errors are spatially correlated; never trust more than ~30 independent samples


@dataclass
class Grid:
    x0: float
    y0: float
    res: float
    W: int
    H: int

    @classmethod
    def around(cls, xy: np.ndarray, res: float, margin: float = 0.4) -> "Grid":
        lo = xy.min(axis=0) - margin
        hi = xy.max(axis=0) + margin
        W = int(np.ceil((hi[0] - lo[0]) / res)) + 1
        H = int(np.ceil((hi[1] - lo[1]) / res)) + 1
        return cls(float(lo[0]), float(lo[1]), res, W, H)

    def cells(self, xy: np.ndarray):
        ix = np.floor((xy[:, 0] - self.x0) / self.res).astype(int)
        iy = np.floor((xy[:, 1] - self.y0) / self.res).astype(int)
        ok = (ix >= 0) & (ix < self.W) & (iy >= 0) & (iy < self.H)
        return ix, iy, ok

    def to_xy(self, ij: np.ndarray) -> np.ndarray:
        return np.stack([self.x0 + (ij[:, 0] + 0.5) * self.res, self.y0 + (ij[:, 1] + 0.5) * self.res], 1)


# ---------------------------------------------------------------- levels
def _hist_peaks(z: np.ndarray, binw: float = 0.01, rel: float = 0.25):
    if len(z) == 0:
        return np.array([]), np.array([]), None
    lo, hi = z.min() - 0.05, z.max() + 0.05
    h, e = np.histogram(z, bins=max(3, int((hi - lo) / binw)))
    hs = ndimage.uniform_filter1d(h.astype(float), 3)
    pk = np.nonzero((hs >= ndimage.maximum_filter1d(hs, 7)) & (hs >= rel * hs.max()) & (hs > 0))[0]
    centers = (e[:-1] + e[1:]) / 2
    return centers[pk], hs[pk], (centers, hs)


def robust_level(z: np.ndarray, near: float, tol: float = 0.03) -> Tuple[float, float, int]:
    sel = z[np.abs(z - near) < tol]
    if len(sel) < 5:
        return near, 0.05, len(sel)
    for _ in range(2):
        m = np.median(sel)
        sel = sel[np.abs(sel - m) < tol]
    mad = 1.4826 * np.median(np.abs(sel - np.median(sel))) + 1e-4
    return float(np.median(sel)), float(mad), int(len(sel))


def global_levels(cloud: Cloud) -> Tuple[float, float]:
    n, z = cloud.nrm, cloud.pts[:, 2]
    ok = ~np.isnan(n[:, 0]) & ~cloud.far
    up = ok & (n[:, 2] > 0.9)
    dn = ok & (n[:, 2] < -0.9)
    fp, _, _ = _hist_peaks(z[up], rel=0.3)
    cp, _, _ = _hist_peaks(z[dn], rel=0.3)
    floor = float(fp.min()) if len(fp) else float(np.percentile(z[ok], 1))
    ceil = float(cp.max()) if len(cp) else float(np.percentile(z[ok], 99))
    return floor, ceil


# ------------------------------------------------------------ room masks
def _count_grid(grid: Grid, xy: np.ndarray) -> np.ndarray:
    ix, iy, inb = grid.cells(xy)
    cnt = np.zeros((grid.H, grid.W), np.int32)
    np.add.at(cnt, (iy[inb], ix[inb]), 1)
    return cnt


def ceiling_room_masks(cloud: Cloud, floor_z: float, ceil_z: float, res: float, min_area: float):
    """Rooms = geodesic flood of ceiling seeds through free cells bounded by the wall band.

    The wall band (1 m above floor .. just below ceiling) includes door headers, so doorways stay
    closed in 2D while the ceiling seeds (which rarely cover the area straight overhead) only need
    to touch each room somewhere.
    """
    n, P = cloud.nrm, cloud.pts
    valid = ~np.isnan(n[:, 0]) & ~cloud.far
    ceil_pts = valid & (n[:, 2] < -0.85) & (P[:, 2] > floor_z + 2.08)
    floor_pts = valid & (n[:, 2] > 0.85) & (np.abs(P[:, 2] - floor_z) < 0.04)
    # Upper band only (above door heads, below ceiling): headers close doorways in 2D, and
    # wardrobes/shelves (rarely above ~2 m) do not carve notches out of the room.
    wall_pts = valid & (np.abs(n[:, 2]) < 0.3) & (P[:, 2] > floor_z + 2.06) & (P[:, 2] < ceil_z - 0.06)
    if wall_pts.sum() < 2000:
        wall_pts = valid & (np.abs(n[:, 2]) < 0.3) & (P[:, 2] > floor_z + 1.0) & (P[:, 2] < ceil_z - 0.12)
    grid = Grid.around(P[ceil_pts | floor_pts, :2], res)
    occ = _count_grid(grid, P[wall_pts, :2]) >= 2
    occ = ndimage.binary_closing(occ, structure=np.ones((3, 3)), iterations=1)
    # the gap between the two faces of an interior wall is free space in 2D; dilating by ~7 cm
    # fills it so floods cannot leak along wall cavities through small gaps in one face
    grow = max(1, int(round(0.07 / res)))
    walls = ndimage.binary_dilation(occ, iterations=grow)
    obs = (_count_grid(grid, P[ceil_pts, :2]) > 0) | (_count_grid(grid, P[floor_pts, :2]) > 0)
    obs = ndimage.binary_closing(obs, iterations=3)
    enclosed = ndimage.binary_fill_holes(walls) & ~walls
    obs_dom = ndimage.binary_dilation(obs, iterations=2) & ~walls
    # observed floor/ceiling keeps rooms whose envelope has gaps (sparse exterior walls) in the domain
    domain = (enclosed | obs_dom) & ~walls

    seeds = _count_grid(grid, P[ceil_pts, :2]) > 0
    seeds = ndimage.binary_opening(ndimage.binary_closing(seeds, iterations=1), iterations=1) & domain
    seeds = ndimage.binary_erosion(seeds, iterations=2)
    lab, nl = ndimage.label(seeds)
    sizes = ndimage.sum(np.ones_like(lab), lab, index=np.arange(1, nl + 1)) * res * res
    keep = np.zeros(nl + 1, bool)
    keep[1:] = sizes >= 0.3
    lab = np.where(keep[lab], lab, 0)
    lab = _geodesic_flood(lab, domain)
    lab = _merge_touching(lab)
    # enclosed free regions no ceiling seed reached (narrow halls, sparse ceiling coverage)
    rest, nr = ndimage.label(domain & (lab == 0))
    nxt = int(lab.max()) + 1
    for L in range(1, nr + 1):
        m = rest == L
        if m.sum() * res * res >= min_area and not (m[0].any() or m[-1].any() or m[:, 0].any() or m[:, -1].any()):
            lab[m] = nxt
            nxt += 1
    rooms = []
    for L in np.unique(lab):
        if L == 0:
            continue
        m = lab == L
        m = ndimage.binary_dilation(m, iterations=grow) & ~occ   # give back the wall dilation margin
        m = ndimage.binary_fill_holes(m)
        if m.sum() * res * res >= min_area:
            rooms.append(m)
    return grid, rooms


def _merge_touching(lab: np.ndarray, min_contact: int = 8) -> np.ndarray:
    """Seeds of one room flood until they meet in free space; rooms separated by a wall never touch."""
    parent = {int(L): int(L) for L in np.unique(lab) if L}

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    pairs = {}
    for a, b in ((lab[:, :-1], lab[:, 1:]), (lab[:-1, :], lab[1:, :])):
        m = (a > 0) & (b > 0) & (a != b)
        for x, y in zip(a[m], b[m]):
            k = (min(x, y), max(x, y))
            pairs[k] = pairs.get(k, 0) + 1
    for (x, y), cnt in pairs.items():
        if cnt >= min_contact:
            parent[find(int(x))] = find(int(y))
    out = lab.copy()
    for L in parent:
        out[lab == L] = find(L)
    return out


def _geodesic_flood(lab: np.ndarray, domain: np.ndarray, max_iter: int = 2000) -> np.ndarray:
    lab = lab.copy()
    for _ in range(max_iter):
        grown = ndimage.grey_dilation(lab, size=(3, 3))
        new = (lab == 0) & domain & (grown > 0)
        if not new.any():
            break
        lab[new] = grown[new]
    return lab


def visibility_room_mask(cloud: Cloud, floor_z: float, ceil_z: float, station: np.ndarray, res: float,
                         n_bins: int = 720):
    """Photo tier: per angular bin around the station, the nearest wall-band return bounds the room."""
    n, P = cloud.nrm, cloud.pts
    band = ~cloud.far & (P[:, 2] > floor_z + 0.9) & (P[:, 2] < ceil_z - 0.15)
    band &= np.isnan(n[:, 0]) | (np.abs(n[:, 2]) < 0.4)
    d = P[band, :2] - station[None, :2]
    ang = np.arctan2(d[:, 1], d[:, 0])
    r = np.linalg.norm(d, axis=1)
    b = ((ang + np.pi) / (2 * np.pi) * n_bins).astype(int) % n_bins
    rad = np.full(n_bins, np.nan)
    order = np.argsort(r)
    seen = np.zeros(n_bins, bool)
    # robust nearest: 10th percentile of ranges per bin (rejects isolated noise in front of the wall)
    for k in range(n_bins):
        rk = r[b == k]
        if len(rk) >= 3:
            rad[k] = np.percentile(rk, 60)
    valid = ~np.isnan(rad)
    if valid.sum() < n_bins * 0.3:
        return None, None
    idx = np.arange(n_bins)
    rad = np.interp(idx, idx[valid], rad[valid], period=n_bins)
    rad = ndimage.median_filter(rad, size=9, mode="wrap")
    th = (idx + 0.5) / n_bins * 2 * np.pi - np.pi
    poly_xy = station[None, :2] + np.stack([np.cos(th), np.sin(th)], 1) * rad[:, None]
    grid = Grid.around(poly_xy, res)
    ij = np.stack([(poly_xy[:, 0] - grid.x0) / res, (poly_xy[:, 1] - grid.y0) / res], 1)
    mask = np.zeros((grid.H, grid.W), np.uint8)
    cv2.fillPoly(mask, [np.round(ij).astype(np.int32)], 1)
    return grid, mask.astype(bool)


# -------------------------------------------------------- rectilinear walls
def rectilinear_polygon(mask: np.ndarray, grid: Grid, min_seg: float = 0.22, eps: float = 0.05):
    cnts, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    c = max(cnts, key=cv2.contourArea)[:, 0, :].astype(float)
    approx = cv2.approxPolyDP(c.astype(np.float32), max(1.5, eps / grid.res), True)[:, 0, :]
    pts = grid.to_xy(approx.astype(float))
    segs = []
    for i in range(len(pts)):
        a, b = pts[i], pts[(i + 1) % len(pts)]
        d = b - a
        L = float(np.hypot(*d))
        if L < 1e-6:
            continue
        if abs(d[0]) >= abs(d[1]):
            segs.append(["H", (a[1] + b[1]) / 2, L])
        else:
            segs.append(["V", (a[0] + b[0]) / 2, L])

    def merge(s):
        changed = True
        while changed and len(s) > 1:
            changed = False
            for i in range(len(s)):
                j = (i + 1) % len(s)
                if s[i][0] == s[j][0] and i != j:
                    L = s[i][2] + s[j][2]
                    s[i] = [s[i][0], (s[i][1] * s[i][2] + s[j][1] * s[j][2]) / L, L]
                    del s[j]
                    changed = True
                    break
        return s

    segs = merge(segs)
    while len(segs) > 4:
        k = int(np.argmin([s[2] for s in segs]))
        if segs[k][2] >= min_seg:
            break
        del segs[k]
        segs = merge(segs)
    if len(segs) < 4 or len(segs) % 2:
        ys, xs = np.nonzero(mask)
        xy = grid.to_xy(np.stack([xs, ys], 1).astype(float))
        x0, y0 = xy.min(0)
        x1, y1 = xy.max(0)
        segs = [["H", y0, x1 - x0], ["V", x1, y1 - y0], ["H", y1, x1 - x0], ["V", x0, y1 - y0]]
    return [(s[0], float(s[1])) for s in segs]


def seg_vertices(segs) -> np.ndarray:
    v = []
    for i in range(len(segs)):
        a, b = segs[i], segs[(i + 1) % len(segs)]
        v.append((b[1], a[1]) if a[0] == "H" else (a[1], b[1]))
    return np.asarray(v, float)


def refine_walls(segs, cloud: Cloud, floor_z: float, ceil_z: float, tier_cfg: Dict, room_id: str,
                 search: float = 0.30) -> List[Wall]:
    P, n = cloud.pts, cloud.nrm
    tol = float(tier_cfg["wall_tol"])
    sys_plane = float(tier_cfg["sys_plane"])
    base = ~cloud.far & ~np.isnan(n[:, 0]) & (P[:, 2] > floor_z + 0.15) & (P[:, 2] < ceil_z - 0.08)
    coords = [s[1] for s in segs]
    sigmas = [None] * len(segs)
    supports = [0] * len(segs)
    obsfrac = [0.0] * len(segs)
    for it in range(2):
        poly = Polygon(seg_vertices([(segs[i][0], coords[i]) for i in range(len(segs))]))
        if not poly.is_valid:
            poly = poly.buffer(0)
        for i, (o, _) in enumerate(segs):
            a = 0 if o == "V" else 1                     # normal axis
            prev_c, next_c = coords[i - 1], coords[(i + 1) % len(segs)]
            lo, hi = sorted([prev_c, next_c])
            c = coords[i]
            mid = np.zeros(2)
            mid[a] = c
            mid[1 - a] = (lo + hi) / 2
            probe = mid.copy()
            probe[a] += 0.05
            inward = 1 if poly.contains(Point(probe)) else -1
            m = base & (np.abs(P[:, a] - c) < search) & (P[:, 1 - a] > lo + 0.08) & (P[:, 1 - a] < hi - 0.08)
            m &= (n[:, a] * inward > 0.8)
            x = P[m, a]
            if len(x) < 25:
                sigmas[i] = combine_sigma(tier_cfg["grid"], 0.03)
                supports[i] = int(len(x))
                continue
            h, e = np.histogram(x, bins=np.arange(c - search, c + search + 0.005, 0.005))
            hs = ndimage.uniform_filter1d(h.astype(float), 3)
            pk = np.nonzero((hs >= ndimage.maximum_filter1d(hs, 5)) & (hs >= 0.25 * hs.max()))[0]
            centers = (e[:-1] + e[1:]) / 2
            cand = centers[pk]
            outer = cand[np.argmin(inward * cand)]      # outermost substantial surface = the wall
            ref, mad, cnt = robust_level(x, outer, tol)
            coords[i] = ref
            sigmas[i] = combine_sigma(mad / np.sqrt(min(cnt, N_EFF_CAP)), sys_plane)
            supports[i] = cnt
            sel = x[np.abs(x - ref) < tol]
            s_along = P[m, 1 - a][np.abs(x - ref) < tol]
            if len(s_along):
                occ = np.unique(np.floor((s_along - lo) / 0.05))
                obsfrac[i] = float(min(1.0, len(occ) * 0.05 / max(hi - lo, 1e-6)))
    verts = seg_vertices([(segs[i][0], coords[i]) for i in range(len(segs))])
    poly = Polygon(verts)
    walls = []
    k = len(segs)
    for i, (o, _) in enumerate(segs):
        a = 0 if o == "V" else 1
        start, end = verts[i - 1], verts[i]
        mid = (start + end) / 2
        probe = mid.copy()
        probe[a] += 0.05
        inward = 1 if poly.contains(Point(probe)) else -1
        walls.append(Wall(id=f"{room_id}/wall_{i}", start=start, end=end, axis=a,
                          coord=Measure(coords[i], sigmas[i]), inward=inward, support=supports[i],
                          observed_fraction=obsfrac[i]))
    for i, w in enumerate(walls):
        L = float(np.linalg.norm(w.end - w.start))
        sp, sn = walls[i - 1].coord.sigma, walls[(i + 1) % k].coord.sigma
        w.length = Measure(L, combine_sigma(sp, sn, float(tier_cfg["sys_scale"]) * L))
    return walls


# -------------------------------------------------------- heights per room
def room_levels(poly: Polygon, cloud: Cloud, floor_g: float, tier_cfg: Dict):
    P, n = cloud.pts, cloud.nrm
    inner = poly.buffer(-0.12)
    if inner.is_empty:
        inner = poly
    ok = ~cloud.far & ~np.isnan(n[:, 0])
    cand = ok & (np.abs(n[:, 2]) > 0.9)
    idx = np.nonzero(cand)[0]
    inside = shapely.contains_xy(inner, P[idx, 0], P[idx, 1])
    idx = idx[inside]
    z = P[idx, 2]
    up = n[idx, 2] > 0
    # beds and sofas can out-vote the visible floor, so peaks are only a few % of the max and
    # the floor must lie near the capture-wide floor level
    fp, fh, _ = _hist_peaks(z[up], 0.005, 0.05)
    cp, ch, _ = _hist_peaks(z[~up], 0.005, 0.05)
    tol = max(0.02, float(tier_cfg["wall_tol"]) * 0.7)
    near_floor = fp[(np.abs(fp - floor_g) < 0.12) & (fh >= 20)] if len(fp) else fp
    f_near = float(near_floor.min()) if len(near_floor) else floor_g
    high = cp[(cp > f_near + 2.05) & (ch >= 20)] if len(cp) else cp
    c_near = float(high.max()) if len(high) else (float(cp.max()) if len(cp) else floor_g + 2.5)
    fz, fmad, fn = _plane_level(P[idx][up], f_near, tol, poly)
    cz, cmad, cn = _plane_level(P[idx][~up], c_near, tol, poly)
    sh = float(tier_cfg["sys_height"])
    sf = combine_sigma(fmad / np.sqrt(max(1, min(fn, N_EFF_CAP))), sh / np.sqrt(2))
    sc = combine_sigma(cmad / np.sqrt(max(1, min(cn, N_EFF_CAP))), sh / np.sqrt(2))
    if fn < 20:
        sf = combine_sigma(sf, 0.05)
    if cn < 20:
        sc = combine_sigma(sc, 0.05)
    H = cz - fz
    hs = combine_sigma(sf, sc, float(tier_cfg["sys_scale"]) * H)
    return Measure(fz, sf), Measure(cz, sc), Measure(H, hs), {"floor_pts": fn, "ceiling_pts": cn}


def _plane_level(Q: np.ndarray, near: float, tol: float, poly: Polygon):
    """Fit z = a x + b y + c to points near a level; evaluate at the room centroid (handles tilt)."""
    if len(Q) == 0:
        return near, 0.05, 0
    sel = Q[np.abs(Q[:, 2] - near) < tol]
    if len(sel) < 10:
        return near, 0.05, len(sel)
    cx, cy = poly.centroid.x, poly.centroid.y
    for _ in range(3):
        A = np.c_[sel[:, 0] - cx, sel[:, 1] - cy, np.ones(len(sel))]
        coef, *_ = np.linalg.lstsq(A, sel[:, 2], rcond=None)
        r = sel[:, 2] - A @ coef
        mad = 1.4826 * np.median(np.abs(r - np.median(r))) + 1e-4
        keep = np.abs(r) < max(3 * mad, 0.004)
        sel = sel[keep]
        if len(sel) < 10:
            break
    return float(coef[2]), float(mad), int(len(sel))


# ---------------------------------------------------------------- openings
def detect_openings(room: Room, cloud: Cloud, cam_inside: np.ndarray, cfg: Dict, tier_cfg: Dict) -> List[Opening]:
    g = cfg["geometry"]
    P = cloud.pts
    C = cloud.cams[cloud.fidx]
    inside = cam_inside[cloud.fidx]
    tol = float(tier_cfg["wall_tol"])
    fz, cz = room.floor_z.value, room.ceiling_z.value
    ds, dz = 0.02, 0.04
    out = []
    k = 0
    for wi, w in enumerate(room.walls):
        a = w.axis
        c = w.coord.value
        s_lo, s_hi = sorted([w.start[1 - a], w.end[1 - a]])
        if s_hi - s_lo < 0.5:
            continue
        nu_o = w.inward * (C[:, a] - c)
        nu_p = w.inward * (P[:, a] - c)
        thru = inside & (nu_o > 0.1) & (nu_p < -0.04)
        hit = inside & ~cloud.far & (np.abs(nu_p) < tol)
        ns = int(np.ceil((s_hi - s_lo) / ds))
        nz = int(np.ceil((cz - fz) / dz))
        if ns < 3 or nz < 3:
            continue
        H_hit = np.zeros((nz, ns), np.int32)
        H_thr = np.zeros((nz, ns), np.int32)
        ii = np.nonzero(hit)[0]
        si = ((P[ii, 1 - a] - s_lo) / ds).astype(int)
        zi = ((P[ii, 2] - fz) / dz).astype(int)
        ok = (si >= 0) & (si < ns) & (zi >= 0) & (zi < nz)
        np.add.at(H_hit, (zi[ok], si[ok]), 1)
        ii = np.nonzero(thru)[0]
        lam = nu_o[ii] / (nu_o[ii] - nu_p[ii])
        X = C[ii] + lam[:, None] * (P[ii] - C[ii])
        si = ((X[:, 1 - a] - s_lo) / ds).astype(int)
        zi = ((X[:, 2] - fz) / dz).astype(int)
        ok = (si >= 0) & (si < ns) & (zi >= 0) & (zi < nz)
        np.add.at(H_thr, (zi[ok], si[ok]), 1)
        hs, hz = P[hit, 1 - a], P[hit, 2]
        xs, xz = X[:, 1 - a], X[:, 2]
        rev = inside & ~cloud.far & (nu_p < tol) & (nu_p > -0.25) & (np.abs(cloud.nrm[:, 1 - a]) > 0.8)
        rs, rz, rn = P[rev, 1 - a], P[rev, 2], np.sign(cloud.nrm[rev, 1 - a])
        open_ = (H_thr >= 2) & (H_thr >= 2 * H_hit + 1)
        open_ = ndimage.binary_closing(open_, structure=np.ones((3, 3)), iterations=1)
        open_ = ndimage.binary_opening(open_, structure=np.ones((3, 3)), iterations=1)
        lab, nl = ndimage.label(open_)
        for L in range(1, nl + 1):
            comp = lab == L
            rows = np.nonzero(comp.any(axis=1))[0]
            cols = np.nonzero(comp.any(axis=0))[0]
            bottom = rows.min() * dz
            top = (rows.max() + 1) * dz
            hgt = top - bottom
            if hgt < 0.3:
                continue
            band = comp[int(rows.min() + 0.15 * len(rows)):int(rows.max() - 0.15 * len(rows)) + 1]
            frac = band.mean(axis=0) if len(band) else comp.mean(axis=0)
            run = _longest_run(frac > 0.5)
            if run is None:
                continue
            c0, c1 = run
            width = (c1 - c0 + 1) * ds
            s0 = s_lo + c0 * ds
            s1 = s_lo + (c1 + 1) * ds
            if g.get("jamb_refine"):
                s0, s1 = _refine_jambs(hs, hz, xs, xz, rs, rz, rn, s0, s1, fz + bottom, fz + top)
                width = s1 - s0
            if width < g["opening_min_width"] or width > g["opening_max_width"]:
                continue
            if bottom > g["door_max_bottom"] and top >= g["door_min_top"]:
                # lower part never observed through: a door if the floor continues past the wall
                # plane across the gap and nothing solid was seen below the gap
                below = H_hit[: rows.min(), c0:c1 + 1]
                solid_below = (below >= 2).mean() if below.size else 0.0
                fl = ~cloud.far & (np.abs(P[:, 2] - fz) < 0.03) & (nu_p < -0.05) & (nu_p > -0.40)
                fl &= (P[:, 1 - a] > s0) & (P[:, 1 - a] < s1)
                if solid_below < 0.15 and fl.sum() >= 10:
                    bottom = 0.0
                    hgt = top
            if bottom <= g["door_max_bottom"] and top >= g["door_min_top"]:
                typ = "passage" if top >= (cz - fz) - 0.12 else "door"
            elif bottom >= g["window_min_sill"]:
                typ = "window"
            else:
                continue
            n_thr = int(H_thr[comp].sum())
            n_far = 0
            sig_edge = combine_sigma(ds / np.sqrt(3), float(tier_cfg["sys_plane"]))
            sw = combine_sigma(sig_edge, sig_edge, float(tier_cfg["sys_scale"]) * width)
            mid = s0 + (s1 - s0) / 2
            if w.start[1 - a] > w.end[1 - a]:
                off = w.start[1 - a] - mid
            else:
                off = mid - w.start[1 - a]
            out.append(Opening(id=f"{room.id}/opening_{k}", type=typ, wall_index=wi,
                               center_offset=Measure(off, sig_edge), width=Measure(width, sw),
                               height=Measure(hgt, combine_sigma(dz, float(tier_cfg["sys_scale"]) * hgt)),
                               sill=Measure(bottom, combine_sigma(dz / 2, float(tier_cfg["sys_plane"]))),
                               evidence={"through_points": n_thr, "s0": s0, "s1": s1}))
            k += 1
    return out


def _longest_run(b: np.ndarray):
    best, cur, start = None, 0, 0
    bl = 0
    for i, v in enumerate(b):
        if v:
            if cur == 0:
                start = i
            cur += 1
            if cur > bl:
                bl, best = cur, (start, i)
        else:
            cur = 0
    return best


def _refine_jambs(hs, hz, xs, xz, rs, rz, rn, s0, s1, z0, z1, win=0.06, step=0.005, min_pts=8):
    """Sub-cell jamb positions from raw evidence instead of the cell vote.

    Primary: the jamb reveal. Points whose normal runs along the wall and faces into the opening
    lie on the jamb surface itself, so their median along-wall position is the edge, which is also
    where a tape measure reads "between the jambs". Through-rays cannot measure this: rays grazing
    the jamb are shadowed by the reveal, so crossings stop short of the edge.
    Fallback (no reveal seen, e.g. a cased opening viewed head-on): the decision stump separating
    wall-plane hits from through-ray crossings, taking the wall-side end of the tie gap.
    """
    h = z1 - z0
    lo, hi = z0 + 0.15 * h, z1 - 0.15 * h
    hm = (hz > lo) & (hz < hi)
    xm = (xz > lo) & (xz < hi)
    rm = (rz > lo) & (rz < hi)

    def edge(e, side):
        # side=+1: opening lies at s > edge (left jamb, reveal faces +s); side=-1: the mirror case
        R = rs[rm & (rn == side) & (np.abs(rs - e) < win)]
        if len(R) >= min_pts:
            return float(np.median(R))
        H = hs[hm & (np.abs(hs - e) < win)]
        X = xs[xm & (np.abs(xs - e) < win)]
        if len(H) < min_pts or len(X) < min_pts:
            return e
        ts = np.arange(e - win, e + win + 1e-9, step)
        if side > 0:
            sc = (H[None, :] < ts[:, None]).mean(1) + (X[None, :] >= ts[:, None]).mean(1)
        else:
            sc = (H[None, :] > ts[:, None]).mean(1) + (X[None, :] <= ts[:, None]).mean(1)
        best = ts[sc >= sc.max() - 1e-9]
        return float(best.min() if side > 0 else best.max())

    n0, n1 = edge(s0, +1), edge(s1, -1)
    if n1 - n0 < 0.5 * (s1 - s0):
        return s0, s1
    return n0, n1


def opening_world_center(room: Room, op: Opening) -> np.ndarray:
    w = room.walls[op.wall_index]
    d = w.end - w.start
    L = np.linalg.norm(d)
    return w.start + d / max(L, 1e-9) * op.center_offset.value
