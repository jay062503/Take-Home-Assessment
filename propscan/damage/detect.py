"""Per-surface damage regions with class and metric extent.

Keyframe pixels are back-projected with their depth onto the reconstructed structural surfaces
(walls, ceiling; floor is excluded - plank seams and rugs dominate the false positives) and
accumulated into a metric orthophoto per surface (CIE Lab mean per cell + observation count).
Furniture in front of a wall is >tol away from the wall plane and never lands on the wall map.

Detection runs on the orthophoto against the surface's own robust median colour, so the extent
is metric by construction:
  water_stain : darker + yellower than the surface (dL < -6, db > +5), blob-shaped
  mold        : clustered dark speckle (dense dL < -30 cells)
  crack       : thin, elongated, *jagged* dark structure (straight axis-aligned lines are frames,
                skirting and shadows and are rejected)
Rectangular high-contrast regions (pictures, switches, vents) are rejected.

The optional `owlv2` backend (open-vocabulary detector, needs `.[ml]`) proposes image regions that
are then projected the same way; the heuristic backend is the default because it is deterministic
and runs without weights. It is tuned on synthetic data: see docs/TECHNICAL_REPORT.md for its
known real-world failure modes.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional

import cv2
import numpy as np
import shapely
from scipy import ndimage
from shapely.geometry import Polygon

from ..types import FrameSet, Measure, Room, combine_sigma


@dataclass
class SurfaceMap:
    sid: str
    kind: str                  # wall | ceiling
    room: str
    res: float
    u0: float
    v0: float
    nu: int
    nv: int
    lab_sum: np.ndarray = None
    cnt: np.ndarray = None
    meta: Dict = field(default_factory=dict)

    def __post_init__(self):
        self.lab_sum = np.zeros((self.nv, self.nu, 3), np.float64)
        self.cnt = np.zeros((self.nv, self.nu), np.int32)

    def add(self, u, v, lab):
        iu = ((u - self.u0) / self.res).astype(int)
        iv = ((v - self.v0) / self.res).astype(int)
        ok = (iu >= 0) & (iu < self.nu) & (iv >= 0) & (iv < self.nv)
        np.add.at(self.cnt, (iv[ok], iu[ok]), 1)
        for c in range(3):
            np.add.at(self.lab_sum[..., c], (iv[ok], iu[ok]), lab[ok, c])

    def mean(self):
        with np.errstate(invalid="ignore", divide="ignore"):
            return self.lab_sum / self.cnt[..., None]

    def filled(self, k: int = 5):
        """Normalised convolution: fills isolated unobserved cells from observed neighbours."""
        c = ndimage.uniform_filter(self.cnt.astype(float), k)
        s = np.stack([ndimage.uniform_filter(self.lab_sum[..., i], k) for i in range(3)], -1)
        own = self.cnt > 0
        with np.errstate(invalid="ignore", divide="ignore"):
            mean = np.where(own[..., None], self.lab_sum / np.maximum(self.cnt, 1)[..., None], s / c[..., None])
        obs = own | (c * k * k >= 4)
        return mean, obs


def build_surface_maps(rooms: List[Room], res: float) -> List[SurfaceMap]:
    maps = []
    for r in rooms:
        fz, cz = r.floor_z.value, r.ceiling_z.value
        for w in r.walls:
            L = float(np.linalg.norm(w.end - w.start))
            if L < 0.3:
                continue
            m = SurfaceMap(w.id, "wall", r.id, res, 0.0, 0.0, int(np.ceil(L / res)), int(np.ceil((cz - fz) / res)))
            m.meta = {"wall": w, "fz": fz, "cz": cz, "L": L}
            maps.append(m)
        x0, y0, x1, y1 = Polygon(r.polygon).bounds
        m = SurfaceMap(f"{r.id}/ceiling", "ceiling", r.id, res, x0, y0, int(np.ceil((x1 - x0) / res)), int(np.ceil((y1 - y0) / res)))
        m.meta = {"cz": cz, "poly": Polygon(r.polygon)}
        maps.append(m)
    return maps


def project_frames(fs: FrameSet, maps: List[SurfaceMap], stride_frames: int, tol: float, pix_stride: int = 2):
    frames = fs.frames[::max(1, stride_frames)] if fs.tier != "photo" else fs.frames
    for fr in frames:
        if fr.rgb is None or fr.depth is None:
            continue
        H, W = fr.rgb.shape[:2]
        d = cv2.resize(fr.depth, (W, H), interpolation=cv2.INTER_NEAREST)
        lab = cv2.cvtColor(fr.rgb, cv2.COLOR_RGB2LAB).astype(np.float32)
        u, v = np.meshgrid(np.arange(0, W, pix_stride), np.arange(0, H, pix_stride))
        dd = d[v, u].ravel()
        ok = (dd > 0.2) & (dd < 5.0)
        uu, vv, dd = u.ravel()[ok], v.ravel()[ok], dd[ok]
        K = fr.K_rgb
        Pc = np.stack([(uu - K[0, 2]) / K[0, 0] * dd, (vv - K[1, 2]) / K[1, 1] * dd, dd], 1)
        P = Pc @ fr.T_wc[:3, :3].T + fr.T_wc[:3, 3]
        L = lab[vv, uu]
        for m in maps:
            if m.kind == "wall":
                w = m.meta["wall"]
                a = w.axis
                sel = np.abs(P[:, a] - w.coord.value) < tol
                if not sel.any():
                    continue
                dirv = (w.end - w.start) / m.meta["L"]
                s = (P[sel, :2] - w.start) @ dirv
                z = P[sel, 2] - m.meta["fz"]
                keep = (s >= 0) & (s <= m.meta["L"]) & (z >= 0) & (z <= m.meta["cz"] - m.meta["fz"])
                m.add(s[keep], z[keep], L[sel][keep])
            else:
                sel = np.abs(P[:, 2] - m.meta["cz"]) < tol
                if not sel.any():
                    continue
                q = P[sel]
                inside = shapely.contains_xy(m.meta["poly"], q[:, 0], q[:, 1])
                m.add(q[inside, 0], q[inside, 1], L[sel][inside])


def _components(mask: np.ndarray, min_cells: int):
    lab, n = ndimage.label(mask)
    for i in range(1, n + 1):
        cm = lab == i
        if cm.sum() >= min_cells:
            yield cm


def _rectangularity(cm: np.ndarray) -> float:
    ys, xs = np.nonzero(cm)
    pts = np.stack([xs, ys], 1).astype(np.float32)
    if len(pts) < 5:
        return 1.0
    (cx, cy), (w, h), ang = cv2.minAreaRect(pts)
    return float(cm.sum() / max((w + 1) * (h + 1), 1))


def _line_stats(cm: np.ndarray):
    ys, xs = np.nonzero(cm)
    pts = np.stack([xs, ys], 1).astype(float)
    c = pts.mean(0)
    U, S, Vt = np.linalg.svd(pts - c, full_matrices=False)
    length = 4 * S[0] / np.sqrt(len(pts)) if len(pts) > 1 else 0
    resid = np.abs((pts - c) @ Vt[1])
    ang = np.degrees(np.arctan2(Vt[0, 1], Vt[0, 0])) % 180
    axis_dev = min(ang % 90, 90 - ang % 90)
    return length, float(np.percentile(resid, 90)), axis_dev


def detect_on_maps(maps: List[SurfaceMap], cfg: Dict, tier: str, min_obs: int = 1) -> List[Dict]:
    dc = cfg["damage"]
    out = []
    k = 0
    for m in maps:
        mean, obs = m.filled()
        if obs.sum() < 50:
            continue
        Lm = np.where(obs, mean[..., 0], np.nan)
        bm = np.where(obs, mean[..., 2], np.nan)
        # fill unobserved cells with the surface median so filters do not invent edges
        Lmed, bmed = np.nanmedian(Lm), np.nanmedian(bm)
        Lf = np.where(obs, Lm, Lmed)
        bf = np.where(obs, bm, bmed)
        Ls = ndimage.uniform_filter(Lf, 3)
        bs = ndimage.uniform_filter(bf, 3)
        # local background (larger than any stain we report as one region)
        win = max(5, int(1.2 / m.res))
        Lbg = ndimage.median_filter(Lf, size=win)
        bbg = ndimage.median_filter(bf, size=win)
        Lbg = np.where(np.abs(Lbg - Lmed) > 25, Lmed, Lbg)
        dL, db = Ls - Lbg, bs - bbg
        dL_raw = Lf - Lbg
        min_cells = max(4, int(dc["min_area_m2"] / (m.res * m.res)))
        cell_a = m.res * m.res

        def emit(cls, cm, conf, extra=None):
            nonlocal k
            ys, xs = np.nonzero(cm)
            u0, u1 = m.u0 + xs.min() * m.res, m.u0 + (xs.max() + 1) * m.res
            v0, v1 = m.v0 + ys.min() * m.res, m.v0 + (ys.max() + 1) * m.res
            area = cm.sum() * cell_a
            sig_edge = combine_sigma(m.res / 2, float(cfg["tiers"][tier]["sys_plane"]))
            rec = {"id": f"dmg_{k}", "surface_id": m.sid, "surface_kind": m.kind, "room_id": m.room, "class": cls,
                   "bbox_surface": {"u0": round(u0, 3), "u1": round(u1, 3), "v0": round(v0, 3), "v1": round(v1, 3)},
                   "extent_w": Measure(u1 - u0, combine_sigma(sig_edge, sig_edge)),
                   "extent_h": Measure(v1 - v0, combine_sigma(sig_edge, sig_edge)),
                   "area": Measure(area, combine_sigma(np.sqrt(cm.sum()) * 2 * m.res * m.res * 2, area * 2 * float(cfg["tiers"][tier]["sys_scale"])), "m2"),
                   "confidence": round(float(np.clip(conf, 0, 1)), 3),
                   "observed_cells": int(m.cnt[cm].sum())}
            if extra:
                rec.update(extra)
            out.append(rec)
            k += 1

        amean = np.where(obs, mean[..., 1], np.nanmedian(np.where(obs, mean[..., 1], np.nan)))
        abg = ndimage.median_filter(amean, size=win)
        chroma = np.hypot(amean - abg, bf - bbg)
        # objects on the surface (pictures, switches, vents): rectangular blocks of strong change
        objm = np.zeros_like(obs)
        strong = obs & ((np.abs(dL_raw) > 18) | (chroma > 14))
        for cm in _components(ndimage.binary_closing(strong, iterations=1), max(4, int(0.02 / cell_a))):
            if _rectangularity(cm) > 0.75 or _axis_run_fraction(cm) > 0.5:
                objm |= ndimage.binary_fill_holes(cm)
        objm = ndimage.binary_dilation(objm, iterations=2)
        edge = ~ndimage.binary_erosion(obs, iterations=1)   # unobserved border / wall ends
        valid = obs & ~objm & ~edge

        stain = valid & (dL < -6) & (db > 5)
        stain = ndimage.binary_opening(ndimage.binary_closing(stain, iterations=2), iterations=1)
        for cm in _components(stain, min_cells):
            if _rectangularity(cm) > 0.88:
                continue
            conf = min(1.0, (-dL[cm].mean() / 20 + db[cm].mean() / 15) / 2)
            emit("water_stain", cm, conf)
        # mold = dark speckle: dark against its *immediate* neighbourhood, densely clustered
        Lloc = ndimage.uniform_filter(Lf, size=5)
        speck = valid & (Lf - Lloc < -12) & (dL_raw < -25)
        dens = ndimage.uniform_filter(speck.astype(float), size=max(3, int(0.10 / m.res)))
        mold = valid & (dens > 0.18) & ~stain
        mold = ndimage.binary_closing(mold, iterations=2)
        for cm in _components(mold, min_cells):
            if _rectangularity(cm) > 0.9:
                continue
            emit("mold", cm, float(min(1.0, dens[cm].mean() * 2.5)))
        if m.kind == "wall":
            Lmed5 = ndimage.median_filter(Lf, size=5)
            thin = valid & (Lf - Lmed5 < -12) & ~stain & ~ndimage.binary_dilation(mold, iterations=2)
            for cm in _components(thin, 3):
                length, resid, axis_dev = _line_stats(cm)
                L_m = length * m.res
                if L_m < 0.2 or cm.sum() * cell_a > 0.02 * L_m * 3:
                    continue
                if (axis_dev < 4 and resid < 1.5) or _axis_run_fraction(cm) > 0.6:
                    continue  # straight / rectilinear: frame, skirting, shadow line
                emit("crack", cm, min(1.0, L_m / 0.6), {"length": Measure(L_m, combine_sigma(m.res, 0.02))})
    return out


def _axis_run_fraction(cm: np.ndarray, run: int = 5) -> float:
    """Fraction of cells lying on horizontal/vertical runs >= `run` cells (rectilinear outlines)."""
    hit = np.zeros_like(cm)
    k = np.ones(run)
    for axis in (0, 1):
        c = ndimage.convolve1d(cm.astype(int), k, axis=axis, mode="constant")
        full = c >= run
        hit |= ndimage.binary_dilation(full, structure=np.ones((run, 1)) if axis == 0 else np.ones((1, run))) & cm
    return float(hit.sum() / max(cm.sum(), 1))


def detect_damage(rooms: List[Room], fsets: List[FrameSet], cfg: Dict, tier: str) -> List[Dict]:
    res = {"lidar": 0.02, "video": 0.03, "photo": 0.03}[tier]
    tol = max(0.035, float(cfg["tiers"][tier]["wall_tol"]))
    maps = build_surface_maps(rooms, res)
    for fs in fsets:
        project_frames(fs, maps, int(cfg["damage"]["keyframe_stride"]), tol)
    return detect_on_maps(maps, cfg, tier)
