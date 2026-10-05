"""Mirror detection by reflection consistency.

A mirror makes the depth sensor report the reflected room *behind* the wall. Those phantom points
cross a wall plane, and when reflected back across that plane they land on real geometry with the
reflected normal matching the real normal. Doors and windows fail this test (what is behind a door is
a different room). Phantom points behind detected mirrors are removed before room segmentation.
"""
from __future__ import annotations

from typing import Dict, List, Tuple

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree

from .cloud import Cloud


def wall_plane_candidates(cloud: Cloud, floor_z: float, min_pts: int = 300) -> List[Tuple[int, float, int]]:
    out = []
    n = cloud.nrm
    ok = ~np.isnan(n[:, 0]) & ~cloud.far & (np.abs(n[:, 2]) < 0.2) & (cloud.pts[:, 2] > floor_z + 0.3)
    for a in (0, 1):
        for f in (1, -1):
            m = ok & (n[:, a] * f > 0.9)
            if m.sum() < min_pts:
                continue
            x = cloud.pts[m, a]
            lo, hi = np.floor(x.min() * 100), np.ceil(x.max() * 100)
            h, edges = np.histogram(x, bins=np.arange(lo, hi + 2) / 100.0)
            hs = ndimage.uniform_filter1d(h.astype(float), 3)
            pk = (hs >= ndimage.maximum_filter1d(hs, 9)) & (hs * 3 >= min_pts)
            for i in np.nonzero(pk)[0]:
                out.append((a, float(edges[i] + 0.005), f))
    return out


def detect_mirrors(cloud: Cloud, floor_z: float, ceil_z: float, cfg: Dict, max_mirrors: int = 4):
    """Greedy: accept the most reflection-consistent cluster, drop its phantoms, re-evaluate.

    Re-evaluation matters: phantom geometry seen through one mirror also crosses walls further
    away, and its second reflection can look consistent too (a 'ghost of the ghost').
    """
    thr = float(cfg["geometry"]["mirror_overlap_threshold"])
    drop = np.zeros(len(cloud.pts), bool)
    mirrors = []
    for _ in range(max_mirrors):
        cands = _mirror_candidates(cloud, floor_z, ceil_z, ~drop)
        cands = [m for m in cands if m["reflection_agreement"] >= thr]
        if not cands:
            break
        # nearest first: a real mirror is the first plane its rays cross; ghosts lie behind it
        best = min(cands, key=lambda m: m["crossing_dist"])
        mirrors.append(best)
        drop |= points_through_rect(cloud, best["axis"], best["coord"], best["facing"],
                                    (best["s0"] - 0.05, best["s1"] + 0.05, best["z0"] - 0.05, best["z1"] + 0.05))
    return mirrors, drop


def _mirror_candidates(cloud: Cloud, floor_z: float, ceil_z: float, alive: np.ndarray, sub: int = 3):
    full_real = alive & ~cloud.far & ~np.isnan(cloud.nrm[:, 0]) & (cloud.pts[:, 2] > floor_z + 0.25) & (cloud.pts[:, 2] < ceil_z - 0.25)
    tree = cKDTree(cloud.pts[full_real])
    Nreal = cloud.nrm[full_real]
    idx = np.nonzero(alive)[0][::sub]
    P = cloud.pts[idx]
    Nn = cloud.nrm[idx]
    far = cloud.far[idx]
    C = cloud.cams[cloud.fidx[idx]]
    real = ~far & ~np.isnan(Nn[:, 0]) & (P[:, 2] > floor_z + 0.25) & (P[:, 2] < ceil_z - 0.25)
    out = []
    for (a, c, f) in wall_plane_candidates(cloud.subset(alive), floor_z):
        nu_o = f * (C[:, a] - c)
        nu_p = f * (P[:, a] - c)
        thru = (nu_o > 0.1) & (nu_p < -0.08)
        if thru.sum() < 200:
            continue
        lam = nu_o[thru] / (nu_o[thru] - nu_p[thru])
        X = C[thru] + lam[:, None] * (P[thru] - C[thru])
        dcross = lam * np.linalg.norm(P[thru] - C[thru], axis=1)
        s, z = X[:, 1 - a], X[:, 2]
        res = 0.05
        s0, z0 = s.min(), z.min()
        gi = ((s - s0) / res).astype(int)
        gj = ((z - z0) / res).astype(int)
        grid = np.zeros((gj.max() + 1, gi.max() + 1), np.int32)
        np.add.at(grid, (gj, gi), 1)
        lab, nl = ndimage.label(ndimage.binary_closing(grid > 0, iterations=1))
        tidx = np.nonzero(thru)[0]
        for L in range(1, nl + 1):
            sel = lab[gj, gi] == L
            if sel.sum() < 150:
                continue
            pts_i = tidx[sel]
            geo = pts_i[real[pts_i]]
            if len(geo) < 100:
                continue
            Q = P[geo].copy()
            Q[:, a] = 2 * c - Q[:, a]
            nq = Nn[geo].copy()
            nq[:, a] *= -1
            dist, j = tree.query(Q, distance_upper_bound=0.05)
            hit = np.isfinite(dist)
            agree = np.zeros(len(Q), bool)
            agree[hit] = np.sum(nq[hit] * Nreal[j[hit]], axis=1) > 0.7
            ratio = float(agree.mean())
            ss, zz = s[sel], z[sel]
            rect = (float(np.percentile(ss, 1)), float(np.percentile(ss, 99)),
                    float(np.percentile(zz, 1)), float(np.percentile(zz, 99)))
            out.append({"axis": a, "coord": c, "facing": f, "s0": rect[0], "s1": rect[1],
                        "z0": rect[2], "z1": rect[3], "reflection_agreement": round(ratio, 3),
                        "n_points": int(len(pts_i)), "crossing_dist": float(np.median(dcross[sel]))})
    return out


def points_through_rect(cloud: Cloud, a: int, c: float, f: int, rect) -> np.ndarray:
    C = cloud.cams[cloud.fidx]
    P = cloud.pts
    nu_o = f * (C[:, a] - c)
    nu_p = f * (P[:, a] - c)
    thru = (nu_o > 0.05) & (nu_p < -0.02)
    out = np.zeros(len(P), bool)
    if not thru.any():
        return out
    ii = np.nonzero(thru)[0]
    lam = nu_o[ii] / (nu_o[ii] - nu_p[ii])
    X = C[ii] + lam[:, None] * (P[ii] - C[ii])
    s, z = X[:, 1 - a], X[:, 2]
    inr = (s > rect[0]) & (s < rect[1]) & (z > rect[2]) & (z < rect[3])
    out[ii[inr]] = True
    return out
