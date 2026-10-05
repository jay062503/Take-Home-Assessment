"""Metric visual odometry for RGB-only tiers: ORB matches + PnP against predicted metric depth.

Each frame keeps its own metric depth prediction. Chaining frame-to-frame depth re-scaling was
tried and rejected: it turns per-frame scale jitter into a random walk in scale (+11% ceiling
height over one walkthrough). Per-frame metric depth keeps the scale error bounded by the model's
own error, which is carried in the tier's sys_scale budget.
"""
from __future__ import annotations

from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from ..types import Frame

_ORB = None


def orb():
    global _ORB
    if _ORB is None:
        _ORB = cv2.ORB_create(nfeatures=2500, scaleFactor=1.2, nlevels=8, fastThreshold=10)
    return _ORB


def features(rgb: np.ndarray):
    g = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    g = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(g)
    kp, des = orb().detectAndCompute(g, None)
    pts = np.array([k.pt for k in kp], np.float32) if kp else np.zeros((0, 2), np.float32)
    return pts, des


def match(des_a, des_b, ratio: float = 0.8):
    if des_a is None or des_b is None or len(des_a) < 8 or len(des_b) < 8:
        return np.zeros((0, 2), int)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING)
    knn = bf.knnMatch(des_a, des_b, k=2)
    good = [(m.queryIdx, m.trainIdx) for m, *rest in knn if rest and m.distance < ratio * rest[0].distance]
    return np.asarray(good, int).reshape(-1, 2)


def lift(pts_rgb: np.ndarray, fr: Frame) -> Tuple[np.ndarray, np.ndarray]:
    """3D camera-frame points for RGB pixel coordinates using the frame's depth map."""
    s = fr.depth.shape[1] / fr.rgb.shape[1]
    u = np.clip(np.round(pts_rgb[:, 0] * s).astype(int), 0, fr.depth.shape[1] - 1)
    v = np.clip(np.round(pts_rgb[:, 1] * s).astype(int), 0, fr.depth.shape[0] - 1)
    d = fr.depth[v, u]
    K = fr.K_rgb
    X = np.stack([(pts_rgb[:, 0] - K[0, 2]) / K[0, 0] * d, (pts_rgb[:, 1] - K[1, 2]) / K[1, 1] * d, d], 1)
    return X, (d > 0.2) & (d < 8.0)


def relative_pose(fa: Frame, fb: Frame, fa_feat, fb_feat, min_inliers: int = 25):
    """T_b_a (maps camera-a points into camera-b) + inlier count + depth scale ratio of b vs a."""
    pa, da = fa_feat
    pb, db = fb_feat
    m = match(da, db)
    if len(m) < min_inliers:
        return None, 0, 1.0
    X, ok = lift(pa[m[:, 0]], fa)
    uv = pb[m[:, 1]]
    X, uv = X[ok], uv[ok]
    if len(X) < min_inliers:
        return None, 0, 1.0
    succ, rvec, tvec, inl = cv2.solvePnPRansac(X.astype(np.float64), uv.astype(np.float64), fb.K_rgb, None,
                                               iterationsCount=300, reprojectionError=2.5, confidence=0.999,
                                               flags=cv2.SOLVEPNP_EPNP)
    if not succ or inl is None or len(inl) < min_inliers:
        return None, 0 if inl is None else len(inl), 1.0
    inl = inl[:, 0]
    rvec, tvec = cv2.solvePnPRefineLM(X[inl].astype(np.float64), uv[inl].astype(np.float64), fb.K_rgb, None, rvec, tvec)
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec[:, 0]
    Xb = X[inl] @ R.T + tvec[:, 0]
    Xb_meas, okb = lift(uv[inl], fb)
    sel = okb & (Xb[:, 2] > 0.2)
    ratio = float(np.median(Xb_meas[sel, 2] / Xb[sel, 2])) if sel.sum() > 10 else 1.0
    return T, len(inl), ratio


def _plausible(T_prev: np.ndarray, T_new: np.ndarray, dt: float, v_max: float = 1.5, w_max_deg: float = 150.0) -> bool:
    """Reject PnP solutions a handheld walk cannot produce (degenerate RANSAC fits on repetitive
    texture occasionally 'succeed' with absurd motion)."""
    dt = max(dt, 1e-3)
    dtr = float(np.linalg.norm(T_new[:3, 3] - T_prev[:3, 3]))
    c = (np.trace(T_prev[:3, :3].T @ T_new[:3, :3]) - 1) / 2
    ang = float(np.degrees(np.arccos(np.clip(c, -1, 1))))
    return np.isfinite(dtr) and dtr <= v_max * dt + 0.05 and ang <= w_max_deg * dt + 5.0


def track_sequence(frames: List[Frame], keyframe_every: int = 1, max_skip: int = 2) -> Dict:
    """Chains frame-to-frame PnP against recent and older tracked frames.

    A frame that matches nothing is skipped and the next frames retry against the last tracked
    ones (most failures are one blurred or blank frame). After `max_skip` consecutive failures the
    frame coasts on the last tracked pose and tracking resumes from it; the error is the motion
    during the gap, which the pose graph's Manhattan priors and loop closures then reduce. A turn
    faster than ~100 deg/s at the sampling rate is the known way to break this (see protocol).
    """
    feats = [features(f.rgb) for f in frames]
    frames[0].T_wc = np.eye(4)
    kept, good = [0], [0]
    lost = 0
    inliers = []
    relocalised = 0
    coasted, skipped = [], []
    pending = 0
    for i in range(1, len(frames)):
        # previous frame first, then recent history, then a sparse sweep of older frames
        # (relocalisation) so one bad frame cannot cascade into losing the rest of the clip
        cands = kept[-1:-6:-1] + kept[-6::-8]
        T_ba = None
        last = frames[kept[-1]]
        for j, ref in enumerate(cands):
            T_ba, n, ratio = relative_pose(frames[ref], frames[i], feats[ref], feats[i])
            if T_ba is not None and not _plausible(last.T_wc, frames[ref].T_wc @ np.linalg.inv(T_ba),
                                                   frames[i].timestamp - last.timestamp):
                T_ba = None
            if T_ba is not None:
                relocalised += int(j >= 5)
                break
        if T_ba is None:
            lost += 1
            pending += 1
            if pending <= max_skip:
                skipped.append(i)
                continue
            # hold the last tracked pose; constant-velocity extrapolation was tried and was worse
            # (operators' look-around wobble reverses direction within a few frames)
            frames[i].T_wc = frames[good[-1]].T_wc.copy()
            coasted.append(i)
            kept.append(i)
            pending = 0
            continue
        pending = 0
        frames[i].T_wc = frames[ref].T_wc @ np.linalg.inv(T_ba)
        kept.append(i)
        good.append(i)
        inliers.append(n)
    return {"kept": kept, "lost": lost, "coasted": coasted, "skipped": skipped, "relocalised": relocalised,
            "median_inliers": float(np.median(inliers)) if inliers else 0.0}
