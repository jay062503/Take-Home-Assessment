"""Back-projection, per-pixel normals, gravity and Manhattan alignment."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional

import numpy as np

from ..types import Frame, FrameSet


def cam_points(depth: np.ndarray, K: np.ndarray) -> np.ndarray:
    H, W = depth.shape
    u, v = np.meshgrid(np.arange(W, dtype=np.float32), np.arange(H, dtype=np.float32))
    x = (u - K[0, 2]) / K[0, 0] * depth
    y = (v - K[1, 2]) / K[1, 1] * depth
    return np.stack([x, y, depth], -1)


def depth_normals(P: np.ndarray, valid: np.ndarray, max_jump: float = 0.05) -> np.ndarray:
    """Camera-frame normals from central differences, oriented towards the camera; NaN where unreliable."""
    n = np.full(P.shape, np.nan, np.float32)
    dx = P[1:-1, 2:] - P[1:-1, :-2]
    dy = P[2:, 1:-1] - P[:-2, 1:-1]
    c = np.cross(dx, dy)
    norm = np.linalg.norm(c, axis=-1, keepdims=True)
    ok = (valid[1:-1, 2:] & valid[1:-1, :-2] & valid[2:, 1:-1] & valid[:-2, 1:-1] & (norm[..., 0] > 1e-9))
    z = P[1:-1, 1:-1, 2:3]
    ok &= (np.abs(dx[..., 2]) < max_jump * z[..., 0] * 2) & (np.abs(dy[..., 2]) < max_jump * z[..., 0] * 2)
    c = c / np.maximum(norm, 1e-12)
    flip = np.sum(c * P[1:-1, 1:-1], axis=-1) > 0  # make n point towards camera (n . p < 0)
    c[flip] *= -1
    c[~ok] = np.nan
    n[1:-1, 1:-1] = c
    return n


@dataclass
class Cloud:
    pts: np.ndarray        # (N,3) world
    nrm: np.ndarray        # (N,3) world normals (NaN if unknown), oriented to camera
    fidx: np.ndarray       # (N,) index into frameset.frames
    far: np.ndarray        # (N,) bool: probe point for a no-return / out-of-range ray
    cams: np.ndarray       # (F,3) camera centres

    def subset(self, m: np.ndarray) -> "Cloud":
        return Cloud(self.pts[m], self.nrm[m], self.fidx[m], self.far[m], self.cams)


def build_cloud(fs: FrameSet, tier_cfg: Dict, far_range: float = 6.0) -> Cloud:
    stride = int(tier_cfg["pixel_stride"])
    max_d = float(tier_cfg["max_depth"])
    min_conf = int(tier_cfg.get("min_confidence", 0))
    P_all, N_all, F_all, FAR_all = [], [], [], []
    for k, fr in enumerate(fs.frames):
        d = fr.depth
        P = cam_points(d, fr.K)
        valid = d > 0.15
        if fr.confidence is not None:
            valid_n = valid & (fr.confidence >= max(min_conf, 1))
        else:
            valid_n = valid
        Nc = depth_normals(P, valid_n)
        sl = (slice(stride // 2, None, stride), slice(stride // 2, None, stride))
        Ps, Ns, ds = P[sl].reshape(-1, 3), Nc[sl].reshape(-1, 3), d[sl].reshape(-1)
        good = (ds > 0.15) & (ds <= max_d)
        if fr.confidence is not None:
            cs = fr.confidence[sl].reshape(-1)
            good &= cs >= min_conf
            far = (ds == 0) | (ds > max_d) | ((cs == 0) & (ds > 3.0))
        else:
            far = (ds == 0) | (ds > max_d)
        good &= ~far
        R, t = fr.T_wc[:3, :3], fr.T_wc[:3, 3]
        pw = Ps[good] @ R.T + t
        nw = Ns[good] @ R.T
        # far probes: unit rays pushed to far_range
        u, v = np.meshgrid(np.arange(d.shape[1], dtype=np.float32), np.arange(d.shape[0], dtype=np.float32))
        rays = np.stack([(u - fr.K[0, 2]) / fr.K[0, 0], (v - fr.K[1, 2]) / fr.K[1, 1], np.ones_like(u)], -1)[sl].reshape(-1, 3)
        rays = rays[far]
        if len(rays):
            rays = rays / np.linalg.norm(rays, axis=1, keepdims=True) * far_range
            pf = rays @ R.T + t
        else:
            pf = np.zeros((0, 3))
        P_all += [pw, pf]
        N_all += [nw, np.full((len(pf), 3), np.nan)]
        F_all += [np.full(len(pw), k, np.int32), np.full(len(pf), k, np.int32)]
        FAR_all += [np.zeros(len(pw), bool), np.ones(len(pf), bool)]
    cams = np.array([fr.T_wc[:3, 3] for fr in fs.frames])
    return Cloud(np.concatenate(P_all).astype(np.float64), np.concatenate(N_all).astype(np.float32),
                 np.concatenate(F_all), np.concatenate(FAR_all), cams)


def rot_to_z(up: np.ndarray) -> np.ndarray:
    """Rotation taking unit vector `up` onto +z."""
    up = up / np.linalg.norm(up)
    z = np.array([0, 0, 1.0])
    v = np.cross(up, z)
    s = np.linalg.norm(v)
    c = float(np.dot(up, z))
    if s < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1, -1, -1.0])
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * ((1 - c) / s ** 2)


def estimate_up(cloud: Cloud, fs: FrameSet) -> np.ndarray:
    """Gravity from geometry: horizontal surfaces (floor/ceiling/tables) vote, walls must be perpendicular."""
    guess = -np.mean([fr.T_wc[:3, 1] for fr in fs.frames], axis=0)  # camera -y ~ up for an upright phone
    guess /= np.linalg.norm(guess)
    n = cloud.nrm[~np.isnan(cloud.nrm[:, 0])]
    if len(n) > 200000:
        n = n[np.random.default_rng(0).choice(len(n), 200000, replace=False)]
    up = guess
    for cone in (35, 15, 6):
        c = n @ up
        hor = np.abs(c) > np.cos(np.radians(cone))
        if hor.sum() < 50:
            break
        nh = n[hor] * np.sign(c[hor])[:, None]
        wall = np.abs(c) < np.sin(np.radians(cone))
        M = nh.T @ nh / max(len(nh), 1) - (n[wall].T @ n[wall]) / max(wall.sum(), 1)
        w, V = np.linalg.eigh(M)
        cand = V[:, -1]
        up = cand * np.sign(cand @ up)
    return up


def manhattan_yaw(nrm: np.ndarray) -> float:
    """Dominant wall orientation (mod 90 deg) from near-vertical surface normals."""
    n = nrm[~np.isnan(nrm[:, 0])]
    n = n[np.abs(n[:, 2]) < 0.2]
    if len(n) < 20:
        return 0.0
    phi = np.arctan2(n[:, 1], n[:, 0])
    z = np.mean(np.exp(4j * phi))
    return float(np.angle(z) / 4.0)


def refine_yaw_histogram(xy: np.ndarray, yaw0: float, span_deg: float = 4.0, step_deg: float = 0.05,
                         binw: float = 0.02) -> float:
    """Normals from noisy depth are biased towards the viewing ray on oblique walls, so the
    normal-based yaw can be ~1-2 deg off. Wall points projected on the correct axes give the
    sharpest x/y histograms; search a small window around the normal estimate."""
    if len(xy) < 100:
        return yaw0
    if len(xy) > 300000:
        xy = xy[np.random.default_rng(0).choice(len(xy), 300000, replace=False)]
    best, best_s = yaw0, -1.0
    for d in np.arange(-span_deg, span_deg + 1e-9, step_deg):
        a = yaw0 + np.radians(d)
        c, s = np.cos(-a), np.sin(-a)
        x = c * xy[:, 0] - s * xy[:, 1]
        y = s * xy[:, 0] + c * xy[:, 1]
        score = 0.0
        for v in (x, y):
            h = np.bincount(((v - v.min()) / binw).astype(int)).astype(float)
            score += float(np.sum(h * h))
        if score > best_s:
            best, best_s = a, score
    return best


def apply_world_transform(fs: FrameSet, cloud: Optional[Cloud], R: np.ndarray, t: np.ndarray = None) -> None:
    t = np.zeros(3) if t is None else t
    for fr in fs.frames:
        T = np.eye(4)
        T[:3, :3] = R @ fr.T_wc[:3, :3]
        T[:3, 3] = R @ fr.T_wc[:3, 3] + t
        fr.T_wc = T
    if cloud is not None:
        cloud.pts = cloud.pts @ R.T + t
        cloud.nrm = cloud.nrm @ R.T
        cloud.cams = cloud.cams @ R.T + t


def rotz(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
