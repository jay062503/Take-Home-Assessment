"""Accumulated-drift correction for multi-room sequences (LiDAR and video tiers).

Gravity from ARKit (or from our floor/wall estimate on video) is reliable, so drift is modelled in
SE(2) per keyframe: x, y, yaw. A pose graph combines
  * odometry edges   : relative motion between consecutive keyframes, sigma grows with distance
  * loop closures    : revisits (close in space, far apart in time) aligned by 4-DoF ICP on local
                       submaps; only accepted with good overlap and low residual
  * Manhattan priors : each keyframe's local walls should be axis aligned in the global Manhattan
                       frame; measured by wall-histogram sharpness (not normals, which are biased),
                       robust loss so non-Manhattan walls do not dominate
and the solved per-keyframe correction is interpolated back to every frame.
`--no-drift-correction` skips this so the ablation can show the footprint with it on and off.
"""
from __future__ import annotations

from typing import Dict, List, Tuple

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree

from ..geometry.cloud import cam_points, refine_yaw_histogram
from ..types import FrameSet


def _yaw(R: np.ndarray) -> float:
    f = R[:, 2]  # optical axis
    return float(np.arctan2(f[1], f[0]))


def _wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def _wrap4(a):
    q = np.pi / 2
    return (a + q / 2) % q - q / 2


def _rz(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def _local_points(fr, stride: int = 4, max_d: float = 4.0) -> np.ndarray:
    P = cam_points(fr.depth, fr.K)[stride // 2::stride, stride // 2::stride].reshape(-1, 3)
    ok = (P[:, 2] > 0.2) & (P[:, 2] < max_d)
    if fr.confidence is not None:
        ok &= fr.confidence[stride // 2::stride, stride // 2::stride].reshape(-1) >= 1
    return P[ok]


def select_keyframes(fs: FrameSet, dist: float, ang_deg: float) -> List[int]:
    kf = [0]
    for i, fr in enumerate(fs.frames[1:], 1):
        ref = fs.frames[kf[-1]]
        d = np.linalg.norm(fr.center[:2] - ref.center[:2])
        a = abs(_wrap(_yaw(fr.T_wc[:3, :3]) - _yaw(ref.T_wc[:3, :3])))
        if d > dist or np.degrees(a) > ang_deg:
            kf.append(i)
    if kf[-1] != len(fs.frames) - 1:
        kf.append(len(fs.frames) - 1)
    return kf


def _icp_4dof(src: np.ndarray, dst_tree: cKDTree, dst: np.ndarray, iters: int = 25, max_corr: float = 0.15):
    """Align src -> dst with rotation about z + translation. Returns (theta, t, fitness, rmse)."""
    th, t = 0.0, np.zeros(3)
    cur = src.copy()
    for k in range(iters):
        d, j = dst_tree.query(cur, distance_upper_bound=max_corr)
        m = np.isfinite(d)
        if m.sum() < 50:
            return th, t, 0.0, np.inf
        a, b = cur[m], dst[j[m]]
        ca, cb = a.mean(0), b.mean(0)
        A, B = a[:, :2] - ca[:2], b[:, :2] - cb[:2]
        H = A.T @ B
        dth = np.arctan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
        R = _rz(dth)
        dt = cb - R @ ca
        cur = cur @ R.T + dt
        th += dth
        t = R @ t + dt
        max_corr = max(0.04, max_corr * 0.8)
    d, _ = dst_tree.query(cur, distance_upper_bound=0.03)
    m = np.isfinite(d)
    return th, t, float(m.mean()), float(np.sqrt(np.mean(d[m] ** 2))) if m.any() else np.inf


def correct_drift(fs: FrameSet, cfg: Dict) -> Dict:
    dc = cfg["drift"]
    frames = fs.frames
    kf = select_keyframes(fs, float(dc["keyframe_dist"]), float(dc["keyframe_angle_deg"]))
    K = len(kf)
    if K < 4:
        return {"applied": False, "reason": "too few keyframes", "keyframes": K}
    poses0 = np.array([[frames[i].center[0], frames[i].center[1], _yaw(frames[i].T_wc[:3, :3])] for i in kf])
    local = [_local_points(frames[i]) for i in kf]
    world = [lp @ frames[i].T_wc[:3, :3].T + frames[i].T_wc[:3, 3] for lp, i in zip(local, kf)]

    def submap(k, r=2):
        ks = range(max(0, k - r), min(K, k + r + 1))
        pts = np.concatenate([world[q] for q in ks])
        return pts[:: max(1, len(pts) // 40000)]

    # ---- Manhattan prior per keyframe (wall points of a small submap)
    zf = np.percentile(np.concatenate(world[:: max(1, K // 20)])[:, 2], 2)
    allwall = np.concatenate([w[(w[:, 2] > zf + 1.0)][::4] for w in world])
    from ..geometry.cloud import manhattan_yaw  # noqa
    g_yaw = refine_yaw_histogram(allwall[:, :2], 0.0, span_deg=45.0, step_deg=0.25)
    g_yaw = refine_yaw_histogram(allwall[:, :2], g_yaw, span_deg=1.0, step_deg=0.05)
    man = []
    for k in range(K):
        sm = submap(k, 1)
        wp = sm[sm[:, 2] > zf + 1.0][:, :2]
        if len(wp) < 800:
            continue
        y = refine_yaw_histogram(wp, g_yaw, span_deg=4.0, step_deg=0.1)
        r = _wrap4(y - g_yaw)
        if abs(np.degrees(r)) < 3.9:
            man.append((k, float(r)))

    # ---- loop closures
    loops = []
    centers = poses0[:, :2]
    tree_c = cKDTree(centers)
    tried = 0
    for k in range(K):
        for j in tree_c.query_ball_point(centers[k], float(dc["loop_max_dist"])):
            if j - k < int(dc["loop_min_gap"]):
                continue
            if np.degrees(abs(_wrap(poses0[j, 2] - poses0[k, 2]))) > float(dc["loop_max_yaw_deg"]):
                continue
            tried += 1
            dst = submap(k)
            src = submap(j)
            th, t, fit, rmse = _icp_4dof(src, cKDTree(dst), dst)
            if fit > 0.45 and rmse < 0.02 and abs(np.degrees(th)) < 8 and np.linalg.norm(t[:2]) < 0.5:
                loops.append((k, j, th, t[:2], fit, rmse))
            break  # one attempt per keyframe pair-neighbourhood
    # ---- pose graph
    sig_t = float(dc["odom_sigma_trans"])
    sig_tm = float(dc["odom_sigma_trans_per_m"])
    sig_y = np.radians(float(dc["odom_sigma_yaw_deg"]))
    sig_m = np.radians(float(dc["manhattan_prior_sigma_deg"]))

    def rel(pa, pb):
        c, s = np.cos(pa[2]), np.sin(pa[2])
        d = pb[:2] - pa[:2]
        return np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1], _wrap(pb[2] - pa[2])])

    odo = [(k, k + 1, rel(poses0[k], poses0[k + 1])) for k in range(K - 1)]
    loop_meas = []
    for (k, j, th, t, fit, rmse) in loops:
        pj = poses0[j].copy()
        R = _rz(th)[:2, :2]
        pj_corr = np.r_[R @ pj[:2] + t, pj[2] + th]
        loop_meas.append((k, j, rel(poses0[k], pj_corr)))

    def residuals(x):
        X = x.reshape(K, 3)
        r = [(X[0] - poses0[0]) * 1e3]
        for (a, b, z) in odo:
            e = rel(X[a], X[b]) - z
            e[2] = _wrap(e[2])
            dist = np.linalg.norm(z[:2])
            r.append(e / np.array([sig_t + sig_tm * dist, sig_t + sig_tm * dist, sig_y]))
        for (a, b, z) in loop_meas:
            e = rel(X[a], X[b]) - z
            e[2] = _wrap(e[2])
            r.append(e / np.array([0.01, 0.01, np.radians(0.3)]))
        for (k, rr) in man:
            r.append(np.array([_wrap((X[k, 2] - poses0[k, 2]) + rr) / sig_m]))
        return np.concatenate(r)

    sol = least_squares(residuals, poses0.ravel(), loss="soft_l1", f_scale=3.0, max_nfev=200)
    X = sol.x.reshape(K, 3)

    # ---- apply: per-keyframe SE(2) correction, interpolated by frame index
    corr = []
    for k in range(K):
        dth = _wrap(X[k, 2] - poses0[k, 2])
        t = X[k, :2] - _rz(dth)[:2, :2] @ poses0[k, :2]
        corr.append((dth, t))
    kf_arr = np.array(kf)
    for i, fr in enumerate(frames):
        k = int(np.searchsorted(kf_arr, i, side="right") - 1)
        k = min(max(k, 0), K - 1)
        if k < K - 1 and kf_arr[k + 1] != kf_arr[k]:
            a = (i - kf_arr[k]) / (kf_arr[k + 1] - kf_arr[k])
            dth = (1 - a) * corr[k][0] + a * corr[k + 1][0]
            t = (1 - a) * corr[k][1] + a * corr[k + 1][1]
        else:
            dth, t = corr[k]
        Rz = _rz(dth)
        T = fr.T_wc.copy()
        T[:3, :3] = Rz @ T[:3, :3]
        T[:3, 3] = Rz @ T[:3, 3] + np.r_[t, 0.0]
        fr.T_wc = T
    shift = np.linalg.norm(X[:, :2] - poses0[:, :2], axis=1)
    return {"applied": True, "method": "SE2 pose graph: odometry + ICP loop closures + Manhattan yaw priors",
            "keyframes": K, "loop_candidates": tried, "loop_closures": len(loops), "manhattan_priors": len(man),
            "max_correction_m": float(shift.max()), "mean_correction_m": float(shift.mean()),
            "max_yaw_correction_deg": float(np.degrees(np.max(np.abs(_wrap(X[:, 2] - poses0[:, 2]))))),
            "final_cost": float(sol.cost)}
