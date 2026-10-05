"""Loader for Stray Scanner (App Store, free) exports: the LiDAR tier.

Layout: rgb.mp4, depth/NNNNNN.png (uint16 mm, 256x192), confidence/NNNNNN.png (0/1/2),
odometry.csv (timestamp, frame, x, y, z, qx, qy, qz, qw; ARKit camera-to-world), camera_matrix.csv
(3x3 intrinsics of the RGB stream), imu.csv.
"""
from __future__ import annotations

import csv
from pathlib import Path
from typing import Dict

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from ..types import Frame, FrameSet

W_AR2INT = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float)
F_CV2AR = np.diag([1.0, -1.0, -1.0])
RGB_MAX_W = 640


def arkit_to_internal(T_ar: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = W_AR2INT @ T_ar[:3, :3] @ F_CV2AR
    T[:3, 3] = W_AR2INT @ T_ar[:3, 3]
    return T


def read_odometry(path: Path):
    rows = []
    with open(path) as f:
        rd = csv.reader(f, skipinitialspace=True)
        header = next(rd)
        for r in rd:
            if r:
                rows.append([float(x) for x in r])
    return np.asarray(rows), [h.strip() for h in header]


def load_stray(root: Path, cfg: Dict) -> FrameSet:
    tc = cfg["tiers"]["lidar"]
    odo, header = read_odometry(root / "odometry.csv")
    col = {h: i for i, h in enumerate(header)}
    K_rgb = np.loadtxt(root / "camera_matrix.csv", delimiter=",")
    depth_files = sorted((root / "depth").glob("*.png"))
    if not depth_files:
        depth_files = sorted((root / "depth").glob("*.npy"))
    n = min(len(depth_files), len(odo))
    keep = list(range(0, n, int(tc["frame_stride"])))

    cap = cv2.VideoCapture(str(root / "rgb.mp4"))
    vid_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or int(round(K_rgb[0, 2] * 2))
    rgb_scale = min(1.0, RGB_MAX_W / vid_w)
    K_rgb_s = K_rgb.copy()
    K_rgb_s[:2] *= rgb_scale
    rgbs = {}
    want = set(keep)
    i = 0
    while cap.isOpened() and i < n:
        ok, img = cap.read()
        if not ok:
            break
        if i in want:
            if rgb_scale < 1:
                img = cv2.resize(img, None, fx=rgb_scale, fy=rgb_scale, interpolation=cv2.INTER_AREA)
            rgbs[i] = img[:, :, ::-1].copy()
        i += 1
    cap.release()

    frames = []
    for i in keep:
        df = depth_files[i]
        if df.suffix == ".png":
            d = cv2.imread(str(df), cv2.IMREAD_UNCHANGED).astype(np.float32) / 1000.0
        else:
            d = np.load(df).astype(np.float32)
        cf = root / "confidence" / df.with_suffix(".png").name
        conf = cv2.imread(str(cf), cv2.IMREAD_UNCHANGED) if cf.exists() else np.full(d.shape, 2, np.uint8)
        K_d = K_rgb.copy()
        K_d[:2] *= d.shape[1] / vid_w
        r = odo[i]
        T_ar = np.eye(4)
        T_ar[:3, :3] = Rotation.from_quat([r[col["qx"]], r[col["qy"]], r[col["qz"]], r[col["qw"]]]).as_matrix()
        T_ar[:3, 3] = [r[col["x"]], r[col["y"]], r[col["z"]]]
        frames.append(Frame(index=i, timestamp=float(r[col["timestamp"]]), K=K_d, T_wc=arkit_to_internal(T_ar),
                            depth=d, confidence=conf, rgb=rgbs.get(i), K_rgb=K_rgb_s, name=f"{i:06d}"))
    return FrameSet(tier="lidar", frames=frames, source=str(root), depth_source="arkit-lidar",
                    pose_source="arkit-vio", gravity_known=True,
                    meta={"n_raw_frames": n, "synthetic": (root / "SYNTHETIC").exists()})
