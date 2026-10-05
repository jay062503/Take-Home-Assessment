"""Video tier: a handheld walkthrough clip from the native Camera app (any iPhone 15+)."""
from __future__ import annotations

import json
from pathlib import Path
from typing import Dict

import cv2
import numpy as np

from ..depth.provider import make_provider
from ..pose.rgbd_odometry import track_sequence
from ..types import Frame, FrameSet
from .photos import image_quality

RGB_W = 640
DEPTH_W = 256


def _f35(folder: Path, default: float) -> float:
    meta = folder / "capture.json"
    if meta.exists():
        return float(json.loads(meta.read_text()).get("f35", default))
    return default


def load_video(path: Path, cfg: Dict) -> FrameSet:
    tc = cfg["tiers"]["video"]
    folder = path.parent
    cap = cv2.VideoCapture(str(path))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    step = max(1, int(round(fps / float(tc["sample_fps"]))))
    provider = make_provider(folder, cfg, folder)
    f35 = _f35(folder, float(tc.get("f35", 28.0)))
    frames, i = [], 0
    quality = []
    while True:
        ok, img = cap.read()
        if not ok:
            break
        if i % step == 0:
            rgb = img[:, :, ::-1]
            s = min(1.0, RGB_W / rgb.shape[1])
            if s < 1:
                rgb = cv2.resize(rgb, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
            rgb = np.ascontiguousarray(rgb)
            H, W = rgb.shape[:2]
            f = f35 / 36.0 * max(W, H)
            K_rgb = np.array([[f, 0, W / 2], [0, f, H / 2], [0, 0, 1.0]])
            dh, dw = int(round(H * DEPTH_W / W)), DEPTH_W
            K = K_rgb.copy()
            K[:2] *= dw / W
            depth = provider(rgb, f"{i:06d}", (dh, dw), fx=float(K_rgb[0, 0]))
            quality.append(image_quality(rgb))
            frames.append(Frame(index=i, timestamp=i / fps, K=K, T_wc=np.eye(4), depth=depth, rgb=rgb,
                                K_rgb=K_rgb, name=f"{i:06d}"))
        i += 1
    cap.release()
    if len(frames) < 10:
        raise ValueError(f"video {path} yielded only {len(frames)} frames")
    track = track_sequence(frames)
    frames = [frames[k] for k in track["kept"]]
    q = {k: float(np.median([x[k] for x in quality])) for k in quality[0]}
    return FrameSet(tier="video", frames=frames, source=str(path), depth_source=getattr(provider, "name", "mono"),
                    pose_source="rgbd-pnp-odometry", gravity_known=False,
                    meta={"fps": fps, "sampled_every": step, "tracking": track, "f35": f35, "image_quality": q,
                          "synthetic": (folder / "SYNTHETIC").exists()})
