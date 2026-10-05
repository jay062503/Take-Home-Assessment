"""Metric scale for RGB-only tiers: fuse the depth model's scale with a camera-height prior.

A metric mono-depth model's global scale error is roughly +-15% per scene (1 sigma) and varies
from room to room. The capture protocol fixes the phone at chest height, about 1.40 +- 0.10 m for
an adult. A wrong scale inflates the measured camera height above the floor by the same factor,
so camera height is an independent scale observation. The two are fused in log space:

    l_model = 0                  sigma_m = model_scale_sigma
    l_cam   = log(h_prior / h)   sigma_c = camera_height_sigma / h_prior
    l       = l_cam * sigma_m^2 / (sigma_m^2 + sigma_c^2)

The posterior sigma replaces the tier's proportional scale error, so intervals carry it.
"""
from __future__ import annotations

import math
from typing import Dict, Tuple

import numpy as np

from ..types import FrameSet


def fuse_scale(cam_height: float, cfg: Dict) -> Tuple[float, float, Dict]:
    sp = cfg["scale_prior"]
    sm = float(sp["model_scale_sigma"])
    sc = float(sp["camera_height_sigma"]) / float(sp["camera_height"])
    if not sp.get("enabled", True) or not (0.3 < cam_height < 4.0):
        return 1.0, sm, {"applied": False, "camera_height_m": cam_height}
    l_cam = math.log(float(sp["camera_height"]) / cam_height)
    w = sm ** 2 / (sm ** 2 + sc ** 2)
    l = w * l_cam
    sig = math.sqrt(1.0 / (1.0 / sm ** 2 + 1.0 / sc ** 2))
    return math.exp(l), sig, {"applied": True, "camera_height_m": round(cam_height, 3), "scale": round(math.exp(l), 4),
                              "weight_camera_prior": round(w, 3), "scale_sigma": round(sig, 4)}


def apply_scale(fs: FrameSet, s: float) -> None:
    for f in fs.frames:
        f.depth = (f.depth * s).astype(np.float32)
        f.T_wc = f.T_wc.copy()
        f.T_wc[:3, 3] *= s
