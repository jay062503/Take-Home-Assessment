"""Depth for RGB-only tiers (photo, video).

Order of preference:
  1. `synthetic_depth/` sidecar next to the images -> only present in synthetic captures; tagged
     depth_source='synthetic-sidecar' so it can never be mistaken for a real-world result.
  2. Monocular metric depth model (Depth Anything V2 Metric-Indoor by default). Outputs are cached
     under <capture>/.cache/depth/<model>/<sha1(image)>.npy, so re-runs replay deterministically
     while the live path stays exercised on new captures.
"""
from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Dict, Optional

import cv2
import numpy as np

from ..config import ROOT


class DepthUnavailable(RuntimeError):
    pass


class SidecarDepth:
    name = "synthetic-sidecar"

    def __init__(self, folder: Path):
        self.folder = folder

    def available(self, key: str) -> bool:
        return (self.folder / f"{key}.npy").exists()

    def __call__(self, rgb: np.ndarray, key: str, out_hw, fx: Optional[float] = None) -> np.ndarray:
        d = np.load(self.folder / f"{key}.npy").astype(np.float32)
        return cv2.resize(d, (out_hw[1], out_hw[0]), interpolation=cv2.INTER_LINEAR)


class MonoDepthModel:
    def __init__(self, cfg: Dict, cache_dir: Optional[Path]):
        self.cfg = cfg["depth_model"]
        self.name = self.cfg["name"]
        self.cache_dir = cache_dir / self.name.replace("/", "__") if (cache_dir and self.cfg.get("cache", True)) else None
        self._pipe = None

    def _load(self):
        if self._pipe is not None:
            return
        try:
            import torch  # noqa: F401
            from transformers import pipeline
        except ImportError as e:
            raise DepthUnavailable(
                "RGB-only tiers need a monocular depth model: `pip install -e .[ml]` then "
                "`scripts/fetch_models.sh` (or ensure cached depth exists in <capture>/.cache)") from e
        import torch
        local = ROOT / "models" / self.name.split("/")[-1]
        model_id = str(local) if local.exists() else self.name
        dev = self.cfg.get("device", "auto")
        if dev == "auto":
            dev = "mps" if torch.backends.mps.is_available() else ("cuda" if torch.cuda.is_available() else "cpu")
        self._pipe = pipeline("depth-estimation", model=model_id, device=dev)

    def focal_scale(self, rgb: np.ndarray, fx: Optional[float]) -> float:
        """Metric mono-depth models learn scale for their training camera; a wider lens makes the same
        room look bigger. Rescale by (fx / long side) relative to the training camera's (canonical
        camera transform, as in Metric3D)."""
        if fx is None or not self.cfg.get("focal_rescale", True):
            return 1.0
        return float(fx / max(rgb.shape[:2]) / float(self.cfg.get("train_fx_over_long_side", 0.866)))

    def __call__(self, rgb: np.ndarray, key: str, out_hw, fx: Optional[float] = None) -> np.ndarray:
        h = hashlib.sha1(rgb.tobytes()).hexdigest()[:16]
        cpath = self.cache_dir / f"{h}.npy" if self.cache_dir else None
        if cpath is not None and cpath.exists():
            d = np.load(cpath).astype(np.float32)
        else:
            self._load()
            from PIL import Image
            res = self._pipe(Image.fromarray(rgb))
            d = np.asarray(res["predicted_depth"], dtype=np.float32).squeeze()
            if cpath is not None:
                cpath.parent.mkdir(parents=True, exist_ok=True)
                np.save(cpath, d.astype(np.float16))
        d = d * self.focal_scale(rgb, fx)
        return cv2.resize(d, (out_hw[1], out_hw[0]), interpolation=cv2.INTER_LINEAR)


def make_provider(folder: Path, cfg: Dict, cache_root: Path):
    side = folder / "synthetic_depth"
    if side.is_dir() and cfg["depth_model"].get("source", "auto") != "model":
        return SidecarDepth(side)
    return MonoDepthModel(cfg, cache_root / ".cache" / "depth")
