"""Photo tier: 2-8 stills per room from any iPhone 15+, one folder per room, no depth, no poses.

Intrinsics come from EXIF FocalLengthIn35mmFilm (fallback 26 mm, the iPhone main camera).
Within a room, photos are registered to each other with ORB + PnP on predicted metric depth
(a maximum spanning tree over pairwise inlier counts); unregistrable photos are dropped and reported.
"""
from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Tuple

import cv2
import numpy as np
from PIL import Image, ImageOps

from ..depth.provider import make_provider
from ..pose.rgbd_odometry import features, relative_pose
from ..types import Frame, FrameSet
from .detect import IMAGE_EXT

PROC_LONG = 1024
DEPTH_LONG = 384

try:  # iPhone default is HEIC
    from pillow_heif import register_heif_opener
    register_heif_opener()
except Exception:  # pragma: no cover
    pass


def image_quality(rgb: np.ndarray) -> Dict[str, float]:
    g = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    return {"brightness": float(g.mean()), "sharpness": float(cv2.Laplacian(g, cv2.CV_64F).var()),
            "clipped_highlights": float((g > 250).mean())}


def read_photo(path: Path, default_f35: float = 26.0):
    im = Image.open(path)
    exif = im.getexif()
    f35 = None
    try:
        f35 = exif.get_ifd(0x8769).get(0xA405)
    except Exception:
        pass
    model = exif.get(0x0110, "")
    im = ImageOps.exif_transpose(im).convert("RGB")
    rgb = np.asarray(im)
    s = PROC_LONG / max(rgb.shape[:2])
    if s < 1:
        rgb = cv2.resize(rgb, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
    H, W = rgb.shape[:2]
    f35 = float(f35) if f35 else default_f35
    f = f35 / 36.0 * max(W, H)
    K = np.array([[f, 0, W / 2], [0, f, H / 2], [0, 0, 1.0]])
    return np.ascontiguousarray(rgb), K, {"f35": f35, "f35_from_exif": bool(exif.get_ifd(0x8769).get(0xA405)) if exif else False,
                                          "model": str(model)}


def room_folders(root: Path) -> List[Path]:
    imgs = [f for f in root.iterdir() if f.is_file() and f.suffix.lower() in IMAGE_EXT]
    if imgs:
        return [root]
    return [d for d in sorted(root.iterdir()) if d.is_dir() and not d.name.startswith((".", "_"))
            and any(f.suffix.lower() in IMAGE_EXT for f in d.iterdir())]


def load_room(folder: Path, cfg: Dict, cache_root: Path) -> Tuple[FrameSet, Dict]:
    provider = make_provider(folder, cfg, cache_root)
    files = sorted(f for f in folder.iterdir() if f.is_file() and f.suffix.lower() in IMAGE_EXT and not f.name.startswith("."))
    frames, notes = [], {"files": [f.name for f in files], "exif": [], "quality": []}
    for i, f in enumerate(files):
        rgb, K_rgb, ex = read_photo(f)
        H, W = rgb.shape[:2]
        s = DEPTH_LONG / max(H, W)
        dh, dw = int(round(H * s)), int(round(W * s))
        K = K_rgb.copy()
        K[:2] *= dw / W
        depth = provider(rgb, f.stem, (dh, dw), fx=float(K_rgb[0, 0]))
        frames.append(Frame(index=i, timestamp=float(i), K=K, T_wc=np.eye(4), depth=depth, rgb=rgb, K_rgb=K_rgb,
                            name=f.name, group=folder.name))
        notes["exif"].append(ex)
        notes["quality"].append(image_quality(rgb))
    reg = register_room(frames)
    notes["registration"] = reg
    kept = [frames[i] for i in reg["registered"]]
    fs = FrameSet(tier="photo", frames=kept, source=str(folder), depth_source=getattr(provider, "name", "mono"),
                  pose_source="pnp-spanning-tree", gravity_known=False,
                  meta={"room_folder": folder.name, "synthetic": (folder.parent / "SYNTHETIC").exists() or (folder / "SYNTHETIC").exists()})
    return fs, notes


def register_room(frames: List[Frame], min_inliers: int = 20) -> Dict:
    n = len(frames)
    feats = [features(f.rgb) for f in frames]
    edges = []
    rel = {}
    for i in range(n):
        for j in range(n):
            if i == j:
                continue
            T, cnt, ratio = relative_pose(frames[i], frames[j], feats[i], feats[j], min_inliers)
            if T is not None:
                rel[(i, j)] = (T, cnt, ratio)
                edges.append((cnt, i, j))
    root = int(np.argmax([sum(c for c, a, b in edges if a == k or b == k) for k in range(n)])) if edges else 0
    frames[root].T_wc = np.eye(4)
    done = {root}
    order = []
    while True:
        cands = [(c, a, b) for c, a, b in edges if a in done and b not in done]
        if not cands:
            break
        c, a, b = max(cands)
        T_ba, _, ratio = rel[(a, b)]
        frames[b].depth = (frames[b].depth / ratio).astype(np.float32)
        frames[b].T_wc = frames[a].T_wc @ np.linalg.inv(T_ba)
        done.add(b)
        order.append((a, b, c))
    return {"registered": sorted(done), "dropped": [frames[k].name for k in range(n) if k not in done],
            "tree": order, "root": frames[root].name}


def load_photo_capture(root: Path, cfg: Dict):
    rooms = []
    for folder in room_folders(root):
        fs, notes = load_room(folder, cfg, root)
        rooms.append((folder.name, fs, notes))
    return rooms
