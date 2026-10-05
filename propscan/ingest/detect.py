"""Decide which input tier a capture path is, from its on-disk layout alone.

  lidar : a Stray Scanner export (odometry.csv + depth/ + camera_matrix.csv), possibly nested one level
  video : a single video file, or a folder containing exactly one video and no depth/
  photo : a folder of room sub-folders containing images (multi-room), or a folder of images (one room)
A .zip of any of these is unpacked next to itself first (macOS __MACOSX debris is ignored).
"""
from __future__ import annotations

from pathlib import Path
from typing import Tuple

VIDEO_EXT = {".mov", ".mp4", ".m4v"}
IMAGE_EXT = {".jpg", ".jpeg", ".png", ".heic", ".heif"}


def _is_stray(p: Path) -> bool:
    return p.is_dir() and (p / "odometry.csv").exists() and (p / "depth").is_dir()


def _images(p: Path):
    return sorted(f for f in p.iterdir() if f.is_file() and f.suffix.lower() in IMAGE_EXT and not f.name.startswith("."))


def _videos(p: Path):
    return sorted(f for f in p.iterdir() if f.is_file() and f.suffix.lower() in VIDEO_EXT and not f.name.startswith("."))


def _unzip(p: Path) -> Path:
    import zipfile
    dst = p.with_name(f".{p.stem}_unzipped")
    if not dst.exists():
        with zipfile.ZipFile(p) as z:
            z.extractall(dst, members=[m for m in z.namelist() if not m.startswith("__MACOSX")])
    kids = [k for k in dst.iterdir() if not k.name.startswith(".")]
    return kids[0] if len(kids) == 1 and kids[0].is_dir() else dst


def resolve_capture_root(path: str) -> Path:
    p = Path(path).expanduser().resolve()
    if p.is_file() and p.suffix.lower() == ".zip":
        p = _unzip(p)
    if p.is_dir() and not _is_stray(p) and (p / "capture").exists():
        return p / "capture"
    if p.is_dir() and not _is_stray(p):
        subs = [s for s in p.iterdir() if s.is_dir() and not s.name.startswith(".")]
        stray = [s for s in subs if _is_stray(s)]
        if len(stray) == 1 and not _images(p):
            return stray[0]
    return p


def detect_tier(path: str) -> Tuple[str, Path]:
    p = resolve_capture_root(path)
    if not p.exists():
        raise FileNotFoundError(p)
    if p.is_file():
        if p.suffix.lower() in VIDEO_EXT:
            return "video", p
        raise ValueError(f"unsupported capture file {p}")
    if _is_stray(p):
        return "lidar", p
    vids = _videos(p)
    if vids and not _images(p):
        if len(vids) > 1:
            raise ValueError(f"{p} contains {len(vids)} videos; pass one video per capture")
        return "video", vids[0]
    if _images(p):
        return "photo", p
    rooms = [s for s in sorted(p.iterdir()) if s.is_dir() and not s.name.startswith((".", "_")) and _images(s)]
    if rooms:
        return "photo", p
    raise ValueError(f"cannot determine tier for {p}: expected a Stray Scanner export, a video, or photo folders")
