"""One capture in -> plan.json + plan.png out. Tier is detected from the files on disk."""
from __future__ import annotations

import copy
import time
from pathlib import Path
from typing import Dict, Optional

import numpy as np

from .config import load_config
from .damage.concealed import concealed_flags
from .damage.detect import detect_damage
from .damage.scope import scope_items
from .geometry.cloud import build_cloud
from .geometry.layout import align_frameset, extract_rooms
from .ingest.detect import detect_tier
from .output.build import build_plan, validate, write_json
from .output.render import render_plan
from .pose.drift import correct_drift


class Timer:
    def __init__(self):
        self.t = {}
        self._t0 = time.perf_counter()
        self._last = self._t0

    def lap(self, name):
        now = time.perf_counter()
        self.t[name] = round(now - self._last, 2)
        self._last = now

    def total(self):
        self.t["total"] = round(time.perf_counter() - self._t0, 2)
        return self.t


def _rgb_scale(fs, cfg: Dict, tier: str, diag: Dict, key: str = None) -> Dict:
    """Fuse model depth scale with the camera-height prior; rescale the frames and widen sys_scale.
    Never for LiDAR: metric depth must not be pulled toward a prior about the operator."""
    if tier == "lidar" or fs.tier == "lidar":
        return cfg
    from .depth.scale import apply_scale, fuse_scale
    from .geometry.cloud import build_cloud as _bc
    from .geometry.structure import global_levels
    cloud = _bc(fs, cfg["tiers"][tier])
    if fs.tier == "photo":
        align_frameset(fs, cloud)
    floor, _ = global_levels(cloud)
    h = float(np.median(cloud.cams[:, 2]) - floor)
    s, sig, info = fuse_scale(h, cfg)
    if info["applied"]:
        apply_scale(fs, s)
        cfg = copy.deepcopy(cfg)
        tc = cfg["tiers"][tier]
        tc["sys_scale"] = float(np.hypot(sig, float(tc["sys_scale"])))
    diag.setdefault("scale_fusion", {})[key or tier] = info
    return cfg


def _quality_widening(cfg: Dict, tier: str, quality: Dict, warnings) -> None:
    """Thin or degraded input must widen intervals, not just emit a warning."""
    if not quality:
        return
    k = 1.0
    if quality.get("brightness", 255) < 60:
        warnings.append(f"low light (median brightness {quality['brightness']:.0f}/255): {tier} intervals widened x1.5")
        k *= 1.5
    if quality.get("sharpness", 1e9) < 40:
        warnings.append(f"motion blur (Laplacian var {quality['sharpness']:.0f}): {tier} intervals widened x1.3")
        k *= 1.3
    if quality.get("clipped_highlights", 0) > 0.08:
        warnings.append("clipped highlights / glare (>8% saturated pixels): wet-look or glossy surfaces may lose depth")
    if k > 1:
        tc = cfg["tiers"][tier]
        for key in ("sys_plane", "sys_scale", "sys_height"):
            tc[key] = float(tc[key]) * k


def run_capture(path: str, out_dir: Optional[str] = None, cfg: Optional[Dict] = None, tier: Optional[str] = None,
                drift: Optional[bool] = None, capture_id: Optional[str] = None, render: bool = True,
                verbose: bool = True) -> Dict:
    cfg = copy.deepcopy(cfg or load_config())
    if drift is not None:
        cfg["drift"]["enabled"] = bool(drift)
    T = Timer()
    det, root = detect_tier(path)
    tier = tier or det
    cid = capture_id or Path(path).resolve().name
    if cid == "capture":
        cid = Path(path).resolve().parent.name
    warnings, diag = [], {}
    log = (lambda *a: print("[propscan]", *a)) if verbose else (lambda *a: None)
    log(f"{cid}: tier={tier} source={root}")

    if tier == "lidar":
        from .ingest.stray import load_stray
        fs = load_stray(root, cfg)
        T.lap("ingest")
        drift_info = correct_drift(fs, cfg) if cfg["drift"]["enabled"] else {"applied": False, "reason": "disabled (--no-drift-correction)"}
        T.lap("drift")
        rooms, adjacency, d = extract_rooms(fs, cfg, "ceiling")
        fsets = [fs]
        stitch_info = {"method": "single global frame from ARKit VIO poses" + (" + pose-graph drift correction" if drift_info.get("applied") else " (poses as-is)")}
        synthetic = bool(fs.meta.get("synthetic"))
        depth_src, pose_src = fs.depth_source, fs.pose_source
        n_frames = len(fs.frames)
    elif tier == "video":
        from .ingest.video import load_video
        fs = load_video(root, cfg)
        _quality_widening(cfg, "video", fs.meta.get("image_quality"), warnings)
        T.lap("ingest+depth+odometry")
        cloud = build_cloud(fs, cfg["tiers"]["video"])
        diag["alignment"] = align_frameset(fs, cloud)
        fs.gravity_known = True
        drift_info = correct_drift(fs, cfg) if cfg["drift"]["enabled"] else {"applied": False, "reason": "disabled (--no-drift-correction)"}
        T.lap("drift")
        cfg = _rgb_scale(fs, cfg, "video", diag)
        rooms, adjacency, d = extract_rooms(fs, cfg, "ceiling")
        fsets = [fs]
        stitch_info = {"method": "single global frame from RGB-D odometry" + (" + pose-graph drift correction" if drift_info.get("applied") else " (poses as-is)")}
        if fs.meta["tracking"]["lost"]:
            warnings.append(f"video: {fs.meta['tracking']['lost']} sampled frames failed to track and coasted on the "
                            f"previous pose (drift correction absorbs the jump; intervals reflect it)")
        synthetic = bool(fs.meta.get("synthetic"))
        depth_src, pose_src = fs.depth_source, fs.pose_source
        n_frames = len(fs.frames)
    elif tier == "photo":
        from .ingest.photos import load_photo_capture
        from .photo.stitch import stitch
        data = load_photo_capture(root, cfg)
        T.lap("ingest+depth+registration")
        rooms, fsets, notes = [], [], {}
        for name, fs, nt in data:
            q = {k: float(np.median([x[k] for x in nt["quality"]])) for k in nt["quality"][0]} if nt["quality"] else {}
            rcfg = copy.deepcopy(cfg)
            _quality_widening(rcfg, "photo", q, warnings)
            if nt["registration"]["dropped"]:
                warnings.append(f"room '{name}': photos not registrable and dropped: {nt['registration']['dropped']}")
            if len(fs.frames) < 2:
                warnings.append(f"room '{name}': fewer than 2 registered photos; room skipped")
                continue
            rcfg = _rgb_scale(fs, rcfg, "photo", diag, key=name)
            rr, _, dg = extract_rooms(fs, rcfg, "visibility", label=name)
            if not rr:
                warnings.append(f"room '{name}': no room footprint could be reconstructed")
                continue
            rooms.append(rr[0])
            fsets.append(fs)
            notes[name] = {"registration": nt["registration"], "exif": nt["exif"][:1], "quality": q,
                           "mirrors": dg.get("mirrors", [])}
        diag["photo_rooms"] = notes
        drift_info = {"applied": False, "reason": "not applicable: photo tier has no trajectory; rooms are placed by door matching"}
        stitch_info = stitch(rooms, fsets, cfg, root)
        adjacency = [{"a": e["a"], "b": e["b"], "via": e["via"]} for e in stitch_info["edges"]]
        for u in stitch_info["unresolved"]:
            warnings.append(f"photo stitch: room '{u}' could not be connected through any door; placed to the side")
        T.lap("rooms+stitch")
        d = {}
        synthetic = any(fs.meta.get("synthetic") for _, fs, _ in data)
        depth_src = data[0][1].depth_source if data else "none"
        pose_src = "per-room PnP spanning tree; rooms placed by door matching"
        n_frames = sum(len(fs.frames) for fs in fsets)
    else:
        raise ValueError(tier)
    if tier != "photo":
        T.lap("structure")
    diag.update({k: v for k, v in d.items() if k != "cloud"})

    damage = detect_damage(rooms, fsets, cfg, tier)
    flags = concealed_flags(rooms, damage)
    scope = scope_items(rooms, damage, flags)
    T.lap("damage+scope")

    if depth_src == "synthetic-sidecar":
        warnings.append("depth from synthetic sidecar (integration test data): not a real-world accuracy result")
    if cfg["calibration"].get("status") != "calibrated":
        warnings.append("intervals use error-budget priors (no benchmark calibration found in configs/calibration.json)")
    for r in rooms:
        for w in r.walls:
            if w.support < 25:
                warnings.append(f"{w.id}: wall plane barely observed ({w.support} points); position from footprint, interval widened")
        for f in r.features:
            if f["type"] == "mirror":
                warnings.append(f"{r.id}: mirror detected (reflection agreement {f['reflection_agreement']:.2f}); phantom geometry removed")
                break

    capture = {"id": cid, "tier": tier, "source": str(root), "depth_source": depth_src, "pose_source": pose_src,
               "frames_used": n_frames, "drift_correction": drift_info, "synthetic": synthetic}
    plan = build_plan(capture, tier, rooms, adjacency, stitch_info, damage, flags, scope, warnings, T.total(), cfg, diag)
    validate(plan)
    if out_dir:
        out = Path(out_dir)
        write_json(plan, out / "plan.json")
        if render:
            render_plan(plan, out / "plan.png")
        log(f"wrote {out / 'plan.json'} and plan.png ({plan['timing']['total']} s)")
    return plan
