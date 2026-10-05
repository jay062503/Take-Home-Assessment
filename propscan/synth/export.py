"""Write synthetic captures in the *same on-disk formats real captures use*.

  lidar/  -> Stray Scanner export (rgb.mp4, depth/*.png uint16 mm, confidence/*.png,
             odometry.csv in ARKit convention, camera_matrix.csv, imu.csv)
  video/  -> walkthrough.mp4 (+ synthetic_depth/ sidecar that emulates monocular-depth error)
  photos/ -> one folder per room, JPEGs with iPhone-like EXIF (+ synthetic_depth/ sidecar)
  gt.yaml -> exact ground truth in the benchmark GT format (docs/GROUND_TRUTH_PROTOCOL.md)

The sidecars exist only so the photo/video code paths can be integration-tested without
downloading model weights. Runs that consume them are tagged depth_source=synthetic-sidecar
and are never reported as real-world accuracy.
"""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Dict, List, Sequence

import cv2
import numpy as np
import yaml
from PIL import Image

from .scene import Scene, look_rotation, render

RGB_W, RGB_H = 384, 288
DEPTH_W, DEPTH_H = 256, 192
F35 = 26.0  # iPhone 15 main camera, 35 mm-equivalent focal length
F35_ULTRAWIDE = 13.0  # 0.5x lens; the photo protocol asks for it (roughly 108 deg horizontal FOV)

# ARKit world (y up) -> internal world (z up)
W_AR2INT = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float)
F_CV2AR = np.diag([1.0, -1.0, -1.0])


def intrinsics(W: int, H: int, f35: float = F35) -> np.ndarray:
    f = f35 / 36.0 * max(W, H)
    return np.array([[f, 0, W / 2], [0, f, H / 2], [0, 0, 1.0]])


def _rotz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def walk_trajectory(path: Sequence, spins: Sequence, rng: np.random.Generator, step: float = 0.11,
                    spin_frames: int = 44, height: float = 1.40, wobble: float = 0.8, turn_deg: float = 12.0,
                    continuous: bool = False) -> List[np.ndarray]:
    """continuous=True keeps the look-around wobble running through spins and corner turns (no yaw
    jumps), i.e. an operator who follows the protocol's 'no faster than a slow head turn'. The
    default (False) reproduces the original LiDAR trajectories bit for bit."""
    path = [np.asarray(p, float) for p in path]
    spins = [np.asarray(p, float) for p in spins]
    poses = []
    phase = rng.uniform(0, 6.28)
    spun = set()

    def pose(p2, yaw, pitch, roll=0.0):
        T = np.eye(4)
        T[:3, :3] = look_rotation(yaw, pitch, roll)
        jitter = rng.normal(0, 0.012, 3)
        T[:3, 3] = [p2[0] + jitter[0], p2[1] + jitter[1], height + 0.04 * math.sin(phase * 0.7) + jitter[2]]
        return T

    def maybe_spin(p, heading):
        nonlocal phase
        for k, s in enumerate(spins):
            if k not in spun and np.linalg.norm(s - p) < 0.05:
                spun.add(k)
                for i in range(spin_frames):
                    a = heading + 2 * math.pi * i / spin_frames
                    if continuous:
                        a += wobble * math.sin(phase)
                        # one slow sweep up to the ceiling-wall edge and back, continuous with walking pitch
                        pitch = -0.1 + 0.4 * math.sin(phase * 0.63) + 0.55 * math.sin(math.pi * i / spin_frames)
                    else:
                        pitch = 0.8 * math.sin(2 * math.pi * 3 * i / spin_frames)  # protocol: sweep up to the ceiling
                    poses.append(pose(p, a, pitch, rng.normal(0, 0.03)))
                    phase += 0.1 if continuous else 0.3

    heading = math.atan2(*(path[1] - path[0])[::-1])
    maybe_spin(path[0], heading)
    cur = heading
    for a, b in zip(path[:-1], path[1:]):
        d = b - a
        L = np.linalg.norm(d)
        heading = math.atan2(d[1], d[0])
        # people turn at a corner gradually (<= ~12 deg per frame), they do not snap 90 deg
        delta = (heading - cur + math.pi) % (2 * math.pi) - math.pi
        for k in range(int(abs(delta) // math.radians(turn_deg))):
            cur += math.copysign(math.radians(turn_deg), delta)
            phase += 0.05
            poses.append(pose(a, cur + wobble * math.sin(phase), -0.1 + 0.4 * math.sin(phase * 0.63)))
        cur = heading
        n = max(2, int(math.ceil(L / step)))
        for k in range(n):
            p = a + d * k / n
            phase += 0.22
            yaw = heading + wobble * math.sin(phase)
            pitch = -0.1 + 0.4 * math.sin(phase * 0.63)
            poses.append(pose(p, yaw, pitch, rng.normal(0, 0.03)))
        maybe_spin(b, heading)
    return poses


def add_odometry_drift(poses: List[np.ndarray], rng: np.random.Generator, yaw_bias_deg: float = 0.006,
                       yaw_rw_deg: float = 0.03, scale_err: float = 0.004) -> List[np.ndarray]:
    """Emulate VIO drift: random-walk + biased yaw, small scale error, applied incrementally."""
    out = [poses[0].copy()]
    yaw_err = 0.0
    for prev, cur in zip(poses[:-1], poses[1:]):
        yaw_err += math.radians(rng.normal(yaw_bias_deg, yaw_rw_deg))
        dp = cur[:3, 3] - prev[:3, 3]
        Rz = _rotz(yaw_err)
        T = np.eye(4)
        T[:3, 3] = out[-1][:3, 3] + Rz @ dp * (1 + scale_err) + rng.normal(0, 0.0007, 3)
        T[:3, :3] = Rz @ cur[:3, :3]
        out.append(T)
    return out


def lidar_noise(depth: np.ndarray, rng: np.random.Generator):
    d = depth.copy()
    valid = d > 0
    sigma = 0.003 + 0.004 * d
    d = d + rng.normal(0, 1, d.shape) * sigma + rng.normal(0, 0.0015)
    gx = np.abs(cv2.Sobel(depth, cv2.CV_32F, 1, 0, ksize=3))
    gy = np.abs(cv2.Sobel(depth, cv2.CV_32F, 0, 1, ksize=3))
    edge = (np.maximum(gx, gy) > 0.4) & valid
    # flying pixels at depth discontinuities
    blur = cv2.blur(depth, (3, 3))
    fly = edge & (rng.random(d.shape) < 0.4)
    d[fly] = blur[fly]
    conf = np.full(d.shape, 2, np.uint8)
    conf[(depth > 3.2) | (np.maximum(gx, gy) > 0.15)] = 1
    conf[edge | (depth > 4.6)] = 0
    d[~valid | (depth > 5.0)] = 0
    conf[d == 0] = 0
    return d.astype(np.float32), conf


def mono_depth_emulation(depth: np.ndarray, rng: np.random.Generator, global_scale: float) -> np.ndarray:
    d = np.where(depth > 0, depth, 9.0).astype(np.float32)
    h, w = d.shape
    low = cv2.resize(rng.normal(0, 1, (6, 8)).astype(np.float32), (w, h), interpolation=cv2.INTER_CUBIC)
    d = d * global_scale * (1 + rng.normal(0, 0.012)) * (1 + 0.025 * low)
    d = cv2.GaussianBlur(d, (0, 0), 1.6)
    return d.astype(np.float16)


def _to_arkit(T_int: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = W_AR2INT.T @ T_int[:3, :3] @ F_CV2AR
    T[:3, 3] = W_AR2INT.T @ T_int[:3, 3]
    return T


def _quat(R: np.ndarray):
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(R).as_quat()  # x y z w


def _world_offset(rng) -> np.ndarray:
    """Captures never start axis-aligned: apply an arbitrary yaw + offset like ARKit's session origin."""
    G = np.eye(4)
    G[:3, :3] = _rotz(rng.uniform(-math.pi, math.pi))
    G[:3, 3] = [rng.uniform(-2, 2), rng.uniform(-2, 2), 0]
    return G


def write_stray(scene: Scene, out: Path, path, spins, seed: int, drift: bool = True) -> Dict:
    rng = np.random.default_rng(seed)
    out.mkdir(parents=True, exist_ok=True)
    (out / "depth").mkdir(exist_ok=True)
    (out / "confidence").mkdir(exist_ok=True)
    true = walk_trajectory(path, spins, rng)
    est = add_odometry_drift(true, rng) if drift else [t.copy() for t in true]
    G = _world_offset(rng)
    Kr = intrinsics(RGB_W, RGB_H)
    Kd = Kr * (DEPTH_W / RGB_W)
    Kd[2, 2] = 1
    vw = cv2.VideoWriter(str(out / "rgb.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 10, (RGB_W, RGB_H))
    with open(out / "odometry.csv", "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["timestamp", "frame", "x", "y", "z", "qx", "qy", "qz", "qw"])
        for i, (Tt, Te) in enumerate(zip(true, est)):
            depth, _, _ = render(scene, Kd, DEPTH_W, DEPTH_H, Tt, want_rgb=False)
            _, rgb, _ = render(scene, Kr, RGB_W, RGB_H, Tt)
            dn, conf = lidar_noise(depth, rng)
            cv2.imwrite(str(out / "depth" / f"{i:06d}.png"), np.round(dn * 1000).astype(np.uint16))
            cv2.imwrite(str(out / "confidence" / f"{i:06d}.png"), conf)
            vw.write(rgb[:, :, ::-1])
            Ta = _to_arkit(G @ Te)
            q = _quat(Ta[:3, :3])
            wr.writerow([f"{i * 0.1:.4f}", i, *[f"{v:.6f}" for v in Ta[:3, 3]], *[f"{v:.7f}" for v in q]])
    vw.release()
    np.savetxt(out / "camera_matrix.csv", Kr, delimiter=",", fmt="%.6f")
    with open(out / "imu.csv", "w") as f:
        f.write("timestamp, a_x, a_y, a_z, alpha_x, alpha_y, alpha_z\n")
        for i in range(len(true)):
            f.write(f"{i * 0.1:.4f}, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0\n")
    (out / "SYNTHETIC").write_text("synthetic capture generated by propscan.synth (seed=%d)\n" % seed)
    return {"frames": len(true), "true_poses": [t.tolist() for t in true]}


def write_video(scene: Scene, out: Path, path, spins, seed: int) -> Dict:
    rng = np.random.default_rng(seed)
    out.mkdir(parents=True, exist_ok=True)
    (out / "synthetic_depth").mkdir(exist_ok=True)
    # a person, not a tripod: operator chest height differs from the 1.40 m prior the pipeline uses
    true = walk_trajectory(path, spins, rng, step=0.08, spin_frames=56, height=1.40 + rng.normal(0, 0.07),
                           wobble=0.35, turn_deg=8.0, continuous=True)
    Kr = intrinsics(RGB_W, RGB_H)
    Kd = Kr * (DEPTH_W / RGB_W)
    Kd[2, 2] = 1
    gscale = 1.0 + rng.normal(0, 0.025)
    vw = cv2.VideoWriter(str(out / "walkthrough.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 10, (RGB_W, RGB_H))
    for i, Tt in enumerate(true):
        depth, _, _ = render(scene, Kd, DEPTH_W, DEPTH_H, Tt, want_rgb=False)
        _, rgb, _ = render(scene, Kr, RGB_W, RGB_H, Tt)
        vw.write(rgb[:, :, ::-1])
        np.save(out / "synthetic_depth" / f"{i:06d}.npy", mono_depth_emulation(depth, rng, gscale))
    vw.release()
    # ground-truth trajectory for diagnostics only; outside capture/ so the pipeline never sees it
    np.save(out.parent / "gt_trajectory.npy", np.stack(true))
    (out / "capture.json").write_text(json.dumps({"f35": F35}))
    (out / "SYNTHETIC").write_text(f"synthetic capture (seed={seed}, emulated mono-depth scale={gscale:.4f})\n")
    return {"frames": len(true), "mono_scale": gscale}


def _exif_bytes(f35: float) -> bytes:
    ex = Image.Exif()
    ex[0x010F] = "Apple"
    ex[0x0110] = "iPhone 15 (synthetic)"
    ifd = ex.get_ifd(0x8769)
    ifd[0xA405] = int(f35)          # FocalLengthIn35mmFilm
    from PIL.TiffImagePlugin import IFDRational
    ifd[0x920A] = IFDRational(61, 10) if f35 >= 20 else IFDRational(27, 10)  # FocalLength (mm)
    return ex.tobytes()


def write_photos(scene: Scene, out: Path, stations: Dict, seed: int, n_per_room: int = 8,
                 rooms: Sequence[str] = None) -> Dict:
    rng = np.random.default_rng(seed)
    W, H = 768, 576
    K = intrinsics(W, H, F35_ULTRAWIDE)
    Kd = intrinsics(W // 2, H // 2, F35_ULTRAWIDE)
    meta = {}
    for room, st in stations.items():
        if rooms and room not in rooms:
            continue
        d = out / room
        (d / "synthetic_depth").mkdir(parents=True, exist_ok=True)
        gscale = 1.0 + rng.normal(0, 0.04)
        n = n_per_room if room != "hall" else max(4, n_per_room - 2)
        yaw0 = rng.uniform(0, 2 * math.pi)
        h_cam = 1.40 + rng.normal(0, 0.07)
        for k in range(n):
            T = np.eye(4)
            T[:3, :3] = look_rotation(yaw0 + 2 * math.pi * k / n + rng.normal(0, 0.05), rng.normal(0.02, 0.04), rng.normal(0, 0.02))
            T[:3, 3] = [st[0] + rng.normal(0, 0.06), st[1] + rng.normal(0, 0.06), h_cam + rng.normal(0, 0.03)]
            depth, _, _ = render(scene, Kd, W // 2, H // 2, T, want_rgb=False)
            _, rgb, _ = render(scene, K, W, H, T)
            name = f"IMG_{1000 + k:04d}"
            Image.fromarray(rgb).save(d / f"{name}.JPG", quality=92, exif=_exif_bytes(F35_ULTRAWIDE))
            np.save(d / "synthetic_depth" / f"{name}.npy", mono_depth_emulation(depth, rng, gscale))
        meta[room] = {"photos": n, "mono_scale": gscale}
    (out / "SYNTHETIC").write_text(f"synthetic photo capture (seed={seed})\n")
    return meta


def ground_truth(scene: Scene, rooms: Sequence[str] = None) -> Dict:
    sp = scene.spec
    out_rooms = []
    for r in sp["rooms"]:
        if rooms and r["name"] not in rooms:
            continue
        ops = []
        for o in sp.get("openings", []):
            if r["name"] in (o.get("a"), o.get("b")):
                h = scene.door_h if o["type"] == "door" else o["top"] - o["sill"]
                ops.append({"type": o["type"], "width": o["width"], "height": round(h, 3),
                            "center": o["center"], "to": o.get("b") if o.get("a") == r["name"] else o.get("a")})
        out_rooms.append({"name": r["name"], "polygon": r["polygon"], "ceiling_height": r["ceiling"], "openings": ops})
    names = {r["name"] for r in out_rooms}
    adj = sorted({tuple(sorted((o["a"], o["b"]))) for o in sp.get("openings", [])
                  if o.get("b") and o["a"] in names and o["b"] in names})
    dmg = [{"class": d["cls"], "room": d["room"], "surface": d["surface"], "size": d["size"]}
           for d in sp.get("damage", []) if not rooms or d["room"] in rooms]
    return {"source": "synthetic-exact", "rooms": out_rooms, "adjacency": [list(a) for a in adj], "damage": dmg,
            "mirrors": [m for m in sp.get("mirrors", []) if not rooms or m["room"] in rooms]}


def generate_all(scene_path: str, out_dir: str, quick: bool = False, force: bool = False) -> Path:
    scene = Scene.from_yaml(scene_path)
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    cap = scene.spec["captures"]
    rep = cap["repeat_room"]
    rep_poly = np.asarray(scene.rooms[rep]["polygon"], float)
    c = rep_poly.mean(axis=0)
    loop = [c, c + [-0.8, -0.3], c + [0.8, -0.3], c + [0.8, 0.4], c + [-0.8, 0.4], c + [-0.8, -0.3], c]
    loop = [p.tolist() for p in loop]
    manifest = {"scene": scene.spec["name"], "synthetic": True, "captures": []}

    def add(cid, tier, rel, gt_rooms, group=None):
        gt = ground_truth(scene, gt_rooms)
        gpath = out / cid / "gt.yaml"
        gpath.parent.mkdir(parents=True, exist_ok=True)
        with open(gpath, "w") as f:
            yaml.safe_dump(gt, f, sort_keys=False)
        manifest["captures"].append({"id": cid, "tier": tier, "path": rel, "gt": f"{cid}/gt.yaml",
                                     "repeat_group": group})

    def todo(cid):
        done = (out / cid / "capture" / "SYNTHETIC").exists()
        if done and not force:
            print(f"[synth] {cid}: exists, skipping")
        return force or not done

    if todo("lidar_full"):
        print("[synth] lidar multi-room ...")
        write_stray(scene, out / "lidar_full" / "capture", cap["lidar_path"], cap["spins"], seed=11)
    add("lidar_full", "lidar", "lidar_full/capture", None)
    for k, seed in enumerate([21, 22]):
        cid = f"lidar_{rep}_r{k + 1}"
        if todo(cid):
            print(f"[synth] {cid} ...")
            write_stray(scene, out / cid / "capture", loop, [c.tolist()], seed=seed)
        add(cid, "lidar", f"{cid}/capture", [rep], f"{rep}-lidar")
    if not quick:
        if todo("video_full"):
            print("[synth] video multi-room ...")
            write_video(scene, out / "video_full" / "capture", cap["lidar_path"], cap["spins"], seed=31)
        add("video_full", "video", "video_full/capture", None)
        if todo("photo_full"):
            print("[synth] photos per room ...")
            write_photos(scene, out / "photo_full" / "capture", cap["photo_stations"], seed=41)
        add("photo_full", "photo", "photo_full/capture", None)
        for k, seed in enumerate([51, 52]):
            cid = f"photo_{rep}_r{k + 1}"
            if todo(cid):
                write_photos(scene, out / cid / "capture", cap["photo_stations"], seed=seed, rooms=[rep])
            add(cid, "photo", f"{cid}/capture", [rep], f"{rep}-photo")
    with open(out / "manifest.yaml", "w") as f:
        yaml.safe_dump(manifest, f, sort_keys=False)
    return out
