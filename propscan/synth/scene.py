"""Procedural indoor scene + vectorised ray caster.

The scene is a set of vertical quads (walls, jambs, furniture sides, mirrors) and horizontal
slabs (floors, ceilings, soffits, furniture tops). Rays that hit a mirror are reflected once, so
the synthetic LiDAR reproduces the real failure mode: phantom geometry behind the mirror plane.
Glass windows return nothing (as LiDAR mostly does through clear glass).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np
import shapely
import yaml
from shapely.geometry import Polygon, box as sbox


@dataclass
class VFace:
    p0: np.ndarray
    p1: np.ndarray
    z0: float
    z1: float
    color: Tuple[int, int, int]
    kind: str = "wall"            # wall | jamb | furniture | mirror
    room: str = ""
    holes: List[Tuple[float, float, float, float]] = field(default_factory=list)  # s0, s1, z0, z1
    damage: List[Dict] = field(default_factory=list)
    fid: int = 0

    @property
    def length(self) -> float:
        return float(np.linalg.norm(self.p1 - self.p0))


@dataclass
class HSlab:
    poly: Polygon
    z: float
    color: Tuple[int, int, int]
    kind: str = "floor"           # floor | ceiling | soffit | furniture
    room: str = ""
    damage: List[Dict] = field(default_factory=list)
    fid: int = 0


class Scene:
    def __init__(self, spec: Dict):
        self.spec = spec
        self.t = float(spec.get("wall_thickness", 0.12))
        self.door_h = float(spec.get("door_height", 2.03))
        self.rooms = {r["name"]: r for r in spec["rooms"]}
        self.vfaces: List[VFace] = []
        self.slabs: List[HSlab] = []
        self._build()

    @classmethod
    def from_yaml(cls, path: str) -> "Scene":
        with open(path) as f:
            return cls(yaml.safe_load(f))

    # ------------------------------------------------------------------ build
    def _build(self) -> None:
        sp = self.spec
        for r in sp["rooms"]:
            poly = np.asarray(r["polygon"], float)
            col = tuple(r.get("color", [210, 210, 210]))
            n = len(poly)
            for i in range(n):
                self.vfaces.append(VFace(poly[i], poly[(i + 1) % n], 0.0, float(r["ceiling"]), col, "wall", r["name"]))
            self.slabs.append(HSlab(Polygon(poly), 0.0, (165, 130, 95), "floor", r["name"]))
            self.slabs.append(HSlab(Polygon(poly), float(r["ceiling"]), (238, 238, 236), "ceiling", r["name"]))

        for o in sp.get("openings", []):
            self._add_opening(o)
        for m in sp.get("mirrors", []):
            self._add_mirror(m)
        for fu in sp.get("furniture", []):
            x0, y0, x1, y1 = fu["box"]
            h = float(fu["height"])
            col = tuple(fu.get("color", [120, 120, 120]))
            c = [np.array(p, float) for p in [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]]
            for i in range(4):
                self.vfaces.append(VFace(c[i], c[(i + 1) % 4], 0.0, h, col, "furniture", fu["room"]))
            self.slabs.append(HSlab(sbox(x0, y0, x1, y1), h, col, "furniture", fu["room"]))
        for d in sp.get("damage", []):
            self._add_damage(d)
        for i, f in enumerate(self.vfaces):
            f.fid = i
        for j, s in enumerate(self.slabs):
            s.fid = len(self.vfaces) + j
            shapely.prepare(s.poly)

    def _faces_on_line(self, center, axis, max_off):
        """Wall faces parallel to `axis` passing within max_off of center, containing center."""
        out = []
        c = np.asarray(center, float)
        for f in self.vfaces:
            if f.kind != "wall":
                continue
            d = f.p1 - f.p0
            along = 1 if axis == "y" else 0
            if abs(d[1 - along]) > 1e-9:
                continue
            off = abs(f.p0[1 - along] - c[1 - along])
            lo, hi = sorted([f.p0[along], f.p1[along]])
            if off <= max_off and lo - 1e-6 <= c[along] <= hi + 1e-6:
                out.append(f)
        return out

    def _cut(self, f: VFace, center, axis, width, z0, z1):
        along = 1 if axis == "y" else 0
        d = f.p1 - f.p0
        sign = np.sign(d[along])
        s_c = (center[along] - f.p0[along]) * sign
        f.holes.append((s_c - width / 2, s_c + width / 2, z0, z1))

    def _add_opening(self, o: Dict) -> None:
        c = np.asarray(o["center"], float)
        axis, w = o["axis"], float(o["width"])
        if o["type"] == "door":
            z0, z1 = 0.0, self.door_h
        else:
            z0, z1 = float(o["sill"]), float(o["top"])
        faces = self._faces_on_line(c, axis, self.t / 2 + 0.02)
        if not faces:
            raise ValueError(f"opening {o} does not lie on any wall")
        for f in faces:
            self._cut(f, c, axis, w, z0, z1)
        along = 1 if axis == "y" else 0
        perp = 1 - along
        offs = sorted(f.p0[perp] for f in faces)
        if len(offs) == 1:  # exterior: extrude reveal outwards by wall thickness
            room_poly = Polygon(self.rooms[o["a"]]["polygon"])
            probe = c.copy()
            probe[perp] += 0.05
            out_sign = -1.0 if room_poly.contains(shapely.Point(probe)) else 1.0
            offs = [offs[0], offs[0] + out_sign * self.t]
        lo_p, hi_p = min(offs), max(offs)
        col = (200, 200, 200)
        for e in (-w / 2, w / 2):
            a = np.zeros(2)
            b = np.zeros(2)
            a[along] = b[along] = c[along] + e
            a[perp], b[perp] = lo_p, hi_p
            self.vfaces.append(VFace(a, b, z0, z1, col, "jamb", o.get("a", "")))
        rect = [None, None]
        rect[along] = (c[along] - w / 2, c[along] + w / 2)
        rect[perp] = (lo_p, hi_p)
        rpoly = sbox(rect[0][0], rect[1][0], rect[0][1], rect[1][1])
        self.slabs.append(HSlab(rpoly, z1, col, "soffit", o.get("a", "")))
        if o["type"] == "door":
            self.slabs.append(HSlab(rpoly, 0.0, (165, 130, 95), "floor", o.get("a", "")))
        else:
            self.slabs.append(HSlab(rpoly, z0, col, "soffit", o.get("a", "")))

    def _add_mirror(self, m: Dict) -> None:
        c = np.asarray(m["center"], float)
        axis, w = m["axis"], float(m["width"])
        faces = [f for f in self._faces_on_line(c, axis, 0.01) if f.room == m["room"]]
        f = faces[0]
        self._cut(f, c, axis, w, float(m["z0"]), float(m["z1"]))
        along = 1 if axis == "y" else 0
        a, b = c.copy(), c.copy()
        a[along] -= w / 2
        b[along] += w / 2
        # nudge 1 mm into the room
        rp = Polygon(self.rooms[m["room"]]["polygon"])
        nrm = np.zeros(2)
        nrm[1 - along] = 1.0
        if not rp.contains(shapely.Point(c + nrm * 0.05)):
            nrm = -nrm
        self.vfaces.append(VFace(a + nrm * 0.001, b + nrm * 0.001, float(m["z0"]), float(m["z1"]), (180, 190, 200), "mirror", m["room"]))

    def _add_damage(self, d: Dict) -> None:
        if d["surface"] == "ceiling":
            for s in self.slabs:
                if s.kind == "ceiling" and s.room == d["room"]:
                    s.damage.append(dict(d))
            return
        wp = np.asarray(d["wall_point"], float)
        best, bd = None, 1e9
        for f in self.vfaces:
            if f.kind != "wall" or f.room != d["room"]:
                continue
            e = f.p1 - f.p0
            u = np.clip(np.dot(wp - f.p0, e) / np.dot(e, e), 0, 1)
            dist = np.linalg.norm(f.p0 + u * e - wp)
            if dist < bd:
                best, bd, bu = f, dist, u
        dd = dict(d)
        dd["s_center"] = float(bu * best.length)
        best.damage.append(dd)

    # ------------------------------------------------------------ ray casting
    def cast(self, origin: np.ndarray, dirs: np.ndarray, allow_mirror: bool = True):
        """Returns (dist, face_id, hit_point, uv) for rays origin + t*dirs (dirs unit, (N,3))."""
        N = dirs.shape[0]
        best_t = np.full(N, np.inf)
        best_id = np.full(N, -1, np.int32)
        best_uv = np.zeros((N, 2))
        o2 = origin[:2]
        d2 = dirs[:, :2]
        for f in self.vfaces:
            if f.kind == "mirror" and not allow_mirror:
                continue
            e = f.p1 - f.p0
            L = np.linalg.norm(e)
            denom = d2[:, 0] * e[1] - d2[:, 1] * e[0]
            ok = np.abs(denom) > 1e-12
            w = f.p0 - o2
            with np.errstate(divide="ignore", invalid="ignore"):
                t = (w[0] * e[1] - w[1] * e[0]) / denom
                u = (w[0] * d2[:, 1] - w[1] * d2[:, 0]) / denom
            z = origin[2] + t * dirs[:, 2]
            m = ok & (t > 1e-4) & (u >= 0) & (u <= 1) & (z >= f.z0) & (z <= f.z1) & (t < best_t)
            if f.holes and m.any():
                s = u * L
                for (s0, s1, hz0, hz1) in f.holes:
                    m &= ~((s > s0) & (s < s1) & (z > hz0) & (z < hz1))
            if m.any():
                best_t[m] = t[m]
                best_id[m] = f.fid
                best_uv[m, 0] = u[m] * L
                best_uv[m, 1] = z[m]
        for s in self.slabs:
            with np.errstate(divide="ignore", invalid="ignore"):
                t = (s.z - origin[2]) / dirs[:, 2]
            cand = np.isfinite(t) & (t > 1e-4) & (t < best_t)
            if not cand.any():
                continue
            idx = np.nonzero(cand)[0]
            px = origin[0] + t[idx] * dirs[idx, 0]
            py = origin[1] + t[idx] * dirs[idx, 1]
            inside = shapely.contains_xy(s.poly, px, py)
            idx = idx[inside]
            best_t[idx] = t[idx]
            best_id[idx] = s.fid
            best_uv[idx, 0] = px[inside]
            best_uv[idx, 1] = py[inside]
        hit = origin[None, :] + dirs * np.where(np.isfinite(best_t), best_t, 0)[:, None]

        if allow_mirror:
            mirror_ids = [f.fid for f in self.vfaces if f.kind == "mirror"]
            mm = np.isin(best_id, mirror_ids)
            if mm.any():
                idx = np.nonzero(mm)[0]
                refl_d = dirs[idx].copy()
                # mirrors are axis aligned: reflect the direction component normal to the face
                for fid in np.unique(best_id[idx]):
                    f = self.vfaces[fid]
                    e = f.p1 - f.p0
                    nrm_axis = 0 if abs(e[0]) < 1e-9 else 1
                    sel = best_id[idx] == fid
                    refl_d[sel, nrm_axis] *= -1
                sub_t = np.empty(len(idx))
                sub_id = np.empty(len(idx), np.int32)
                sub_uv = np.empty((len(idx), 2))
                for i0 in range(0, len(idx), 2048):
                    ch = idx[i0:i0 + 2048]
                    t2, id2, uv2 = self._cast_multi_origin(hit[ch], refl_d[i0:i0 + 2048])
                    sub_t[i0:i0 + len(ch)] = t2
                    sub_id[i0:i0 + len(ch)] = id2
                    sub_uv[i0:i0 + len(ch)] = uv2
                best_t[idx] = best_t[idx] + sub_t
                best_id[idx] = sub_id
                best_uv[idx] = sub_uv
                hit = origin[None, :] + dirs * np.where(np.isfinite(best_t), best_t, 0)[:, None]
        return best_t, best_id, hit, best_uv

    def _cast_multi_origin(self, origins: np.ndarray, dirs: np.ndarray):
        """Exact cast with per-ray origins (used only for mirror bounces)."""
        N = dirs.shape[0]
        best_t = np.full(N, np.inf)
        best_id = np.full(N, -1, np.int32)
        best_uv = np.zeros((N, 2))
        o2 = origins[:, :2]
        d2 = dirs[:, :2]
        for f in self.vfaces:
            if f.kind == "mirror":
                continue
            e = f.p1 - f.p0
            L = np.linalg.norm(e)
            denom = d2[:, 0] * e[1] - d2[:, 1] * e[0]
            w = f.p0[None, :] - o2
            with np.errstate(divide="ignore", invalid="ignore"):
                t = (w[:, 0] * e[1] - w[:, 1] * e[0]) / denom
                u = (w[:, 0] * d2[:, 1] - w[:, 1] * d2[:, 0]) / denom
            z = origins[:, 2] + t * dirs[:, 2]
            m = np.isfinite(t) & (t > 1e-3) & (u >= 0) & (u <= 1) & (z >= f.z0) & (z <= f.z1) & (t < best_t)
            if f.holes and m.any():
                s = u * L
                for (s0, s1, hz0, hz1) in f.holes:
                    m &= ~((s > s0) & (s < s1) & (z > hz0) & (z < hz1))
            best_t[m] = t[m]
            best_id[m] = f.fid
            best_uv[m, 0] = (u * L)[m]
            best_uv[m, 1] = z[m]
        for s in self.slabs:
            with np.errstate(divide="ignore", invalid="ignore"):
                t = (s.z - origins[:, 2]) / dirs[:, 2]
            cand = np.isfinite(t) & (t > 1e-3) & (t < best_t)
            if not cand.any():
                continue
            idx = np.nonzero(cand)[0]
            px = origins[idx, 0] + t[idx] * dirs[idx, 0]
            py = origins[idx, 1] + t[idx] * dirs[idx, 1]
            inside = shapely.contains_xy(s.poly, px, py)
            idx = idx[inside]
            best_t[idx] = t[idx]
            best_id[idx] = s.fid
            best_uv[idx, 0] = px[inside]
            best_uv[idx, 1] = py[inside]
        return best_t, best_id, best_uv

    def face(self, fid: int):
        if fid < len(self.vfaces):
            return self.vfaces[fid]
        return self.slabs[fid - len(self.vfaces)]

    # ---------------------------------------------------------------- shading
    def shade(self, fid: np.ndarray, uv: np.ndarray, dirs: np.ndarray) -> np.ndarray:
        N = len(fid)
        out = np.zeros((N, 3), np.float32)
        nf = len(self.vfaces)
        for u_id in np.unique(fid):
            sel = fid == u_id
            if u_id < 0:
                out[sel] = (235, 240, 250)  # sky through glass
                continue
            f = self.face(int(u_id))
            base = np.array(f.color, np.float32)
            s, z = uv[sel, 0], uv[sel, 1]
            tex = 0.80 + 0.12 * _value_noise(s, z, 0.09, u_id) + 0.08 * _value_noise(s, z, 0.025, u_id + 7)
            col = base[None, :] * tex[:, None]
            if isinstance(f, HSlab) and f.kind == "floor":
                seam = (np.mod(s, 0.18) < 0.006)
                col[seam] *= 0.6
            if isinstance(f, VFace):
                e = (f.p1 - f.p0) / max(f.length, 1e-9)
                n = np.array([-e[1], e[0], 0.0])
                lam = 0.55 + 0.45 * np.abs(dirs[sel] @ n)
                if f.kind == "wall":
                    col = _posters(col, s, z, u_id)
            else:
                lam = 0.55 + 0.45 * np.abs(dirs[sel, 2])
            col *= lam[:, None]
            for d in f.damage:
                col = _apply_damage(col, s, z, d, isinstance(f, HSlab))
            out[sel] = col
        return np.clip(out, 0, 255)


def _hash(ix, iz, seed):
    h = (ix.astype(np.int64) * 73856093) ^ (iz.astype(np.int64) * 19349663) ^ (int(seed) * 83492791)
    h = (h ^ (h >> 13)) * 1274126177
    return ((h ^ (h >> 16)) & 0xFFFF).astype(np.float32) / 65535.0


def _value_noise(s, z, cell, seed):
    gs, gz = s / cell, z / cell
    i0, j0 = np.floor(gs), np.floor(gz)
    fs, fz = gs - i0, gz - j0
    fs = fs * fs * (3 - 2 * fs)
    fz = fz * fz * (3 - 2 * fz)
    a = _hash(i0, j0, seed)
    b = _hash(i0 + 1, j0, seed)
    c = _hash(i0, j0 + 1, seed)
    d = _hash(i0 + 1, j0 + 1, seed)
    return (a * (1 - fs) + b * fs) * (1 - fz) + (c * (1 - fs) + d * fs) * fz - 0.5


def _posters(col, s, z, seed):
    """A couple of framed pictures per wall: realistic texture + a damage-detector distractor."""
    rng = np.random.default_rng(int(seed) * 31 + 5)
    for _ in range(2):
        s0 = rng.uniform(0.3, 2.5)
        z0 = rng.uniform(1.2, 1.6)
        w, h = rng.uniform(0.3, 0.6), rng.uniform(0.25, 0.45)
        m = (s > s0) & (s < s0 + w) & (z > z0) & (z < z0 + h)
        if m.any():
            pc = rng.uniform(40, 220, 3).astype(np.float32)
            frame = m & ((s < s0 + 0.02) | (s > s0 + w - 0.02) | (z < z0 + 0.02) | (z > z0 + h - 0.02))
            col[m] = pc * (0.75 + 0.5 * (_value_noise(s[m], z[m], 0.05, seed + 99) + 0.5))[:, None]
            col[frame] = (30, 30, 30)
    return col


def _apply_damage(col, s, z, d, horizontal):
    if horizontal:
        cx, cz = d["center"]
    else:
        cx, cz = d["s_center"], d["z_center"]
    w, h = d["size"]
    if d["cls"] == "crack":
        m = (s > cx - w / 2) & (s < cx + w / 2)
        path = cz + 0.04 * np.sin((s - cx) * 23.0) + 0.015 * np.sin((s - cx) * 71.0)
        m &= np.abs(z - path) < 0.005
        col[m] = col[m] * 0.25
        return col
    r = np.sqrt(((s - cx) / (w / 2)) ** 2 + ((z - cz) / (h / 2)) ** 2)
    m = r < 1.0
    if not m.any():
        return col
    if d["cls"] == "water_stain":
        a = (0.45 * (1 - r[m] ** 2) ** 0.4 + 0.35 * np.exp(-((r[m] - 0.9) / 0.06) ** 2))[:, None]
        stain = np.array([150, 112, 60], np.float32)
        col[m] = col[m] * (1 - a) + stain * a
    elif d["cls"] == "mold":
        speck = _value_noise(s[m], z[m], 0.012, 4242) + 0.5
        dens = 0.9 - 0.6 * r[m]
        dark = speck < dens
        mm = np.nonzero(m)[0][dark]
        col[mm] = np.array([38, 46, 32], np.float32) + 10 * np.random.default_rng(0).random((len(mm), 1))
    return col


def look_rotation(yaw: float, pitch: float, roll: float = 0.0) -> np.ndarray:
    """Camera-to-world rotation (OpenCV camera) for a world z-up frame."""
    f = np.array([np.cos(pitch) * np.cos(yaw), np.cos(pitch) * np.sin(yaw), np.sin(pitch)])
    up = np.array([0.0, 0.0, 1.0])
    r = np.cross(f, up)
    r /= np.linalg.norm(r)
    dn = np.cross(f, r)
    R = np.stack([r, dn, f], axis=1)
    if roll:
        c, s_ = np.cos(roll), np.sin(roll)
        Rz = np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1]])
        R = R @ Rz
    return R


def pixel_rays(K: np.ndarray, W: int, H: int) -> np.ndarray:
    u, v = np.meshgrid(np.arange(W, dtype=float), np.arange(H, dtype=float))  # OpenCV pixel-centre convention
    pts = np.stack([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], np.ones_like(u)], -1).reshape(-1, 3)
    return pts  # not normalised: z = 1


def render(scene: Scene, K: np.ndarray, W: int, H: int, T_wc: np.ndarray, want_rgb: bool = True):
    """Returns depth (HxW, metres along optical axis, 0 = no return), rgb (HxWx3 uint8 or None), face ids."""
    rays_c = pixel_rays(K, W, H)
    norm = np.linalg.norm(rays_c, axis=1)
    dirs = (T_wc[:3, :3] @ (rays_c / norm[:, None]).T).T
    t, fid, hit, uv = scene.cast(T_wc[:3, 3], dirs)
    depth = np.where(np.isfinite(t), t / norm, 0.0).reshape(H, W).astype(np.float32)
    rgb = None
    if want_rgb:
        rgb = scene.shade(fid, uv, dirs).reshape(H, W, 3).astype(np.uint8)
    return depth, rgb, fid.reshape(H, W)
