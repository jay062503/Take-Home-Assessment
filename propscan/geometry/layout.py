"""FrameSet -> list of Rooms (+ adjacency) in a gravity/Manhattan-aligned frame."""
from __future__ import annotations

from typing import Dict, List, Optional, Tuple

import numpy as np
import shapely
from shapely.geometry import Point, Polygon

from ..types import FrameSet, Measure, Room, combine_sigma
from .cloud import (Cloud, apply_world_transform, build_cloud, estimate_up, manhattan_yaw, refine_yaw_histogram,
                    rot_to_z, rotz)
from .mirrors import detect_mirrors
from .structure import (ceiling_room_masks, detect_openings, global_levels, opening_world_center,
                        rectilinear_polygon, refine_walls, room_levels, visibility_room_mask)


def align_frameset(fs: FrameSet, cloud: Cloud) -> Dict:
    info = {}
    if not fs.gravity_known:
        up = estimate_up(cloud, fs)
        R = rot_to_z(up)
        apply_world_transform(fs, cloud, R)
        info["gravity_from"] = "horizontal-surface normals"
        info["gravity_correction_deg"] = float(np.degrees(np.arccos(np.clip(up[2], -1, 1))))
    yaw0 = manhattan_yaw(cloud.nrm)
    n = cloud.nrm
    z = cloud.pts[:, 2]
    zf = np.percentile(z[~cloud.far], 2)
    wall = ~cloud.far & ~np.isnan(n[:, 0]) & (np.abs(n[:, 2]) < 0.3) & (z > zf + 1.0)
    yaw = refine_yaw_histogram(cloud.pts[wall, :2], yaw0)
    apply_world_transform(fs, cloud, rotz(-yaw))
    info["manhattan_yaw_deg"] = float(np.degrees(yaw))
    info["manhattan_yaw_refinement_deg"] = float(np.degrees(yaw - yaw0))
    return info


def extract_rooms(fs: FrameSet, cfg: Dict, mode: str, room_prefix: str = "room",
                  cloud: Optional[Cloud] = None, label: Optional[str] = None) -> Tuple[List[Room], List[Dict], Dict]:
    tc = cfg["tiers"][fs.tier]
    g = cfg["geometry"]
    if cloud is None:
        cloud = build_cloud(fs, tc)
        diag = align_frameset(fs, cloud)
    else:
        diag = {}
    floor, ceil = global_levels(cloud)
    diag.update({"global_floor_z": floor, "global_ceiling_z": ceil, "n_points": int(len(cloud.pts))})
    mirrors, drop = detect_mirrors(cloud, floor, ceil, cfg)
    if drop.any():
        cloud = cloud.subset(~drop)
    diag["mirrors"] = mirrors
    diag["phantom_points_removed"] = int(drop.sum())

    res = float(tc["grid"])
    if mode == "ceiling":
        grid, masks = ceiling_room_masks(cloud, floor, ceil, res, float(g["min_room_area"]))
    else:
        station = cloud.cams.mean(axis=0)
        grid, m = visibility_room_mask(cloud, floor, ceil, station, res)
        masks = [m] if m is not None else []
    rooms: List[Room] = []
    for i, m in enumerate(masks):
        rid = f"{room_prefix}_{i}" if label is None else label
        segs = rectilinear_polygon(m, grid, float(tc.get("min_wall", 0.22)), float(tc.get("poly_eps", 0.05)))
        walls = refine_walls(segs, cloud, floor, ceil, tc, rid)
        poly = Polygon([w.start for w in walls])
        if not poly.is_valid or poly.area < float(g["min_room_area"]):
            continue
        fz, cz, H, q = room_levels(poly, cloud, floor, tc)
        walls = refine_walls(segs, cloud, fz.value, cz.value, tc, rid)
        for _ in range(6):  # drop jogs that refinement shrank to (almost) nothing, then refit
            short = [i for i, w in enumerate(walls) if w.length.value < float(tc.get("min_jog", 0.08))]
            if not short or len(segs) <= 4:
                break
            segs = [(o, w.coord.value) for (o, _), w in zip(segs, walls)]
            del segs[short[0]]
            segs = _merge_collinear(segs)
            walls = refine_walls(segs, cloud, fz.value, cz.value, tc, rid)
        poly = Polygon([w.start for w in walls])
        if not poly.is_valid:
            poly = poly.buffer(0)
        area_sigma = combine_sigma(*[w.length.value * w.coord.sigma for w in walls])
        per = sum(w.length.value for w in walls)
        per_sigma = combine_sigma(*[w.length.sigma for w in walls])
        minor = _min_width(poly)
        kind = "connector" if minor < float(g["connector_max_width"]) else "room"
        q.update({"wall_support": [w.support for w in walls], "wall_observed_fraction": [round(w.observed_fraction, 2) for w in walls]})
        rooms.append(Room(id=rid, polygon=np.asarray(poly.exterior.coords)[:-1], walls=walls, floor_z=fz, ceiling_z=cz,
                          ceiling_height=H, floor_area=Measure(poly.area, area_sigma, "m2"),
                          perimeter=Measure(per, per_sigma), kind=kind, label=label or rid, quality=q))
    for r in rooms:
        poly = Polygon(r.polygon).buffer(0.05)
        cam_inside = shapely.contains_xy(poly, cloud.cams[:, 0], cloud.cams[:, 1])
        if not cam_inside.any():
            # photo tier: the station may sit slightly outside a fitted polygon edge; use all cameras
            cam_inside = np.ones(len(cloud.cams), bool)
        r.openings = detect_openings(r, cloud, cam_inside, cfg, tc)
        for mi in mirrors:
            for w in r.walls:
                lo, hi = sorted([w.start[1 - w.axis], w.end[1 - w.axis]])
                if (w.axis == mi["axis"] and abs(w.coord.value - mi["coord"]) < 0.06 and w.inward == mi["facing"]
                        and mi["s0"] < hi and mi["s1"] > lo):
                    r.features.append({"type": "mirror", "wall_id": w.id,
                                       **{k: v for k, v in mi.items() if k not in ("n_points", "axis", "coord", "facing")}})
                    break
    adjacency = link_openings(rooms)
    diag["cloud"] = cloud
    return rooms, adjacency, diag


def _merge_collinear(segs):
    out = list(segs)
    i = 0
    while len(out) > 4 and i < len(out):
        j = (i + 1) % len(out)
        if out[i][0] == out[j][0]:
            out[i] = (out[i][0], (out[i][1] + out[j][1]) / 2)
            del out[j]
            i = 0
            continue
        i += 1
    return out


def _min_width(poly: Polygon) -> float:
    x0, y0, x1, y1 = poly.bounds
    return float(min(x1 - x0, y1 - y0))


def link_openings(rooms: List[Room], reach: float = 0.35) -> List[Dict]:
    polys = {r.id: Polygon(r.polygon).buffer(0.03) for r in rooms}
    adj = {}
    for r in rooms:
        for op in r.openings:
            if op.type not in ("door", "passage"):
                continue
            w = r.walls[op.wall_index]
            c = opening_world_center(r, op)
            nrm = np.zeros(2)
            nrm[w.axis] = -w.inward
            probe = c + nrm * reach
            for other in rooms:
                if other.id != r.id and polys[other.id].contains(Point(probe)):
                    op.connects_to = other.id
                    key = tuple(sorted((r.id, other.id)))
                    adj.setdefault(key, {"a": key[0], "b": key[1], "via": []})["via"].append(op.id)
    return list(adj.values())
