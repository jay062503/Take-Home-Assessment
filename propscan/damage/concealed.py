"""Concealed-damage flags: explicit, auditable rules. Every flag names the rule that fired."""
from __future__ import annotations

from typing import Dict, List

import numpy as np

from ..types import Room

RULES = {
    "CD-01": "Water stain on a ceiling -> possible active leak above (roof, plumbing, or wet room overhead); "
             "ceiling cavity insulation/joists may be wet beyond the visible stain.",
    "CD-02": "Water stain whose lower edge is within 0.30 m of the floor -> wicking from floor-level water; "
             "wall cavity, bottom plate and the far face of the wall are likely affected.",
    "CD-03": "Elevated wall water stain (lower edge > 0.30 m) -> leak inside the wall cavity "
             "(supply/drain line, or window flashing when below a window).",
    "CD-04": "Visible mold -> sustained moisture; growth behind the board is likely larger than the visible area. "
             ">= 0.93 m^2 (10 sq ft, US EPA guidance) requires professional remediation.",
    "CD-05": "Crack >= 0.5 m long or running from an opening corner -> possible structural movement / settlement.",
    "CD-06": "Moisture damage on an interior wall -> the same wall's face in the adjacent room is flagged for inspection.",
}


def _wall_of(rooms: List[Room], sid: str):
    for r in rooms:
        for w in r.walls:
            if w.id == sid:
                return r, w
    return None, None


def _twin_face(rooms: List[Room], room: Room, wall, max_gap: float = 0.35):
    """The wall face on the other side of `wall` (interior partition), if reconstructed."""
    lo, hi = sorted([wall.start[1 - wall.axis], wall.end[1 - wall.axis]])
    for r in rooms:
        if r.id == room.id:
            continue
        for w in r.walls:
            if w.axis != wall.axis or w.inward == wall.inward:
                continue
            gap = (w.coord.value - wall.coord.value) * (-wall.inward)
            if 0 < gap < max_gap:
                l2, h2 = sorted([w.start[1 - w.axis], w.end[1 - w.axis]])
                if min(hi, h2) - max(lo, l2) > 0.2:
                    return r, w
    return None, None


def concealed_flags(rooms: List[Room], damage: List[Dict]) -> List[Dict]:
    flags = []

    def add(rule, d, surface, severity, extra=""):
        flags.append({"id": f"cd_{len(flags)}", "rule_id": rule, "rule": RULES[rule], "surface_id": surface,
                      "damage_ids": [d["id"]], "severity": severity, "rationale": extra})

    for d in damage:
        cls = d["class"]
        bb = d["bbox_surface"]
        if d["surface_kind"] == "ceiling" and cls in ("water_stain", "mold"):
            add("CD-01", d, d["surface_id"], "high", f"{cls} {d['area'].value:.2f} m2 on ceiling")
        if d["surface_kind"] == "wall" and cls == "water_stain":
            if bb["v0"] <= 0.30:
                add("CD-02", d, d["surface_id"], "high", f"lower edge {bb['v0']:.2f} m above floor")
            else:
                r, w = _wall_of(rooms, d["surface_id"])
                below_window = False
                if r is not None:
                    for op in r.openings:
                        if op.type == "window" and r.walls[op.wall_index].id == w.id:
                            c, half = op.center_offset.value, op.width.value / 2
                            if bb["u1"] > c - half and bb["u0"] < c + half and bb["v1"] <= op.sill.value + 0.15:
                                below_window = True
                add("CD-03", d, d["surface_id"], "medium",
                    "below a window: check flashing/sill" if below_window else f"lower edge {bb['v0']:.2f} m")
        if cls == "mold":
            add("CD-04", d, d["surface_id"], "high" if d["area"].value >= 0.93 else "medium",
                f"visible area {d['area'].value:.2f} m2")
        if cls == "crack":
            L = d.get("length").value if d.get("length") is not None else max(d["extent_w"].value, d["extent_h"].value)
            if L >= 0.5:
                add("CD-05", d, d["surface_id"], "medium", f"length {L:.2f} m")
        if d["surface_kind"] == "wall" and cls in ("water_stain", "mold"):
            r, w = _wall_of(rooms, d["surface_id"])
            if r is not None:
                r2, w2 = _twin_face(rooms, r, w)
                if w2 is not None:
                    add("CD-06", d, w2.id, "medium", f"twin face of {w.id} in {r2.id}")
    return flags
