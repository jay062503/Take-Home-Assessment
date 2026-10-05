"""Scope-of-work line items keyed to surfaces, quantities derived from metric extents.

Codes are generic trade categories (not a proprietary estimating catalogue). Quantities carry
intervals propagated from the geometry. Items that depend on an on-site moisture reading are
emitted with `conditional` set, rather than assumed.
"""
from __future__ import annotations

from typing import Dict, List

import numpy as np

from ..types import Measure, Room, combine_sigma

MARGIN = 0.30          # remove/treat beyond visible extent on each side (common remediation practice)
SHEET = 1.2 * 2.4      # drywall sheet area


def wall_net_area(room: Room, wall_index: int) -> Measure:
    w = room.walls[wall_index]
    H = room.ceiling_height
    gross = w.length.value * H.value
    sig = combine_sigma(w.length.sigma * H.value, H.sigma * w.length.value)
    minus = 0.0
    for op in room.openings:
        if op.wall_index == wall_index:
            minus += op.width.value * op.height.value
    return Measure(max(gross - minus, 0.0), sig, "m2")


def _surface(rooms: List[Room], sid: str):
    for r in rooms:
        for i, w in enumerate(r.walls):
            if w.id == sid:
                return r, i
        if sid == f"{r.id}/ceiling":
            return r, None
    return None, None


def scope_items(rooms: List[Room], damage: List[Dict], flags: List[Dict]) -> List[Dict]:
    items = []
    painted = set()

    def add(code, desc, sid, unit, q: Measure, basis, conditional=None):
        it = {"id": f"li_{len(items)}", "code": code, "description": desc, "surface_id": sid, "unit": unit,
              "quantity": q, "basis": basis}
        if conditional:
            it["conditional"] = conditional
        items.append(it)

    flagged = {f["damage_ids"][0]: f["rule_id"] for f in flags}
    for d in damage:
        r, wi = _surface(rooms, d["surface_id"])
        if r is None:
            continue
        w_ext = d["extent_w"].value + 2 * MARGIN
        h_ext = d["extent_h"].value + 2 * MARGIN
        a_treat = Measure(w_ext * h_ext, combine_sigma(d["extent_w"].sigma * h_ext, d["extent_h"].sigma * w_ext), "m2")
        sid = d["surface_id"]
        if d["class"] == "water_stain":
            add("MOIST-INSP", "Moisture mapping of affected board and cavity (meter + thermal)", sid, "ea", Measure(1, 0, "ea"), [d["id"]])
            add("SEAL-STAIN", "Stain-blocking primer over stain + margin", sid, "m2", a_treat, [d["id"]])
            add("DRY-RR", "Remove and replace board where moisture > 17% WME", sid, "m2",
                Measure(np.ceil(a_treat.value / SHEET) * SHEET, a_treat.sigma, "m2"), [d["id"]],
                conditional="moisture reading above threshold")
            if wi is not None and d["bbox_surface"]["v0"] <= 0.30:
                add("BASE-RR", "Detach and reset / replace skirting board", sid, "m",
                    Measure(w_ext, d["extent_w"].sigma, "m"), [d["id"]])
                add("FLOOD-CUT", "Flood cut board to 0.6 m and dry cavity", sid, "m",
                    Measure(w_ext, d["extent_w"].sigma, "m"), [d["id"]], conditional="moisture reading above threshold")
        elif d["class"] == "mold":
            add("MOLD-REM", "Mold remediation: containment, HEPA, antimicrobial, remove affected board", sid, "m2", a_treat, [d["id"]])
            add("DRY-RR", "Replace removed board", sid, "m2", Measure(np.ceil(a_treat.value / SHEET) * SHEET, a_treat.sigma, "m2"), [d["id"]])
            add("MOIST-INSP", "Locate moisture source", sid, "ea", Measure(1, 0, "ea"), [d["id"]])
        elif d["class"] == "crack":
            L = d.get("length") or d["extent_w"]
            add("CRACK-TAPE", "Rake out, tape, joint compound and texture crack", sid, "m", L, [d["id"]])
            if flagged.get(d["id"]) == "CD-05":
                add("STRUCT-INSP", "Structural assessment of crack movement", sid, "ea", Measure(1, 0, "ea"), [d["id"]], conditional="if crack is active / > 3 mm")
        if sid not in painted:
            painted.add(sid)
            if wi is not None:
                add("PAINT-WALL", "Paint full wall face, 2 coats (colour match requires full face)", sid, "m2", wall_net_area(r, wi), [d["id"]])
            else:
                add("PAINT-CEIL", "Paint full ceiling, 2 coats", sid, "m2", r.floor_area, [d["id"]])
    return items
