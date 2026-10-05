"""Assemble the published JSON (schema/plan.schema.json) and apply interval calibration."""
from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, List

import jsonschema
import numpy as np
from shapely.geometry import Polygon
from shapely.ops import unary_union

from .. import SCHEMA_VERSION, __version__
from ..config import ROOT, calib_factor
from ..damage.scope import wall_net_area
from ..types import Measure, Room, combine_sigma

SCHEMA = ROOT / "schema" / "plan.schema.json"


def _m(x: Measure, cfg: Dict, tier: str, quantity: str) -> Dict:
    return x.scaled(calib_factor(cfg, tier, quantity)).to_json()


def _r(v, n=4):
    return [round(float(a), n) for a in v]


def room_json(r: Room, cfg: Dict, tier: str) -> Dict:
    walls = []
    for i, w in enumerate(r.walls):
        walls.append({"id": w.id, "start": _r(w.start), "end": _r(w.end),
                      "length": _m(w.length, cfg, tier, "wall_length"),
                      "height": _m(r.ceiling_height, cfg, tier, "ceiling_height"),
                      "net_area": _m(wall_net_area(r, i), cfg, tier, "floor_area"),
                      "observed_fraction": round(w.observed_fraction, 3), "support_points": int(w.support)})
    ops = []
    for o in r.openings:
        ops.append({"id": o.id, "type": o.type, "wall_id": r.walls[o.wall_index].id,
                    "center_offset": _m(o.center_offset, cfg, tier, "opening_width"),
                    "width": _m(o.width, cfg, tier, "opening_width"),
                    "height": _m(o.height, cfg, tier, "opening_width"),
                    "sill_height": _m(o.sill, cfg, tier, "opening_width"),
                    "connects_to": o.connects_to})
    surfaces = [{"id": w.id, "type": "wall"} for w in r.walls] + \
               [{"id": f"{r.id}/ceiling", "type": "ceiling"}, {"id": f"{r.id}/floor", "type": "floor"}]
    return {"id": r.id, "label": r.label, "kind": r.kind, "polygon": [_r(p) for p in r.polygon], "walls": walls,
            "ceiling_height": _m(r.ceiling_height, cfg, tier, "ceiling_height"),
            "floor_area": _m(r.floor_area, cfg, tier, "floor_area"),
            "perimeter": _m(r.perimeter, cfg, tier, "wall_length"),
            "openings": ops, "surfaces": surfaces, "features": r.features, "quality": _clean(r.quality)}


def _clean(o):
    if isinstance(o, dict):
        return {k: _clean(v) for k, v in o.items() if k != "cloud"}
    if isinstance(o, (list, tuple)):
        return [_clean(v) for v in o]
    if isinstance(o, Measure):
        return o.to_json()
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    return o


def footprint(rooms: List[Room]) -> Measure:
    polys = [Polygon(r.polygon) for r in rooms]
    if not polys:
        return Measure(0.0, 0.0, "m2")
    area = unary_union(polys).area
    sig = combine_sigma(*[r.floor_area.sigma for r in rooms])
    place = [r.quality.get("placement_sigma") or 0.0 for r in rooms]
    sig = combine_sigma(sig, *[p * np.sqrt(Polygon(r.polygon).area) for p, r in zip(place, rooms)])
    return Measure(area, sig, "m2")


def build_plan(capture: Dict, tier: str, rooms: List[Room], adjacency: List[Dict], stitch: Dict,
               damage: List[Dict], flags: List[Dict], scope: List[Dict], warnings: List[str], timing: Dict,
               cfg: Dict, diagnostics: Dict) -> Dict:
    fp = footprint(rooms)
    plan = {
        "schema_version": SCHEMA_VERSION,
        "generator": f"propscan {__version__}",
        "capture": capture,
        "units": "metres",
        "confidence_level": float(cfg["output"]["confidence_level"]),
        "calibration": {"status": cfg["calibration"].get("status", "prior"),
                        "factors": cfg["calibration"].get("factors", {}).get(tier, {}),
                        "source": cfg["calibration"].get("source")},
        "rooms": [room_json(r, cfg, tier) for r in rooms],
        "adjacency": adjacency,
        "stitched_plan": {"footprint_area": _m(fp, cfg, tier, "floor_area"), "n_rooms": len(rooms),
                          "method": stitch.get("method", "global poses"), "overlaps": stitch.get("overlaps", []),
                          "unresolved_rooms": stitch.get("unresolved", []), "details": _clean(stitch)},
        "damage": [_clean({**d, "extent_w": _m(d["extent_w"], cfg, tier, "damage_extent"),
                           "extent_h": _m(d["extent_h"], cfg, tier, "damage_extent"),
                           "area": _m(d["area"], cfg, tier, "damage_extent"),
                           **({"length": _m(d["length"], cfg, tier, "damage_extent")} if d.get("length") else {})})
                   for d in damage],
        "concealed_damage_flags": flags,
        "scope": [_clean({**s, "quantity": s["quantity"].to_json()}) for s in scope],
        "warnings": warnings,
        "timing": timing,
        "diagnostics": _clean(diagnostics),
    }
    return plan


def validate(plan: Dict) -> None:
    with open(SCHEMA) as f:
        jsonschema.validate(plan, json.load(f))


def write_json(plan: Dict, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        json.dump(plan, f, indent=2)
