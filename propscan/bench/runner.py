"""Benchmark runner: run every capture in a manifest, score it against ground truth, apply the gates.

Manifest format (synthetic and real benchmarks share it; see benchmark/README.md):

    captures:
      - {id: lidar_full, tier: lidar, path: lidar_full/capture, gt: lidar_full/gt.yaml, repeat_group: null}

Every number in the report is computed here from plan.json files and gt.yaml files, so
`propscan bench <manifest>` regenerates all of it.
"""
from __future__ import annotations

import json
import math
import time
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import yaml
from shapely.geometry import Polygon

from ..pipeline import run_capture
from . import match as M

Z90 = 1.645

WALL_TOL = {"photo": ("rel", 0.08), "video": ("rel", 0.03), "lidar": ("rel", 0.01)}
WALL_PASS_FRACTION = 0.90
OPENING_TOL = 0.02
OPENING_PASS_FRACTION = 0.85
CEILING_TOL = 0.015
CEILING_SPREAD = 0.010
FOOTPRINT_TOL = 0.08


def _load_yaml(p: Path) -> Dict:
    with open(p) as f:
        return yaml.safe_load(f)


def _rec(records, cap, kind, room, pred_m: Dict, gt: float, **extra):
    v, s = float(pred_m["value"]), float(pred_m["sigma"])
    records.append({"capture": cap["id"], "tier": cap["tier"], "kind": kind, "room": room, "pred": v, "gt": float(gt),
                    "err": v - float(gt), "sigma": s, "lo": float(pred_m["lo"]), "hi": float(pred_m["hi"]),
                    "covered": bool(pred_m["lo"] <= gt <= pred_m["hi"]), **extra})


def score_capture(plan: Dict, gt: Dict, cap: Dict) -> Dict:
    rooms, groom = plan["rooms"], gt["rooms"]
    matches, un_p, un_g = M.match_rooms(rooms, groom)
    name_of = {rooms[i]["id"]: groom[j]["name"] for i, j, _, _ in matches}
    rec: List[Dict] = []
    walls_missed, walls_phantom = [], []
    op_hits, op_rows, op_missed, op_phantom = 0, [], [], []
    for i, j, tf, s in matches:
        pr, gr = rooms[i], groom[j]
        gname = gr["name"]
        gp = Polygon(gr["polygon"])
        _rec(rec, cap, "floor_area", gname, pr["floor_area"], gp.area, iou=round(s, 3))
        _rec(rec, cap, "ceiling_height", gname, pr["ceiling_height"], gr["ceiling_height"])
        pairs, ph, ms = M.match_walls(pr, gr, tf)
        for w, e in pairs:
            _rec(rec, cap, "wall_length", gname, w["length"], e["length"], gt_wall=f"{gname}/{e['index']}",
                 observed_fraction=w.get("observed_fraction"))
        walls_phantom += [f"{gname}:{w['id']}" for w in ph]
        walls_missed += [f"{gname}/{e['index']} ({e['length']:.2f} m)" for e in ms]
        opairs, oph, oms = M.match_openings(pr, gr, tf)
        for o, g in opairs:
            _rec(rec, cap, "opening_width", gname, o["width"], g["width"], type_pred=o["type"], type_gt=g["type"])
            hit = abs(o["width"]["value"] - g["width"]) <= OPENING_TOL
            op_hits += int(hit)
            op_rows.append({"room": gname, "type": g["type"], "gt": g["width"], "pred": o["width"]["value"],
                            "err_cm": round(100 * (o["width"]["value"] - g["width"]), 1), "hit": hit,
                            "type_ok": o["type"] == g["type"] or {o["type"], g["type"]} <= {"door", "passage"}})
        op_phantom += [f"{gname}:{o['id']} ({o['type']} {o['width']['value']:.2f})" for o in oph]
        op_missed += [f"{gname}:{g['type']} {g['width']:.2f}" for g in oms]
    for j in un_g:
        for g in groom[j].get("openings", []):
            op_missed.append(f"{groom[j]['name']}:{g['type']} {g['width']:.2f} (room missing)")
        walls_missed += [f"{groom[j]['name']}/{e['index']} (room missing)" for e in M.gt_edges(groom[j]["polygon"])]

    gt_adj = {tuple(sorted(a)) for a in gt.get("adjacency", [])}
    pred_adj = {tuple(sorted((name_of.get(a["a"], "?" + a["a"]), name_of.get(a["b"], "?" + a["b"]))))
                for a in plan.get("adjacency", [])}
    gfp = M.footprint_area(groom)
    sp = plan["stitched_plan"]
    fp = sp["footprint_area"]
    stitch = {"gt_area": round(gfp, 3), "pred_area": fp["value"], "rel_err": (fp["value"] - gfp) / gfp if gfp else None,
              "covered": fp["lo"] <= gfp <= fp["hi"], "overlaps": M.overlaps(rooms),
              "unresolved": sp.get("unresolved_rooms", []),
              "adjacency_gt": sorted(gt_adj), "adjacency_pred": sorted(pred_adj),
              "adjacency_correct": pred_adj == gt_adj, "rooms_gt": len(groom), "rooms_pred": len(rooms),
              "rooms_matched": len(matches)}
    _rec(rec, cap, "footprint_area", "*", fp, gfp)

    dmg = _score_damage(plan, gt, name_of)
    return {"capture": cap["id"], "tier": cap["tier"], "records": rec,
            "room_matches": [{"pred": rooms[i]["id"], "gt": groom[j]["name"], "iou": round(s, 3)} for i, j, _, s in matches],
            "rooms_unmatched_pred": [rooms[i]["id"] for i in un_p], "rooms_missed": [groom[j]["name"] for j in un_g],
            "walls_missed": walls_missed, "walls_phantom": walls_phantom,
            "openings": {"rows": op_rows, "hits": op_hits, "missed": op_missed, "phantom": op_phantom,
                         "n_gt": sum(len(r.get("openings", [])) for r in groom)},
            "stitch": stitch, "damage": dmg, "timing": plan.get("timing", {}),
            "drift": plan["capture"].get("drift_correction"), "warnings": plan.get("warnings", [])}


def _score_damage(plan: Dict, gt: Dict, name_of: Dict) -> Dict:
    gts = [dict(g, matched=False) for g in gt.get("damage", [])]
    tp, fp = [], []
    for d in sorted(plan.get("damage", []), key=lambda d: -d.get("confidence", 0)):
        room = name_of.get(d["room_id"])
        g = next((g for g in gts if not g["matched"] and g["room"] == room and g["class"] == d["class"]
                  and g["surface"] == d["surface_kind"]), None)
        if g is None:
            fp.append({"room": room or d["room_id"], "class": d["class"], "surface": d["surface_kind"]})
            continue
        g["matched"] = True
        ext = [d["extent_w"]["value"], d["extent_h"]["value"]]
        tp.append({"room": room, "class": d["class"], "surface": d["surface_kind"], "gt_size": g["size"],
                   "pred_size": [round(x, 3) for x in ext]})
    fn = [{k: g[k] for k in ("room", "class", "surface", "size")} for g in gts if not g["matched"]]
    n = len(gts)
    return {"tp": tp, "fp": fp, "fn": fn, "recall": len(tp) / n if n else None,
            "precision": len(tp) / (len(tp) + len(fp)) if (tp or fp) else None}


# --------------------------------------------------------------------------------------------- gates

def _wall_ok(tier: str, err: float, gt: float) -> bool:
    mode, tol = WALL_TOL[tier]
    return abs(err) <= (tol * gt if mode == "rel" else tol)


def gates(scores: List[Dict], manifest: Dict, ablation: Dict) -> List[Dict]:
    by_tier = defaultdict(list)
    for s in scores:
        by_tier[s["tier"]].append(s)
    out = []
    for tier, ss in sorted(by_tier.items()):
        # opening widths: hits / (gt + phantoms); a miss and a phantom each count as a miss
        hits = sum(s["openings"]["hits"] for s in ss)
        denom = sum(s["openings"]["n_gt"] + len(s["openings"]["phantom"]) for s in ss)
        frac = hits / denom if denom else None
        out.append({"gate": "opening_width", "tier": tier, "value": frac, "threshold": f">= {OPENING_PASS_FRACTION:.0%} within 2 cm",
                    "detail": f"{hits} hits / ({denom - sum(len(s['openings']['phantom']) for s in ss)} GT + "
                              f"{sum(len(s['openings']['phantom']) for s in ss)} phantom)",
                    "pass": frac is not None and frac >= OPENING_PASS_FRACTION})
        ch = [r for s in ss for r in s["records"] if r["kind"] == "ceiling_height"]
        worst = max((abs(r["err"]) for r in ch), default=None)
        out.append({"gate": "ceiling_height", "tier": tier, "value": worst, "threshold": "<= 1.5 cm every room",
                    "detail": f"{sum(abs(r['err']) <= CEILING_TOL for r in ch)}/{len(ch)} rooms within 1.5 cm",
                    "pass": worst is not None and worst <= CEILING_TOL})
        wl = [r for s in ss for r in s["records"] if r["kind"] == "wall_length"]
        n_missed = sum(len([w for w in s["walls_missed"]]) for s in ss)
        ok = sum(_wall_ok(tier, r["err"], r["gt"]) for r in wl)
        tot = len(wl) + n_missed
        mode, tol = WALL_TOL[tier]
        out.append({"gate": "wall_length", "tier": tier, "value": ok / tot if tot else None,
                    "threshold": f">= {WALL_PASS_FRACTION:.0%} of GT walls within ±{tol:.0%}" + (" (own target)" if tier == "lidar" else ""),
                    "detail": f"{ok}/{tot} (missed walls count as failures: {n_missed})",
                    "pass": tot > 0 and ok / tot >= WALL_PASS_FRACTION})
        cal = [r for s in ss for r in s["records"]]
        cov = np.mean([r["covered"] for r in cal]) if cal else None
        out.append({"gate": "calibration", "tier": tier, "value": None if cov is None else float(cov),
                    "threshold": "90% intervals cover 80-97% of truths",
                    "detail": f"{sum(r['covered'] for r in cal)}/{len(cal)} measurements covered",
                    "pass": cov is not None and 0.80 <= cov <= 0.97})
        multi = [s for s in ss if s["stitch"]["rooms_gt"] >= 3]
        if tier == "photo":
            for s in multi:
                st = s["stitch"]
                ok = (st["rel_err"] is not None and abs(st["rel_err"]) <= FOOTPRINT_TOL and not st["overlaps"]
                      and st["adjacency_correct"] and not st["unresolved"] and st["rooms_matched"] == st["rooms_gt"])
                out.append({"gate": "photo_stitch", "tier": tier, "value": st["rel_err"], "threshold": "footprint ±8%, correct adjacency, no overlaps, all rooms placed",
                            "detail": f"{s['capture']}: footprint {st['pred_area']:.2f} vs {st['gt_area']:.2f} m2, "
                                      f"overlaps {len(st['overlaps'])}, unresolved {st['unresolved']}, adjacency "
                                      f"{'ok' if st['adjacency_correct'] else 'wrong'}",
                            "pass": bool(ok)})
    out += _repeat_gates(scores, manifest)
    for cid, ab in ablation.items():
        on, off = ab["on"], ab["off"]
        better = on["wall_mae"] <= off["wall_mae"] + 1e-4 and abs(on["footprint_rel_err"]) <= abs(off["footprint_rel_err"]) + 1e-4
        out.append({"gate": "drift_accountability", "tier": ab["tier"], "value": on["wall_mae"],
                    "threshold": "drift handled (pose graph) + on/off ablation; on must not be worse",
                    "detail": f"{cid}: wall MAE {100 * on['wall_mae']:.1f} cm on vs {100 * off['wall_mae']:.1f} cm off; footprint err "
                              f"{100 * on['footprint_rel_err']:+.1f}% on vs {100 * off['footprint_rel_err']:+.1f}% off; "
                              f"{on['drift'].get('loop_closures', 0)} loop closures",
                    "pass": bool(on["drift"].get("applied") and better)})
    return out


def _repeat_gates(scores: List[Dict], manifest: Dict) -> List[Dict]:
    groups = defaultdict(list)
    for c in manifest["captures"]:
        if c.get("repeat_group"):
            groups[c["repeat_group"]].append(c["id"])
    sc = {s["capture"]: s for s in scores}
    out = []
    for g, ids in sorted(groups.items()):
        ss = [sc[i] for i in ids if i in sc]
        if len(ss) < 2:
            continue
        tier = ss[0]["tier"]
        walls = defaultdict(list)
        ceil = defaultdict(list)
        for s in ss:
            for r in s["records"]:
                if r["kind"] == "wall_length":
                    walls[r["gt_wall"]].append((r["pred"], r["gt"]))
                if r["kind"] == "ceiling_height":
                    ceil[r["room"]].append(r["pred"] - r["gt"])
        rows = []
        for w, v in sorted(walls.items()):
            p = [a for a, _ in v]
            sp = max(p) - min(p) if len(p) >= 2 else None
            tol = max(0.01, 0.005 * v[0][1])
            rows.append({"wall": w, "gt": v[0][1], "preds": p, "spread": sp, "tol": tol,
                         "pass": sp is not None and sp <= tol})
        ok = bool(rows) and all(r["pass"] for r in rows)
        out.append({"gate": "repeatability", "tier": tier, "value": max((r["spread"] or 9) for r in rows) if rows else None,
                    "threshold": "every wall within max(1 cm, 0.5%) across captures",
                    "detail": f"{g}: {sum(r['pass'] for r in rows)}/{len(rows)} walls", "rows": rows, "pass": bool(ok)})
        for room, errs in ceil.items():
            spread = max(errs) - min(errs) if len(errs) >= 2 else None
            bias = float(np.mean(errs))
            verdict = ("unrepeatable" if spread is None or spread > CEILING_SPREAD else
                       "repeatable-but-biased" if abs(bias) > CEILING_TOL else "repeatable and accurate")
            out.append({"gate": "ceiling_spread", "tier": tier, "value": spread, "threshold": "spread <= 1 cm and |bias| <= 1.5 cm",
                        "detail": f"{g}/{room}: spread {100 * (spread or 0):.2f} cm, bias {100 * bias:+.2f} cm -> {verdict}",
                        "verdict": verdict, "pass": verdict == "repeatable and accurate"})
    return out


# --------------------------------------------------------------------------------------------- driver

def _ablation_summary(score: Dict) -> Dict:
    wl = [abs(r["err"]) for r in score["records"] if r["kind"] == "wall_length"]
    st = score["stitch"]
    return {"wall_mae": float(np.mean(wl)) if wl else float("nan"), "walls_matched": len(wl),
            "walls_missed": len(score["walls_missed"]), "footprint_rel_err": st["rel_err"] or 0.0,
            "rooms_matched": st["rooms_matched"], "drift": score["drift"] or {}}


def run_benchmark(manifest_path: str, out_dir: str, cfg: Dict, only: Optional[List[str]] = None,
                  ablation: bool = True, reuse: bool = False) -> Dict:
    mpath = Path(manifest_path).resolve()
    base = mpath.parent
    manifest = _load_yaml(mpath)
    out = Path(out_dir)
    (out / "captures").mkdir(parents=True, exist_ok=True)
    scores, abl, runtime = [], {}, {}
    for cap in manifest["captures"]:
        if only and cap["id"] not in only and cap["tier"] not in only:
            continue
        gt = _load_yaml(base / cap["gt"])
        cdir = out / "captures" / cap["id"]
        plan = _run(base / cap["path"], cdir, cfg, cap, None, reuse)
        runtime[cap["id"]] = plan.get("timing", {}).get("total")
        s = score_capture(plan, gt, cap)
        scores.append(s)
        print(f"[bench] {cap['id']}: rooms {s['stitch']['rooms_matched']}/{s['stitch']['rooms_gt']}, "
              f"openings {s['openings']['hits']}/{s['openings']['n_gt']} (+{len(s['openings']['phantom'])} phantom)", flush=True)
        if ablation and cap["tier"] in ("lidar", "video") and len(gt["rooms"]) >= 3:
            p_off = _run(base / cap["path"], out / "captures" / f"{cap['id']}__drift_off", cfg, cap, False, reuse)
            s_off = score_capture(p_off, gt, dict(cap, id=cap["id"] + "__drift_off"))
            abl[cap["id"]] = {"tier": cap["tier"], "on": _ablation_summary(s), "off": _ablation_summary(s_off)}
    G = gates(scores, manifest, abl)
    res = {"manifest": str(mpath), "synthetic": bool(manifest.get("synthetic")), "generated": time.strftime("%Y-%m-%d %H:%M:%S"),
           "config_calibration": cfg.get("calibration", {}).get("status"), "gates": G, "scores": scores,
           "ablation": abl, "timing_s": runtime}
    with open(out / "results.json", "w") as f:
        json.dump(res, f, indent=1, default=_json_default)
    from .report import write_report
    write_report(res, out / "REPORT.md")
    print(f"[bench] wrote {out / 'results.json'} and {out / 'REPORT.md'}")
    for g in G:
        print(f"  {'PASS' if g['pass'] else 'FAIL'}  {g['tier']:6s} {g['gate']:22s} {g['detail']}")
    return res


def _run(path: Path, cdir: Path, cfg: Dict, cap: Dict, drift: Optional[bool], reuse: bool) -> Dict:
    pj = cdir / "plan.json"
    if reuse and pj.exists():
        return json.loads(pj.read_text())
    return run_capture(str(path), str(cdir), cfg, tier=cap["tier"], drift=drift, capture_id=cap["id"], render=True,
                       verbose=False)


def _json_default(o):
    if isinstance(o, (np.floating, np.integer, np.bool_)):
        return o.item()
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, tuple):
        return list(o)
    raise TypeError(type(o))
