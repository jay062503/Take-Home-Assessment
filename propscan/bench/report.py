"""results.json -> REPORT.md (gates, per-tier accuracy, calibration, repeatability, ablation, damage, timing)."""
from __future__ import annotations

from collections import defaultdict
from pathlib import Path
from typing import Dict

import numpy as np


def _cm(x):
    return "n/a" if x is None else f"{100 * x:.1f} cm"


def _fmt(g):
    v = g["value"]
    if v is None:
        return "n/a"
    if g["gate"] in ("opening_width", "wall_length", "calibration"):
        return f"{100 * v:.0f}%"
    if g["gate"] == "photo_stitch":
        return f"{100 * v:+.1f}%"
    return _cm(v)


def write_report(res: Dict, path: Path) -> None:
    L = []
    w = L.append
    w(f"# Benchmark report\n\nGenerated {res['generated']} from `{Path(res['manifest']).name}` "
      f"(interval calibration: **{res['config_calibration']}**).\n")
    if res["synthetic"]:
        w("> **Synthetic benchmark.** Scenes are ray-cast with known geometry; depth noise, VIO drift and "
          "mono-depth error are emulated. These numbers test the pipeline logic and the gate scorer. They are "
          "**not** real-world accuracy claims; real-capture numbers come from `benchmark/real/`.\n")
    w("## Gates\n\n| Gate | Tier | Value | Threshold | Detail | Result |\n|---|---|---|---|---|---|")
    for g in res["gates"]:
        w(f"| {g['gate']} | {g['tier']} | {_fmt(g)} | {g['threshold']} | {g['detail']} | {'PASS' if g['pass'] else '**FAIL**'} |")

    w("\n## Accuracy by tier and quantity\n\nErrors are prediction minus ground truth. Coverage is the fraction of "
      "truths inside the reported 90% interval. z90 is the 90th percentile of |error| / sigma, which is 1.645 "
      "when the intervals are calibrated.\n")
    w("| Tier | Quantity | n | Mean error | MAE | Max abs error | Coverage | z90 |\n|---|---|---|---|---|---|---|---|")
    agg = defaultdict(list)
    for s in res["scores"]:
        for r in s["records"]:
            agg[(r["tier"], r["kind"])].append(r)
    for (tier, kind), rs in sorted(agg.items()):
        e = np.array([r["err"] for r in rs])
        z = np.array([abs(r["err"]) / r["sigma"] for r in rs if r["sigma"] > 0])
        unit = "m2" if "area" in kind else "m"
        f = (lambda x: f"{x:.3f} m2") if unit == "m2" else _cm
        w(f"| {tier} | {kind} | {len(rs)} | {f(e.mean())} | {f(np.abs(e).mean())} | {f(np.abs(e).max())} | "
          f"{np.mean([r['covered'] for r in rs]):.0%} | {np.quantile(z, 0.9) if len(z) else float('nan'):.2f} |")

    w("\n## Per capture\n")
    for s in res["scores"]:
        st = s["stitch"]
        w(f"### {s['capture']} ({s['tier']})\n")
        pairs = ", ".join("{}->{} IoU {:.2f}".format(m["pred"], m["gt"], m["iou"]) for m in s["room_matches"])
        w(f"Rooms matched {st['rooms_matched']}/{st['rooms_gt']} ({pairs}). "
          f"Footprint {st['pred_area']:.2f} m2 vs {st['gt_area']:.2f} m2"
          + (f" ({100 * st['rel_err']:+.1f}%)" if st['rel_err'] is not None else "")
          + f". Adjacency {'correct' if st['adjacency_correct'] else 'WRONG: ' + str(st['adjacency_pred'])}"
          + (f"; overlaps {st['overlaps']}" if st["overlaps"] else "")
          + (f"; unresolved {st['unresolved']}" if st["unresolved"] else "") + ".\n")
        if s["rooms_missed"]:
            w(f"Missed rooms: {', '.join(s['rooms_missed'])}.\n")
        rows = s["openings"]["rows"]
        if rows:
            w("| Room | Opening | GT width | Predicted | Error | Within 2 cm |\n|---|---|---|---|---|---|")
            for o in rows:
                w(f"| {o['room']} | {o['type']} | {o['gt']:.3f} | {o['pred']:.3f} | {o['err_cm']:+.1f} cm | {'yes' if o['hit'] else 'no'} |")
        if s["openings"]["missed"] or s["openings"]["phantom"]:
            w(f"\nMissed openings: {s['openings']['missed'] or 'none'}. Phantom openings: {s['openings']['phantom'] or 'none'}.\n")
        d = s["damage"]
        missed = ", ".join("{} {} on {}".format(x["room"], x["class"], x["surface"]) for x in d["fn"])
        w(f"\nDamage: {len(d['tp'])} detected / {len(d['tp']) + len(d['fn'])} staged, {len(d['fp'])} false positives. "
          + (f"Missed: {missed}." if d["fn"] else "") + "\n")
        if s["timing"]:
            tm = ", ".join("{} {:.1f} s".format(k, v) for k, v in s["timing"].items())
            w(f"Timing: {tm}.\n")

    rep = [g for g in res["gates"] if g["gate"] == "repeatability"]
    if rep:
        w("## Repeatability\n\n| Group | Wall | GT | Captures | Spread | Tolerance | Pass |\n|---|---|---|---|---|---|---|")
        for g in rep:
            for r in g["rows"]:
                w(f"| {g['detail'].split(':')[0]} | {r['wall']} | {r['gt']:.3f} | {', '.join(f'{p:.3f}' for p in r['preds'])} | "
                  f"{_cm(r['spread'])} | {_cm(r['tol'])} | {'yes' if r['pass'] else 'no'} |")
    if res["ablation"]:
        w("\n## Drift ablation (pose-graph correction on vs off)\n\n| Capture | Tier | Wall MAE on | Wall MAE off | Footprint err on | "
          "Footprint err off | Walls matched on/off | Loop closures | Max correction |\n|---|---|---|---|---|---|---|---|---|")
        for cid, a in res["ablation"].items():
            on, off = a["on"], a["off"]
            w(f"| {cid} | {a['tier']} | {_cm(on['wall_mae'])} | {_cm(off['wall_mae'])} | {100 * on['footprint_rel_err']:+.1f}% | "
              f"{100 * off['footprint_rel_err']:+.1f}% | {on['walls_matched']}/{off['walls_matched']} | "
              f"{on['drift'].get('loop_closures', '-')} | {_cm(on['drift'].get('max_correction_m'))} |")
        w("\nPlans for both runs are in `captures/<id>/plan.png` and `captures/<id>__drift_off/plan.png`.\n")
    w("\n## Timing (end-to-end per capture)\n\n| Capture | Seconds |\n|---|---|")
    for k, v in res["timing_s"].items():
        w(f"| {k} | {v if v is None else round(v, 1)} |")
    Path(path).write_text("\n".join(L) + "\n")
