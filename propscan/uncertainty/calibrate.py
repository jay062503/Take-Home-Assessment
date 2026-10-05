"""Fit per-tier, per-quantity sigma multipliers from benchmark residuals.

For each (tier, quantity) the reported sigma already includes the current multiplier. The new
multiplier is old * q90(|err| / sigma) / 1.645, so the 90% interval would have covered 90% of the
benchmark truths. Groups with fewer than MIN_N residuals keep the prior (multiplier 1) rather than
being fitted to noise, and multipliers are clamped to [0.5, 6] so a lucky benchmark cannot shrink
intervals to nothing.
"""
from __future__ import annotations

import json
import time
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

from ..config import CALIBRATION_FILE

Z90 = 1.645
MIN_N = 4
KIND_TO_QUANTITY = {"wall_length": "wall_length", "ceiling_height": "ceiling_height", "floor_area": "floor_area",
                    "opening_width": "opening_width", "footprint_area": "floor_area"}


def fit_calibration(results: List[str], out: Optional[str] = None) -> Dict:
    groups = defaultdict(list)
    sources = []
    for p in results:
        res = json.loads(Path(p).read_text())
        sources.append({"results": str(p), "manifest": res["manifest"], "synthetic": res["synthetic"]})
        for s in res["scores"]:
            for r in s["records"]:
                q = KIND_TO_QUANTITY.get(r["kind"])
                if q and r["sigma"] > 0:
                    groups[(r["tier"], q)].append(abs(r["err"]) / r["sigma"])
    prev = json.loads(CALIBRATION_FILE.read_text()).get("factors", {}) if CALIBRATION_FILE.exists() else {}
    factors: Dict[str, Dict[str, float]] = defaultdict(dict)
    table = []
    for (tier, q), z in sorted(groups.items()):
        z = np.asarray(z)
        old = float(prev.get(tier, {}).get(q, 1.0))
        if len(z) < MIN_N:
            factors[tier][q] = old
            table.append((tier, q, len(z), None, old))
            continue
        k = float(np.clip(old * np.quantile(z, 0.9) / Z90, 0.5, 6.0))
        factors[tier][q] = round(k, 3)
        table.append((tier, q, len(z), float(np.quantile(z, 0.9)), k))
    data = {"status": "calibrated", "fitted": time.strftime("%Y-%m-%d %H:%M:%S"), "method": "q90(|err|/sigma)/1.645, clamp [0.5, 6]",
            "min_n": MIN_N, "source": sources, "factors": factors}
    if any(s["synthetic"] for s in sources):
        data["warning"] = ("fitted on synthetic residuals: valid for the synthetic noise model only; refit on "
                           "benchmark/real before quoting real-world intervals")
    data["leave_one_capture_out"] = _loo(results, prev)
    path = Path(out) if out else CALIBRATION_FILE
    path.write_text(json.dumps(data, indent=1))
    print(f"[calibrate] wrote {path}")
    for tier, q, n, z90, k in table:
        print(f"  {tier:6s} {q:15s} n={n:3d}  z90={'  -  ' if z90 is None else f'{z90:5.2f}'}  factor={k:.3f}")
    for tier, v in data["leave_one_capture_out"].items():
        print(f"  LOO coverage {tier:6s}: prior {v['prior']:.0%}  calibrated {v['calibrated']:.0%}  (n={v['n']})")
    return data


def _loo(results: List[str], prev: Dict) -> Dict:
    """Held-out coverage: for each capture, fit factors on the other captures, score it."""
    recs = []
    for p in results:
        for s in json.loads(Path(p).read_text())["scores"]:
            for r in s["records"]:
                q = KIND_TO_QUANTITY.get(r["kind"])
                if q and r["sigma"] > 0:
                    recs.append((s["capture"], r["tier"], q, abs(r["err"]), r["sigma"]))
    out = defaultdict(lambda: {"n": 0, "prior": 0, "calibrated": 0})
    for cap in sorted({r[0] for r in recs}):
        train = defaultdict(list)
        for c, t, q, e, sg in recs:
            if c != cap:
                train[(t, q)].append(e / sg)
        for c, t, q, e, sg in recs:
            if c != cap:
                continue
            z = train.get((t, q), [])
            k = float(np.clip(np.quantile(z, 0.9) / Z90, 0.5, 6.0)) if len(z) >= MIN_N else 1.0
            o = out[t]
            o["n"] += 1
            o["prior"] += int(e <= Z90 * sg)
            o["calibrated"] += int(e <= Z90 * sg * k)
    return {t: {"n": v["n"], "prior": v["prior"] / v["n"], "calibrated": v["calibrated"] / v["n"]} for t, v in out.items()}
