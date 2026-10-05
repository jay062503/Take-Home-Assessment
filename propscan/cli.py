"""propscan command line.

  propscan run <capture> [-o out/]          one command per capture, any tier (auto-detected)
  propscan detect <capture>                 print the detected tier
  propscan synth [--scene ...] [--out ...]  generate the synthetic integration benchmark
  propscan bench <manifest.yaml> [-o ...]   run a benchmark set, score every gate, write the report
  propscan calibrate <bench_results.json>   fit interval multipliers -> configs/calibration.json
  propscan validate <plan.json>             validate against schema/plan.schema.json
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from .config import load_config


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="propscan", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    r = sub.add_parser("run", help="process one capture")
    r.add_argument("capture")
    r.add_argument("-o", "--out", default=None, help="output dir (default: out/<capture-name>)")
    r.add_argument("--tier", choices=["lidar", "video", "photo"], default=None)
    r.add_argument("--no-drift-correction", action="store_true")
    r.add_argument("--config", default=None)
    r.add_argument("--quiet", action="store_true")
    r.add_argument("--depth", choices=["auto", "model"], default="auto",
                   help="model: ignore synthetic depth sidecars and run the monocular depth model")

    d = sub.add_parser("detect")
    d.add_argument("capture")

    s = sub.add_parser("synth")
    s.add_argument("--scene", default="scenes/apartment_a.yaml")
    s.add_argument("--out", default="data/synthetic/apartment_a")
    s.add_argument("--quick", action="store_true", help="LiDAR captures only")
    s.add_argument("--force", action="store_true")

    b = sub.add_parser("bench")
    b.add_argument("manifest")
    b.add_argument("-o", "--out", default="reports/latest")
    b.add_argument("--only", default=None, help="comma-separated capture ids")
    b.add_argument("--config", default=None)
    b.add_argument("--no-ablation", action="store_true", help="skip the drift on/off ablation runs")
    b.add_argument("--reuse", action="store_true", help="score existing plan.json files instead of re-running")
    b.add_argument("--depth", choices=["auto", "model"], default="auto")

    c = sub.add_parser("calibrate")
    c.add_argument("results", nargs="+", help="bench_results.json file(s)")
    c.add_argument("--out", default=None)

    v = sub.add_parser("validate")
    v.add_argument("plan")

    a = ap.parse_args(argv)
    if a.cmd == "run":
        from .pipeline import run_capture
        cfg = load_config(a.config, {"depth_model": {"source": a.depth}})
        name = Path(a.capture).resolve().name
        if name == "capture":
            name = Path(a.capture).resolve().parent.name
        out = a.out or str(Path("out") / name)
        plan = run_capture(a.capture, out, cfg, tier=a.tier, drift=False if a.no_drift_correction else None,
                           verbose=not a.quiet)
        _summary(plan)
        return 0
    if a.cmd == "detect":
        from .ingest.detect import detect_tier
        t, p = detect_tier(a.capture)
        print(t, p)
        return 0
    if a.cmd == "synth":
        from .synth.export import generate_all
        out = generate_all(a.scene, a.out, quick=a.quick, force=a.force)
        print(f"synthetic benchmark written to {out} (manifest: {out / 'manifest.yaml'})")
        return 0
    if a.cmd == "bench":
        from .bench.runner import run_benchmark
        only = a.only.split(",") if a.only else None
        run_benchmark(a.manifest, a.out, load_config(a.config, {"depth_model": {"source": a.depth}}), only=only, ablation=not a.no_ablation, reuse=a.reuse)
        return 0
    if a.cmd == "calibrate":
        from .uncertainty.calibrate import fit_calibration
        fit_calibration(a.results, a.out)
        return 0
    if a.cmd == "validate":
        from .output.build import validate
        with open(a.plan) as f:
            validate(json.load(f))
        print("valid")
        return 0
    return 1


def _summary(plan) -> None:
    print(f"\n{plan['capture']['id']} [{plan['capture']['tier']}]  rooms={len(plan['rooms'])}  "
          f"footprint={plan['stitched_plan']['footprint_area']['value']:.2f} m2  damage={len(plan['damage'])}  "
          f"flags={len(plan['concealed_damage_flags'])}  scope_items={len(plan['scope'])}")
    for r in plan["rooms"]:
        ch = r["ceiling_height"]
        print(f"  {r['label']:<14} {r['kind']:<9} area {r['floor_area']['value']:6.2f} m2  "
              f"H {ch['value']:.3f} [{ch['lo']:.3f},{ch['hi']:.3f}]  walls " +
              " ".join(f"{w['length']['value']:.3f}" for w in r["walls"]) +
              "  openings " + " ".join(f"{o['type'][0]}{o['width']['value']:.2f}" for o in r["openings"]))
    for w in plan["warnings"][:8]:
        print("  ! " + w)


if __name__ == "__main__":
    sys.exit(main())
