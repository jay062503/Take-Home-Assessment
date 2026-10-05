"""Per-edge opening error: is the width error a centre shift or a symmetric loss at both jambs?

    python scripts/diag_openings.py reports/synthetic_before/captures/lidar_full/plan.json \
        data/synthetic/apartment_a/lidar_full/gt.yaml
"""
import sys

import numpy as np
import yaml

from propscan.bench import match as M


def main(plan_path, gt_path):
    import json
    plan = json.load(open(plan_path))
    gt = yaml.safe_load(open(gt_path))
    matches, _, _ = M.match_rooms(plan["rooms"], gt["rooms"])
    rows = []
    for i, j, tf, _ in matches:
        pr, gr = plan["rooms"][i], gt["rooms"][j]
        for o, g in M.match_openings(pr, gr, tf)[0]:
            w = next(w for w in pr["walls"] if w["id"] == o["wall_id"])
            a, b = tf(w["start"]), tf(w["end"])
            u = (b - a) / np.linalg.norm(b - a)
            c_pred = tf(M.opening_center(pr, o))
            hw = o["width"]["value"] / 2
            gc = np.asarray(g["center"], float)
            # signed along-wall position of each edge relative to GT, positive = opening wider on that side
            e_lo = ((gc - g["width"] / 2 * u) - (c_pred - hw * u)) @ u
            e_hi = ((c_pred + hw * u) - (gc + g["width"] / 2 * u)) @ u
            rows.append((gr["name"], g["type"], g["width"], o["width"]["value"], e_lo, e_hi))
    print(f"{'room':8s} {'type':7s} {'gt':>6s} {'pred':>6s} {'edge A cm':>10s} {'edge B cm':>10s}")
    for r in rows:
        print(f"{r[0]:8s} {r[1]:7s} {r[2]:6.3f} {r[3]:6.3f} {100 * r[4]:10.1f} {100 * r[5]:10.1f}")
    e = np.array([[r[4], r[5]] for r in rows])
    print(f"\nedges: n={e.size}  mean {100 * e.mean():+.2f} cm  median {100 * np.median(e):+.2f} cm  "
          f"inward (narrowing) {np.mean(e < -0.002):.0%}  outward {np.mean(e > 0.002):.0%}")


if __name__ == "__main__":
    main(*sys.argv[1:3])
