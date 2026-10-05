"""Render the stitched plan the way a homeowner expects from a consumer scanning app:
thick walls, door swings, windows, room labels with area and ceiling height, wall dimensions
with intervals, damage markers."""
from __future__ import annotations

from pathlib import Path
from typing import Dict

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import Arc, Polygon as MPoly  # noqa: E402

DMG_COLORS = {"water_stain": "#b5651d", "mold": "#2e7d32", "crack": "#c62828"}


def _fmt(m: Dict) -> str:
    half = (m["hi"] - m["lo"]) / 2
    return f"{m['value']:.2f}±{half:.2f}" if half >= 0.005 else f"{m['value']:.3f}"


def render_plan(plan: Dict, path: Path, title: str = None) -> None:
    rooms = plan["rooms"]
    fig, ax = plt.subplots(figsize=(13, 9))
    pal = plt.cm.Pastel1(np.linspace(0, 1, max(3, len(rooms))))
    for k, r in enumerate(rooms):
        P = np.asarray(r["polygon"])
        ax.add_patch(MPoly(P, closed=True, facecolor=pal[k % len(pal)], edgecolor="none", alpha=0.9))
        for w in r["walls"]:
            s, e = np.asarray(w["start"]), np.asarray(w["end"])
            ax.plot([s[0], e[0]], [s[1], e[1]], color="#222", lw=4.5, solid_capstyle="projecting", zorder=3)
            mid = (s + e) / 2
            d = (e - s) / max(np.linalg.norm(e - s), 1e-9)
            nrm = np.array([-d[1], d[0]])
            c = P.mean(axis=0)
            if np.dot(c - mid, nrm) > 0:
                nrm = -nrm
            if w["length"]["value"] > 0.45:
                ang = np.degrees(np.arctan2(d[1], d[0]))
                ang = ang - 180 if ang > 90 else (ang + 180 if ang < -90 else ang)
                ax.text(*(mid - nrm * 0.22), _fmt(w["length"]), fontsize=7, ha="center", va="center", rotation=ang,
                        color="#1a237e", zorder=5)
        wall_by_id = {w["id"]: w for w in r["walls"]}
        for o in r["openings"]:
            w = wall_by_id[o["wall_id"]]
            s, e = np.asarray(w["start"]), np.asarray(w["end"])
            d = (e - s) / max(np.linalg.norm(e - s), 1e-9)
            cpt = s + d * o["center_offset"]["value"]
            half = o["width"]["value"] / 2
            a, b = cpt - d * half, cpt + d * half
            if o["type"] == "window":
                ax.plot([a[0], b[0]], [a[1], b[1]], color="white", lw=4.6, zorder=4)
                ax.plot([a[0], b[0]], [a[1], b[1]], color="#0288d1", lw=1.6, zorder=4)
            else:
                ax.plot([a[0], b[0]], [a[1], b[1]], color="white", lw=5.0, zorder=4)
                if o["type"] == "door":
                    nrm = np.array([-d[1], d[0]])
                    if np.dot(P.mean(axis=0) - cpt, nrm) < 0:
                        nrm = -nrm
                    w_ = o["width"]["value"]
                    ax.plot([a[0], a[0] + nrm[0] * w_], [a[1], a[1] + nrm[1] * w_], color="#555", lw=0.8, zorder=4)
                    th0 = np.degrees(np.arctan2(d[1], d[0]))
                    th1 = np.degrees(np.arctan2(nrm[1], nrm[0]))
                    lo, hi = sorted([th0, th1])
                    if hi - lo > 180:
                        lo, hi = hi, lo + 360
                    ax.add_patch(Arc(a, 2 * w_, 2 * w_, theta1=lo, theta2=hi, color="#555", lw=0.6, zorder=4))
            ax.text(*cpt, f"{o['width']['value']:.2f}", fontsize=6, color="#01579b", ha="center", va="bottom", zorder=6)
        c = P.mean(axis=0)
        lab = r["label"] if r["label"] != r["id"] else r["id"]
        ax.text(c[0], c[1], f"{lab}\n{r['floor_area']['value']:.1f} m²  ±{(r['floor_area']['hi'] - r['floor_area']['lo']) / 2:.2f}\n"
                            f"H {_fmt(r['ceiling_height'])} m", ha="center", va="center", fontsize=8.5, zorder=6)
    rid = {r["id"]: r for r in rooms}
    for d in plan["damage"]:
        r = rid.get(d["room_id"])
        if r is None:
            continue
        bb = d["bbox_surface"]
        if d["surface_kind"] == "ceiling":
            pt = np.array([(bb["u0"] + bb["u1"]) / 2, (bb["v0"] + bb["v1"]) / 2])
        else:
            w = next(w for w in r["walls"] if w["id"] == d["surface_id"])
            s, e = np.asarray(w["start"]), np.asarray(w["end"])
            dd = (e - s) / max(np.linalg.norm(e - s), 1e-9)
            pt = s + dd * (bb["u0"] + bb["u1"]) / 2
            pt = pt + (np.asarray(r["polygon"]).mean(axis=0) - pt) * 0.08
        ax.scatter(*pt, s=90, marker="X", color=DMG_COLORS.get(d["class"], "k"), edgecolor="white", zorder=7)
    for cls, col in DMG_COLORS.items():
        ax.scatter([], [], marker="X", color=col, label=cls.replace("_", " "))
    ax.legend(loc="upper right", fontsize=8, frameon=False)
    ax.set_aspect("equal")
    ax.autoscale_view()
    ax.margins(0.08)
    ax.axis("off")
    cap = plan["capture"]
    fp = plan["stitched_plan"]["footprint_area"]
    ttl = title or f"{cap['id']}  ·  tier: {cap['tier']}  ·  {len(rooms)} rooms  ·  footprint {fp['value']:.1f} m² [{fp['lo']:.1f}, {fp['hi']:.1f}]"
    if cap.get("synthetic"):
        ttl += "  ·  SYNTHETIC"
    ax.set_title(ttl, fontsize=10)
    ax.text(0.0, -0.02, f"dimensions in metres, ± = {int(plan['confidence_level'] * 100)}% interval "
                        f"(calibration: {plan['calibration']['status']})", transform=ax.transAxes, fontsize=7, color="#555")
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=130, bbox_inches="tight")
    plt.close(fig)
