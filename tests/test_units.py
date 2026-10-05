import json
import math
import zipfile
from pathlib import Path

import numpy as np
import pytest

from propscan.bench import match as M
from propscan.depth.scale import fuse_scale
from propscan.geometry.structure import _refine_jambs
from propscan.ingest.detect import detect_tier
from propscan.types import Measure

ROOT = Path(__file__).resolve().parents[1]
CFG = {"scale_prior": {"enabled": True, "camera_height": 1.40, "camera_height_sigma": 0.10, "model_scale_sigma": 0.15}}


def test_measure_interval_is_90pct():
    lo, hi = Measure(2.0, 0.01).interval()
    assert hi - lo == pytest.approx(2 * 1.645 * 0.01, rel=1e-3)


def _room(poly, openings=()):
    P = np.asarray(poly, float)
    walls = [{"id": f"w{i}", "start": P[i].tolist(), "end": P[(i + 1) % len(P)].tolist(),
              "length": {"value": float(np.linalg.norm(P[(i + 1) % len(P)] - P[i])), "sigma": 0.01}}
             for i in range(len(P))]
    return {"id": "r", "polygon": P.tolist(), "walls": walls, "openings": list(openings)}


def test_align_recovers_rotation_and_translation():
    gt = [[0, 0], [4.2, 0], [4.2, 3.6], [2.0, 3.6], [2.0, 4.6], [0, 4.6]]
    R = M.rot90(1)
    pred = (np.asarray(gt) @ R.T + [10.0, -3.0]).tolist()
    m, up, ug = M.match_rooms([_room(pred)], [{"name": "living", "polygon": gt}])
    assert len(m) == 1 and m[0][3] > 0.99
    pairs, ph, ms = M.match_walls(_room(pred), {"polygon": gt}, m[0][2])
    assert len(pairs) == 6 and not ph and not ms
    assert all(abs(w["length"]["value"] - e["length"]) < 1e-6 for w, e in pairs)


def test_symmetric_room_orientation_broken_by_openings():
    gt_poly = [[0, 0], [3.2, 0], [3.2, 2.5], [0, 2.5]]
    gt = {"name": "bed", "polygon": gt_poly, "openings": [
        {"type": "door", "width": 0.8, "center": [3.2, 0.6]}, {"type": "window", "width": 1.2, "center": [1.6, 2.5]}]}
    # prediction rotated by 180 deg: shape alone is ambiguous, openings are not
    R = M.rot90(2)
    room = _room((np.asarray(gt_poly) @ R.T).tolist())
    for k, g in enumerate(gt["openings"]):
        c = np.asarray(g["center"], float) @ R.T
        for w in room["walls"]:
            a, b = np.asarray(w["start"]), np.asarray(w["end"])
            u = (b - a) / np.linalg.norm(b - a)
            if abs(u[0] * (c - a)[1] - u[1] * (c - a)[0]) < 1e-9 and 0 <= (c - a) @ u <= np.linalg.norm(b - a):
                room["openings"].append({"id": f"o{k}", "type": g["type"], "wall_id": w["id"],
                                         "center_offset": {"value": float((c - a) @ u)}, "width": {"value": g["width"]}})
                break
    assert len(room["openings"]) == 2
    m, _, _ = M.match_rooms([room], [gt])
    pairs, ph, ms = M.match_openings(room, gt, m[0][2])
    assert len(pairs) == 2 and all(o["type"] == g["type"] for o, g in pairs)


def test_jamb_reveal_localises_edge_subcell():
    rng = np.random.default_rng(0)
    e0, e1 = 1.003, 1.807                     # true jambs
    hs = np.r_[rng.uniform(0.9, e0, 400), rng.uniform(e1, 1.9, 400)]
    hz = rng.uniform(0.2, 1.9, len(hs))
    xs = rng.uniform(e0 + 0.012, e1 - 0.012, 2000)  # crossings shadowed ~1 cm from each jamb
    xz = rng.uniform(0.2, 1.9, len(xs))
    rs = np.r_[e0 + rng.normal(0, 0.004, 200), e1 + rng.normal(0, 0.004, 200)]
    rz = rng.uniform(0.2, 1.9, 400)
    rn = np.r_[np.ones(200), -np.ones(200)]
    s0, s1 = _refine_jambs(hs, hz, xs, xz, rs, rz, rn, 1.02, 1.78, 0.0, 2.03)
    assert abs(s0 - e0) < 0.002 and abs(s1 - e1) < 0.002


def test_scale_fusion_pulls_toward_camera_height_prior():
    s, sig, info = fuse_scale(1.40 * 1.3, CFG)      # depth 30% too large -> camera looks 1.82 m high
    assert info["applied"] and 0.78 < s < 0.85 and sig < 0.08
    s1, _, _ = fuse_scale(1.40, CFG)
    assert s1 == pytest.approx(1.0)


def test_detect_tiers(tmp_path):
    (tmp_path / "stray" / "depth").mkdir(parents=True)
    (tmp_path / "stray" / "odometry.csv").write_text("x")
    assert detect_tier(str(tmp_path / "stray"))[0] == "lidar"
    v = tmp_path / "walk.MOV"
    v.write_bytes(b"0")
    assert detect_tier(str(v))[0] == "video"
    for room in ("kitchen", "hall"):
        (tmp_path / "flat" / room).mkdir(parents=True)
        (tmp_path / "flat" / room / "IMG_1.HEIC").write_bytes(b"0")
    assert detect_tier(str(tmp_path / "flat"))[0] == "photo"
    z = tmp_path / "flat.zip"
    with zipfile.ZipFile(z, "w") as zf:
        zf.write(tmp_path / "flat" / "kitchen" / "IMG_1.HEIC", "flat/kitchen/IMG_1.HEIC")
        zf.writestr("__MACOSX/._junk", "x")
    assert detect_tier(str(z))[0] == "photo"


def test_calibration_fit(tmp_path, monkeypatch):
    from propscan.uncertainty import calibrate as C
    monkeypatch.setattr(C, "CALIBRATION_FILE", tmp_path / "none.json")
    rng = np.random.default_rng(1)
    recs = [{"tier": "video", "kind": "wall_length", "err": float(e), "sigma": 0.01} for e in rng.normal(0, 0.02, 400)]
    res = tmp_path / "r.json"
    res.write_text(json.dumps({"manifest": "m", "synthetic": True, "scores": [{"records": recs}]}))
    out = C.fit_calibration([str(res)], str(tmp_path / "cal.json"))
    assert out["factors"]["video"]["wall_length"] == pytest.approx(2.0, rel=0.15)


@pytest.mark.parametrize("plan", sorted((ROOT / "reports").glob("*/captures/*/plan.json"))[:3])
def test_committed_plans_validate_against_schema(plan):
    from propscan.output.build import validate
    validate(json.loads(plan.read_text()))
