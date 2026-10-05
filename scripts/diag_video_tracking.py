"""Video odometry vs the synthetic ground-truth trajectory (Sim(3)-aligned ATE, first divergence).

    python scripts/diag_video_tracking.py data/synthetic/apartment_a/video_full
"""
import sys
from pathlib import Path

import numpy as np

from propscan.config import load_config
from propscan.ingest.video import load_video


def umeyama(src, dst):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    S, D = src - mu_s, dst - mu_d
    U, sig, Vt = np.linalg.svd(D.T @ S / len(src))
    E = np.eye(3)
    E[2, 2] = np.sign(np.linalg.det(U @ Vt))
    R = U @ E @ Vt
    s = np.trace(np.diag(sig) @ E) / S.var(0).sum()
    return s, R, mu_d - s * R @ mu_s


def main(root):
    root = Path(root)
    gt = np.load(root / "gt_trajectory.npy")
    fs = load_video(root / "capture" / "walkthrough.mp4", load_config())
    idx = np.array([int(f.name) for f in fs.frames])
    est = np.stack([f.T_wc[:3, 3] for f in fs.frames])
    g = gt[idx, :3, 3]
    s, R, t = umeyama(est, g)
    err = np.linalg.norm((s * est @ R.T + t) - g, axis=1)
    tr = fs.meta["tracking"]
    print(f"frames {len(idx)}/{len(gt)} tracked, lost {tr['lost']}, coasted {tr.get('coasted')}, relocalised {tr['relocalised']}")
    print(f"Sim3 scale {s:.4f}  ATE rmse {np.sqrt((err ** 2).mean()):.3f} m  max {err.max():.3f} m")
    bad = np.flatnonzero(err > 0.15)
    if len(bad):
        print(f"first frame with error > 15 cm: {idx[bad[0]]}")
    for k in range(0, len(idx), max(1, len(idx) // 15)):
        print(f"  frame {idx[k]:4d}  err {err[k]:.3f} m")


if __name__ == "__main__":
    main(sys.argv[1])
