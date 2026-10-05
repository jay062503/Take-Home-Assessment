import numpy as np, pickle
import propscan.geometry.structure as S
orig = S._refine_jambs
log = []
def wrap(hs, hz, xs, xz, rs, rz, rn, s0, s1, z0, z1, **k):
    r = orig(hs, hz, xs, xz, rs, rz, rn, s0, s1, z0, z1, **k)
    log.append(dict(rs=rs, rn=rn, rz=rz, hs=hs, hz=hz, xs=xs, xz=xz, s0=s0, s1=s1, z0=z0, z1=z1, r=r))
    return r
S._refine_jambs = wrap
from propscan.pipeline import run_capture
run_capture('data/synthetic/apartment_a/lidar_bedroom_r2/capture', '/tmp/dbg_r2', render=False, verbose=False)
pickle.dump(log, open('/tmp/jamb_log.pkl', 'wb'))
for L in log:
    h = L['z1'] - L['z0']; lo, hi = L['z0'] + .15*h, L['z1'] - .15*h
    for e in (L['s0'], L['s1']):
        H = L['hs'][(L['hz']>lo)&(L['hz']<hi)&(np.abs(L['hs']-e)<.06)]
        X = L['xs'][(L['xz']>lo)&(L['xz']<hi)&(np.abs(L['xs']-e)<.06)]
        print(f"edge {e:.3f} -> nH {len(H)} nX {len(X)}  H range {H.min() if len(H) else 0:.3f}..{H.max() if len(H) else 0:.3f}  X range {X.min() if len(X) else 0:.3f}..{X.max() if len(X) else 0:.3f}")
    print('result', L['s0'], L['s1'], '->', L['r'])
for L in log:
    for e, sd in ((L['s0'], 1), (L['s1'], -1)):
        R = L['rs'][(L['rn']==sd)&(np.abs(L['rs']-e)<.06)]
        print(f"reveal side {sd}: n={len(R)} median {np.median(R) if len(R) else 0:.4f} (coarse {e:.4f})")
