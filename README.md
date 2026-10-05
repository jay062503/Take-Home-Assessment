# Handheld property scan (`handheld-property-scan`)

Independent take-home repo: phone capture → dimensioned floor plan + damage + calibrated intervals. **No sample zips in git** — add your three interviewer files under [`data/uploads/`](data/README.md).

Phone capture in, whole-property floor plan out. Per-room walls, ceiling height, floor area and openings, a stitched multi-room plan with adjacency, per-surface damage with concealed-damage flags and scope line items, and a 90% interval on every number. One command per capture, at every tier:

| Tier | Input | Capture app |
|---|---|---|
| LiDAR | Stray Scanner export (depth, ARKit poses, intrinsics) | Stray Scanner (App Store, free) |
| Video | one handheld walkthrough `.MOV` | built-in Camera |
| Photos | 2–8 photos per room, one folder per room | built-in Camera, 0.5× lens |

The capture protocol is [`docs/CAPTURE_PROTOCOL.md`](docs/CAPTURE_PROTOCOL.md) (one page). Hardware and honest accuracy per tier are in [`docs/DEVICE_MATRIX.md`](docs/DEVICE_MATRIX.md).

## Running in under 15 minutes (clean macOS or Linux, Python 3.9+)

```bash
cd ~/Desktop/handheld-property-scan
make install        # venv + package + torch/transformers (~3-5 min)
make models         # Depth Anything V2 Metric-Indoor Small, ~100 MB (photo/video tiers only)
# copy your 3 zips → data/uploads/lidar.zip video.zip photo.zip  (see data/README.md)
make ui             # browser UI to upload and test
```

Without `make`, run `python3 -m venv .venv && .venv/bin/pip install -e ".[ml,ui]" && scripts/fetch_models.sh`, then `scripts/run_ui.sh`. The LiDAR tier needs only `pip install -e .`, with no torch and no model.

### One command per capture

```bash
propscan run ~/Downloads/2026-10-04_flat/            # Stray Scanner folder  -> LiDAR tier
propscan run ~/Downloads/IMG_0412.MOV                # one video file        -> video tier
propscan run ~/Downloads/my_flat/                    # folder of room folders -> photo tier, stitched
propscan run ~/Downloads/kitchen/                    # folder of images      -> photo tier, one room
propscan run ~/Downloads/capture.zip                 # any of the above, zipped
```

The tier is detected from the layout (`propscan detect <path>` prints the decision). `--tier` overrides it. Outputs go to `out/<capture name>/`, or wherever `-o` points:

- `plan.json`: the full contract, validated against [`schema/plan.schema.json`](schema/plan.schema.json) (`propscan validate plan.json`). Every measurement is `{value, lo, hi, sigma, unit}`.
- `plan.png`: the rendered whole-property plan, with dimensions and ± values, doors, windows and damage markers.
- A console summary: rooms, areas, heights, walls, openings, and every warning (mirrors, low light, untracked frames, unstitched rooms).

This works on **any capture you have**, not only ours. Formats are detected by structure, not by file names. HEIC, JPEG and PNG all work; EXIF orientation and 35 mm-equivalent focal length are read automatically, with a fallback of 26 mm (iPhone main camera) when EXIF is missing. Portrait or landscape and `.MOV`/`.MP4`/`.M4V` are all accepted, and macOS `__MACOSX` zip debris is ignored. Useful flags:

| Flag | Effect |
|---|---|
| `--no-drift-correction` | poses as-is (the ablation) |
| `--depth model` | force the live depth model even when synthetic depth sidecars exist |
| `--config my.yaml` | deep-merge override of [`configs/default.yaml`](configs/default.yaml) |

Model outputs are cached under `<capture>/.cache/depth/` keyed by image hash. Re-runs replay them deterministically, and a new capture always runs the model live.

## Benchmark, calibration, fix loop

```bash
make bench          # synthesise the benchmark set, run all captures, score every gate -> reports/synthetic/REPORT.md
make bench-model    # photo + video tiers with the live depth model (instead of emulated depth)
make calibrate      # fit per-tier interval multipliers (writes configs/calibration.json, with leave-one-out check)
make fixloop        # regenerate fix-loop before/after from git tags fixloop-before / fixloop-after + diff
make test           # unit tests (~2 s)
make real           # real benchmark, once benchmark/real/ is captured (see benchmark/README.md)
```

| Document | Contents |
|---|---|
| [`docs/COMPLIANCE_MATRIX.md`](docs/COMPLIANCE_MATRIX.md) | requirement → file → artifact → status |
| [`docs/TECHNICAL_REPORT.md`](docs/TECHNICAL_REPORT.md) | architecture, tiers, drift, error budget, calibration, fix loop, failure modes (≤ 6 pages) |
| [`docs/FIX_DECLARATION.md`](docs/FIX_DECLARATION.md) and [`docs/FIX_RESULT.md`](docs/FIX_RESULT.md) | the fix loop: declared before the fix, then the result and post-mortem |
| [`reports/synthetic/REPORT.md`](reports/synthetic/REPORT.md) | the benchmark report: gates for all tiers, repeatability, ablation, damage, timing |
| [`benchmark/README.md`](benchmark/README.md) | real-benchmark layout, laser/tape ground-truth procedure, scoring rules |

## What is and isn't proven

The numbers in `reports/` come from a **synthetic** benchmark: ray-cast rooms with exact ground truth, emulated LiDAR noise, VIO drift and mono-depth error. They show that the pipeline logic, the scorer and the gates work, and they drove the fix loop. They are **not** real-world accuracy. The real benchmark (tape and laser ground truth, consumer-app head-to-head) has a procedure and tooling here (`benchmark/README.md`, `scripts/head_to_head.py`), but its numbers exist only once the rooms are captured. Nothing in this repo claims them before that.

## Layout

```
propscan/ingest      tier detection, Stray Scanner / video / photo loaders (EXIF, HEIC, zip)
propscan/depth       synthetic sidecar | Depth Anything V2 (cached) | focal rescale + camera-height scale fusion
propscan/pose        RGB-D PnP odometry (video), SE(2) pose graph drift correction (loop closures + Manhattan priors)
propscan/geometry    point cloud, gravity/Manhattan alignment, room segmentation, walls, levels, openings, mirrors
propscan/photo       photo-tier room stitching (door pairing, appearance, connections.txt hints)
propscan/damage      per-surface orthophotos, stain/mold/crack detectors, concealed-damage rules, scope items
propscan/output      plan JSON (schema), interval calibration, renderer
propscan/bench       frame-free GT matching, gates, report
propscan/synth       synthetic scene ray caster + Stray/video/photo exporters
```
