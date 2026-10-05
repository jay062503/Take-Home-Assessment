# Benchmark report

Generated 2026-10-04 23:34:10 from `manifest.yaml` (interval calibration: **prior**).

> **Synthetic benchmark.** Scenes are ray-cast with known geometry; depth noise, VIO drift and mono-depth error are emulated. These numbers test the pipeline logic and the gate scorer. They are **not** real-world accuracy claims; real-capture numbers come from `benchmark/real/`.

## Gates

| Gate | Tier | Value | Threshold | Detail | Result |
|---|---|---|---|---|---|
| opening_width | lidar | 93% | >= 85% within 2 cm | 13 hits / (14 GT + 0 phantom) | PASS |
| ceiling_height | lidar | 0.4 cm | <= 1.5 cm every room | 6/6 rooms within 1.5 cm | PASS |
| wall_length | lidar | 96% | >= 90% of GT walls within ±1% (own target) | 25/26 (missed walls count as failures: 0) | PASS |
| calibration | lidar | 93% | 90% intervals cover 80-97% of truths | 51/55 measurements covered | PASS |
| opening_width | photo | 0% | >= 85% within 2 cm | 0 hits / (14 GT + 3 phantom) | **FAIL** |
| ceiling_height | photo | 18.8 cm | <= 1.5 cm every room | 0/6 rooms within 1.5 cm | **FAIL** |
| wall_length | photo | 42% | >= 90% of GT walls within ±8% | 11/26 (missed walls count as failures: 5) | **FAIL** |
| calibration | photo | 46% | 90% intervals cover 80-97% of truths | 19/41 measurements covered | **FAIL** |
| photo_stitch | photo | +20.2% | footprint ±8%, correct adjacency, no overlaps, all rooms placed | photo_full: footprint 43.01 vs 35.79 m2, overlaps 0, unresolved ['kitchen', 'bedroom', 'hall'], adjacency wrong | **FAIL** |
| opening_width | video | 13% | >= 85% within 2 cm | 2 hits / (10 GT + 5 phantom) | **FAIL** |
| ceiling_height | video | 3.8 cm | <= 1.5 cm every room | 1/3 rooms within 1.5 cm | **FAIL** |
| wall_length | video | 39% | >= 90% of GT walls within ±3% | 7/18 (missed walls count as failures: 6) | **FAIL** |
| calibration | video | 65% | 90% intervals cover 80-97% of truths | 15/23 measurements covered | **FAIL** |
| repeatability | lidar | 0.4 cm | every wall within max(1 cm, 0.5%) across captures | bedroom-lidar: 4/4 walls | PASS |
| ceiling_spread | lidar | 0.3 cm | spread <= 1 cm and |bias| <= 1.5 cm | bedroom-lidar/bedroom: spread 0.33 cm, bias -0.19 cm -> repeatable and accurate | PASS |
| repeatability | photo | 7.3 cm | every wall within max(1 cm, 0.5%) across captures | bedroom-photo: 1/3 walls | **FAIL** |
| ceiling_spread | photo | 4.7 cm | spread <= 1 cm and |bias| <= 1.5 cm | bedroom-photo/bedroom: spread 4.74 cm, bias -8.78 cm -> unrepeatable | **FAIL** |
| drift_accountability | lidar | 0.5 cm | drift handled (pose graph) + on/off ablation; on must not be worse | lidar_full: wall MAE 0.5 cm on vs 4.2 cm off; footprint err +0.2% on vs +3.6% off; 7 loop closures | PASS |
| drift_accountability | video | 36.4 cm | drift handled (pose graph) + on/off ablation; on must not be worse | video_full: wall MAE 36.4 cm on vs 28.3 cm off; footprint err -0.5% on vs -0.5% off; 5 loop closures | **FAIL** |

## Accuracy by tier and quantity

Errors are prediction minus ground truth. Coverage is the fraction of truths inside the reported 90% interval. z90 is the 90th percentile of |error| / sigma, which is 1.645 when the intervals are calibrated.

| Tier | Quantity | n | Mean error | MAE | Max abs error | Coverage | z90 |
|---|---|---|---|---|---|---|---|
| lidar | ceiling_height | 6 | -0.1 cm | 0.2 cm | 0.4 cm | 100% | 0.47 |
| lidar | floor_area | 6 | 0.008 m2 | 0.017 m2 | 0.046 m2 | 100% | 1.08 |
| lidar | footprint_area | 3 | 0.749 m2 | 0.759 m2 | 2.203 m2 | 67% | 18.98 |
| lidar | opening_width | 14 | 0.5 cm | 0.6 cm | 3.0 cm | 93% | 0.90 |
| lidar | wall_length | 26 | 0.1 cm | 0.4 cm | 2.4 cm | 92% | 0.71 |
| photo | ceiling_height | 6 | -8.5 cm | 9.8 cm | 18.8 cm | 100% | 1.20 |
| photo | floor_area | 6 | 1.408 m2 | 1.837 m2 | 4.967 m2 | 17% | 14.81 |
| photo | footprint_area | 3 | 2.817 m2 | 2.817 m2 | 7.226 m2 | 33% | 16.35 |
| photo | opening_width | 5 | -50.6 cm | 50.6 cm | 77.5 cm | 0% | 16.12 |
| photo | wall_length | 21 | -0.5 cm | 56.4 cm | 312.8 cm | 52% | 6.94 |
| video | ceiling_height | 3 | -2.4 cm | 2.4 cm | 3.8 cm | 100% | 0.52 |
| video | floor_area | 3 | 1.786 m2 | 1.959 m2 | 5.618 m2 | 33% | 43.27 |
| video | footprint_area | 1 | -0.162 m2 | 0.162 m2 | 0.162 m2 | 100% | 1.01 |
| video | opening_width | 4 | 10.1 cm | 10.1 cm | 23.6 cm | 50% | 5.49 |
| video | wall_length | 12 | 24.1 cm | 36.4 cm | 198.8 cm | 67% | 10.88 |

## Per capture

### lidar_full (lidar)

Rooms matched 4/4 (room_0->bedroom IoU 1.00, room_1->living IoU 1.00, room_2->hall IoU 0.99, room_3->kitchen IoU 1.00). Footprint 35.85 m2 vs 35.79 m2 (+0.2%). Adjacency correct.

| Room | Opening | GT width | Predicted | Error | Within 2 cm |
|---|---|---|---|---|---|
| bedroom | window | 1.200 | 1.205 | +0.5 cm | yes |
| bedroom | door | 0.800 | 0.803 | +0.3 cm | yes |
| living | door | 0.900 | 0.900 | -0.0 cm | yes |
| living | window | 1.400 | 1.419 | +1.9 cm | yes |
| living | window | 1.000 | 1.001 | +0.1 cm | yes |
| hall | door | 0.800 | 0.802 | +0.2 cm | yes |
| hall | door | 0.850 | 0.880 | +3.0 cm | no |
| hall | door | 0.900 | 0.900 | -0.0 cm | yes |
| kitchen | window | 1.000 | 1.002 | +0.2 cm | yes |
| kitchen | door | 0.850 | 0.849 | -0.1 cm | yes |

Damage: 2 detected / 4 staged, 3 false positives. Missed: bedroom mold on wall, living crack on wall.

Timing: ingest 0.6 s, drift 20.1 s, structure 14.8 s, damage+scope 31.0 s, total 66.5 s.

### lidar_bedroom_r1 (lidar)

Rooms matched 1/1 (room_0->bedroom IoU 1.00). Footprint 7.99 m2 vs 8.00 m2 (-0.2%). Adjacency correct.

| Room | Opening | GT width | Predicted | Error | Within 2 cm |
|---|---|---|---|---|---|
| bedroom | window | 1.200 | 1.203 | +0.3 cm | yes |
| bedroom | door | 0.800 | 0.797 | -0.3 cm | yes |

Damage: 1 detected / 3 staged, 0 false positives. Missed: bedroom water_stain on ceiling, bedroom mold on wall.

Timing: ingest 0.1 s, drift 2.2 s, structure 2.6 s, damage+scope 6.5 s, total 11.4 s.

### lidar_bedroom_r2 (lidar)

Rooms matched 1/1 (room_0->bedroom IoU 1.00). Footprint 10.20 m2 vs 8.00 m2 (+27.5%). Adjacency WRONG: [('?room_1', 'bedroom')].

| Room | Opening | GT width | Predicted | Error | Within 2 cm |
|---|---|---|---|---|---|
| bedroom | window | 1.200 | 1.203 | +0.3 cm | yes |
| bedroom | door | 0.800 | 0.808 | +0.8 cm | yes |

Damage: 2 detected / 3 staged, 0 false positives. Missed: bedroom water_stain on ceiling.

Timing: ingest 0.1 s, drift 2.4 s, structure 2.4 s, damage+scope 9.4 s, total 14.4 s.

### video_full (video)

Rooms matched 3/4 (room_0->living IoU 0.95, room_1->bedroom IoU 0.58, room_2->kitchen IoU 0.98). Footprint 35.63 m2 vs 35.79 m2 (-0.5%). Adjacency WRONG: [('bedroom', 'kitchen'), ('bedroom', 'living')].

Missed rooms: hall.

| Room | Opening | GT width | Predicted | Error | Within 2 cm |
|---|---|---|---|---|---|
| living | window | 1.000 | 1.236 | +23.6 cm | no |
| living | door | 0.900 | 0.904 | +0.4 cm | yes |
| kitchen | window | 1.000 | 1.153 | +15.3 cm | no |
| kitchen | door | 0.850 | 0.862 | +1.2 cm | yes |

Missed openings: ['living:window 1.40', 'bedroom:door 0.80', 'bedroom:window 1.20', 'hall:door 0.90 (room missing)', 'hall:door 0.80 (room missing)', 'hall:door 0.85 (room missing)']. Phantom openings: ['living:room_0/opening_2 (door 1.45)', 'bedroom:room_1/opening_0 (door 0.89)', 'bedroom:room_1/opening_1 (window 1.16)', 'bedroom:room_1/opening_2 (window 1.23)', 'bedroom:room_1/opening_3 (door 0.82)'].


Damage: 3 detected / 4 staged, 20 false positives. Missed: living crack on wall.

Timing: ingest+depth+odometry 10.1 s, drift 26.8 s, structure 21.3 s, damage+scope 9.1 s, total 67.3 s.

### photo_full (photo)

Rooms matched 4/4 (bedroom->bedroom IoU 0.73, hall->hall IoU 0.77, kitchen->kitchen IoU 0.51, living->living IoU 0.79). Footprint 43.01 m2 vs 35.79 m2 (+20.2%). Adjacency WRONG: []; unresolved ['kitchen', 'bedroom', 'hall'].

| Room | Opening | GT width | Predicted | Error | Within 2 cm |
|---|---|---|---|---|---|
| bedroom | window | 1.200 | 0.760 | -44.0 cm | no |
| kitchen | window | 1.000 | 0.794 | -20.6 cm | no |
| living | window | 1.400 | 0.625 | -77.5 cm | no |

Missed openings: ['bedroom:door 0.80', 'hall:door 0.90', 'hall:door 0.80', 'hall:door 0.85', 'kitchen:door 0.85', 'living:door 0.90', 'living:window 1.00']. Phantom openings: ['bedroom:bedroom/opening_0 (window 0.64)'].


Damage: 2 detected / 4 staged, 6 false positives. Missed: bedroom water_stain on ceiling, living crack on wall.

Timing: ingest+depth+registration 4.3 s, rooms+stitch 3.9 s, damage+scope 7.5 s, total 15.8 s.

### photo_bedroom_r1 (photo)

Rooms matched 1/1 (bedroom->bedroom IoU 0.82). Footprint 8.11 m2 vs 8.00 m2 (+1.4%). Adjacency correct.

| Room | Opening | GT width | Predicted | Error | Within 2 cm |
|---|---|---|---|---|---|
| bedroom | window | 1.200 | 0.720 | -48.0 cm | no |

Missed openings: ['bedroom:door 0.80']. Phantom openings: none.


Damage: 1 detected / 3 staged, 1 false positives. Missed: bedroom water_stain on ceiling, bedroom mold on wall.

Timing: ingest+depth+registration 1.2 s, rooms+stitch 0.2 s, damage+scope 1.8 s, total 3.3 s.

### photo_bedroom_r2 (photo)

Rooms matched 1/1 (bedroom->bedroom IoU 0.74). Footprint 9.11 m2 vs 8.00 m2 (+13.9%). Adjacency correct.

| Room | Opening | GT width | Predicted | Error | Within 2 cm |
|---|---|---|---|---|---|
| bedroom | window | 1.200 | 0.568 | -63.2 cm | no |

Missed openings: ['bedroom:door 0.80']. Phantom openings: ['bedroom:bedroom/opening_0 (window 0.64)', 'bedroom:bedroom/opening_2 (window 0.72)'].


Damage: 1 detected / 3 staged, 0 false positives. Missed: bedroom water_stain on ceiling, bedroom mold on wall.

Timing: ingest+depth+registration 1.2 s, rooms+stitch 0.2 s, damage+scope 1.7 s, total 3.1 s.

## Repeatability

| Group | Wall | GT | Captures | Spread | Tolerance | Pass |
|---|---|---|---|---|---|---|
| bedroom-lidar | bedroom/0 | 3.200 | 3.201, 3.202 | 0.2 cm | 1.6 cm | yes |
| bedroom-lidar | bedroom/1 | 2.500 | 2.495, 2.499 | 0.4 cm | 1.2 cm | yes |
| bedroom-lidar | bedroom/2 | 3.200 | 3.201, 3.202 | 0.2 cm | 1.6 cm | yes |
| bedroom-lidar | bedroom/3 | 2.500 | 2.495, 2.499 | 0.4 cm | 1.2 cm | yes |
| bedroom-photo | bedroom/0 | 3.200 | 2.479, 2.552 | 7.3 cm | 1.6 cm | no |
| bedroom-photo | bedroom/1 | 2.500 | 2.445, 2.405 | 4.0 cm | 1.2 cm | no |
| bedroom-photo | bedroom/2 | 3.200 | 3.109, 3.106 | 0.3 cm | 1.6 cm | yes |

## Drift ablation (pose-graph correction on vs off)

| Capture | Tier | Wall MAE on | Wall MAE off | Footprint err on | Footprint err off | Walls matched on/off | Loop closures | Max correction |
|---|---|---|---|---|---|---|---|---|
| lidar_full | lidar | 0.5 cm | 4.2 cm | +0.2% | +3.6% | 18/18 | 7 | 21.6 cm |
| video_full | video | 36.4 cm | 28.3 cm | -0.5% | -0.5% | 12/13 | 5 | 28.7 cm |

Plans for both runs are in `captures/<id>/plan.png` and `captures/<id>__drift_off/plan.png`.


## Timing (end-to-end per capture)

| Capture | Seconds |
|---|---|
| lidar_full | 66.5 |
| lidar_bedroom_r1 | 11.4 |
| lidar_bedroom_r2 | 14.4 |
| video_full | 67.3 |
| photo_full | 15.8 |
| photo_bedroom_r1 | 3.3 |
| photo_bedroom_r2 | 3.1 |
