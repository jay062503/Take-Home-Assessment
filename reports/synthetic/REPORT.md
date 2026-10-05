# Benchmark report

Generated 2026-10-06 01:54:34 from `manifest.yaml` (interval calibration: **prior**).

> **Synthetic benchmark.** Scenes are ray-cast with known geometry; depth noise, VIO drift and mono-depth error are emulated. These numbers test the pipeline logic and the gate scorer. They are **not** real-world accuracy claims; real-capture numbers come from `benchmark/real/`.

## Gates

| Gate | Tier | Value | Threshold | Detail | Result |
|---|---|---|---|---|---|
| opening_width | lidar | 69% | >= 85% within 2 cm | 11 hits / (14 GT + 2 phantom) | **FAIL** |
| ceiling_height | lidar | 0.7 cm | <= 1.5 cm every room | 6/6 rooms within 1.5 cm | PASS |
| wall_length | lidar | 85% | >= 90% of GT walls within ±1% (own target) | 22/26 (missed walls count as failures: 2) | **FAIL** |
| calibration | lidar | 88% | 90% intervals cover 80-97% of truths | 44/50 measurements covered | PASS |
| opening_width | photo | 0% | >= 85% within 2 cm | 0 hits / (14 GT + 3 phantom) | **FAIL** |
| ceiling_height | photo | 12.9 cm | <= 1.5 cm every room | 2/6 rooms within 1.5 cm | **FAIL** |
| wall_length | photo | 42% | >= 90% of GT walls within ±8% | 11/26 (missed walls count as failures: 5) | **FAIL** |
| calibration | photo | 50% | 90% intervals cover 80-97% of truths | 19/38 measurements covered | **FAIL** |
| photo_stitch | photo | +6.0% | footprint ±8%, correct adjacency, no overlaps, all rooms placed | photo_full: footprint 37.95 vs 35.79 m2, overlaps 0, unresolved ['bedroom', 'kitchen', 'hall'], adjacency wrong | **FAIL** |
| opening_width | video | 33% | >= 85% within 2 cm | 4 hits / (10 GT + 2 phantom) | **FAIL** |
| ceiling_height | video | 10.8 cm | <= 1.5 cm every room | 0/4 rooms within 1.5 cm | **FAIL** |
| wall_length | video | 56% | >= 90% of GT walls within ±3% | 10/18 (missed walls count as failures: 3) | **FAIL** |
| calibration | video | 78% | 90% intervals cover 80-97% of truths | 25/32 measurements covered | **FAIL** |
| repeatability | lidar | 0.5 cm | every wall within max(1 cm, 0.5%) across captures | bedroom-lidar: 4/4 walls | PASS |
| ceiling_spread | lidar | 0.1 cm | spread <= 1 cm and |bias| <= 1.5 cm | bedroom-lidar/bedroom: spread 0.08 cm, bias -0.06 cm -> repeatable and accurate | PASS |
| repeatability | photo | 7.8 cm | every wall within max(1 cm, 0.5%) across captures | bedroom-photo: 0/3 walls | **FAIL** |
| ceiling_spread | photo | 0.1 cm | spread <= 1 cm and |bias| <= 1.5 cm | bedroom-photo/bedroom: spread 0.07 cm, bias +0.65 cm -> repeatable and accurate | PASS |
| drift_accountability | lidar | 30.2 cm | drift handled (pose graph) + on/off ablation; on must not be worse | lidar_full: wall MAE 30.2 cm on vs 1.0 cm off; footprint err -1.9% on vs -0.4% off; 17 loop closures | **FAIL** |
| drift_accountability | video | 8.3 cm | drift handled (pose graph) + on/off ablation; on must not be worse | video_full: wall MAE 8.3 cm on vs 13.0 cm off; footprint err -7.9% on vs -18.0% off; 1 loop closures | PASS |

## Accuracy by tier and quantity

Errors are prediction minus ground truth. Coverage is the fraction of truths inside the reported 90% interval. z90 is the 90th percentile of |error| / sigma, which is 1.645 when the intervals are calibrated.

| Tier | Quantity | n | Mean error | MAE | Max abs error | Coverage | z90 |
|---|---|---|---|---|---|---|---|
| lidar | ceiling_height | 6 | 0.0 cm | 0.3 cm | 0.7 cm | 100% | 0.90 |
| lidar | floor_area | 6 | -0.475 m2 | 0.490 m2 | 2.894 m2 | 83% | 41.34 |
| lidar | footprint_area | 3 | 1.350 m2 | 1.795 m2 | 2.714 m2 | 0% | 27.66 |
| lidar | opening_width | 11 | 0.3 cm | 0.3 cm | 0.8 cm | 100% | 0.35 |
| lidar | wall_length | 24 | -19.8 cm | 20.2 cm | 251.5 cm | 92% | 1.30 |
| photo | ceiling_height | 6 | -0.5 cm | 5.3 cm | 12.9 cm | 100% | 0.51 |
| photo | floor_area | 6 | 1.063 m2 | 1.610 m2 | 2.329 m2 | 0% | 16.66 |
| photo | footprint_area | 3 | 2.126 m2 | 2.126 m2 | 2.229 m2 | 0% | 16.12 |
| photo | opening_width | 2 | -65.7 cm | 65.7 cm | 88.0 cm | 0% | 15.41 |
| photo | wall_length | 21 | 0.1 cm | 32.8 cm | 136.4 cm | 62% | 2.94 |
| video | ceiling_height | 4 | 7.6 cm | 7.6 cm | 10.8 cm | 100% | 0.56 |
| video | floor_area | 4 | -0.708 m2 | 1.249 m2 | 3.915 m2 | 0% | 23.50 |
| video | footprint_area | 1 | -2.834 m2 | 2.834 m2 | 2.834 m2 | 0% | 14.35 |
| video | opening_width | 8 | 9.4 cm | 9.5 cm | 58.8 cm | 88% | 2.13 |
| video | wall_length | 15 | 2.2 cm | 8.3 cm | 38.6 cm | 93% | 0.64 |

## Per capture

### lidar_full (lidar)

Rooms matched 4/4 (room_0->bedroom IoU 1.00, room_1->kitchen IoU 1.00, room_2->hall IoU 0.47, room_3->living IoU 1.00). Footprint 35.12 m2 vs 35.79 m2 (-1.9%). Adjacency WRONG: [('?room_4', 'bedroom'), ('hall', 'kitchen'), ('hall', 'living')].

| Room | Opening | GT width | Predicted | Error | Within 2 cm |
|---|---|---|---|---|---|
| bedroom | door | 0.800 | 0.800 | -0.0 cm | yes |
| bedroom | window | 1.200 | 1.205 | +0.5 cm | yes |
| kitchen | window | 1.000 | 1.001 | +0.1 cm | yes |
| kitchen | door | 0.850 | 0.851 | +0.1 cm | yes |
| living | window | 1.400 | 1.402 | +0.2 cm | yes |
| living | window | 1.000 | 1.003 | +0.3 cm | yes |
| living | door | 0.900 | 0.906 | +0.6 cm | yes |

Missed openings: ['hall:door 0.90', 'hall:door 0.80', 'hall:door 0.85']. Phantom openings: ['hall:room_2/opening_0 (door 0.65)', 'hall:room_2/opening_1 (door 0.85)'].


Damage: 2 detected / 4 staged, 8 false positives. Missed: bedroom water_stain on ceiling, living crack on wall.

Timing: ingest 1.3 s, drift 33.0 s, structure 32.7 s, damage+scope 40.6 s, total 107.7 s.

### lidar_bedroom_r1 (lidar)

Rooms matched 1/1 (room_0->bedroom IoU 1.00). Footprint 10.00 m2 vs 8.00 m2 (+25.1%). Adjacency WRONG: [('?room_1', 'bedroom')].

| Room | Opening | GT width | Predicted | Error | Within 2 cm |
|---|---|---|---|---|---|
| bedroom | door | 0.800 | 0.808 | +0.8 cm | yes |
| bedroom | window | 1.200 | 1.203 | +0.3 cm | yes |

Damage: 3 detected / 3 staged, 0 false positives. 

Timing: ingest 0.3 s, drift 7.7 s, structure 6.7 s, damage+scope 10.5 s, total 25.2 s.

### lidar_bedroom_r2 (lidar)

Rooms matched 1/1 (room_0->bedroom IoU 1.00). Footprint 10.71 m2 vs 8.00 m2 (+33.9%). Adjacency WRONG: [('?room_1', 'bedroom')].

| Room | Opening | GT width | Predicted | Error | Within 2 cm |
|---|---|---|---|---|---|
| bedroom | door | 0.800 | 0.800 | +0.0 cm | yes |
| bedroom | window | 1.200 | 1.204 | +0.4 cm | yes |

Damage: 3 detected / 3 staged, 0 false positives. 

Timing: ingest 0.3 s, drift 7.9 s, structure 6.5 s, damage+scope 10.7 s, total 25.3 s.

### video_full (video)

Rooms matched 4/4 (room_0->living IoU 0.96, room_1->hall IoU 0.94, room_2->kitchen IoU 0.96, room_3->bedroom IoU 0.50). Footprint 32.95 m2 vs 35.79 m2 (-7.9%). Adjacency WRONG: [('hall', 'kitchen'), ('hall', 'living')].

| Room | Opening | GT width | Predicted | Error | Within 2 cm |
|---|---|---|---|---|---|
| living | door | 0.900 | 0.897 | -0.3 cm | yes |
| living | window | 1.400 | 1.468 | +6.8 cm | no |
| living | window | 1.000 | 1.029 | +2.9 cm | no |
| hall | door | 0.900 | 1.488 | +58.8 cm | no |
| hall | door | 0.800 | 0.818 | +1.8 cm | yes |
| hall | door | 0.850 | 0.875 | +2.5 cm | no |
| kitchen | window | 1.000 | 1.014 | +1.4 cm | yes |
| kitchen | door | 0.850 | 0.864 | +1.4 cm | yes |

Missed openings: ['bedroom:door 0.80', 'bedroom:window 1.20']. Phantom openings: ['living:room_0/opening_0 (window 0.47)', 'bedroom:room_3/opening_0 (door 0.84)'].


Damage: 3 detected / 4 staged, 9 false positives. Missed: living crack on wall.

Timing: ingest+depth+odometry 40.7 s, drift 99.8 s, structure 91.3 s, damage+scope 24.1 s, total 256.0 s.

### photo_full (photo)

Rooms matched 4/4 (bedroom->bedroom IoU 0.73, hall->kitchen IoU 0.81, kitchen->hall IoU 0.61, living->living IoU 0.94). Footprint 37.95 m2 vs 35.79 m2 (+6.0%). Adjacency WRONG: []; unresolved ['bedroom', 'kitchen', 'hall'].

| Room | Opening | GT width | Predicted | Error | Within 2 cm |
|---|---|---|---|---|---|
| living | window | 1.400 | 0.520 | -88.0 cm | no |

Missed openings: ['bedroom:door 0.80', 'bedroom:window 1.20', 'kitchen:door 0.85', 'kitchen:window 1.00', 'hall:door 0.90', 'hall:door 0.80', 'hall:door 0.85', 'living:door 0.90', 'living:window 1.00']. Phantom openings: ['bedroom:bedroom/opening_0 (window 0.50)', 'hall:kitchen/opening_0 (window 1.00)'].


Damage: 2 detected / 4 staged, 2 false positives. Missed: bedroom water_stain on ceiling, living crack on wall.

Timing: ingest+depth+registration 14.4 s, rooms+stitch 13.0 s, damage+scope 13.0 s, total 40.4 s.

### photo_bedroom_r1 (photo)

Rooms matched 1/1 (bedroom->bedroom IoU 0.73). Footprint 9.99 m2 vs 8.00 m2 (+24.8%). Adjacency correct.

| Room | Opening | GT width | Predicted | Error | Within 2 cm |
|---|---|---|---|---|---|
| bedroom | window | 1.200 | 0.765 | -43.5 cm | no |

Missed openings: ['bedroom:door 0.80']. Phantom openings: none.


Damage: 1 detected / 3 staged, 0 false positives. Missed: bedroom water_stain on ceiling, bedroom mold on wall.

Timing: ingest+depth+registration 3.5 s, rooms+stitch 1.8 s, damage+scope 4.1 s, total 9.4 s.

### photo_bedroom_r2 (photo)

Rooms matched 1/1 (bedroom->bedroom IoU 0.72). Footprint 10.23 m2 vs 8.00 m2 (+27.9%). Adjacency correct.


Missed openings: ['bedroom:door 0.80', 'bedroom:window 1.20']. Phantom openings: ['bedroom:bedroom/opening_0 (window 0.50)'].


Damage: 1 detected / 3 staged, 0 false positives. Missed: bedroom water_stain on ceiling, bedroom mold on wall.

Timing: ingest+depth+registration 3.4 s, rooms+stitch 0.9 s, damage+scope 3.3 s, total 7.6 s.

## Repeatability

| Group | Wall | GT | Captures | Spread | Tolerance | Pass |
|---|---|---|---|---|---|---|
| bedroom-lidar | bedroom/0 | 3.200 | 3.202, 3.196 | 0.5 cm | 1.6 cm | yes |
| bedroom-lidar | bedroom/1 | 2.500 | 2.501, 2.502 | 0.1 cm | 1.2 cm | yes |
| bedroom-lidar | bedroom/2 | 3.200 | 3.202, 3.196 | 0.5 cm | 1.6 cm | yes |
| bedroom-lidar | bedroom/3 | 2.500 | 2.501, 2.502 | 0.1 cm | 1.2 cm | yes |
| bedroom-photo | bedroom/0 | 3.200 | 2.576, 2.624 | 4.9 cm | 1.6 cm | no |
| bedroom-photo | bedroom/1 | 2.500 | 2.497, 2.528 | 3.0 cm | 1.2 cm | no |
| bedroom-photo | bedroom/2 | 3.200 | 4.486, 4.564 | 7.8 cm | 1.6 cm | no |

## Drift ablation (pose-graph correction on vs off)

| Capture | Tier | Wall MAE on | Wall MAE off | Footprint err on | Footprint err off | Walls matched on/off | Loop closures | Max correction |
|---|---|---|---|---|---|---|---|---|
| lidar_full | lidar | 30.2 cm | 1.0 cm | -1.9% | -0.4% | 16/18 | 17 | 9.4 cm |
| video_full | video | 8.3 cm | 13.0 cm | -7.9% | -18.0% | 15/10 | 1 | 27.0 cm |

Plans for both runs are in `captures/<id>/plan.png` and `captures/<id>__drift_off/plan.png`.


## Timing (end-to-end per capture)

| Capture | Seconds |
|---|---|
| lidar_full | 107.7 |
| lidar_bedroom_r1 | 25.2 |
| lidar_bedroom_r2 | 25.3 |
| video_full | 256.0 |
| photo_full | 40.4 |
| photo_bedroom_r1 | 9.4 |
| photo_bedroom_r2 | 7.6 |
