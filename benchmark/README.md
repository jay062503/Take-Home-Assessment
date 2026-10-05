# Benchmark data

Two benchmark sets share one manifest format and one scorer (`propscan bench <manifest.yaml>`).

| Set | Where | Status |
|---|---|---|
| **Synthetic** (`scenes/apartment_a.yaml`) | generated into `data/synthetic/apartment_a/` by `propscan synth` (gitignored, deterministic seeds) | built; reports in `reports/synthetic*/` |
| **Real** | `benchmark/real/` (this folder), raw sensor data shipped as a release archive | **to be captured**: layout and procedure below; no real numbers are claimed until it exists |

The synthetic set mirrors the required composition: one multi-room capture (living, bedroom and kitchen plus a hall connector) at all three tiers, the furnished bedroom with staged damage in two classes (water stain and mold; plus a crack in the living room), and the bedroom captured twice at the LiDAR and photo tiers. Its ground truth is exact, because it is the scene definition. It tests pipeline logic and the scorer, not real-world accuracy.

## Real benchmark layout

```
benchmark/real/
  manifest.yaml
  flat_lidar/capture/        Stray Scanner export (rgb.mp4, depth/, confidence/, odometry.csv, camera_matrix.csv, imu.csv)
  flat_lidar/gt.yaml
  flat_video/capture/IMG_0001.MOV
  flat_photo/capture/<room>/IMG_*.HEIC     one folder per room
  bedroom_lidar_r1/ ...     repeat captures (same room, same tier)
  bedroom_lidar_r2/ ...
  head_to_head/APP.txt       consumer app name + version + date
  head_to_head/<app export>  the app's own export (PDF / JSON / USDZ), unmodified
  head_to_head/consumer.csv  dimensions read off that export (format: scripts/head_to_head.py)
  measurements/              photos of the tape/laser readings, raw notes
```

`manifest.yaml` uses the same format as the synthetic one. Several captures may share one `gt.yaml` when they cover the same rooms:

```yaml
synthetic: false
captures:
  - {id: flat_lidar, tier: lidar, path: flat_lidar/capture, gt: gt_flat.yaml, repeat_group: null}
  - {id: flat_video, tier: video, path: flat_video/capture, gt: gt_flat.yaml, repeat_group: null}
  - {id: flat_photo, tier: photo, path: flat_photo/capture, gt: gt_flat.yaml, repeat_group: null}
  - {id: bedroom_lidar_r1, tier: lidar, path: bedroom_lidar_r1/capture, gt: gt_bedroom.yaml, repeat_group: bedroom-lidar}
  - {id: bedroom_lidar_r2, tier: lidar, path: bedroom_lidar_r2/capture, gt: gt_bedroom.yaml, repeat_group: bedroom-lidar}
```

## Ground-truth procedure (laser plus tape)

Tools: a laser distance meter (±1.5 mm class), a 5 m steel tape, and a spirit level. Measure every value **twice** and record both readings. If they differ by more than 3 mm, measure a third time and take the median.

1. **Walls:** measure each wall's interior length at about 1 m height, corner to corner, with the laser square to the wall. Number the walls counter-clockwise from the room's entry door.
2. **Ceiling height:** take a laser reading floor to ceiling at the room centre and in two corners about 30 cm from the walls. GT is the mean; the spread is recorded and shows how flat the ceiling is.
3. **Openings:** measure width with the tape **between the jambs** (not the casing) at mid-height; height from floor (or sill) to head; sill height for windows. Also measure the opening's centre offset from the wall's start corner.
4. **Polygon:** write the room corners as coordinates in a frame you choose (start at a corner, x along wall 0). For rectilinear rooms the corners follow directly from the wall lengths. Close the loop: if opposite walls disagree by more than 1 cm, re-measure.
5. **Damage:** measure each staged region's bounding width and height with the tape, and note its surface (`wall` or `ceiling`) and class.
6. Photograph the laser display for every reading into `measurements/`.

### `gt.yaml`

```yaml
source: laser+tape
rooms:
  - name: bedroom
    polygon: [[0, 0], [3.212, 0], [3.212, 2.498], [0, 2.498]]   # interior corners, metres
    ceiling_height: 2.574
    openings:
      - {type: door, width: 0.812, height: 2.035, center: [3.212, 0.62], to: hall}
      - {type: window, width: 1.198, height: 1.20, center: [1.60, 2.498]}
adjacency: [[bedroom, hall], [hall, living]]
damage:
  - {class: water_stain, room: bedroom, surface: ceiling, size: [0.55, 0.45]}
```

`center` is optional. Without it, openings are matched by type and closest width, which is weaker; the report flags this.

## Scoring rules (implemented in `propscan/bench/`)

- Rooms are matched by best-fit IoU, up to a 90° rotation and a translation, using geometry only and never names. A rectangle's 180° ambiguity is resolved by opening agreement.
- Walls are matched by orientation, distance and overlap. A GT wall with no prediction is a **miss**, and an unmatched predicted wall is a **phantom**. Both count against the wall-length gate.
- An opening scores a hit if its width is within 2 cm. Score = hits / (GT openings + phantom openings).
- Calibration: the fraction of truths inside the reported 90% interval, per tier.
