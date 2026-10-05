# Your capture data (not in git)

Put **your three sample zips** here. The repo ships **no** sample captures.

## Option A — named zips (recommended)

```text
data/uploads/
  lidar.zip      # Stray Scanner export (odometry.csv, depth/, rgb.mp4, …)
  video.zip      # one walkthrough .MOV/.MP4 (or a folder zip)
  photo.zip      # one folder per room inside the zip
```

Unzip is automatic when you use the **Web UI** or:

```bash
propscan run data/uploads/lidar.zip -o out/lidar
propscan run data/uploads/video.zip -o out/video
propscan run data/uploads/photo.zip -o out/photo
```

## Option B — any names

Drop any `.zip` under `data/uploads/`. CLI and UI detect tier from contents:

| Tier | Inside the zip |
|------|----------------|
| **lidar** | `odometry.csv` + `depth/` |
| **video** | single `.mov` / `.mp4` |
| **photo** | subfolders with `.jpg` / `.heic` per room |

## After upload

- Extracted trees live in `data/uploads/<zip_stem>/` (gitignored).
- Pipeline output: `out/<run_name>/plan.json` and `plan.png`.
- Depth model cache (if used): inside the capture folder under `.cache/depth/`.

## First-time model weights (video & photo only)

```bash
scripts/fetch_models.sh   # ~100 MB into models/ (gitignored)
```

LiDAR tier does not need the depth model.
