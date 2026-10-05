#!/usr/bin/env bash
# Fetch model weights into models/ (gitignored). Only the photo and video tiers need them; LiDAR runs without.
#   Depth Anything V2 Metric-Indoor (Small, ~100 MB, Apache-2.0): monocular metric depth for RGB-only tiers.
# Set MODEL=depth-anything/Depth-Anything-V2-Metric-Indoor-Base-hf for the larger variant, and point
# depth_model.name at it in a config override.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PY="${PY:-$ROOT/.venv/bin/python}"
MODEL="${MODEL:-depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf}"
"$PY" - "$MODEL" "$ROOT/models" <<'EOF'
import sys
from pathlib import Path
from huggingface_hub import snapshot_download
model, root = sys.argv[1], Path(sys.argv[2])
dst = root / model.split("/")[-1]
snapshot_download(model, local_dir=str(dst))
print(f"[fetch_models] {model} -> {dst}")
EOF
