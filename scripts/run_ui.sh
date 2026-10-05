#!/usr/bin/env bash
# Local test UI (Streamlit). Install once: pip install -e ".[ui,ml]"
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
exec "${PY:-$ROOT/.venv/bin/python}" -m streamlit run ui/app.py --server.headless true
