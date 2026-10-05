from __future__ import annotations

import copy
import json
from pathlib import Path
from typing import Any, Dict, Optional

import yaml

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG = ROOT / "configs" / "default.yaml"
CALIBRATION_FILE = ROOT / "configs" / "calibration.json"


def deep_merge(base: Dict, over: Dict) -> Dict:
    out = copy.deepcopy(base)
    for k, v in (over or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def load_config(path: Optional[str] = None, overrides: Optional[Dict[str, Any]] = None) -> Dict:
    with open(DEFAULT_CONFIG) as f:
        cfg = yaml.safe_load(f)
    if path:
        with open(path) as f:
            cfg = deep_merge(cfg, yaml.safe_load(f) or {})
    if overrides:
        cfg = deep_merge(cfg, overrides)
    cfg["calibration"] = load_calibration()
    return cfg


def load_calibration(path: Path = CALIBRATION_FILE) -> Dict:
    """Per-tier, per-quantity sigma multipliers. Missing file -> priors (multiplier 1)."""
    if path.exists():
        with open(path) as f:
            data = json.load(f)
        data.setdefault("status", "calibrated")
        return data
    return {"status": "prior", "factors": {}}


def calib_factor(cfg: Dict, tier: str, quantity: str) -> float:
    return float(cfg.get("calibration", {}).get("factors", {}).get(tier, {}).get(quantity, 1.0))
