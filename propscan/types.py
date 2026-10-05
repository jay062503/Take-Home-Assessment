"""Core data types shared by every tier.

Conventions (internal, after ingest):
  * camera frame: OpenCV (x right, y down, z forward)
  * world frame:  metres, z up (gravity), x/y horizontal
  * T_wc: 4x4 camera-to-world
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

Z90 = 1.6448536269514722  # two-sided 90% normal quantile


@dataclass
class Frame:
    index: int
    timestamp: float
    K: np.ndarray                       # intrinsics at depth resolution
    T_wc: np.ndarray                    # 4x4 camera->world
    depth: Optional[np.ndarray] = None  # HxW float32 metres, 0 = invalid
    confidence: Optional[np.ndarray] = None  # HxW uint8 (0 low .. 2 high)
    rgb: Optional[np.ndarray] = None    # H'xW'x3 uint8 RGB
    K_rgb: Optional[np.ndarray] = None
    name: str = ""
    group: str = ""                     # photo tier: room folder name

    @property
    def center(self) -> np.ndarray:
        return self.T_wc[:3, 3]


@dataclass
class FrameSet:
    tier: str                            # 'lidar' | 'video' | 'photo'
    frames: List[Frame]
    source: str
    depth_source: str                    # e.g. 'arkit-lidar', 'depth-anything-v2-metric-indoor', 'synthetic-sidecar'
    pose_source: str                     # e.g. 'arkit-vio', 'rgbd-pnp-odometry', 'pnp-rotation-registration'
    gravity_known: bool
    meta: Dict = field(default_factory=dict)


@dataclass
class Measure:
    """A measurement with a 1-sigma uncertainty; intervals are 90% two-sided."""
    value: float
    sigma: float
    unit: str = "m"

    def interval(self, z: float = Z90) -> Tuple[float, float]:
        return self.value - z * self.sigma, self.value + z * self.sigma

    def scaled(self, k: float) -> "Measure":
        return Measure(self.value, self.sigma * k, self.unit)

    def to_json(self, ndigits: int = 4) -> Dict:
        lo, hi = self.interval()
        return {
            "value": round(float(self.value), ndigits),
            "lo": round(float(lo), ndigits),
            "hi": round(float(hi), ndigits),
            "sigma": round(float(self.sigma), ndigits + 1),
            "unit": self.unit,
        }


def combine_sigma(*sigmas: float) -> float:
    return float(np.sqrt(np.sum(np.square(sigmas))))


@dataclass
class Opening:
    id: str
    type: str                  # door | window | passage
    wall_index: int
    center_offset: Measure     # along wall, from wall start
    width: Measure
    height: Measure
    sill: Measure
    connects_to: Optional[str] = None
    evidence: Dict = field(default_factory=dict)


@dataclass
class Wall:
    id: str
    start: np.ndarray
    end: np.ndarray
    axis: int                  # 0: wall is a constant-x line, 1: constant-y line
    coord: Measure             # position of the wall plane along its normal axis
    inward: int                # +1 / -1: direction of room interior along the normal axis
    length: Optional[Measure] = None
    support: int = 0
    observed_fraction: float = 0.0


@dataclass
class Room:
    id: str
    polygon: np.ndarray        # (N,2) metres, world frame (Manhattan-aligned frame internally)
    walls: List[Wall]
    floor_z: Measure
    ceiling_z: Measure
    ceiling_height: Measure
    floor_area: Measure
    perimeter: Measure
    openings: List[Opening] = field(default_factory=list)
    kind: str = "room"
    label: str = ""
    quality: Dict = field(default_factory=dict)
    features: List[Dict] = field(default_factory=list)
