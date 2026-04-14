"""
Minimal arena zone + cell logic (no publishers, no ROS input).

Deliverables:
- get_currentcell(pose) -> str  (E1..E22, O1..O55, or "Out of Bounds")
- get_currentzone(pose) -> str  ("Starting Zone", "Construction Zone", "Excavation Zone", "Obstacle Zone", or "Out of Bounds")
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Dict, Tuple

import yaml


def _xy_from_pose(pose: Any) -> Tuple[float, float]:
    """
    Accepts:
    - (x, y) tuple/list
    - objects with .x/.y
    - objects with .position.x/.position.y
    """
    if isinstance(pose, (tuple, list)) and len(pose) >= 2:
        return float(pose[0]), float(pose[1])

    if hasattr(pose, "x") and hasattr(pose, "y"):
        return float(pose.x), float(pose.y)

    if hasattr(pose, "position") and hasattr(pose.position, "x") and hasattr(pose.position, "y"):
        return float(pose.position.x), float(pose.position.y)

    raise TypeError("pose must provide x,y (tuple/list or attributes)")


def _load_yaml(path: Path | str) -> Dict[str, Any]:
    p = Path(path)
    with p.open(encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError("arena_zones.yaml must parse to a dict")
    return data


def _point_in_rect_closed(x: float, y: float, rect: Any) -> bool:
    x0, x1, y0, y1 = (float(rect[0]), float(rect[1]), float(rect[2]), float(rect[3]))
    return x0 <= x <= x1 and y0 <= y <= y1


def _in_arena_bounds(x: float, y: float) -> bool:
    # As specified in human_notes.md:
    # - coordinates are in arena frame
    # - x == 6.88 or y == 11 are out of bounds (max edges excluded)
    return (0.0 <= x < 6.88) and (0.0 <= y < 11.0)


def get_currentcell(pose: Any) -> str:
    x, y = _xy_from_pose(pose)
    if not _in_arena_bounds(x, y):
        return "Out of Bounds"

    # 1m grid, origin at bottom-left.
    # Tie-break "top-right" naturally falls out of using max-edge-excluded bounds + floor indexing.
    col = int(math.floor(x / 1.0))  # 0..6 (7 cols, last is partial due to 6.88m)
    row = int(math.floor(y / 1.0))  # 0..10 (11 rows)

    if col < 2:
        # Excavation side: 2 columns, 11 rows => E1..E22, row-major (x fastest)
        n = row * 2 + col + 1
        return f"E{n}"

    # Obstacle side: 5 columns (col 2..6), 11 rows => O1..O55, row-major (x fastest)
    ocol = col - 2
    n = row * 5 + ocol + 1
    return f"O{n}"


def get_currentzone(pose: Any, *, arena_zones_yaml: Path | str = None) -> str:
    """
    Uses arena_zones.yaml rectangles to return the most specific zone.
    Default YAML path: ros2_ws/src/luna_nav/config/arena_zones.yaml (relative to repo root).
    """
    x, y = _xy_from_pose(pose)
    if not _in_arena_bounds(x, y):
        return "Out of Bounds"

    if arena_zones_yaml is None:
        arena_zones_yaml = Path(__file__).resolve().parents[1] / "config" / "arena_zones.yaml"

    data = _load_yaml(arena_zones_yaml)
    zones = data.get("zones") or {}

    # Most-specific priority: Starting/Construction beats Excavation/Obstacle.
    sz = zones.get("starting_zone", {})
    if "rectangle" in sz and _point_in_rect_closed(x, y, sz["rectangle"]):
        return "Starting Zone"

    cz = zones.get("construction_zone", {})
    if "rectangle" in cz and _point_in_rect_closed(x, y, cz["rectangle"]):
        return "Construction Zone"

    ez = zones.get("excavation_zone", {})
    if "rectangle" in ez and _point_in_rect_closed(x, y, ez["rectangle"]):
        return "Excavation Zone"

    oz = zones.get("obstacle_zone", {})
    if "rectangle_main" in oz:
        inside_main = _point_in_rect_closed(x, y, oz["rectangle_main"])
        inside_exclude = "rectangle_exclude" in oz and _point_in_rect_closed(x, y, oz["rectangle_exclude"])
        if inside_main and not inside_exclude:
            return "Obstacle Zone"

    return "Out of Bounds"
