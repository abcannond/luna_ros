"""
Pure geometry for arena square grid and zone labels (no ROS).
Loads config from arena_zones.yaml (shared with zone_publisher).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional, Sequence, Set, Tuple

import yaml

Rect = Tuple[float, float, float, float]  # x_min, x_max, y_min, y_max


def _closed_interval_overlap_1d(a0: float, a1: float, b0: float, b1: float) -> float:
    """Overlap length of two closed intervals; zero if disjoint."""
    lo = max(a0, b0)
    hi = min(a1, b1)
    return max(0.0, hi - lo)


def rect_intersection_area_closed(r1: Rect, r2: Rect) -> float:
    """Area of intersection of two axis-aligned closed rectangles."""
    x0, x1, y0, y1 = r1
    u0, u1, v0, v1 = r2
    dx = _closed_interval_overlap_1d(x0, x1, u0, u1)
    dy = _closed_interval_overlap_1d(y0, y1, v0, v1)
    return dx * dy


def point_in_rect_closed(x: float, y: float, rect: Rect) -> bool:
    x0, x1, y0, y1 = rect
    return x0 <= x <= x1 and y0 <= y <= y1


def load_arena_config(path: Path | str) -> Dict[str, Any]:
    path = Path(path)
    with path.open(encoding="utf-8") as f:
        return yaml.safe_load(f)


@dataclass(frozen=True)
class ArenaGrid:
    """Uniform square grid over arena; row-major G index k = ri * ncols + ci + 1."""

    arena: Rect
    cell_size_m: float
    ncols: int
    nrows: int
    prefer_excavation_e_label: bool
    excavation_rect: Rect
    e_index_by_cell: Dict[Tuple[int, int], int]

    @staticmethod
    def from_config(data: Dict[str, Any]) -> ArenaGrid:
        ax = data["arena"]
        arena: Rect = (ax["x_min"], ax["x_max"], ax["y_min"], ax["y_max"])
        grid = data.get("grid") or {}
        s = float(grid.get("cell_size_m", 1.0))
        prefer_e = bool(grid.get("prefer_excavation_e_label", True))

        xspan = arena[1] - arena[0]
        yspan = arena[3] - arena[2]
        ncols = max(1, math.ceil(xspan / s))
        nrows = max(1, math.ceil(yspan / s))

        ex = data["zones"]["excavation_zone"]["rectangle"]
        excavation_rect: Rect = tuple(float(v) for v in ex)  # type: ignore[assignment]

        e_map: Dict[Tuple[int, int], int] = {}
        m = 0
        for ri in range(nrows):
            for ci in range(ncols):
                clip = cell_rect_clipped_to_arena(ci, ri, arena, s)
                if clip is None:
                    continue
                if rect_intersection_area_closed(clip, excavation_rect) > 0.0:
                    m += 1
                    e_map[(ci, ri)] = m

        return ArenaGrid(
            arena=arena,
            cell_size_m=s,
            ncols=ncols,
            nrows=nrows,
            prefer_excavation_e_label=prefer_e,
            excavation_rect=excavation_rect,
            e_index_by_cell=e_map,
        )


def cell_rect_clipped_to_arena(ci: int, ri: int, arena: Rect, s: float) -> Optional[Rect]:
    """Closed intersection of cell with arena; None if empty."""
    ax0, ax1, ay0, ay1 = arena
    cx0 = ax0 + ci * s
    cx1 = ax0 + (ci + 1) * s
    cy0 = ay0 + ri * s
    cy1 = ay0 + (ri + 1) * s
    x0 = max(ax0, cx0)
    x1 = min(ax1, cx1)
    y0 = max(ay0, cy0)
    y1 = min(ay1, cy1)
    if x1 < x0 or y1 < y0:
        return None
    return (x0, x1, y0, y1)


def cell_center(ci: int, ri: int, arena: Rect, s: float) -> Optional[Tuple[float, float]]:
    clip = cell_rect_clipped_to_arena(ci, ri, arena, s)
    if clip is None:
        return None
    x0, x1, y0, y1 = clip
    return ((x0 + x1) * 0.5, (y0 + y1) * 0.5)


def pose_cell_indices(x: float, y: float, arena: Rect, s: float, ncols: int, nrows: int) -> Optional[Tuple[int, int]]:
    ax0, ax1, ay0, ay1 = arena
    if not (ax0 <= x <= ax1 and ay0 <= y <= ay1):
        return None
    ci = int(math.floor((x - ax0) / s))
    ri = int(math.floor((y - ay0) / s))
    ci = max(0, min(ncols - 1, ci))
    ri = max(0, min(nrows - 1, ri))
    return ci, ri


def global_g_index(ci: int, ri: int, ncols: int) -> int:
    return ri * ncols + ci + 1


def zone_codes_at_point(x: float, y: float, data: Dict[str, Any], origin_tol: float) -> Set[str]:
    """All zone short codes containing (x, y); matches coarse overlap semantics (closed rects)."""
    codes: Set[str] = set()
    if abs(x) <= origin_tol and abs(y) <= origin_tol:
        codes.add("s")
    zones = data["zones"]

    s_rect = tuple(zones["starting_zone"]["rectangle"])
    if point_in_rect_closed(x, y, s_rect):
        codes.add("s")
    e_rect = tuple(zones["excavation_zone"]["rectangle"])
    if point_in_rect_closed(x, y, e_rect):
        codes.add("e")
    oz = zones["obstacle_zone"]
    main = tuple(oz["rectangle_main"])
    exc = tuple(oz["rectangle_exclude"])
    if point_in_rect_closed(x, y, main) and not point_in_rect_closed(x, y, exc):
        codes.add("o")
    c_rect = tuple(zones["construction_zone"]["rectangle"])
    if point_in_rect_closed(x, y, c_rect):
        codes.add("c")

    return codes


def format_border(codes: Set[str]) -> str:
    return "border:" + "&".join(sorted(codes))


def label_for_pose(
    x: float,
    y: float,
    data: Dict[str, Any],
    grid: ArenaGrid,
    *,
    outside_arena_label: str = "outside_arena",
    outside_named_label: str = "outside",
) -> str:
    origin_tol = float(data.get("origin_tolerance_m", 0.01))
    arena = grid.arena

    if not (arena[0] <= x <= arena[1] and arena[2] <= y <= arena[3]):
        return outside_arena_label

    pi = pose_cell_indices(x, y, arena, grid.cell_size_m, grid.ncols, grid.nrows)
    if pi is None:
        return outside_arena_label
    ci, ri = pi
    center = cell_center(ci, ri, arena, grid.cell_size_m)
    if center is None:
        return outside_arena_label
    cx, cy = center

    codes = zone_codes_at_point(cx, cy, data, origin_tol)
    if len(codes) >= 2:
        return format_border(codes)
    if len(codes) == 0:
        return outside_named_label

    gk = global_g_index(ci, ri, grid.ncols)
    em = grid.e_index_by_cell.get((ci, ri))
    if grid.prefer_excavation_e_label and em is not None:
        return f"E{em}"
    return f"G{gk}"


def get_coarse_zone_name(x: float, y: float, data: Dict[str, Any]) -> str:
    """First-match coarse string for zone_publisher (same priority as original)."""
    origin_tol = float(data.get("origin_tolerance_m", 0.01))
    if abs(x) <= origin_tol and abs(y) <= origin_tol:
        return data["zones"]["starting_zone"]["coarse_name"]

    priority: Sequence[str] = data["zone_priority"]
    zones = data["zones"]

    for zname in priority:
        z = zones[zname]
        if zname == "obstacle_zone":
            main = tuple(z["rectangle_main"])
            exc = tuple(z["rectangle_exclude"])
            if point_in_rect_closed(x, y, main) and not point_in_rect_closed(x, y, exc):
                return z["coarse_name"]
        else:
            rect = tuple(z["rectangle"])
            if point_in_rect_closed(x, y, rect):
                return z["coarse_name"]

    return "outside bounds"
