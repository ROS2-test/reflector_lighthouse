"""Shared reflective-star helpers."""

from __future__ import annotations

import math
from typing import List, Sequence, Tuple


def yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def quat_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def find_stars(
    ranges: Sequence[float],
    intensities: Sequence[float],
    angle_min: float,
    angle_inc: float,
    intensity_min: float,
    cluster_r: float = 0.07,
) -> List[Tuple[float, float]]:
    """Cluster bright LaserScan hits -> list of (dist_m, ang_rad)."""
    n = min(len(ranges), len(intensities))
    groups: List[List[Tuple[float, float]]] = []
    last_i = -100
    r2 = cluster_r * cluster_r

    for i in range(n):
        d = ranges[i]
        if not math.isfinite(d) or d <= 0.05:
            continue
        if intensities[i] < intensity_min:
            continue
        ang = angle_min + i * angle_inc
        if groups and 0 < (i - last_i) <= 5:
            pd, pa = groups[-1][-1]
            dan = abs(ang - pa)
            if (d * d + pd * pd - 2.0 * d * pd * math.cos(dan)) < r2:
                groups[-1].append((d, ang))
                last_i = i
                continue
        groups.append([(d, ang)])
        last_i = i

    return [
        (sum(p[0] for p in g) / len(g), sum(p[1] for p in g) / len(g))
        for g in groups
    ]


def stars_to_map(
    stars: Sequence[Tuple[float, float]],
    px: float,
    py: float,
    yaw: float,
    lidar_x: float = 0.0,
    lidar_y: float = 0.0,
    lidar_yaw: float = 0.0,
) -> List[Tuple[float, float]]:
    """Polar stars in laser frame -> (x,y) in map using robot pose."""
    c, s = math.cos(yaw), math.sin(yaw)
    lmx = px + lidar_x * c - lidar_y * s
    lmy = py + lidar_x * s + lidar_y * c
    out = []
    for dist, ang in stars:
        a = ang + lidar_yaw + yaw
        out.append((lmx + dist * math.cos(a), lmy + dist * math.sin(a)))
    return out


def build_pairs(markers: List[Tuple[float, float]], max_dist: float):
    """Build pairs all to all."""
    pairs = []
    for i, (x0, y0) in enumerate(markers):
        for j in range(i + 1, len(markers)):
            x1, y1 = markers[j]
            d = math.hypot(x1 - x0, y1 - y0)
            if 0.0 < d <= max_dist:
                pairs.append((d, x0, y0, x1, y1))
                pairs.append((d, x1, y1, x0, y0))
    pairs.sort(key=lambda p: p[0])
    return pairs


def read_pairs(markers: List[Tuple[float, float]], collections: list):
    """Read and return pairs from file."""
    pairs = []
    for pair in collections:
        pairs.append((
            pair['dist'], pair['x0'], pair['y0'], pair['x1'], pair['y1']
        ))
    pairs.sort(key=lambda p: p[0])
    return pairs
