"""Trajectory resampling for streaming waypoints to a :class:`~soarm_sdk.robot.base.Robot`.

A planner (or a recorded demonstration) samples a path at whatever
resolution suits it — a short edge can come back as two waypoints well
over the per-step bound a controller is safe commanding. Streaming that
verbatim risks a large single-tick jump; :func:`resample` interpolates so
no joint moves more than ``max_step`` between consecutive commands.

This generalizes a pattern every downstream repo has needed and
reimplemented independently around a planned or recorded path (soarm_tamp's
waypoint-manifest executor is one instance) — the safety-relevant part
belongs in the SDK once, not once per caller.
"""

from __future__ import annotations

import math
from typing import Sequence

import numpy as np

__all__ = ["resample"]


def resample(waypoints: Sequence[np.ndarray], max_step: float) -> list[np.ndarray]:
    """Linearly interpolate *waypoints* so no joint moves more than *max_step* per step.

    Parameters
    ----------
    waypoints
        A sequence of joint-position arrays, all the same shape.
    max_step
        Maximum allowed per-joint change (radians) between consecutive
        output waypoints. Must be positive.

    Returns
    -------
    list[np.ndarray]
        The resampled path. Always starts with ``waypoints[0]`` unchanged;
        every consecutive pair in the output differs by at most
        *max_step* in each joint. Input waypoints are preserved as
        interpolation endpoints (never skipped), so downstream phase
        boundaries keyed to a waypoint index still land correctly.
    """
    if max_step <= 0:
        raise ValueError(f"max_step must be positive, got {max_step}")
    if len(waypoints) < 2:
        return [np.asarray(w, dtype=np.float64).copy() for w in waypoints]

    out: list[np.ndarray] = [np.asarray(waypoints[0], dtype=np.float64).copy()]
    for a, b in zip(waypoints, waypoints[1:]):
        a = np.asarray(a, dtype=np.float64)
        b = np.asarray(b, dtype=np.float64)
        biggest = float(np.max(np.abs(b - a)))
        n = max(1, math.ceil(biggest / max_step))
        for i in range(1, n + 1):
            t = i / n
            out.append(a + (b - a) * t)
    return out
