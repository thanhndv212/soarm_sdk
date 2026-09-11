"""Shared data types for the soarm_sdk robot interface layer."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class Pose:
    """6-DoF pose (position + quaternion orientation).

    Attributes
    ----------
    position : np.ndarray
        Cartesian position ``[x, y, z]``.
    orientation : np.ndarray
        Unit quaternion ``[w, x, y, z]``.
    """

    position: np.ndarray  # (3,)
    orientation: np.ndarray  # (4,) wxyz

    @staticmethod
    def identity() -> "Pose":
        return Pose(
            position=np.zeros(3),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
        )

    def as_array(self) -> np.ndarray:
        """Return ``[x, y, z, qw, qx, qy, qz]``."""
        return np.concatenate([self.position, self.orientation])


@dataclass
class JointState:
    """Snapshot of joint-level state.

    Attributes
    ----------
    positions : np.ndarray
        Joint angles in radians, shape ``(n_dof,)``.
    velocities : np.ndarray | None
        Joint velocities in rad/s.
    efforts : np.ndarray | None
        Joint torques or currents.
    timestamp : float
        Monotonic timestamp in seconds.
    """

    positions: np.ndarray  # (n_dof,)
    velocities: Optional[np.ndarray] = None
    efforts: Optional[np.ndarray] = None
    timestamp: float = 0.0
