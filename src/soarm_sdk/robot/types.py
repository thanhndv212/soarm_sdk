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


@dataclass
class ServoHealth:
    """Per-servo telemetry from the read-only SRAM block (addresses 56-70).

    Distinct from :class:`JointState`, which carries the control-relevant
    quantities in joint-frame SI units. This is servo-frame diagnostic data:
    one entry per servo, ordered like the interface's joint IDs.

    Attributes
    ----------
    loads_percent : np.ndarray
        Signed PWM duty, -100.0..100.0. The cheapest torque proxy the servo
        offers.
    currents_mA : np.ndarray
        Signed motor current in mA.
    voltages_V : np.ndarray
        Bus voltage at the servo. Sags under load.
    temperatures_C : np.ndarray
        Case temperature in degrees Celsius.
    status_flags : np.ndarray
        Raw STATUS register byte per servo (overload/overheat/voltage latches).
    moving : np.ndarray
        Boolean per servo, the servo's own "still slewing" flag.
    timestamp : float
        Monotonic timestamp of the read these values came from.
    """

    loads_percent: np.ndarray  # (n_joints,)
    currents_mA: np.ndarray  # (n_joints,)
    voltages_V: np.ndarray  # (n_joints,)
    temperatures_C: np.ndarray  # (n_joints,)
    status_flags: np.ndarray  # (n_joints,) uint8
    moving: np.ndarray  # (n_joints,) bool
    timestamp: float = 0.0
