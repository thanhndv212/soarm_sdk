"""Robot interface contract for soarm_sdk.

Defines the minimal Protocol a robot arm (real hardware or a simulation
equivalent) must satisfy for high-level application code to treat it
generically. Structural typing via ``typing.Protocol`` — no inheritance
required, so a simulation backend elsewhere in the workspace can satisfy
this contract without depending on soarm_sdk at all.
"""

from __future__ import annotations

from typing import Optional, Protocol, runtime_checkable

import numpy as np

from .types import JointState, Pose

__all__ = ["RobotInterface", "Pose", "JointState"]


@runtime_checkable
class RobotInterface(Protocol):
    """Minimal contract every robot (sim or real) must satisfy."""

    @property
    def n_dof(self) -> int:
        """Number of actuated joints."""
        ...

    def connect(self) -> None:
        """Establish connection (open port / load model)."""
        ...

    def disconnect(self) -> None:
        """Release resources."""
        ...

    def get_joint_positions(self) -> np.ndarray:
        """Current joint angles in radians, shape ``(n_dof,)``."""
        ...

    def set_joint_positions(
        self,
        positions: np.ndarray,
        dq: Optional[np.ndarray] = None,
    ) -> None:
        """Command joint positions.

        Parameters
        ----------
        positions : np.ndarray
            Target joint angles in radians.
        dq : np.ndarray, optional
            Velocity hint (rad/s) for feedforward on hardware.
        """
        ...

    def get_joint_state(self) -> JointState:
        """Full joint state snapshot (positions + optional vel/effort)."""
        ...

    def get_ee_pose(self) -> Pose:
        """Forward-kinematics for the end-effector."""
        ...

    def get_joint_limits(self) -> tuple[np.ndarray, np.ndarray]:
        """Return ``(lower, upper)`` joint limits in radians."""
        ...

    @property
    def home_position(self) -> np.ndarray:
        """Default home joint configuration."""
        ...
