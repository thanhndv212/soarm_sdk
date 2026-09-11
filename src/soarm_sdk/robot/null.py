"""An in-memory :class:`~soarm_sdk.robot.base.Robot` with no hardware behind it.

Every consumer of this SDK across the workspace has independently grown a
``dry_run: bool`` flag that special-cases "pretend there's an arm" —
m5teleop's ``ArmInterface``, soarm_tamp's ``execute.py --dry-run``, and
mjlab's sim/real switch all reimplement the same idea slightly differently.
``NullRobot`` gives them one real implementation to depend on instead: it
satisfies :class:`~soarm_sdk.robot.interfaces.RobotInterface` completely,
tracks commanded positions in memory, and never touches a serial port.

Useful for offline testing, CI, and any code path that wants "a robot" but
doesn't have (or want) one connected — swap it for :class:`ServoRobot
<soarm_sdk.robot.servo.ServoRobot>` at the call site and nothing else
changes.
"""

from __future__ import annotations

import time
from typing import Any, Callable, Dict, Optional

import numpy as np

from .base import Robot, load_robot_config
from .types import JointState, Pose

__all__ = ["NullRobot"]


class NullRobot(Robot):
    """No-hardware stand-in robot: commands are recorded, not sent anywhere.

    Parameters
    ----------
    config
        Robot config dict. Defaults to ``soarm100.yaml``.
    fk_fn
        Optional FK callable ``(q) -> (pos, quat)`` for :meth:`get_ee_pose`.
        Without one, :meth:`get_ee_pose` returns :meth:`Pose.identity`.
    """

    def __init__(
        self,
        config: Optional[Dict[str, Any]] = None,
        *,
        fk_fn: Optional[Callable] = None,
    ) -> None:
        cfg = config or load_robot_config("soarm100")
        super().__init__(cfg)
        self._fk_fn = fk_fn
        self._connected = False
        self._positions = self._home.copy()

    # -- lifecycle -------------------------------------------------------

    def connect(self) -> None:
        self._connected = True

    def disconnect(self) -> None:
        self._connected = False

    # -- joint access ------------------------------------------------------

    def get_joint_positions(self) -> np.ndarray:
        return self._positions.copy()

    def set_joint_positions(
        self,
        positions: np.ndarray,
        dq: Optional[np.ndarray] = None,
    ) -> None:
        self._positions = np.asarray(positions, dtype=np.float64).copy()

    # -- kinematics ----------------------------------------------------------

    def get_ee_pose(self) -> Pose:
        if self._fk_fn is None:
            return Pose.identity()
        pos, quat = self._fk_fn(self._positions)
        return Pose(position=np.asarray(pos), orientation=np.asarray(quat))

    def get_joint_state(self) -> JointState:
        return JointState(positions=self._positions.copy(), timestamp=time.time())

    # -- introspection (not part of RobotInterface) ---------------------------

    @property
    def connected(self) -> bool:
        return self._connected
