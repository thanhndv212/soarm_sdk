"""LeRobot-backed robot: the same arm, driven through lerobot's ``SOFollower``.

There are two independent ways to talk to this servo bus in practice — this
SDK's own Feetech protocol stack (:class:`~soarm_sdk.robot.servo.ServoRobot`)
and lerobot's ``SOFollower``, which owns its own calibration in servo EEPROM
and is what the imitation-learning tooling already speaks. m5teleop grew a
hand-rolled wrapper around the latter (``ArmInterface``) that did not
implement :class:`~soarm_sdk.robot.interfaces.RobotInterface`, so teleop code
could not be pointed at a planner's robot, a simulation, or
:class:`~soarm_sdk.robot.null.NullRobot` without rewriting the call site.

This backend closes that gap: whichever transport you pick, application code
above sees one interface.

Frames, and a caveat worth reading
----------------------------------
``SOFollower`` reports and accepts **normalized degrees**, whose zero is the
midpoint of each joint's calibrated travel — *not* the URDF's kinematic zero.
This class converts degrees <-> radians and nothing more, which is the same
assumption m5teleop's ``ArmInterface`` made. That is fine for teleoperation,
where the operator closes the loop visually, but it is **not** the URDF frame
a motion planner speaks. For that mapping see
:mod:`soarm_sdk.calibration.frame`, and pass its
:class:`~soarm_sdk.calibration.frame.RobotCalibration` through
``ServoRobot`` instead of assuming the two frames agree here.

lerobot is an optional dependency (``pip install soarm-sdk[lerobot]``) and is
imported lazily inside :meth:`connect`, so importing this module — or running
the rest of the SDK — never requires it.
"""

from __future__ import annotations

import time
from typing import Any, Callable, Dict, List, Optional

import numpy as np

from .base import Robot, load_robot_config
from .types import JointState, Pose

__all__ = ["LeRobotRobot", "DEFAULT_MOTOR_NAMES"]

#: lerobot's motor names for the SO-100/SO-101 follower, in servo-ID order.
#: Identical to this SDK's own ``joint_names`` and to the URDF's — they all
#: name the same six joints in the same order.
DEFAULT_MOTOR_NAMES: List[str] = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


class LeRobotRobot(Robot):
    """SO-ARM100/SO-101 driven through lerobot's ``SOFollower``.

    Parameters
    ----------
    port
        Serial device path (e.g. ``/dev/tty.usbserial-XXXX``).
    config
        Robot config dict. Defaults to ``so101.yaml``.
    motor_names
        lerobot motor names in joint order. Defaults to
        :data:`DEFAULT_MOTOR_NAMES`.
    max_relative_target
        Per-command step bound handed to lerobot, in **degrees**. lerobot
        enforces it on its side; ``None`` disables it.
    fk_fn
        Optional FK callable ``(q) -> (pos, quat)`` for :meth:`get_ee_pose`.
    follower_factory
        Callable ``(port, max_relative_target, motor_names) -> follower``
        used instead of constructing a real ``SOFollower``. Exists so this
        backend can be tested, and driven against a stand-in, without
        lerobot installed.
    """

    def __init__(
        self,
        port: str,
        config: Optional[Dict[str, Any]] = None,
        *,
        motor_names: Optional[List[str]] = None,
        max_relative_target: Optional[float] = 5.0,
        fk_fn: Optional[Callable] = None,
        follower_factory: Optional[Callable[..., Any]] = None,
    ) -> None:
        cfg = config or load_robot_config("soarm100")
        super().__init__(cfg)

        self._port = port
        self._motor_names = list(motor_names or DEFAULT_MOTOR_NAMES)
        if len(self._motor_names) != self._n_dof:
            raise ValueError(
                f"motor_names has {len(self._motor_names)} entries, "
                f"expected n_dof={self._n_dof}"
            )
        self._max_relative_target = max_relative_target
        self._fk_fn = fk_fn
        self._follower_factory = follower_factory
        self._follower: Optional[Any] = None

    # -- lifecycle -------------------------------------------------------

    def connect(self) -> None:
        if self._follower is not None:
            return
        if self._follower_factory is not None:
            self._follower = self._follower_factory(
                port=self._port,
                max_relative_target=self._max_relative_target,
                motor_names=self._motor_names,
            )
        else:
            self._follower = self._make_so_follower()
        self._follower.connect(calibrate=False)

    def _make_so_follower(self) -> Any:
        """Build a real lerobot ``SOFollower``; imported here, not at module
        scope, so lerobot stays an optional dependency."""
        try:
            from lerobot.robots.so_follower.config_so_follower import (
                SOFollowerRobotConfig,
            )
            from lerobot.robots.so_follower.so_follower import SOFollower
        except ImportError as exc:  # pragma: no cover - depends on env
            raise ImportError(
                "lerobot is required for LeRobotRobot. Install with: "
                "pip install soarm-sdk[lerobot]  — or use ServoRobot, which "
                "drives the same bus through this SDK's own protocol stack."
            ) from exc

        return SOFollower(
            SOFollowerRobotConfig(
                port=self._port,
                use_degrees=True,
                max_relative_target=self._max_relative_target,
            )
        )

    def disconnect(self) -> None:
        if self._follower is None:
            return
        try:
            self._follower.disconnect()
        finally:
            self._follower = None

    # -- joint access ------------------------------------------------------

    def get_joint_positions(self) -> np.ndarray:
        obs = self._assert_connected().get_observation()
        return np.array(
            [np.radians(float(obs.get(f"{n}.pos", 0.0))) for n in self._motor_names],
            dtype=np.float64,
        )

    def set_joint_positions(
        self,
        positions: np.ndarray,
        dq: Optional[np.ndarray] = None,
    ) -> None:
        positions = np.asarray(positions, dtype=np.float64)
        if positions.shape != (self._n_dof,):
            raise ValueError(
                f"expected {self._n_dof} joint positions, got {positions.shape}"
            )
        action = {
            f"{name}.pos": float(np.degrees(value))
            for name, value in zip(self._motor_names, positions)
        }
        self._assert_connected().send_action(action)

    def get_joint_state(self) -> JointState:
        return JointState(
            positions=self.get_joint_positions(),
            timestamp=time.time(),
        )

    # -- kinematics ----------------------------------------------------------

    def get_ee_pose(self) -> Pose:
        if self._fk_fn is None:
            return Pose.identity()
        pos, quat = self._fk_fn(self.get_joint_positions())
        return Pose(position=np.asarray(pos), orientation=np.asarray(quat))

    # -- lerobot-specific accessors (not part of the Robot ABC) --------------

    @property
    def motor_names(self) -> List[str]:
        """lerobot motor names, in joint order."""
        return list(self._motor_names)

    @property
    def port(self) -> str:
        return self._port

    @property
    def connected(self) -> bool:
        return self._follower is not None

    @property
    def follower(self) -> Any:
        """The underlying lerobot follower (after :meth:`connect`)."""
        return self._follower

    def get_joint_degrees(self) -> Dict[str, float]:
        """Present positions as lerobot's ``{"<motor>.pos": degrees}`` dict.

        The native shape of lerobot's own API, kept for call sites (IK
        solvers, dataset recorders) that already speak it.
        """
        obs = self._assert_connected().get_observation()
        return {k: float(v) for k, v in obs.items() if k.endswith(".pos")}

    def send_joint_degrees(self, degrees: Dict[str, float]) -> None:
        """Command positions from a lerobot ``{"<motor>.pos": degrees}`` dict.

        Passed through untouched, so a caller may send a subset of joints —
        unlike :meth:`set_joint_positions`, which commands the full vector.
        """
        self._assert_connected().send_action(dict(degrees))

    # -- internal ------------------------------------------------------------

    def _assert_connected(self) -> Any:
        if self._follower is None:
            raise RuntimeError("Robot not connected — call robot.connect() first")
        return self._follower
