"""Real-hardware robot backed by STS3215 servos.

Wraps :class:`~soarm_sdk.robot.hardware.ServoHardwareInterface` and
exposes the :class:`~soarm_sdk.robot.base.Robot` ABC, satisfying
:class:`~soarm_sdk.robot.interfaces.RobotInterface`.
"""

from __future__ import annotations

from typing import Any, Callable, Dict, Optional, TYPE_CHECKING

import numpy as np

from .hardware import ServoHardwareInterface

if TYPE_CHECKING:  # pragma: no cover
    from ..calibration.frame import RobotCalibration
from .base import Robot, load_robot_config
from .types import JointState, Pose

__all__ = ["ServoRobot"]


class ServoRobot(Robot):
    """SO-ARM100 (or compatible) servo arm on real RS-485 hardware.

    Parameters
    ----------
    port
        Serial device path (e.g. ``/dev/tty.usbserial-XXXX``).
    config
        Robot config dict. Defaults to ``soarm100.yaml``.
    fk_fn
        Optional FK callable ``(q) -> (pos, quat)`` for EE pose queries.
    """

    def __init__(
        self,
        port: str,
        config: Optional[Dict[str, Any]] = None,
        *,
        fk_fn: Optional[Callable] = None,
        calibration: Optional["RobotCalibration"] = None,
        max_step_rad: Optional[float] = None,
        enforce_limits: bool = True,
    ) -> None:
        cfg = config or load_robot_config("soarm100")
        super().__init__(cfg)

        self._port = port
        self._fk_fn = fk_fn
        self._calibration = calibration
        self._max_step_rad = max_step_rad
        self._enforce_limits = enforce_limits
        self._hw: Optional[ServoHardwareInterface] = None

    # -- lifecycle -------------------------------------------------------

    def connect(self) -> None:
        if self._hw is not None:
            return

        hw_cfg = self._config.get("hardware", {})
        self._hw = ServoHardwareInterface(
            port=self._port,
            baud=hw_cfg.get("baud", 1_000_000),
            joint_ids=hw_cfg.get("servo_ids", [1, 2, 3, 4, 5, 6]),
            zero_offsets=hw_cfg.get("zero_offsets"),
            direction_signs=hw_cfg.get("direction_signs"),
            default_speed=hw_cfg.get("default_speed", 300),
            default_acc=hw_cfg.get("default_acc", 50),
            state_freq=hw_cfg.get("state_freq", 100),
            fk_fn=self._fk_fn,
            joint_limits=self.effective_joint_limits() if self._enforce_limits else None,
            max_step_rad=self._max_step_rad,
            calibration=self._calibration,
        )
        self._hw.start()

    def effective_joint_limits(self) -> tuple[np.ndarray, np.ndarray]:
        """The limits actually enforced on every write.

        The config's declared limits, **intersected with the arm's measured
        travel** when a calibration is supplied. Hardware wins on the tight
        side: a config is a model's opinion about the joint, while the
        measured range is where the mechanism physically stops, and a
        command must never be driven past the latter.

        Without a calibration this is just the config's limits — the only
        bound available, and the reason a calibration is worth having before
        streaming anything open-loop.
        """
        lo, hi = self.get_joint_limits()
        if self._calibration is None:
            return lo, hi

        cal_lo, cal_hi = self._calibration.reachable_limits()
        if len(cal_lo) != len(lo):
            raise ValueError(
                f"calibration covers {len(cal_lo)} joints, config declares "
                f"{len(lo)} — they must describe the same arm"
            )
        return (
            np.maximum(lo, np.asarray(cal_lo, dtype=np.float64)),
            np.minimum(hi, np.asarray(cal_hi, dtype=np.float64)),
        )

    def disconnect(self) -> None:
        if self._hw is not None:
            self._hw.stop()
            self._hw = None

    # -- joint access ------------------------------------------------------

    def get_joint_positions(self) -> np.ndarray:
        self._assert_hw()
        return self._hw.get_robot_joint_positions()

    def set_joint_positions(
        self,
        positions: np.ndarray,
        dq: Optional[np.ndarray] = None,
    ) -> None:
        self._assert_hw()
        self._hw.set_robot_joint_positions(positions, dq=dq)

    # -- kinematics ----------------------------------------------------------

    def get_ee_pose(self) -> Pose:
        self._assert_hw()
        pos, quat = self._hw.get_body_pose("ee")
        return Pose(position=pos, orientation=quat)

    def get_joint_state(self) -> JointState:
        self._assert_hw()
        return self._hw.get_robot_joint_state()

    # -- servo-specific accessors (not part of Robot ABC) --------------------

    @property
    def hw(self) -> ServoHardwareInterface:
        """Raw :class:`ServoHardwareInterface` (after ``connect()``)."""
        return self._hw

    def set_torque(self, enabled: bool) -> None:
        """Enable or disable torque on every joint. Blocks until applied.

        Servo-specific, so it is not on :class:`~soarm_sdk.robot.base.Robot`:
        a simulated arm has no torque to switch. Disabling makes the joints
        back-driveable, which is how a calibration's direction signs get
        checked by hand and how a servo that has tripped its overload
        protection against a stop is released.
        """
        self._assert_hw()
        self._hw.set_torque(enabled)

    def disable_torque(self) -> None:
        """Go limp. Support the arm first — gravity-loaded joints will drop."""
        self.set_torque(False)

    def enable_torque(self) -> None:
        """Hold station at the current measured pose."""
        self.set_torque(True)

    def state_age(self) -> float:
        """Seconds since last successful servo read."""
        if self._hw is None:
            return float("inf")
        return self._hw.state_age()

    # -- internal ------------------------------------------------------------

    def _assert_hw(self) -> None:
        if self._hw is None:
            raise RuntimeError("Robot not connected — call robot.connect() first")
