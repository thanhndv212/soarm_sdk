"""Abstract robot base class.

Every robot (simulation or real hardware) inherits from :class:`Robot` and
implements the handful of methods high-level application code needs. This
is the single abstraction boundary between *algorithm code* and *hardware
code* — see :mod:`soarm_sdk.robot.interfaces` for the structural Protocol
this class satisfies.

Design inspired by LeRobot's ``Robot`` + ``RobotConfig`` pattern: one YAML
config per platform, one Python class per back-end.
"""

from __future__ import annotations

import json
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Any, Dict, Optional, Union

import numpy as np

from .config import ConfigError, validate_robot_config
from .types import JointState, Pose

try:
    import yaml

    _HAS_YAML = True
except ImportError:
    _HAS_YAML = False

__all__ = ["Robot", "load_robot_config"]

_CONFIGS_DIR = Path(__file__).parent.parent / "configs"


def load_robot_config(
    name_or_path: Union[str, Path] = "soarm100",
) -> Dict[str, Any]:
    """Load a robot config by short name or file path.

    Parameters
    ----------
    name_or_path
        Either a short name (e.g. ``"soarm100"``) resolved to
        ``configs/<name>.yaml``, or an explicit path.
    """
    p = Path(name_or_path)
    if not p.suffix:
        p = _CONFIGS_DIR / f"{name_or_path}.yaml"
    if not p.exists():
        raise FileNotFoundError(f"Robot config not found: {p}")

    if p.suffix in (".yaml", ".yml"):
        if not _HAS_YAML:
            raise ImportError("PyYAML is required for YAML configs")
        with open(p) as f:
            return yaml.safe_load(f)
    else:
        with open(p) as f:
            return json.load(f)


class Robot(ABC):
    """Abstract robot base.

    Subclasses must implement the six ``@abstractmethod`` methods below.
    Everything else (``home_position``, ``joint_limits``, ``n_dof``, ``config``)
    is resolved from the YAML config at construction time.

    Parameters
    ----------
    config
        Robot configuration dict (typically from :func:`load_robot_config`).
    """

    def __init__(self, config: Dict[str, Any]) -> None:
        validate_robot_config(config)
        self._config = config
        self._n_dof: int = config["n_dof"]
        self._joint_names: list[str] = config["joint_names"]
        self._home: np.ndarray = np.asarray(config["home_position"], dtype=np.float64)
        self._limits_lo: np.ndarray = np.asarray(
            config["joint_limits_lower"], dtype=np.float64
        )
        self._limits_hi: np.ndarray = np.asarray(
            config["joint_limits_upper"], dtype=np.float64
        )

    # -- properties ----------------------------------------------------------

    @property
    def config(self) -> Dict[str, Any]:
        return self._config

    @property
    def n_dof(self) -> int:
        return self._n_dof

    @property
    def joint_names(self) -> list[str]:
        return list(self._joint_names)

    @property
    def home_position(self) -> np.ndarray:
        return self._home.copy()

    def get_joint_limits(self) -> tuple[np.ndarray, np.ndarray]:
        return self._limits_lo.copy(), self._limits_hi.copy()

    # -- lifecycle -------------------------------------------------------

    @abstractmethod
    def connect(self) -> None:
        """Open port / load model. Called once before first use."""

    @abstractmethod
    def disconnect(self) -> None:
        """Release resources (close serial, free memory)."""

    # -- joint access ------------------------------------------------------

    @abstractmethod
    def get_joint_positions(self) -> np.ndarray:
        """Read current joint angles in radians. Shape ``(n_dof,)``."""

    @abstractmethod
    def set_joint_positions(
        self,
        positions: np.ndarray,
        dq: Optional[np.ndarray] = None,
    ) -> None:
        """Command target joint positions.

        Parameters
        ----------
        positions
            Target joint angles (radians), shape ``(n_dof,)``.
        dq
            Optional velocity hint (rad/s) for hardware feedforward.
        """

    # -- kinematics ----------------------------------------------------------

    @abstractmethod
    def get_ee_pose(self) -> Pose:
        """Forward-kinematics for the end-effector link/site."""

    @abstractmethod
    def get_joint_state(self) -> JointState:
        """Full joint state snapshot (positions + optional vel/effort)."""

    # -- gripper -------------------------------------------------------------
    #
    # Opening and closing the jaw is a robot capability, not an application's
    # business: m5teleop and soarm_tamp each carried their own
    # GRIPPER_OPEN/CLOSED constants and their own "which joint is the jaw"
    # assumption. Driven here by an optional ``gripper`` block in the config:
    #
    #   gripper:
    #     joint_index: 5      # defaults to the last joint
    #     open_rad: 1.4
    #     closed_rad: 0.0

    @property
    def has_gripper(self) -> bool:
        """Whether this robot's config declares a gripper."""
        return "gripper" in self._config

    @property
    def gripper_index(self) -> int:
        """Index of the jaw joint in the joint vector."""
        self._assert_gripper()
        return int(self._config["gripper"].get("joint_index", self._n_dof - 1))

    def set_gripper(self, open: bool) -> None:
        """Open or close the jaw, leaving every other joint where it is."""
        self._assert_gripper()
        g = self._config["gripper"]
        positions = self.get_joint_positions()
        positions[self.gripper_index] = float(
            g["open_rad"] if open else g["closed_rad"]
        )
        self.set_joint_positions(positions)

    @property
    def gripper_is_open(self) -> bool:
        """Whether the jaw is nearer its open pose than its closed one."""
        self._assert_gripper()
        g = self._config["gripper"]
        pos = float(self.get_joint_positions()[self.gripper_index])
        return abs(pos - float(g["open_rad"])) < abs(pos - float(g["closed_rad"]))

    def toggle_gripper(self) -> None:
        """Close the jaw if it is open, open it if it is closed."""
        self.set_gripper(not self.gripper_is_open)

    def _assert_gripper(self) -> None:
        if "gripper" not in self._config:
            raise ConfigError(
                f"{type(self).__name__}'s config declares no 'gripper' block; "
                "add gripper.open_rad/closed_rad (and optionally joint_index) "
                "to use the gripper API"
            )

    # -- optional overrides --------------------------------------------------

    def go_home(self) -> None:
        """Move to the home configuration."""
        self.set_joint_positions(self._home)

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *exc):
        self.disconnect()

    def __repr__(self) -> str:
        return f"{type(self).__name__}(n_dof={self._n_dof})"
