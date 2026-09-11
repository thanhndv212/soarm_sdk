"""Validation for the robot config dict consumed by :class:`~soarm_sdk.robot.base.Robot`.

The config is still a plain ``dict`` (loaded from YAML/JSON by
:func:`~soarm_sdk.robot.base.load_robot_config`) rather than a dataclass —
:class:`ServoRobot` and downstream repos read arbitrary nested keys
(``hardware.servo_ids``, ``hardware.zero_offsets``, ...) that a strict
schema would need to special-case anyway. What this module fixes is the
failure mode: an incomplete config used to surface as a bare ``KeyError``
from deep inside ``Robot.__init__``, or a wrong-length array producing a
confusing shape mismatch three calls later in ``ServoHardwareInterface``.
:func:`validate_robot_config` checks all of that up front and raises one
``ConfigError`` naming every problem at once.
"""

from __future__ import annotations

from typing import Any, Dict, List

__all__ = ["ConfigError", "REQUIRED_KEYS", "validate_robot_config"]

REQUIRED_KEYS: tuple[str, ...] = (
    "n_dof",
    "joint_names",
    "home_position",
    "joint_limits_lower",
    "joint_limits_upper",
)


class ConfigError(ValueError):
    """A robot config dict is missing or has malformed required fields."""


def validate_robot_config(config: Dict[str, Any]) -> None:
    """Raise :class:`ConfigError` naming every problem, or return silently.

    Checked eagerly in :meth:`Robot.__init__` so a bad config fails at
    construction with one readable message instead of a ``KeyError`` (or a
    silent shape mismatch) surfacing later from whichever accessor happens
    to touch the missing/malformed field first.
    """
    problems: List[str] = []

    missing = [k for k in REQUIRED_KEYS if k not in config]
    if missing:
        problems.append(f"missing required key(s): {', '.join(missing)}")

    n_dof = config.get("n_dof")
    if "n_dof" not in missing:
        if not isinstance(n_dof, int) or n_dof <= 0:
            problems.append(f"n_dof must be a positive int, got {n_dof!r}")

    if isinstance(n_dof, int) and n_dof > 0:
        for key in (
            "joint_names",
            "home_position",
            "joint_limits_lower",
            "joint_limits_upper",
        ):
            if key in missing:
                continue
            value = config[key]
            try:
                length = len(value)
            except TypeError:
                problems.append(f"{key} must be a sequence, got {value!r}")
                continue
            if length != n_dof:
                problems.append(
                    f"{key} has length {length}, expected n_dof={n_dof}"
                )

    lo = config.get("joint_limits_lower")
    hi = config.get("joint_limits_upper")
    if (
        "joint_limits_lower" not in missing
        and "joint_limits_upper" not in missing
        and isinstance(lo, (list, tuple))
        and isinstance(hi, (list, tuple))
        and len(lo) == len(hi)
    ):
        for i, (a, b) in enumerate(zip(lo, hi)):
            if a > b:
                problems.append(
                    f"joint_limits_lower[{i}]={a} exceeds "
                    f"joint_limits_upper[{i}]={b}"
                )

    if problems:
        detail = "\n  - ".join(problems)
        raise ConfigError(f"invalid robot config:\n  - {detail}")
