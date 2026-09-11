"""Unit tests for the config-driven gripper API on soarm_sdk.robot.base.Robot.

Exercised through NullRobot, since the API is implemented once on the base
class and inherited by every backend.
"""

from __future__ import annotations

import numpy as np
import pytest

from soarm_sdk.robot.config import ConfigError
from soarm_sdk.robot.null import NullRobot


def _config(with_gripper=True, **gripper_overrides):
    cfg = {
        "n_dof": 3,
        "joint_names": ["a", "b", "jaw"],
        "home_position": [0.0, 0.0, 0.0],
        "joint_limits_lower": [-2.0, -2.0, -2.0],
        "joint_limits_upper": [2.0, 2.0, 2.0],
    }
    if with_gripper:
        gripper = {"joint_index": 2, "open_rad": 1.4, "closed_rad": 0.0}
        gripper.update(gripper_overrides)
        cfg["gripper"] = gripper
    return cfg


def test_has_gripper_reflects_config():
    assert NullRobot(_config()).has_gripper is True
    assert NullRobot(_config(with_gripper=False)).has_gripper is False


def test_gripper_index_from_config():
    assert NullRobot(_config()).gripper_index == 2


def test_gripper_index_defaults_to_last_joint():
    cfg = _config()
    del cfg["gripper"]["joint_index"]
    assert NullRobot(cfg).gripper_index == 2


def test_set_gripper_open_and_closed():
    robot = NullRobot(_config())
    robot.set_gripper(open=True)
    assert robot.get_joint_positions()[2] == pytest.approx(1.4)
    robot.set_gripper(open=False)
    assert robot.get_joint_positions()[2] == pytest.approx(0.0)


def test_gripper_is_open_uses_nearest_pose():
    robot = NullRobot(_config())
    robot.set_joint_positions(np.array([0.0, 0.0, 1.3]))
    assert robot.gripper_is_open is True
    robot.set_joint_positions(np.array([0.0, 0.0, 0.2]))
    assert robot.gripper_is_open is False


def test_toggle_gripper_flips_state():
    robot = NullRobot(_config())
    robot.set_gripper(open=False)
    robot.toggle_gripper()
    assert robot.gripper_is_open is True
    robot.toggle_gripper()
    assert robot.gripper_is_open is False


def test_set_gripper_preserves_other_joints():
    robot = NullRobot(_config())
    robot.set_joint_positions(np.array([0.5, -0.5, 0.0]))
    robot.set_gripper(open=True)
    np.testing.assert_allclose(robot.get_joint_positions()[:2], [0.5, -0.5])


def test_gripper_api_without_config_block_raises_clearly():
    robot = NullRobot(_config(with_gripper=False))
    with pytest.raises(ConfigError, match="declares no 'gripper' block"):
        robot.set_gripper(open=True)
    with pytest.raises(ConfigError):
        robot.gripper_is_open
    with pytest.raises(ConfigError):
        robot.gripper_index


def test_shipped_soarm100_config_declares_a_gripper():
    robot = NullRobot()  # defaults to configs/soarm100.yaml
    assert robot.has_gripper is True
    assert robot.gripper_index == 5
