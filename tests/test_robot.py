"""Unit tests for soarm_sdk.robot (Robot ABC + load_robot_config)."""

from __future__ import annotations

import json

import numpy as np
import pytest

from soarm_sdk.robot import Robot, load_robot_config
from soarm_sdk.types import JointState, Pose

_TEST_CONFIG = {
    "n_dof": 2,
    "joint_names": ["a", "b"],
    "home_position": [0.0, 1.0],
    "joint_limits_lower": [-1.0, -2.0],
    "joint_limits_upper": [1.0, 2.0],
}


class _FakeRobot(Robot):
    """Minimal concrete Robot for exercising the ABC's shared behavior."""

    def __init__(self, config):
        super().__init__(config)
        self.connected = False
        self._q = np.array(config["home_position"])

    def connect(self):
        self.connected = True

    def disconnect(self):
        self.connected = False

    def get_joint_positions(self):
        return self._q

    def set_joint_positions(self, positions, dq=None):
        self._q = np.asarray(positions)

    def get_ee_pose(self):
        return Pose.identity()

    def get_joint_state(self):
        return JointState(positions=self._q)


# ---------------------------------------------------------------------------
# load_robot_config
# ---------------------------------------------------------------------------


def test_load_robot_config_default_soarm100():
    cfg = load_robot_config("soarm100")
    assert cfg["n_dof"] == 6
    assert len(cfg["joint_names"]) == 6
    assert len(cfg["home_position"]) == 6


def test_load_robot_config_missing_raises():
    with pytest.raises(FileNotFoundError):
        load_robot_config("does-not-exist")


def test_load_robot_config_explicit_json_path(tmp_path):
    path = tmp_path / "custom.json"
    path.write_text(json.dumps(_TEST_CONFIG))
    cfg = load_robot_config(path)
    assert cfg == _TEST_CONFIG


def test_load_robot_config_explicit_yaml_path(tmp_path):
    pytest.importorskip("yaml")
    import yaml

    path = tmp_path / "custom.yaml"
    path.write_text(yaml.safe_dump(_TEST_CONFIG))
    cfg = load_robot_config(path)
    assert cfg == _TEST_CONFIG


# ---------------------------------------------------------------------------
# Robot ABC — properties derived from config
# ---------------------------------------------------------------------------


def test_robot_n_dof_and_joint_names():
    robot = _FakeRobot(_TEST_CONFIG)
    assert robot.n_dof == 2
    assert robot.joint_names == ["a", "b"]


def test_robot_home_position_returns_a_copy():
    robot = _FakeRobot(_TEST_CONFIG)
    home = robot.home_position
    home[0] = 999.0
    assert robot.home_position[0] == 0.0  # mutation didn't leak into internal state


def test_robot_joint_limits():
    robot = _FakeRobot(_TEST_CONFIG)
    lo, hi = robot.get_joint_limits()
    assert np.array_equal(lo, [-1.0, -2.0])
    assert np.array_equal(hi, [1.0, 2.0])


def test_robot_go_home_commands_home_position():
    robot = _FakeRobot(_TEST_CONFIG)
    robot.set_joint_positions(np.array([0.5, 0.5]))
    robot.go_home()
    assert np.array_equal(robot.get_joint_positions(), [0.0, 1.0])


def test_robot_context_manager_connects_and_disconnects():
    robot = _FakeRobot(_TEST_CONFIG)
    with robot as r:
        assert r.connected is True
    assert robot.connected is False


def test_robot_repr_includes_n_dof():
    robot = _FakeRobot(_TEST_CONFIG)
    assert "n_dof=2" in repr(robot)
