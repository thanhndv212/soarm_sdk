"""Unit tests for soarm_sdk.robot.null.NullRobot.

Exercises the RobotInterface contract with no hardware involved — the
same shape of test a caller can run in CI, unlike ServoRobot.
"""

from __future__ import annotations

import numpy as np

from soarm_sdk.robot.interfaces import RobotInterface
from soarm_sdk.robot.null import NullRobot
from soarm_sdk.types import Pose


def _config():
    return {
        "n_dof": 3,
        "joint_names": ["a", "b", "c"],
        "home_position": [0.1, 0.2, 0.3],
        "joint_limits_lower": [-1.0, -1.0, -1.0],
        "joint_limits_upper": [1.0, 1.0, 1.0],
    }


def test_satisfies_robot_interface_protocol():
    robot = NullRobot(_config())
    assert isinstance(robot, RobotInterface)


def test_starts_at_home_position():
    robot = NullRobot(_config())
    np.testing.assert_allclose(robot.get_joint_positions(), [0.1, 0.2, 0.3])


def test_connect_disconnect_track_state():
    robot = NullRobot(_config())
    assert robot.connected is False
    robot.connect()
    assert robot.connected is True
    robot.disconnect()
    assert robot.connected is False


def test_set_joint_positions_updates_state():
    robot = NullRobot(_config())
    robot.set_joint_positions(np.array([0.5, -0.5, 0.0]))
    np.testing.assert_allclose(robot.get_joint_positions(), [0.5, -0.5, 0.0])


def test_get_joint_positions_returns_a_copy():
    robot = NullRobot(_config())
    positions = robot.get_joint_positions()
    positions[0] = 99.0
    np.testing.assert_allclose(robot.get_joint_positions(), [0.1, 0.2, 0.3])


def test_get_joint_state_reflects_last_command():
    robot = NullRobot(_config())
    robot.set_joint_positions(np.array([0.5, -0.5, 0.0]))
    state = robot.get_joint_state()
    np.testing.assert_allclose(state.positions, [0.5, -0.5, 0.0])
    assert state.timestamp > 0


def test_get_ee_pose_without_fk_fn_returns_identity():
    robot = NullRobot(_config())
    pose = robot.get_ee_pose()
    np.testing.assert_allclose(pose.position, [0.0, 0.0, 0.0])
    np.testing.assert_allclose(pose.orientation, [1.0, 0.0, 0.0, 0.0])


def test_get_ee_pose_with_fk_fn_delegates():
    calls = []

    def fk_fn(q):
        calls.append(q.copy())
        return np.array([1.0, 2.0, 3.0]), np.array([0.0, 1.0, 0.0, 0.0])

    robot = NullRobot(_config(), fk_fn=fk_fn)
    robot.set_joint_positions(np.array([0.5, -0.5, 0.0]))
    pose = robot.get_ee_pose()
    assert isinstance(pose, Pose)
    np.testing.assert_allclose(pose.position, [1.0, 2.0, 3.0])
    np.testing.assert_allclose(calls[0], [0.5, -0.5, 0.0])


def test_go_home_resets_to_home_position():
    robot = NullRobot(_config())
    robot.set_joint_positions(np.array([0.9, 0.9, 0.9]))
    robot.go_home()
    np.testing.assert_allclose(robot.get_joint_positions(), [0.1, 0.2, 0.3])


def test_context_manager_connects_and_disconnects():
    robot = NullRobot(_config())
    with robot as r:
        assert r.connected is True
    assert robot.connected is False
