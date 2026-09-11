"""Unit tests for soarm_sdk.robot.lerobot.LeRobotRobot.

Driven against a fake follower injected via ``follower_factory``, so these
run without lerobot installed and without a serial port.
"""

from __future__ import annotations

import numpy as np
import pytest

from soarm_sdk.robot.interfaces import RobotInterface
from soarm_sdk.robot.lerobot import DEFAULT_MOTOR_NAMES, LeRobotRobot


class _FakeFollower:
    """Stand-in for lerobot's SOFollower, in normalized degrees."""

    def __init__(self, port, max_relative_target, motor_names):
        self.port = port
        self.max_relative_target = max_relative_target
        self.motor_names = motor_names
        self.degrees = {f"{n}.pos": 0.0 for n in motor_names}
        self.connected = False
        self.calibrate_arg = None
        self.sent = []

    def connect(self, calibrate=True):
        self.connected = True
        self.calibrate_arg = calibrate

    def disconnect(self):
        self.connected = False

    def get_observation(self):
        # Real followers return more than positions; make sure the wrapper
        # tolerates extra keys.
        return {**self.degrees, "temperature": 31.5}

    def send_action(self, action):
        self.sent.append(dict(action))
        self.degrees.update(action)


def _make(**kwargs):
    holder = {}

    def factory(port, max_relative_target, motor_names):
        holder["follower"] = _FakeFollower(port, max_relative_target, motor_names)
        return holder["follower"]

    robot = LeRobotRobot("/dev/fake", follower_factory=factory, **kwargs)
    return robot, holder


def test_satisfies_robot_interface_protocol():
    robot, _ = _make()
    assert isinstance(robot, RobotInterface)


def test_connect_builds_follower_and_skips_lerobot_calibration():
    robot, holder = _make()
    robot.connect()
    follower = holder["follower"]
    assert follower.connected is True
    # calibrate=False: this SDK does not want lerobot re-homing the arm.
    assert follower.calibrate_arg is False
    assert follower.port == "/dev/fake"
    assert follower.motor_names == DEFAULT_MOTOR_NAMES


def test_connect_is_idempotent():
    robot, _ = _make()
    robot.connect()
    first = robot.follower
    robot.connect()
    assert robot.follower is first


def test_disconnect_releases_follower():
    robot, holder = _make()
    robot.connect()
    robot.disconnect()
    assert holder["follower"].connected is False
    assert robot.connected is False


def test_accessors_before_connect_raise():
    robot, _ = _make()
    with pytest.raises(RuntimeError, match="not connected"):
        robot.get_joint_positions()
    with pytest.raises(RuntimeError, match="not connected"):
        robot.set_joint_positions(np.zeros(6))


def test_degrees_are_converted_to_radians_on_read():
    robot, holder = _make()
    robot.connect()
    holder["follower"].degrees["shoulder_pan.pos"] = 90.0
    q = robot.get_joint_positions()
    assert q.shape == (6,)
    assert q[0] == pytest.approx(np.pi / 2)


def test_radians_are_converted_to_degrees_on_write():
    robot, holder = _make()
    robot.connect()
    target = np.zeros(6)
    target[1] = np.pi / 2
    robot.set_joint_positions(target)
    sent = holder["follower"].sent[-1]
    assert sent["shoulder_lift.pos"] == pytest.approx(90.0)
    assert set(sent) == {f"{n}.pos" for n in DEFAULT_MOTOR_NAMES}


def test_read_write_roundtrip():
    robot, _ = _make()
    robot.connect()
    target = np.array([0.1, -0.2, 0.3, -0.4, 0.5, 0.6])
    robot.set_joint_positions(target)
    np.testing.assert_allclose(robot.get_joint_positions(), target, atol=1e-12)


def test_wrong_length_command_raises():
    robot, _ = _make()
    robot.connect()
    with pytest.raises(ValueError, match="expected 6 joint positions"):
        robot.set_joint_positions(np.zeros(3))


def test_motor_names_length_must_match_n_dof():
    with pytest.raises(ValueError, match="motor_names has 2 entries"):
        LeRobotRobot("/dev/fake", motor_names=["a", "b"])


def test_max_relative_target_is_passed_to_lerobot():
    robot, holder = _make(max_relative_target=2.5)
    robot.connect()
    assert holder["follower"].max_relative_target == 2.5


def test_degrees_passthrough_api_matches_lerobot_shape():
    robot, holder = _make()
    robot.connect()
    holder["follower"].degrees["elbow_flex.pos"] = 12.0

    degrees = robot.get_joint_degrees()
    assert degrees["elbow_flex.pos"] == 12.0
    assert "temperature" not in degrees  # non-.pos keys filtered out

    # A partial dict passes through untouched — the point of this API.
    robot.send_joint_degrees({"gripper.pos": 80.0})
    assert holder["follower"].sent[-1] == {"gripper.pos": 80.0}


def test_get_joint_state_snapshot():
    robot, _ = _make()
    robot.connect()
    state = robot.get_joint_state()
    assert state.positions.shape == (6,)
    assert state.timestamp > 0


def test_get_ee_pose_without_fk_fn_is_identity():
    robot, _ = _make()
    robot.connect()
    pose = robot.get_ee_pose()
    np.testing.assert_allclose(pose.position, np.zeros(3))


def test_get_ee_pose_with_fk_fn_delegates():
    robot, _ = _make(
        fk_fn=lambda q: (np.array([1.0, 2.0, 3.0]), np.array([1.0, 0.0, 0.0, 0.0]))
    )
    robot.connect()
    np.testing.assert_allclose(robot.get_ee_pose().position, [1.0, 2.0, 3.0])


def test_context_manager_connects_and_disconnects():
    robot, holder = _make()
    with robot as r:
        assert r.connected is True
    assert holder["follower"].connected is False


def test_gripper_api_drives_the_jaw_joint():
    robot, _ = _make()
    robot.connect()
    robot.set_gripper(open=True)
    assert robot.gripper_is_open is True
    robot.set_gripper(open=False)
    assert robot.gripper_is_open is False
    robot.toggle_gripper()
    assert robot.gripper_is_open is True


def test_gripper_leaves_other_joints_untouched():
    robot, _ = _make()
    robot.connect()
    arm = np.array([0.1, -0.2, 0.3, -0.4, 0.5, 0.0])
    robot.set_joint_positions(arm)
    robot.set_gripper(open=True)
    q = robot.get_joint_positions()
    np.testing.assert_allclose(q[:5], arm[:5], atol=1e-12)
    assert q[5] != pytest.approx(0.0)
