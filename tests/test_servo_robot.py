"""Unit tests for soarm_sdk.servo_robot.ServoRobot.

Covers construction and pre-connect() behavior only — a real connect()
opens a serial port and starts a background thread, which needs real
hardware and is out of scope for unit tests.
"""

from __future__ import annotations

import pytest

from soarm_sdk.interfaces import RobotInterface
from soarm_sdk.servo_robot import ServoRobot


def test_servo_robot_loads_default_config():
    robot = ServoRobot(port="/dev/nonexistent")
    assert robot.n_dof == 6
    assert len(robot.home_position) == 6


def test_servo_robot_satisfies_robot_interface_protocol():
    robot = ServoRobot(port="/dev/nonexistent")
    assert isinstance(robot, RobotInterface)


def test_servo_robot_state_age_is_infinite_before_connect():
    robot = ServoRobot(port="/dev/nonexistent")
    assert robot.state_age() == float("inf")


@pytest.mark.parametrize(
    "method,args",
    [
        ("get_joint_positions", ()),
        ("set_joint_positions", ([0.0] * 6,)),
        ("get_ee_pose", ()),
        ("get_joint_state", ()),
    ],
)
def test_servo_robot_methods_raise_before_connect(method, args):
    robot = ServoRobot(port="/dev/nonexistent")
    with pytest.raises(RuntimeError, match="not connected"):
        getattr(robot, method)(*args)


def test_servo_robot_hw_property_is_none_before_connect():
    robot = ServoRobot(port="/dev/nonexistent")
    assert robot.hw is None
