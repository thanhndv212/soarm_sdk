"""Unit tests for soarm_sdk.hardware_interface.ServoHardwareInterface.

Covers pure logic reachable without an open serial port: state caching,
tick<->radian conversion round trips, and command-buffer constructon.
start()/stop() and the background read/write loop need real hardware and
are out of scope for unit tests.
"""

from __future__ import annotations

import numpy as np
import pytest

from soarm_sdk.hardware_interface import ServoHardwareInterface


def test_state_age_is_infinite_before_first_read():
    hw = ServoHardwareInterface(port="/dev/nonexistent")
    assert hw.state_age() == float("inf")


def test_get_robot_joint_positions_reads_from_seeded_cache():
    hw = ServoHardwareInterface(port="/dev/nonexistent", joint_ids=[1, 2])
    hw._cached_positions_ticks = [2048, 2048]  # TICK_ZERO for both joints

    positions = hw.get_robot_joint_positions()

    assert positions == pytest.approx([0.0, 0.0], abs=1e-6)


def test_get_robot_joint_state_converts_ticks_to_physical_units():
    hw = ServoHardwareInterface(port="/dev/nonexistent", joint_ids=[1])
    hw._cached_positions_ticks = [2048]
    hw._cached_speeds_ticks = [100]
    hw._cached_currents_mA = [250.0]
    hw._last_read_time = 42.0

    state = hw.get_robot_joint_state()

    assert state.positions == pytest.approx([0.0], abs=1e-6)
    assert state.velocities[0] != 0.0  # 100 ticks/s converts to a nonzero rad/s
    assert state.efforts == pytest.approx([250.0])
    assert state.timestamp == pytest.approx(42.0)


def test_set_robot_joint_positions_queues_a_command():
    hw = ServoHardwareInterface(port="/dev/nonexistent", joint_ids=[1, 2])

    hw.set_robot_joint_positions(np.array([0.0, 0.0]))

    assert hw._pending_command is not None
    assert hw._pending_command.ticks_list == [2048, 2048]
    assert hw._pending_command.speed_ticks_list == [hw._default_speed] * 2


def test_set_robot_joint_positions_speed_override_takes_priority_over_dq():
    hw = ServoHardwareInterface(port="/dev/nonexistent", joint_ids=[1])

    hw.set_robot_joint_positions(np.array([0.0]), dq=np.array([5.0]), speed=42)

    assert hw._pending_command.speed_ticks_list == [42]


def test_get_body_pose_without_fk_fn_raises():
    hw = ServoHardwareInterface(port="/dev/nonexistent")
    with pytest.raises(NotImplementedError):
        hw.get_body_pose("ee")


def test_get_body_pose_delegates_to_fk_fn():
    expected = (np.array([1.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0, 0.0]))
    hw = ServoHardwareInterface(port="/dev/nonexistent", fk_fn=lambda q: expected)

    pos, quat = hw.get_body_pose("ee")

    assert np.array_equal(pos, expected[0])
    assert np.array_equal(quat, expected[1])


def test_write_if_pending_is_a_noop_with_no_pending_command():
    hw = ServoHardwareInterface(port="/dev/nonexistent")
    hw._write_if_pending()  # must not raise even though _srv is None


def test_read_once_is_a_noop_before_start():
    hw = ServoHardwareInterface(port="/dev/nonexistent")
    hw._read_once()  # _gsr/_srv are None pre-start(); must not raise
    assert hw.read_errors == 0
