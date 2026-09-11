"""Unit tests for soarm_sdk.robot.types."""

from __future__ import annotations

import numpy as np
import pytest

from soarm_sdk.robot.types import JointState, Pose


def test_pose_identity():
    pose = Pose.identity()
    assert np.array_equal(pose.position, [0.0, 0.0, 0.0])
    assert np.array_equal(pose.orientation, [1.0, 0.0, 0.0, 0.0])


def test_pose_as_array_concatenates_position_and_orientation():
    pose = Pose(position=np.array([1.0, 2.0, 3.0]), orientation=np.array([0.0, 1.0, 0.0, 0.0]))
    result = pose.as_array()
    assert np.array_equal(result, [1.0, 2.0, 3.0, 0.0, 1.0, 0.0, 0.0])


def test_joint_state_defaults_to_none_velocities_and_efforts():
    state = JointState(positions=np.array([0.1, 0.2]))
    assert state.velocities is None
    assert state.efforts is None
    assert state.timestamp == 0.0


def test_joint_state_holds_full_snapshot():
    state = JointState(
        positions=np.array([0.1]),
        velocities=np.array([0.5]),
        efforts=np.array([10.0]),
        timestamp=123.456,
    )
    assert state.positions[0] == pytest.approx(0.1)
    assert state.velocities[0] == pytest.approx(0.5)
    assert state.efforts[0] == pytest.approx(10.0)
    assert state.timestamp == pytest.approx(123.456)
