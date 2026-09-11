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


# ===========================================================================
# Safety layer: joint limits and step clamping
#
# These exercise _apply_safety directly rather than set_robot_joint_positions,
# because the latter queues a command for a bus thread that is not running
# without hardware. The clamping is what needs testing; the queueing is not.
# ===========================================================================


def _iface(**kw) -> ServoHardwareInterface:
    """Interface over a nonexistent port with a seeded position cache.

    2048 ticks is 0 rad under the default zero offsets, so "current
    position" is the origin unless a test says otherwise.
    """
    hw = ServoHardwareInterface(port="/dev/nonexistent", joint_ids=[1, 2], **kw)
    hw._cached_positions_ticks = [2048, 2048]
    return hw


def test_no_limits_configured_lets_anything_through():
    """Defaults must stay off, or adding safety would break every caller."""
    hw = _iface()
    out = hw._apply_safety(np.array([5.0, -5.0]))
    assert out == pytest.approx([5.0, -5.0])
    assert hw.limit_clamps == 0 and hw.step_clamps == 0


def test_joint_limits_are_enforced_on_write():
    hw = _iface(joint_limits=([-1.0, -1.0], [1.0, 1.0]))
    out = hw._apply_safety(np.array([2.5, -0.5]))
    assert out == pytest.approx([1.0, -0.5])
    assert hw.limit_clamps == 1


def test_limit_clamps_accumulate_across_calls():
    hw = _iface(joint_limits=([-1.0, -1.0], [1.0, 1.0]))
    hw._apply_safety(np.array([2.0, 2.0]))
    hw._apply_safety(np.array([2.0, 0.0]))
    assert hw.limit_clamps == 3


def test_in_range_target_is_not_counted_as_a_clamp():
    hw = _iface(joint_limits=([-1.0, -1.0], [1.0, 1.0]))
    hw._apply_safety(np.array([1.0, -1.0]))  # exactly on the limits
    assert hw.limit_clamps == 0


def test_step_clamp_limits_distance_from_the_measured_position():
    hw = _iface(max_step_rad=0.05)
    out = hw._apply_safety(np.array([1.0, -1.0]))
    assert out == pytest.approx([0.05, -0.05])
    assert hw.step_clamps == 2


def test_step_clamp_measures_against_actual_not_previous_command():
    """A lagging joint must not accumulate an ever-larger commanded jump.

    Commanding repeatedly while the arm does not move should keep producing
    the same bounded target, not walk away from the measured position.
    """
    hw = _iface(max_step_rad=0.05)
    first = hw._apply_safety(np.array([1.0, 1.0]))
    second = hw._apply_safety(np.array([1.0, 1.0]))
    assert first == pytest.approx(second)


def test_small_step_passes_untouched():
    hw = _iface(max_step_rad=0.05)
    out = hw._apply_safety(np.array([0.01, -0.02]))
    assert out == pytest.approx([0.01, -0.02])
    assert hw.step_clamps == 0


def test_limits_apply_before_the_step_clamp():
    """Order matters: clamping to a reachable pose first, then bounding the
    step toward it, is what makes an out-of-range target approach the limit
    rather than stall at the current position."""
    hw = _iface(joint_limits=([-0.2, -0.2], [0.2, 0.2]), max_step_rad=0.05)
    out = hw._apply_safety(np.array([10.0, 10.0]))
    assert out == pytest.approx([0.05, 0.05])
    assert hw.limit_clamps == 2 and hw.step_clamps == 2


def test_bad_safety_configuration_is_rejected_at_construction():
    with pytest.raises(ValueError, match="max_step_rad"):
        _iface(max_step_rad=0.0)
    with pytest.raises(ValueError, match="lower"):
        _iface(joint_limits=([1.0, 1.0], [-1.0, -1.0]))
    with pytest.raises(ValueError, match="entries per side"):
        _iface(joint_limits=([-1.0], [1.0]))


def test_wrong_length_target_is_rejected():
    hw = _iface()
    with pytest.raises(ValueError, match="joint values"):
        hw._apply_safety(np.array([0.0, 0.0, 0.0]))


def test_calibration_supplies_offsets_and_signs_together():
    """Passing a calibration must override both, so the two cannot be set
    from different sources and disagree."""
    from soarm_sdk.frame_calibration import seed_from_travel

    cal = seed_from_travel(
        names=["a", "b"],
        urdf_limits=[(-1.0, 1.0), (-1.0, 1.0)],
        tick_ranges=[(1000, 3000), (500, 2500)],
    )
    hw = ServoHardwareInterface(
        port="/dev/nonexistent", joint_ids=[1, 2], calibration=cal
    )
    assert hw._zero_offsets == pytest.approx([2000.0, 1500.0])
    assert hw._direction_signs == [1, 1]


def test_calibration_joint_count_must_match():
    from soarm_sdk.frame_calibration import seed_from_travel

    cal = seed_from_travel(["a"], [(-1.0, 1.0)], [(1000, 3000)])
    with pytest.raises(ValueError, match="calibration has 1 joints"):
        ServoHardwareInterface(
            port="/dev/nonexistent", joint_ids=[1, 2], calibration=cal
        )
