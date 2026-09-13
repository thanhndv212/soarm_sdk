"""Unit tests for re-zeroing a calibration from a physically held pose.

seed_from_travel infers the zero by matching measured travel endpoints to the
URDF's limits, assuming both describe the same hard stops. On this arm they
disagree by 7-34% (see span_ratio), so that inference misplaces every zero.
rezero_from_pose takes the zero from a pose the operator can verify instead.
"""

from __future__ import annotations

import pytest

from soarm_sdk.calibration.frame import (
    JointCalibration,
    RobotCalibration,
    rezero_from_pose,
)
from soarm_sdk.conversions import TICKS_PER_RAD


def _calib(signs=(1, 1)):
    return RobotCalibration(
        joints=[
            JointCalibration(name="a", zero_offset_ticks=2048.0, direction_sign=signs[0],
                             tick_min=100, tick_max=4000, span_ratio=1.2),
            JointCalibration(name="b", zero_offset_ticks=2048.0, direction_sign=signs[1],
                             tick_min=200, tick_max=3900, span_ratio=0.9),
        ],
        arm_id="arm", validated=True,
    )


def test_zero_reference_pose_makes_the_measured_ticks_the_zero():
    out = rezero_from_pose(_calib(), ticks=[1500, 2500])

    assert out.zero_offsets == pytest.approx([1500.0, 2500.0])
    assert out.ticks_to_rad([1500, 2500]) == pytest.approx([0.0, 0.0])


def test_a_nonzero_reference_pose_offsets_the_zero_by_that_angle():
    out = rezero_from_pose(_calib(), ticks=[1500, 2500], reference_rad=[0.5, -0.25])

    assert out.ticks_to_rad([1500, 2500]) == pytest.approx([0.5, -0.25])


def test_a_negative_direction_sign_is_honoured():
    out = rezero_from_pose(_calib(signs=(1, -1)), ticks=[1500, 2500],
                           reference_rad=[0.0, 0.3])

    assert out.ticks_to_rad([1500, 2500]) == pytest.approx([0.0, 0.3])
    # sign -1 puts the zero on the other side of the measured tick
    assert out.joints[1].zero_offset_ticks == pytest.approx(2500 + 0.3 * TICKS_PER_RAD)


def test_measured_travel_is_carried_over_untouched():
    out = rezero_from_pose(_calib(), ticks=[1500, 2500])

    assert [(j.tick_min, j.tick_max) for j in out.joints] == [(100, 4000), (200, 3900)]
    # span_ratio is the record that travel and URDF limits disagree; keep it
    assert [j.span_ratio for j in out.joints] == pytest.approx([1.2, 0.9])


def test_the_seed_residual_is_cleared_because_no_seeding_happened():
    seeded = RobotCalibration(
        joints=[
            JointCalibration(name="a", zero_offset_ticks=2048.0, direction_sign=1,
                             tick_min=100, tick_max=4000, seed_residual_rad=0.21),
            JointCalibration(name="b", zero_offset_ticks=2048.0, direction_sign=1,
                             tick_min=200, tick_max=3900, seed_residual_rad=0.33),
        ],
        arm_id="arm",
    )

    out = rezero_from_pose(seeded, ticks=[1500, 2500])

    assert out.joints[0].seed_residual_rad == 0.0


def test_result_is_unvalidated_even_from_a_validated_input():
    out = rezero_from_pose(_calib(), ticks=[1500, 2500])
    assert out.validated is False


def test_wrong_length_inputs_are_rejected():
    with pytest.raises(ValueError):
        rezero_from_pose(_calib(), ticks=[1500])
    with pytest.raises(ValueError):
        rezero_from_pose(_calib(), ticks=[1500, 2500], reference_rad=[0.0])
