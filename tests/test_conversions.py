"""Unit tests for soarm_sdk.conversions.

Pure math, no hardware — encoder ticks <-> radians/rad-s conversions.
"""

from __future__ import annotations

import math

import pytest

from soarm_sdk.conversions import (
    TICK_ZERO,
    TICKS_PER_REV,
    joint_radians_to_ticks,
    joint_ticks_to_radians,
    rad_s_to_speed_ticks,
    radians_to_ticks,
    speed_ticks_to_rad_s,
    ticks_to_radians,
)


def test_zero_offset_maps_to_zero_radians():
    assert ticks_to_radians(TICK_ZERO) == 0.0


def test_ticks_to_radians_full_revolution():
    result = ticks_to_radians(TICK_ZERO + TICKS_PER_REV)
    assert result == pytest.approx(2 * math.pi)


def test_ticks_to_radians_respects_custom_zero_offset():
    assert ticks_to_radians(2148, zero_offset=2048) == pytest.approx(
        ticks_to_radians(100, zero_offset=0)
    )


@pytest.mark.parametrize("rad", [0.0, 1.57, -1.57, 3.14, -3.14])
def test_radians_ticks_round_trip(rad):
    ticks = radians_to_ticks(rad)
    back = ticks_to_radians(ticks)
    # Round-trip through an integer tick loses at most one quantization step.
    assert back == pytest.approx(rad, abs=2e-3)


def test_radians_to_ticks_clamps_to_valid_range():
    assert radians_to_ticks(1000.0) == TICKS_PER_REV - 1
    assert radians_to_ticks(-1000.0) == 0


def test_speed_round_trip():
    for rad_s in (0.0, 1.0, -1.0, 4.5):
        ticks = rad_s_to_speed_ticks(rad_s)
        assert speed_ticks_to_rad_s(ticks) == pytest.approx(rad_s, abs=1e-2)


def test_speed_ticks_clamped_to_servo_max():
    assert rad_s_to_speed_ticks(1e6) == 3000
    assert rad_s_to_speed_ticks(-1e6) == -3000


def test_joint_ticks_to_radians_uses_defaults_for_six_joints():
    ticks = [TICK_ZERO] * 6
    assert joint_ticks_to_radians(ticks) == [0.0] * 6


def test_joint_ticks_to_radians_applies_direction_signs():
    ticks = [TICK_ZERO + 100] * 2
    signs = [1, -1]
    result = joint_ticks_to_radians(ticks, direction_signs=signs)
    assert result[0] == pytest.approx(-result[1])


def test_joint_radians_to_ticks_is_inverse_of_joint_ticks_to_radians():
    radians = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
    ticks = joint_radians_to_ticks(radians)
    back = joint_ticks_to_radians(ticks)
    for expected, actual in zip(radians, back):
        assert actual == pytest.approx(expected, abs=2e-3)


def test_joint_conversions_respect_custom_zero_offsets():
    zeros = [2000, 2048, 2100]
    ticks = [2000, 2048, 2100]
    assert joint_ticks_to_radians(ticks, zero_offsets=zeros) == [0.0, 0.0, 0.0]
