"""Unit tests for soarm_sdk.trajectory.resample."""

from __future__ import annotations

import numpy as np
import pytest

from soarm_sdk.trajectory import resample


def test_single_waypoint_passes_through_unchanged():
    wp = [np.array([1.0, 2.0])]
    out = resample(wp, max_step=0.1)
    assert len(out) == 1
    np.testing.assert_allclose(out[0], [1.0, 2.0])


def test_empty_list_passes_through():
    assert resample([], max_step=0.1) == []


def test_small_step_needs_no_interpolation():
    wp = [np.array([0.0]), np.array([0.05])]
    out = resample(wp, max_step=0.1)
    assert len(out) == 2
    np.testing.assert_allclose(out[0], [0.0])
    np.testing.assert_allclose(out[1], [0.05])


def test_large_step_is_interpolated_within_bound():
    wp = [np.zeros(3), np.array([1.0, 0.0, 0.0])]
    out = resample(wp, max_step=0.3)
    # ceil(1.0 / 0.3) = 4 sub-steps -> 5 points total
    assert len(out) == 5
    for a, b in zip(out, out[1:]):
        assert np.max(np.abs(b - a)) <= 0.3 + 1e-9
    np.testing.assert_allclose(out[0], wp[0])
    np.testing.assert_allclose(out[-1], wp[-1])


def test_preserves_input_waypoints_as_interpolation_endpoints():
    wp = [np.array([0.0]), np.array([1.0]), np.array([1.05])]
    out = resample(wp, max_step=0.3)
    out_list = [o[0] for o in out]
    assert 0.0 in out_list
    assert 1.0 in out_list
    assert 1.05 in out_list


def test_zero_or_negative_max_step_raises():
    wp = [np.zeros(1), np.ones(1)]
    with pytest.raises(ValueError):
        resample(wp, max_step=0.0)
    with pytest.raises(ValueError):
        resample(wp, max_step=-1.0)


def test_first_output_is_a_copy_not_input_array():
    wp = [np.array([1.0, 2.0]), np.array([1.0, 2.0])]
    out = resample(wp, max_step=0.1)
    out[0][0] = 999.0
    assert wp[0][0] == 1.0
