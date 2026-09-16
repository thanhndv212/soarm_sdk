"""Unit tests for soarm_sdk.tuning.watchdog."""

from __future__ import annotations

import pytest

from soarm_sdk.tuning.step_test import Sample
from soarm_sdk.tuning.watchdog import Watchdog, WatchdogLimits, check_sample

LIMITS = WatchdogLimits(current_max_mA=2000.0, temperature_max_C=55.0, oscillation_hard_max=5)


@pytest.mark.parametrize(
    "field,value",
    [("current_max_mA", 0.0), ("temperature_max_C", 0.0)],
)
def test_rejects_non_positive_limits(field, value):
    defaults = dict(current_max_mA=2000.0, temperature_max_C=55.0, oscillation_hard_max=5)
    defaults[field] = value
    with pytest.raises(ValueError):
        WatchdogLimits(**defaults)


def test_rejects_negative_oscillation_hard_max():
    with pytest.raises(ValueError):
        WatchdogLimits(current_max_mA=2000.0, temperature_max_C=55.0, oscillation_hard_max=-1)


def test_passes_normal_sample():
    sample = Sample(t_s=0.0, position_ticks=1500.0, current_mA=500.0, temperature_C=30.0)
    assert check_sample(sample, LIMITS) is None


def test_flags_overcurrent():
    sample = Sample(t_s=0.0, position_ticks=1500.0, current_mA=3000.0)
    reason = check_sample(sample, LIMITS)
    assert reason is not None and "current" in reason


def test_flags_overtemperature():
    sample = Sample(t_s=0.0, position_ticks=1500.0, temperature_C=80.0)
    reason = check_sample(sample, LIMITS)
    assert reason is not None and "temperature" in reason


def test_missing_channel_is_not_a_breach():
    sample = Sample(t_s=0.0, position_ticks=1500.0)
    assert check_sample(sample, LIMITS) is None


def test_flags_oscillation_hard_ceiling():
    reason = check_sample(
        Sample(t_s=0.0, position_ticks=1500.0), LIMITS, sign_changes_so_far=6
    )
    assert reason is not None and "oscillation" in reason


def test_watchdog_class_tracks_sign_changes_against_target():
    wd = Watchdog(
        WatchdogLimits(current_max_mA=99999.0, temperature_max_C=999.0, oscillation_hard_max=2),
        target_ticks=2000.0,
    )
    # Error alternates sign each sample: above, below, above, below, above...
    samples = [
        Sample(t_s=0.0, position_ticks=2100.0),  # error +100
        Sample(t_s=0.1, position_ticks=1900.0),  # error -100 (1 sign change)
        Sample(t_s=0.2, position_ticks=2100.0),  # error +100 (2)
        Sample(t_s=0.3, position_ticks=1900.0),  # error -100 (3) -> exceeds hard_max=2
    ]
    reasons = [wd(s) for s in samples]
    assert reasons[:3] == [None, None, None]
    assert reasons[3] is not None and "oscillation" in reasons[3]


def test_watchdog_class_ignores_current_and_temperature_when_within_limits():
    wd = Watchdog(LIMITS, target_ticks=2000.0)
    sample = Sample(t_s=0.0, position_ticks=1500.0, current_mA=100.0, temperature_C=25.0)
    assert wd(sample) is None
