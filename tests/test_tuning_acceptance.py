"""Unit tests for soarm_sdk.tuning.acceptance."""

from __future__ import annotations

from dataclasses import replace

import pytest

from soarm_sdk.tuning.acceptance import PIDAcceptanceCriteria, TuningStage
from soarm_sdk.tuning.metrics import StepMetrics

GOOD_METRICS = StepMetrics(
    rise_time_s=0.2,
    overshoot_pct=2.0,
    settling_time_s=0.5,
    steady_state_error_ticks=1.0,
    oscillation_count=0,
    peak_current_mA=500.0,
    peak_temperature_C=35.0,
    aborted_reason=None,
)


def _criteria(**overrides):
    defaults = dict(
        overshoot_max_pct=10.0,
        settling_time_max_s=1.0,
        steady_state_error_max_ticks=5.0,
        oscillation_count_max=1,
        current_peak_max_mA=1000.0,
        temperature_max_C=60.0,
        repeat_trials=3,
    )
    defaults.update(overrides)
    return PIDAcceptanceCriteria(**defaults)


@pytest.mark.parametrize(
    "field,value",
    [
        ("overshoot_max_pct", 0.0),
        ("overshoot_max_pct", -1.0),
        ("settling_time_max_s", 0.0),
        ("steady_state_error_max_ticks", 0.0),
        ("current_peak_max_mA", 0.0),
        ("temperature_max_C", 0.0),
    ],
)
def test_rejects_non_positive_thresholds(field, value):
    with pytest.raises(ValueError):
        _criteria(**{field: value})


def test_rejects_negative_oscillation_count_max():
    with pytest.raises(ValueError):
        _criteria(oscillation_count_max=-1)


def test_rejects_non_positive_repeat_trials():
    with pytest.raises(ValueError):
        _criteria(repeat_trials=0)


def test_good_metrics_pass_every_stage():
    report = _criteria().evaluate(GOOD_METRICS)
    assert report.ready
    assert all(stage.passed for stage in report.stages)


def test_overshoot_breach_fails_only_that_stage():
    bad = replace(GOOD_METRICS, overshoot_pct=50.0)
    report = _criteria().evaluate(bad)
    assert not report.ready
    assert not report.stage(TuningStage.OVERSHOOT).passed
    assert report.stage(TuningStage.SETTLING_TIME).passed


def test_aborted_trial_fails_every_stage():
    aborted = replace(GOOD_METRICS, aborted_reason="current too high")
    report = _criteria().evaluate(aborted)
    assert not report.ready
    assert not report.stage(TuningStage.ABORTED).passed
    for stage in TuningStage:
        assert not report.stage(stage).passed


def test_missing_current_reading_fails_current_stage():
    no_current = replace(GOOD_METRICS, peak_current_mA=None)
    report = _criteria().evaluate(no_current)
    assert not report.stage(TuningStage.CURRENT).passed


def test_as_markdown_contains_every_stage():
    report = _criteria().evaluate(GOOD_METRICS)
    md = report.as_markdown()
    for stage in TuningStage:
        assert stage.value in md
