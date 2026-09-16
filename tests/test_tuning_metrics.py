"""Unit tests for soarm_sdk.tuning.metrics."""

from __future__ import annotations

import pytest

from soarm_sdk.tuning.metrics import StepResponse, compute_step_metrics


def _clean_step(n=30, start=1000.0, target=2000.0):
    """A well-behaved first-order-ish approach with no overshoot or ringing."""
    t = [i * 0.05 for i in range(n)]
    pos = []
    for i in range(n):
        frac = min(1.0, i / (n - 5))
        pos.append(start + frac * (target - start))
    current = [100.0 + i for i in range(n)]
    temp = [30.0] * n
    return StepResponse(
        t_s=tuple(t),
        position_ticks=tuple(pos),
        current_mA=tuple(current),
        temperature_C=tuple(temp),
        start_ticks=start,
        target_ticks=target,
    )


def test_rejects_zero_length_step():
    resp = StepResponse(
        t_s=(0.0, 0.05, 0.1),
        position_ticks=(1000.0, 1000.0, 1000.0),
        current_mA=(None, None, None),
        temperature_C=(None, None, None),
        start_ticks=1000.0,
        target_ticks=1000.0,
    )
    with pytest.raises(ValueError, match="no step to measure"):
        compute_step_metrics(resp)


def test_rejects_too_few_samples():
    resp = StepResponse(
        t_s=(0.0, 0.05),
        position_ticks=(1000.0, 1500.0),
        current_mA=(None, None),
        temperature_C=(None, None),
        start_ticks=1000.0,
        target_ticks=2000.0,
    )
    with pytest.raises(ValueError, match="insufficient"):
        compute_step_metrics(resp)


def test_clean_step_has_no_overshoot_and_small_final_error():
    metrics = compute_step_metrics(_clean_step())
    assert metrics.overshoot_pct == pytest.approx(0.0, abs=1e-6)
    assert metrics.steady_state_error_ticks < 1.0
    assert metrics.rise_time_s is not None and metrics.rise_time_s > 0
    assert metrics.oscillation_count == 0
    assert metrics.aborted_reason is None


def test_overshoot_detected_past_target():
    n = 20
    t = [i * 0.05 for i in range(n)]
    pos = [1000.0 + i * 60.0 for i in range(n)]  # overshoots 2000 then stays high
    resp = StepResponse(
        t_s=tuple(t),
        position_ticks=tuple(pos),
        current_mA=tuple([None] * n),
        temperature_C=tuple([None] * n),
        start_ticks=1000.0,
        target_ticks=2000.0,
    )
    metrics = compute_step_metrics(resp)
    assert metrics.overshoot_pct > 0.0


def test_oscillation_counts_ringing_after_settling():
    # Approaches target, then rings around it a few times.
    t = [i * 0.05 for i in range(12)]
    pos = [1000, 1200, 1500, 1800, 2050, 1950, 2030, 1980, 2010, 1995, 2002, 1999]
    resp = StepResponse(
        t_s=tuple(t),
        position_ticks=tuple(float(p) for p in pos),
        current_mA=tuple([None] * 12),
        temperature_C=tuple([None] * 12),
        start_ticks=1000.0,
        target_ticks=2000.0,
    )
    metrics = compute_step_metrics(resp, settle_band_frac=0.01)
    assert metrics.oscillation_count > 0


def test_peak_current_and_temperature_ignore_missing_channel():
    resp = _clean_step()
    metrics = compute_step_metrics(resp)
    assert metrics.peak_current_mA == max(resp.current_mA)
    assert metrics.peak_temperature_C == 30.0


def test_peak_is_none_when_channel_entirely_missing():
    resp = _clean_step()
    resp_no_current = StepResponse(
        t_s=resp.t_s,
        position_ticks=resp.position_ticks,
        current_mA=tuple([None] * len(resp.t_s)),
        temperature_C=resp.temperature_C,
        start_ticks=resp.start_ticks,
        target_ticks=resp.target_ticks,
    )
    metrics = compute_step_metrics(resp_no_current)
    assert metrics.peak_current_mA is None


def test_aborted_reason_carried_through():
    resp = StepResponse(
        t_s=(0.0, 0.05, 0.1),
        position_ticks=(1000.0, 1100.0, 1150.0),
        current_mA=(None, None, None),
        temperature_C=(None, None, None),
        start_ticks=1000.0,
        target_ticks=2000.0,
        aborted_reason="current 5000mA exceeds 3000mA",
    )
    metrics = compute_step_metrics(resp)
    assert metrics.aborted_reason == "current 5000mA exceeds 3000mA"
