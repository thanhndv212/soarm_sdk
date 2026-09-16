"""Non-GUI logic in the PID Tuning panel.

Matches this repo's convention for dashboard panels (see
``test_calibration_panel.py``): the GUI wiring itself isn't tested here —
the tuning algorithm and metrics are already covered under
``tests/test_tuning_*.py`` — only the two plain functions this panel adds
of its own: formatting a metrics report, and turning three register reads
into one :class:`Sample`.
"""

from __future__ import annotations

from soarm_sdk.dashboard.panels.pid import _format_metrics_md, _read_sample
from soarm_sdk.protocol.registers import COMM_RX_FAIL, COMM_SUCCESS
from soarm_sdk.tuning.metrics import StepMetrics


def test_format_metrics_md_includes_every_field():
    metrics = StepMetrics(
        rise_time_s=0.234,
        overshoot_pct=5.5,
        settling_time_s=0.8,
        steady_state_error_ticks=2.0,
        oscillation_count=1,
        peak_current_mA=650.0,
        peak_temperature_C=42.0,
        aborted_reason=None,
    )
    md = _format_metrics_md(metrics)
    assert "0.234" in md
    assert "5.5" in md
    assert "0.8" in md
    assert "650" in md
    assert "42.0" in md
    assert "ABORTED" not in md


def test_format_metrics_md_shows_na_for_missing_channels():
    metrics = StepMetrics(
        rise_time_s=None,
        overshoot_pct=0.0,
        settling_time_s=0.1,
        steady_state_error_ticks=0.0,
        oscillation_count=0,
        peak_current_mA=None,
        peak_temperature_C=None,
        aborted_reason=None,
    )
    md = _format_metrics_md(metrics)
    assert md.count("n/a") == 3  # rise time, current, temperature


def test_format_metrics_md_surfaces_abort_reason():
    metrics = StepMetrics(
        rise_time_s=0.1,
        overshoot_pct=0.0,
        settling_time_s=0.1,
        steady_state_error_ticks=0.0,
        oscillation_count=0,
        peak_current_mA=100.0,
        peak_temperature_C=30.0,
        aborted_reason="current too high",
    )
    md = _format_metrics_md(metrics)
    assert "ABORTED" in md
    assert "current too high" in md


class _FakeServo:
    def __init__(self, pos=1500, pos_result=COMM_SUCCESS, current=650.0,
                 current_result=COMM_SUCCESS, temp=35, temp_result=COMM_SUCCESS):
        self._pos = (pos, pos_result, 0)
        self._current = (current, current_result, 0)
        self._temp = (temp, temp_result, 0)

    def ReadPos(self, sid):
        return self._pos

    def ReadCurrent(self, sid):
        return self._current

    def ReadTemperature(self, sid):
        return self._temp


def test_read_sample_builds_sample_from_three_reads():
    srv = _FakeServo(pos=1500, current=-650.0, temp=35)
    sample = _read_sample(srv, 1, t0=0.0)
    assert sample is not None
    assert sample.position_ticks == 1500.0
    assert sample.current_mA == 650.0  # sign-agnostic magnitude
    assert sample.temperature_C == 35.0
    assert sample.t_s >= 0.0


def test_read_sample_returns_none_on_position_read_failure():
    srv = _FakeServo(pos_result=COMM_RX_FAIL)
    assert _read_sample(srv, 1, t0=0.0) is None


def test_read_sample_tolerates_current_and_temperature_failures():
    srv = _FakeServo(current_result=COMM_RX_FAIL, temp_result=COMM_RX_FAIL)
    sample = _read_sample(srv, 1, t0=0.0)
    assert sample is not None
    assert sample.current_mA is None
    assert sample.temperature_C is None
