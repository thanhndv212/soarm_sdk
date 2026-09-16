"""Unit tests for soarm_sdk.tuning.step_test."""

from __future__ import annotations

from soarm_sdk.tuning.step_test import Sample, run_step_test


def test_records_commanded_step_and_samples():
    positions = iter([1000.0, 1200.0, 1500.0, 1900.0, 2000.0])
    commanded = []

    def command():
        commanded.append(True)

    def read_sample():
        pos = next(positions, None)
        if pos is None:
            return None
        return Sample(t_s=0.0, position_ticks=pos, current_mA=100.0, temperature_C=30.0)

    response = run_step_test(
        command=command,
        read_sample=read_sample,
        start_ticks=1000.0,
        target_ticks=2000.0,
        duration_s=0.05,
        poll_interval_s=0.0,
    )

    assert commanded == [True]
    assert response.start_ticks == 1000.0
    assert response.target_ticks == 2000.0
    assert list(response.position_ticks[:5]) == [1000.0, 1200.0, 1500.0, 1900.0, 2000.0]
    assert response.aborted_reason is None


def test_watchdog_aborts_early_and_stops_polling():
    samples = iter(
        [
            Sample(t_s=0.0, position_ticks=1000.0, current_mA=100.0),
            Sample(t_s=0.05, position_ticks=1500.0, current_mA=200.0),
            Sample(t_s=0.10, position_ticks=1800.0, current_mA=5000.0),  # trips it
            Sample(t_s=0.15, position_ticks=1900.0, current_mA=100.0),
        ]
    )

    def read_sample():
        return next(samples, None)

    def watchdog(sample):
        if sample.current_mA is not None and sample.current_mA > 3000.0:
            return "current too high"
        return None

    response = run_step_test(
        command=lambda: None,
        read_sample=read_sample,
        start_ticks=1000.0,
        target_ticks=2000.0,
        duration_s=10.0,  # would never finish on its own within this test
        poll_interval_s=0.0,
        watchdog=watchdog,
    )

    assert response.aborted_reason == "current too high"
    # The sample after the trip must never have been folded into the response.
    assert response.position_ticks[-1] == 1800.0


def test_should_stop_ends_trial_without_marking_it_aborted():
    samples = iter(
        [
            Sample(t_s=0.0, position_ticks=1000.0),
            Sample(t_s=0.05, position_ticks=1500.0),
        ]
    )
    stop_after = {"count": 0}

    def read_sample():
        return next(samples, None)

    def should_stop():
        stop_after["count"] += 1
        return stop_after["count"] > 2

    response = run_step_test(
        command=lambda: None,
        read_sample=read_sample,
        start_ticks=1000.0,
        target_ticks=2000.0,
        duration_s=100.0,
        poll_interval_s=0.0,
        should_stop=should_stop,
    )

    assert response.aborted_reason is None


def test_on_sample_callback_receives_growing_response():
    samples = iter(
        [
            Sample(t_s=0.0, position_ticks=1000.0),
            Sample(t_s=0.05, position_ticks=1500.0),
        ]
    )
    seen_lengths = []

    def read_sample():
        return next(samples, None)

    def on_sample(partial):
        seen_lengths.append(len(partial.position_ticks))

    run_step_test(
        command=lambda: None,
        read_sample=read_sample,
        start_ticks=1000.0,
        target_ticks=2000.0,
        duration_s=0.03,
        poll_interval_s=0.0,
        on_sample=on_sample,
    )

    assert seen_lengths == [1, 2]
