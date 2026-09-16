"""Unit tests for soarm_sdk.tuning.search, against a synthetic PID plant.

No hardware, no serial port: `_simulate_step` is a small discrete
mass-spring-damper-ish plant driven by a P/D/I position controller, standing
in for "the servo and its onboard controller" so the search algorithm's
convergence and bounds behavior can be verified on its own.
"""

from __future__ import annotations

from soarm_sdk.tuning.acceptance import PIDAcceptanceCriteria
from soarm_sdk.tuning.gains_io import Gains
from soarm_sdk.tuning.metrics import StepResponse, compute_step_metrics
from soarm_sdk.tuning.search import SearchBounds, coordinate_descent_tune


def _simulate_step(gains: Gains, *, target=2000.0, start=1000.0, dt=0.01, n=200):
    """A critically-tunable discrete plant: position, velocity, PID force.

    Deliberately simple (not a Feetech model) — it exists so the search
    algorithm has *something* with a real optimum to find, with gains that
    are too low producing a sluggish, never-arriving response and gains
    that are too high producing overshoot/ringing.
    """
    pos = start
    vel = 0.0
    integral = 0.0
    prev_error = target - pos
    t = []
    positions = []
    mass = 1.0
    damping_natural = 2.0  # mechanical damping, independent of gains

    kp = gains.p / 32.0 * 40.0
    kd = gains.d / 32.0 * 8.0
    ki = gains.i / 32.0 * 2.0

    for step in range(n):
        error = target - pos
        integral += error * dt
        derivative = (error - prev_error) / dt
        prev_error = error

        force = kp * error + kd * derivative + ki * integral
        accel = (force - damping_natural * vel) / mass
        vel += accel * dt
        pos += vel * dt

        t.append(step * dt)
        positions.append(pos)

    return StepResponse(
        t_s=tuple(t),
        position_ticks=tuple(positions),
        current_mA=tuple([200.0 + abs(gains.p - 32) * 5.0] * n),
        temperature_C=tuple([30.0] * n),
        start_ticks=start,
        target_ticks=target,
    )


def _criteria(**overrides):
    defaults = dict(
        overshoot_max_pct=20.0,
        settling_time_max_s=1.5,
        steady_state_error_max_ticks=10.0,
        oscillation_count_max=3,
        current_peak_max_mA=5000.0,
        temperature_max_C=80.0,
        repeat_trials=2,
    )
    defaults.update(overrides)
    return PIDAcceptanceCriteria(**defaults)


def _run_trial_factory():
    def run_trial(gains: Gains):
        response = _simulate_step(gains)
        return compute_step_metrics(response)

    return run_trial


def test_converges_to_a_lower_cost_than_a_deliberately_bad_start():
    bounds = SearchBounds(p_min=0, p_max=100, d_min=0, d_max=100, i_min=0, i_max=20)
    criteria = _criteria()

    bad_start = Gains(p=4, d=0, i=0)  # far too weak: sluggish, never settles in time
    result = coordinate_descent_tune(
        initial=bad_start,
        bounds=bounds,
        run_trial=_run_trial_factory(),
        criteria=criteria,
        max_trials=40,
        initial_step=16,
    )

    from soarm_sdk.tuning.search import cost as cost_fn

    start_metrics = compute_step_metrics(_simulate_step(bad_start))
    start_cost = cost_fn(start_metrics, criteria)

    assert result.best.cost < start_cost
    assert len(result.trials) <= 40


def test_never_proposes_gains_outside_bounds():
    bounds = SearchBounds(p_min=10, p_max=50, d_min=10, d_max=50, i_min=0, i_max=5)
    result = coordinate_descent_tune(
        initial=Gains(p=32, d=32, i=0),
        bounds=bounds,
        run_trial=_run_trial_factory(),
        criteria=_criteria(),
        max_trials=30,
        initial_step=16,
    )

    for trial in result.trials:
        assert bounds.p_min <= trial.gains.p <= bounds.p_max
        assert bounds.d_min <= trial.gains.d <= bounds.d_max
        assert bounds.i_min <= trial.gains.i <= bounds.i_max


def test_validated_only_after_repeat_trials_consecutive_passes():
    bounds = SearchBounds(p_min=0, p_max=100, d_min=0, d_max=100, i_min=0, i_max=20)
    result = coordinate_descent_tune(
        initial=Gains(p=32, d=32, i=0),
        bounds=bounds,
        run_trial=_run_trial_factory(),
        criteria=_criteria(repeat_trials=3),
        max_trials=40,
    )

    if result.validated:
        assert result.consecutive_passes_at_best >= 3
    else:
        assert result.exhausted


def test_a_single_flaky_repeat_fails_validation():
    """A run_trial that always regresses on repeat must not be marked validated."""

    calls = {"n": 0}

    def flaky_run_trial(gains: Gains):
        calls["n"] += 1
        response = _simulate_step(gains)
        if calls["n"] > 3:
            # Force every repeat-phase call to look aborted.
            response = StepResponse(
                t_s=response.t_s,
                position_ticks=response.position_ticks,
                current_mA=response.current_mA,
                temperature_C=response.temperature_C,
                start_ticks=response.start_ticks,
                target_ticks=response.target_ticks,
                aborted_reason="simulated flake",
            )
        return compute_step_metrics(response)

    bounds = SearchBounds(p_min=0, p_max=100, d_min=0, d_max=100, i_min=0, i_max=20)
    result = coordinate_descent_tune(
        initial=Gains(p=40, d=20, i=0),
        bounds=bounds,
        run_trial=flaky_run_trial,
        criteria=_criteria(repeat_trials=3),
        max_trials=10,
        initial_step=4,
    )

    assert not result.validated


def test_unmeasurable_trial_does_not_crash_the_search():
    def exploding_run_trial(gains: Gains):
        if gains.p > 60:
            raise RuntimeError("simulated hardware read failure")
        response = _simulate_step(gains)
        return compute_step_metrics(response)

    bounds = SearchBounds(p_min=0, p_max=100, d_min=0, d_max=0, i_min=0, i_max=0)
    result = coordinate_descent_tune(
        initial=Gains(p=55, d=0, i=0),
        bounds=bounds,
        run_trial=exploding_run_trial,
        criteria=_criteria(),
        max_trials=10,
        initial_step=16,
    )

    # Must complete without raising, and never pick an unmeasurable trial as best
    # over one that was actually measured, when a measured one exists.
    assert result.best.metrics is not None
