"""Black-box coordinate-descent search over (P, D, I) gains.

The onboard STS3215 controller is a fixed-point embedded PID with 1-byte
integer gains and an unknown internal update law — not a plant to model or
single-step, and not classic enough for Ziegler-Nichols relay-feedback math
to transfer cleanly. This treats it as a black box: vary one gain at a time
by a step size, keep the change if the cost improves, halve the step size
on repeated failure to improve (Hooke-Jeeves-style pattern search).

Pure with respect to hardware: everything below talks to a caller-supplied
``run_trial(gains) -> StepMetrics`` callback, never to a bus directly. That
is what makes this testable against a synthetic plant with zero hardware,
and it's the same reusable shape a dashboard, a CLI, or a test can each wire
up differently. See ``docs/pid_autotune_plan.md``.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Callable, List, Optional, Sequence

from .acceptance import PIDAcceptanceCriteria, PIDTuningReport
from .gains_io import Gains
from .metrics import StepMetrics

__all__ = [
    "SearchBounds",
    "TrialResult",
    "SearchResult",
    "cost",
    "coordinate_descent_tune",
]

#: Penalty added to a trial's cost per stage that fails acceptance, on top
#: of the continuous cost terms below — large enough that any passing trial
#: always beats any failing one, so the search moves toward acceptance
#: before it starts optimizing within it.
_FAIL_PENALTY = 1_000.0

#: Cost assigned to a trial whose metrics couldn't be computed at all
#: (aborted before a step could be measured, or a hardware read failed) —
#: worse than any real failing trial, so the search always prefers a trial
#: it could actually measure.
_UNMEASURABLE_COST = 1e9


@dataclass(frozen=True)
class SearchBounds:
    p_min: int
    p_max: int
    d_min: int
    d_max: int
    i_min: int
    i_max: int

    def __post_init__(self) -> None:
        for lo, hi, name in (
            (self.p_min, self.p_max, "p"),
            (self.d_min, self.d_max, "d"),
            (self.i_min, self.i_max, "i"),
        ):
            if lo > hi:
                raise ValueError(f"{name}_min ({lo}) must be <= {name}_max ({hi})")
            if lo < 0 or hi > 254:
                raise ValueError(f"{name} bounds must lie within [0, 254]")

    def clamp(self, gains: Gains) -> Gains:
        return Gains(
            p=max(self.p_min, min(self.p_max, gains.p)),
            d=max(self.d_min, min(self.d_max, gains.d)),
            i=max(self.i_min, min(self.i_max, gains.i)),
        )

    def step(self, gains: Gains, axis: str, delta: int) -> Gains:
        """*gains* with *axis* shifted by *delta*, clamped to this range.

        Clamps the raw integer before constructing :class:`Gains`, not
        after — ``Gains.__post_init__`` rejects out-of-[0, 254] values
        outright, so a step that would land at e.g. ``p=-12`` must never
        reach the constructor unclamped.
        """
        lo, hi = {
            "p": (self.p_min, self.p_max),
            "d": (self.d_min, self.d_max),
            "i": (self.i_min, self.i_max),
        }[axis]
        raw = getattr(gains, axis) + delta
        clamped = max(lo, min(hi, raw))
        return replace(gains, **{axis: clamped})


@dataclass(frozen=True)
class TrialResult:
    gains: Gains
    metrics: Optional[StepMetrics]
    report: Optional[PIDTuningReport]
    cost: float


@dataclass(frozen=True)
class SearchResult:
    best: TrialResult
    trials: Sequence[TrialResult]
    consecutive_passes_at_best: int
    validated: bool
    #: True when the trial budget ran out before ``validated`` could be set.
    exhausted: bool


def cost(metrics: StepMetrics, criteria: PIDAcceptanceCriteria) -> float:
    """Lower is better. Continuous terms plus a hard penalty per failed stage.

    The continuous terms are normalized by the criteria's own thresholds so
    no single metric dominates just because its units are numerically
    larger (settling time in seconds vs. current in milliamps).
    """
    report = criteria.evaluate(metrics)
    if metrics.aborted_reason is not None:
        return _FAIL_PENALTY * len(report.stages)

    penalty = _FAIL_PENALTY * sum(1 for stage in report.stages if not stage.passed)

    rise_term = metrics.rise_time_s if metrics.rise_time_s is not None else (
        criteria.settling_time_max_s * 2.0
    )
    continuous = (
        rise_term / max(criteria.settling_time_max_s, 1e-9)
        + metrics.overshoot_pct / max(criteria.overshoot_max_pct, 1e-9)
        + metrics.settling_time_s / max(criteria.settling_time_max_s, 1e-9)
        + metrics.steady_state_error_ticks
        / max(criteria.steady_state_error_max_ticks, 1e-9)
        + metrics.oscillation_count / max(criteria.oscillation_count_max, 1)
    )
    return penalty + continuous


def _run_and_score(
    gains: Gains,
    run_trial: Callable[[Gains], StepMetrics],
    criteria: PIDAcceptanceCriteria,
) -> TrialResult:
    try:
        metrics = run_trial(gains)
    except Exception:
        return TrialResult(gains=gains, metrics=None, report=None, cost=_UNMEASURABLE_COST)
    report = criteria.evaluate(metrics)
    return TrialResult(gains=gains, metrics=metrics, report=report, cost=cost(metrics, criteria))


def coordinate_descent_tune(
    *,
    initial: Gains,
    bounds: SearchBounds,
    run_trial: Callable[[Gains], StepMetrics],
    criteria: PIDAcceptanceCriteria,
    max_trials: int = 30,
    initial_step: int = 16,
    min_step: int = 1,
) -> SearchResult:
    """Search (P, D, I) for the lowest-cost gains within *bounds*, then
    verify the winner's repeatability.

    Two phases, both spending from the same *max_trials* budget:

    1. **Converge**: starting from *initial* (read the servo's current
       gains, don't assume factory defaults — it may already be
       hand-tuned), vary one gain at a time by *initial_step*, keep any
       change that lowers :func:`cost`, halve the step on a round with no
       improvement, stop when the step can't shrink below *min_step* and
       still hasn't improved. Never proposes gains outside *bounds*, which
       should be conservative relative to the register's full 0-254 range
       on real hardware.
    2. **Verify**: re-run the converged gains up to
       ``criteria.repeat_trials`` more times. ``validated=True`` only if
       every one of those repeats independently passes acceptance — one
       lucky trial during convergence is not enough, since the point of
       repeating at a fixed setting is to catch noise the search's own
       trial can't.

    ``exhausted=True`` means the trial budget ran out before verification
    completed; the caller should treat ``best`` as a candidate, not as
    something to write to EEPROM as final.
    """
    trials: List[TrialResult] = []
    axes = ("p", "d", "i")

    def _budget_left() -> int:
        return max_trials - len(trials)

    # -- phase 1: converge by cost -----------------------------------
    best = _run_and_score(bounds.clamp(initial), run_trial, criteria)
    trials.append(best)

    step = initial_step
    while _budget_left() > 0:
        improved_this_round = False

        for axis in axes:
            if _budget_left() <= 0:
                break
            for direction in (+1, -1):
                if _budget_left() <= 0:
                    break
                candidate = bounds.step(best.gains, axis, direction * step)
                if candidate == best.gains:
                    continue  # clamped back to the same point; nothing to learn

                result = _run_and_score(candidate, run_trial, criteria)
                trials.append(result)

                if result.cost < best.cost:
                    best = result
                    improved_this_round = True
                    break  # move on to re-searching this axis from the new best

        if not improved_this_round:
            if step <= min_step:
                break
            step = max(min_step, step // 2)

    # -- phase 2: verify repeatability at the winning point ------------
    consecutive_passes = 1 if best.report is not None and best.report.ready else 0
    while (
        consecutive_passes > 0
        and consecutive_passes < criteria.repeat_trials
        and _budget_left() > 0
    ):
        repeat = _run_and_score(best.gains, run_trial, criteria)
        trials.append(repeat)
        if repeat.report is not None and repeat.report.ready:
            consecutive_passes += 1
        else:
            consecutive_passes = 0
            break  # a repeat failed: the winner is not actually reliable

    validated = consecutive_passes >= criteria.repeat_trials
    exhausted = not validated

    return SearchResult(
        best=best,
        trials=tuple(trials),
        consecutive_passes_at_best=consecutive_passes,
        validated=validated,
        exhausted=exhausted,
    )
