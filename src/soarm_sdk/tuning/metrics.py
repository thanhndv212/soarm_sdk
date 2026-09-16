"""Step-response metrics: turn a recorded step into pass/fail-able numbers.

Generalizes the dashboard's original ``_compute_step_metrics`` (rise time,
overshoot, settling time) and adds the fields a search or a safety gate
needs that a human eyeballing a chart didn't: steady-state error, an
oscillation count, and the peak current/temperature seen during the trial.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np

__all__ = ["StepResponse", "StepMetrics", "compute_step_metrics"]


@dataclass(frozen=True)
class StepResponse:
    """Raw samples recorded while commanding one position step.

    ``current_mA`` / ``temperature_C`` entries are ``None`` where the sample
    source didn't have that channel (e.g. a legacy poll loop that only reads
    position) — :func:`compute_step_metrics` treats an all-``None`` column as
    "unknown", not "zero".
    """

    t_s: Tuple[float, ...]
    position_ticks: Tuple[float, ...]
    current_mA: Tuple[Optional[float], ...]
    temperature_C: Tuple[Optional[float], ...]
    start_ticks: float
    target_ticks: float
    #: ``None`` means the trial ran to completion (or was stopped manually,
    #: which is not a safety event). Any other value is why a watchdog cut
    #: it short, and callers should treat the trial as failed regardless of
    #: what the partial metrics say.
    aborted_reason: Optional[str] = None

    def __post_init__(self) -> None:
        n = len(self.t_s)
        if len(self.position_ticks) != n:
            raise ValueError("position_ticks must be the same length as t_s")
        if len(self.current_mA) != n:
            raise ValueError("current_mA must be the same length as t_s")
        if len(self.temperature_C) != n:
            raise ValueError("temperature_C must be the same length as t_s")


@dataclass(frozen=True)
class StepMetrics:
    rise_time_s: Optional[float]
    overshoot_pct: float
    settling_time_s: float
    steady_state_error_ticks: float
    oscillation_count: int
    peak_current_mA: Optional[float]
    peak_temperature_C: Optional[float]
    aborted_reason: Optional[str]


def _peak(values: Sequence[Optional[float]]) -> Optional[float]:
    present = [v for v in values if v is not None]
    return max(present) if present else None


def compute_step_metrics(
    response: StepResponse, *, settle_band_frac: float = 0.02
) -> StepMetrics:
    """Reduce a recorded :class:`StepResponse` to scoreable metrics.

    Raises ``ValueError`` on degenerate input (no step to measure, or too
    few samples to say anything) rather than returning fabricated numbers —
    callers that need a trial to always produce a cost (e.g. a search loop)
    should catch this and score it as a failed trial explicitly.
    """
    delta = response.target_ticks - response.start_ticks
    if abs(delta) < 1e-6:
        raise ValueError("target equals start position — no step to measure")
    if len(response.t_s) < 3:
        raise ValueError("fewer than 3 samples — insufficient for metrics")

    t = np.asarray(response.t_s, dtype=np.float64)
    a = np.asarray(response.position_ticks, dtype=np.float64)

    # Rise time: 10% -> 90% of the way from start to target.
    frac = (a - response.start_ticks) / delta
    rise_hits = np.where(frac >= 0.1)[0]
    rise_lo = int(rise_hits[0]) if rise_hits.size else None
    rise_hits_hi = np.where(frac >= 0.9)[0]
    rise_hi = int(rise_hits_hi[0]) if rise_hits_hi.size else None
    rise_time_s: Optional[float]
    if rise_lo is not None and rise_hi is not None and rise_hi > rise_lo:
        rise_time_s = float(t[rise_hi] - t[rise_lo])
    else:
        rise_time_s = None

    # Overshoot: how far past target, as a % of the step size.
    target = response.target_ticks
    if delta > 0:
        overshoot_pct = max(0.0, float((a.max() - target) / delta * 100.0))
    else:
        overshoot_pct = max(0.0, float((target - a.min()) / delta * 100.0))

    # Settling time: last sample outside +/-settle_band_frac of the step
    # that never returns.
    band = settle_band_frac * abs(delta)
    error = a - response.target_ticks
    outside = np.where(np.abs(error) > band)[0]
    settling_time_s = float(t[outside[-1]]) if outside.size else float(t[0])

    steady_state_error_ticks = float(abs(error[-1]))

    # Oscillation count: sign changes of the error after the step first
    # enters the settle band, i.e. ringing rather than the initial approach.
    first_inside = np.where(np.abs(error) <= band)[0]
    oscillation_count = 0
    if first_inside.size:
        tail_error = error[int(first_inside[0]):]
        signs = np.sign(tail_error)
        nonzero = signs[signs != 0]
        if nonzero.size > 1:
            oscillation_count = int(np.count_nonzero(np.diff(nonzero) != 0))

    peak_current_mA = _peak(response.current_mA)
    peak_temperature_C = _peak(response.temperature_C)

    return StepMetrics(
        rise_time_s=rise_time_s,
        overshoot_pct=overshoot_pct,
        settling_time_s=settling_time_s,
        steady_state_error_ticks=steady_state_error_ticks,
        oscillation_count=oscillation_count,
        peak_current_mA=peak_current_mA,
        peak_temperature_C=peak_temperature_C,
        aborted_reason=response.aborted_reason,
    )


