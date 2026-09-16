"""Safety limits checked on every sample while a step trial is running.

There is no current/temperature guard anywhere in
:class:`~soarm_sdk.robot.hardware.ServoHardwareInterface` today — it only
clamps commanded position and step size. Auto-tuning is exactly the
scenario that finds bad gains by making a servo oscillate or stall against
a load, so this check runs on every sample of every trial (see
:func:`~soarm_sdk.tuning.step_test.run_step_test`'s ``watchdog`` argument),
not just at the end when the numbers are already in.

Checked inline in the same loop that is already polling at the trial's
cadence — a real second thread would add nothing here, since a reaction
within one poll interval is already as fast as this loop can act.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from .step_test import Sample

__all__ = ["WatchdogLimits", "Watchdog", "check_sample"]


@dataclass(frozen=True)
class WatchdogLimits:
    """Hard ceilings, above the acceptance criteria's own thresholds.

    These exist to cut a trial short *during* the trial, not merely to fail
    it afterward — set them above
    :class:`~soarm_sdk.tuning.acceptance.PIDAcceptanceCriteria`'s
    corresponding thresholds so a trial that would fail acceptance still
    completes and reports real numbers, while one that is actually
    dangerous is stopped immediately.
    """

    current_max_mA: float
    temperature_max_C: float
    #: Emergency ceiling on oscillation count, checked as the trial runs —
    #: distinct from (and above) acceptance's oscillation_count_max, which
    #: only judges the finished trial.
    oscillation_hard_max: int

    def __post_init__(self) -> None:
        if self.current_max_mA <= 0:
            raise ValueError("current_max_mA must be positive")
        if self.temperature_max_C <= 0:
            raise ValueError("temperature_max_C must be positive")
        if self.oscillation_hard_max < 0:
            raise ValueError("oscillation_hard_max must be non-negative")


def check_sample(
    sample: Sample,
    limits: WatchdogLimits,
    *,
    sign_changes_so_far: int = 0,
) -> Optional[str]:
    """Return an abort reason if *sample* breaches *limits*, else ``None``.

    Only checks what the sample actually reports: a ``None`` current or
    temperature reading is not itself an abort condition (some sample
    sources don't carry that channel), it just means this call can't judge
    that limit.
    """
    current = sample.current_mA
    if current is not None and current > limits.current_max_mA:
        return f"current {current:.0f}mA exceeds {limits.current_max_mA:.0f}mA"

    temp = sample.temperature_C
    if temp is not None and temp > limits.temperature_max_C:
        return f"temperature {temp:.1f}C exceeds {limits.temperature_max_C:.1f}C"

    if sign_changes_so_far > limits.oscillation_hard_max:
        return (
            f"oscillation count {sign_changes_so_far} exceeds hard limit "
            f"{limits.oscillation_hard_max}"
        )
    return None


class Watchdog:
    """Stateful wrapper binding :func:`check_sample` into a single-arg
    callback, for :func:`~soarm_sdk.tuning.step_test.run_step_test`'s
    ``watchdog=`` parameter.

    Tracks the running oscillation count itself (error sign changes against
    *target_ticks*) so the caller doesn't have to.
    """

    def __init__(self, limits: WatchdogLimits, *, target_ticks: float) -> None:
        self._limits = limits
        self._target_ticks = target_ticks
        self._last_sign = 0
        self._sign_changes = 0

    def __call__(self, sample: Sample) -> Optional[str]:
        error = sample.position_ticks - self._target_ticks
        sign = 1 if error > 0 else (-1 if error < 0 else 0)
        if sign != 0 and self._last_sign != 0 and sign != self._last_sign:
            self._sign_changes += 1
        if sign != 0:
            self._last_sign = sign

        return check_sample(
            sample, self._limits, sign_changes_so_far=self._sign_changes
        )
