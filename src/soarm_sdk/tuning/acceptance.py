"""Fail-closed acceptance checks for a PID tuning trial.

Mirrors :mod:`soarm_sdk.calibration.pipeline`'s ``AcceptanceTolerances`` /
``CalibrationReport`` shape deliberately: the same "explicit thresholds,
per-stage pass/fail, `ready` only when every stage passes" pattern the arm's
zero calibration already uses, so a tuned joint's gains are held to the same
kind of evidence bar as its zero — see ``docs/pid_autotune_plan.md``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Sequence

from .metrics import StepMetrics

__all__ = [
    "PIDAcceptanceCriteria",
    "PIDTuningReport",
    "StageResult",
    "TuningStage",
]


class TuningStage(str, Enum):
    ABORTED = "aborted"
    OVERSHOOT = "overshoot"
    SETTLING_TIME = "settling_time"
    STEADY_STATE_ERROR = "steady_state_error"
    OSCILLATION = "oscillation"
    CURRENT = "current"
    TEMPERATURE = "temperature"


@dataclass(frozen=True)
class PIDAcceptanceCriteria:
    """Explicit, per-joint thresholds a trial's :class:`StepMetrics` must meet."""

    overshoot_max_pct: float
    settling_time_max_s: float
    steady_state_error_max_ticks: float
    oscillation_count_max: int
    current_peak_max_mA: float
    temperature_max_C: float
    #: Consecutive passing trials required before a search may mark gains
    #: ``validated`` — one good run can be noise, same reasoning as the
    #: calibration pipeline's reference-pose repeatability check.
    repeat_trials: int = 3

    def __post_init__(self) -> None:
        for name in (
            "overshoot_max_pct",
            "settling_time_max_s",
            "steady_state_error_max_ticks",
            "current_peak_max_mA",
            "temperature_max_C",
        ):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if (
            isinstance(self.oscillation_count_max, bool)
            or not isinstance(self.oscillation_count_max, int)
            or self.oscillation_count_max < 0
        ):
            raise ValueError("oscillation_count_max must be a non-negative integer")
        if (
            isinstance(self.repeat_trials, bool)
            or not isinstance(self.repeat_trials, int)
            or self.repeat_trials < 1
        ):
            raise ValueError("repeat_trials must be a positive integer")

    def evaluate(self, metrics: StepMetrics) -> "PIDTuningReport":
        """Score one trial's metrics against every stage."""
        stages = []

        if metrics.aborted_reason is not None:
            stages.append(
                StageResult(TuningStage.ABORTED, False, metrics.aborted_reason)
            )
            # An aborted trial's other numbers are partial by construction
            # (the watchdog cut the trial short) — reporting them as passes
            # would be misleading, so every remaining stage is BLOCKED too.
            for stage in (
                TuningStage.OVERSHOOT,
                TuningStage.SETTLING_TIME,
                TuningStage.STEADY_STATE_ERROR,
                TuningStage.OSCILLATION,
                TuningStage.CURRENT,
                TuningStage.TEMPERATURE,
            ):
                stages.append(StageResult(stage, False, "trial aborted"))
            return PIDTuningReport(tuple(stages))

        stages.append(StageResult(TuningStage.ABORTED, True, "trial completed"))

        passed = metrics.overshoot_pct <= self.overshoot_max_pct
        stages.append(
            StageResult(
                TuningStage.OVERSHOOT,
                passed,
                f"{metrics.overshoot_pct:.1f}% <= {self.overshoot_max_pct:.1f}%"
                if passed
                else f"{metrics.overshoot_pct:.1f}% exceeds {self.overshoot_max_pct:.1f}%",
            )
        )

        passed = metrics.settling_time_s <= self.settling_time_max_s
        stages.append(
            StageResult(
                TuningStage.SETTLING_TIME,
                passed,
                f"{metrics.settling_time_s:.3f}s <= {self.settling_time_max_s:.3f}s"
                if passed
                else f"{metrics.settling_time_s:.3f}s exceeds {self.settling_time_max_s:.3f}s",
            )
        )

        passed = metrics.steady_state_error_ticks <= self.steady_state_error_max_ticks
        stages.append(
            StageResult(
                TuningStage.STEADY_STATE_ERROR,
                passed,
                f"{metrics.steady_state_error_ticks:.1f} ticks "
                + ("<=" if passed else "exceeds")
                + f" {self.steady_state_error_max_ticks:.1f} ticks",
            )
        )

        passed = metrics.oscillation_count <= self.oscillation_count_max
        stages.append(
            StageResult(
                TuningStage.OSCILLATION,
                passed,
                f"{metrics.oscillation_count} <= {self.oscillation_count_max}"
                if passed
                else f"{metrics.oscillation_count} exceeds {self.oscillation_count_max}",
            )
        )

        if metrics.peak_current_mA is None:
            stages.append(
                StageResult(TuningStage.CURRENT, False, "no current reading available")
            )
        else:
            passed = metrics.peak_current_mA <= self.current_peak_max_mA
            stages.append(
                StageResult(
                    TuningStage.CURRENT,
                    passed,
                    f"{metrics.peak_current_mA:.0f}mA <= {self.current_peak_max_mA:.0f}mA"
                    if passed
                    else f"{metrics.peak_current_mA:.0f}mA exceeds {self.current_peak_max_mA:.0f}mA",
                )
            )

        if metrics.peak_temperature_C is None:
            stages.append(
                StageResult(TuningStage.TEMPERATURE, False, "no temperature reading available")
            )
        else:
            passed = metrics.peak_temperature_C <= self.temperature_max_C
            stages.append(
                StageResult(
                    TuningStage.TEMPERATURE,
                    passed,
                    f"{metrics.peak_temperature_C:.1f}C <= {self.temperature_max_C:.1f}C"
                    if passed
                    else f"{metrics.peak_temperature_C:.1f}C exceeds {self.temperature_max_C:.1f}C",
                )
            )

        return PIDTuningReport(tuple(stages))


@dataclass(frozen=True)
class StageResult:
    stage: TuningStage
    passed: bool
    detail: str


@dataclass(frozen=True)
class PIDTuningReport:
    stages: Sequence[StageResult]

    @property
    def ready(self) -> bool:
        return all(stage.passed for stage in self.stages)

    def stage(self, stage: TuningStage) -> StageResult:
        return next(result for result in self.stages if result.stage == stage)

    def as_markdown(self) -> str:
        lines = ["| Check | Status | Detail |", "|---|---|---|"]
        for result in self.stages:
            status = "PASS" if result.passed else "FAIL"
            lines.append(f"| {result.stage.value} | {status} | {result.detail} |")
        return "\n".join(lines)
