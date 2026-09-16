"""PID auto-tuning: step-test metrics, acceptance gates, search, safety.

See ``docs/pid_autotune_plan.md`` for the design this module implements.
"""

from __future__ import annotations

from .acceptance import PIDAcceptanceCriteria, PIDTuningReport, StageResult, TuningStage
from .gains_io import Gains, read_gains, write_gains
from .metrics import StepMetrics, StepResponse, compute_step_metrics
from .provenance import TuningRecord, append_record, load_records
from .search import SearchBounds, SearchResult, TrialResult, coordinate_descent_tune
from .step_test import Sample, run_step_test
from .watchdog import Watchdog, WatchdogLimits, check_sample

__all__ = [
    "PIDAcceptanceCriteria",
    "PIDTuningReport",
    "StageResult",
    "TuningStage",
    "Gains",
    "read_gains",
    "write_gains",
    "StepMetrics",
    "StepResponse",
    "compute_step_metrics",
    "TuningRecord",
    "append_record",
    "load_records",
    "SearchBounds",
    "SearchResult",
    "TrialResult",
    "coordinate_descent_tune",
    "Sample",
    "run_step_test",
    "Watchdog",
    "WatchdogLimits",
    "check_sample",
]
