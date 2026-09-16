"""Run one commanded position step and record the response.

Deliberately hardware-agnostic: the caller supplies a ``command`` (send the
step) and a ``read_sample`` (pull the next measurement), so this loop is the
same whether the sample comes from a
:class:`~soarm_sdk.robot.telemetry.TelemetryStream` (streaming dashboard
mode) or a direct register read (legacy poll mode) or, in a test, a canned
sequence. That is also what lets a safety watchdog and a search loop reuse
this one control loop instead of each growing their own copy.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, Optional

from .metrics import StepResponse

__all__ = ["Sample", "run_step_test"]


@dataclass(frozen=True)
class Sample:
    """One measurement pulled by :func:`run_step_test`.

    ``current_mA`` / ``temperature_C`` are optional so a caller without that
    channel (a legacy position-only poll) can still run a step test — the
    resulting metrics just won't have a safety-relevant peak reading.
    """

    t_s: float
    position_ticks: float
    current_mA: Optional[float] = None
    temperature_C: Optional[float] = None


def run_step_test(
    *,
    command: Callable[[], None],
    read_sample: Callable[[], Optional[Sample]],
    start_ticks: float,
    target_ticks: float,
    duration_s: float,
    poll_interval_s: float = 0.05,
    watchdog: Optional[Callable[[Sample], Optional[str]]] = None,
    should_stop: Optional[Callable[[], bool]] = None,
    on_sample: Optional[Callable[[StepResponse], None]] = None,
) -> StepResponse:
    """Send *command* once, then poll *read_sample* for up to *duration_s*.

    *watchdog*, when given, is called with every sample; a non-``None``
    return value is taken as an abort reason and ends the trial immediately
    (recorded on the returned :class:`StepResponse`, not raised), so a
    caller always gets back whatever was measured up to the abort point.

    *should_stop* lets an interactive caller (a dashboard "Stop" button)
    end the trial early without that being treated as an abort.

    *on_sample* is called after every accepted sample with the
    :class:`StepResponse` built so far, for a live chart to redraw from —
    it must not block, since it runs on this loop's own thread.
    """
    t_samples: list = []
    pos_samples: list = []
    current_samples: list = []
    temp_samples: list = []
    aborted_reason: Optional[str] = None

    command()
    t0 = time.monotonic()

    while True:
        t_loop = time.monotonic()
        now = t_loop - t0

        sample = read_sample()
        if sample is not None:
            t_samples.append(sample.t_s)
            pos_samples.append(sample.position_ticks)
            current_samples.append(sample.current_mA)
            temp_samples.append(sample.temperature_C)

            if on_sample is not None:
                on_sample(
                    StepResponse(
                        t_s=tuple(t_samples),
                        position_ticks=tuple(pos_samples),
                        current_mA=tuple(current_samples),
                        temperature_C=tuple(temp_samples),
                        start_ticks=start_ticks,
                        target_ticks=target_ticks,
                    )
                )

            if watchdog is not None:
                aborted_reason = watchdog(sample)
                if aborted_reason is not None:
                    break

        if should_stop is not None and should_stop():
            break
        if now >= duration_s:
            break

        sleep_s = poll_interval_s - (time.monotonic() - t_loop)
        if sleep_s > 0:
            time.sleep(sleep_s)

    return StepResponse(
        t_s=tuple(t_samples),
        position_ticks=tuple(pos_samples),
        current_mA=tuple(current_samples),
        temperature_C=tuple(temp_samples),
        start_ticks=start_ticks,
        target_ticks=target_ticks,
        aborted_reason=aborted_reason,
    )
