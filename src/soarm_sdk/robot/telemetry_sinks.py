"""Consumers for a :class:`~soarm_sdk.robot.telemetry.TelemetryStream`.

Everything here runs on :class:`TelemetryRecorder`'s own thread, never on the
bus thread. That separation is the point: the bus thread also writes to the
servos, so a sink that blocks on a file, a socket, or a viewer must not be
able to delay a command to the arm.
"""

from __future__ import annotations

import json
import logging
import threading
from pathlib import Path
from typing import Any, Callable, Iterable, List, Optional, Sequence

from .telemetry import DEFAULT_BUFFER_SAMPLES, ServoSample, TelemetryStream

logger = logging.getLogger(__name__)

__all__ = ["TelemetrySink", "JsonlSink", "RerunSink", "TelemetryRecorder"]


class TelemetrySink:
    """Base class for sinks. Subclasses override :meth:`write`."""

    def write(self, sample: ServoSample) -> None:  # pragma: no cover - abstract
        raise NotImplementedError

    def flush(self) -> None:
        """Optional: push buffered data onward."""

    def close(self) -> None:
        """Optional: release resources. Always called by the recorder."""


class JsonlSink(TelemetrySink):
    """Append one JSON object per sample to a file.

    Same shape as ``soarm_tamp``'s ``live.jsonl``, so a file is again the whole
    cross-process channel: a viewer, a plot, or an analysis script tails it
    without ever touching the serial port.

    Writes are buffered and flushed every *flush_every* samples — flushing each
    line at 100 Hz is a syscall per tick for no benefit, and :meth:`close`
    flushes regardless.
    """

    def __init__(self, path, *, flush_every: int = 100) -> None:
        if flush_every < 1:
            raise ValueError("flush_every must be at least 1")
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = self.path.open("a", encoding="utf-8")
        self._flush_every = flush_every
        self._since_flush = 0
        self.written = 0

    def write(self, sample: ServoSample) -> None:
        self._fh.write(json.dumps(sample.as_dict()) + "\n")
        self.written += 1
        self._since_flush += 1
        if self._since_flush >= self._flush_every:
            self.flush()

    def flush(self) -> None:
        self._fh.flush()
        self._since_flush = 0

    def close(self) -> None:
        if not self._fh.closed:
            self.flush()
            self._fh.close()


class RerunSink(TelemetrySink):
    """Log each sample to rerun as per-joint scalar series.

    ``rerun-sdk`` is not a dependency of this package; the import is deferred
    to construction, so nothing pays for it unless a caller asks for this sink.
    Pass *rr_module* to inject a double.

    The rerun scalar and timeline APIs have been spelled several ways across
    versions (``Scalar`` vs ``Scalars``; ``set_time_seconds`` vs ``set_time``),
    so both are probed once at construction and the working one is bound. If no
    timeline setter is found, samples are logged in arrival order without an
    explicit timestamp.
    """

    def __init__(
        self,
        *,
        prefix: str = "soarm",
        joint_names: Optional[Sequence[str]] = None,
        timeline: str = "bus",
        rr_module: Any = None,
    ) -> None:
        if rr_module is None:
            try:
                import rerun as rr_module  # type: ignore[no-redef]
            except ImportError as exc:  # pragma: no cover - depends on env
                raise ImportError(
                    "RerunSink needs rerun-sdk: pip install 'soarm-sdk[telemetry]'"
                ) from exc
        self._rr = rr_module
        self._prefix = prefix.rstrip("/")
        self._joint_names = list(joint_names) if joint_names else None
        self._timeline = timeline
        self._scalar = self._resolve_scalar()
        self._set_time = self._resolve_set_time()

    def _resolve_scalar(self) -> Callable[[float], Any]:
        for name in ("Scalars", "Scalar"):
            builder = getattr(self._rr, name, None)
            if builder is not None:
                return builder
        raise AttributeError(
            "rerun module exposes neither Scalars nor Scalar; "
            "cannot log scalar series"
        )

    def _resolve_set_time(self) -> Optional[Callable[[float], None]]:
        legacy = getattr(self._rr, "set_time_seconds", None)
        if legacy is not None:
            return lambda t: legacy(self._timeline, t)
        modern = getattr(self._rr, "set_time", None)
        if modern is not None:
            return lambda t: modern(self._timeline, duration=t)
        logger.warning(
            "rerun exposes no timeline setter; telemetry will be logged in "
            "arrival order without timestamps"
        )
        return None

    def _entity(self, index: int, servo_id: int, field: str) -> str:
        if self._joint_names is not None and index < len(self._joint_names):
            label = self._joint_names[index]
        else:
            label = f"id_{servo_id}"
        return f"{self._prefix}/{label}/{field}"

    def write(self, sample: ServoSample) -> None:
        if self._set_time is not None:
            self._set_time(sample.t_mono)

        error = sample.tracking_error_rad()
        for i, servo_id in enumerate(sample.ids):
            log = self._rr.log
            log(self._entity(i, servo_id, "position_rad"),
                self._scalar(sample.position_rad[i]))
            log(self._entity(i, servo_id, "velocity_rad_s"),
                self._scalar(sample.velocity_rad_s[i]))
            log(self._entity(i, servo_id, "load_percent"),
                self._scalar(sample.load_percent[i]))
            log(self._entity(i, servo_id, "current_mA"),
                self._scalar(sample.current_mA[i]))
            log(self._entity(i, servo_id, "temperature_C"),
                self._scalar(float(sample.temperature_C[i])))
            log(self._entity(i, servo_id, "voltage_V"),
                self._scalar(sample.voltage_V[i]))
            if sample.goal_position_rad is not None:
                log(self._entity(i, servo_id, "goal_position_rad"),
                    self._scalar(sample.goal_position_rad[i]))
            if error is not None:
                log(self._entity(i, servo_id, "tracking_error_rad"),
                    self._scalar(error[i]))


class TelemetryRecorder:
    """Drain an interface's telemetry on a thread and fan it out to sinks.

    ::

        with TelemetryRecorder(hw, [JsonlSink("run/telemetry.jsonl")]):
            ...  # drive the arm

    A sink that raises is logged and **dropped** rather than retried: one
    broken sink must not cost the others their data, and a sink raising every
    tick would otherwise fill the log at the bus rate.
    """

    def __init__(
        self,
        interface: Any,
        sinks: Iterable[TelemetrySink],
        *,
        maxlen: int = DEFAULT_BUFFER_SAMPLES,
        poll_timeout: float = 0.5,
    ) -> None:
        self._interface = interface
        self._sinks: List[TelemetrySink] = list(sinks)
        self._maxlen = maxlen
        self._poll_timeout = poll_timeout
        self._stream: Optional[TelemetryStream] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self.samples_written = 0
        self.failed_sinks: List[TelemetrySink] = []

    @property
    def dropped(self) -> int:
        """Samples lost because this recorder fell behind the bus thread."""
        return self._stream.dropped if self._stream is not None else 0

    def start(self) -> None:
        if self._running:
            return
        self._stream = self._interface.subscribe(maxlen=self._maxlen)
        self._running = True
        self._thread = threading.Thread(
            target=self._loop, name="TelemetryRecorder", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        """Stop the thread, drain what is left, and close every sink."""
        if not self._running:
            return
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._stream is not None:
            self._drain_once()  # whatever arrived during shutdown
            self._interface.unsubscribe(self._stream)
            self._stream = None
        for sink in self._sinks:
            try:
                sink.close()
            except Exception:
                logger.exception("telemetry sink failed to close")

    def _loop(self) -> None:
        while self._running:
            assert self._stream is not None
            if self._stream.wait(timeout=self._poll_timeout):
                self._drain_once()

    def _drain_once(self) -> None:
        if self._stream is None:
            return
        for sample in self._stream.drain():
            for sink in list(self._sinks):
                try:
                    sink.write(sample)
                except Exception:
                    logger.exception(
                        "telemetry sink %r raised; dropping it", sink
                    )
                    self._sinks.remove(sink)
                    self.failed_sinks.append(sink)
            self.samples_written += 1

    def __enter__(self) -> "TelemetryRecorder":
        self.start()
        return self

    def __exit__(self, *_) -> None:
        self.stop()
