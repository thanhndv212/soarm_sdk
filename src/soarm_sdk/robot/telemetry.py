"""Telemetry tap for the servo bus thread.

Why this lives here and not in a service of its own
---------------------------------------------------
The serial port is exclusive: one process owns it at a time. A standalone
telemetry process cannot read the bus while teleop, a TAMP execution, or a
policy is driving it — which is exactly when telemetry is worth having. So
the producer has to be whoever already owns the bus
(:class:`~soarm_sdk.robot.hardware.ServoHardwareInterface`), and consumers
subscribe out-of-band.

The contract
------------
* The bus thread **never calls consumer code**. It appends a
  :class:`ServoSample` to each subscribed :class:`TelemetryStream`'s bounded
  deque and returns. Consumers drain on their own thread. That thread also
  issues servo *writes*, so a slow or blocking consumer there would delay
  commands to the arm — a safety property, not a performance one.
* Overflow **drops the oldest sample** and counts the drop. A consumer that
  cannot keep up loses history, never liveness.
* ``seq`` counts bus ticks, not published samples. A gap means a read failed
  and nothing was published for that tick, so a consumer can always tell "the
  arm did not move" from "we missed the sample".
* Every sample carries the *commanded* state alongside the measured state,
  stamped with one timestamp taken next to the wire. Tracking error is then a
  subtraction rather than a join across two logs with different clocks.
"""

from __future__ import annotations

import threading
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional, Tuple

__all__ = ["ServoSample", "TelemetryStream"]

#: Samples retained by default. At 100 Hz this is ~100 s of history.
DEFAULT_BUFFER_SAMPLES = 10_000


@dataclass(frozen=True)
class ServoSample:
    """One bus tick: what every servo reported, and what it had been told.

    Tuples rather than arrays keep construction cheap on the bus thread and
    the record immutable once published; convert to numpy in the consumer.

    The ``goal_*`` fields are the command **in force when the measurement was
    taken** — the last write actually put on the wire, which happens after the
    read within a tick, so it is the previous tick's command. They are ``None``
    until the first write, and stay ``None`` for a passive (read-only) session.
    """

    t_mono: float
    """``time.monotonic()`` taken immediately after the sync-read returned."""

    seq: int
    """Bus-tick counter. Gaps mark ticks whose read failed."""

    ids: Tuple[int, ...]

    # -- measured ----------------------------------------------------------
    position_ticks: Tuple[int, ...]
    position_rad: Tuple[float, ...]
    velocity_ticks: Tuple[int, ...]
    velocity_rad_s: Tuple[float, ...]
    load_percent: Tuple[float, ...]
    current_mA: Tuple[float, ...]
    voltage_V: Tuple[float, ...]
    temperature_C: Tuple[int, ...]
    status_flags: Tuple[int, ...]
    moving: Tuple[bool, ...]

    # -- commanded ---------------------------------------------------------
    goal_position_ticks: Optional[Tuple[int, ...]] = None
    goal_position_rad: Optional[Tuple[float, ...]] = None
    goal_speed_ticks: Optional[Tuple[int, ...]] = None
    goal_t_mono: Optional[float] = None

    read_errors: int = 0
    """Cumulative failed reads on this interface, for corroborating a gap."""

    def tracking_error_rad(self) -> Optional[Tuple[float, ...]]:
        """Commanded minus measured, per joint, or ``None`` before any write."""
        if self.goal_position_rad is None:
            return None
        return tuple(
            g - m for g, m in zip(self.goal_position_rad, self.position_rad)
        )

    def as_dict(self) -> Dict:
        """A JSON-serialisable view, for file sinks."""
        out = {
            "t_mono": self.t_mono,
            "seq": self.seq,
            "ids": list(self.ids),
            "position_ticks": list(self.position_ticks),
            "position_rad": list(self.position_rad),
            "velocity_ticks": list(self.velocity_ticks),
            "velocity_rad_s": list(self.velocity_rad_s),
            "load_percent": list(self.load_percent),
            "current_mA": list(self.current_mA),
            "voltage_V": list(self.voltage_V),
            "temperature_C": list(self.temperature_C),
            "status_flags": list(self.status_flags),
            "moving": list(self.moving),
            "read_errors": self.read_errors,
        }
        if self.goal_position_ticks is not None:
            out["goal_position_ticks"] = list(self.goal_position_ticks)
            out["goal_position_rad"] = list(self.goal_position_rad or ())
            out["goal_speed_ticks"] = list(self.goal_speed_ticks or ())
            out["goal_t_mono"] = self.goal_t_mono
        return out


@dataclass
class TelemetryStream:
    """A bounded, drop-oldest queue of :class:`ServoSample`, one per consumer.

    Obtain one from
    :meth:`~soarm_sdk.robot.hardware.ServoHardwareInterface.subscribe`; do not
    construct directly. Safe to drain from any thread.

    ::

        stream = hw.subscribe()
        while running:
            if stream.wait(timeout=0.5):
                for sample in stream.drain():
                    ...
        hw.unsubscribe(stream)
    """

    maxlen: int = DEFAULT_BUFFER_SAMPLES
    _buf: Deque[ServoSample] = field(init=False, repr=False)
    _lock: threading.Lock = field(init=False, repr=False)
    _event: threading.Event = field(init=False, repr=False)
    _dropped: int = field(default=0, init=False)
    _published: int = field(default=0, init=False)
    _closed: bool = field(default=False, init=False)

    def __post_init__(self) -> None:
        if self.maxlen < 1:
            raise ValueError("maxlen must be at least 1")
        self._buf = deque(maxlen=self.maxlen)
        self._lock = threading.Lock()
        self._event = threading.Event()

    # -- producer side (bus thread) ---------------------------------------

    def _publish(self, sample: ServoSample) -> None:
        """Append one sample. Never blocks, never raises for the producer."""
        with self._lock:
            if self._closed:
                return
            if len(self._buf) == self._buf.maxlen:
                self._dropped += 1
            self._buf.append(sample)
            self._published += 1
        self._event.set()

    # -- consumer side -----------------------------------------------------

    def drain(self) -> List[ServoSample]:
        """Remove and return everything buffered, oldest first."""
        with self._lock:
            out = list(self._buf)
            self._buf.clear()
            self._event.clear()
        return out

    def wait(self, timeout: Optional[float] = None) -> bool:
        """Block until at least one sample is buffered, or *timeout* elapses."""
        return self._event.wait(timeout)

    def close(self) -> None:
        """Stop accepting samples and release anyone waiting."""
        with self._lock:
            self._closed = True
            self._buf.clear()
        self._event.set()

    @property
    def closed(self) -> bool:
        return self._closed

    @property
    def dropped(self) -> int:
        """Samples discarded because the consumer fell behind."""
        return self._dropped

    @property
    def published(self) -> int:
        """Samples accepted since subscription, drops included."""
        return self._published

    def __len__(self) -> int:
        with self._lock:
            return len(self._buf)
