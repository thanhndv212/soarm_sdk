"""Shared connection/polling state for soarm_sdk dashboard panels.

Every :class:`~soarm_sdk.dashboard.app.Panel` receives the same
:class:`DashboardContext` instance, so a new panel never has to
reimplement "open the bus," "parse an ID range," or "poll the servos in
the background" — it just reads ``ctx.state`` or calls ``ctx.bus()``.
"""

from __future__ import annotations

import threading
import time
from contextlib import contextmanager
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Generator, List, Optional

from ..bus import parse_range
from ..port_handler import PortHandler
from ..stservo_def import (
    COMM_SUCCESS,
    STS_PRESENT_POSITION_L,
    STS_PRESENT_SPEED_L,
)
from ..sts import sts as _Sts

__all__ = ["JointState", "DashboardContext"]

_HEALTH_EVERY = 5  # read temp/current every Nth poll iteration

SampleHook = Callable[[int, Dict[int, int], Dict[int, int]], None]
"""Called from the poll thread after each successful poll, with
``(poll_count, positions, speeds)`` — see :meth:`DashboardContext.add_sample_hook`."""


@dataclass
class JointState:
    """Latest polled state for every joint, shared across all panels."""

    positions: Dict[int, int] = field(default_factory=dict)
    speeds: Dict[int, int] = field(default_factory=dict)
    temps: Dict[int, Optional[float]] = field(default_factory=dict)
    currents: Dict[int, Optional[float]] = field(default_factory=dict)
    connected: bool = False
    poll_error: Optional[str] = None
    poll_count: int = 0


class DashboardContext:
    """Shared bus/connection/polling state, passed to every registered panel.

    Owns the sidebar GUI handles (device/baud/interval/connection status —
    built once by :class:`~soarm_sdk.dashboard.app.DashboardApp`, shared by
    every tab) plus the background polling thread and its results.
    """

    def __init__(
        self,
        device_h: Any,
        baud_h: Any,
        interval_h: Any,
        conn_status_md: Any,
        joint_ids: List[int],
    ) -> None:
        self.device_h = device_h
        self.baud_h = baud_h
        self.interval_h = interval_h
        self.conn_status_md = conn_status_md
        self.joint_ids = joint_ids

        self.lock = threading.Lock()
        self.state = JointState()

        self._bus_lock = threading.Lock()
        self.stop_event = threading.Event()
        self._poll_thread: Optional[threading.Thread] = None
        self._sample_hooks: List[SampleHook] = []

    # ------------------------------------------------------------------
    # Bus access
    # ------------------------------------------------------------------

    @contextmanager
    def bus(
        self, device: Optional[str] = None, baud: Optional[int] = None
    ) -> Generator[_Sts, None, None]:
        """Open the servo bus exclusively, yield an ``sts`` instance, close on exit.

        Defaults to the sidebar's device/baud fields when not given explicitly.
        """
        dev = device if device is not None else self.device_h.value
        bd = baud if baud is not None else int(self.baud_h.value)
        with self._bus_lock:
            ph = PortHandler(dev)
            try:
                if not ph.openPort():
                    raise RuntimeError(f"Cannot open port {dev!r}")
                if not ph.setBaudRate(bd):
                    raise RuntimeError(f"Cannot set baud rate {bd}")
                yield _Sts(ph)
            finally:
                ph.closePort()

    @staticmethod
    def parse_ids(raw: str) -> List[int]:
        """Parse a ``"1-6"`` / ``"1,3,5"`` / ``"1-3,6"`` style ID range string."""
        ids: set = set()
        for token in [
            s.strip() for s in raw.replace(" ", ",").split(",") if s.strip()
        ]:
            if "-" in token:
                for v in parse_range(token):
                    ids.add(int(v))
            else:
                ids.add(int(token, 0))
        return sorted(ids)

    # ------------------------------------------------------------------
    # Background polling
    # ------------------------------------------------------------------

    def add_sample_hook(self, fn: SampleHook) -> None:
        """Register *fn* to be called after every successful poll iteration.

        Runs on the poll thread, so hooks must be data-only (e.g. append to
        a buffer) — never touch Viser GUI handles from here; use
        ``Panel.on_tick`` (main thread) for that instead.
        """
        self._sample_hooks.append(fn)

    @property
    def polling(self) -> bool:
        return self._poll_thread is not None and self._poll_thread.is_alive()

    def start_polling(self) -> None:
        """Stop any existing poll thread and start a fresh one."""
        self.stop_polling()
        self.stop_event.clear()
        interval_s = float(self.interval_h.value) / 1000.0
        device = self.device_h.value
        baud = int(self.baud_h.value)
        t = threading.Thread(
            target=self._poll_loop,
            args=(device, baud, interval_s),
            daemon=True,
        )
        t.start()
        self._poll_thread = t
        self.conn_status_md.content = f"**Polling** `{device}` @ {baud} baud"

    def stop_polling(self) -> None:
        """Stop the background poll thread, if running."""
        self.stop_event.set()
        if self._poll_thread is not None and self._poll_thread.is_alive():
            self._poll_thread.join(timeout=3.0)
        self._poll_thread = None
        with self.lock:
            self.state.connected = False
        self.conn_status_md.content = "*Disconnected.*"

    def _poll_loop(self, device: str, baud: int, interval_s: float) -> None:
        poll_count = 0
        while not self.stop_event.is_set():
            t_start = time.monotonic()
            try:
                with self.bus(device, baud) as srv:
                    gsr = srv.groupSyncRead
                    gsr.clearParam()
                    for sid in self.joint_ids:
                        gsr.addParam(sid)
                    sync_result = gsr.txRxPacket()

                    new_pos: Dict[int, int] = {}
                    new_spd: Dict[int, int] = {}
                    new_temp: Dict[int, Optional[float]] = {}
                    new_curr: Dict[int, Optional[float]] = {}

                    for sid in self.joint_ids:
                        if sync_result == COMM_SUCCESS:
                            avail, _ = gsr.isAvailable(
                                sid, STS_PRESENT_POSITION_L, 2
                            )
                            if avail:
                                raw_pos = gsr.getData(
                                    sid, STS_PRESENT_POSITION_L, 2
                                )
                                raw_spd = gsr.getData(
                                    sid, STS_PRESENT_SPEED_L, 2
                                )
                                new_pos[sid] = srv.sts_tohost(raw_pos, 15)
                                new_spd[sid] = srv.sts_tohost(raw_spd, 15)
                        else:
                            pos, speed, r, _ = srv.ReadPosSpeed(sid)
                            if r == COMM_SUCCESS:
                                new_pos[sid] = pos
                                new_spd[sid] = speed

                        if poll_count % _HEALTH_EVERY == 0:
                            temp, r2, _ = srv.ReadTemperature(sid)
                            curr, r3, _ = srv.ReadCurrent(sid)
                            new_temp[sid] = (
                                temp if r2 == COMM_SUCCESS else None
                            )
                            new_curr[sid] = (
                                curr if r3 == COMM_SUCCESS else None
                            )

                with self.lock:
                    self.state.positions.update(new_pos)
                    self.state.speeds.update(new_spd)
                    if new_temp:
                        self.state.temps.update(new_temp)
                    if new_curr:
                        self.state.currents.update(new_curr)
                    self.state.connected = True
                    self.state.poll_error = None
                    self.state.poll_count = poll_count + 1

                for hook in self._sample_hooks:
                    hook(poll_count, new_pos, new_spd)

                poll_count += 1

            except Exception as exc:
                with self.lock:
                    self.state.connected = False
                    self.state.poll_error = str(exc)

            elapsed = time.monotonic() - t_start
            remaining = interval_s - elapsed
            if remaining > 0:
                self.stop_event.wait(timeout=remaining)
