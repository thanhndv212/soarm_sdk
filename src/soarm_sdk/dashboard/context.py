"""Shared connection/polling state for soarm_sdk dashboard panels.

Every :class:`~soarm_sdk.dashboard.app.Panel` receives the same
:class:`DashboardContext` instance, so a new panel never has to
reimplement "open the bus," "parse an ID range," or "poll the servos in
the background" — it just reads ``ctx.state`` or calls ``ctx.bus()``.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from contextlib import contextmanager
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Generator, List, Optional

from ..bus import parse_range
from ..protocol.port_handler import PortHandler
from ..protocol.registers import (
    COMM_SUCCESS,
    STS_PRESENT_POSITION_L,
    STS_PRESENT_SPEED_L,
)
from ..protocol.sts import sts as _Sts

logger = logging.getLogger(__name__)

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
        use_stream: bool = False,
        rerun: bool = False,
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

        # Streaming mode: one ServoHardwareInterface holds the port open and
        # the dashboard consumes its telemetry stream, instead of reopening
        # the port every poll iteration and issuing per-servo health reads.
        # Opt-in for now; the legacy poll loop below stays until it has run
        # on hardware for a release.
        self.use_stream = use_stream
        self._interface: Optional[Any] = None
        self._stream: Optional[Any] = None

        # Optional: feed the same interface's telemetry to a Rerun viewer
        # for its better multi-channel charts, while every control here
        # (torque, joint commands, PID tuning, scan) stays on this one
        # shared connection — see soarm_sdk.monitoring.blueprint. Requires
        # use_stream, since the legacy poll loop never holds a persistent
        # interface to subscribe a second consumer to.
        self.rerun_enabled = rerun
        self._rerun_recorder: Optional[Any] = None

        # Set by DashboardApp once the URDF and calibration are loaded. They
        # live here, not on the app, because the 3-D view and the Calibration
        # tab have to agree about which calibration is in force: the tab
        # rewrites this attribute and the next FK tick picks it up.
        self.calibration: Optional[Any] = None
        self.calibration_path: Optional[Any] = None
        self.urdf: Optional[Any] = None

    # ------------------------------------------------------------------
    # Calibration: the 3-D view's copy vs. the one on disk
    # ------------------------------------------------------------------

    def calibration_drift(self) -> List[tuple]:
        """``[(joint, degrees)]`` where the live calibration differs from disk.

        The 3-D mirror renders :attr:`calibration`, which a panel can edit in
        memory. Everything else re-reads the file: the planner runs in a
        container and can only ever see a file, and the pose capture, the
        executor and the manifest player each load their own copy. So an
        unsaved edit means the mirror is showing one arm and the planner is
        planning for another, with nothing on screen to say so.

        Empty when they agree, when there is nothing loaded, or when the file
        is unreadable — this reports a divergence it can actually demonstrate,
        and a missing file is a different problem with its own message.
        """
        cal = self.calibration
        if cal is None or self.calibration_path is None:
            return []
        try:
            from ..calibration.frame import RobotCalibration

            on_disk = RobotCalibration.load(self.calibration_path)
        except Exception:
            return []

        saved = {j.name: j for j in on_disk.joints}
        out: List[tuple] = []
        for j in cal.joints:
            other = saved.get(j.name)
            if other is None:
                continue
            # Compare what the ticks are taken to mean, not the stored
            # numbers: a zero shift and a sign flip can cancel in the fields
            # and still render the arm somewhere else entirely.
            delta = math.degrees(j.to_rad(0.0) - other.to_rad(0.0))
            if abs(delta) > 0.01 or j.direction_sign != other.direction_sign:
                out.append((j.name, delta))
        return out

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
        # In streaming mode the interface owns the port, so a second
        # PortHandler on the same device would simply fail. Borrow its handle
        # instead — every existing caller keeps working unchanged.
        if self._interface is not None:
            with self._interface.lend_bus() as srv:
                yield srv
            return

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
        if self.use_stream:
            self._start_stream(device, baud)
            return
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
        if self._rerun_recorder is not None:
            recorder, self._rerun_recorder = self._rerun_recorder, None
            try:
                recorder.stop()
            except Exception:  # pragma: no cover - defensive teardown
                pass
        if self._interface is not None:
            iface, self._interface = self._interface, None
            self._stream = None
            try:
                iface.stop()
            except Exception:  # pragma: no cover - defensive teardown
                pass
        with self.lock:
            self.state.connected = False
        self.conn_status_md.content = "*Disconnected.*"

    # ------------------------------------------------------------------
    # Streaming mode (opt-in): consume ServoHardwareInterface telemetry
    # ------------------------------------------------------------------

    def _start_stream(self, device: str, baud: int) -> None:
        """Hold the port open with one interface and consume its telemetry.

        Replaces two costs the legacy poll loop pays every cycle: reopening
        the serial port (milliseconds, and it resets the device), and — every
        fifth poll — twelve per-servo round trips for temperature and current,
        measured at 7.36 ms against a 2.09 ms full-block sync-read that
        carries the same fields. Here they arrive in the block, every tick.
        """
        from ..robot.hardware import ServoHardwareInterface

        try:
            iface = ServoHardwareInterface(
                port=device,
                baud=baud,
                joint_ids=list(self.joint_ids),
                # The dashboard must never energise the arm just by connecting.
                torque_on_start=False,
            )
            iface.start()
        except Exception as exc:
            with self.lock:
                self.state.connected = False
                self.state.poll_error = str(exc)
            self.conn_status_md.content = f"*Connect failed: {exc}*"
            return

        self._interface = iface
        self._stream = iface.subscribe(maxlen=2000)
        t = threading.Thread(target=self._stream_loop, daemon=True)
        t.start()
        self._poll_thread = t

        rerun_note = ""
        if self.rerun_enabled:
            rerun_note = self._start_rerun(iface)

        self.conn_status_md.content = (
            f"**Streaming** `{device}` @ {baud} baud "
            f"({iface._state_freq:.0f} Hz){rerun_note}"
        )

    def _start_rerun(self, iface: Any) -> str:
        """Attach a Rerun sink to *iface*'s telemetry stream.

        A second, independent :meth:`subscribe` — the interface already
        supports multiple simultaneous consumers, so this never competes
        with the dashboard's own :meth:`_stream_loop` for samples. Failure
        here is reported but never fatal: the dashboard's actual controls
        (torque, joint commands, PID tuning) do not depend on Rerun, so a
        missing ``rerun-sdk`` or a viewer that failed to launch should not
        take those down too.

        Returns a short suffix for the connection status line, empty on
        success (the status line already says "Streaming"; failure is
        worth calling out there too).
        """
        try:
            import rerun as rr

            from .fk import SOARM100_JOINT_NAMES
            from ..monitoring.blueprint import build_monitor_blueprint
            from ..robot.telemetry_sinks import RerunSink, TelemetryRecorder

            joint_names = [SOARM100_JOINT_NAMES[i - 1] for i in self.joint_ids]
            blueprint = build_monitor_blueprint(joint_names)
            rr.init("soarm_dashboard", spawn=True, default_blueprint=blueprint)

            recorder = TelemetryRecorder(iface, [RerunSink(joint_names=joint_names)])
            recorder.start()
            self._rerun_recorder = recorder
            return " · Rerun viewer spawned"
        except Exception as exc:
            logger.warning("Rerun sink failed to start: %s", exc)
            return f" · Rerun failed to start: {exc}"

    def _stream_loop(self) -> None:
        """Drain telemetry into ``self.state``. Never touches the serial port."""
        count = 0
        while not self.stop_event.is_set():
            stream = self._stream
            if stream is None:
                break
            if not stream.wait(timeout=0.2):
                continue
            samples = stream.drain()
            if not samples:
                continue
            latest = samples[-1]
            new_pos = dict(zip(latest.ids, latest.position_ticks))
            new_spd = dict(zip(latest.ids, latest.velocity_ticks))
            with self.lock:
                self.state.positions.update(new_pos)
                self.state.speeds.update(new_spd)
                # Health data is in every sample now, not every fifth poll.
                self.state.temps.update(
                    dict(zip(latest.ids, latest.temperature_C))
                )
                self.state.currents.update(
                    dict(zip(latest.ids, latest.current_mA))
                )
                self.state.connected = True
                self.state.poll_error = None
                self.state.poll_count = latest.seq
            # One hook call per drain rather than per sample: hooks exist to
            # feed the GUI, which repaints at ~10 Hz and cannot use 100.
            count += 1
            for hook in self._sample_hooks:
                try:
                    hook(count, new_pos, new_spd)
                except Exception:
                    # A broken hook used to surface as "disconnected", because
                    # it raised inside the poll loop's shared try. It is a hook
                    # bug, not a bus failure; keep them apart.
                    logger.exception("dashboard sample hook raised")

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
