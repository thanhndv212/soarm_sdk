"""Hardware interface for STS3215 servos on the soarm100 arm.

Satisfies :class:`~soarm_sdk.robot.interfaces.RobotInterface` (via
:class:`~soarm_sdk.robot.servo.ServoRobot`, which wraps this class) so a
:class:`ServoHardwareInterface`-backed robot can be passed anywhere the
system expects a robot adapter — on real hardware or a simulation
equivalent implementing the same Protocol.

Architecture
------------
* A **background bus thread** runs at *state_freq* Hz (default 100 Hz) and
  alternates between two operations each tick:

  1. ``GroupSyncRead.txRxPacket`` — one transaction reading the whole
     read-only SRAM telemetry block (addresses 56-70: position, speed, load,
     voltage, temperature, status, moving, current) from all servos, and
     updates the thread-safe cache. One transaction per tick is deliberate:
     bus bandwidth is not the constraint, per-transaction USB turnaround is,
     so per-servo reads must never appear in this loop.
  2. ``_write_if_pending`` — drains the pending-command slot and sends one
     ``GroupSyncWrite.txPacket`` if a new command is waiting.

* :meth:`set_robot_joint_positions` is **non-blocking**: it stores the target
  into a shared ``_CommandBuffer`` slot (guarded by ``_cmd_lock``) and
  returns immediately. The bus thread transmits it on the next tick.
  This keeps the RS-485 bus access fully serialised and eliminates write
  latency from the high-level control thread.

* **Velocity feedforward**: pass ``dq`` (rad/s, shape [n_joints]) to
  :meth:`set_robot_joint_positions` to set per-joint ``GOAL_SPEED`` to
  ``|dq[i]|`` in ticks/s rather than the fixed ``default_speed``.

* **Forward kinematics**: pass an optional ``fk_fn`` callable to the
  constructor. :meth:`get_body_pose` will delegate to it instead of raising
  :class:`NotImplementedError`.

Usage
-----
::

    from soarm_sdk.robot.hardware import ServoHardwareInterface

    hw = ServoHardwareInterface(
        port="/dev/tty.usbserial-XXXX",
        fk_fn=lambda q: my_plant.get_body_pose("gripper_link", q),
    )
    hw.start()          # opens port, starts background bus thread
    try:
        q  = hw.get_robot_joint_positions()
        hw.set_robot_joint_positions(target_q, dq=velocity_feedforward)
        print(hw.state_age())   # seconds since last successful read
    finally:
        hw.stop()

Notes
-----
* EEPROM is never modified here; this class only writes to SRAM registers.
* ``state_age()`` returns ``float('inf')`` until the first read succeeds.
"""

from __future__ import annotations

import logging
import threading
import time
from contextlib import contextmanager
from dataclasses import dataclass
from typing import (
    Callable,
    Dict,
    Generator,
    List,
    Optional,
    Sequence,
    Tuple,
    TYPE_CHECKING,
)

import numpy as np

from ..conversions import (
    RADS_PER_TICK,
    SOARM100_DIRECTION_SIGNS,
    TICK_ZERO,
    joint_radians_to_ticks,
    joint_ticks_to_radians,
    rad_s_to_speed_ticks,
    speed_ticks_to_rad_s,
)
from ..protocol.group_sync_read import GroupSyncRead
from ..protocol.port_handler import PortHandler
from ..rate_limiter import RateLimiter

from ..protocol.sts import sts
from ..protocol.registers import (
    COMM_SUCCESS,
    STS_MAX_ANGLE_LIMIT_L,
    STS_MIN_ANGLE_LIMIT_L,
    STS_CURRENT_MA_PER_LSB,
    STS_CURRENT_SIGN_BIT,
    STS_LOAD_PERCENT_PER_LSB,
    STS_LOAD_SIGN_BIT,
    STS_MOVING,
    STS_POSITION_SIGN_BIT,
    STS_PRESENT_CURRENT_L,
    STS_PRESENT_LOAD_L,
    STS_PRESENT_POSITION_L,
    STS_PRESENT_SPEED_L,
    STS_PRESENT_TEMPERATURE,
    STS_PRESENT_VOLTAGE,
    STS_SPEED_SIGN_BIT,
    STS_STATUS,
    STS_TELEMETRY_LENGTH,
    STS_TELEMETRY_START,
    STS_TORQUE_ENABLE,
    STS_VOLTAGE_V_PER_LSB,
)
from .telemetry import DEFAULT_BUFFER_SAMPLES, ServoSample, TelemetryStream
from .types import JointState, ServoHealth

logger = logging.getLogger(__name__)

#: Below half an encoder tick, a clamp cannot change what the servo does:
#: the commanded value and the clamped value land on the same tick. Used as
#: the threshold for *counting* a limit clamp, not for applying it.
_CLAMP_EPS_RAD = RADS_PER_TICK / 2.0

if TYPE_CHECKING:  # pragma: no cover
    from ..calibration.frame import RobotCalibration

__all__ = ["ServoHardwareInterface"]

# Default motion parameters for soarm100
_DEFAULT_SPEED: int = 300  # ticks/s
_DEFAULT_ACC: int = 50  # acceleration units


@dataclass
class _CommandBuffer:
    """Pending write command stored by the control thread, consumed by the bus thread."""

    ticks_list: list
    speed_ticks_list: list
    acc: int
    timestamp: float


class ServoHardwareInterface:
    """Real-hardware robot interface backed by soarm_sdk.

    Parameters
    ----------
    port:
        Serial device path (e.g. ``/dev/tty.usbserial-XXXX`` on macOS,
        ``/dev/ttyUSB0`` on Linux).
    baud:
        Bus baud rate. Must match the EEPROM setting of all servos. Default
        is 1 000 000 bps (factory default for STS3215).
    joint_ids:
        Ordered list of servo IDs that map to robot joints 0..N-1.
        Default is ``[1, 2, 3, 4, 5, 6]`` for the soarm100.
    zero_offsets:
        Per-joint tick value that corresponds to 0 rad. Defaults to 2048
        for every joint.
    direction_signs:
        Per-joint direction multiplier (+1 or -1). Defaults to
        :data:`~soarm_sdk.conversions.SOARM100_DIRECTION_SIGNS`.
    default_speed:
        Speed (ticks/s) used by :meth:`set_robot_joint_positions` unless
        overridden.
    default_acc:
        Acceleration used by :meth:`set_robot_joint_positions` unless
        overridden.
    state_freq:
        Background state-read loop frequency in Hz. Default 100 Hz.
    torque_on_start:
        If True, torque is enabled on all joints when :meth:`start` is called.
    fk_fn:
        Optional forward-kinematics callable ``(q: np.ndarray) -> (pos, quat)``
        where *pos* is shape ``[3]`` and *quat* is ``[4]`` (w, x, y, z). When
        provided, :meth:`get_body_pose` will delegate to this function instead
        of raising :class:`NotImplementedError`.
    """

    def __init__(
        self,
        port: str,
        baud: int = 1_000_000,
        joint_ids: Optional[List[int]] = None,
        zero_offsets: Optional[List[int]] = None,
        direction_signs: Optional[List[int]] = None,
        default_speed: int = _DEFAULT_SPEED,
        default_acc: int = _DEFAULT_ACC,
        state_freq: float = 100.0,
        torque_on_start: bool = True,
        fk_fn: Optional[Callable[[np.ndarray], tuple[np.ndarray, np.ndarray]]] = None,
        joint_limits: Optional[tuple[Sequence[float], Sequence[float]]] = None,
        max_step_rad: Optional[float] = None,
        calibration: Optional["RobotCalibration"] = None,
        verify_eeprom_limits: bool = True,
    ) -> None:
        self._port = port
        self._baud = baud
        self._joint_ids: List[int] = joint_ids if joint_ids is not None else [1, 2, 3, 4, 5, 6]
        n = len(self._joint_ids)
        self._zero_offsets: List[int] = zero_offsets if zero_offsets is not None else [TICK_ZERO] * n
        self._direction_signs: List[int] = (
            direction_signs if direction_signs is not None else SOARM100_DIRECTION_SIGNS[:n]
        )
        self._default_speed = default_speed
        self._default_acc = default_acc
        self._state_freq = state_freq
        self._torque_on_start = torque_on_start
        self._fk_fn = fk_fn

        # A calibration supplies zero_offsets/direction_signs together and
        # overrides them, so the two can never be set from different sources
        # and drift apart.
        self._calibration = calibration
        if calibration is not None:
            if len(calibration.joints) != n:
                raise ValueError(
                    f"calibration has {len(calibration.joints)} joints, "
                    f"interface has {n}"
                )
            self._zero_offsets = list(calibration.zero_offsets)
            self._direction_signs = list(calibration.direction_signs)

        # Safety. Both default to off so existing callers are unaffected,
        # but soarm100 config supplies limits and callers streaming a
        # planned trajectory should set max_step_rad.
        if joint_limits is not None:
            lo, hi = joint_limits
            if len(lo) != n or len(hi) != n:
                raise ValueError(f"joint_limits must have {n} entries per side")
            if any(a >= b for a, b in zip(lo, hi)):
                raise ValueError("every joint_limits lower must be below its upper")
            self._limit_lo: Optional[List[float]] = [float(v) for v in lo]
            self._limit_hi: Optional[List[float]] = [float(v) for v in hi]
        else:
            self._limit_lo = None
            self._limit_hi = None

        if max_step_rad is not None and max_step_rad <= 0:
            raise ValueError("max_step_rad must be positive")
        self._max_step_rad = max_step_rad

        self._limit_clamps = 0
        self._step_clamps = 0

        # Servo bus objects — initialised in start()
        self._ph: Optional[PortHandler] = None
        self._srv: Optional[sts] = None
        # The full read-only telemetry block, not just position + speed:
        # 15 bytes costs 1.40 ms/tick on the wire against 0.74 ms for 4, and
        # buys load/current/voltage/temperature for free relative to the
        # per-servo reads they would otherwise need.
        self._gsr: Optional[GroupSyncRead] = None

        # Cached state (written by background thread, read by callers)
        self._lock = threading.Lock()
        self._cached_positions_ticks: List[int] = [TICK_ZERO] * n
        self._cached_speeds_ticks: List[int] = [0] * n
        self._cached_currents_mA: List[float] = [0.0] * n
        self._cached_loads_pct: List[float] = [0.0] * n
        self._cached_voltages_V: List[float] = [0.0] * n
        self._cached_temperatures_C: List[int] = [0] * n
        self._cached_status_flags: List[int] = [0] * n
        self._cached_moving: List[bool] = [False] * n
        self._last_read_time: float = 0.0
        self._read_errors: int = 0

        # Telemetry subscribers. Nothing is built when the list is empty, so
        # an interface nobody is watching pays nothing for this.
        self._stream_lock = threading.Lock()
        self._streams: List[TelemetryStream] = []
        self._seq: int = 0
        # The last command actually put on the wire, so a sample can carry
        # commanded and measured together. Written by the bus thread only.
        self._sent_ticks: Optional[List[int]] = None
        self._sent_speed_ticks: Optional[List[int]] = None
        self._sent_time: Optional[float] = None

        # Pending write command — filled by control thread, drained by bus thread
        self._cmd_lock = threading.Lock()
        self._pending_command: Optional[_CommandBuffer] = None
        # Torque requests take the same route: the bus thread owns the port,
        # so a caller-thread write would interleave with its sync-read packets.
        self._pending_torque: Optional[bool] = None
        self._torque_enabled: bool = torque_on_start

        self._verify_eeprom_limits = verify_eeprom_limits

        self._thread: Optional[threading.Thread] = None
        self._running = False
        # Held by the bus thread for the duration of each tick's I/O, and by
        # lend_bus() while a caller borrows the port. Deliberately NOT held
        # across the inter-tick sleep, so a borrower waits at most one tick.
        self._io_lock = threading.Lock()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Open the serial port and start the background state-read loop."""
        self._ph = PortHandler(self._port)
        if not self._ph.openPort():
            raise RuntimeError(f"ServoHardwareInterface: cannot open port {self._port!r}")
        if not self._ph.setBaudRate(self._baud):
            self._ph.closePort()
            raise RuntimeError(f"ServoHardwareInterface: cannot set baud rate {self._baud}")

        self._srv = sts(self._ph)

        # A GroupSyncRead of our own rather than ``self._srv.groupSyncRead``:
        # that shared instance is 4 bytes wide and other callers (the
        # dashboard) rely on it staying that way.
        self._gsr = GroupSyncRead(
            self._srv, STS_TELEMETRY_START, STS_TELEMETRY_LENGTH
        )
        self._gsr.clearParam()
        for sid in self._joint_ids:
            self._gsr.addParam(sid)

        # Prime the cache before anything can act on it. Until the first
        # successful read every joint reports TICK_ZERO — "the arm is at its
        # zero pose" — which is a lie whenever it is not, and one that has
        # already cost a real run (see soarm_tamp's README on the first
        # command being clamped against a placeholder pose).
        for _ in range(3):
            self._read_once()
            if self._last_read_time > 0.0:
                break

        # A servo's MIN/MAX_ANGLE_LIMIT is enforced underneath every layer of
        # software here and is never consulted by any of them — that gap is
        # what let ``wrist_flex`` cap at +0.86 rad while the calibration, the
        # planner and the servo's own commanded clamp all believed +1.27, with
        # nothing anywhere to say so; the joint just stopped moving. Checking
        # here, before torque is even enabled, means a disagreement is refused
        # at connect time rather than discovered as a stalled joint mid-plan.
        # Silent when nothing was ever recorded (every calibration written
        # before this field existed) rather than blocking arms that have not
        # opted in yet.
        if self._verify_eeprom_limits and self._calibration is not None:
            self._check_eeprom_limits()

        if self._torque_on_start:
            if self._last_read_time == 0.0:
                raise RuntimeError(
                    "cannot enable torque on start: no servo state could be "
                    "read, so goals cannot be parked at the measured pose and "
                    "enabling torque would drive the arm toward whatever stale "
                    "goal is in servo SRAM. Check the bus, or construct with "
                    "torque_on_start=False."
                )
            # Enabling torque makes a servo chase its goal. Park the goal at
            # the measured pose first or the arm lurches — the same reason
            # _apply_pending_torque does it.
            self._park_goals_at_measured()
            for sid in self._joint_ids:
                self._srv.write1ByteTxRx(sid, STS_TORQUE_ENABLE, 1)

        self._running = True
        self._thread = threading.Thread(
            target=self._read_loop,
            name="ServoStateReader",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        """Stop the background loop and close the serial port."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._ph is not None:
            self._ph.closePort()
            self._ph = None
        self._srv = None
        self._gsr = None
        with self._stream_lock:
            streams, self._streams = self._streams, []
        for stream in streams:
            stream.close()

    def __enter__(self) -> "ServoHardwareInterface":
        self.start()
        return self

    def __exit__(self, *_) -> None:
        self.stop()

    # ------------------------------------------------------------------
    # RobotInterface-facing API
    # ------------------------------------------------------------------

    def get_robot_joint_positions(self) -> np.ndarray:
        """Return cached joint positions in radians (shape [n_joints])."""
        with self._lock:
            ticks = list(self._cached_positions_ticks)
        rads = joint_ticks_to_radians(ticks, self._zero_offsets, self._direction_signs)
        return np.array(rads, dtype=float)

    def _apply_safety(self, target: np.ndarray) -> np.ndarray:
        """Clamp a commanded target to the joint limits and the step bound.

        Both clamps are deliberately *clamps* and not refusals. A refusal
        mid-trajectory leaves the arm wherever it was, which is rarely safer
        than moving a little less far than asked; and a planner that wants
        strict rejection can compare against get_joint_limits() first. What
        matters is that neither silently succeeds — every clamp is counted,
        and the counters are part of the public surface so a caller can
        assert on them after a run.

        The step bound is measured against the last *measured* position, not
        the last commanded one, so a joint that is lagging its target cannot
        accumulate an ever-larger jump.
        """
        out = np.asarray(target, dtype=float).copy()
        if out.shape != (len(self._joint_ids),):
            raise ValueError(
                f"expected {len(self._joint_ids)} joint values, got {out.shape}"
            )

        if self._limit_lo is not None and self._limit_hi is not None:
            lo = np.asarray(self._limit_lo)
            hi = np.asarray(self._limit_hi)
            clamped = np.clip(out, lo, hi)
            # Clip always; only *count* an excursion the hardware could
            # actually express. A planner that plans right up to a joint
            # limit emits waypoints that round a hair past it — real
            # trajectories here overshoot by 0.03 of an encoder tick — and
            # counting those as limit hits makes the counter cry wolf at
            # arithmetic, which is worse than useless when its whole job is
            # to say "the plan asked for a pose this arm cannot reach".
            n = int(np.count_nonzero(~np.isclose(clamped, out, atol=_CLAMP_EPS_RAD)))
            if n:
                self._limit_clamps += n
                logger.warning(
                    "joint limit clamp on %d joint(s): %s -> %s",
                    n, np.round(out, 4), np.round(clamped, 4),
                )
            out = clamped

        if self._max_step_rad is not None:
            current = self.get_robot_joint_positions()
            delta = out - current
            over = np.abs(delta) > self._max_step_rad
            if over.any():
                self._step_clamps += int(over.sum())
                logger.warning(
                    "step clamp on %d joint(s): max |delta| %.4f > %.4f rad",
                    int(over.sum()), float(np.abs(delta).max()), self._max_step_rad,
                )
                out = current + np.clip(delta, -self._max_step_rad, self._max_step_rad)

        return out

    @property
    def limit_clamps(self) -> int:
        """Joint-limit clamps applied since start. Non-zero means a caller
        asked for a pose this arm cannot reach."""
        return self._limit_clamps

    @property
    def step_clamps(self) -> int:
        """Step-size clamps applied since start. Non-zero during a
        trajectory replay means it is being sent faster than max_step_rad
        allows, and the arm is lagging the plan."""
        return self._step_clamps

    def set_robot_joint_positions(
        self,
        positions: np.ndarray,
        dq: Optional[np.ndarray] = None,
        speed: Optional[int] = None,
        acc: Optional[int] = None,
    ) -> None:
        """Queue target joint positions (in radians) for the next bus write.

        This method is **non-blocking**: it stores the command in a shared
        buffer that the background bus thread will transmit after its next
        state read, keeping the RS-485 bus access fully serialised.

        Parameters
        ----------
        positions:
            Target joint angles in radians, shape [n_joints].
        dq:
            Optional velocity feedforward in rad/s, shape [n_joints]. When
            provided, each joint's ``GOAL_SPEED`` is set to ``|dq[i]|``
            converted to ticks/s. Ignored if *speed* is also supplied.
        speed:
            Override per-joint speed (ticks/s) for all joints. Takes
            priority over *dq*.
        acc:
            Override default acceleration.
        """
        accel = acc if acc is not None else self._default_acc

        positions = self._apply_safety(np.asarray(positions, dtype=float))

        ticks_list = joint_radians_to_ticks(
            list(positions), self._zero_offsets, self._direction_signs
        )

        if speed is not None:
            speed_ticks_list = [speed] * len(self._joint_ids)
        elif dq is not None:
            speed_ticks_list = [max(1, abs(rad_s_to_speed_ticks(float(v)))) for v in dq]
        else:
            speed_ticks_list = [self._default_speed] * len(self._joint_ids)

        cmd = _CommandBuffer(
            ticks_list=ticks_list,
            speed_ticks_list=speed_ticks_list,
            acc=accel,
            timestamp=time.monotonic(),
        )
        with self._cmd_lock:
            self._pending_command = cmd

    def get_robot_joint_state(self) -> JointState:
        """Return a full :class:`~soarm_sdk.robot.types.JointState` snapshot.

        Includes positions, velocities, and efforts (signed motor current in
        mA, read from the same sync-read as position). For load, voltage,
        temperature and the status flags see :meth:`get_servo_health`.
        """
        with self._lock:
            pos_ticks = list(self._cached_positions_ticks)
            spd_ticks = list(self._cached_speeds_ticks)
            currents = list(self._cached_currents_mA)
            ts = self._last_read_time

        positions = np.array(
            joint_ticks_to_radians(pos_ticks, self._zero_offsets, self._direction_signs),
            dtype=float,
        )
        velocities = np.array([speed_ticks_to_rad_s(s) for s in spd_ticks], dtype=float)
        efforts = np.array(currents, dtype=float)
        return JointState(
            positions=positions,
            velocities=velocities,
            efforts=efforts,
            timestamp=ts,
        )

    def get_servo_health(self) -> ServoHealth:
        """Return the diagnostic half of the telemetry block.

        Servo-frame data — one entry per servo, ordered like ``joint_ids`` —
        as opposed to :meth:`get_robot_joint_state`'s joint-frame SI units.
        All of it comes from the same single sync-read, so every field shares
        one timestamp and needs no extra bus traffic.
        """
        with self._lock:
            return ServoHealth(
                loads_percent=np.array(self._cached_loads_pct, dtype=float),
                currents_mA=np.array(self._cached_currents_mA, dtype=float),
                voltages_V=np.array(self._cached_voltages_V, dtype=float),
                temperatures_C=np.array(self._cached_temperatures_C, dtype=float),
                status_flags=np.array(self._cached_status_flags, dtype=np.uint8),
                moving=np.array(self._cached_moving, dtype=bool),
                timestamp=self._last_read_time,
            )

    def get_body_pose(self, body_name: str) -> tuple[np.ndarray, np.ndarray]:
        """Return EE pose via the injected FK function, or raise if none was provided.

        Parameters
        ----------
        body_name:
            Name of the body/frame (passed through to *fk_fn*).

        Raises
        ------
        NotImplementedError
            When no ``fk_fn`` was supplied to the constructor.
        """
        if self._fk_fn is None:
            raise NotImplementedError(
                "get_body_pose requires a fk_fn. Pass fk_fn=your_fk_callable "
                "to ServoHardwareInterface.__init__."
            )
        q = self.get_robot_joint_positions()
        return self._fk_fn(q)

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------

    @property
    def read_errors(self) -> int:
        """Cumulative count of failed GroupSyncRead calls since start."""
        return self._read_errors

    @property
    def last_read_time(self) -> float:
        """Timestamp of the last successful state read (time.monotonic)."""
        return self._last_read_time

    def state_age(self) -> float:
        """Seconds elapsed since the last successful state read.

        Returns ``float('inf')`` if no read has succeeded yet.
        """
        ts = self._last_read_time
        if ts == 0.0:
            return float("inf")
        return time.monotonic() - ts

    # ------------------------------------------------------------------
    # Background state reader + writer
    # ------------------------------------------------------------------

    def read_angle_limits(self) -> Dict[int, Tuple[int, int]]:
        """Each servo's own MIN/MAX_ANGLE_LIMIT, in ticks, straight from EEPROM.

        These are the limits the *firmware* enforces, and nothing else in
        this stack reads them. That gap is not theoretical: measured on
        thanh_arm, servo 4 (``wrist_flex``) caps at 3046 ticks while the
        calibration recorded its travel as reaching 3314, so every layer
        above believed in 0.41 rad of range the servo refuses to deliver.
        A goal past the cap is accepted into GOAL_POSITION and then simply
        not acted on — no error, no status flag, and no current draw, because
        as far as the servo is concerned it is already where it was told to
        be. The joint silently stops moving and the trajectory carries on
        without it.

        So: read them, and let callers intersect them with whatever else
        they think the reachable set is.
        """
        out: Dict[int, Tuple[int, int]] = {}
        with self.lend_bus() as srv:
            for sid in self._joint_ids:
                lo = srv.read2ByteTxRx(sid, STS_MIN_ANGLE_LIMIT_L)
                hi = srv.read2ByteTxRx(sid, STS_MAX_ANGLE_LIMIT_L)
                out[sid] = (
                    lo.data[0] if lo.data else -1,
                    hi.data[0] if hi.data else -1,
                )
        return out

    def _check_eeprom_limits(self) -> None:
        """Refuse to start if a servo's live EEPROM disagrees with what the
        calibration recorded for it.

        Only compares joints the calibration has actually recorded (see
        :meth:`~soarm_sdk.calibration.frame.JointCalibration.eeprom_mismatch`)
        — a calibration that has never run the recording step says nothing
        either way, the same gate ``measured_is_trusted`` uses for travel
        acceptance. This is deliberately a hard refusal, not a printed
        warning: a mismatch here means some *other* command upstream (the
        planner, ``effective_limits()``) is working from a window the servo
        will not actually honor, silently, which is exactly what happened to
        ``wrist_flex``. ``verify_eeprom_limits=False`` opts out — for the
        tool that measures and re-records EEPROM limits in the first place,
        which must be able to connect to a servo it is about to correct.
        """
        live = self.read_angle_limits()
        problems = []
        for sid, joint in zip(self._joint_ids, self._calibration.joints):
            lo, hi = live.get(sid, (-1, -1))
            if lo < 0 or hi < 0:
                continue  # a failed read is state_age()'s problem, not this one
            msg = joint.eeprom_mismatch(lo, hi)
            if msg is not None:
                problems.append(msg)
        if problems:
            raise RuntimeError(
                "EEPROM angle limits have changed since this calibration "
                "recorded them:\n  " + "\n  ".join(problems) + "\n"
                "Every layer above the servo (planner bounds, "
                "effective_limits()) is now working from a window the "
                "firmware will not actually honor — a goal past it is "
                "accepted and silently ignored, not clamped or refused. "
                "Re-record with the EEPROM-limit tool, or construct with "
                "verify_eeprom_limits=False if this is deliberate."
            )

    def _read_loop(self) -> None:
        rate = RateLimiter(frequency=self._state_freq, warn=False)
        while self._running:
            with self._io_lock:
                self._read_once()
                self._apply_pending_torque()  # before the write: a limp joint takes no goal
                self._write_if_pending()  # serial: write only after read, bus is free
            rate.sleep()

    @contextmanager
    def lend_bus(
        self, *, timeout_s: float = 2.0
    ) -> Generator["sts", None, None]:
        """Pause the bus thread and hand the caller the live servo handle.

        The serial port is exclusive, so code that needs register access the
        interface does not wrap — EEPROM configuration, servo ID changes, a
        one-off diagnostic read — cannot simply open its own connection while
        this interface holds the port. It borrows this one instead.

        The bus thread finishes its current tick and then blocks, so a borrower
        waits at most one tick period. State goes stale for the duration:
        :meth:`state_age` grows and subscribers see a gap in ``seq``, which is
        correct — nothing was measured while somebody else owned the wire.

        ::

            with hw.lend_bus() as srv:
                srv.write1ByteTxRx(sid, STS_ACC, 20)

        Raises :class:`TimeoutError` rather than waiting forever, since the
        usual cause is another borrower holding the bus.
        """
        if self._srv is None:
            raise RuntimeError("lend_bus() needs an open port; call start() first")
        if not self._io_lock.acquire(timeout=timeout_s):
            raise TimeoutError(
                f"could not borrow the servo bus within {timeout_s:.1f}s — "
                "another caller is holding it"
            )
        try:
            yield self._srv
        finally:
            self._io_lock.release()

    # ------------------------------------------------------------------
    # Torque
    # ------------------------------------------------------------------

    @property
    def torque_enabled(self) -> bool:
        """Whether torque was last commanded on. Not read back from the servos."""
        return self._torque_enabled

    def set_torque(self, enabled: bool, *, timeout_s: float = 2.0) -> None:
        """Enable or disable torque on every joint, and wait until it lands.

        Disabling torque is what makes a joint back-driveable — needed to
        check a calibration's direction signs by hand, and the way to release
        a servo that has tripped its overload protection while pressed against
        a stop. Cutting the supply instead would take the bus down with it,
        leaving nothing to read the joint angles the check depends on.

        Blocking, unlike :meth:`set_robot_joint_positions`: a caller that is
        about to tell someone the arm is safe to move needs to know the servos
        were actually told, not that a request was queued.
        """
        enabled = bool(enabled)
        with self._cmd_lock:
            self._pending_torque = enabled
            if not enabled:
                # A queued move must not outlive the decision to go limp.
                self._pending_command = None

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            with self._cmd_lock:
                if self._pending_torque is None and self._torque_enabled == enabled:
                    return
            time.sleep(0.01)
        raise RuntimeError(
            f"torque {'enable' if enabled else 'disable'} not applied within "
            f"{timeout_s:.1f}s — is the bus thread running?"
        )

    def disable_torque(self, *, timeout_s: float = 2.0) -> None:
        """Go limp. See :meth:`set_torque`."""
        self.set_torque(False, timeout_s=timeout_s)

    def enable_torque(self, *, timeout_s: float = 2.0) -> None:
        """Hold station at the current measured pose. See :meth:`set_torque`."""
        self.set_torque(True, timeout_s=timeout_s)

    def _park_goals_at_measured(self) -> None:
        """Set every servo's goal to where it currently measures.

        A servo drives to its goal the moment torque is enabled, and that goal
        is whatever was last written to its SRAM — from a previous session, or
        from whatever it was chasing when torque was cut. After the arm has
        been moved by hand, or simply left at an arbitrary pose, that is a
        lurch. Parking first makes enabling torque a hold rather than a move.

        Requires a fresh measurement in the cache: parking against the
        ``TICK_ZERO`` seed would command the zero pose, which is the very
        thing this exists to prevent.
        """
        if self._srv is None:
            return
        with self._lock:
            ticks = list(self._cached_positions_ticks)
        self._srv.groupSyncWrite.clearParam()
        for sid, t in zip(self._joint_ids, ticks):
            self._srv.SyncWritePosEx(sid, t, self._default_speed, self._default_acc)
        self._srv.groupSyncWrite.txPacket()

    def _apply_pending_torque(self) -> None:
        """Drain a queued torque request. Runs on the bus thread only."""
        with self._cmd_lock:
            want = self._pending_torque
        if want is None or self._srv is None:
            return

        if want:
            self._park_goals_at_measured()

        for sid in self._joint_ids:
            self._srv.write1ByteTxRx(sid, STS_TORQUE_ENABLE, 1 if want else 0)

        with self._cmd_lock:
            self._torque_enabled = want
            self._pending_torque = None

    def _write_if_pending(self) -> None:
        """Drain the pending command buffer and send one sync-write packet."""
        with self._cmd_lock:
            cmd = self._pending_command
            self._pending_command = None
        if cmd is None or self._srv is None or not self._torque_enabled:
            return
        self._srv.groupSyncWrite.clearParam()
        for sid, ticks, spd in zip(self._joint_ids, cmd.ticks_list, cmd.speed_ticks_list):
            self._srv.SyncWritePosEx(sid, ticks, spd, cmd.acc)
        self._srv.groupSyncWrite.txPacket()
        # Remember what went out: the next tick's sample reports this as the
        # goal that was in force while it was measured.
        self._sent_ticks = list(cmd.ticks_list)
        self._sent_speed_ticks = list(cmd.speed_ticks_list)
        self._sent_time = time.monotonic()

    # ------------------------------------------------------------------
    # Telemetry
    # ------------------------------------------------------------------

    def subscribe(
        self, *, maxlen: int = DEFAULT_BUFFER_SAMPLES
    ) -> TelemetryStream:
        """Start receiving a :class:`ServoSample` per successful bus tick.

        Returns a bounded, drop-oldest queue to drain from your own thread.
        With no subscribers the bus thread builds nothing, so subscribing is
        the switch that turns telemetry on.

        The stream keeps filling until :meth:`unsubscribe` or
        :meth:`TelemetryStream.close`; a consumer that stops draining loses
        the oldest samples, never the newest.
        """
        stream = TelemetryStream(maxlen=maxlen)
        with self._stream_lock:
            self._streams.append(stream)
        return stream

    def unsubscribe(self, stream: TelemetryStream) -> None:
        """Detach *stream* and close it. Safe to call twice."""
        with self._stream_lock:
            if stream in self._streams:
                self._streams.remove(stream)
        stream.close()

    @property
    def subscriber_count(self) -> int:
        with self._stream_lock:
            return len(self._streams)

    def _publish_sample(
        self,
        t_read: float,
        pos_ticks: List[int],
        spd_ticks: List[int],
        loads_pct: List[float],
        currents_mA: List[float],
        voltages_V: List[float],
        temps_C: List[int],
        status_flags: List[int],
        moving: List[bool],
    ) -> None:
        """Fan one tick out to every subscriber. Bus thread; must not raise."""
        with self._stream_lock:
            streams = list(self._streams)
        if not streams:
            return

        goal_ticks = self._sent_ticks
        goal_rad: Optional[Tuple[float, ...]] = None
        if goal_ticks is not None:
            goal_rad = tuple(
                joint_ticks_to_radians(
                    goal_ticks, self._zero_offsets, self._direction_signs
                )
            )

        sample = ServoSample(
            t_mono=t_read,
            seq=self._seq,
            ids=tuple(self._joint_ids),
            position_ticks=tuple(pos_ticks),
            position_rad=tuple(
                joint_ticks_to_radians(
                    pos_ticks, self._zero_offsets, self._direction_signs
                )
            ),
            velocity_ticks=tuple(spd_ticks),
            velocity_rad_s=tuple(speed_ticks_to_rad_s(v) for v in spd_ticks),
            load_percent=tuple(loads_pct),
            current_mA=tuple(currents_mA),
            voltage_V=tuple(voltages_V),
            temperature_C=tuple(temps_C),
            status_flags=tuple(status_flags),
            moving=tuple(moving),
            goal_position_ticks=(
                tuple(goal_ticks) if goal_ticks is not None else None
            ),
            goal_position_rad=goal_rad,
            goal_speed_ticks=(
                tuple(self._sent_speed_ticks)
                if self._sent_speed_ticks is not None
                else None
            ),
            goal_t_mono=self._sent_time,
            read_errors=self._read_errors,
        )

        for stream in streams:
            try:
                stream._publish(sample)
            except Exception:  # pragma: no cover - defensive
                # A broken consumer must never take the bus thread down with
                # it: this thread also writes to the servos.
                logger.exception("telemetry stream rejected a sample")

    def _read_once(self) -> None:
        if self._gsr is None or self._srv is None:
            return
        # seq counts ticks, not published samples: a gap in what a subscriber
        # sees is how it learns a read failed.
        self._seq += 1
        result = self._gsr.txRxPacket()
        if result != COMM_SUCCESS:
            self._read_errors += 1
            return

        # Stamp as close to the wire as possible: this is the timestamp every
        # downstream consumer aligns commanded against measured on.
        t_read = time.monotonic()

        pos_ticks: List[int] = []
        spd_ticks: List[int] = []
        loads_pct: List[float] = []
        currents_mA: List[float] = []
        voltages_V: List[float] = []
        temps_C: List[int] = []
        status_flags: List[int] = []
        moving: List[bool] = []
        all_ok = True
        for sid in self._joint_ids:
            # Availability is checked against the LAST register in the block,
            # so a short or truncated response fails here rather than decoding
            # into garbage.
            avail, _ = self._gsr.isAvailable(sid, STS_PRESENT_CURRENT_L, 2)
            if not avail:
                all_ok = False
                break
            raw_pos = self._gsr.getData(sid, STS_PRESENT_POSITION_L, 2)
            raw_spd = self._gsr.getData(sid, STS_PRESENT_SPEED_L, 2)
            raw_load = self._gsr.getData(sid, STS_PRESENT_LOAD_L, 2)
            raw_curr = self._gsr.getData(sid, STS_PRESENT_CURRENT_L, 2)
            raw_volt = self._gsr.getData(sid, STS_PRESENT_VOLTAGE, 1)
            raw_temp = self._gsr.getData(sid, STS_PRESENT_TEMPERATURE, 1)
            raw_status = self._gsr.getData(sid, STS_STATUS, 1)
            raw_moving = self._gsr.getData(sid, STS_MOVING, 1)

            # Sign-magnitude decode: the sign bit differs per register.
            pos_ticks.append(self._srv.sts_tohost(raw_pos, STS_POSITION_SIGN_BIT))
            spd_ticks.append(self._srv.sts_tohost(raw_spd, STS_SPEED_SIGN_BIT))
            loads_pct.append(
                self._srv.sts_tohost(raw_load, STS_LOAD_SIGN_BIT)
                * STS_LOAD_PERCENT_PER_LSB
            )
            currents_mA.append(
                self._srv.sts_tohost(raw_curr, STS_CURRENT_SIGN_BIT)
                * STS_CURRENT_MA_PER_LSB
            )
            voltages_V.append(raw_volt * STS_VOLTAGE_V_PER_LSB)
            temps_C.append(raw_temp)
            status_flags.append(raw_status)
            moving.append(bool(raw_moving))

        if not all_ok or len(pos_ticks) != len(self._joint_ids):
            self._read_errors += 1
            return

        with self._lock:
            self._cached_positions_ticks = pos_ticks
            self._cached_speeds_ticks = spd_ticks
            self._cached_loads_pct = loads_pct
            self._cached_currents_mA = currents_mA
            self._cached_voltages_V = voltages_V
            self._cached_temperatures_C = temps_C
            self._cached_status_flags = status_flags
            self._cached_moving = moving
            self._last_read_time = t_read

        self._publish_sample(
            t_read,
            pos_ticks,
            spd_ticks,
            loads_pct,
            currents_mA,
            voltages_V,
            temps_C,
            status_flags,
            moving,
        )
