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

  1. ``GroupSyncRead.txRxPacket`` — reads position, speed, and current from
     all servos and updates the thread-safe cache.
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
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence, TYPE_CHECKING

import numpy as np

from ..conversions import (
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
    STS_PRESENT_POSITION_L,
    STS_PRESENT_SPEED_L,
    STS_TORQUE_ENABLE,
)
from .types import JointState

logger = logging.getLogger(__name__)

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
        self._gsr: Optional[GroupSyncRead] = None  # position + speed (4 bytes)

        # Cached state (written by background thread, read by callers)
        self._lock = threading.Lock()
        self._cached_positions_ticks: List[int] = [TICK_ZERO] * n
        self._cached_speeds_ticks: List[int] = [0] * n
        self._cached_currents_mA: List[float] = [0.0] * n
        self._last_read_time: float = 0.0
        self._read_errors: int = 0

        # Pending write command — filled by control thread, drained by bus thread
        self._cmd_lock = threading.Lock()
        self._pending_command: Optional[_CommandBuffer] = None

        self._thread: Optional[threading.Thread] = None
        self._running = False

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

        # Register all joint IDs in the sync-read group (position + speed, 4 bytes)
        self._gsr = self._srv.groupSyncRead
        self._gsr.clearParam()
        for sid in self._joint_ids:
            self._gsr.addParam(sid)

        if self._torque_on_start:
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
            n = int(np.count_nonzero(~np.isclose(clamped, out, atol=1e-9)))
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

        Includes positions, velocities, and efforts (motor currents in mA).
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

    def _read_loop(self) -> None:
        rate = RateLimiter(frequency=self._state_freq, warn=False)
        while self._running:
            self._read_once()
            self._write_if_pending()  # serial: write only after read, bus is free
            rate.sleep()

    def _write_if_pending(self) -> None:
        """Drain the pending command buffer and send one sync-write packet."""
        with self._cmd_lock:
            cmd = self._pending_command
            self._pending_command = None
        if cmd is None or self._srv is None:
            return
        self._srv.groupSyncWrite.clearParam()
        for sid, ticks, spd in zip(self._joint_ids, cmd.ticks_list, cmd.speed_ticks_list):
            self._srv.SyncWritePosEx(sid, ticks, spd, cmd.acc)
        self._srv.groupSyncWrite.txPacket()

    def _read_once(self) -> None:
        if self._gsr is None or self._srv is None:
            return
        result = self._gsr.txRxPacket()
        if result != COMM_SUCCESS:
            self._read_errors += 1
            return

        pos_ticks: List[int] = []
        spd_ticks: List[int] = []
        all_ok = True
        for sid in self._joint_ids:
            avail, _ = self._gsr.isAvailable(sid, STS_PRESENT_POSITION_L, 2)
            if not avail:
                all_ok = False
                break
            raw_pos = self._gsr.getData(sid, STS_PRESENT_POSITION_L, 2)
            raw_spd = self._gsr.getData(sid, STS_PRESENT_SPEED_L, 2)
            # Decode sign bits (15-bit signed)
            pos_ticks.append(self._srv.sts_tohost(raw_pos, 15))
            spd_ticks.append(self._srv.sts_tohost(raw_spd, 15))

        if not all_ok or len(pos_ticks) != len(self._joint_ids):
            self._read_errors += 1
            return

        with self._lock:
            self._cached_positions_ticks = pos_ticks
            self._cached_speeds_ticks = spd_ticks
            self._last_read_time = time.monotonic()
