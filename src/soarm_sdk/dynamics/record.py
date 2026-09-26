"""Stream an excitation trajectory and record what the servos report.

Works with any :class:`~soarm_sdk.robot.interfaces.RobotInterface`. When the
robot exposes the bus thread's telemetry tap (``robot.hw.subscribe()``, i.e.
a connected :class:`~soarm_sdk.robot.servo.ServoRobot`), every bus tick is
recorded with the timestamp taken next to the wire. Anything else — a
:class:`~soarm_sdk.robot.null.NullRobot` for a dry run, a lerobot-backed
arm — is polled once per command instead.

Safety
------
The trajectory itself stays inside the limits it was planned against, and
``ServoRobot`` clamps every write to its effective limits and step bound
regardless. On top of that, each tick checks that the arm is actually
following: a joint lagging its command by more than ``abort_tracking_rad``
(something in the way, a stalled servo) or drawing more than
``abort_current_mA`` stops the run, re-commands the measured pose so the arm
stops pushing, and raises :class:`ExcitationAborted` carrying whatever was
recorded up to that point.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Sequence

import numpy as np

from ..robot.interfaces import RobotInterface
from ..robot.telemetry import ServoSample
from .log import IdentificationLog

__all__ = ["ExcitationAborted", "record_excitation"]


class ExcitationAborted(RuntimeError):
    """The run was stopped by a safety check. ``log`` holds the partial data."""

    def __init__(self, message: str, log: Optional[IdentificationLog]) -> None:
        super().__init__(message)
        self.log = log


class _Stop(Exception):
    """Internal: a safety check tripped inside the streaming loop."""


@dataclass
class _Polled:
    t: float
    q: np.ndarray
    dq: Optional[np.ndarray]
    effort: Optional[np.ndarray]
    q_cmd: np.ndarray


def _telemetry(robot: Any):
    hw = getattr(robot, "hw", None)
    if hw is not None and callable(getattr(hw, "subscribe", None)):
        return hw
    return None


#: The ``dq`` hint sent with each command is a per-joint *speed cap*, not a
#: feedforward: ServoRobot writes ``|dq|`` straight into each servo's
#: GOAL_SPEED (floored at 1 tick/s). Sending the plan's own velocity would cap
#: every servo at exactly the planned speed, so any lag could never be made up
#: — and at each direction reversal the cap drops to ~0, stalling the joint.
#: Headroom plus a floor keeps the servo able to catch up.
SPEED_HEADROOM = 1.5
MIN_SPEED_RAD_S = 0.3


def _stream(
    robot: RobotInterface,
    path: np.ndarray,
    dt: float,
    clock: Callable[[], float],
    sleep: Callable[[float], None],
    on_tick: Optional[Callable[[int, np.ndarray], None]] = None,
) -> None:
    dq = np.gradient(path, dt, axis=0) if len(path) > 1 else np.zeros_like(path)
    cap = np.maximum(np.abs(dq) * SPEED_HEADROOM, MIN_SPEED_RAD_S)
    start = clock()
    for k, (q, v) in enumerate(zip(path, cap)):
        robot.set_joint_positions(q, dq=v)
        if on_tick is not None:
            on_tick(k, q)
        wait = start + (k + 1) * dt - clock()
        if wait > 0:
            sleep(wait)


def record_excitation(
    robot: RobotInterface,
    q_plan: np.ndarray,
    rate_hz: float,
    *,
    joint_names: Sequence[str],
    direction_signs: Sequence[int],
    approach_speed_rad_s: float = 0.25,
    settle_s: float = 1.0,
    abort_tracking_rad: Optional[float] = 0.35,
    abort_current_mA: Optional[float] = None,
    meta: Optional[Dict[str, Any]] = None,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> IdentificationLog:
    """Move to the start of *q_plan*, stream it at *rate_hz*, and record.

    Parameters
    ----------
    q_plan
        ``(N, n_dof)`` joint targets, URDF frame, one per ``1/rate_hz``.
    direction_signs
        The calibration's per-joint signs, used to bring motor-frame
        velocity, current and load into the URDF frame (see
        :mod:`soarm_sdk.dynamics.log`).
    """
    q_plan = np.asarray(q_plan, dtype=float)
    n = len(joint_names)
    if q_plan.ndim != 2 or q_plan.shape[1] != n:
        raise ValueError(f"q_plan must be (N, {n}), got {q_plan.shape}")
    dt = 1.0 / rate_hz
    sign = np.asarray(direction_signs, dtype=float)

    # 1. Approach the start pose slowly, then let it settle.
    q0 = np.asarray(robot.get_joint_positions(), dtype=float)
    dist = float(np.max(np.abs(q_plan[0] - q0)))
    steps = max(2, math.ceil(dist / (approach_speed_rad_s * dt)) + 1)
    _stream(robot, np.linspace(q0, q_plan[0], steps), dt, clock, sleep)
    sleep(settle_s)

    # 2. Stream the excitation, recording as we go.
    hw = _telemetry(robot)
    stream = hw.subscribe() if hw is not None else None
    samples: List[ServoSample] = []
    polled: List[_Polled] = []
    abort: List[str] = []

    def check(k: int, q_cmd: np.ndarray) -> None:
        if stream is not None:
            samples.extend(stream.drain())
            if not samples:
                return
            last = samples[-1]
            q_meas = np.asarray(last.position_rad, dtype=float)
            cur = np.asarray(last.current_mA, dtype=float)
        else:
            state = robot.get_joint_state()
            q_meas = np.asarray(state.positions, dtype=float)
            cur = None if state.efforts is None else np.asarray(state.efforts, dtype=float)
            polled.append(
                _Polled(
                    t=clock(),
                    q=q_meas,
                    dq=None if state.velocities is None else np.asarray(state.velocities, dtype=float),
                    effort=cur,
                    q_cmd=np.asarray(q_cmd, dtype=float),
                )
            )
        if abort_tracking_rad is not None and k > 0:
            # Against the previous command: the servo has had one tick to act on it.
            err = np.abs(q_plan[k - 1] - q_meas)
            j = int(np.argmax(err))
            if err[j] > abort_tracking_rad:
                abort.append(
                    f"{joint_names[j]} lagging its command by {err[j]:.3f} rad "
                    f"(> {abort_tracking_rad} rad) at t={k * dt:.2f}s"
                )
        if abort_current_mA is not None and cur is not None:
            j = int(np.argmax(np.abs(cur)))
            if abs(cur[j]) > abort_current_mA:
                abort.append(
                    f"{joint_names[j]} drawing {abs(cur[j]):.0f} mA "
                    f"(> {abort_current_mA:.0f} mA) at t={k * dt:.2f}s"
                )
        if abort:
            raise _Stop()

    info = dict(meta or {})
    info.update(
        {
            "commanded_rate_hz": float(rate_hz),
            "source": "telemetry" if stream is not None else "polled",
        }
    )
    try:
        _stream(robot, q_plan, dt, clock, sleep, on_tick=check)
    except _Stop:
        robot.set_joint_positions(np.asarray(robot.get_joint_positions(), dtype=float))
    finally:
        if stream is not None:
            samples.extend(stream.drain())
            hw.unsubscribe(stream)

    if abort:
        info["aborted"] = abort[0]

    log: Optional[IdentificationLog]
    if stream is not None:
        log = (
            IdentificationLog.from_samples(samples, joint_names, direction_signs, meta=info)
            if samples
            else None
        )
    else:
        log = _from_polled(polled, joint_names, sign, info) if polled else None

    if abort:
        raise ExcitationAborted(abort[0], log)
    if log is None:
        raise RuntimeError("the run finished without recording a single sample")
    return log


def _from_polled(
    polled: List[_Polled], joint_names: Sequence[str], sign: np.ndarray, meta: Dict[str, Any]
) -> IdentificationLog:
    t = np.array([p.t for p in polled])
    t -= t[0]
    q = np.array([p.q for p in polled])
    if all(p.dq is not None for p in polled):
        dq = np.array([p.dq for p in polled]) * sign
    else:
        dq = np.gradient(q, t, axis=0) if len(t) > 1 else np.zeros_like(q)
    if all(p.effort is not None for p in polled):
        cur = np.array([p.effort for p in polled]) * sign
    else:
        cur = np.zeros_like(q)
    return IdentificationLog(
        joint_names=list(joint_names),
        t=t,
        q=q,
        dq=dq,
        current_mA=cur,
        load_percent=np.zeros_like(q),
        q_cmd=np.array([p.q_cmd for p in polled]),
        meta=meta,
    )
