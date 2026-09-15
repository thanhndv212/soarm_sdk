"""Correct one servo's EEPROM angle limit against its actual mechanical stop.

    soarm-widen-limit wrist_flex --port /dev/cu.usbmodem... --direction max --apply

RUNS ON THE HOST, with the arm connected.

Why this exists
----------------
A servo's ``MIN/MAX_ANGLE_LIMIT`` (EEPROM registers 9, 11) is enforced
beneath every software layer in this stack, and nothing in this stack ever
read it before this module's twin, :func:`~soarm_sdk.robot.hardware.ServoHardwareInterface.read_angle_limits`,
was added. On ``thanh_arm`` that let ``wrist_flex``'s cap sit at +0.86 rad
while the calibration, the planner, and the servo's own commanded clamp all
agreed the joint reached +1.27 — a goal past the cap is accepted into
GOAL_POSITION and then simply never acted on: no error, no status flag, no
current draw, because as far as the servo is concerned it already arrived.
The joint drops out of the trajectory while the rest of the arm keeps
going.

That fix was done by hand, once, with a scratch script: back up EEPROM,
widen past where the model *thinks* the stop is, walk toward the real stop
in small steps watching current, stop the moment progress ceases, then set
the limit just short of what was actually found. This module is that
procedure, generalized to either direction and any joint, and wired to
:meth:`~soarm_sdk.calibration.frame.JointCalibration.with_eeprom_limits` so
the result is recorded rather than left to drift out of sync again.

What it will not do
--------------------
Decide *whether* to change your calibration's zero or measured travel
(:meth:`~soarm_sdk.calibration.frame.JointCalibration.with_travel`) based on
what it finds — a widened EEPROM cap and a corrected travel record are two
separate facts, and conflating them is how the original bug happened. It
only ever writes the two ``eeprom_*`` fields.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Literal, Optional

import numpy as np

from ..bus.discovery import write1, write2
from ..protocol.registers import (
    STS_LOCK,
    STS_MAX_ANGLE_LIMIT_L,
    STS_MIN_ANGLE_LIMIT_L,
    STS_PRESENT_CURRENT_L,
    STS_PRESENT_POSITION_L,
)
from .frame import JointCalibration, RobotCalibration

__all__ = ["WidenResult", "probe_and_set_limit", "main"]

Direction = Literal["min", "max"]

_STEP_TICKS = 12
_SETTLE_S = 0.35
_STALL_STEPS = 3
_PROGRESS_TICKS = 3
_CURRENT_CEILING_MA = 500.0
_MARGIN_TICKS = 20
_DEFAULT_DQ = 0.4


@dataclass(frozen=True)
class WidenResult:
    joint: str
    direction: Direction
    before_ticks: int
    stop_ticks: int
    final_ticks: int
    steps_taken: int
    stopped_reason: str  # "no_progress" | "current_ceiling" | "hit_probe_bound"


def probe_and_set_limit(
    hw: "Any",  # noqa: F821 — ServoHardwareInterface, avoided to keep this importable standalone
    servo_id: int,
    calibration: JointCalibration,
    *,
    direction: Direction,
    probe_bound_ticks: int,
    step_ticks: int = _STEP_TICKS,
    settle_s: float = _SETTLE_S,
    stall_steps: int = _STALL_STEPS,
    progress_ticks: int = _PROGRESS_TICKS,
    current_ceiling_ma: float = _CURRENT_CEILING_MA,
    margin_ticks: int = _MARGIN_TICKS,
    dq_rad_s: float = _DEFAULT_DQ,
    joint_index: int = 0,
    n_joints: int = 6,
) -> WidenResult:
    """Widen one EEPROM bound to *probe_bound_ticks*, then set it just short
    of wherever the joint actually stops.

    *hw* must already be started (``hw.start()``), ideally with
    ``verify_eeprom_limits=False`` — this function is what corrects a
    mismatch the connect-time check would otherwise refuse to start
    against. Moves only *servo_id*'s joint; every other joint is held at its
    current measured position throughout.

    The stop is found empirically, never assumed: walk *step_ticks* at a
    time toward *probe_bound_ticks*, watching present current, and stop the
    first time either (a) *stall_steps* consecutive steps make less than
    *progress_ticks* of progress — the joint has stopped moving under this
    much commanded speed, which is what a hard stop looks like — or (b)
    current exceeds *current_ceiling_ma*, a stall aggressive enough that
    continuing risks the servo or the mechanism. The EEPROM bound is then
    set *margin_ticks* short of the stop, never past it, and never past
    whatever the caller already trusted (a widen can only ever be undone
    back to the original, not overshoot the mechanism).

    *dq_rad_s* is the commanded speed feedforward, not just the default
    ``0.4``: a joint fighting gravity (``elbow_flex`` moving toward its low
    end, on this arm) can fail to complete a small step within *settle_s* at
    a gentle speed — indistinguishable from ``no_progress`` unless ruled
    out. Measured directly on ``elbow_flex``: 12-tick steps at ``dq=0.4``
    stalled after five steps at 52-104 mA (real torque, not near-zero); the
    same joint pushed at ``dq=0.8`` for a sustained 150-tick move kept
    closing the gap the whole time. A probe that stalls quickly at
    moderate, non-negligible current has not distinguished "hit a wall" from
    "under-driven" — raise *dq_rad_s* and re-run before trusting that result.
    """
    reg = STS_MIN_ANGLE_LIMIT_L if direction == "min" else STS_MAX_ANGLE_LIMIT_L
    sign = -1 if direction == "min" else 1

    def rd2(srv: "Any", addr: int) -> int:  # noqa: F821
        r = srv.read2ByteTxRx(servo_id, addr)
        return r.data[0] if r.data else -1

    with hw.lend_bus() as srv:
        before = rd2(srv, reg)
        write1(srv, servo_id, STS_LOCK, 0, "unlock EEPROM")
        write2(srv, servo_id, reg, probe_bound_ticks, "widen for probing")

    reached = None
    stalled = 0
    steps_taken = 0
    stopped_reason = "hit_probe_bound"
    while True:
        with hw.lend_bus() as srv:
            pos = rd2(srv, STS_PRESENT_POSITION_L)
        if reached is None:
            reached = pos
        if (direction == "max" and reached >= probe_bound_ticks) or (
            direction == "min" and reached <= probe_bound_ticks
        ):
            break

        goal = reached + sign * step_ticks
        if (direction == "max" and goal > probe_bound_ticks) or (
            direction == "min" and goal < probe_bound_ticks
        ):
            goal = probe_bound_ticks

        q = np.asarray(hw.get_robot_joint_positions(), dtype=float)
        q[joint_index] = calibration.to_rad(goal)
        hw.set_robot_joint_positions(q, dq=np.full(n_joints, dq_rad_s))
        time.sleep(settle_s)
        steps_taken += 1

        with hw.lend_bus() as srv:
            new = rd2(srv, STS_PRESENT_POSITION_L)
            cur_raw = rd2(srv, STS_PRESENT_CURRENT_L)
        current_ma = (cur_raw & 0x7FFF) * 6.5

        progress = (new - reached) if direction == "max" else (reached - new)
        reached = new
        if current_ma > current_ceiling_ma:
            stopped_reason = "current_ceiling"
            break
        stalled = stalled + 1 if progress < progress_ticks else 0
        if stalled >= stall_steps:
            stopped_reason = "no_progress"
            break

    final = (
        max(before, reached - margin_ticks)
        if direction == "max"
        else min(before, reached + margin_ticks)
    )
    with hw.lend_bus() as srv:
        write2(srv, servo_id, reg, final, "set corrected limit")
        write1(srv, servo_id, STS_LOCK, 1, "lock EEPROM")

    return WidenResult(
        joint=calibration.name,
        direction=direction,
        before_ticks=before,
        stop_ticks=reached,
        final_ticks=final,
        steps_taken=steps_taken,
        stopped_reason=stopped_reason,
    )


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("joint", help="URDF joint name, e.g. wrist_flex")
    ap.add_argument("--port", required=True)
    ap.add_argument("--direction", choices=["min", "max"], required=True)
    ap.add_argument(
        "--probe-bound-rad", type=float, default=None,
        help="widen to this many radians before probing (default: the URDF's "
        "own limit on that side — never further)",
    )
    ap.add_argument("--apply", action="store_true", help="without this, only backs up and reports")
    ap.add_argument(
        "--backup", type=Path, default=None,
        help="where to write the pre-change EEPROM backup (default: alongside the calibration)",
    )
    ap.add_argument(
        "--dq", type=float, default=_DEFAULT_DQ,
        help=f"commanded speed feedforward, rad/s (default {_DEFAULT_DQ}). Raise "
        "this for a joint fighting gravity — a probe that stalls quickly at "
        "moderate current (tens of mA, not near-zero) has not ruled out "
        "'under-driven' as the reason; try again at a higher --dq before "
        "trusting a 'no_progress' result.",
    )
    ap.add_argument(
        "--settle-s", type=float, default=_SETTLE_S,
        help=f"seconds to wait after each step before reading position/current "
        f"(default {_SETTLE_S})",
    )
    a = ap.parse_args()

    from soarm_tamp.conventions import URDF_LIMITS, calibration_path  # local: soarm_tamp is a sibling package, not a dependency

    cal = RobotCalibration.load(calibration_path())
    try:
        idx = cal.names.index(a.joint)
    except ValueError:
        print(f"unknown joint {a.joint!r}; have {cal.names}", file=sys.stderr)
        sys.exit(2)
    joint = cal.joints[idx]
    servo_id = idx + 1  # servo IDs are 1-indexed, JOINT_ORDER-positional

    if a.probe_bound_rad is None:
        lo, hi = URDF_LIMITS[a.joint]
        probe_rad = hi if a.direction == "max" else lo
    else:
        probe_rad = a.probe_bound_rad
    probe_ticks = int(round(joint.to_ticks(probe_rad)))

    from ..robot.hardware import ServoHardwareInterface

    hw = ServoHardwareInterface(
        port=a.port, calibration=cal, torque_on_start=True,
        state_freq=100.0, verify_eeprom_limits=False,
    )
    hw.start()
    try:
        with hw.lend_bus() as srv:
            def rd2(addr: int) -> int:
                r = srv.read2ByteTxRx(servo_id, addr)
                return r.data[0] if r.data else -1
            backup = {
                "min_angle": rd2(STS_MIN_ANGLE_LIMIT_L),
                "max_angle": rd2(STS_MAX_ANGLE_LIMIT_L),
            }
        backup_path = a.backup or (
            Path(calibration_path()).parent
            / f"servo_eeprom_backup_{a.joint}_{int(time.time())}.json"
        )
        backup_path.write_text(json.dumps({str(servo_id): backup}, indent=2))
        print(f"backed up servo {servo_id} ({a.joint}) -> {backup_path}")
        print(f"  before: {backup}")

        if not a.apply:
            print("\n(dry run — pass --apply to write EEPROM)")
            return

        result = probe_and_set_limit(
            hw, servo_id, joint, direction=a.direction,
            probe_bound_ticks=probe_ticks, joint_index=idx,
            dq_rad_s=a.dq, settle_s=a.settle_s,
        )
        print(
            f"\n{result.joint} {result.direction}: "
            f"{result.before_ticks} -> stop measured at {result.stop_ticks} "
            f"({result.steps_taken} steps, stopped: {result.stopped_reason}) "
            f"-> limit set to {result.final_ticks}"
        )

        with hw.lend_bus() as srv:
            def rd2b(addr: int) -> int:
                r = srv.read2ByteTxRx(servo_id, addr)
                return r.data[0] if r.data else -1
            live_min, live_max = rd2b(STS_MIN_ANGLE_LIMIT_L), rd2b(STS_MAX_ANGLE_LIMIT_L)

        updated = list(cal.joints)
        updated[idx] = joint.with_eeprom_limits(live_min, live_max)
        cal2 = RobotCalibration(
            joints=updated, arm_id=cal.arm_id, validated=cal.validated,
            source=cal.source, created=cal.created, notes=cal.notes,
        )
        cal2.save(calibration_path())
        print(f"calibration updated: eeprom_limits_ticks = ({live_min}, {live_max})")
    finally:
        hw.stop()
        print("bus closed")


if __name__ == "__main__":
    main()
