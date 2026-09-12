"""Re-centre a servo's homing offset so its travel fits inside the encoder.

The problem this solves
-----------------------
``Present_Position`` is 12 bits — it wraps at 4095/0 — and the servo adds its
EEPROM homing offset (``STS_OFS``) before the value reaches the bus. If that
offset places a joint's mechanical travel across the wrap point, the joint
reports, say, 3900 → 4095 → 0 → 150 as it moves steadily in one direction.

:func:`~soarm_sdk.calibration.rom_sweep.run_rom_sweep` tracks a plain min and
max, so a wrapped joint measures as ``0..4095`` — the encoder's range, not the
joint's. Worse, :class:`~soarm_sdk.calibration.frame.JointCalibration` maps
ticks to radians with a single linear expression, and *no* linear map fits a
coordinate that jumps mid-travel. The joint is uncalibratable until the offset
moves.

The measurement here differs from the plain sweep in one respect: it
accumulates *unwrapped* displacement, treating any jump larger than half the
encoder as a wrap rather than as motion. That gives the true travel — and
therefore the true midpoint — even when the reported position is
discontinuous. The offset that puts that midpoint at 2048 is then arithmetic.

Why the angle limits are rewritten too
--------------------------------------
The sign convention is the trap here: the servo reports ``raw - STS_OFS``, so
the offset moves the reported frame *opposite* to the direction one expects.

``STS_MIN/MAX_ANGLE_LIMIT`` are expressed in the same reported frame as the
position, so changing the offset without moving them leaves the servo
enforcing a band that no longer corresponds to the physical stops it was
derived from. They are rewritten to the freshly measured travel, which also
fixes the more dangerous version of that bug: a limit *wider* than the real
stop lets the servo drive into the mechanism and stall there.
"""

from __future__ import annotations

import time
from collections import deque
from typing import Any, Callable, Dict, Tuple

from ..protocol.registers import (
    COMM_SUCCESS,
    STS_LOCK,
    STS_MAX_ANGLE_LIMIT_L,
    STS_MIN_ANGLE_LIMIT_L,
    STS_MODE,
    STS_OFS_L,
    STS_TORQUE_ENABLE,
)

__all__ = [
    "centring_offset",
    "measure_travel_range",
    "measure_unwrapped_travel",
    "recentre_joint",
    "decode_ofs",
    "encode_ofs",
]

ENCODER_TICKS = 4096
CENTRE_TICKS = 2048
_POLL_S = 0.08

# STS_OFS is sign-magnitude: 11 bits of value, bit 11 is the sign.
_OFS_SIGN_BIT = 0x800
_OFS_MAX = 0x7FF


def decode_ofs(raw: int) -> int:
    """Signed homing offset from the register's sign-magnitude encoding."""
    return -(raw & _OFS_MAX) if raw & _OFS_SIGN_BIT else raw & _OFS_MAX


def encode_ofs(value: int) -> int:
    """Sign-magnitude encoding for the register, wrapping into representable range."""
    # The offset is modular — adding a full encoder turn is a no-op — so an
    # out-of-range value is folded rather than rejected.
    value = ((value + CENTRE_TICKS) % ENCODER_TICKS) - CENTRE_TICKS
    if value < 0:
        return (abs(value) & _OFS_MAX) | _OFS_SIGN_BIT
    return value & _OFS_MAX


def centring_offset(
    zero: float,
    pos_min: float,
    pos_max: float,
    target_ref: int = CENTRE_TICKS,
) -> Tuple[int, int, int]:
    """``(STS_OFS, min_angle_limit, max_angle_limit)`` putting *zero* at *target_ref*.

    All three inputs are raw-frame ticks, as a wheel-mode sweep reports them.

    The servo reports ``raw - STS_OFS``, so the offset that makes *zero* read
    back as *target_ref* is ``zero - target_ref`` — the subtraction runs the
    opposite way to the intuition, and the wrong sign is silent: it shifts the
    joint's frame by twice the error instead of cancelling it. The angle
    limits are in the reported frame, hence ``- off`` rather than ``+ off``.
    """
    off = int(round(zero - target_ref))
    lo = max(0, min(ENCODER_TICKS - 1, int(round(pos_min - off))))
    hi = max(0, min(ENCODER_TICKS - 1, int(round(pos_max - off))))
    return off, lo, hi


def _read_word(srv: Any, sid: int, addr: int) -> int:
    res = srv.read2ByteTxRx(sid, addr)
    data = res.data if hasattr(res, "data") else res
    if isinstance(data, list):
        return data[0] | (data[1] << 8) if len(data) > 1 else data[0]
    return int(data)


def _settled_position(
    srv: Any,
    sid: int,
    *,
    settle_s: float = 0.35,
    tol: int = 2,
    need: int = 3,
    tries: int = 30,
) -> int:
    """Read the position once the servo agrees with itself.

    A single read taken right after a mode switch or an EEPROM write can come
    back in the frame the servo was using a moment ago — wheel mode reports
    the raw encoder, servo mode applies the homing offset, and the changeover
    is not instantaneous. Anchoring a whole calibration to one such read puts
    every tick out by the offset, so this waits for *need* consecutive reads
    within *tol* ticks of each other before believing any of them.
    """
    srv.write1ByteTxRx(sid, STS_MODE, 0)
    time.sleep(settle_s)
    recent: deque = deque(maxlen=need)
    for _ in range(tries):
        p, r, _ = srv.ReadPos(sid)
        if r == COMM_SUCCESS:
            recent.append(p)
            if len(recent) == need and max(recent) - min(recent) <= tol:
                return int(recent[-1])
        time.sleep(0.05)
    raise RuntimeError(
        f"J{sid}: position never settled — read {list(recent)} over "
        f"{tries} attempts. The joint may still be moving, or the bus is noisy."
    )


def _unwrap(delta: int) -> int:
    """Interpret a jump larger than half the encoder as a wrap, not as motion."""
    if delta > CENTRE_TICKS:
        return delta - ENCODER_TICKS
    if delta < -CENTRE_TICKS:
        return delta + ENCODER_TICKS
    return delta


def _sweep_one_direction(
    srv: Any,
    sid: int,
    speed: int,
    stall_thr: int,
    stall_win: int,
    timeout_s: float,
) -> Tuple[float, bool]:
    """Drive until the joint stops moving; return unwrapped displacement.

    Displacement is relative to wherever the joint started, in ticks, and is
    accumulated across encoder wraps.
    """
    srv.WriteSpec(sid, speed, 5)
    prev, res, _ = srv.ReadPos(sid)
    if res != COMM_SUCCESS:
        prev = 0
    travelled = 0.0
    extreme = 0.0
    recent: deque = deque(maxlen=stall_win)
    min_travel = max(stall_thr * 4, 30)
    t0 = time.time()
    stalled = False
    while time.time() - t0 < timeout_s:
        p, r, _ = srv.ReadPos(sid)
        if r == COMM_SUCCESS:
            travelled += _unwrap(p - prev)
            prev = p
            if abs(travelled) > abs(extreme):
                extreme = travelled
            if abs(travelled) >= min_travel:
                recent.append(travelled)
        if len(recent) >= stall_win and (max(recent) - min(recent)) <= stall_thr:
            stalled = True
            break
        time.sleep(_POLL_S)
    srv.WriteSpec(sid, 0, 5)
    time.sleep(0.4)
    return extreme, stalled


def measure_unwrapped_travel(
    srv: Any,
    sid: int,
    *,
    speed: int,
    stall_thr: int,
    stall_win: int,
    timeout_s: float,
    log_fn: Callable[[str], None] = print,
) -> Dict[str, Any]:
    """Drive *sid* to both hard stops, accumulating displacement across wraps.

    Returns the travel relative to the starting position, so it is meaningful
    even when the absolute reported position is discontinuous.
    """
    # Read the anchor *before* entering wheel mode. While a joint is driving
    # in wheel mode the servo reports the raw encoder, with STS_OFS not
    # applied — the position jumps by exactly the offset the moment motion
    # starts. Displacements are unaffected (a delta is a delta in either
    # frame), so the travel is anchored to a servo-mode reading and measured
    # with deltas from there.
    start = _settled_position(srv, sid)

    srv.write1ByteTxRx(sid, STS_TORQUE_ENABLE, 1)
    srv.write1ByteTxRx(sid, STS_LOCK, 0)
    srv.WheelMode(sid)

    log_fn(f"**J{sid}** unwrapped sweep → positive…")
    up, stalled_fwd = _sweep_one_direction(srv, sid, speed, stall_thr, stall_win, timeout_s)
    log_fn(f"**J{sid}** (+) {'stalled' if stalled_fwd else 'timed-out'} at {up:+.0f} ticks from start")

    log_fn(f"**J{sid}** unwrapped sweep ← negative…")
    down, stalled_rev = _sweep_one_direction(srv, sid, -speed, stall_thr, stall_win, timeout_s)
    # `down` is measured from the positive stop, so fold it back to the start.
    low = up + down
    log_fn(f"**J{sid}** (−) {'stalled' if stalled_rev else 'timed-out'} at {low:+.0f} ticks from start")

    srv.write1ByteTxRx(sid, STS_MODE, 0)
    return {
        "start": int(start),
        "rel_min": min(0.0, low),
        "rel_max": max(0.0, up),
        "span": abs(up - low),
        "stalled_fwd": stalled_fwd,
        "stalled_rev": stalled_rev,
    }


def recentre_joint(
    srv: Any,
    sid: int,
    *,
    speed: int,
    stall_thr: int,
    stall_win: int,
    timeout_s: float,
    hold: bool = True,
    log_fn: Callable[[str], None] = print,
) -> Dict[str, Any]:
    """Measure *sid*'s travel and move its homing offset so the midpoint reads 2048.

    The EEPROM write happens with torque **off**, because ``STS_OFS`` changes
    the position the servo believes it is at and a powered servo would drive
    straight to its old goal in the new frame. Torque is then restored with
    the goal set to where the joint actually is, so it holds station: leaving
    a gravity-loaded joint limp would drop the arm the moment the offset lands.
    Pass ``hold=False`` for a joint that is safer slack than powered.
    """
    travel = measure_unwrapped_travel(
        srv, sid, speed=speed, stall_thr=stall_thr, stall_win=stall_win,
        timeout_s=timeout_s, log_fn=log_fn,
    )
    if travel["span"] >= ENCODER_TICKS - 1:
        raise RuntimeError(
            f"J{sid}: measured {travel['span']:.0f} ticks of travel, a full turn "
            "or more. A continuously rotating joint has no centre to move to."
        )

    old_raw = _read_word(srv, sid, STS_OFS_L)
    old_ofs = decode_ofs(old_raw)
    # Midpoint in the frame the servo reports today — virtual, because it may
    # sit outside 0..4095 precisely when the travel wraps.
    midpoint = travel["start"] + (travel["rel_min"] + travel["rel_max"]) / 2.0
    # The servo SUBTRACTS its homing offset: reported = raw - STS_OFS. Measured
    # on an STS3215 by stepping the register and watching the reported position
    # move the other way (+100 offset -> -100 reported). Getting this backwards
    # does not fail loudly — it moves the joint's frame the wrong way and
    # doubles the very error the re-centring is meant to remove.
    new_ofs = old_ofs + int(round(midpoint - CENTRE_TICKS))
    half = travel["span"] / 2.0
    new_min = int(round(CENTRE_TICKS - half))
    new_max = int(round(CENTRE_TICKS + half))

    # Torque off before the frame moves under the servo's feet.
    srv.write1ByteTxRx(sid, STS_TORQUE_ENABLE, 0)
    time.sleep(0.2)
    srv.write1ByteTxRx(sid, STS_LOCK, 0)
    srv.write2ByteTxRx(sid, STS_OFS_L, encode_ofs(new_ofs))
    srv.write2ByteTxRx(sid, STS_MIN_ANGLE_LIMIT_L, max(0, new_min))
    srv.write2ByteTxRx(sid, STS_MAX_ANGLE_LIMIT_L, min(ENCODER_TICKS - 1, new_max))
    srv.write1ByteTxRx(sid, STS_LOCK, 1)
    time.sleep(0.2)

    readback_raw = _read_word(srv, sid, STS_OFS_L)
    now = _settled_position(srv, sid)

    if hold:
        # Goal first, then torque — the other order drives to a stale goal.
        srv.WritePosEx(sid, int(now), max(1, speed), 20)
        srv.write1ByteTxRx(sid, STS_TORQUE_ENABLE, 1)
        time.sleep(0.2)
    log_fn(
        f"**J{sid}** offset {old_ofs:+d} → {decode_ofs(readback_raw):+d}, "
        f"limits {max(0, new_min)}..{min(4095, new_max)}, now reading {now}"
    )
    return {
        "old_offset": old_ofs,
        "new_offset": decode_ofs(readback_raw),
        "requested_offset": new_ofs,
        "angle_limits": [max(0, new_min), min(ENCODER_TICKS - 1, new_max)],
        "span_ticks": travel["span"],
        "position_after": int(now),
        "holding": hold,
        "stalled_both_ends": bool(travel["stalled_fwd"] and travel["stalled_rev"]),
    }


def measure_travel_range(
    srv: Any,
    sid: int,
    *,
    speed: int,
    stall_thr: int,
    stall_win: int,
    timeout_s: float,
    park: bool = True,
    log_fn: Callable[[str], None] = print,
) -> Dict[str, Any]:
    """Measure one joint's travel as absolute ticks in the servo-mode frame.

    Returns the same shape as
    :func:`~soarm_sdk.calibration.rom_sweep.run_rom_sweep` produces per joint,
    so it is a drop-in for calibration — but it is measured differently, and
    the difference matters.

    ``run_rom_sweep`` records the smallest and largest *reported* position it
    sees. Two things break that. The reported position wraps at 4095/0, so a
    joint whose travel crosses the boundary measures as the encoder's range
    rather than its own. And a driving joint in wheel mode reports the raw
    encoder — the homing offset that ``ServoRobot`` sees at runtime is absent
    — so even an unwrapped result is in the wrong frame by that offset.

    Accumulating displacement sidesteps both: a displacement means the same
    thing in either frame, and a jump larger than half the encoder is read as
    a wrap rather than as motion. Anchoring the total to a servo-mode reading
    taken before the joint moves puts the answer back in the frame the runtime
    uses.
    """
    travel = measure_unwrapped_travel(
        srv, sid, speed=speed, stall_thr=stall_thr, stall_win=stall_win,
        timeout_s=timeout_s, log_fn=log_fn,
    )
    lo = int(round(travel["start"] + travel["rel_min"]))
    hi = int(round(travel["start"] + travel["rel_max"]))
    zero = (lo + hi) // 2

    fits = 0 <= lo and hi <= ENCODER_TICKS - 1
    if park and fits:
        # Back in servo mode (measure_unwrapped_travel restored it), park at
        # the middle of the measured travel so the joint ends nowhere near a
        # stop — and holds, rather than being left limp. Skipped when the
        # range did not fit the encoder: the midpoint is meaningless then, and
        # commanding it would drive the joint somewhere arbitrary.
        srv.WritePosEx(sid, zero, max(1, speed), 20)
        srv.write1ByteTxRx(sid, STS_TORQUE_ENABLE, 1)
        time.sleep(0.3)
    elif park:
        log_fn(f"**J{sid}** not parked — measured range does not fit the encoder")

    log_fn(
        f"**J{sid}** travel {lo}..{hi} ({hi - lo} ticks), midpoint {zero}"
        + ("" if lo >= 0 and hi <= ENCODER_TICKS - 1
           else "  <- outside 0..4095, needs --recentre")
    )
    return {
        "pos_min": lo,
        "pos_max": hi,
        "zero": zero,
        "range_ticks": hi - lo,
        "stalled_fwd": travel["stalled_fwd"],
        "stalled_rev": travel["stalled_rev"],
        "anchor": travel["start"],
    }
