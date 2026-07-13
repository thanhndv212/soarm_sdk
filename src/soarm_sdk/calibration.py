"""Operation planning and execution for soarm_sdk calibration.

Provides data structures and functions to plan and apply a batch of
servo configuration changes (ID reassignment, angle limits, acceleration,
speed, torque, operating mode, baud rate) over a serial bus.

Typical usage
-------------
>>> from soarm_sdk import apply_plan, OperationPlan, parse_range
>>> plan = OperationPlan(
...     assign_id={},
...     angle_limits={1: (100, 3900)},
...     acceleration={1: 50},
...     speed={},
...     torque={1: True},
...     mode={},
...     baud={},
... )
>>> apply_plan("/dev/tty.usbserial-XXXX", 1_000_000, "1-6", plan, unlock=True, lock=True)

The legacy :func:`run_calibration` function (which accepts an
``argparse.Namespace``) is kept for backward compatibility.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from .bus import scan_servos, write1, write2
from .port_handler import PortHandler
from .sts import sts
from .stservo_def import (
    STS_ACC,
    STS_BAUD_RATE,
    STS_GOAL_SPEED_L,
    STS_ID,
    STS_LOCK,
    STS_MIN_ANGLE_LIMIT_L,
    STS_MAX_ANGLE_LIMIT_L,
    STS_MODE,
    STS_TORQUE_ENABLE,
)


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class OperationPlan:
    """Container for a batch of servo configuration changes.

    All dicts use the *original* (pre-reassignment) servo ID as the key.
    Use :func:`resolve_id` to map IDs through the ``assign_id`` remap chain
    before writing to the bus.

    Attributes
    ----------
    assign_id:
        ``{old_id: new_id}`` — ID reassignments to perform first.
    angle_limits:
        ``{servo_id: (min_ticks, max_ticks)}`` — absolute angle limits.
    acceleration:
        ``{servo_id: acc_value}`` — STS_ACC register value.
    speed:
        ``{servo_id: speed_value}`` — STS_GOAL_SPEED_L register value.
    torque:
        ``{servo_id: enabled}`` — torque enable flag.
    mode:
        ``{servo_id: mode_code}`` — operating mode (0 = servo, 1 = wheel).
    baud:
        ``{servo_id: baud_code}`` — baud-rate register code.
    """

    assign_id: Dict[int, int]
    angle_limits: Dict[int, Tuple[int, int]]
    acceleration: Dict[int, int]
    speed: Dict[int, int]
    torque: Dict[int, bool]
    mode: Dict[int, int]
    baud: Dict[int, int]


# ---------------------------------------------------------------------------
# Parsing helpers
# ---------------------------------------------------------------------------

def parse_range(raw: str) -> range:
    """Parse an ``"START-END"`` string into a :class:`range`.

    Parameters
    ----------
    raw:
        String in the form ``"1-10"`` (both bounds inclusive).

    Returns
    -------
    range
        ``range(start, end + 1)``.

    Raises
    ------
    ValueError
        If *raw* is not well-formed or the bounds are invalid.
    """
    try:
        start_str, end_str = raw.split("-", 1)
        start, end = int(start_str), int(end_str)
    except ValueError as exc:
        raise ValueError(
            f"Invalid range '{raw}'. Expected the form START-END (e.g. 1-10)."
        ) from exc
    if start < 0 or end < 0 or start > end:
        raise ValueError(
            f"Invalid range '{raw}'. Start must be <= end and both >= 0."
        )
    return range(start, end + 1)


def parse_mapping(
    raw_entries: Sequence[str],
    expected_parts: int,
    label: str,
) -> Dict[int, Tuple[int, ...]]:
    """Parse a list of ``"ID:V1:V2:…"`` strings into a mapping.

    Parameters
    ----------
    raw_entries:
        Each string must have exactly *expected_parts* colon-separated fields.
    expected_parts:
        Number of fields (including the leading servo ID).
    label:
        Used in error messages to identify the option type.

    Returns
    -------
    dict[int, tuple[int, ...]]
        ``{servo_id: (v1, v2, …)}``.
    """
    mapping: Dict[int, Tuple[int, ...]] = {}
    for raw in raw_entries:
        parts = raw.split(":")
        if len(parts) != expected_parts:
            raise ValueError(
                f"Invalid {label} '{raw}'. Expected "
                f"{expected_parts} colon-separated values."
            )
        try:
            ints = tuple(int(p, 0) for p in parts)
        except ValueError as exc:
            raise ValueError(
                f"Invalid integer in {label} '{raw}'."
            ) from exc
        mapping[ints[0]] = ints[1:]
    return mapping


def parse_bool_mapping(
    raw_entries: Sequence[str],
    label: str,
) -> Dict[int, bool]:
    """Parse a list of ``"ID:STATE"`` strings where STATE is on/off/1/0/…

    Parameters
    ----------
    raw_entries:
        Entries in the form ``"11:on"`` or ``"12:0"``.
    label:
        Used in error messages.

    Returns
    -------
    dict[int, bool]
        ``{servo_id: True/False}``.
    """
    mapping: Dict[int, bool] = {}
    for raw in raw_entries:
        parts = raw.split(":")
        if len(parts) != 2:
            raise ValueError(
                f"Invalid {label} '{raw}'. Expected the form ID:VALUE "
                "(e.g. 11:on)."
            )
        try:
            servo_id = int(parts[0], 0)
        except ValueError as exc:
            raise ValueError(
                f"Invalid integer ID in {label} '{raw}'."
            ) from exc
        state_str = parts[1].strip().lower()
        if state_str in {"1", "on", "true", "yes", "enable"}:
            state = True
        elif state_str in {"0", "off", "false", "no", "disable"}:
            state = False
        else:
            raise ValueError(
                f"Invalid state '{parts[1]}' in {label}. Use on/off or 1/0."
            )
        mapping[servo_id] = state
    return mapping


def build_operation_plan(args: argparse.Namespace) -> OperationPlan:
    """Construct an :class:`OperationPlan` from a parsed ``argparse.Namespace``.

    Expects the following attributes on *args* (all default to empty lists
    when the corresponding CLI flag is absent):

    * ``assign_id`` — list of ``"OLD:NEW"`` strings
    * ``angle_limit`` — list of ``"ID:MIN:MAX"`` strings
    * ``set_acc`` — list of ``"ID:ACC"`` strings
    * ``set_speed`` — list of ``"ID:SPEED"`` strings
    * ``torque`` — list of ``"ID:STATE"`` strings
    * ``set_mode`` — list of ``"ID:MODE"`` strings
    * ``set_baud`` — list of ``"ID:BAUD_CODE"`` strings
    """
    assign_id: Dict[int, int] = {
        k: v[0]
        for k, v in parse_mapping(args.assign_id, 2, "--assign-id").items()
    }
    angle_limits = {
        k: (v[0], v[1])
        for k, v in parse_mapping(args.angle_limit, 3, "--angle-limit").items()
    }
    acceleration = {
        k: v[0]
        for k, v in parse_mapping(args.set_acc, 2, "--set-acc").items()
    }
    speed = {
        k: v[0]
        for k, v in parse_mapping(args.set_speed, 2, "--set-speed").items()
    }
    mode = {
        k: v[0]
        for k, v in parse_mapping(args.set_mode, 2, "--set-mode").items()
    }
    baud = {
        k: v[0]
        for k, v in parse_mapping(args.set_baud, 2, "--set-baud").items()
    }
    torque = parse_bool_mapping(args.torque, "--torque")
    return OperationPlan(assign_id, angle_limits, acceleration, speed, torque, mode, baud)


# ---------------------------------------------------------------------------
# ID resolution helpers
# ---------------------------------------------------------------------------

def resolve_id(remap: Dict[int, int], servo_id: int) -> int:
    """Follow the ID remap chain to find the final ID for *servo_id*.

    Parameters
    ----------
    remap:
        Mapping of ``old_id → new_id`` built up as ID reassignments are applied.
    servo_id:
        The original (pre-reassignment) servo ID.

    Returns
    -------
    int
        The ID that *servo_id* eventually maps to (or *servo_id* itself if not
        in *remap*). Cycle-safe.
    """
    seen: set[int] = set()
    current = servo_id
    while current in remap and current not in seen:
        seen.add(current)
        current = remap[current]
    return current


def collect_final_ids(plan: OperationPlan, remap: Dict[int, int]) -> List[int]:
    """Return sorted list of final IDs affected by non-assign-id operations."""
    ids: set[int] = set()
    for source in (
        *plan.angle_limits,
        *plan.acceleration,
        *plan.speed,
        *plan.torque,
        *plan.mode,
        *plan.baud,
    ):
        ids.add(resolve_id(remap, source))
    return sorted(ids)


# ---------------------------------------------------------------------------
# Plan execution
# ---------------------------------------------------------------------------

def apply_plan(
    device: str,
    baud: int,
    id_range_str: str,
    plan: OperationPlan,
    *,
    unlock: bool = False,
    lock: bool = False,
    log: Optional[Callable[[str], None]] = None,
) -> int:
    """Apply *plan* to the servos on *device*.

    This is the primary API for programmatic use. It opens and closes the
    serial port internally, so callers do not manage the connection.

    Parameters
    ----------
    device:
        Serial port path.
    baud:
        Bus baud rate.
    id_range_str:
        Scan range string in ``"START-END"`` form (e.g. ``"1-6"``).
    plan:
        :class:`OperationPlan` describing all changes to apply.
    unlock:
        Unlock EEPROM on affected servos before writing.
    lock:
        Lock EEPROM on affected servos after writing.
    log:
        Callable used for progress messages. Defaults to :func:`print`.

    Returns
    -------
    int
        ``0`` on success.

    Raises
    ------
    RuntimeError
        If the port cannot be opened.
    """
    logger = log or print
    id_range = parse_range(id_range_str)

    port_handler = PortHandler(device)
    packet_handler = sts(port_handler)

    try:
        if not port_handler.openPort():
            raise RuntimeError(f"Failed to open serial port {device}.")
        if not port_handler.setBaudRate(baud):
            raise RuntimeError(f"Failed to set baudrate to {baud}.")

        logger(f"Scanning IDs {id_range.start}–{id_range.stop - 1}…")
        detected = scan_servos(packet_handler, id_range)
        if detected:
            for sid, model in sorted(detected.items()):
                logger(f"  Found ID {sid:3d} (model {model})")
        else:
            logger("No servos discovered in the specified range.")

        remap: Dict[int, int] = {}

        # --- ID reassignments ---
        for current_id, new_id in plan.assign_id.items():
            active_id = resolve_id(remap, current_id)
            logger(f"Reassigning ID {active_id} → {new_id}…")
            if unlock:
                try:
                    write1(packet_handler, active_id, STS_LOCK, 0, "unlock")
                except RuntimeError as exc:
                    logger(f"  Warning: {exc}")
            try:
                write1(packet_handler, active_id, STS_ID, new_id, "ID")
            except RuntimeError as exc:
                logger(f"  Error: {exc}")
                continue
            remap[active_id] = new_id
            if active_id in detected:
                detected[new_id] = detected.pop(active_id)
            logger("  Success")

        target_ids = collect_final_ids(plan, remap)

        if unlock and target_ids:
            logger("Unlocking EEPROM for target servos…")
            for sid in target_ids:
                try:
                    write1(packet_handler, resolve_id(remap, sid), STS_LOCK, 0, "unlock")
                except RuntimeError as exc:
                    logger(f"  Warning: {exc}")

        # --- Angle limits ---
        for sid, (min_angle, max_angle) in plan.angle_limits.items():
            actual = resolve_id(remap, sid)
            logger(f"Setting angle limits for ID {actual}: min={min_angle}, max={max_angle}")
            if min_angle < 0 or max_angle < 0 or min_angle > max_angle:
                logger("  Skipping: invalid angle limits.")
                continue
            try:
                write2(packet_handler, actual, STS_MIN_ANGLE_LIMIT_L, min_angle, "min angle limit")
                write2(packet_handler, actual, STS_MAX_ANGLE_LIMIT_L, max_angle, "max angle limit")
            except RuntimeError as exc:
                logger(f"  Error: {exc}")

        # --- Acceleration ---
        for sid, acc in plan.acceleration.items():
            actual = resolve_id(remap, sid)
            logger(f"Setting acceleration for ID {actual}: {acc}")
            try:
                write1(packet_handler, actual, STS_ACC, acc, "acceleration")
            except RuntimeError as exc:
                logger(f"  Error: {exc}")

        # --- Speed ---
        for sid, speed in plan.speed.items():
            actual = resolve_id(remap, sid)
            logger(f"Setting speed for ID {actual}: {speed}")
            try:
                write2(packet_handler, actual, STS_GOAL_SPEED_L, speed, "speed")
            except RuntimeError as exc:
                logger(f"  Error: {exc}")

        # --- Torque ---
        for sid, state in plan.torque.items():
            actual = resolve_id(remap, sid)
            logger(f"Setting torque for ID {actual}: {'on' if state else 'off'}")
            try:
                write1(packet_handler, actual, STS_TORQUE_ENABLE, int(state), "torque")
            except RuntimeError as exc:
                logger(f"  Error: {exc}")

        # --- Mode ---
        for sid, mode_code in plan.mode.items():
            actual = resolve_id(remap, sid)
            logger(f"Setting mode for ID {actual}: {mode_code}")
            try:
                write1(packet_handler, actual, STS_MODE, mode_code, "mode")
            except RuntimeError as exc:
                logger(f"  Error: {exc}")

        # --- Baud ---
        for sid, baud_code in plan.baud.items():
            actual = resolve_id(remap, sid)
            logger(f"Setting baud code for ID {actual}: {baud_code}")
            try:
                write1(packet_handler, actual, STS_BAUD_RATE, baud_code, "baud rate")
            except RuntimeError as exc:
                logger(f"  Error: {exc}")

        # --- EEPROM lock ---
        if lock and (plan.assign_id or target_ids):
            logger("Locking EEPROM after calibration…")
            lock_targets: set[int] = set(remap.values()) if remap else set()
            lock_targets.update(target_ids)
            for sid in sorted(lock_targets):
                try:
                    write1(packet_handler, resolve_id(remap, sid), STS_LOCK, 1, "lock")
                except RuntimeError as exc:
                    logger(f"  Warning: {exc}")

    finally:
        port_handler.closePort()

    return 0


def run_calibration(
    args: argparse.Namespace,
    log: Optional[Callable[[str], None]] = None,
) -> int:
    """Backward-compatible wrapper: build a plan from *args* and apply it.

    Parameters
    ----------
    args:
        Parsed :class:`argparse.Namespace` as produced by the
        ``calibrate.py`` CLI parser.
    log:
        Progress message callable. Defaults to :func:`print`.

    Returns
    -------
    int
        ``0`` on success.
    """

    logger = log or print
    if getattr(args, "list_ports", False):
        from .bus import print_ports
        print_ports(log=logger)

    plan = build_operation_plan(args)
    return apply_plan(
        args.device,
        args.baud,
        args.scan_range,
        plan,
        unlock=args.unlock,
        lock=args.lock,
        log=logger,
    )
