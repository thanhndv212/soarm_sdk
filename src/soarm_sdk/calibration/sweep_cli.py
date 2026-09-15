#!/usr/bin/env python3
"""Measure an arm's travel on the hardware, then write its URDF-frame calibration.

    soarm-calibrate-rom --arm-id thanh_arm

RUNS ON THE HOST, with the arm connected and free to move.

This measures the travel itself, rather than borrowing it from another
tool's file: it drives every joint into both of its mechanical hard stops
(:func:`~soarm_sdk.calibration.rom_sweep.run_rom_sweep`) and feeds the
result to :func:`~soarm_sdk.calibration.frame.seed_from_travel`. A borrowed
travel range is a measurement of whatever produced that file, not of this
arm, which is why there is no offline "seed from an existing calibration"
path — only this sweep, or the guided ``soarm-dashboard-calibration``
workflow that wraps it.

Why the sweep is the honest starting point
------------------------------------------
The arm's pose when you plug it in says nothing — a servo reports ticks
against whatever ``Homing_Offset`` is in its EEPROM, and a joint sitting at
a random angle is indistinguishable from one sitting at zero. Only the hard
stops are a physical reference both the servo and the URDF can name, which
is why the calibration is derived from them rather than from a "put the arm
in the home pose and press enter" step.

What this still cannot settle
-----------------------------
The direction signs. A travel range is a min and a max; it does not say
which physical end is the URDF's lower limit. They are assumed +1 and the
file is written ``validated: false`` — see
:mod:`soarm_sdk.calibration.frame`. Confirm them on the arm with
``python -m soarm_tamp.validate_calibration`` before anything streams a
planned trajectory against this file.
"""

from __future__ import annotations

import argparse
import sys
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Dict, Generator, List, Optional, Sequence, Tuple

from ..bus.discovery import discover_servos, list_ports, read_diagnostics
from ..protocol.port_handler import PortHandler
from ..protocol.sts import sts as _Sts
from ..robot.base import load_robot_config
from .frame import RobotCalibration, seed_from_travel
from .recentre import ENCODER_TICKS, measure_travel_range, recentre_joint
from .rom_sweep import simulate_rom_sweep

__all__ = ["main"]

DEFAULT_OUT = Path.home() / ".soarm_sdk" / "calibration.json"

# Same values the dashboard's Homing Wizard ships with — the sweep is the
# same call, so a CLI run and a browser run should measure the same travel.
DEFAULT_SPEED = 150       # ticks/s
DEFAULT_STALL_THR = 5     # ticks of movement that still counts as stalled
DEFAULT_STALL_WIN = 8     # consecutive samples inside that threshold
DEFAULT_TIMEOUT_S = 30.0  # per direction, per joint


@contextmanager
def _bus(device: str, baud: int) -> Generator[_Sts, None, None]:
    ph = PortHandler(device)
    try:
        if not ph.openPort():
            raise RuntimeError(f"cannot open port {device!r}")
        if not ph.setBaudRate(baud):
            raise RuntimeError(f"cannot set baud rate {baud}")
        yield _Sts(ph)
    finally:
        ph.closePort()


def _resolve_device(requested: str) -> str:
    """Pick the serial port, refusing to guess when the answer is ambiguous."""
    if requested != "auto":
        return requested
    ports = list_ports()
    if not ports:
        raise RuntimeError(
            "no USB serial port found — the arm is not enumerated on this "
            "machine.\n"
            "  Check: USB cable seated, servo bus powered (12 V), and the "
            "CH340/CP210x driver installed.\n"
            "  Then re-run, or pass --device explicitly."
        )
    if len(ports) > 1:
        listing = "\n".join(f"    {d}  ({desc})" for d, desc in ports)
        raise RuntimeError(
            f"{len(ports)} USB serial ports present; pass --device to say "
            f"which is the arm:\n{listing}"
        )
    device, description = ports[0]
    print(f"port           : {device}  ({description})")
    return device


def _check_servos(device: str, baud: int, servo_ids: Sequence[int]) -> None:
    """Refuse to sweep unless every joint answers — a partial sweep is worse
    than none, since it leaves some joints measured and some not."""
    found = discover_servos(device, baud, servo_ids)
    missing = [sid for sid in servo_ids if sid not in found]
    if missing:
        raise RuntimeError(
            f"servo(s) {missing} did not respond on {device} at {baud} baud "
            f"(found {sorted(found)}).\n"
            "  Check bus power and daisy-chain wiring before calibrating."
        )
    print(f"servos         : {sorted(found)} — all responding")


# Travel is measured by accumulating displacement, so a range that falls
# outside 0..4095 is not a measurement artefact — it says the joint's stops
# genuinely straddle the encoder boundary in the frame the runtime reads, and
# no linear tick->radian map covers it until the homing offset moves.


def _sweep_warnings(
    results: Dict[int, dict],
    id_to_name: Dict[int, str],
    span_ratio: Dict[str, float],
) -> Dict[str, List[str]]:
    """Reasons a measured range may not be the joint's real travel.

    Keyed by joint name so a partial re-sweep replaces only that joint's
    warnings — a flat list would lose the ones raised by earlier runs.

    A direction that timed out never reached a hard stop, so that endpoint is
    wherever the joint happened to be when the clock ran out. A range that
    spans almost the whole encoder probably wrapped instead."""
    warnings: Dict[str, List[str]] = {}
    for sid, d in sorted(results.items()):
        name = id_to_name.get(sid, f"J{sid}")
        found: List[str] = []
        ends = [
            end
            for end, ok in (("+", d["stalled_fwd"]), ("-", d["stalled_rev"]))
            if not ok
        ]
        if ends:
            found.append(
                f"{'/'.join(ends)} direction timed out without stalling — "
                "that endpoint is not a hard stop"
            )
        if d["pos_min"] < 0 or d["pos_max"] > ENCODER_TICKS - 1:
            found.append(
                f"travel {d['pos_min']}..{d['pos_max']} falls outside the encoder's "
                "0..4095. The joint's stops straddle the wrap point, so no linear "
                "tick->radian mapping fits it. Re-run this joint with --recentre."
            )
        if d["range_ticks"] >= ENCODER_TICKS - 1:
            found.append(
                f"measured {d['range_ticks']} ticks — a full turn or more, so the "
                "position is ambiguous on a single-turn encoder"
            )
        # An explicit empty list still matters: it is how a clean re-sweep
        # clears a warning an earlier run recorded for this joint.
        warnings[name] = found
    return warnings


def _pre_sweep_state(
    device: str, baud: int, servo_ids: Sequence[int], id_to_name: Dict[int, str]
) -> Dict[str, Any]:
    """Snapshot what the servos say before anything moves.

    ``Correction`` is the servo's EEPROM homing offset, applied *before* the
    position reaches the bus — so every tick in this calibration is measured
    against it. Re-running lerobot's calibration rewrites it and silently
    invalidates this file, which is only detectable if the values were
    recorded. Voltage matters for a different reason: a sagging supply means
    less torque, and a joint that stops short against gravity is recorded as
    a hard stop.
    """
    diag = read_diagnostics(device, baud, servo_ids)
    state: Dict[str, Any] = {}
    for sid in servo_ids:
        row = diag.get(sid, {})
        state[id_to_name[sid]] = {
            "correction": row.get("Correction"),
            "position": row.get("Position"),
            "voltage": row.get("Voltage"),
            "temperature": row.get("Temperature"),
        }
    return state


# STS3215s are specified to 12.6 V and lose torque as the rail sags. Below
# this, a joint that has to lift the arm can stop short of its hard stop and
# be recorded as having reached it — a wrong travel range, not a failure.
_LOW_VOLTAGE_V = 10.0


def _print_pre_state(state: Dict[str, Any]) -> None:
    print()
    print(f"{'joint':16s}{'position':>10}{'correction':>12}{'volts':>8}{'degC':>7}")
    for name, row in state.items():
        pos = row["position"]
        corr = row["correction"]
        volts = row["voltage"]
        temp = row["temperature"]
        print(
            f"{name:16s}{pos if pos is not None else '—':>10}"
            f"{corr if corr is not None else '—':>12}"
            f"{volts if volts is not None else '—':>8}"
            f"{temp if temp is not None else '—':>7}"
        )
    low = [
        (n, r["voltage"]) for n, r in state.items()
        if r["voltage"] is not None and r["voltage"] < _LOW_VOLTAGE_V
    ]
    if low:
        worst = min(v for _, v in low)
        print()
        print(f"  ! bus voltage {worst} V is below {_LOW_VOLTAGE_V} V. The servos")
        print("    have less torque than they are rated for, so a gravity-loaded")
        print("    joint may stall before its hard stop and record a short range.")
        print("    Check the span ratios below against the URDF before trusting it.")


def _print_report(cal: RobotCalibration, out: Path) -> None:
    print("=" * 72)
    print(f"Measured URDF-frame calibration for '{cal.arm_id}'")
    print("=" * 72)
    print(f"{'joint':16s}{'zero tick':>11}{'sign':>6}{'seed +/-':>11}{'span ratio':>12}")
    for j in cal.joints:
        flag = "  <- check" if j.suspect else ""
        print(
            f"{j.name:16s}{j.zero_offset_ticks:>11.1f}{j.direction_sign:>+6d}"
            f"{j.seed_residual_rad:>10.3f}r{j.span_ratio:>12.3f}{flag}"
        )
    print("-" * 72)
    standing = cal.notes.get("sweep_warnings") or {}
    if standing:
        print("UNUSABLE JOINTS        : the rows above are not trustworthy for —")
        for name, msgs in standing.items():
            for msg in msgs:
                print(f"  ! {name}: {msg}")
    if cal.notes.get("incomplete"):
        print(f"INCOMPLETE             : {cal.notes['incomplete']}")
        print("  This file cannot drive the arm until every joint is swept.")
    print(f"worst seed uncertainty : {cal.worst_seed_residual_rad:.3f} rad")
    if cal.suspect_joints:
        print(f"span mismatch          : {', '.join(cal.suspect_joints)}")
        print("  Measured travel and the URDF disagree about range on these.")
        print("  The gearing is fixed at 4096 ticks/rev, so the discrepancy is")
        print("  in the URDF's (conservative) limits or in the sweep itself.")
    print(f"written                : {out}")
    print()
    print("validated: FALSE. The direction signs are assumed +1 and cannot be")
    print("recovered from a travel range. Confirm them on the arm:")
    print("    python -m soarm_tamp.validate_calibration --port <device>")
    print("=" * 72)


def _select_joints(
    raw: str | None, names: Sequence[str], servo_ids: Sequence[int]
) -> List[int]:
    """Resolve ``--joints`` to servo IDs, accepting either names or IDs."""
    if raw is None:
        return list(servo_ids)
    by_name = dict(zip(names, servo_ids))
    selected: List[int] = []
    for token in (t.strip() for t in raw.split(",") if t.strip()):
        if token in by_name:
            selected.append(by_name[token])
        elif token.isdigit() and int(token) in servo_ids:
            selected.append(int(token))
        else:
            raise SystemExit(
                f"--joints: {token!r} is neither a joint name nor a servo ID.\n"
                f"  names: {', '.join(names)}\n"
                f"  IDs  : {', '.join(str(i) for i in servo_ids)}"
            )
    if not selected:
        raise SystemExit("--joints selected nothing")
    # Sweep in servo-ID order regardless of how they were listed, so the
    # physical sequence is predictable to whoever is watching the arm.
    return [sid for sid in servo_ids if sid in set(selected)]


def _parse_args(argv: Sequence[str] | None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--device", default="auto",
                    help="serial port, or 'auto' to discover a single one")
    ap.add_argument("--baud", type=int, default=1_000_000, help="bus baudrate")
    ap.add_argument("--config", default="so101",
                    help="robot config supplying joint names, IDs and URDF limits")
    ap.add_argument("--arm-id", default=None, help="name for this physical arm")
    ap.add_argument("--joints", default=None,
                    help="sweep only these joints (names or servo IDs, comma-separated). "
                         "The rest are carried over from the existing --out file, so an "
                         "arm can be calibrated a few joints at a time.")
    ap.add_argument("--out", default=str(DEFAULT_OUT), help="where to write")
    ap.add_argument("--speed", type=int, default=DEFAULT_SPEED,
                    help="sweep speed in ticks/s")
    ap.add_argument("--stall-threshold", type=int, default=DEFAULT_STALL_THR,
                    help="movement (ticks) within the stall window that still counts as stopped")
    ap.add_argument("--stall-window", type=int, default=DEFAULT_STALL_WIN,
                    help="consecutive samples that must sit inside the threshold")
    ap.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S,
                    help="give up on one direction after this many seconds")
    ap.add_argument("--recentre", "--recenter", action="store_true", dest="recentre",
                    help="before sweeping, move each selected joint's homing offset so "
                         "its travel is centred at tick 2048, and reset its angle limits "
                         "to match. Writes servo EEPROM. Needed when a joint's travel "
                         "wraps past 4095/0, which makes it uncalibratable.")
    ap.add_argument("--yes", action="store_true",
                    help="skip the confirmation prompt")
    ap.add_argument("--dry-run", action="store_true",
                    help="simulate the sweep; writes a file, touches no hardware")
    return ap.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)

    config: Dict[str, Any] = load_robot_config(args.config)
    names: List[str] = list(config["joint_names"])
    servo_ids: List[int] = list(config["hardware"]["servo_ids"])
    urdf_limits: List[Tuple[float, float]] = list(
        zip(config["joint_limits_lower"], config["joint_limits_upper"])
    )
    if not (len(names) == len(servo_ids) == len(urdf_limits)):
        raise SystemExit(
            f"config {args.config!r} is inconsistent: {len(names)} joint names, "
            f"{len(servo_ids)} servo IDs, {len(urdf_limits)} limit pairs"
        )
    id_to_name = dict(zip(servo_ids, names))
    limit_of = dict(zip(names, urdf_limits))

    sweep_ids = _select_joints(args.joints, names, servo_ids)
    partial = len(sweep_ids) < len(servo_ids)
    out = Path(args.out).expanduser()

    print(f"config         : {args.config} ({len(names)} joints)")
    print(f"sweeping       : {', '.join(id_to_name[s] for s in sweep_ids)}")
    if partial:
        print(f"carried over   : {', '.join(n for n in names if n not in {id_to_name[s] for s in sweep_ids})}")

    device = "(none — dry run)"
    pre_state: Dict[str, Any] = {}
    if not args.dry_run:
        try:
            device = _resolve_device(args.device)
            # Ping every servo, not just the swept ones: a joint that has
            # dropped off the bus is worth knowing about before anything moves.
            _check_servos(device, args.baud, servo_ids)
            pre_state = _pre_sweep_state(device, args.baud, servo_ids, id_to_name)
        except RuntimeError as exc:
            print(f"\nerror: {exc}", file=sys.stderr)
            return 2
        _print_pre_state(pre_state)

    if not args.yes and not args.dry_run:
        print()
        print("The sweep drives each selected joint into both of its mechanical")
        print("stops, one joint at a time, in wheel mode. Clear the workspace,")
        print("support the arm, and be ready to cut power.")
        if args.recentre:
            print()
            print("--recentre will also REWRITE servo EEPROM (homing offset and")
            print("angle limits) for the selected joints. The previous offsets are")
            print("recorded in the calibration file's notes.")
        if not input("Proceed? [y/N] ").strip().lower().startswith("y"):
            print("aborted.")
            return 1

    recentred: Dict[str, Any] = {}
    if args.recentre and not args.dry_run:
        print()
        with _bus(device, args.baud) as srv:
            for sid in sweep_ids:
                recentred[id_to_name[sid]] = recentre_joint(
                    srv, sid,
                    speed=args.speed,
                    stall_thr=args.stall_threshold,
                    stall_win=args.stall_window,
                    timeout_s=args.timeout,
                )

    print()
    if args.dry_run:
        results = simulate_rom_sweep(
            joint_ids=sweep_ids,
            sweep_speed=args.speed,
            timeout_s=args.timeout,
        )
        source = "SIMULATED sweep — not measured on hardware"
    else:
        results = {}
        with _bus(device, args.baud) as srv:
            for sid in sweep_ids:
                results[sid] = measure_travel_range(
                    srv, sid,
                    speed=args.speed,
                    stall_thr=args.stall_threshold,
                    stall_win=args.stall_window,
                    timeout_s=args.timeout,
                )
        source = f"measured travel on {device} + URDF limits"

    swept_names = [id_to_name[sid] for sid in sweep_ids]
    fresh = seed_from_travel(
        names=swept_names,
        urdf_limits=[limit_of[n] for n in swept_names],
        tick_ranges=[(results[sid]["pos_min"], results[sid]["pos_max"])
                     for sid in sweep_ids],
        arm_id=args.arm_id or ("simulated" if args.dry_run else "unknown"),
        source=source,
    )
    warnings = _sweep_warnings(
        results, id_to_name, {j.name: j.span_ratio for j in fresh.joints}
    )
    if any(warnings.values()):
        print("\nsweep warnings:")
        for name, msgs in warnings.items():
            for msg in msgs:
                print(f"  ! {name}: {msg}")

    sweep_notes = {
        id_to_name[sid]: {k: results[sid][k]
                          for k in ("pos_min", "pos_max", "range_ticks",
                                    "stalled_fwd", "stalled_rev")}
        for sid in sweep_ids
    }

    if partial:
        cal = _merge_into_existing(
            fresh, out, names, sweep_notes, warnings, args
        )
    else:
        cal = fresh
        cal.notes["sweep"] = sweep_notes
        cal.notes["sweep_warnings"] = {k: v for k, v in warnings.items() if v}
        prior = RobotCalibration.load(out) if out.exists() else None
        _append_rom_endpoint_samples(cal, prior, sweep_notes, args.dry_run)

    if pre_state:
        cal.notes["servo_state_before_sweep"] = pre_state
    if recentred:
        # The old offset is the only way back to the frame the arm was in
        # before this run, so it is kept rather than just the new one — and
        # re-centring the same joint twice must not overwrite the value it
        # started with, or the change stops being reversible.
        prior_rc = cal.notes.get("recentred", {})
        merged_rc = dict(prior_rc)
        for name, entry in recentred.items():
            was = prior_rc.get(name, {})
            merged_rc[name] = {
                **entry,
                "original_offset": was.get("original_offset", was.get("old_offset", entry["old_offset"])),
            }
        cal.notes["recentred"] = merged_rc
    if args.dry_run:
        cal.notes["dry_run"] = "travel is simulated; do not drive an arm with this"

    saved = cal.save(out)
    print()
    _print_report(cal, saved)
    return 0


def _append_rom_endpoint_samples(
    calibration: RobotCalibration,
    prior: Optional[RobotCalibration],
    sweep_notes: Dict[str, Any],
    dry_run: bool,
) -> None:
    """Append one complete endpoint observation per joint to calibration evidence."""
    old = prior.notes.get("rom_endpoint_samples", {}) if prior else {}
    samples = {
        name: list(entries)
        for name, entries in old.items()
        if isinstance(entries, list)
    } if isinstance(old, dict) else {}
    for name, measurement in sweep_notes.items():
        samples.setdefault(name, []).append(
            {
                "min": measurement["pos_min"],
                "max": measurement["pos_max"],
                "simulated": dry_run,
            }
        )
    calibration.notes["rom_endpoint_samples"] = samples


def _merge_into_existing(
    fresh: RobotCalibration,
    out: Path,
    names: Sequence[str],
    sweep_notes: Dict[str, Any],
    warnings: Dict[str, List[str]],
    args: argparse.Namespace,
) -> RobotCalibration:
    """Fold a partial sweep into the calibration already on disk.

    Sweeping a subset is how a heavy arm gets calibrated safely — a joint at
    a time, checking the numbers between runs — but the file has to stay
    whole, so the joints that were not swept are carried over verbatim.
    """
    prior = RobotCalibration.load(out) if out.exists() else None
    by_name = {j.name: j for j in (prior.joints if prior else [])}
    by_name.update({j.name: j for j in fresh.joints})
    # A joint nobody has swept yet is simply absent. The file then has fewer
    # joints than the arm, which every consumer already treats as unusable
    # (``soarm_tamp.conventions.check_ready`` compares the joint list against
    # the URDF order) — better than inventing a placeholder that reads as real.
    missing = [n for n in names if n not in by_name]

    notes = dict(prior.notes) if prior else {}
    notes["sweep"] = {**notes.get("sweep", {}), **sweep_notes}
    _append_rom_endpoint_samples(
        fresh, prior, sweep_notes, args.dry_run
    )
    notes["rom_endpoint_samples"] = fresh.notes["rom_endpoint_samples"]
    # Earlier versions stored a flat list, which lost warnings on a partial
    # re-sweep — that is why this is a dict now. Drop a list we find rather
    # than trying to attribute its entries back to joints.
    prior_warnings = notes.get("sweep_warnings")
    if not isinstance(prior_warnings, dict):
        prior_warnings = {}
    merged_warnings = {**prior_warnings, **warnings}
    notes["sweep_warnings"] = {k: v for k, v in merged_warnings.items() if v}
    if not notes["sweep_warnings"]:
        del notes["sweep_warnings"]
    # Any re-measured joint moves the zero, so an earlier physical check no
    # longer covers this file. Validation has to be re-earned.
    notes.pop("validated_by", None)
    notes.pop("validated_at", None)
    notes["direction_signs"] = fresh.notes["direction_signs"]
    notes["next_step"] = fresh.notes["next_step"]
    notes["partial_sweeps"] = notes.get("partial_sweeps", []) + [
        {"joints": sorted(sweep_notes), "source": fresh.source}
    ]
    if missing:
        notes["incomplete"] = f"never swept: {', '.join(missing)}"
    else:
        notes.pop("incomplete", None)
    return RobotCalibration(
        joints=[by_name[n] for n in names if n in by_name],
        arm_id=args.arm_id or (prior.arm_id if prior else "unknown"),
        validated=False,
        source=f"{fresh.source} (partial; other joints from an earlier sweep)",
        created=fresh.created,
        notes=notes,
    )


if __name__ == "__main__":
    sys.exit(main())
