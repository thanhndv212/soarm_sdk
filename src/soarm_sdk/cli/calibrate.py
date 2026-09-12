"""Interactive + scriptable servo calibration CLI for soarm_sdk.

Features
--------
* Scan a range of actuator IDs on a serial bus.
* Assign new IDs to discovered servos.
* Update angle limits, acceleration, speed, torque, mode, and baud-rate
  settings across multiple devices in one run.
* Optionally unlock EEPROM before changes and re-lock afterwards.

Example
-------
```bash
soarm-calibrate \
    --device /dev/ttyUSB0 --scan-range 1-6 \
    --assign-id 1:11 --assign-id 2:12 \
    --angle-limit 11:100:4000 --angle-limit 12:200:3800 \
    --set-acc 11:60 --set-speed 11:300 \
    --torque 11:on --lock
```

All calibration logic lives in :mod:`soarm_sdk.bus` (servo EEPROM
configuration) and :mod:`soarm_sdk.bus.discovery` (port/servo discovery).
This module adds the interactive text-based UI and argparse wiring on top.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

from .. import print_ports, run_calibration
from ..bus import parse_range

# ---------------------------------------------------------------------------
# Interactive TUI data structures
# ---------------------------------------------------------------------------

@dataclass
class UIState:
    device: str
    baud: int
    scan_range: str
    assign_id: Dict[int, int]
    angle_limits: Dict[int, Tuple[int, int]]
    acceleration: Dict[int, int]
    speed: Dict[int, int]
    torque: Dict[int, bool]
    mode: Dict[int, int]
    baud_map: Dict[int, int]
    unlock: bool
    lock: bool


# ---------------------------------------------------------------------------
# TUI helpers — prompt utilities
# ---------------------------------------------------------------------------

def _safe_input(prompt: str) -> Optional[str]:
    try:
        return input(prompt)
    except EOFError:
        print()
        return None


def _prompt_int(prompt: str, default: Optional[int] = None) -> Optional[int]:
    while True:
        suffix = f" [{default}]" if default is not None else ""
        raw = _safe_input(f"{prompt}{suffix}: ")
        if raw is None:
            return default
        raw = raw.strip()
        if not raw:
            return default
        try:
            return int(raw, 0)
        except ValueError:
            print("  Please enter a valid integer (decimal or 0x-prefixed).")


def _prompt_bool(prompt: str, default: bool) -> bool:
    options = " [Y/n]" if default else " [y/N]"
    while True:
        raw = _safe_input(f"{prompt}{options}: ")
        if raw is None:
            return default
        raw = raw.strip().lower()
        if not raw:
            return default
        if raw in {"y", "yes"}:
            return True
        if raw in {"n", "no"}:
            return False
        print("  Enter 'y' or 'n'.")


def _prompt_range_string(current: str) -> str:
    while True:
        raw = _safe_input(f"Enter scan range [current {current}]: ")
        if raw is None:
            return current
        raw = raw.strip()
        if not raw:
            return current
        try:
            parse_range(raw)
            return raw
        except ValueError as exc:
            print(f"  {exc}")


def _format_mapping(mapping: Dict, formatter) -> str:
    if not mapping:
        return "  (none)"
    return "\n".join(f"  {formatter(k, mapping[k])}" for k in sorted(mapping))


# ---------------------------------------------------------------------------
# TUI — per-field editors
# ---------------------------------------------------------------------------

def _edit_assign_id(state: UIState) -> None:
    while True:
        print("\nCurrent ID remaps:")
        print(_format_mapping(state.assign_id, lambda s, d: f"ID {s} -> {d}"))
        choice = (_safe_input("[A]dd [R]emove [C]lear [B]ack: ") or "").strip().lower()
        if choice == "a":
            current = _prompt_int("  Existing ID")
            new = _prompt_int("  New ID")
            if current is not None and new is not None:
                state.assign_id[current] = new
        elif choice == "r":
            target = _prompt_int("  ID to remove")
            if target is not None and state.assign_id.pop(target, None) is None:
                print("  ID not present.")
        elif choice == "c":
            if _prompt_bool("  Clear all ID remaps?", False):
                state.assign_id.clear()
        elif choice in {"b", ""}:
            break
        else:
            print("  Invalid choice.")


def _edit_angle_limits(state: UIState) -> None:
    while True:
        print("\nCurrent angle limits:")
        print(_format_mapping(
            state.angle_limits, lambda s, v: f"ID {s}: min={v[0]} max={v[1]}"
        ))
        choice = (_safe_input("[A]dd [R]emove [C]lear [B]ack: ") or "").strip().lower()
        if choice == "a":
            sid = _prompt_int("  Servo ID")
            lo = _prompt_int("  Min angle")
            hi = _prompt_int("  Max angle")
            if sid is not None and lo is not None and hi is not None:
                if lo > hi:
                    print("  Min angle must be <= max angle.")
                else:
                    state.angle_limits[sid] = (lo, hi)
        elif choice == "r":
            target = _prompt_int("  Servo ID to remove")
            if target is not None and state.angle_limits.pop(target, None) is None:
                print("  ID not present.")
        elif choice == "c":
            if _prompt_bool("  Clear all angle limits?", False):
                state.angle_limits.clear()
        elif choice in {"b", ""}:
            break
        else:
            print("  Invalid choice.")


def _edit_scalar_mapping(mapping: Dict[int, int], label: str, value_label: str) -> None:
    while True:
        print(f"\nCurrent {label}:")
        print(_format_mapping(mapping, lambda s, v: f"ID {s}: {value_label}={v}"))
        choice = (_safe_input("[A]dd [R]emove [C]lear [B]ack: ") or "").strip().lower()
        if choice == "a":
            sid = _prompt_int("  Servo ID")
            val = _prompt_int(f"  {value_label.capitalize()}")
            if sid is not None and val is not None:
                mapping[sid] = val
        elif choice == "r":
            target = _prompt_int("  Servo ID to remove")
            if target is not None and mapping.pop(target, None) is None:
                print("  ID not present.")
        elif choice == "c":
            if _prompt_bool(f"  Clear all {label}?", False):
                mapping.clear()
        elif choice in {"b", ""}:
            break
        else:
            print("  Invalid choice.")


def _edit_torque(state: UIState) -> None:
    while True:
        print("\nCurrent torque settings:")
        print(_format_mapping(
            state.torque, lambda s, v: f"ID {s}: {'on' if v else 'off'}"
        ))
        choice = (_safe_input("[A]dd [R]emove [C]lear [B]ack: ") or "").strip().lower()
        if choice == "a":
            sid = _prompt_int("  Servo ID")
            if sid is not None:
                state.torque[sid] = _prompt_bool("  Enable torque?", True)
        elif choice == "r":
            target = _prompt_int("  Servo ID to remove")
            if target is not None and state.torque.pop(target, None) is None:
                print("  ID not present.")
        elif choice == "c":
            if _prompt_bool("  Clear all torque entries?", False):
                state.torque.clear()
        elif choice in {"b", ""}:
            break
        else:
            print("  Invalid choice.")


# ---------------------------------------------------------------------------
# TUI — state <-> args conversion and summary
# ---------------------------------------------------------------------------

def _state_to_args(state: UIState) -> argparse.Namespace:
    return argparse.Namespace(
        device=state.device,
        baud=state.baud,
        scan_range=state.scan_range,
        assign_id=[f"{s}:{d}" for s, d in sorted(state.assign_id.items())],
        angle_limit=[
            f"{s}:{v[0]}:{v[1]}" for s, v in sorted(state.angle_limits.items())
        ],
        set_acc=[f"{s}:{v}" for s, v in sorted(state.acceleration.items())],
        set_speed=[f"{s}:{v}" for s, v in sorted(state.speed.items())],
        torque=[
            f"{s}:{'on' if v else 'off'}" for s, v in sorted(state.torque.items())
        ],
        set_mode=[f"{s}:{v}" for s, v in sorted(state.mode.items())],
        set_baud=[f"{s}:{v}" for s, v in sorted(state.baud_map.items())],
        unlock=state.unlock,
        lock=state.lock,
        list_ports=False,
        ui=False,
    )


def _print_state_summary(state: UIState) -> None:
    print("\nCurrent configuration:")
    print(f"  Device: {state.device}")
    print(f"  Baud: {state.baud}")
    print(f"  Scan range: {state.scan_range}")
    print(f"  Unlock EEPROM: {'yes' if state.unlock else 'no'}")
    print(f"  Relock EEPROM: {'yes' if state.lock else 'no'}")
    print("  ID remaps:")
    print(_format_mapping(state.assign_id, lambda s, d: f"ID {s} -> {d}"))
    print("  Angle limits:")
    print(_format_mapping(
        state.angle_limits, lambda s, v: f"ID {s}: min={v[0]} max={v[1]}"
    ))
    print("  Acceleration overrides:")
    print(_format_mapping(state.acceleration, lambda s, v: f"ID {s}: acc={v}"))
    print("  Speed overrides:")
    print(_format_mapping(state.speed, lambda s, v: f"ID {s}: speed={v}"))
    print("  Torque toggles:")
    print(_format_mapping(
        state.torque, lambda s, v: f"ID {s}: {'on' if v else 'off'}"
    ))
    print("  Mode overrides:")
    print(_format_mapping(state.mode, lambda s, v: f"ID {s}: mode={v}"))
    print("  Baud overrides:")
    print(_format_mapping(state.baud_map, lambda s, v: f"ID {s}: code={v}"))


# ---------------------------------------------------------------------------
# Interactive UI entry point
# ---------------------------------------------------------------------------

def launch_ui() -> int:
    parser = create_parser()
    defaults = parser.parse_args([])
    state = UIState(
        device=defaults.device,
        baud=defaults.baud,
        scan_range=defaults.scan_range,
        assign_id={},
        angle_limits={},
        acceleration={},
        speed={},
        torque={},
        mode={},
        baud_map={},
        unlock=defaults.unlock,
        lock=defaults.lock,
    )

    print("Interactive soarm_sdk calibration UI")
    print("-" * 36)

    while True:
        _print_state_summary(state)
        print(
            "\nMenu:\n"
            "  1) Change serial device\n"
            "  2) Change baud rate\n"
            "  3) Change scan range\n"
            "  4) Edit ID remaps\n"
            "  5) Edit angle limits\n"
            "  6) Edit acceleration overrides\n"
            "  7) Edit speed overrides\n"
            "  8) Edit torque toggles\n"
            "  9) Edit mode overrides\n"
            "  B) Edit baud overrides\n"
            "  U) Toggle EEPROM unlock\n"
            "  L) Toggle EEPROM lock\n"
            "  P) List serial ports\n"
            "  R) Run calibration\n"
            "  Q) Quit\n"
        )
        choice = (_safe_input("Select an option: ") or "").strip().lower()
        if not choice:
            continue
        if choice == "1":
            new = (_safe_input(f"Enter serial device [current {state.device}]: ") or "").strip()
            if new:
                state.device = new
        elif choice == "2":
            baud = _prompt_int("Enter baud rate", state.baud)
            if baud is not None:
                state.baud = baud
        elif choice == "3":
            state.scan_range = _prompt_range_string(state.scan_range)
        elif choice == "4":
            _edit_assign_id(state)
        elif choice == "5":
            _edit_angle_limits(state)
        elif choice == "6":
            _edit_scalar_mapping(state.acceleration, "acceleration overrides", "acc")
        elif choice == "7":
            _edit_scalar_mapping(state.speed, "speed overrides", "speed")
        elif choice == "8":
            _edit_torque(state)
        elif choice == "9":
            _edit_scalar_mapping(state.mode, "mode overrides", "mode")
        elif choice == "b":
            _edit_scalar_mapping(state.baud_map, "baud overrides", "baud code")
        elif choice == "u":
            state.unlock = _prompt_bool("Unlock EEPROM before applying changes?", state.unlock)
        elif choice == "l":
            state.lock = _prompt_bool("Lock EEPROM after applying changes?", state.lock)
        elif choice == "p":
            print()
            print_ports()
            _safe_input("Press Enter to continue...")
        elif choice == "r":
            args = _state_to_args(state)
            try:
                run_calibration(args)
            except ValueError as exc:
                print(f"\n  Configuration error: {exc}")
            except RuntimeError as exc:
                print(f"\n  Runtime error: {exc}")
            else:
                print("\n  Calibration run complete.")
            _safe_input("Press Enter to return to the menu...")
        elif choice in {"q", "x"}:
            if _prompt_bool("Exit the UI?", True):
                return 0
        else:
            print("  Invalid choice.")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def create_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Calibrate soarm_sdk devices connected to a serial bus.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--device", default="/dev/ttyUSB0", help="Serial port path")
    parser.add_argument("--baud", type=int, default=1_000_000, help="Bus baudrate")
    parser.add_argument("--scan-range", default="1-10",
                        help="Range of servo IDs to probe, formatted as START-END")
    parser.add_argument("--assign-id", action="append", default=[], metavar="OLD:NEW",
                        help="Reassign an existing servo ID to a new value.")
    parser.add_argument("--angle-limit", action="append", default=[], metavar="ID:MIN:MAX",
                        help="Set new minimum/maximum angle limits for a servo.")
    parser.add_argument("--set-acc", action="append", default=[], metavar="ID:ACC",
                        help="Set the acceleration register (STS_ACC).")
    parser.add_argument("--set-speed", action="append", default=[], metavar="ID:SPEED",
                        help="Set the goal speed register (STS_GOAL_SPEED_L/H).")
    parser.add_argument("--torque", action="append", default=[], metavar="ID:STATE",
                        help="Enable or disable torque (STATE = on/off or 1/0).")
    parser.add_argument("--set-mode", action="append", default=[], metavar="ID:MODE",
                        help="Set the operating mode register (0=servo, 1=wheel).")
    parser.add_argument("--set-baud", action="append", default=[], metavar="ID:BAUD_CODE",
                        help="Set the baud-rate register (soarm_sdk-specific encoding).")
    parser.add_argument("--unlock", action="store_true",
                        help="Unlock servo EEPROM before applying calibration changes.")
    parser.add_argument("--lock", action="store_true",
                        help="Lock servo EEPROM after applying calibration changes.")
    parser.add_argument("--list-ports", action="store_true",
                        help="List detected serial ports before connecting.")
    parser.add_argument("--ui", action="store_true",
                        help="Launch an interactive text-based user interface.")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = create_parser()
    args = parser.parse_args(argv)

    if getattr(args, "ui", False):
        return launch_ui()

    if args.list_ports:
        # A query, not a modifier: print and stop. Falling through ran a
        # full calibration afterwards, which opened --device (default
        # /dev/ttyUSB0) and raised SerialException on any machine that
        # simply wanted to know which ports exist -- and printed the port
        # list twice on the way, since run_calibration prints it again for
        # callers that pass the flag in a Namespace.
        print_ports()
        return 0

    try:
        return run_calibration(args)
    except (ValueError, RuntimeError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    sys.exit(main())
