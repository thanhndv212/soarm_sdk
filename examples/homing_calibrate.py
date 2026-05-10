#!/usr/bin/env python
"""Utility script for homing and calibration of STServo actuators.

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
python homing_calibrate.py \
    --device /dev/ttyUSB0 --scan-range 1-6 \
    --assign-id 1:11 --assign-id 2:12 \
    --angle-limit 11:100:4000 --angle-limit 12:200:3800 \
    --set-acc 11:60 --set-speed 11:300 \
    --torque 11:on --lock
```
"""

from __future__ import annotations

import argparse
import importlib
import sys
from dataclasses import dataclass
from pathlib import Path
from collections import OrderedDict
from typing import (
    Any,
    Callable,
    Dict,
    Iterable,
    List,
    Optional,
    Sequence,
    Tuple,
)

try:
    from serial.tools import list_ports
except ModuleNotFoundError as exc:  # pragma: no cover - import-time guard
    raise ModuleNotFoundError(
        "The 'pyserial' package is required for STServo calibration. "
        "Install it with 'python -m pip install pyserial' using the same "
        "Python interpreter that launches this script. If you're running "
        "the Streamlit dashboard, prefer 'python -m streamlit run "
        "examples/homing_dashboard.py' so it shares the environment."
    ) from exc


def _import_stservo():
    try:
        return importlib.import_module("stservo_sdk")
    except ModuleNotFoundError:  # pragma: no cover - local source checkout
        src_root = Path(__file__).resolve().parents[1] / "src"
        if src_root.is_dir() and str(src_root) not in sys.path:
            sys.path.insert(0, str(src_root))
        return importlib.import_module("stservo_sdk")


_stservo_sdk = _import_stservo()
PortHandler = _stservo_sdk.PortHandler
sts = _stservo_sdk.sts

_stservo_def = importlib.import_module("stservo_sdk.stservo_def")
COMM_SUCCESS = _stservo_def.COMM_SUCCESS
COMM_RX_FAIL = _stservo_def.COMM_RX_FAIL
STS_ACC = _stservo_def.STS_ACC
STS_BAUD_RATE = _stservo_def.STS_BAUD_RATE
STS_GOAL_SPEED_L = _stservo_def.STS_GOAL_SPEED_L
STS_ID = _stservo_def.STS_ID
STS_LOCK = _stservo_def.STS_LOCK
STS_MIN_ANGLE_LIMIT_L = _stservo_def.STS_MIN_ANGLE_LIMIT_L
STS_MAX_ANGLE_LIMIT_L = _stservo_def.STS_MAX_ANGLE_LIMIT_L
STS_MODE = _stservo_def.STS_MODE
STS_TORQUE_ENABLE = _stservo_def.STS_TORQUE_ENABLE


def _extract_result_error(response) -> Tuple[int, int]:
    if isinstance(response, tuple):
        return response
    if hasattr(response, "result") and hasattr(response, "error"):
        return response.result, response.error
    raise TypeError(
        "Unexpected response type from packet handler: " f"{type(response)!r}"
    )


def _extract_ping_response(
    response,
) -> Tuple[int, int, List[int]]:
    if hasattr(response, "result") and hasattr(response, "data"):
        data = getattr(response, "data", []) or []
        return response.result, getattr(response, "error", 0), list(data)
    if isinstance(response, tuple):
        if not response:
            raise TypeError("Ping response tuple is empty.")
        result = response[0]
        error = response[1] if len(response) > 1 else 0
        payload = response[2] if len(response) > 2 else []
        if isinstance(payload, (bytes, bytearray)):
            data = list(payload)
        elif isinstance(payload, (list, tuple)):
            data = list(payload)
        elif payload is None:
            data = []
        else:
            data = [payload]
        return result, error, data
    raise TypeError(
        "Unexpected ping response type from packet handler: "
        f"{type(response)!r}"
    )


@dataclass
class OperationPlan:
    """Container for parsed CLI operations."""

    assign_id: Dict[int, int]
    angle_limits: Dict[int, Tuple[int, int]]
    acceleration: Dict[int, int]
    speed: Dict[int, int]
    torque: Dict[int, bool]
    mode: Dict[int, int]
    baud: Dict[int, int]


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


def get_available_ports() -> List[Tuple[str, str]]:
    """Return serial port device/description pairs."""

    ports = [
        (port.device, port.description or "")
        for port in list_ports.comports()
        if port.device.startswith("/dev/tty.usb")
    ]

    if not ports:
        ports = [
            (str(path), "")
            for path in Path("/dev").glob("tty.usb*")
            if path.is_char_device()
        ]

    return ports


def _format_mapping(mapping: Dict[int, object], formatter) -> str:
    if not mapping:
        return "  (none)"
    lines = [
        f"  {formatter(key, mapping[key])}" for key in sorted(mapping.keys())
    ]
    return "\n".join(lines)


def _safe_input(prompt: str) -> Optional[str]:
    try:
        return input(prompt)
    except EOFError:
        print()
        return None


def _prompt_int(prompt: str, default: Optional[int] = None) -> Optional[int]:
    while True:
        suffix = f" [{default}]" if default is not None else ""
        raw_value = _safe_input(f"{prompt}{suffix}: ")
        if raw_value is None:
            return default
        raw = raw_value.strip()
        if not raw:
            return default
        try:
            return int(raw, 0)
        except ValueError:
            print("  Please enter a valid integer (decimal or 0x-prefixed).")


def _prompt_bool(prompt: str, default: bool) -> bool:
    options = " [Y/n]" if default else " [y/N]"
    while True:
        raw_value = _safe_input(f"{prompt}{options}: ")
        if raw_value is None:
            return default
        raw = raw_value.strip().lower()
        if not raw:
            return default
        if raw in {"y", "yes"}:
            return True
        if raw in {"n", "no"}:
            return False
        print("  Enter 'y' or 'n'.")


def _prompt_range_string(current: str) -> str:
    while True:
        raw_value = _safe_input(f"Enter scan range [current {current}]: ")
        if raw_value is None:
            return current
        raw = raw_value.strip()
        if not raw:
            return current
        try:
            parse_range(raw)
            return raw
        except ValueError as exc:
            print(f"  {exc}")


def _edit_assign_id(state: UIState) -> None:
    while True:
        print("\nCurrent ID remaps:")
        print(
            _format_mapping(
                state.assign_id, lambda src, dst: f"ID {src} -> {dst}"
            )
        )
        choice_raw = _safe_input("[A]dd [R]emove [C]lear [B]ack: ")
        if choice_raw is None:
            break
        choice = choice_raw.strip().lower()
        if choice == "a":
            current = _prompt_int("  Existing ID")
            if current is None:
                continue
            new = _prompt_int("  New ID")
            if new is None:
                continue
            state.assign_id[current] = new
        elif choice == "r":
            target = _prompt_int("  ID to remove")
            if target is None:
                continue
            if state.assign_id.pop(target, None) is None:
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
        print(
            _format_mapping(
                state.angle_limits,
                lambda sid, vals: f"ID {sid}: min={vals[0]} max={vals[1]}",
            )
        )
        choice_raw = _safe_input("[A]dd [R]emove [C]lear [B]ack: ")
        if choice_raw is None:
            break
        choice = choice_raw.strip().lower()
        if choice == "a":
            servo_id = _prompt_int("  Servo ID")
            if servo_id is None:
                continue
            min_angle = _prompt_int("  Min angle")
            max_angle = _prompt_int("  Max angle")
            if min_angle is None or max_angle is None:
                continue
            if min_angle > max_angle:
                print("  Min angle must be <= max angle.")
                continue
            state.angle_limits[servo_id] = (min_angle, max_angle)
        elif choice == "r":
            target = _prompt_int("  Servo ID to remove")
            if target is None:
                continue
            if state.angle_limits.pop(target, None) is None:
                print("  ID not present.")
        elif choice == "c":
            if _prompt_bool("  Clear all angle limits?", False):
                state.angle_limits.clear()
        elif choice in {"b", ""}:
            break
        else:
            print("  Invalid choice.")


def _edit_scalar_mapping(
    mapping: Dict[int, int], label: str, value_label: str
) -> None:
    while True:
        print(f"\nCurrent {label}:")
        print(
            _format_mapping(
                mapping, lambda sid, val: f"ID {sid}: {value_label}={val}"
            )
        )
        choice_raw = _safe_input("[A]dd [R]emove [C]lear [B]ack: ")
        if choice_raw is None:
            break
        choice = choice_raw.strip().lower()
        if choice == "a":
            servo_id = _prompt_int("  Servo ID")
            if servo_id is None:
                continue
            value = _prompt_int(f"  {value_label.capitalize()}")
            if value is None:
                continue
            mapping[servo_id] = value
        elif choice == "r":
            target = _prompt_int("  Servo ID to remove")
            if target is None:
                continue
            if mapping.pop(target, None) is None:
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
        print(
            _format_mapping(
                state.torque,
                lambda sid, val: f"ID {sid}: {'on' if val else 'off'}",
            )
        )
        choice_raw = _safe_input("[A]dd [R]emove [C]lear [B]ack: ")
        if choice_raw is None:
            break
        choice = choice_raw.strip().lower()
        if choice == "a":
            servo_id = _prompt_int("  Servo ID")
            if servo_id is None:
                continue
            state.torque[servo_id] = _prompt_bool("  Enable torque?", True)
        elif choice == "r":
            target = _prompt_int("  Servo ID to remove")
            if target is None:
                continue
            if state.torque.pop(target, None) is None:
                print("  ID not present.")
        elif choice == "c":
            if _prompt_bool("  Clear all torque entries?", False):
                state.torque.clear()
        elif choice in {"b", ""}:
            break
        else:
            print("  Invalid choice.")


def _state_to_args(state: UIState) -> argparse.Namespace:
    return argparse.Namespace(
        device=state.device,
        baud=state.baud,
        scan_range=state.scan_range,
        assign_id=[
            f"{src}:{dst}" for src, dst in sorted(state.assign_id.items())
        ],
        angle_limit=[
            f"{sid}:{vals[0]}:{vals[1]}"
            for sid, vals in sorted(state.angle_limits.items())
        ],
        set_acc=[
            f"{sid}:{value}"
            for sid, value in sorted(state.acceleration.items())
        ],
        set_speed=[
            f"{sid}:{value}" for sid, value in sorted(state.speed.items())
        ],
        torque=[
            f"{sid}:{'on' if value else 'off'}"
            for sid, value in sorted(state.torque.items())
        ],
        set_mode=[
            f"{sid}:{value}" for sid, value in sorted(state.mode.items())
        ],
        set_baud=[
            f"{sid}:{value}" for sid, value in sorted(state.baud_map.items())
        ],
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
    print(
        _format_mapping(state.assign_id, lambda src, dst: f"ID {src} -> {dst}")
    )
    print("  Angle limits:")
    print(
        _format_mapping(
            state.angle_limits,
            lambda sid, vals: f"ID {sid}: min={vals[0]} max={vals[1]}",
        )
    )
    print("  Acceleration overrides:")
    print(
        _format_mapping(
            state.acceleration,
            lambda sid, val: f"ID {sid}: acc={val}",
        )
    )
    print("  Speed overrides:")
    print(
        _format_mapping(
            state.speed,
            lambda sid, val: f"ID {sid}: speed={val}",
        )
    )
    print("  Torque toggles:")
    print(
        _format_mapping(
            state.torque,
            lambda sid, val: f"ID {sid}: {'on' if val else 'off'}",
        )
    )
    print("  Mode overrides:")
    print(
        _format_mapping(
            state.mode,
            lambda sid, val: f"ID {sid}: mode={val}",
        )
    )
    print("  Baud overrides:")
    print(
        _format_mapping(
            state.baud_map,
            lambda sid, val: f"ID {sid}: code={val}",
        )
    )


def parse_range(raw: str) -> range:
    try:
        start_str, end_str = raw.split("-", 1)
        start = int(start_str)
        end = int(end_str)
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
    mapping: Dict[int, Tuple[int, ...]] = {}
    for raw in raw_entries:
        parts = raw.split(":")
        if len(parts) != expected_parts:
            raise ValueError(
                f"Invalid {label} '{raw}'. Expected "
                f"{expected_parts} colon-separated values."
            )
        try:
            ints = tuple(int(part, 0) for part in parts)
        except ValueError as exc:
            raise ValueError(f"Invalid integer in {label} '{raw}'.") from exc
        mapping[ints[0]] = ints[1:]
    return mapping


def parse_bool_mapping(
    raw_entries: Sequence[str], label: str
) -> Dict[int, bool]:
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
    assign_id: Dict[int, int] = {}
    for key, value in parse_mapping(args.assign_id, 2, "--assign-id").items():
        assign_id[key] = value[0]
    angle_limits = {
        key: (value[0], value[1])
        for key, value in parse_mapping(
            args.angle_limit, 3, "--angle-limit"
        ).items()
    }
    acceleration = {
        key: value[0]
        for key, value in parse_mapping(args.set_acc, 2, "--set-acc").items()
    }
    speed = {
        key: value[0]
        for key, value in parse_mapping(
            args.set_speed, 2, "--set-speed"
        ).items()
    }
    mode = {
        key: value[0]
        for key, value in parse_mapping(args.set_mode, 2, "--set-mode").items()
    }
    baud = {
        key: value[0]
        for key, value in parse_mapping(args.set_baud, 2, "--set-baud").items()
    }
    torque = parse_bool_mapping(args.torque, "--torque")
    return OperationPlan(
        assign_id,
        angle_limits,
        acceleration,
        speed,
        torque,
        mode,
        baud,
    )


def resolve_id(remap: Dict[int, int], servo_id: int) -> int:
    seen: set[int] = set()
    current = servo_id
    while current in remap and current not in seen:
        seen.add(current)
        current = remap[current]
    return current


def collect_final_ids(plan: OperationPlan, remap: Dict[int, int]) -> List[int]:
    ids: set[int] = set()
    for source in plan.angle_limits:
        ids.add(resolve_id(remap, source))
    for source in plan.acceleration:
        ids.add(resolve_id(remap, source))
    for source in plan.speed:
        ids.add(resolve_id(remap, source))
    for source in plan.torque:
        ids.add(resolve_id(remap, source))
    for source in plan.mode:
        ids.add(resolve_id(remap, source))
    for source in plan.baud:
        ids.add(resolve_id(remap, source))
    return sorted(ids)


def print_available_ports(
    log: Optional[Callable[[str], None]] = None,
) -> None:
    logger = log or print
    ports = get_available_ports()
    if not ports:
        logger("No serial ports detected.")
        return
    logger("Detected serial ports:")
    for device, desc in ports:
        logger(f"  {device}\t{desc}")


def scan_servos(
    packet_handler: "sts", id_range: Iterable[int]
) -> Dict[int, int]:
    found: Dict[int, int] = {}
    for servo_id in id_range:
        try:
            result, _error, data = _extract_ping_response(
                packet_handler.ping(servo_id)
            )
        except TypeError:
            continue
        if result == COMM_SUCCESS and data:
            model = data[0]
            found[servo_id] = model
    return found


def discover_servos(
    device: str, baud: int, id_range: Iterable[int]
) -> Dict[int, int]:
    port_handler = PortHandler(device)
    packet_handler = sts(port_handler)
    try:
        if not port_handler.openPort():
            raise RuntimeError(f"Failed to open serial port {device}.")
        if not port_handler.setBaudRate(baud):
            raise RuntimeError(f"Failed to set baudrate to {baud}.")
        return scan_servos(packet_handler, id_range)
    finally:
        port_handler.closePort()


def read_servo_diagnostics(
    device: str, baud: int, servo_ids: Iterable[int]
) -> Dict[int, OrderedDict[str, Optional[Any]]]:
    unique_ids = sorted({int(servo_id) for servo_id in servo_ids})
    diagnostics: Dict[int, OrderedDict[str, Optional[Any]]] = {}
    if not unique_ids:
        return diagnostics

    port_handler = PortHandler(device)
    packet_handler = sts(port_handler)

    def record(
        label: str,
        reader: Callable[[int], Tuple[Any, int, int]],
        target: int,
    ) -> Tuple[Optional[Any], int, int]:
        try:
            value, result, error = reader(target)
        except Exception:  # pragma: no cover - defensive guard
            return None, COMM_RX_FAIL, 0
        if result == COMM_SUCCESS and error == 0:
            return value, result, error
        return None, result, error

    try:
        if not port_handler.openPort():
            raise RuntimeError(f"Failed to open serial port {device}.")
        if not port_handler.setBaudRate(baud):
            raise RuntimeError(f"Failed to set baudrate to {baud}.")

        for servo_id in unique_ids:
            details: OrderedDict[str, Optional[Any]] = OrderedDict()
            errors: Dict[str, Tuple[int, int]] = {}

            details["Baudrate"] = packet_handler.GetBaudrate()

            for label, reader in (
                ("Load", packet_handler.ReadLoad),
                ("Voltage", packet_handler.ReadVoltage),
                ("Current", packet_handler.ReadCurrent),
                ("Temperature", packet_handler.ReadTemperature),
                ("Acceleration", packet_handler.ReadAccelaration),
                ("Mode", packet_handler.ReadMode),
                ("Correction", packet_handler.ReadCorrection),
                ("Is Moving", packet_handler.IsMoving),
                ("Position", packet_handler.ReadPosition),
                ("Speed", packet_handler.ReadSpeed),
                ("Status", packet_handler.ReadStatus),
            ):
                value, result, error = record(label, reader, servo_id)
                details[label] = value
                if result != COMM_SUCCESS or error != 0:
                    errors[label] = (result, error)

            if errors:
                details["_errors"] = errors

            diagnostics[servo_id] = details
    finally:
        port_handler.closePort()

    return diagnostics


def write2(
    packet_handler: "sts",
    servo_id: int,
    address: int,
    value: int,
    label: str,
) -> None:
    result, error = _extract_result_error(
        packet_handler.write2ByteTxRx(servo_id, address, value)
    )
    if result != COMM_SUCCESS or error:
        raise RuntimeError(
            f"Failed to set {label} for ID {servo_id}: "
            f"result={result}, error={error}"
        )


def write1(
    packet_handler: "sts",
    servo_id: int,
    address: int,
    value: int,
    label: str,
) -> None:
    result, error = _extract_result_error(
        packet_handler.write1ByteTxRx(servo_id, address, value)
    )
    if result != COMM_SUCCESS or error:
        raise RuntimeError(
            f"Failed to set {label} for ID {servo_id}: "
            f"result={result}, error={error}"
        )


def create_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Calibrate STServo devices connected to a serial bus.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--device",
        default="/dev/ttyUSB0",
        help="Serial port path",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=1_000_000,
        help="Bus baudrate",
    )
    parser.add_argument(
        "--scan-range",
        default="1-10",
        help="Range of servo IDs to probe, formatted as START-END",
    )
    parser.add_argument(
        "--assign-id",
        action="append",
        default=[],
        metavar="OLD:NEW",
        help="Reassign an existing servo ID to a new value.",
    )
    parser.add_argument(
        "--angle-limit",
        action="append",
        default=[],
        metavar="ID:MIN:MAX",
        help="Set new minimum/maximum angle limits for a servo.",
    )
    parser.add_argument(
        "--set-acc",
        action="append",
        default=[],
        metavar="ID:ACC",
        help="Set the acceleration register (STS_ACC).",
    )
    parser.add_argument(
        "--set-speed",
        action="append",
        default=[],
        metavar="ID:SPEED",
        help="Set the goal speed register (STS_GOAL_SPEED_L/H).",
    )
    parser.add_argument(
        "--torque",
        action="append",
        default=[],
        metavar="ID:STATE",
        help="Enable or disable torque (STATE = on/off or 1/0).",
    )
    parser.add_argument(
        "--set-mode",
        action="append",
        default=[],
        metavar="ID:MODE",
        help="Set the operating mode register (0=servo, 1=wheel).",
    )
    parser.add_argument(
        "--set-baud",
        action="append",
        default=[],
        metavar="ID:BAUD_CODE",
        help="Set the baud-rate register (use STServo-specific encoding).",
    )
    parser.add_argument(
        "--unlock",
        action="store_true",
        help="Unlock servo EEPROM before applying calibration changes.",
    )
    parser.add_argument(
        "--lock",
        action="store_true",
        help="Lock servo EEPROM after applying calibration changes.",
    )
    parser.add_argument(
        "--list-ports",
        action="store_true",
        help="List detected serial ports before connecting.",
    )
    parser.add_argument(
        "--ui",
        action="store_true",
        help="Launch an interactive text-based user interface.",
    )
    return parser


def run_calibration(
    args: argparse.Namespace, log: Optional[Callable[[str], None]] = None
) -> int:
    logger = log or print
    id_range = parse_range(args.scan_range)
    plan = build_operation_plan(args)

    if args.list_ports:
        print_available_ports(log=logger)

    port_handler = PortHandler(args.device)
    packet_handler = sts(port_handler)

    try:
        if not port_handler.openPort():
            raise RuntimeError(f"Failed to open serial port {args.device}.")
        if not port_handler.setBaudRate(args.baud):
            raise RuntimeError(f"Failed to set baudrate to {args.baud}.")

        logger(f"Scanning IDs {id_range.start}-{id_range.stop - 1}...")
        detected = scan_servos(packet_handler, id_range)
        if detected:
            for servo_id, model in sorted(detected.items()):
                logger(f"  Found ID {servo_id:3d} (model {model})")
        else:
            logger("No servos discovered in the specified range.")

        remap: Dict[int, int] = {}

        for current_id, new_id in plan.assign_id.items():
            active_id = resolve_id(remap, current_id)
            logger(f"Reassigning ID {active_id} -> {new_id}...")
            if args.unlock:
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
        if args.unlock and target_ids:
            logger("Unlocking EEPROM for target servos...")
            for servo_id in target_ids:
                actual_id = resolve_id(remap, servo_id)
                try:
                    write1(packet_handler, actual_id, STS_LOCK, 0, "unlock")
                except RuntimeError as exc:
                    logger(f"  Warning: {exc}")

        for servo_id, (min_angle, max_angle) in plan.angle_limits.items():
            actual_id = resolve_id(remap, servo_id)
            logger(
                f"Setting angle limits for ID {actual_id}: "
                f"min={min_angle}, max={max_angle}"
            )
            if min_angle < 0 or max_angle < 0 or min_angle > max_angle:
                logger("  Skipping: invalid angle limits.")
                continue
            try:
                write2(
                    packet_handler,
                    actual_id,
                    STS_MIN_ANGLE_LIMIT_L,
                    min_angle,
                    "min angle limit",
                )
                write2(
                    packet_handler,
                    actual_id,
                    STS_MAX_ANGLE_LIMIT_L,
                    max_angle,
                    "max angle limit",
                )
            except RuntimeError as exc:
                logger(f"  Error: {exc}")

        for servo_id, acc in plan.acceleration.items():
            actual_id = resolve_id(remap, servo_id)
            logger(f"Setting acceleration for ID {actual_id}: {acc}")
            try:
                write1(packet_handler, actual_id, STS_ACC, acc, "acceleration")
            except RuntimeError as exc:
                logger(f"  Error: {exc}")

        for servo_id, speed in plan.speed.items():
            actual_id = resolve_id(remap, servo_id)
            logger(f"Setting speed for ID {actual_id}: {speed}")
            try:
                write2(
                    packet_handler,
                    actual_id,
                    STS_GOAL_SPEED_L,
                    speed,
                    "speed",
                )
            except RuntimeError as exc:
                logger(f"  Error: {exc}")

        for servo_id, state in plan.torque.items():
            actual_id = resolve_id(remap, servo_id)
            printable = "on" if state else "off"
            logger(f"Setting torque for ID {actual_id}: {printable}")
            try:
                write1(
                    packet_handler,
                    actual_id,
                    STS_TORQUE_ENABLE,
                    int(state),
                    "torque",
                )
            except RuntimeError as exc:
                logger(f"  Error: {exc}")

        for servo_id, mode in plan.mode.items():
            actual_id = resolve_id(remap, servo_id)
            logger(f"Setting mode for ID {actual_id}: {mode}")
            try:
                write1(packet_handler, actual_id, STS_MODE, mode, "mode")
            except RuntimeError as exc:
                logger(f"  Error: {exc}")

        for servo_id, baud_code in plan.baud.items():
            actual_id = resolve_id(remap, servo_id)
            logger(f"Setting baud code for ID {actual_id}: {baud_code}")
            try:
                write1(
                    packet_handler,
                    actual_id,
                    STS_BAUD_RATE,
                    baud_code,
                    "baud rate",
                )
            except RuntimeError as exc:
                logger(f"  Error: {exc}")

        if args.lock and (plan.assign_id or target_ids):
            logger("Locking EEPROM after calibration...")
            lock_targets = set(remap.values()) if remap else set()
            lock_targets.update(target_ids)
            for servo_id in sorted(lock_targets):
                actual_id = resolve_id(remap, servo_id)
                try:
                    write1(packet_handler, actual_id, STS_LOCK, 1, "lock")
                except RuntimeError as exc:
                    logger(f"  Warning: {exc}")

    finally:
        port_handler.closePort()

    return 0


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

    print("Interactive STServo calibration UI")
    print("---------------------------------")

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

        choice_raw = _safe_input("Select an option: ")
        if choice_raw is None:
            print("Exiting UI.")
            return 0
        choice = choice_raw.strip().lower()
        if not choice:
            continue
        if choice == "1":
            new_device_raw = _safe_input(
                f"Enter serial device [current {state.device}]: "
            )
            if new_device_raw is None:
                continue
            new_device = new_device_raw.strip()
            if new_device:
                state.device = new_device
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
            _edit_scalar_mapping(
                state.acceleration,
                "acceleration overrides",
                "acc",
            )
        elif choice == "7":
            _edit_scalar_mapping(
                state.speed,
                "speed overrides",
                "speed",
            )
        elif choice == "8":
            _edit_torque(state)
        elif choice == "9":
            _edit_scalar_mapping(state.mode, "mode overrides", "mode")
        elif choice == "b":
            _edit_scalar_mapping(
                state.baud_map,
                "baud overrides",
                "baud code",
            )
        elif choice == "u":
            state.unlock = _prompt_bool(
                "Unlock EEPROM before applying changes?",
                state.unlock,
            )
        elif choice == "l":
            state.lock = _prompt_bool(
                "Lock EEPROM after applying changes?",
                state.lock,
            )
        elif choice == "p":
            print()
            print_available_ports()
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


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = create_parser()
    args = parser.parse_args(argv)

    if getattr(args, "ui", False):
        return launch_ui()

    try:
        return run_calibration(args)
    except (ValueError, RuntimeError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    sys.exit(main())
