"""Serial-bus helpers for soarm_sdk.

Provides port discovery, servo scanning, diagnostics, and low-level
register read/write primitives. These functions are the hardware-access
layer shared by all higher-level tools (calibration CLI, dashboards, …).

Typical usage
-------------
>>> from soarm_sdk import discover_servos, read_diagnostics, write1, write2
>>> found = discover_servos("/dev/tty.usbserial-XXXX", 1_000_000, range(1, 7))
>>> diag = read_diagnostics("/dev/tty.usbserial-XXXX", 1_000_000, found.keys())
"""

from __future__ import annotations

from collections import OrderedDict
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Optional, Tuple

from serial.tools import list_ports as _list_ports_mod

from ..protocol.port_handler import PortHandler
from ..protocol.sts import sts
from ..protocol.registers import COMM_SUCCESS, COMM_RX_FAIL


# ---------------------------------------------------------------------------
# Internal response helpers
# ---------------------------------------------------------------------------

def _extract_result_error(response) -> Tuple[int, int]:
    if isinstance(response, tuple):
        return response
    if hasattr(response, "result") and hasattr(response, "error"):
        return response.result, response.error
    raise TypeError(
        f"Unexpected response type from packet handler: {type(response)!r}"
    )


def _extract_ping_response(response) -> Tuple[int, int, List[int]]:
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
        f"Unexpected ping response type from packet handler: {type(response)!r}"
    )


# ---------------------------------------------------------------------------
# Port discovery
# ---------------------------------------------------------------------------

def list_ports() -> List[Tuple[str, str]]:
    """Return ``(device, description)`` pairs for connected USB serial ports.

    Only ports whose device path starts with ``/dev/tty.usb`` are returned
    (macOS naming convention). Falls back to a ``glob`` scan if ``pyserial``
    reports nothing.

    Returns
    -------
    list[tuple[str, str]]
        Each element is ``(device_path, human_readable_description)``.
    """
    ports = [
        (p.device, p.description or "")
        for p in _list_ports_mod.comports()
        if p.device.startswith("/dev/tty.usb")
    ]
    if not ports:
        ports = [
            (str(path), "")
            for path in Path("/dev").glob("tty.usb*")
            if path.is_char_device()
        ]
    return ports


#: Backward-compatible alias.
get_available_ports = list_ports


def print_ports(log: Optional[Callable[[str], None]] = None) -> None:
    """Print available serial ports using *log* (defaults to :func:`print`)."""
    logger = log or print
    ports = list_ports()
    if not ports:
        logger("No serial ports detected.")
        return
    logger("Detected serial ports:")
    for device, desc in ports:
        logger(f"  {device}\t{desc}")


# ---------------------------------------------------------------------------
# Servo scanning
# ---------------------------------------------------------------------------

def scan_servos(
    packet_handler: sts,
    id_range: Iterable[int],
) -> Dict[int, int]:
    """Ping each ID in *id_range* on an **already-open** connection.

    Parameters
    ----------
    packet_handler:
        An :class:`sts` instance bound to an open :class:`PortHandler`.
    id_range:
        Iterable of integer servo IDs to probe.

    Returns
    -------
    dict[int, int]
        Mapping of ``servo_id → model_number`` for every responding servo.
    """
    found: Dict[int, int] = {}
    for servo_id in id_range:
        try:
            result, _error, data = _extract_ping_response(
                packet_handler.ping(servo_id)
            )
        except TypeError:
            continue
        if result == COMM_SUCCESS and data:
            found[servo_id] = data[0]
    return found


def discover_servos(
    device: str,
    baud: int,
    id_range: Iterable[int],
) -> Dict[int, int]:
    """Open *device*, scan *id_range*, close, and return discovered servos.

    Parameters
    ----------
    device:
        Serial port path, e.g. ``"/dev/tty.usbserial-XXXX"``.
    baud:
        Bus baud rate, typically ``1_000_000``.
    id_range:
        Iterable of integer servo IDs to probe.

    Returns
    -------
    dict[int, int]
        ``servo_id → model_number`` for every responding servo.

    Raises
    ------
    RuntimeError
        If the port cannot be opened or the baud rate cannot be set.
    """
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


# ---------------------------------------------------------------------------
# Diagnostics
# ---------------------------------------------------------------------------

def read_diagnostics(
    device: str,
    baud: int,
    servo_ids: Iterable[int],
) -> Dict[int, OrderedDict[str, Optional[Any]]]:
    """Read a full diagnostics snapshot for each servo in *servo_ids*.

    Opens the serial port, reads all available registers (position, speed,
    load, voltage, current, temperature, mode, correction, moving flag,
    status), then closes the port.

    Parameters
    ----------
    device:
        Serial port path.
    baud:
        Bus baud rate.
    servo_ids:
        Servo IDs to query.

    Returns
    -------
    dict[int, OrderedDict]
        One ``OrderedDict`` per servo ID with register labels as keys.
        A ``"_errors"`` key is added when any register read fails; its value
        is a ``dict`` mapping label → ``(result_code, error_code)``.

    Raises
    ------
    RuntimeError
        If the port cannot be opened or the baud rate cannot be set.
    """
    unique_ids = sorted({int(sid) for sid in servo_ids})
    diagnostics: Dict[int, OrderedDict[str, Optional[Any]]] = {}
    if not unique_ids:
        return diagnostics

    port_handler = PortHandler(device)
    packet_handler = sts(port_handler)

    def _record(
        label: str,
        reader: Callable[[int], Tuple[Any, int, int]],
        target: int,
    ) -> Tuple[Optional[Any], int, int]:
        try:
            value, result, error = reader(target)
        except Exception:
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
                value, result, error = _record(label, reader, servo_id)
                details[label] = value
                if result != COMM_SUCCESS or error != 0:
                    errors[label] = (result, error)

            if errors:
                details["_errors"] = errors

            diagnostics[servo_id] = details
    finally:
        port_handler.closePort()

    return diagnostics


#: Backward-compatible alias.
read_servo_diagnostics = read_diagnostics


# ---------------------------------------------------------------------------
# Low-level register writers
# ---------------------------------------------------------------------------

def write2(
    packet_handler: sts,
    servo_id: int,
    address: int,
    value: int,
    label: str,
) -> None:
    """Write a 2-byte value to a servo register.

    Parameters
    ----------
    packet_handler:
        An open :class:`sts` instance.
    servo_id:
        Target servo ID.
    address:
        Register address (e.g. :data:`~soarm_sdk.stservo_def.STS_MIN_ANGLE_LIMIT_L`).
    value:
        Integer value to write.
    label:
        Human-readable label used in the error message if the write fails.

    Raises
    ------
    RuntimeError
        If the write is not acknowledged successfully.
    """
    result, error = _extract_result_error(
        packet_handler.write2ByteTxRx(servo_id, address, value)
    )
    if result != COMM_SUCCESS or error:
        raise RuntimeError(
            f"Failed to set {label} for ID {servo_id}: "
            f"result={result}, error={error}"
        )


def write1(
    packet_handler: sts,
    servo_id: int,
    address: int,
    value: int,
    label: str,
) -> None:
    """Write a 1-byte value to a servo register.

    Parameters
    ----------
    packet_handler:
        An open :class:`sts` instance.
    servo_id:
        Target servo ID.
    address:
        Register address (e.g. :data:`~soarm_sdk.stservo_def.STS_TORQUE_ENABLE`).
    value:
        Integer value to write.
    label:
        Human-readable label used in the error message if the write fails.

    Raises
    ------
    RuntimeError
        If the write is not acknowledged successfully.
    """
    result, error = _extract_result_error(
        packet_handler.write1ByteTxRx(servo_id, address, value)
    )
    if result != COMM_SUCCESS or error:
        raise RuntimeError(
            f"Failed to set {label} for ID {servo_id}: "
            f"result={result}, error={error}"
        )
