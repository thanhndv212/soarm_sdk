"""Serial-bus access: port discovery, scanning, diagnostics, and servo configuration.

- :mod:`soarm_sdk.bus.discovery` — list ports, scan/ping servos, read
  diagnostics, raw register read/write. The hardware-access layer shared
  by every higher-level tool (calibration CLI, dashboards, ``robot``
  backends, ...).
- :mod:`soarm_sdk.bus.servo_config` — plan and apply a batch of servo
  EEPROM changes (IDs, angle limits, speed, torque, mode, baud) over that
  bus. Formerly the flat ``soarm_sdk.calibration`` module; renamed to free
  "calibration" for :mod:`soarm_sdk.calibration` (the URDF-frame mapping),
  a different concern this has nothing to do with.
"""

from __future__ import annotations

from .discovery import (
    list_ports,
    get_available_ports,
    print_ports,
    scan_servos,
    discover_servos,
    read_diagnostics,
    read_servo_diagnostics,
    write1,
    write2,
)
from .discovery import _list_ports_mod  # noqa: F401 -- test monkeypatch hook
from .servo_config import (
    OperationPlan,
    parse_range,
    parse_mapping,
    parse_bool_mapping,
    build_operation_plan,
    resolve_id,
    collect_final_ids,
    apply_plan,
    run_calibration,
)

__all__ = [
    # Discovery / diagnostics / raw register access
    "list_ports",
    "get_available_ports",
    "print_ports",
    "scan_servos",
    "discover_servos",
    "read_diagnostics",
    "read_servo_diagnostics",
    "write1",
    "write2",
    # Servo EEPROM configuration planning
    "OperationPlan",
    "parse_range",
    "parse_mapping",
    "parse_bool_mapping",
    "build_operation_plan",
    "resolve_id",
    "collect_final_ids",
    "apply_plan",
    "run_calibration",
]
