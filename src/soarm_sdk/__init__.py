"""Public exports for the soarm-sdk package."""

from .port_handler import PortHandler
from .protocol_packet_handler import (
    ProtocolPacketHandler,
    protocol_packet_handler,
)
from .group_sync_read import GroupSyncRead
from .group_sync_write import GroupSyncWrite
from .sts import sts
from .scscl import scscl
from .conversions import (
    TICKS_PER_REV,
    TICK_ZERO,
    TICKS_PER_RAD,
    RADS_PER_TICK,
    SOARM100_DIRECTION_SIGNS,
    ticks_to_radians,
    radians_to_ticks,
    speed_ticks_to_rad_s,
    rad_s_to_speed_ticks,
    joint_ticks_to_radians,
    joint_radians_to_ticks,
)

# -- Bus access (port discovery, scanning, diagnostics, register writes) ----
from .bus import (
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

# -- Calibration (operation planning and execution) -------------------------
from .calibration import (
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

# -- Frequently-used register constants re-exported for convenience ---------
from .stservo_def import (
    COMM_SUCCESS,
    COMM_RX_FAIL,
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

__all__ = [
    # Core protocol
    "PortHandler",
    "ProtocolPacketHandler",
    "protocol_packet_handler",
    "GroupSyncRead",
    "GroupSyncWrite",
    "sts",
    "scscl",
    # Conversions
    "TICKS_PER_REV",
    "TICK_ZERO",
    "TICKS_PER_RAD",
    "RADS_PER_TICK",
    "SOARM100_DIRECTION_SIGNS",
    "ticks_to_radians",
    "radians_to_ticks",
    "speed_ticks_to_rad_s",
    "rad_s_to_speed_ticks",
    "joint_ticks_to_radians",
    "joint_radians_to_ticks",
    # Bus access
    "list_ports",
    "get_available_ports",
    "print_ports",
    "scan_servos",
    "discover_servos",
    "read_diagnostics",
    "read_servo_diagnostics",
    "write1",
    "write2",
    # Calibration
    "OperationPlan",
    "parse_range",
    "parse_mapping",
    "parse_bool_mapping",
    "build_operation_plan",
    "resolve_id",
    "collect_final_ids",
    "apply_plan",
    "run_calibration",
    # Register constants
    "COMM_SUCCESS",
    "COMM_RX_FAIL",
    "STS_ACC",
    "STS_BAUD_RATE",
    "STS_GOAL_SPEED_L",
    "STS_ID",
    "STS_LOCK",
    "STS_MIN_ANGLE_LIMIT_L",
    "STS_MAX_ANGLE_LIMIT_L",
    "STS_MODE",
    "STS_TORQUE_ENABLE",
]
