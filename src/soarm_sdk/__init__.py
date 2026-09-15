"""Public exports for the soarm-sdk package.

Layout
------
- :mod:`soarm_sdk.protocol`    -- Feetech wire protocol (ports, packets, sync r/w)
- :mod:`soarm_sdk.bus`         -- port discovery, diagnostics, servo EEPROM config
- :mod:`soarm_sdk.robot`       -- the ``RobotInterface``/``Robot`` abstraction + backends
- :mod:`soarm_sdk.calibration` -- tick <-> URDF-frame mapping (measurement, seeding, storage)
- :mod:`soarm_sdk.kinematics`  -- URDF loading + forward kinematics (no viewer dependency)
- :mod:`soarm_sdk.trajectory`  -- waypoint resampling for streaming to a robot
- :mod:`soarm_sdk.dashboard`   -- the Viser-based operator dashboard
- :mod:`soarm_sdk.cli`         -- console-script entry points (``soarm-reconfigure``, ...)

This top-level module re-exports the names most applications need.
Everything else is still reachable through its owning submodule.
"""

from __future__ import annotations

# -- Protocol layer (Feetech wire protocol) ----------------------------------
from .protocol import (
    PortHandler,
    ProtocolPacketHandler,
    protocol_packet_handler,
    GroupSyncRead,
    GroupSyncWrite,
    sts,
    scscl,
)

# -- Conversions --------------------------------------------------------------
from .conversions import (
    TICKS_PER_REV,
    TICK_ZERO,
    TICKS_PER_RAD,
    RADS_PER_TICK,
    ticks_to_radians,
    radians_to_ticks,
    speed_ticks_to_rad_s,
    rad_s_to_speed_ticks,
    joint_ticks_to_radians,
    joint_radians_to_ticks,
)

# -- Bus access (port discovery, scanning, diagnostics, servo EEPROM config) --
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
    OperationPlan,
    apply_plan,
    run_calibration,
)

# -- Frequently-used register constants re-exported for convenience ---------
from .protocol.registers import (
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

# -- Characterisation rigs ---------------------------------------------------
from .diagnostics import (
    BacklashResult,
    DroopResult,
    measure_backlash,
    measure_droop,
)

# -- Robot interface layer: types, Protocol, backends ------------------------
from .robot import (
    Pose,
    JointState,
    ServoHealth,
    ServoSample,
    TelemetryStream,
    TelemetrySink,
    TelemetryRecorder,
    JsonlSink,
    RerunSink,
    RobotInterface,
    ConfigError,
    Robot,
    load_robot_config,
    ServoRobot,
    ServoHardwareInterface,
    NullRobot,
    LeRobotRobot,
)
from .rate_limiter import RateLimiter

# -- Calibration: tick <-> URDF-frame mapping --------------------------------
from .calibration import (
    JointCalibration,
    RobotCalibration,
    seed_from_travel,
)

# -- Trajectory: resampling for streaming to a robot -------------------------
from .trajectory import resample

__all__ = [
    # Protocol
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
    "OperationPlan",
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
    # Robot interface layer
    "Pose",
    "JointState",
    "ServoHealth",
    "ServoSample",
    "TelemetryStream",
    "TelemetrySink",
    "TelemetryRecorder",
    "JsonlSink",
    "RerunSink",
    "BacklashResult",
    "DroopResult",
    "measure_backlash",
    "measure_droop",
    "RobotInterface",
    "ConfigError",
    "Robot",
    "load_robot_config",
    "ServoRobot",
    "ServoHardwareInterface",
    "NullRobot",
    "LeRobotRobot",
    "RateLimiter",
    # Calibration
    "JointCalibration",
    "RobotCalibration",
    "seed_from_travel",
    # Trajectory
    "resample",
]
