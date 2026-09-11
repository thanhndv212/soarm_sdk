# soarm_sdk Usage Guide

This guide highlights the most common workflows when interacting with soarm_sdk
actuators using the modernized Python SDK.

## Connecting to a Servo Bus

```python
from soarm_sdk import PortHandler

handler = PortHandler("/dev/ttyUSB0")
if not handler.openPort():
    raise RuntimeError("Failed to open port")

# Optional: change the baud rate from the default 1 Mbps.
if not handler.setBaudRate(1_000_000):
    raise RuntimeError("Failed to set baudrate")
```

The `PortHandler` preserves the original method names so existing scripts keep
working. The implementation relies on `pyserial` under the hood.

## Working with STS Servos

```python
from soarm_sdk import sts

servo = sts(handler)

# Read the current position (returns position, comm_result, error)
position, comm_result, err = servo.ReadPos(servo_id=1)
if comm_result == 0:
    print("Position:", position)
else:
    print("ReadPos failed with", comm_result, err)

# Queue a synchronous trajectory for multiple servos
servo.SyncWritePosEx(servo_id=1, position=2048, speed=200, acc=50)
servo.SyncWritePosEx(servo_id=2, position=1024, speed=200, acc=50)
servo.groupSyncWrite.txPacket()
```

## Working with SCSCL Servos

```python
from soarm_sdk import scscl

arm = scscl(handler)
arm.WritePos(servo_id=1, position=2048, time=500, speed=100)
```

## Protocol-Level Access

The original `protocol_packet_handler` class remains available for lower-level
packet manipulations:

```python
from soarm_sdk import protocol_packet_handler

packet_handler = protocol_packet_handler(handler, protocol_end=0)
data, result, error = packet_handler.readTxRx(1, address=40, length=2)
```

Refer to the class docstrings in the `soarm_sdk` package for additional
methods. All modules ship with type hints and docstrings to improve IDE support
and readability.

## Dashboard UI

For an interactive dashboard that mirrors the CLI options, launch the Streamlit
app from the repository root (using `python -m` ensures Streamlit shares the
same environment as your calibration tools):

```bash
python -m streamlit run examples/homing_dashboard.py
```

If the dashboard reports that the `serial` module is missing, install pyserial
for the same interpreter:

```bash
python -m pip install pyserial
```

The app lists detected serial ports, lets you configure calibration parameters,
and streams live logs as the process runs.

---

## Unit Conversion

The `soarm_sdk.conversions` module bridges raw encoder ticks and SI units so
higher-level code works in radians and rad/s rather than hardware register values.

```python
from soarm_sdk.conversions import (
    ticks_to_radians,
    radians_to_ticks,
    speed_ticks_to_rad_s,
    rad_s_to_speed_ticks,
    joint_ticks_to_radians,
    joint_radians_to_ticks,
    TICK_ZERO,           # 2048 — electrical midpoint
    TICKS_PER_RAD,       # ~651.9 ticks per radian
    SOARM100_DIRECTION_SIGNS,   # per-joint axis signs, default [1,1,1,1,1,1]
)

# Scalar helpers
rad  = ticks_to_radians(3072)              # (3072 - 2048) * 2π/4096 ≈ 1.571 rad
tick = radians_to_ticks(1.571)            # → 3073
spd  = speed_ticks_to_rad_s(300)          # → ~0.46 rad/s
```

### soarm100 array helpers

```python
# Convert all 6 joints at once (zero_offsets and direction_signs can be
# overridden per-joint; defaults match a freshly homed soarm100)
rads  = joint_ticks_to_radians([2048, 2048, 2048, 2048, 2048, 2048])
# → [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

ticks = joint_radians_to_ticks([0.0, 0.785, -0.785, 0.0, 0.0, 0.0])
# → [2048, 2561, 1535, 2048, 2048, 2048]
```

Each joint can have a custom `zero_offset` (tick value corresponding to 0 rad)
and a `direction_sign` (+1 or -1). Verify direction signs against your physical
robot: command +0.1 rad on each joint and confirm the arm moves in the expected
direction.

---

## GroupSyncRead for Bulk State Reads

For high-frequency control loops, prefer `GroupSyncRead` over 6 separate
`ReadPosSpeed` calls. A single sync-read packet returns position + speed for
all servos in one bus transaction (~6× fewer round-trips).

```python
from soarm_sdk import PortHandler, sts
from soarm_sdk.protocol.registers import (
    COMM_SUCCESS, STS_PRESENT_POSITION_L, STS_PRESENT_SPEED_L
)

ph = PortHandler("/dev/ttyUSB0")
ph.openPort(); ph.setBaudRate(1_000_000)
srv = sts(ph)

gsr = srv.groupSyncRead  # GroupSyncRead(STS_PRESENT_POSITION_L, 4)
joint_ids = [1, 2, 3, 4, 5, 6]

# Register all joints once
for sid in joint_ids:
    gsr.addParam(sid)

# In the control loop:
result = gsr.txRxPacket()
if result == COMM_SUCCESS:
    for sid in joint_ids:
        avail, _ = gsr.isAvailable(sid, STS_PRESENT_POSITION_L, 2)
        if avail:
            raw_pos = gsr.getData(sid, STS_PRESENT_POSITION_L, 2)
            raw_spd = gsr.getData(sid, STS_PRESENT_SPEED_L, 2)
            pos = srv.sts_tohost(raw_pos, 15)   # decode 15-bit signed
            spd = srv.sts_tohost(raw_spd, 15)
```

---

## ServoHardwareInterface

`ServoHardwareInterface` exposes the `RobotInterface` Protocol so it can be
used anywhere `fullstack_manip` currently expects a MuJoCo robot object — no
changes to controllers, planners, or state estimators are needed.

```python
from fullstack_manip.core.hardware_interface import ServoHardwareInterface

hw = ServoHardwareInterface(
    port="/dev/tty.usbserial-XXXX",   # or /dev/ttyUSB0 on Linux
    baud=1_000_000,
    joint_ids=[1, 2, 3, 4, 5, 6],
    zero_offsets=None,                # defaults to [2048]*6
    direction_signs=None,             # defaults to SOARM100_DIRECTION_SIGNS
    default_speed=300,                # ticks/s for set_robot_joint_positions
    default_acc=50,
    state_freq=100.0,                 # background read loop Hz
    torque_on_start=True,
)

with hw:
    # Poll cached joint state (updated by background thread at 100 Hz)
    q   = hw.get_robot_joint_positions()    # np.ndarray, radians
    state = hw.get_robot_joint_state()      # RobotState: pos, vel, effort

    # Command all joints with one sync-write packet
    import numpy as np
    hw.set_robot_joint_positions(np.zeros(6))

    # Check communication health
    print(f"Read errors since start: {hw.read_errors}")
```

### Integrating with MotionExecutor

```python
from fullstack_manip.control.motion_executor import MotionExecutor

executor = MotionExecutor(
    robot=hw,            # was: mujoco_robot
    planner=planner,
    controller=pid,
)
executor.move_to_pose(target_ee_pos, duration=2.0)
```

The `set_robot_joint_positions` write path is thread-safe and fires a sync
packet immediately on the calling thread. The background reader thread holds a
lock only while updating the cached positions array.

### Notes

- EEPROM is never written by `ServoHardwareInterface`; only SRAM registers
  (ACC, GOAL_POSITION, GOAL_SPEED, TORQUE_ENABLE) are touched.
- `get_body_pose(body_name)` raises `NotImplementedError`. Forward kinematics
  requires a `ManipPlant` object; pass `hw.get_robot_joint_positions()` to it
  separately.
- Always call `hw.stop()` (or use the context-manager form) before exiting to
  close the serial port cleanly.
