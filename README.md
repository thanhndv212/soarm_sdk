# soarm_sdk

Python SDK for SO-ARM type manipulators driven by Feetech STS/SCS series
serial bus servos. Provides typed, pip-installable access to the Feetech
protocol via familiar class names (`PortHandler`, `protocol_packet_handler`,
`sts`, `scscl`, etc.) with a clean module layout compatible with contemporary
Python tooling.

## Package layout

```
soarm_sdk/
├── protocol/     Feetech wire protocol (ports, packets, sync r/w)
├── bus/          port discovery, diagnostics, servo EEPROM config
├── robot/        RobotInterface / Robot abstraction + backends (ServoRobot, NullRobot)
├── calibration/  tick <-> URDF-frame mapping (measurement, seeding, storage)
├── kinematics/   URDF loading + forward kinematics (no viewer dependency)
├── trajectory.py waypoint resampling for streaming to a robot
├── dashboard/    the Viser-based operator dashboard
└── cli/          console-script entry points (soarm-calibrate, soarm-dashboard, ...)
```

Every name importable from the flat top level in earlier releases
(`from soarm_sdk import PortHandler, sts, ...`) still works, and that
top-level namespace is the recommended entry point.

The pre-0.2.0 *submodule* paths (`soarm_sdk.servo_robot`,
`soarm_sdk.stservo_def`, ...) were deprecated in 0.2.0 and **removed in
0.3.0**. If you are coming from 0.1.x, the [changelog](CHANGELOG.md) has the
full old → new mapping.

## Installation

```bash
pip install soarm-sdk
```

To install from a local checkout:

```bash
pip install .
```

## Quickstart

```python
from soarm_sdk import PortHandler, sts

# Create a port handler and open the serial connection
handler = PortHandler("/dev/ttyUSB0")
if not handler.openPort():
    raise RuntimeError("Failed to open port")

# Configure baudrate (defaults to 1 Mbps)
if not handler.setBaudRate(1_000_000):
    raise RuntimeError("Failed to set baud rate")

# Construct an STS helper
servo = sts(handler)

# Ping a servo with ID 1
model, result, error = servo.ping(1)
if result != 0:
    raise RuntimeError(f"Ping failed: error={error}")

print("Servo model:", model)
```

## Examples

The [`examples/`](examples/) folder hosts runnable utilities that rely on the
package. For instance, the homing and calibration helper can be invoked from a
fresh source checkout without installing the project:

```bash
python examples/calibrate.py --help
```

That launcher covers both calibration tools, which do unrelated jobs:
`bus` sets up servos on the wire (IDs, EEPROM angle limits, speed), while
`rom` drives each joint into its hard stops to measure travel and writes
the arm's URDF-frame calibration.

```bash
python examples/calibrate.py bus --scan-range 1-6
python examples/calibrate.py rom --arm-id <name>
```

The script automatically adds the local `src/` directory to `PYTHONPATH` when
needed, so the command above works as long as it's run from the repository
root. Alternatively, install the package (for example with `pip install .`) and
call the tool from anywhere.

Launch the interactive text UI if you prefer guided prompts instead of CLI
flags:

```bash
python examples/calibrate.py bus --ui
```

Prefer a full dashboard? Start the Viser-based interface:

```bash
python examples/viser_dashboard.py
```

Optional arguments:

```
--device /dev/ttyXXX   Serial port (auto-detected if omitted)
--baud 1000000         Baud rate (default 1 000 000)
--port 8080            Viser HTTP port
--urdf PATH            URDF for 3-D FK visualisation
--interval-ms 200      Background polling interval
```

The dashboard requires the `viser` extra:

```bash
pip install soarm-sdk[viser]
```

Installing the package also gives you these directly on `$PATH` — no
checkout needed: `soarm-calibrate`, `soarm-calibrate --ui`,
`soarm-dashboard`, `soarm-dashboard-setup`, `soarm-seed-calibration`.

## Features

### Unit Conversion — `soarm_sdk.conversions`

Converts between raw encoder ticks (0–4095) and SI units so higher-level
modules never have to hard-code the encoder resolution.

**Implementation:** `src/soarm_sdk/conversions.py`  
All symbols are re-exported from `soarm_sdk` at the package level.

| Function | Description |
|---|---|
| `ticks_to_radians(ticks, zero_offset=2048)` | Encoder ticks → radians |
| `radians_to_ticks(radians, zero_offset=2048)` | Radians → ticks, clamped to [0, 4095] |
| `speed_ticks_to_rad_s(speed_ticks)` | Signed speed register → rad/s |
| `rad_s_to_speed_ticks(rad_s)` | rad/s → signed speed ticks, clamped to ±3000 |
| `joint_ticks_to_radians(ticks_list, zero_offsets, direction_signs)` | Batch conversion for soarm100 (6 joints) |
| `joint_radians_to_ticks(rads_list, ...)` | Inverse batch conversion |

Constants:

| Constant | Value | Meaning |
|---|---|---|
| `TICKS_PER_REV` | 4096 | Encoder resolution |
| `TICK_ZERO` | 2048 | Default electrical midpoint (= 0 rad) |
| `SOARM100_DIRECTION_SIGNS` | `[1,1,1,1,1,1]` | Per-joint axis sign (verify against hardware) |

```python
from soarm_sdk.conversions import ticks_to_radians, radians_to_ticks

rad = ticks_to_radians(2560)          # → ~0.8 rad
tick = radians_to_ticks(1.57)         # → 3073
```

---

### Robot Interface — `soarm_sdk.robot`

The abstraction boundary between algorithm code and hardware. Application code
(planners, teleop, RL policies) should speak `RobotInterface` — a structural
`typing.Protocol`, so a simulation backend elsewhere can satisfy it without
depending on this package at all — and never a specific backend.

| Backend | What it drives |
|---|---|
| `ServoRobot` | Real STS3215 hardware over RS-485 |
| `NullRobot` | Nothing — tracks commanded state in memory (tests, CI, offline runs) |

```python
from soarm_sdk import ServoRobot

robot = ServoRobot(
    port="/dev/tty.usbserial-XXXX",   # macOS; Linux: /dev/ttyUSB0
    max_step_rad=0.05,                # per-command bound, on top of joint limits
)

with robot:                            # opens port, starts background reader
    q = robot.get_joint_positions()    # -> np.ndarray (radians)
    state = robot.get_joint_state()    # -> JointState (pos, vel, effort, timestamp)
    robot.set_joint_positions(target_q)
```

Swapping in the no-hardware backend changes nothing else at the call site:

```python
from soarm_sdk import NullRobot
robot = NullRobot()                    # same interface, no serial port
```

**Under the hood** (`robot/hardware.py`, `ServoHardwareInterface` — reach for it
directly only if you need register-level control):

- A background daemon thread runs at `state_freq` Hz (default 100 Hz) and keeps
  a cached joint state updated via one `GroupSyncRead.txRxPacket()` per
  iteration.
- Write calls convert radians -> ticks and broadcast a single `GroupSyncWrite`
  packet to all joints atomically.
- Declared joint limits and the per-step bound are enforced on every write, and
  clamps are counted (`limit_clamps`, `step_clamps`) rather than failing
  silently.

---

### Dashboard — `soarm-dashboard`

A browser-based control panel built on [Viser](https://viser.studio) with
real-time 3-D forward-kinematics visualisation. The server starts on
`http://localhost:8080` by default.

```bash
soarm-dashboard --device /dev/ttyUSB0
# from a checkout without installing: python examples/viser_dashboard.py ...
```

The dashboard has 7 tabs:

| Tab | Description |
|---|---|
| **Start Up** | Connect / disconnect polling thread; quick torque on/off; scan servos by ID range |
| **Homing Wizard** | Automatic motor-sweep ROM detection or manual hand-teach; write offsets + angle limits to EEPROM; save `soarm100_rom.json` |
| **PID Tuning** | Read / write P / D / I gains (EEPROM); step-response chart with live 20 Hz position trace and automatic metrics |
| **Command Panel** | Per-joint position / speed / acceleration sliders; servo and wheel mode; sync-write to all joints |
| **Recorder** | Record joint trajectories from hardware to CSV; replay at configurable speed and loop count |
| **Monitor** | 20 s rolling uPlot charts (position + speed); joint telemetry table; health (temp / current); full register inspector |
| **Reconfigure** | Calibration: scan, assign IDs, angle limits, acc/speed/mode/torque/baud; Config: export/import register snapshots as JSON |

Shared sidebar controls (visible across all tabs): serial device, baud rate,
polling interval, and connection status.

#### 3-D FK Visualisation

When a URDF is available (default: `SO-ARM100/Simulation/SO100/so100.urdf`)
all link meshes are loaded into the Viser 3-D scene and updated at ~3 Hz from
the live encoder positions. FK is computed with `yourdfpy` + `trimesh`.

Install the optional dependencies:

```bash
pip install yourdfpy trimesh
```

#### Homing Wizard

**Automatic mode** — drives each joint in wheel mode, detects stall at both
limits, records min/max ticks, restores servo mode, moves to midpoint. A
**Dry-run** checkbox simulates the sweep with FK animation without touching
hardware.

**Manual mode** — disable torque, move joints by hand, record min/max via
per-joint buttons, then compute offsets. Both modes share the same
*Apply* (write EEPROM) and *Save JSON* actions.

#### PID Tuning — Step Response

After writing new P/D/I gains, fire a position step from the **Step Response**
folder:

1. Set **Step target**, speed, acceleration, and duration.
2. Press **Send Step** — the servo moves and positions are polled at 20 Hz.
3. The uPlot chart shows **Reference** (red) and **Actual** (blue) traces,
   updating live and freezing at completion.
4. Metrics are computed automatically:

| Metric | Definition |
|---|---|
| Steady-state error | Mean of last 20 % of samples vs target |
| Overshoot | Peak excursion beyond target (ticks and %) |
| Peak time | Time to reach the peak |
| Rise time | 10 % → 90 % of step amplitude |
| Settling time | Last instant \|pos − target\| > 2 % band |

#### Monitor — Live Charts

Two 20-second rolling uPlot charts (position and speed) are updated at
100 ms from a shared `deque` buffer in the main display loop. A joint
telemetry table, health snapshot (temp / current, updated every 5 polls),
and a full register inspector (`read_servo_diagnostics`) are co-located in
the same tab.

#### GroupSyncRead Poll Architecture

The background daemon thread issues a single `GroupSyncRead.txRxPacket()`
per interval to retrieve position + speed for all joints in one bus
transaction, falling back to per-servo `ReadPosSpeed` on failure. Temperature
and current are sampled every 5th iteration to reduce bus load.

---

## Documentation

Extended usage notes and API details live in [`docs/usage.md`](docs/usage.md).

## Development

```bash
# lint
ruff check src

# tests
pytest
```

The project uses [Hatch](https://hatch.pypa.io/) for packaging. Run
`hatch build` to produce distribution artifacts.
