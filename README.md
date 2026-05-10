# STServo SDK for Python

A modernized, typed, and pip-installable wrapper around the legacy STServo
serial SDK. This package keeps the familiar class names (`PortHandler`,
`protocol_packet_handler`, `sts`, `scscl`, etc.) while providing a clean module
layout that plays nicely with contemporary Python tooling.

## Installation

```bash
pip install stservo-sdk
```

To install from a local checkout:

```bash
pip install .
```

## Quickstart

```python
from stservo_sdk import PortHandler, sts

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
python examples/homing_calibrate.py --help
```

The script automatically adds the local `src/` directory to `PYTHONPATH` when
needed, so the command above works as long as it's run from the repository
root. Alternatively, install the package (for example with `pip install .`) and
call the tool from anywhere.

Launch the interactive text UI if you prefer guided prompts instead of CLI
flags:

```bash
python examples/homing_calibrate.py --ui
```

Prefer a dashboard? Start the Streamlit interface (note the `python -m` so
Streamlit uses the same interpreter environment as your calibration tools):

```bash
python -m streamlit run examples/homing_dashboard.py
```

If Streamlit reports that `serial` (pyserial) is missing, install it into the
same environment you used above:

```bash
python -m pip install pyserial
```

## Features

### Unit Conversion — `stservo_sdk.conversions`

Converts between raw encoder ticks (0–4095) and SI units so higher-level
modules never have to hard-code the encoder resolution.

**Implementation:** `src/stservo_sdk/conversions.py`  
All symbols are re-exported from `stservo_sdk` at the package level.

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
from stservo_sdk.conversions import ticks_to_radians, radians_to_ticks

rad = ticks_to_radians(2560)          # → ~0.8 rad
tick = radians_to_ticks(1.57)         # → 3073
```

---

### Real-Hardware Robot Interface — `ServoHardwareInterface`

Drop-in replacement for the MuJoCo-backed robot in `fullstack_manip`. Implements
`RobotInterface` so it can be passed directly to `MotionExecutor` without any
changes to controllers or planners.

**Implementation:** `fullstack_manip/core/hardware_interface.py`

**Architecture:**
- A background daemon thread runs at `state_freq` Hz (default 100 Hz) and
  keeps a cached joint state updated via one `GroupSyncRead.txRxPacket()` call
  per iteration.
- Write calls convert radians → ticks and broadcast a single `GroupSyncWrite`
  packet to all joints atomically.

```python
from fullstack_manip.core.hardware_interface import ServoHardwareInterface

hw = ServoHardwareInterface(
    port="/dev/tty.usbserial-XXXX",  # macOS; Linux: /dev/ttyUSB0
    baud=1_000_000,
    joint_ids=[1, 2, 3, 4, 5, 6],   # default soarm100 IDs
    torque_on_start=True,
)

with hw:  # opens port, starts background reader, enables torque
    q = hw.get_robot_joint_positions()          # → np.ndarray (radians)
    state = hw.get_robot_joint_state()          # → RobotState (pos, vel, effort)
    hw.set_robot_joint_positions(target_q)      # broadcast sync packet
```

Swap into an existing `MotionExecutor`:

```python
# Before (simulation):
# executor = MotionExecutor(robot=mujoco_robot, ...)

# After (real hardware):
executor = MotionExecutor(robot=hw, ...)
```

---

### Dashboard — `examples/homing_dashboard.py`

Launch with:

```bash
cd stservo-sdk
conda activate robot-irl
python -m streamlit run examples/homing_dashboard.py
```

The dashboard has 9 tabs:

| Tab | Feature |
|---|---|
| Calibration | Scan, assign IDs, set limits/acc/speed/mode/torque/baud |
| Inspector | Full telemetry snapshot for selected servos |
| Live Telemetry | Real-time position & speed charts (configurable Hz) |
| Command | Per-joint sliders; servo & wheel mode; sync-write |
| Homing Wizard | 6-step guided zero-calibration + offset fine-tuning |
| Recorder | Record demo trajectories → CSV; replay via sync packets |
| Health | Continuous thermal/current/overload monitor + alert log |
| Config | Export/import register snapshots; workspace sweep; [**EEPROM Diff**](#eeprom-diff) |
| Visual Servoing | Live camera feed, ArUco detection, joint command panel |

#### GroupSyncRead Poll Optimization

`_maybe_poll` now issues a single `GroupSyncRead.txRxPacket()` that returns
position + speed for all monitored joints in one bus transaction, then extracts
per-joint values with `gsr.getData()`. This replaces the previous per-servo
`ReadPosSpeed` loop and reduces round-trips by ~6× at 50 Hz.

#### Persistent Telemetry Log (Health tab)

Logs `(timestamp, servo_id, position, speed, temperature, current, voltage,
status)` rows to a SQLite database in real-time.

- **Start Logging** / **Stop Logging** toggle; database path is configurable.
- **Download DB** streams the `.db` file to your browser.
- **Query UI**: filter by servo IDs and time window, preview results as a table.

Default database path: `~/.stservo_telemetry.db`

#### EEPROM Diff (Config tab)

Compares a previously exported JSON snapshot against the current live register
state of the servos.

1. Export a snapshot with **Read & Export** (or load an existing one).
2. Click **Compare Snapshot vs Live** — the tool reads the current register
   state and renders a side-by-side table. Rows where the values differ are
   highlighted in red.

Useful after a firmware update, hardware swap, or re-homing to confirm that
the servo configuration is unchanged.

#### Trajectory Preview (Recorder tab)

After uploading a trajectory CSV, the tab renders an overlaid Plotly
time-series of all joint positions before any packets are sent. A **Dry-Run
Preview** button confirms the chart is accurate without touching the servos.
Duration and frame-count readouts are shown above the chart.

#### Visual Servoing Tab

Provides a live camera debug view alongside joint state and command controls.

- **Camera Feed**: captures frames from any `cv2.VideoCapture` index; frame
  rate matches the Streamlit rerun interval.
- **ArUco Detection**: overlay of detected marker IDs and centroid positions
  (dictionary and parameters are configurable via the UI).
- **Freeze Frame**: captures a single frame for annotation or export without
  stopping the servo bus.
- **Send All Joints**: broadcasts a sync-write packet to all commanded joints
  directly from the tab.

Requires OpenCV (already available in the `robot-irl` conda environment):

```bash
conda activate robot-irl
python -m pip install opencv-contrib-python-headless  # if not present
```

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
