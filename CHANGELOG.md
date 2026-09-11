# Changelog

All notable changes to `soarm-sdk` are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed — package reorganization

The flat 20-module top-level namespace is now grouped by layer, each with
a real subpackage. Every old import path still works (see "Deprecated"
below), so this is additive for existing code, but new code should use
the new paths:

- `soarm_sdk.protocol` — the Feetech wire protocol (`port_handler`,
  `packet_handler` [was `protocol_packet_handler`], `group_sync_read`,
  `group_sync_write`, `sts`, `scscl`, `registers` [was `stservo_def`]).
- `soarm_sdk.bus` — port discovery/diagnostics (`discovery`, was the flat
  `bus.py`) plus servo EEPROM configuration planning (`servo_config`,
  **moved from `soarm_sdk.calibration`** — see below).
- `soarm_sdk.robot` — the `RobotInterface`/`Robot` abstraction layer
  (`types`, `interfaces`, `base`, `hardware`, `servo`), plus a new `null`
  backend (see "Added").
- `soarm_sdk.calibration` — **now the URDF-frame mapping package**
  (`frame`, was `frame_calibration.py`; `seed`, was `seed_calibration.py`),
  plus a new `rom_sweep` module (see "Added").
- `soarm_sdk.kinematics` — URDF loading + forward kinematics, split out of
  `dashboard/fk.py` so it has no Viser dependency.
- `soarm_sdk.cli` — the calibration TUI and dashboard launchers, moved out
  of `examples/` into the installable package (see "Added").

**Breaking:** `soarm_sdk.calibration` (servo EEPROM reconfiguration —
`OperationPlan`, `apply_plan`, `parse_range`, ...) moved to
`soarm_sdk.bus.servo_config` / `soarm_sdk.bus`. The name `calibration` was
freed up because it was doing double duty for two unrelated concepts —
servo register configuration vs. the tick-to-URDF-frame mapping — and the
latter (`frame_calibration.py`) wasn't even reachable from the top-level
`soarm_sdk` namespace. Everything reachable via `from soarm_sdk import
...` (`OperationPlan`, `apply_plan`, `run_calibration`) is unaffected;
only a direct `from soarm_sdk.calibration import ...` submodule import
needs updating, and only for the servo-config names.

### Deprecated

The following top-level modules are now thin re-export shims over their new
subpackage location, and **emit a `DeprecationWarning` on import naming
their replacement**. They still work — no import using them is broken — but
they are **scheduled for removal in 0.3.0**:

| Deprecated | Use instead |
|---|---|
| `soarm_sdk.port_handler` | `soarm_sdk.protocol.port_handler` |
| `soarm_sdk.protocol_packet_handler` | `soarm_sdk.protocol.packet_handler` |
| `soarm_sdk.group_sync_read` | `soarm_sdk.protocol.group_sync_read` |
| `soarm_sdk.group_sync_write` | `soarm_sdk.protocol.group_sync_write` |
| `soarm_sdk.sts` | `soarm_sdk.protocol.sts` |
| `soarm_sdk.scscl` | `soarm_sdk.protocol.scscl` |
| `soarm_sdk.stservo_def` | `soarm_sdk.protocol.registers` |
| `soarm_sdk.types` | `soarm_sdk.robot.types` |
| `soarm_sdk.interfaces` | `soarm_sdk.robot.interfaces` |
| `soarm_sdk.hardware_interface` | `soarm_sdk.robot.hardware` |
| `soarm_sdk.servo_robot` | `soarm_sdk.robot.servo` |
| `soarm_sdk.frame_calibration` | `soarm_sdk.calibration.frame` |
| `soarm_sdk.seed_calibration` | `soarm_sdk.calibration.seed` |
| `python -m soarm_sdk.seed_calibration` | `soarm-seed-calibration` |

Importing from the top-level `soarm_sdk` namespace is **not** deprecated and
never warns — `from soarm_sdk import ServoRobot, RobotCalibration, ...` stays
the recommended entry point.

Python hides `DeprecationWarning` by default outside `__main__`; run with
`python -W default::DeprecationWarning` (or under pytest, which shows them)
to see which call sites still need updating.

Nothing inside the package, its tests, or any package in this workspace
imports through a shim any more — `tests/test_deprecated_shims.py` asserts
that importing `soarm_sdk` emits no deprecation warnings of its own, so the
only warnings you can see are from your own code.

### Added

- `soarm_sdk.robot.NullRobot` — an in-memory `Robot` implementation with
  no hardware behind it, satisfying `RobotInterface` completely. Useful
  for testing SDK-consuming code, CI, and any call site that wants "a
  robot" without one connected. Note it is *not* a drop-in for every
  existing `dry_run` flag in the workspace: soarm_tamp's `execute.py`
  defers its `soarm_sdk` import specifically so `--dry-run` stays
  runnable inside the HPP planning container, where the SDK isn't
  installed at all — using `NullRobot` there would reintroduce the
  import it is avoiding.
- `soarm_sdk.robot.LeRobotRobot` — the same servo bus driven through
  lerobot's `SOFollower` instead of this SDK's protocol stack, behind the
  same `RobotInterface`. m5teleop had a hand-rolled wrapper (`ArmInterface`)
  that did not implement the interface, so teleop code could not be pointed
  at a planner's robot, a simulation, or `NullRobot` without a rewrite.
  lerobot stays an optional dependency (`pip install soarm-sdk[lerobot]`),
  imported lazily in `connect()`; a `follower_factory` hook makes the
  backend testable without it. Note it converts lerobot's *normalized*
  degrees to radians and nothing more — that is not the URDF frame a
  planner speaks; see `calibration/frame.py`.
- **Config-driven gripper API** on the `Robot` base class — `set_gripper()`,
  `toggle_gripper()`, `gripper_is_open`, `gripper_index`, driven by an
  optional `gripper:` block (`joint_index`, `open_rad`, `closed_rad`) now
  present in `configs/soarm100.yaml`. m5teleop and soarm_tamp each carried
  their own `GRIPPER_OPEN_DEG`/`CLOSED_DEG` constants and their own "which
  joint is the jaw" assumption; this makes it a property of the robot.
- `soarm_sdk.trajectory.resample()` — linear waypoint interpolation
  bounding the per-joint step between consecutive commands. Generalizes a
  pattern reimplemented per-caller around planned/recorded paths (e.g.
  soarm_tamp's waypoint-manifest executor).
- `soarm_sdk.calibration.rom_sweep` (`run_rom_sweep`, `simulate_rom_sweep`)
  — the range-of-motion sweep `seed_from_travel()` consumes as input,
  extracted from the Homing Wizard dashboard panel so it's testable and
  usable without a browser. The panel is now a thin GUI wrapper over it.
- `soarm_sdk.robot.config.validate_robot_config()` — checks a robot config
  dict's required keys, types, and array lengths up front. `Robot.__init__`
  now raises one readable `ConfigError` naming every problem, instead of a
  bare `KeyError` (or a silent shape mismatch) surfacing later from
  whichever accessor happens to touch the bad field first.
- `soarm_sdk.kinematics` — URDF loading (`load_urdf`) and forward
  kinematics (`link_transforms`) with no dependency on a viewer, so a
  planner or a headless test can use FK without pulling in Viser.
- `[project.scripts]`: `soarm-calibrate`, `soarm-dashboard`,
  `soarm-dashboard-setup`, `soarm-seed-calibration` — the calibration CLI
  and dashboard launchers are now real console scripts (`pip install
  soarm-sdk` puts them on `$PATH`), not just scripts you run from a
  checkout. The `examples/*.py` scripts still work identically as thin
  launchers over the same code.
- `frame_calibration.py` — the mapping between raw servo ticks and the
  URDF's joint frame, which nothing in this workspace previously had.
  `ServoHardwareInterface` always accepted `zero_offsets`/`direction_signs`
  but no code ever produced them, so the package config's static 2048 was
  in effect a guess. `seed_from_travel()` estimates them from measured
  travel plus the URDF's joint limits, records the resulting uncertainty
  per joint, and persists a `RobotCalibration` marked `validated: false`
  until a physical check confirms the direction signs — which a travel
  range cannot determine, since it says how far a joint moves and not
  which end is which.
- `seed_calibration.py` — CLI that seeds a calibration from an existing
  lerobot calibration file, offline, no hardware. It reads only the travel
  ranges; `homing_offset` is deliberately ignored because lerobot writes it
  into servo EEPROM, so the ticks this SDK reads already include it.
- **Joint-limit enforcement on the write path.** `joint_limits_lower/upper`
  had been declared in `configs/soarm100.yaml` and exposed via
  `get_joint_limits()` since the beginning, but nothing read them when
  commanding — the only clamp was `radians_to_ticks` pinning to [0, 4095],
  which is protocol validity, not safety.
- **Per-step clamping** (`max_step_rad`), bounding how far a joint may be
  commanded from its last *measured* position, so a lagging joint cannot
  accumulate an ever-larger jump. Both clamps count rather than fail
  silently; `limit_clamps` and `step_clamps` are public so a caller can
  assert on them after a run.
- `ServoRobot` now accepts `calibration`, `max_step_rad` and
  `enforce_limits`, and passes the config's declared joint limits through
  by default.

- `tests/` — first test suite for the package: `test_conversions.py`,
  `test_protocol_packet_handler.py`, `test_servo_config.py` (originally
  `test_calibration.py`, renamed alongside the module move above),
  `test_bus.py`. Covers unit conversions, packet framing/checksum logic,
  servo-config planning, and bus discovery control flow with fake
  port/packet-handler doubles — no real hardware required to run them.
- `tests/test_null_robot.py`, `tests/test_trajectory.py`,
  `tests/test_robot_config.py`, `tests/test_rom_sweep.py` — coverage for
  the additions above.
- `py.typed` marker, so consumers get real type checking instead of `Any`.
- `LICENSE` (MIT) file, matching the license already declared in
  `pyproject.toml`.
- `hatch run test` script (`pytest`), and `[tool.pytest.ini_options]`
  `testpaths` so plain `pytest` works from the package root.
- `.github/workflows/ci.yml` — lint (`ruff check src tests`) and test
  (`pytest`) on push/PR, matrixed over Python 3.9–3.12.
- This file.

### Fixed

- **`run_calibration(args)` crashed with `ModuleNotFoundError` when
  `--list-ports` was passed** (so `soarm-calibrate --list-ports` was broken).
  The reorganization moved this module under `bus/`, which silently changed
  what its function-local `from .bus import print_ports` resolved to. Being a
  lazy import inside a rarely-taken branch, nothing caught it until every
  relative import in the package was resolved against the filesystem; now
  covered by a regression test.
- **A `TYPE_CHECKING` import in `robot/hardware.py` pointed at
  `soarm_sdk.robot.frame_calibration`**, which does not exist. Invisible at
  runtime (the branch never executes) but wrong for any type checker.
- Removed unused imports in `calibration.py` (now `bus/servo_config.py`)
  (`Iterable`, `discover_servos`, `COMM_SUCCESS`) flagged by
  `ruff check src`, which had never been run in CI.
- `[project.urls]` pointed at the `fullstack-manip` repo (this package's
  origin before it was split out); now points at `soarm_sdk`'s own repo.

## [0.1.0] - 2026-05-10

Initial release: Feetech STS/SCS servo protocol (`PortHandler`,
`ProtocolPacketHandler`, `sts`, `scscl`, `GroupSyncRead`/`GroupSyncWrite`),
`bus.py` port discovery/diagnostics helpers, `calibration.py` operation
planning, `conversions.py` unit conversions, and the `examples/` calibration
CLI and Viser dashboard.
