# Changelog

All notable changes to `soarm-sdk` are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

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
  `test_protocol_packet_handler.py`, `test_calibration.py`, `test_bus.py`.
  Covers unit conversions, packet framing/checksum logic, calibration
  planning, and bus discovery control flow with fake port/packet-handler
  doubles — no real hardware required to run them.
- `py.typed` marker, so consumers get real type checking instead of `Any`.
- `LICENSE` (MIT) file, matching the license already declared in
  `pyproject.toml`.
- `hatch run test` script (`pytest`), and `[tool.pytest.ini_options]`
  `testpaths` so plain `pytest` works from the package root.
- `.github/workflows/ci.yml` — lint (`ruff check src tests`) and test
  (`pytest`) on push/PR, matrixed over Python 3.9–3.12.
- This file.

### Fixed

- Removed unused imports in `calibration.py` (`Iterable`, `discover_servos`,
  `COMM_SUCCESS`) flagged by `ruff check src`, which had never been run in CI.
- `[project.urls]` pointed at the `fullstack-manip` repo (this package's
  origin before it was split out); now points at `soarm_sdk`'s own repo.

## [0.1.0] - 2026-05-10

Initial release: Feetech STS/SCS servo protocol (`PortHandler`,
`ProtocolPacketHandler`, `sts`, `scscl`, `GroupSyncRead`/`GroupSyncWrite`),
`bus.py` port discovery/diagnostics helpers, `calibration.py` operation
planning, `conversions.py` unit conversions, and the `examples/` calibration
CLI and Viser dashboard.
