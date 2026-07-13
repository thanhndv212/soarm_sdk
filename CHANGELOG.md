# Changelog

All notable changes to `soarm-sdk` are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

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
