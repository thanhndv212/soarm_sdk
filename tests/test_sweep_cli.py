"""Tests for the ROM-sweep calibration CLI.

The sweep itself is covered by ``test_rom_sweep``; what is tested here is
the chain the CLI adds on top — sweep results -> ``seed_from_travel`` ->
a loadable calibration file — exercised through the simulated sweep so it
needs no serial port.
"""

from __future__ import annotations

import pytest

from soarm_sdk.calibration.frame import RobotCalibration
from soarm_sdk.calibration.sweep_cli import main
from soarm_sdk.robot.base import load_robot_config


def _run(tmp_path, *extra):
    out = tmp_path / "calibration.json"
    rc = main(
        ["--dry-run", "--timeout", "0.05", "--out", str(out), *extra]
    )
    return rc, out


def test_dry_run_writes_a_loadable_calibration(tmp_path):
    rc, out = _run(tmp_path, "--arm-id", "test_arm")
    assert rc == 0
    cal = RobotCalibration.load(out)
    assert cal.arm_id == "test_arm"
    assert cal.names == list(load_robot_config("so101")["joint_names"])


def test_joint_order_follows_the_config_not_the_sweep(tmp_path):
    _, out = _run(tmp_path)
    cal = RobotCalibration.load(out)
    config = load_robot_config("so101")
    assert cal.names == list(config["joint_names"])
    # Servo IDs and joint names are positional partners; a calibration
    # written in a different order would silently mis-map every joint.
    assert len(cal.joints) == config["n_dof"]


def test_result_is_never_written_validated(tmp_path):
    """The direction signs are assumed, so nothing here may claim otherwise.

    wrist_roll is assumed -1, not +1 — see DEFAULT_DIRECTION_SIGN_OVERRIDES —
    but "assumed" either way; only a physical check may set validated=True.
    """
    _, out = _run(tmp_path)
    cal = RobotCalibration.load(out)
    assert cal.validated is False
    by_name = {j.name: j.direction_sign for j in cal.joints}
    assert by_name["wrist_roll"] == -1
    assert all(s == 1 for n, s in by_name.items() if n != "wrist_roll")


def test_measured_travel_is_recorded_in_notes(tmp_path):
    """The sweep is the evidence behind the numbers; losing it would leave
    the file unauditable a year from now."""
    _, out = _run(tmp_path)
    cal = RobotCalibration.load(out)
    sweep = cal.notes["sweep"]
    assert set(sweep) == set(cal.names)
    for entry in sweep.values():
        assert entry["pos_min"] < entry["pos_max"]
        assert entry["range_ticks"] == entry["pos_max"] - entry["pos_min"]


def test_dry_run_is_flagged_in_the_file(tmp_path):
    _, out = _run(tmp_path)
    cal = RobotCalibration.load(out)
    assert "dry_run" in cal.notes
    assert "SIMULATED" in cal.source


def test_bad_config_name_is_rejected(tmp_path):
    with pytest.raises(FileNotFoundError):
        _run(tmp_path, "--config", "no_such_robot")
