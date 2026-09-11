"""The legacy top-level module paths still work, and say they are deprecated.

Two things are being pinned here:

1. Code written against the pre-reorg layout keeps importing successfully —
   the whole point of the shims.
2. Each one emits a ``DeprecationWarning`` naming its replacement, so the
   deprecation is discoverable rather than folklore.

Both matter until the shims are removed in 0.3.0; when that happens this
file goes with them.
"""

from __future__ import annotations

import importlib
import sys

import pytest

# legacy module -> (canonical module, a name that must survive the move)
SHIMS = {
    "soarm_sdk.port_handler": ("soarm_sdk.protocol.port_handler", "PortHandler"),
    "soarm_sdk.protocol_packet_handler": (
        "soarm_sdk.protocol.packet_handler",
        "ProtocolPacketHandler",
    ),
    "soarm_sdk.group_sync_read": (
        "soarm_sdk.protocol.group_sync_read",
        "GroupSyncRead",
    ),
    "soarm_sdk.group_sync_write": (
        "soarm_sdk.protocol.group_sync_write",
        "GroupSyncWrite",
    ),
    "soarm_sdk.sts": ("soarm_sdk.protocol.sts", "sts"),
    "soarm_sdk.scscl": ("soarm_sdk.protocol.scscl", "scscl"),
    "soarm_sdk.stservo_def": ("soarm_sdk.protocol.registers", "STS_TORQUE_ENABLE"),
    "soarm_sdk.types": ("soarm_sdk.robot.types", "JointState"),
    "soarm_sdk.interfaces": ("soarm_sdk.robot.interfaces", "RobotInterface"),
    "soarm_sdk.hardware_interface": (
        "soarm_sdk.robot.hardware",
        "ServoHardwareInterface",
    ),
    "soarm_sdk.servo_robot": ("soarm_sdk.robot.servo", "ServoRobot"),
    "soarm_sdk.frame_calibration": (
        "soarm_sdk.calibration.frame",
        "RobotCalibration",
    ),
    "soarm_sdk.seed_calibration": ("soarm_sdk.calibration.seed", "main"),
}


def _fresh_import(name):
    """Import *name* with a clean slate, so the module-level warning re-fires."""
    sys.modules.pop(name, None)
    return importlib.import_module(name)


@pytest.mark.parametrize("legacy", sorted(SHIMS))
def test_legacy_path_still_imports(legacy):
    with pytest.warns(DeprecationWarning):
        module = _fresh_import(legacy)
    assert module is not None


@pytest.mark.parametrize("legacy", sorted(SHIMS))
def test_legacy_path_warns_with_a_usable_message(legacy):
    canonical, _ = SHIMS[legacy]
    with pytest.warns(DeprecationWarning) as record:
        _fresh_import(legacy)
    message = str(record[0].message)
    assert legacy in message, "the warning should name the deprecated module"
    assert canonical.rsplit(".", 1)[0] in message or canonical in message, (
        "the warning should point at the replacement"
    )
    assert "0.3.0" in message, "the warning should state when it goes away"


@pytest.mark.parametrize("legacy", sorted(SHIMS))
def test_legacy_path_re_exports_the_same_object(legacy):
    """A shim must hand back the *same* object, not a copy of it."""
    canonical, symbol = SHIMS[legacy]
    with pytest.warns(DeprecationWarning):
        legacy_mod = _fresh_import(legacy)
    canonical_mod = importlib.import_module(canonical)
    assert getattr(legacy_mod, symbol) is getattr(canonical_mod, symbol)


def test_canonical_paths_do_not_warn(recwarn):
    """The replacements must be silent, or the advice is self-defeating."""
    for canonical, _ in SHIMS.values():
        sys.modules.pop(canonical, None)
        importlib.import_module(canonical)
    assert [w for w in recwarn if issubclass(w.category, DeprecationWarning)] == []


def test_package_itself_does_not_import_its_own_shims(recwarn):
    """Importing soarm_sdk must not trip its own deprecation warnings.

    If it does, some module inside the package is still importing through a
    legacy path and every downstream user would see the warning for code
    they do not control.
    """
    for name in [n for n in sys.modules if n.startswith("soarm_sdk")]:
        sys.modules.pop(name, None)
    importlib.import_module("soarm_sdk")
    offenders = [
        str(w.message)
        for w in recwarn
        if issubclass(w.category, DeprecationWarning) and "soarm_sdk" in str(w.message)
    ]
    assert offenders == [], f"soarm_sdk imports its own deprecated paths: {offenders}"
