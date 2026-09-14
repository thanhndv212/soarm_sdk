"""Which bound a consumer obeys: the model's opinion, or the mechanism.

A URDF's <limit> is conservative and, on this arm, wrong in places. The
measured travel is where the mechanism stops. Once the travel is trustworthy
it replaces the model's number; until then the two are intersected.
"""

from __future__ import annotations

import math

import pytest

from soarm_sdk.calibration.frame import JointCalibration, RobotCalibration
from soarm_sdk.calibration.limits import effective_limits, measured_is_trusted
from soarm_sdk.calibration.pipeline import AcceptanceTolerances

NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex",
         "wrist_flex", "wrist_roll", "gripper"]
DECLARED = [(-1.0, 1.0)] * 6


def _cal(tick_min=1024, tick_max=3072, accepted=False):
    joints = [
        JointCalibration(n, 2048.0, 1, tick_min, tick_max) for n in NAMES
    ]
    cal = RobotCalibration(joints=joints, arm_id="t", validated=True)
    if accepted:
        cal.notes["acceptance_tolerances"] = AcceptanceTolerances(
            pose_repeatability_rad=math.radians(0.5),
            model_deviation_rad=math.radians(2.0),
            rom_endpoint_repeatability_ticks=8,
        ).to_dict()
        cal.notes["rom_endpoint_samples"] = {
            n: [{"min": tick_min, "max": tick_max, "simulated": False},
                {"min": tick_min + 2, "max": tick_max - 2, "simulated": False}]
            for n in NAMES
        }
    return cal


def test_no_calibration_leaves_the_declared_bounds_alone():
    assert effective_limits(None, DECLARED) == DECLARED


def test_unaccepted_travel_is_intersected_not_trusted():
    """An unaccepted sweep can be the encoder's range, not the joint's.

    A wrapped wrist_roll measures a full turn; handing that to a planner as
    permission would be worse than staying conservative.
    """
    cal = _cal(tick_min=0, tick_max=4095, accepted=False)
    assert not measured_is_trusted(cal)
    assert effective_limits(cal, DECLARED) == DECLARED


def test_accepted_travel_replaces_the_models_opinion():
    """Intersecting keeps the conservative number, which is what clamped a
    planned trajectory on 55% of its waypoints."""
    cal = _cal(tick_min=0, tick_max=4095, accepted=True)
    assert measured_is_trusted(cal)

    out = effective_limits(cal, DECLARED)
    lo, hi = out[0]
    assert lo < -1.0 and hi > 1.0, "measured travel is wider and should win"
    expected = list(zip(*cal.reachable_limits()))
    assert out == [(float(a), float(b)) for a, b in expected]


def test_accepted_travel_narrower_than_the_model_also_wins():
    """Replacement, not 'whichever is wider' — the stop is the stop."""
    cal = _cal(tick_min=2000, tick_max=2100, accepted=True)
    for lo, hi in effective_limits(cal, DECLARED):
        assert -1.0 < lo < hi < 1.0


def test_the_gate_can_be_overridden_only_explicitly():
    cal = _cal(tick_min=0, tick_max=4095, accepted=False)
    assert effective_limits(cal, DECLARED) == DECLARED
    assert effective_limits(cal, DECLARED, trust_measured=True) != DECLARED


def test_a_calibration_for_a_different_arm_is_refused():
    with pytest.raises(ValueError):
        effective_limits(_cal(), DECLARED[:3])


def test_servo_robot_and_the_planner_share_one_policy():
    """Two copies of this rule would drift, and the drift would be silent."""
    import inspect

    from soarm_sdk.robot import servo

    assert "effective_limits" in inspect.getsource(servo)
