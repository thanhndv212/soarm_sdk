"""The enforced joint limits are in the URDF frame, and hardware wins.

Two separate guarantees are pinned here, because they failed independently:

1. The shipped configs declare limits in the **URDF's** kinematic frame.
   They previously did not: shoulder_lift was offset -pi/2 and elbow_flex
   +pi/2, while the other four joints matched. Nothing noticed until
   `ServoRobot` began enforcing them on every write, at which point a
   planner commanding URDF-frame angles had 55% of a trajectory clamped.

2. When a calibration is supplied, what is enforced is the **intersection**
   of the declared limits and the arm's measured travel. A config is a
   model's opinion; the measured range is where the mechanism actually
   stops, and a command must never be driven past that.
"""

from __future__ import annotations

import numpy as np
import pytest

from soarm_sdk import load_robot_config
from soarm_sdk.calibration.frame import JointCalibration, RobotCalibration
from soarm_sdk.robot.servo import ServoRobot

# SO-ARM100/Simulation/SO101/so101_new_calib.urdf, the authoritative source.
URDF_LIMITS = {
    "shoulder_pan": (-1.91986, 1.91986),
    "shoulder_lift": (-1.74533, 1.74533),
    "elbow_flex": (-1.69000, 1.69000),
    "wrist_flex": (-1.65806, 1.65806),
    "wrist_roll": (-2.74385, 2.84121),
    "gripper": (-0.17453, 1.74533),
}
JOINT_ORDER = list(URDF_LIMITS)


# ---------------------------------------------------------------------------
# 1. the shipped configs are in the URDF frame
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("config_name", ["soarm100", "so101"])
def test_shipped_config_limits_match_the_urdf(config_name):
    cfg = load_robot_config(config_name)
    lo = cfg["joint_limits_lower"]
    hi = cfg["joint_limits_upper"]
    for i, joint in enumerate(JOINT_ORDER):
        u_lo, u_hi = URDF_LIMITS[joint]
        assert lo[i] == pytest.approx(u_lo, abs=1e-4), f"{joint} lower"
        assert hi[i] == pytest.approx(u_hi, abs=1e-4), f"{joint} upper"


@pytest.mark.parametrize("config_name", ["soarm100", "so101"])
def test_home_position_is_inside_the_declared_limits(config_name):
    cfg = load_robot_config(config_name)
    home = np.array(cfg["home_position"])
    lo = np.array(cfg["joint_limits_lower"])
    hi = np.array(cfg["joint_limits_upper"])
    assert np.all(home >= lo) and np.all(home <= hi), (
        "go_home() would immediately be clamped"
    )


def test_the_two_shipped_configs_agree_on_limits():
    a, b = load_robot_config("soarm100"), load_robot_config("so101")
    assert a["joint_limits_lower"] == b["joint_limits_lower"]
    assert a["joint_limits_upper"] == b["joint_limits_upper"]
    assert a["joint_names"] == b["joint_names"]


# ---------------------------------------------------------------------------
# 2. hardware travel wins on the tight side
# ---------------------------------------------------------------------------


def _calibration(reach=None):
    """A calibration whose measured travel is *reach* (URDF rad) per joint."""
    reach = reach or {}
    joints = []
    for name in JOINT_ORDER:
        lo, hi = reach.get(name, URDF_LIMITS[name])
        # zero at tick 2048, +1 sign: ticks are symmetric about the zero.
        joints.append(
            JointCalibration(
                name=name,
                zero_offset_ticks=2048.0,
                direction_sign=1,
                tick_min=int(round(2048 + lo * 4096 / (2 * np.pi))),
                tick_max=int(round(2048 + hi * 4096 / (2 * np.pi))),
            )
        )
    return RobotCalibration(joints=joints, arm_id="test")


def test_reachable_rad_is_ordered_even_with_a_negative_sign():
    j = JointCalibration("j", zero_offset_ticks=2048.0, direction_sign=-1,
                         tick_min=1000, tick_max=3000)
    lo, hi = j.reachable_rad
    assert lo < hi, "a -1 sign swaps which tick endpoint is the larger angle"


def test_without_a_calibration_the_config_limits_are_enforced():
    robot = ServoRobot("/dev/null", calibration=None)
    lo, hi = robot.effective_joint_limits()
    cfg_lo, cfg_hi = robot.get_joint_limits()
    np.testing.assert_allclose(lo, cfg_lo)
    np.testing.assert_allclose(hi, cfg_hi)


def test_a_tighter_measured_travel_overrides_the_config():
    # This arm physically stops well short of the URDF on shoulder_lift.
    cal = _calibration({"shoulder_lift": (-1.0, 0.8)})
    robot = ServoRobot("/dev/null", calibration=cal)
    lo, hi = robot.effective_joint_limits()
    i = JOINT_ORDER.index("shoulder_lift")
    assert lo[i] == pytest.approx(-1.0, abs=2e-3)
    assert hi[i] == pytest.approx(0.8, abs=2e-3)


def test_a_wider_measured_travel_does_not_loosen_the_config():
    # Servo can reach further than the model allows — the model still wins.
    cal = _calibration({"elbow_flex": (-3.0, 3.0)})
    robot = ServoRobot("/dev/null", calibration=cal)
    lo, hi = robot.effective_joint_limits()
    i = JOINT_ORDER.index("elbow_flex")
    assert lo[i] == pytest.approx(URDF_LIMITS["elbow_flex"][0], abs=1e-4)
    assert hi[i] == pytest.approx(URDF_LIMITS["elbow_flex"][1], abs=1e-4)


def test_effective_limits_are_never_wider_than_either_source():
    cal = _calibration({"wrist_roll": (-2.0, 3.5), "gripper": (0.0, 1.0)})
    robot = ServoRobot("/dev/null", calibration=cal)
    lo, hi = robot.effective_joint_limits()
    cfg_lo, cfg_hi = robot.get_joint_limits()
    cal_lo, cal_hi = cal.reachable_limits()
    assert np.all(lo >= cfg_lo - 1e-9) and np.all(hi <= cfg_hi + 1e-9)
    assert np.all(lo >= np.array(cal_lo) - 1e-9)
    assert np.all(hi <= np.array(cal_hi) + 1e-9)


def test_a_calibration_for_a_different_arm_is_rejected():
    cal = _calibration()
    cal.joints = cal.joints[:4]  # 4-joint arm against a 6-joint config
    robot = ServoRobot("/dev/null", calibration=cal)
    with pytest.raises(ValueError, match="must describe the same arm"):
        robot.effective_joint_limits()
