"""Tests for soarm_sdk.dynamics — excitation, logging, recording, gravity.

No hardware and no workspace files: URDFs are synthesized inline, since this
package's CI runs without the SO-ARM100 checkout next to it.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import List, Optional

import numpy as np
import pytest

from soarm_sdk.dynamics import (
    ExcitationAborted,
    ExcitationSpec,
    GravityModel,
    IdentificationLog,
    IdentifiedDynamics,
    fourier_excitation,
    record_excitation,
)
from soarm_sdk.dynamics import cli
from soarm_sdk.dynamics.identified import FORMAT as IDENTIFIED_FORMAT
from soarm_sdk.robot.null import NullRobot
from soarm_sdk.robot.telemetry import ServoSample, TelemetryStream
from soarm_sdk.robot.types import JointState

G = 9.81

SO101_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


def _chain_urdf(
    names: List[str],
    *,
    link_len: float = 0.1,
    masses: Optional[List[float]] = None,
    axes: Optional[List[str]] = None,
    tail_fixed_mass: float = 0.0,
) -> str:
    """A serial chain: joint i at the end of link i-1, COM mid-link along x."""
    masses = masses or [0.1] * len(names)
    axes = axes or ["0 1 0"] * len(names)
    parts = ['<robot name="chain">', '<link name="base"/>']
    parent = "base"
    for i, (n, m, ax) in enumerate(zip(names, masses, axes)):
        child = f"link{i}"
        xyz = "0 0 0" if i == 0 else f"{link_len} 0 0"
        parts.append(
            f'<link name="{child}"><inertial><origin xyz="{link_len / 2} 0 0" rpy="0 0 0"/>'
            f'<mass value="{m}"/><inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" '
            f'izz="1e-5"/></inertial></link>'
        )
        parts.append(
            f'<joint name="{n}" type="revolute"><origin xyz="{xyz}" rpy="0 0 0"/>'
            f'<parent link="{parent}"/><child link="{child}"/><axis xyz="{ax}"/>'
            f'<limit lower="-3" upper="3" effort="1" velocity="1"/></joint>'
        )
        parent = child
    if tail_fixed_mass:
        parts.append(
            f'<link name="tool"><inertial><origin xyz="0 0 0"/><mass value="{tail_fixed_mass}"/>'
            f'<inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/></inertial></link>'
        )
        parts.append(
            f'<joint name="tool_joint" type="fixed"><origin xyz="{link_len} 0 0" rpy="0 0 0"/>'
            f'<parent link="{parent}"/><child link="tool"/></joint>'
        )
    parts.append("</robot>")
    return "\n".join(parts)


# -- gravity -----------------------------------------------------------------


def test_single_pendulum_matches_closed_form():
    # Link along +x, rotating about +y: R_y(q) maps x to (cos q, 0, -sin q),
    # so the COM height is -(l/2) sin q and g(q) = dV/dq = -m g (l/2) cos q.
    model = GravityModel.from_urdf(_chain_urdf(["j"], masses=[2.0]), ["j"])
    for q in (0.0, 0.4, -1.1, np.pi / 2):
        np.testing.assert_allclose(model.torque([q]), [-2.0 * G * 0.05 * np.cos(q)], atol=1e-12)


def test_two_link_chain_sums_distal_mass():
    names = ["a", "b"]
    model = GravityModel.from_urdf(_chain_urdf(names, masses=[1.0, 0.5]), names)
    tau = model.torque([0.0, 0.0])
    # Joint a carries link0 (COM 0.05) and link1 (COM 0.15); b carries link1 (0.05).
    np.testing.assert_allclose(tau, [-G * (1.0 * 0.05 + 0.5 * 0.15), -G * 0.5 * 0.05], atol=1e-12)


def test_vertical_axis_carries_no_gravity():
    model = GravityModel.from_urdf(_chain_urdf(["pan"], axes=["0 0 1"]), ["pan"])
    np.testing.assert_allclose(model.torque([0.7]), [0.0], atol=1e-12)


def test_unlisted_joint_is_held_and_still_carries_mass():
    names = ["a", "b"]
    urdf = _chain_urdf(names, masses=[1.0, 0.5])
    only_a = GravityModel.from_urdf(urdf, ["a"], held_positions={"b": 0.3})
    both = GravityModel.from_urdf(urdf, names)
    np.testing.assert_allclose(only_a.torque([0.2]), both.torque([0.2, 0.3])[:1], atol=1e-12)


def test_rejects_prismatic_joints():
    urdf = _chain_urdf(["a"]).replace('type="revolute"', 'type="prismatic"')
    with pytest.raises(ValueError, match="prismatic"):
        GravityModel.from_urdf(urdf, [])


def test_body_params_replace_the_welded_links():
    names = ["a", "b"]
    urdf = _chain_urdf(names, masses=[1.0, 0.5], tail_fixed_mass=0.2)
    nominal = GravityModel.from_urdf(urdf, names)
    # b's body = link1 (0.5 kg at 0.05) + tool (0.2 kg at 0.1), lumped:
    lumped = {"b": {"m": 0.7, "mx": 0.5 * 0.05 + 0.2 * 0.1, "my": 0.0, "mz": 0.0}}
    identified = nominal.with_body_params(lumped)
    np.testing.assert_allclose(identified.torque([0.3, -0.4]), nominal.torque([0.3, -0.4]), atol=1e-12)
    assert identified.links["tool"].mass == 0.0


def test_matches_pinocchio_when_available():
    pin = pytest.importorskip("pinocchio")
    names = ["a", "b", "c"]
    urdf = _chain_urdf(names, masses=[0.3, 0.2, 0.1], axes=["0 0 1", "0 1 0", "1 0 0"])
    model = GravityModel.from_urdf(urdf, names)
    pm = pin.buildModelFromXML(urdf)
    pd = pm.createData()
    rng = np.random.default_rng(1)
    for _ in range(5):
        q = rng.uniform(-2, 2, 3)
        np.testing.assert_allclose(model.torque(q), pin.computeGeneralizedGravity(pm, pd, q), atol=1e-10)


def test_lowest_point_tracks_the_chain():
    model = GravityModel.from_urdf(_chain_urdf(["a", "b"]), ["a", "b"])
    name, z = model.lowest_point([np.pi / 2, 0.0])  # arm pointing straight down
    assert name == "link1"
    assert z == pytest.approx(-0.1)


# -- excitation --------------------------------------------------------------


LO = np.array([-1.0, -2.0, -0.5])
HI = np.array([1.0, 0.5, 2.0])


def test_excitation_stays_inside_the_margin():
    spec = ExcitationSpec(duration_s=40, margin_rad=0.2, amplitude_scale=1.0, max_vel_rad_s=5.0)
    _, q = fourier_excitation(LO, HI, spec)
    assert np.all(q >= LO + 0.2 - 1e-12) and np.all(q <= HI - 0.2 + 1e-12)


def test_excitation_starts_and_ends_at_rest_at_the_center():
    spec = ExcitationSpec(duration_s=40)
    center = np.array([0.1, -0.5, 0.4])
    t, q = fourier_excitation(LO, HI, spec, center=center)
    np.testing.assert_allclose(q[0], center, atol=1e-12)
    np.testing.assert_allclose(q[-1], center, atol=1e-9)
    dq = np.gradient(q, t, axis=0)
    assert np.max(np.abs(dq[0])) < 1e-3 and np.max(np.abs(dq[-1])) < 1e-3


def test_excitation_respects_peak_velocity():
    spec = ExcitationSpec(duration_s=40, max_vel_rad_s=0.2, amplitude_scale=1.0)
    t, q = fourier_excitation(LO, HI, spec)
    assert np.max(np.abs(np.gradient(q, t, axis=0))) <= 0.2 + 1e-9


def test_inactive_joints_hold_the_center():
    t, q = fourier_excitation(LO, HI, ExcitationSpec(duration_s=40), active=[True, False, True])
    assert np.ptp(q[:, 1]) == 0.0
    assert np.ptp(q[:, 0]) > 0.1


def test_center_outside_the_range_is_refused():
    with pytest.raises(ValueError, match="outside the usable range"):
        fourier_excitation(LO, HI, ExcitationSpec(duration_s=40), center=[0.95, 0.0, 0.0])


def test_excitation_is_reproducible_per_seed():
    a = fourier_excitation(LO, HI, ExcitationSpec(duration_s=40, seed=3))[1]
    b = fourier_excitation(LO, HI, ExcitationSpec(duration_s=40, seed=3))[1]
    c = fourier_excitation(LO, HI, ExcitationSpec(duration_s=40, seed=4))[1]
    np.testing.assert_array_equal(a, b)
    assert not np.allclose(a, c)


# -- log ---------------------------------------------------------------------


def _sample(k: int, t: float, *, pos, vel, cur, load, goal=None) -> ServoSample:
    n = len(pos)
    return ServoSample(
        t_mono=t,
        seq=k,
        ids=tuple(range(1, n + 1)),
        position_ticks=(0,) * n,
        position_rad=tuple(pos),
        velocity_ticks=(0,) * n,
        velocity_rad_s=tuple(vel),
        load_percent=tuple(load),
        current_mA=tuple(cur),
        voltage_V=(12.0,) * n,
        temperature_C=(30,) * n,
        status_flags=(0,) * n,
        moving=(False,) * n,
        goal_position_rad=None if goal is None else tuple(goal),
    )


def test_from_samples_moves_motor_frame_signals_into_the_urdf_frame():
    samples = [
        _sample(k, 100.0 + 0.02 * k, pos=[0.1, 0.2], vel=[1.0, 1.0], cur=[50.0, 50.0], load=[3.0, 3.0])
        for k in range(3)
    ]
    log = IdentificationLog.from_samples(samples, ["a", "b"], [1, -1])
    # Positions are already joint-frame; velocity/current/load are not.
    np.testing.assert_allclose(log.q[0], [0.1, 0.2])
    np.testing.assert_allclose(log.dq[0], [1.0, -1.0])
    np.testing.assert_allclose(log.current_mA[0], [50.0, -50.0])
    np.testing.assert_allclose(log.load_percent[0], [3.0, -3.0])
    np.testing.assert_allclose(log.t, [0.0, 0.02, 0.04])
    assert log.q_cmd is None
    assert log.meta["missed_ticks"] == 0


def test_from_samples_counts_missed_bus_ticks():
    samples = [_sample(k, 0.01 * k, pos=[0], vel=[0], cur=[0], load=[0]) for k in (1, 2, 5)]
    assert IdentificationLog.from_samples(samples, ["a"], [1]).meta["missed_ticks"] == 2


def test_resampled_is_uniform_and_records_the_largest_gap():
    t = np.array([0.0, 0.02, 0.05, 0.06, 0.1])
    q = np.column_stack([t * 10])
    log = IdentificationLog(["a"], t, q, q * 0, q * 0, q * 0)
    r = log.resampled(50.0)
    np.testing.assert_allclose(np.diff(r.t), 0.02)
    np.testing.assert_allclose(r.q[:, 0], r.t * 10)
    assert r.meta["max_source_gap_s"] == pytest.approx(0.04)


def test_save_load_round_trip(tmp_path: Path):
    t = np.linspace(0, 1, 11)
    q = np.column_stack([np.sin(t), np.cos(t)])
    log = IdentificationLog(["a", "b"], t, q, q * 2, q * 3, q * 4, q_cmd=q * 5, meta={"arm_id": "x"})
    back = IdentificationLog.load(log.save(tmp_path / "run"))
    assert back.joint_names == ["a", "b"]
    for name in ("t", "q", "dq", "current_mA", "load_percent", "q_cmd"):
        np.testing.assert_allclose(getattr(back, name), getattr(log, name), atol=1e-8)
    assert back.meta["arm_id"] == "x"
    header = (tmp_path / "run" / "q.csv").read_text().splitlines()[0]
    assert header == "t,a,b"


# -- record ------------------------------------------------------------------


def _config(n: int = 2):
    return {
        "n_dof": n,
        "joint_names": [f"j{i}" for i in range(n)],
        "home_position": [0.0] * n,
        "joint_limits_lower": [-1.0] * n,
        "joint_limits_upper": [1.0] * n,
    }


class _Clock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def sleep(self, s: float) -> None:
        self.now += max(0.0, s)


def _plan(n: int = 2, N: int = 50) -> np.ndarray:
    s = np.sin(np.linspace(0, np.pi, N))
    return np.column_stack([0.3 * s * (i + 1) for i in range(n)])


def test_speed_hint_never_caps_a_servo_below_the_floor():
    """ServoRobot turns |dq| into GOAL_SPEED: it must not approach zero."""
    from soarm_sdk.dynamics.record import MIN_SPEED_RAD_S, SPEED_HEADROOM

    hints = []

    class _Spy(NullRobot):
        def set_joint_positions(self, positions, dq=None):
            hints.append(np.asarray(dq))
            super().set_joint_positions(positions, dq)

    clock = _Clock()
    plan = _plan()
    record_excitation(
        _Spy(_config()), plan, 50.0, joint_names=["j0", "j1"], direction_signs=[1, 1],
        clock=clock, sleep=clock.sleep,
    )
    hints = np.array(hints)
    assert hints.min() >= MIN_SPEED_RAD_S
    peak = np.abs(np.gradient(plan, 0.02, axis=0)).max()
    assert hints.max() == pytest.approx(max(peak * SPEED_HEADROOM, MIN_SPEED_RAD_S))


def test_record_on_a_polled_robot_follows_the_plan():
    robot = NullRobot(_config())
    robot.set_joint_positions(np.array([0.5, -0.5]))  # off the start pose
    clock = _Clock()
    plan = _plan()
    log = record_excitation(
        robot, plan, 50.0, joint_names=["j0", "j1"], direction_signs=[1, 1],
        clock=clock, sleep=clock.sleep,
    )
    assert log.meta["source"] == "polled"
    assert log.q.shape == plan.shape
    np.testing.assert_allclose(log.q, plan)  # a NullRobot is exactly where it was told
    np.testing.assert_allclose(log.q_cmd, plan)
    np.testing.assert_allclose(log.current_mA, 0.0)


class _LaggyRobot(NullRobot):
    """Stops following after a few commands, like an arm hitting something."""

    def __init__(self, stick_after: int) -> None:
        super().__init__(_config())
        self._n = 0
        self._stick_after = stick_after

    def set_joint_positions(self, positions, dq=None):
        self._n += 1
        if self._n <= self._stick_after:
            super().set_joint_positions(positions, dq)


def test_tracking_error_aborts_and_keeps_the_partial_log():
    plan = np.column_stack([np.linspace(0, 0.9, 60), np.zeros(60)])
    robot = _LaggyRobot(stick_after=20)  # 2 approach commands + 18 of the plan
    clock = _Clock()
    with pytest.raises(ExcitationAborted, match="j0 lagging") as exc:
        record_excitation(
            robot, plan, 50.0, joint_names=["j0", "j1"], direction_signs=[1, 1],
            abort_tracking_rad=0.2, clock=clock, sleep=clock.sleep,
        )
    assert exc.value.log is not None
    assert 0 < exc.value.log.t.size < 60
    assert "aborted" in exc.value.log.meta


class _FakeHw:
    """A telemetry tap that publishes one sample per command, like the bus thread."""

    def __init__(self, robot: "_TelemetryRobot") -> None:
        self._robot = robot
        self.stream: Optional[TelemetryStream] = None

    def subscribe(self) -> TelemetryStream:
        self.stream = TelemetryStream()
        return self.stream

    def unsubscribe(self, stream: TelemetryStream) -> None:
        stream.close()
        self.stream = None


class _TelemetryRobot(NullRobot):
    def __init__(self, current_mA: float) -> None:
        super().__init__(_config())
        self.hw = _FakeHw(self)
        self._k = 0
        self._current = current_mA

    def set_joint_positions(self, positions, dq=None):
        super().set_joint_positions(positions, dq)
        if self.hw.stream is not None:
            self._k += 1
            q = self.get_joint_positions()
            self.hw.stream._publish(
                _sample(self._k, 0.02 * self._k, pos=q, vel=[0.1, 0.1],
                        cur=[self._current, self._current], load=[1.0, 1.0], goal=q)
            )

    def get_joint_state(self):  # pragma: no cover - must not be used on this path
        raise AssertionError("telemetry path should not poll")


def test_record_uses_the_telemetry_tap_and_fixes_signs():
    robot = _TelemetryRobot(current_mA=100.0)
    clock = _Clock()
    log = record_excitation(
        robot, _plan(), 50.0, joint_names=["j0", "j1"], direction_signs=[1, -1],
        clock=clock, sleep=clock.sleep,
    )
    assert log.meta["source"] == "telemetry"
    assert log.t.size == 50
    np.testing.assert_allclose(log.current_mA[0], [100.0, -100.0])
    np.testing.assert_allclose(log.dq[0], [0.1, -0.1])


def test_overcurrent_aborts():
    robot = _TelemetryRobot(current_mA=2000.0)
    clock = _Clock()
    with pytest.raises(ExcitationAborted, match="drawing 2000 mA"):
        record_excitation(
            robot, _plan(), 50.0, joint_names=["j0", "j1"], direction_signs=[1, 1],
            abort_current_mA=1500.0, clock=clock, sleep=clock.sleep,
        )


def test_polled_efforts_are_sign_corrected():
    class _Effort(NullRobot):
        def get_joint_state(self):
            s = super().get_joint_state()
            return JointState(positions=s.positions, velocities=np.array([0.2, 0.2]),
                              efforts=np.array([30.0, 30.0]))

    robot = _Effort(_config())
    clock = _Clock()
    log = record_excitation(
        robot, _plan(), 50.0, joint_names=["j0", "j1"], direction_signs=[1, -1],
        clock=clock, sleep=clock.sleep,
    )
    np.testing.assert_allclose(log.current_mA[0], [30.0, -30.0])
    np.testing.assert_allclose(log.dq[0], [0.2, -0.2])


# -- identified dynamics -----------------------------------------------------


def _identified(names, **over):
    data = {
        "format": IDENTIFIED_FORMAT,
        "joint_names": names,
        "signal": "current_mA",
        "nm_per_unit": 0.0005,
        "bodies": {},
        "friction": {n: {"fv": 0.1, "fs": 0.02} for n in names},
        "offset": {n: 0.01 for n in names},
    }
    data.update(over)
    return data


def test_identified_without_bodies_is_the_nominal_urdf_plus_offsets():
    names = ["a", "b"]
    urdf = _chain_urdf(names, masses=[1.0, 0.5])
    dyn = IdentifiedDynamics.from_dict(_identified(names), urdf)
    q = np.array([0.3, -0.2])
    nominal = GravityModel.from_urdf(urdf, names).torque(q)
    np.testing.assert_allclose(dyn.torque(q), nominal + 0.01)
    np.testing.assert_allclose(
        dyn.torque(q, [0.5, -0.5]), nominal + 0.01 + [0.05 + 0.02, -0.05 - 0.02]
    )


def test_current_conversion_round_trips():
    dyn = IdentifiedDynamics.from_dict(_identified(["a"]), _chain_urdf(["a"]))
    assert dyn.signal == "current_mA"
    np.testing.assert_allclose(dyn.to_signal([0.25]), [500.0])
    np.testing.assert_allclose(dyn.from_signal(dyn.to_signal([0.3, -0.1])), [0.3, -0.1])


def test_friction_deadband_ramps_instead_of_switching():
    dyn = IdentifiedDynamics.from_dict(_identified(["a"]), _chain_urdf(["a"]))
    assert dyn.friction_torque([0.0], deadband_rad_s=0.1)[0] == 0.0
    assert dyn.friction_torque([0.05], deadband_rad_s=0.1)[0] == pytest.approx(0.1 * 0.05 + 0.01)
    assert dyn.friction_torque([1e-6])[0] == pytest.approx(0.02, abs=1e-6)


def test_identified_bodies_and_locked_joints(tmp_path: Path):
    names = ["a", "b", "jaw"]
    urdf = _chain_urdf(names, masses=[1.0, 0.5, 0.1])
    # b's body with the jaw locked at 0: link1 (0.5 @ 0.05) + link2 (0.1 @ 0.15).
    body = {"m": 0.6, "mx": 0.5 * 0.05 + 0.1 * 0.15, "my": 0.0, "mz": 0.0}
    data = _identified(["a", "b"], bodies={"b": body}, locked_joints=["jaw"],
                       held_positions={"jaw": 0.0}, offset={}, friction={})
    p = tmp_path / "dyn.yaml"
    import yaml

    p.write_text(yaml.safe_dump(data))
    dyn = IdentifiedDynamics.load(p, urdf)
    q = np.array([0.4, -0.3])
    full = GravityModel.from_urdf(urdf, names).torque([0.4, -0.3, 0.0])[:2]
    np.testing.assert_allclose(dyn.torque(q), full, atol=1e-12)
    np.testing.assert_allclose(dyn.select([9, 0.4, 7, -0.3], ["x", "a", "y", "b"]), q)


def test_identified_rejects_other_formats():
    with pytest.raises(ValueError, match="not a"):
        IdentifiedDynamics.from_dict({"format": "nope"}, _chain_urdf(["a"]))


# -- CLI ---------------------------------------------------------------------


def _so101_urdf(tmp_path: Path) -> Path:
    # Pan about z, then pitch joints about y: the SO-101's layout, roughly.
    urdf = _chain_urdf(SO101_NAMES, axes=["0 0 1", "0 1 0", "0 1 0", "0 1 0", "1 0 0", "0 1 0"])
    urdf = urdf.replace('<joint name="shoulder_pan" type="revolute"><origin xyz="0 0 0"',
                        '<joint name="shoulder_pan" type="revolute"><origin xyz="0 0 0.3"')
    p = tmp_path / "so101.urdf"
    p.write_text(urdf)
    return p


def test_cli_dry_run_writes_a_loadable_log(tmp_path: Path, capsys):
    out = tmp_path / "run"
    rc = cli.main([
        "--dry-run", "--duration", "20", "--out", str(out),
        "--calibration", str(tmp_path / "none.json"), "--urdf", str(_so101_urdf(tmp_path)),
    ])
    assert rc == 0
    log = IdentificationLog.load(out)
    assert log.joint_names == SO101_NAMES
    assert log.meta["simulated"] is True
    assert log.meta["active_joints"] == SO101_NAMES[:5]
    assert np.ptp(log.q[:, 5]) == 0.0  # gripper held
    meta = json.loads((out / "meta.json").read_text())
    assert meta["frame"] == "urdf"


def test_cli_refuses_a_plan_that_goes_too_low(tmp_path: Path, capsys):
    rc = cli.main([
        "--dry-run", "--duration", "20", "--out", str(tmp_path / "run"),
        "--calibration", str(tmp_path / "none.json"), "--urdf", str(_so101_urdf(tmp_path)),
        "--min-height", "0.5",
    ])
    assert rc == 2
    assert "below --min-height" in capsys.readouterr().err
    assert not (tmp_path / "run").exists()


def test_cli_refuses_an_unvalidated_calibration_on_hardware(tmp_path: Path, capsys):
    from soarm_sdk.calibration.frame import JointCalibration, RobotCalibration

    cal = RobotCalibration(
        joints=[JointCalibration(name=n, zero_offset_ticks=2048, direction_sign=1,
                                 tick_min=0, tick_max=4095)
                for n in SO101_NAMES],
    )
    path = cal.save(tmp_path / "cal.json")
    rc = cli.main(["--calibration", str(path), "--urdf", str(_so101_urdf(tmp_path))])
    assert rc == 2
    assert "not validated" in capsys.readouterr().err
