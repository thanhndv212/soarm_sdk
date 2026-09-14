"""Acceptance and provenance rules for the guided calibration pipeline."""

from __future__ import annotations

import math

from soarm_sdk.calibration.frame import JointCalibration, RobotCalibration
from soarm_sdk.calibration.pipeline import (
    AcceptanceTolerances,
    CalibrationPipeline,
    PipelineStage,
)
from soarm_sdk.calibration.reference import FOLDED_FLAT


def _calibration(*, tolerances=None) -> RobotCalibration:
    return RobotCalibration(
        arm_id="test-arm",
        joints=[
            JointCalibration(
                name=name,
                zero_offset_ticks=2048.0,
                direction_sign=1,
                tick_min=500,
                tick_max=3500,
                zero_source="reference_pose"
                if name in FOLDED_FLAT.covers
                else "travel_and_urdf_limits",
            )
            for name in (
                "shoulder_pan",
                "shoulder_lift",
                "elbow_flex",
                "wrist_flex",
                "wrist_roll",
                "gripper",
            )
        ],
        notes={} if tolerances is None else {"acceptance_tolerances": tolerances},
    )


def test_pipeline_rejects_calibration_without_explicit_tolerances():
    report = CalibrationPipeline(_calibration()).report()

    assert report.stage(PipelineStage.ACCEPTANCE).passed is False
    assert "explicit acceptance tolerances" in report.stage(PipelineStage.ACCEPTANCE).detail
    assert report.ready is False


def test_pipeline_accepts_explicit_tolerances_and_pose_provenance():
    tolerances = AcceptanceTolerances(
        pose_repeatability_rad=math.radians(1.0),
        model_deviation_rad=math.radians(3.0),
        rom_endpoint_repeatability_ticks=10,
    )
    calibration = _calibration(tolerances=tolerances.to_dict())
    calibration.notes["rezeroed_from_dashboard"] = {
        "pose": FOLDED_FLAT.key,
        "joints": list(FOLDED_FLAT.covers),
        "ticks": {"shoulder_lift": 1200, "elbow_flex": 3200},
    }

    report = CalibrationPipeline(calibration).report()

    assert report.stage(PipelineStage.ACCEPTANCE).passed is True
    assert report.stage(PipelineStage.ZERO_PROVENANCE).passed is True


def test_pipeline_rejects_mislabeled_uncovered_reference_pose_joint():
    tolerances = AcceptanceTolerances(
        pose_repeatability_rad=math.radians(1.0),
        model_deviation_rad=math.radians(3.0),
        rom_endpoint_repeatability_ticks=10,
    )
    calibration = _calibration(tolerances=tolerances.to_dict())
    calibration.notes["rezeroed_from_dashboard"] = {
        "pose": FOLDED_FLAT.key,
        "joints": list(FOLDED_FLAT.covers),
        "ticks": {"shoulder_lift": 1200, "elbow_flex": 3200},
    }
    wrist_roll = next(j for j in calibration.joints if j.name == "wrist_roll")
    calibration.joints[calibration.names.index("wrist_roll")] = JointCalibration(
        **{**wrist_roll.__dict__, "zero_source": "reference_pose"}
    )

    report = CalibrationPipeline(calibration).report()

    stage = report.stage(PipelineStage.ZERO_PROVENANCE)
    assert stage.passed is False
    assert "wrist_roll" in stage.detail


def test_pipeline_rejects_reference_pose_measurements_above_repeatability_tolerance():
    tolerances = AcceptanceTolerances(
        pose_repeatability_rad=math.radians(0.25),
        model_deviation_rad=math.radians(3.0),
        rom_endpoint_repeatability_ticks=10,
    )
    calibration = _calibration(tolerances=tolerances.to_dict())

    report = CalibrationPipeline(calibration).record_reference_pose(
        FOLDED_FLAT,
        [
            {"shoulder_lift": 1200, "elbow_flex": 3200},
            {"shoulder_lift": 1220, "elbow_flex": 3200},
        ],
    )

    stage = report.stage(PipelineStage.REFERENCE_POSE)
    assert stage.passed is False
    assert "shoulder_lift" in stage.detail


def test_pipeline_reuses_persisted_reference_pose_evidence():
    tolerances = AcceptanceTolerances(
        pose_repeatability_rad=math.radians(1.0),
        model_deviation_rad=math.radians(3.0),
        rom_endpoint_repeatability_ticks=10,
    )
    calibration = _calibration(tolerances=tolerances.to_dict())
    calibration.notes["rezeroed_from_dashboard"] = {
        "pose": FOLDED_FLAT.key,
        "joints": list(FOLDED_FLAT.covers),
        "ticks": {"shoulder_lift": 1200, "elbow_flex": 3200},
    }
    calibration.notes["reference_pose_samples"] = [
        {"shoulder_lift": 1200, "elbow_flex": 3200},
        {"shoulder_lift": 1202, "elbow_flex": 3200},
    ]

    report = CalibrationPipeline(calibration).report()

    assert report.stage(PipelineStage.REFERENCE_POSE).passed is True


def test_non_finite_tolerances_are_rejected():
    import pytest

    with pytest.raises(ValueError, match="finite"):
        AcceptanceTolerances(float("nan"), math.radians(3.0), 10)
    with pytest.raises(ValueError, match="finite"):
        AcceptanceTolerances(math.radians(1.0), float("inf"), 10)
    with pytest.raises(ValueError, match="positive integer"):
        AcceptanceTolerances(math.radians(1.0), math.radians(3.0), 1.5)


def test_pipeline_requires_repeated_rom_endpoint_evidence():
    tolerances = AcceptanceTolerances(
        pose_repeatability_rad=math.radians(1.0),
        model_deviation_rad=math.radians(3.0),
        rom_endpoint_repeatability_ticks=10,
    )
    calibration = _calibration(tolerances=tolerances.to_dict())

    report = CalibrationPipeline(calibration).report()

    assert report.stage(PipelineStage.ROM).passed is False
    assert "repeated ROM endpoint" in report.stage(PipelineStage.ROM).detail
