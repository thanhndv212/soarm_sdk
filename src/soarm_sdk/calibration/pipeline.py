"""Fail-closed acceptance and provenance checks for arm calibration."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Mapping, Optional, Sequence

from ..conversions import RADS_PER_TICK
from .frame import RobotCalibration
from .reference import REFERENCE_POSES, ReferencePose

__all__ = [
    "AcceptanceTolerances",
    "CalibrationPipeline",
    "CalibrationReport",
    "PipelineStage",
    "StageResult",
]


class PipelineStage(str, Enum):
    ACCEPTANCE = "acceptance"
    DIRECTION_SIGNS = "direction_signs"
    ROM = "rom"
    ZERO_PROVENANCE = "zero_provenance"
    REFERENCE_POSE = "reference_pose"


@dataclass(frozen=True)
class AcceptanceTolerances:
    """Explicit, per-arm thresholds in the calibration's URDF frame."""

    pose_repeatability_rad: float
    model_deviation_rad: float
    rom_endpoint_repeatability_ticks: int

    def __post_init__(self) -> None:
        if not math.isfinite(self.pose_repeatability_rad) or self.pose_repeatability_rad <= 0:
            raise ValueError("pose_repeatability_rad must be finite and positive")
        if not math.isfinite(self.model_deviation_rad) or self.model_deviation_rad <= 0:
            raise ValueError("model_deviation_rad must be finite and positive")
        if (
            isinstance(self.rom_endpoint_repeatability_ticks, bool)
            or not isinstance(self.rom_endpoint_repeatability_ticks, int)
            or self.rom_endpoint_repeatability_ticks <= 0
        ):
            raise ValueError("rom_endpoint_repeatability_ticks must be a positive integer")

    def to_dict(self) -> Dict[str, float | int]:
        return {
            "pose_repeatability_rad": self.pose_repeatability_rad,
            "model_deviation_rad": self.model_deviation_rad,
            "rom_endpoint_repeatability_ticks": self.rom_endpoint_repeatability_ticks,
        }

    @classmethod
    def from_notes(cls, notes: Mapping[str, object]) -> Optional["AcceptanceTolerances"]:
        raw = notes.get("acceptance_tolerances")
        if not isinstance(raw, Mapping):
            return None
        try:
            endpoint_tolerance = raw["rom_endpoint_repeatability_ticks"]
            return cls(
                pose_repeatability_rad=float(raw["pose_repeatability_rad"]),
                model_deviation_rad=float(raw["model_deviation_rad"]),
                rom_endpoint_repeatability_ticks=endpoint_tolerance,
            )
        except (KeyError, TypeError, ValueError):
            return None


@dataclass(frozen=True)
class StageResult:
    stage: PipelineStage
    passed: bool
    detail: str


@dataclass(frozen=True)
class CalibrationReport:
    stages: Sequence[StageResult]

    @property
    def ready(self) -> bool:
        return all(stage.passed for stage in self.stages)

    def stage(self, stage: PipelineStage) -> StageResult:
        return next(result for result in self.stages if result.stage == stage)

    def as_markdown(self) -> str:
        lines = ["| Step | Status | Result |", "|---|---|---|"]
        # Numbered for the Calibration tab's folders, so a BLOCKED row names
        # the step to reopen rather than a stage number that appears nowhere
        # in the UI. Two rows share step 4 because one operator action --
        # pinning a pose -- is the evidence for both.
        labels = {
            PipelineStage.ACCEPTANCE: "Step 1 - Acceptance tolerances",
            PipelineStage.DIRECTION_SIGNS: "Step 2 - Direction signs",
            PipelineStage.ROM: "Step 3 - ROM limits",
            PipelineStage.ZERO_PROVENANCE: "Step 4 - Zero provenance",
            PipelineStage.REFERENCE_POSE: "Step 4 - Reference-pose stability",
        }
        for result in self.stages:
            status = "PASS" if result.passed else "BLOCKED"
            lines.append(f"| {labels[result.stage]} | {status} | {result.detail} |")
        return "\n".join(lines)


class CalibrationPipeline:
    """Evaluate calibration evidence without commanding the arm.

    The dashboard owns interaction and measurement. This class owns the
    reusable policy that decides whether the evidence is sufficient to use a
    calibration for safety-sensitive consumers.
    """

    def __init__(self, calibration: RobotCalibration) -> None:
        self.calibration = calibration
        self._reference_result: Optional[StageResult] = None

    def record_reference_pose(
        self,
        pose: ReferencePose,
        samples: Sequence[Mapping[str, int]],
    ) -> CalibrationReport:
        tolerances = AcceptanceTolerances.from_notes(self.calibration.notes)
        if tolerances is None:
            self._reference_result = StageResult(
                PipelineStage.REFERENCE_POSE,
                False,
                "cannot assess pose repeatability without explicit acceptance tolerances",
            )
            return self.report()
        if len(samples) < 2:
            self._reference_result = StageResult(
                PipelineStage.REFERENCE_POSE,
                False,
                "record at least two stable samples before pinning a reference pose",
            )
            return self.report()

        exceeded: List[str] = []
        for name in pose.covers:
            values = [sample.get(name) for sample in samples]
            if any(value is None for value in values):
                exceeded.append(f"{name} has a missing reading")
                continue
            spread_rad = (max(values) - min(values)) * RADS_PER_TICK
            if spread_rad > tolerances.pose_repeatability_rad:
                exceeded.append(
                    f"{name} spread {math.degrees(spread_rad):.2f} deg exceeds "
                    f"{math.degrees(tolerances.pose_repeatability_rad):.2f} deg"
                )

        self._reference_result = StageResult(
            PipelineStage.REFERENCE_POSE,
            not exceeded,
            "stable reference-pose samples accepted"
            if not exceeded
            else "; ".join(exceeded),
        )
        return self.report()

    def report(self) -> CalibrationReport:
        tolerances = AcceptanceTolerances.from_notes(self.calibration.notes)
        acceptance = StageResult(
            PipelineStage.ACCEPTANCE,
            tolerances is not None,
            "explicit per-arm acceptance tolerances recorded"
            if tolerances is not None
            else "explicit acceptance tolerances are required before calibration can be accepted",
        )
        signs = StageResult(
            PipelineStage.DIRECTION_SIGNS,
            self.calibration.validated,
            "physical direction-sign check recorded"
            if self.calibration.validated
            else "direction signs have not been physically verified",
        )
        rom = self._rom_result(tolerances)
        provenance = self._zero_provenance_result()
        reference = self._reference_result or self._reference_result_from_notes()
        return CalibrationReport((acceptance, signs, rom, provenance, reference))

    def _zero_provenance_result(self) -> StageResult:
        entry = self.calibration.notes.get("rezeroed_from_dashboard")
        if not isinstance(entry, Mapping):
            pose_anchored = [j.name for j in self.calibration.joints if j.zero_source == "reference_pose"]
            if pose_anchored:
                return StageResult(
                    PipelineStage.ZERO_PROVENANCE,
                    False,
                    "reference-pose zero(s) lack a recorded pose witness: "
                    + ", ".join(pose_anchored),
                )
            return StageResult(
                PipelineStage.ZERO_PROVENANCE,
                True,
                "no reference-pose zero claims require a pose witness",
            )
        covered = entry.get("joints")
        pose_key = entry.get("pose")
        pose = REFERENCE_POSES.get(pose_key) if isinstance(pose_key, str) else None
        if pose is None or not isinstance(covered, list) or not all(isinstance(name, str) for name in covered):
            return StageResult(
                PipelineStage.ZERO_PROVENANCE,
                False,
                "reference-pose provenance has no constrained-joint list",
            )
        expected = set(pose.covers)
        unsupported = [
            j.name
            for j in self.calibration.joints
            if j.zero_source == "reference_pose" and j.name not in expected
        ]
        problems = []
        if unsupported:
            # Name the joint and the pose that fails to constrain it. The old
            # wording -- "provenance incorrectly covers: gripper" -- read as
            # though the provenance covered the gripper, when it is the
            # gripper claiming a witness the provenance does not give it.
            problems.append(
                f"{', '.join(unsupported)} "
                f"{'claims' if len(unsupported) == 1 else 'claim'} a "
                f"reference-pose zero, but '{pose.key}' does not constrain "
                f"{'it' if len(unsupported) == 1 else 'them'} — withdraw the "
                "claim in Step 4"
            )
        if set(covered) != expected:
            problems.append("provenance joint list does not match " + pose.key)
        return StageResult(
            PipelineStage.ZERO_PROVENANCE,
            not problems,
            "reference-pose provenance matches constrained joints"
            if not problems
            else "; ".join(problems),
        )

    def _rom_result(self, tolerances: Optional[AcceptanceTolerances]) -> StageResult:
        samples = self.calibration.notes.get("rom_endpoint_samples")
        if tolerances is None or not isinstance(samples, Mapping):
            return StageResult(
                PipelineStage.ROM,
                False,
                "record repeated ROM endpoint samples before accepting travel limits",
            )
        if self.calibration.notes.get("dry_run"):
            return StageResult(
                PipelineStage.ROM,
                False,
                "simulated ROM data cannot be accepted for a physical arm",
            )
        failures = []
        for joint in self.calibration.joints:
            values = samples.get(joint.name)
            if not isinstance(values, list) or len(values) < 2:
                failures.append(f"{joint.name} has fewer than two endpoint samples")
                continue
            try:
                if any(sample.get("simulated") for sample in values if isinstance(sample, Mapping)):
                    failures.append(f"{joint.name} includes simulated endpoint samples")
                    continue
                endpoints = [(int(sample["min"]), int(sample["max"])) for sample in values]
            except (KeyError, TypeError, ValueError):
                failures.append(f"{joint.name} has malformed endpoint samples")
                continue
            for label, endpoint_values in (("min", [p[0] for p in endpoints]), ("max", [p[1] for p in endpoints])):
                if max(endpoint_values) - min(endpoint_values) > tolerances.rom_endpoint_repeatability_ticks:
                    failures.append(f"{joint.name} {label} endpoint is not repeatable")
        return StageResult(
            PipelineStage.ROM,
            not failures,
            "repeated ROM endpoint measurements accepted"
            if not failures
            else "; ".join(failures),
        )

    def _reference_result_from_notes(self) -> StageResult:
        entry = self.calibration.notes.get("rezeroed_from_dashboard")
        samples = self.calibration.notes.get("reference_pose_samples")
        if not isinstance(entry, Mapping) or not isinstance(samples, list):
            return StageResult(
                PipelineStage.REFERENCE_POSE,
                False,
                "record stable samples from a named reference pose",
            )
        pose_key = entry.get("pose")
        pose = REFERENCE_POSES.get(pose_key) if isinstance(pose_key, str) else None
        if pose is None or not all(isinstance(sample, Mapping) for sample in samples):
            return StageResult(
                PipelineStage.REFERENCE_POSE,
                False,
                "reference-pose measurement provenance is incomplete",
            )
        self.record_reference_pose(pose, samples)
        assert self._reference_result is not None
        return self._reference_result
