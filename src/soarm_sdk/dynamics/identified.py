"""Load identified dynamics and evaluate them at runtime.

The file is written by the identification side (FIGAROH's
``examples/so101/update_model.py``) and is deliberately small — only what
gravity compensation and friction feedforward need::

    format: soarm_sdk.dynamics.identified/v1
    joint_names: [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll]
    locked_joints: [gripper]          # welded into wrist_roll's body
    held_positions: {gripper: 0.0}    # where they were held, rad
    signal: current_mA                # what the fit measured torque through
    nm_per_unit: 0.001                # N·m per unit of that signal (1 N·m/A)
    bodies:                           # per joint: mass, first moment (link frame)
      shoulder_lift: {m: 0.21, mx: -0.019, my: -0.001, mz: 0.004}
    friction:                         # per joint: viscous, Coulomb
      shoulder_lift: {fv: 0.02, fs: 0.05}
    offset:                           # per joint: constant torque bias
      shoulder_lift: 0.003
    provenance: {...}

Kinematics are *not* in the file: they come from the nominal URDF, which is
not what identification changes.

Units, and why the torque scale does not have to be right
---------------------------------------------------------
The servos do not measure torque. The fit used one of their signals —
``current_mA`` or ``load_percent``, named by ``signal`` — scaled by
``nm_per_unit``, and every torque in the file is in N·m **as measured
through** that factor. If it is off by 20%, every identified mass is off by
the same 20% — but :meth:`IdentifiedDynamics.to_signal` divides by the same
factor on the way back, so the *predicted servo reading*, which is what a
controller compares against, is still right. The factor only matters when
the torques are used in physical units; refine it by identifying once with
a known payload.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Union

import numpy as np

from .gravity import GravityModel

try:
    import yaml
except ImportError:  # pragma: no cover - pyyaml is a hard dependency
    yaml = None  # type: ignore[assignment]

__all__ = ["FORMAT", "IdentifiedDynamics"]

FORMAT = "soarm_sdk.dynamics.identified/v1"


@dataclass
class IdentifiedDynamics:
    """Gravity, friction and offset per joint, from one identification run.

    All joint vectors are ordered like :attr:`joint_names`, in the URDF
    joint frame; torques are in N·m (see the module docstring on units).
    """

    joint_names: List[str]
    gravity: GravityModel
    fv: np.ndarray
    fs: np.ndarray
    offset: np.ndarray
    signal: str
    nm_per_unit: float
    meta: Dict[str, Any] = field(default_factory=dict)

    @classmethod
    def load(cls, path: Union[str, Path], urdf: Union[str, Path]) -> "IdentifiedDynamics":
        """Read *path*, taking kinematics and un-identified links from *urdf*."""
        if yaml is None:  # pragma: no cover
            raise ImportError("PyYAML is required to load identified dynamics")
        data = yaml.safe_load(Path(path).read_text())
        return cls.from_dict(data, urdf)

    @classmethod
    def from_dict(cls, data: Dict[str, Any], urdf: Union[str, Path]) -> "IdentifiedDynamics":
        if data.get("format") != FORMAT:
            raise ValueError(f"not a {FORMAT} file (format={data.get('format')!r})")
        names = list(data["joint_names"])
        model = GravityModel.from_urdf(
            urdf, names, held_positions=data.get("held_positions") or {}
        ).with_body_params(data.get("bodies") or {}, locked_joints=data.get("locked_joints") or ())

        def per_joint(section: Dict[str, Any], key: Optional[str]) -> np.ndarray:
            out = np.zeros(len(names))
            for i, n in enumerate(names):
                v = section.get(n)
                if isinstance(v, dict):
                    v = v.get(key) if key else None
                out[i] = float(v) if v is not None else 0.0
            return out

        friction = data.get("friction") or {}
        nm_per_unit = float(data["nm_per_unit"])
        if nm_per_unit <= 0:
            raise ValueError("nm_per_unit must be positive")
        meta = {
            k: v
            for k, v in data.items()
            if k
            not in ("format", "joint_names", "bodies", "friction", "offset", "signal", "nm_per_unit")
        }
        return cls(
            joint_names=names,
            gravity=model,
            fv=per_joint(friction, "fv"),
            fs=per_joint(friction, "fs"),
            offset=per_joint(data.get("offset") or {}, None),
            signal=str(data["signal"]),
            nm_per_unit=nm_per_unit,
            meta=meta,
        )

    # -- evaluation ------------------------------------------------------

    def select(self, q: Sequence[float], names: Sequence[str]) -> np.ndarray:
        """Pick this model's joints out of a vector ordered like *names*.

        For passing a full robot state (e.g. all six SO-101 servos, gripper
        included) to a model of the five arm joints.
        """
        idx = {n: i for i, n in enumerate(names)}
        missing = [n for n in self.joint_names if n not in idx]
        if missing:
            raise ValueError(f"joints {missing} are not in {list(names)}")
        q = np.asarray(q, dtype=float)
        return q[[idx[n] for n in self.joint_names]]

    def gravity_torque(self, q: Sequence[float]) -> np.ndarray:
        """``g(q)``: the torque needed to hold the arm still at *q*."""
        return self.gravity.torque(q)

    def friction_torque(self, dq: Sequence[float], *, deadband_rad_s: float = 0.0) -> np.ndarray:
        """Viscous plus Coulomb friction at joint speed *dq*.

        The Coulomb term flips with the sign of *dq*, which is undefined at
        rest; ``deadband_rad_s`` replaces the sign with a linear ramp across
        ``[-deadband, deadband]`` so a feedforward does not chatter there.
        """
        dq = np.asarray(dq, dtype=float)
        if deadband_rad_s > 0:
            s = np.clip(dq / deadband_rad_s, -1.0, 1.0)
        else:
            s = np.sign(dq)
        return self.fv * dq + self.fs * s

    def torque(
        self,
        q: Sequence[float],
        dq: Optional[Sequence[float]] = None,
        *,
        deadband_rad_s: float = 0.0,
    ) -> np.ndarray:
        """Everything the fit modelled: gravity, offset, and (with *dq*) friction."""
        tau = self.gravity_torque(q) + self.offset
        if dq is not None:
            tau = tau + self.friction_torque(dq, deadband_rad_s=deadband_rad_s)
        return tau

    def to_signal(self, tau: Sequence[float]) -> np.ndarray:
        """Joint torque -> the servo reading named by :attr:`signal`.

        URDF-frame sign: multiply by the calibration's direction sign before
        comparing against a raw servo register (see the units note).
        """
        return np.asarray(tau, dtype=float) / self.nm_per_unit

    def from_signal(self, value: Sequence[float]) -> np.ndarray:
        """The servo reading named by :attr:`signal`, URDF-frame sign -> torque."""
        return np.asarray(value, dtype=float) * self.nm_per_unit
