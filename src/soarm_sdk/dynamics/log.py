"""What an identification run records, and the on-disk format for it.

The format is the contract with the identification side (FIGAROH's
``examples/so101/``), which runs in a different environment and never
imports this package: a directory of plain CSVs plus a ``meta.json``.

::

    <run>/
      meta.json          joint names, units, sample rate, arm, calibration
      q.csv              t, <joint>...    rad, URDF joint frame
      dq.csv             t, <joint>...    rad/s, URDF joint frame
      current_mA.csv     t, <joint>...    mA, signed in the URDF joint frame
      load_percent.csv   t, <joint>...    % PWM duty, signed in the URDF joint frame
      q_cmd.csv          t, <joint>...    rad, the command in force (if any)

Signs
-----
A servo reports speed, load and current in its **motor** direction. Joint
angles are already mapped into the URDF frame by the calibration, but these
three are not — so on a joint whose direction sign is ``-1`` (``wrist_roll``
on this arm) the raw current opposes the URDF torque. :meth:`IdentificationLog.
from_samples` multiplies all three by the direction sign so every column in
the files above is in one frame.
"""

from __future__ import annotations

import csv
import json
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Union

import numpy as np

from ..robot.telemetry import ServoSample

__all__ = ["IdentificationLog", "FORMAT"]

FORMAT = "soarm_sdk.dynamics.log/v1"

_SIGNALS = ("q", "dq", "current_mA", "load_percent", "q_cmd")


@dataclass
class IdentificationLog:
    """Time series from one excitation run, all in the URDF joint frame."""

    joint_names: List[str]
    t: np.ndarray  # (N,) seconds from the first sample
    q: np.ndarray  # (N, n) rad
    dq: np.ndarray  # (N, n) rad/s
    current_mA: np.ndarray  # (N, n)
    load_percent: np.ndarray  # (N, n)
    q_cmd: Optional[np.ndarray] = None  # (N, n) rad
    meta: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        n = len(self.joint_names)
        N = self.t.shape[0]
        for name in _SIGNALS:
            arr = getattr(self, name)
            if arr is None:
                continue
            if arr.shape != (N, n):
                raise ValueError(f"{name} has shape {arr.shape}, expected {(N, n)}")

    # -- construction ----------------------------------------------------

    @classmethod
    def from_samples(
        cls,
        samples: Sequence[ServoSample],
        joint_names: Sequence[str],
        direction_signs: Sequence[int],
        *,
        meta: Optional[Dict[str, Any]] = None,
    ) -> "IdentificationLog":
        """Build a log from bus-thread telemetry, fixing motor-frame signs."""
        if not samples:
            raise ValueError("no samples recorded")
        sign = np.asarray(direction_signs, dtype=float)
        if sign.shape != (len(joint_names),):
            raise ValueError("one direction sign per joint is required")
        t0 = samples[0].t_mono
        t = np.array([s.t_mono - t0 for s in samples])
        q = np.array([s.position_rad for s in samples], dtype=float)
        dq = np.array([s.velocity_rad_s for s in samples], dtype=float) * sign
        cur = np.array([s.current_mA for s in samples], dtype=float) * sign
        load = np.array([s.load_percent for s in samples], dtype=float) * sign
        q_cmd = None
        if all(s.goal_position_rad is not None for s in samples):
            q_cmd = np.array([s.goal_position_rad for s in samples], dtype=float)
        seqs = np.array([s.seq for s in samples])
        info = dict(meta or {})
        info.setdefault("missed_ticks", int(np.sum(np.diff(seqs) - 1)) if seqs.size > 1 else 0)
        return cls(
            joint_names=list(joint_names),
            t=t,
            q=q,
            dq=dq,
            current_mA=cur,
            load_percent=load,
            q_cmd=q_cmd,
            meta=info,
        )

    # -- processing ------------------------------------------------------

    @property
    def rate_hz(self) -> float:
        """Mean sample rate."""
        if self.t.size < 2:
            return 0.0
        return float((self.t.size - 1) / (self.t[-1] - self.t[0]))

    def resampled(self, rate_hz: float) -> "IdentificationLog":
        """Linearly interpolate every signal onto a uniform grid.

        Identification differentiates positions and assumes a fixed sample
        time, but bus reads jitter and occasionally fail. The largest gap in
        the source is recorded in ``meta`` so a badly broken log is visible
        rather than silently smoothed over.
        """
        if rate_hz <= 0:
            raise ValueError("rate_hz must be positive")
        grid = np.arange(0.0, self.t[-1] + 0.5 / rate_hz, 1.0 / rate_hz)
        grid = grid[grid <= self.t[-1]]

        def interp(arr: Optional[np.ndarray]) -> Optional[np.ndarray]:
            if arr is None:
                return None
            return np.column_stack([np.interp(grid, self.t, arr[:, j]) for j in range(arr.shape[1])])

        meta = dict(self.meta)
        meta["resampled_hz"] = float(rate_hz)
        meta["max_source_gap_s"] = float(np.max(np.diff(self.t))) if self.t.size > 1 else 0.0
        return replace(
            self,
            t=grid,
            q=interp(self.q),
            dq=interp(self.dq),
            current_mA=interp(self.current_mA),
            load_percent=interp(self.load_percent),
            q_cmd=interp(self.q_cmd),
            meta=meta,
        )

    # -- persistence -----------------------------------------------------

    def save(self, out_dir: Union[str, Path]) -> Path:
        out = Path(out_dir)
        out.mkdir(parents=True, exist_ok=True)
        header = ["t", *self.joint_names]
        for name in _SIGNALS:
            arr = getattr(self, name)
            if arr is None:
                continue
            with open(out / f"{name}.csv", "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(header)
                for ti, row in zip(self.t, arr):
                    w.writerow([f"{ti:.6f}", *(f"{v:.9g}" for v in row)])
        meta = {
            "format": FORMAT,
            "joint_names": self.joint_names,
            "n_samples": int(self.t.size),
            "rate_hz": self.rate_hz,
            "frame": "urdf",
            "units": {
                "q": "rad",
                "dq": "rad/s",
                "current_mA": "mA",
                "load_percent": "percent",
                "q_cmd": "rad",
            },
            **self.meta,
        }
        (out / "meta.json").write_text(json.dumps(meta, indent=2, default=str) + "\n")
        return out

    @classmethod
    def load(cls, run_dir: Union[str, Path]) -> "IdentificationLog":
        d = Path(run_dir)
        meta = json.loads((d / "meta.json").read_text())
        if meta.get("format") != FORMAT:
            raise ValueError(f"{d} is not a {FORMAT} log (format={meta.get('format')!r})")
        names = list(meta["joint_names"])
        arrays: Dict[str, Optional[np.ndarray]] = {}
        t = None
        for name in _SIGNALS:
            p = d / f"{name}.csv"
            if not p.exists():
                arrays[name] = None
                continue
            data = np.loadtxt(p, delimiter=",", skiprows=1, ndmin=2)
            t = data[:, 0]
            arrays[name] = data[:, 1:]
        extra = {
            k: v
            for k, v in meta.items()
            if k not in ("format", "joint_names", "n_samples", "rate_hz", "frame", "units")
        }
        return cls(joint_names=names, t=t, meta=extra, **arrays)  # type: ignore[arg-type]
