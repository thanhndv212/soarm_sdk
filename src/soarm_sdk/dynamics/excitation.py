"""Excitation trajectories for dynamic identification.

A band-limited Fourier series per joint (Swevers et al., 1997) — the usual
choice for identification because it is periodic, smooth, and moves every
joint through both directions many times. Two things about it are specific
to this arm:

* **Slow by default.** STS3215 current is a noisy, friction-dominated torque
  proxy, and what this data is mostly for is gravity. Keeping velocity and
  acceleration low keeps the inertial terms negligible, so the fit is
  ``tau = g(q) + fv*qd + fs*sign(qd) + offset`` and nothing else — and a slow
  arm is also a safe one to leave running for two minutes.
* **Starts and ends at rest, at the centre pose.** Each joint's excursion is
  multiplied by a smooth window that is zero, with zero slope, at both ends.
  The trajectory can therefore be joined to a slow approach move without a
  velocity step, and a run cut short still has a clean start.

Every angle here is in the **URDF joint frame**.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np

__all__ = ["ExcitationSpec", "fourier_excitation"]


@dataclass(frozen=True)
class ExcitationSpec:
    """Shape of the excitation. Defaults suit a gravity + friction fit."""

    duration_s: float = 120.0
    rate_hz: float = 50.0
    #: Fundamental of the Fourier series; the pattern repeats every 1/base.
    base_freq_hz: float = 0.05
    n_harmonics: int = 3
    #: Peak joint speed, rad/s. The amplitude is scaled down to respect it.
    max_vel_rad_s: float = 0.5
    #: Kept clear of each joint limit, rad.
    margin_rad: float = 0.15
    #: Fraction of the available half-range (centre to nearer limit) used.
    amplitude_scale: float = 0.6
    #: Length of the fade-in and fade-out windows, s.
    ramp_s: float = 5.0
    seed: int = 0

    def __post_init__(self) -> None:
        if self.duration_s <= 2 * self.ramp_s:
            raise ValueError("duration_s must exceed twice ramp_s")
        if self.rate_hz <= 0 or self.base_freq_hz <= 0 or self.n_harmonics < 1:
            raise ValueError("rate_hz, base_freq_hz and n_harmonics must be positive")
        if self.max_vel_rad_s <= 0:
            raise ValueError("max_vel_rad_s must be positive")
        if not 0.0 < self.amplitude_scale <= 1.0:
            raise ValueError("amplitude_scale must be in (0, 1]")


def _window(t: np.ndarray, duration: float, ramp: float) -> np.ndarray:
    """1 in the middle, easing to 0 with zero slope at both ends."""
    def smooth(x: np.ndarray) -> np.ndarray:
        x = np.clip(x, 0.0, 1.0)
        return x * x * x * (x * (6.0 * x - 15.0) + 10.0)  # smootherstep

    return smooth(t / ramp) * smooth((duration - t) / ramp)


def fourier_excitation(
    lower: Sequence[float],
    upper: Sequence[float],
    spec: ExcitationSpec = ExcitationSpec(),
    *,
    active: Optional[Sequence[bool]] = None,
    center: Optional[Sequence[float]] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """Plan an excitation. Returns ``(t, q)``, shapes ``(N,)`` and ``(N, n)``.

    Parameters
    ----------
    lower, upper
        Joint limits in radians, URDF frame — pass the *effective* limits
        (declared intersected with measured travel), not the model's.
    active
        Which joints move. Inactive joints hold ``center`` throughout.
        Defaults to all.
    center
        The pose the motion is centred on and starts/ends at. Defaults to
        the midpoint of each joint's usable range. Must lie inside
        ``[lower + margin, upper - margin]``.
    """
    lo = np.asarray(lower, dtype=float) + spec.margin_rad
    hi = np.asarray(upper, dtype=float) - spec.margin_rad
    n = lo.size
    if hi.size != n:
        raise ValueError("lower and upper must have the same length")
    if np.any(hi <= lo):
        bad = np.flatnonzero(hi <= lo).tolist()
        raise ValueError(f"joints {bad} have no range left inside the margin")
    act = np.ones(n, dtype=bool) if active is None else np.asarray(active, dtype=bool)
    c = (lo + hi) / 2.0 if center is None else np.asarray(center, dtype=float)
    if act.shape != (n,) or c.shape != (n,):
        raise ValueError(f"active and center must have {n} entries")
    outside = np.flatnonzero(act & ((c < lo) | (c > hi)))
    if outside.size:
        raise ValueError(
            f"center is outside the usable range (limits minus margin) for joints "
            f"{outside.tolist()}"
        )

    t = np.arange(0.0, spec.duration_s + 0.5 / spec.rate_hz, 1.0 / spec.rate_hz)
    w = _window(t, spec.duration_s, spec.ramp_s)
    rng = np.random.default_rng(spec.seed)
    k = np.arange(1, spec.n_harmonics + 1)
    omega = 2.0 * np.pi * spec.base_freq_hz * k

    q = np.tile(c, (t.size, 1))
    for j in np.flatnonzero(act):
        a = rng.uniform(-1.0, 1.0, k.size) / k
        b = rng.uniform(-1.0, 1.0, k.size) / k
        s = np.sin(np.outer(t, omega)) @ a + np.cos(np.outer(t, omega)) @ b
        s -= s[0]
        shape = w * s
        shape /= np.max(np.abs(shape))
        amp = spec.amplitude_scale * min(c[j] - lo[j], hi[j] - c[j])
        # The peak speed of amp*shape, measured on the sampled signal.
        peak = amp * np.max(np.abs(np.gradient(shape, t)))
        if peak > spec.max_vel_rad_s:
            amp *= spec.max_vel_rad_s / peak
        q[:, j] = c[j] + amp * shape
    return t, q
