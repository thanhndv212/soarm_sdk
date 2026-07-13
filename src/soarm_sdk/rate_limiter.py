"""Simple rate limiter for controlling loop execution frequency."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional

__all__ = ["RateLimiter"]


@dataclass
class RateLimiter:
    """Maintain a fixed execution frequency for control loops.

    Parameters
    ----------
    frequency: float
        Target frequency in hertz. If zero or negative, sleep becomes a no-op.
    warn: bool
        When True, log overruns via print statements. Defaults to False to
        avoid clutter when running in tight loops.
    """

    frequency: float
    warn: bool = False
    _last_time: Optional[float] = field(default=None, init=False, repr=False)

    def __post_init__(self) -> None:
        self.dt = 0.0 if self.frequency <= 0.0 else 1.0 / self.frequency
        self.reset()

    def reset(self) -> None:
        """Reset the internal timer to the current time."""
        self._last_time = time.perf_counter()

    def sleep(self) -> None:
        """Sleep just long enough to maintain the requested rate."""
        if self.frequency <= 0.0:
            return

        if self._last_time is None:
            self.reset()

        target = self._last_time + self.dt
        now = time.perf_counter()
        remaining = target - now
        if remaining > 0:
            time.sleep(remaining)
            self._last_time = target
        else:
            if self.warn:
                overrun_ms = -remaining * 1e3
                print(
                    "RateLimiter overrun by "
                    f"{overrun_ms:.3f} ms at {now:.6f}",
                )
            self._last_time = now

    def wait(self) -> None:
        """Alias for :meth:`sleep` for compatibility with other libraries."""
        self.sleep()
