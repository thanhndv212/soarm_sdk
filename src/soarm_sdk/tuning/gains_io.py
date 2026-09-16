"""Read/write the STS3215 P/D/I EEPROM gains.

Shared by the dashboard's manual Read/Write buttons and the auto-tune search
loop, so both go through one unlock/write/lock sequence instead of two
copies of it drifting apart.
"""

from __future__ import annotations

from dataclasses import dataclass

from .. import COMM_SUCCESS, STS_LOCK, write1
from ..protocol.registers import STS_D_COEF, STS_I_COEF, STS_P_COEF

__all__ = ["Gains", "read_gains", "write_gains"]


@dataclass(frozen=True)
class Gains:
    p: int
    d: int
    i: int

    def __post_init__(self) -> None:
        for name in ("p", "d", "i"):
            value = getattr(self, name)
            if not isinstance(value, int) or isinstance(value, bool) or not (0 <= value <= 254):
                raise ValueError(f"gain {name}={value!r} must be an int in [0, 254]")


def read_gains(bus, servo_id: int) -> Gains:
    """Read the current P/D/I gains for *servo_id* from EEPROM.

    ``read1ByteTxRx`` returns a :class:`~soarm_sdk.protocol.packet_handler.PacketResult`
    (``.data`` / ``.result`` / ``.error``) — a dataclass, not a plain tuple,
    so it is not unpackable as ``val, result, _ = ...``. Every other reader
    in :mod:`soarm_sdk.protocol.sts` (``ReadVoltage``, ``ReadCurrent``, ...)
    already goes through ``.data``/``.result`` for exactly this reason.
    """

    def _read1(addr: int, label: str) -> int:
        packet = bus.read1ByteTxRx(servo_id, addr)
        if packet.result != COMM_SUCCESS or not packet.data:
            raise IOError(f"Read {label} failed for ID {servo_id} (result={packet.result})")
        return packet.data[0]

    return Gains(
        p=_read1(STS_P_COEF, "P"),
        d=_read1(STS_D_COEF, "D"),
        i=_read1(STS_I_COEF, "I"),
    )


def write_gains(bus, servo_id: int, gains: Gains) -> None:
    """Write *gains* to *servo_id*'s EEPROM (unlock, write, lock)."""
    write1(bus, servo_id, STS_LOCK, 0, "unlock EEPROM")
    try:
        write1(bus, servo_id, STS_P_COEF, gains.p, "P gain")
        write1(bus, servo_id, STS_D_COEF, gains.d, "D gain")
        write1(bus, servo_id, STS_I_COEF, gains.i, "I gain")
    finally:
        write1(bus, servo_id, STS_LOCK, 1, "lock EEPROM")
