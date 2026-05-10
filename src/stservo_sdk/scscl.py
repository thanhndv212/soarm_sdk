"""High-level helpers for SCSCL series servos."""

from __future__ import annotations

from typing import Tuple

from .group_sync_write import GroupSyncWrite
from .protocol_packet_handler import ProtocolPacketHandler
from .stservo_def import (
    BROADCAST_ID,
    SCSCL_GOAL_POSITION_L,
    SCSCL_GOAL_TIME_L,
    SCSCL_LOCK,
    SCSCL_MIN_ANGLE_LIMIT_L,
    SCSCL_PRESENT_POSITION_L,
    SCSCL_PRESENT_SPEED_L,
    SCSCL_MOVING,
)


class scscl(ProtocolPacketHandler):  # noqa: N801 - legacy name
    """Backwards-compatible SCSCL servo helper."""

    def __init__(self, port_handler) -> None:
        super().__init__(port_handler, 1)
        self.groupSyncWrite = GroupSyncWrite(self, SCSCL_GOAL_POSITION_L, 6)

    def WritePos(  # noqa: N802
        self, servo_id: int, position: int, time: int, speed: int
    ) -> Tuple[int, int]:
        payload = [
            self.sts_lobyte(position),
            self.sts_hibyte(position),
            self.sts_lobyte(time),
            self.sts_hibyte(time),
            self.sts_lobyte(speed),
            self.sts_hibyte(speed),
        ]
        return self.writeTxRx(
            servo_id, SCSCL_GOAL_POSITION_L, len(payload), payload
        )

    def ReadPos(self, servo_id: int) -> Tuple[int, int, int]:  # noqa: N802
        packet = self.read2ByteTxRx(servo_id, SCSCL_PRESENT_POSITION_L)
        value = packet.data[0] if packet.result == 0 else 0
        return value, packet.result, packet.error

    def ReadSpeed(self, servo_id: int) -> Tuple[int, int, int]:  # noqa: N802
        packet = self.read2ByteTxRx(servo_id, SCSCL_PRESENT_SPEED_L)
        if packet.result == 0:
            value = self.sts_tohost(packet.data[0], 15)
        else:
            value = 0
        return value, packet.result, packet.error

    def ReadPosSpeed(  # noqa: N802
        self, servo_id: int
    ) -> Tuple[int, int, int, int]:
        packet = self.read4ByteTxRx(servo_id, SCSCL_PRESENT_POSITION_L)
        if packet.result == 0:
            combined = packet.data[0]
            position = self.sts_loword(combined)
            speed = self.sts_hiword(combined)
            speed = self.sts_tohost(speed, 15)
        else:
            position = 0
            speed = 0
        return position, speed, packet.result, packet.error

    def ReadMoving(self, servo_id: int) -> Tuple[int, int, int]:  # noqa: N802
        packet = self.read1ByteTxRx(servo_id, SCSCL_MOVING)
        value = packet.data[0] if packet.result == 0 else 0
        return value, packet.result, packet.error

    def SyncWritePos(  # noqa: N802
        self, servo_id: int, position: int, time: int, speed: int
    ) -> bool:
        payload = [
            self.sts_lobyte(position),
            self.sts_hibyte(position),
            self.sts_lobyte(time),
            self.sts_hibyte(time),
            self.sts_lobyte(speed),
            self.sts_hibyte(speed),
        ]
        return self.groupSyncWrite.addParam(servo_id, payload)

    def RegWritePos(  # noqa: N802
        self, servo_id: int, position: int, time: int, speed: int
    ) -> Tuple[int, int]:
        payload = [
            self.sts_lobyte(position),
            self.sts_hibyte(position),
            self.sts_lobyte(time),
            self.sts_hibyte(time),
            self.sts_lobyte(speed),
            self.sts_hibyte(speed),
        ]
        return self.regWriteTxRx(
            servo_id, SCSCL_GOAL_POSITION_L, len(payload), payload
        )

    def RegAction(self) -> int:  # noqa: N802
        return self.action(BROADCAST_ID)

    def PWMMode(self, servo_id: int) -> Tuple[int, int]:  # noqa: N802
        payload = [0, 0, 0, 0]
        return self.writeTxRx(
            servo_id, SCSCL_MIN_ANGLE_LIMIT_L, len(payload), payload
        )

    def WritePWM(  # noqa: N802
        self, servo_id: int, time: int
    ) -> Tuple[int, int]:
        signed_time = self.sts_toscs(time, 10)
        return self.write2ByteTxRx(servo_id, SCSCL_GOAL_TIME_L, signed_time)

    def LockEprom(self, servo_id: int) -> Tuple[int, int]:  # noqa: N802
        return self.write1ByteTxRx(servo_id, SCSCL_LOCK, 1)

    def unLockEprom(self, servo_id: int) -> Tuple[int, int]:  # noqa: N802
        return self.write1ByteTxRx(servo_id, SCSCL_LOCK, 0)
