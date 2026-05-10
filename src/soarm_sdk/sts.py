"""High-level helpers for STS series servos."""

from __future__ import annotations

from typing import Tuple

from .group_sync_read import GroupSyncRead
from .group_sync_write import GroupSyncWrite
from .protocol_packet_handler import ProtocolPacketHandler
from .stservo_def import (
    BROADCAST_ID,
    COMM_SUCCESS,
    STS_ACC,
    STS_LOCK,
    STS_MODE,
    STS_MOVING,
    STS_OFS_L,
    STS_PRESENT_CURRENT_L,
    STS_PRESENT_LOAD_L,
    STS_PRESENT_POSITION_L,
    STS_PRESENT_SPEED_L,
    STS_PRESENT_TEMPERATURE,
    STS_PRESENT_VOLTAGE,
    STS_STATUS,
)


class sts(ProtocolPacketHandler):  # noqa: N801 - legacy name
    """Backwards-compatible STS servo helper."""

    def __init__(self, port_handler) -> None:
        super().__init__(port_handler, 0)
        self.groupSyncWrite = GroupSyncWrite(self, STS_ACC, 7)
        self.groupSyncRead = GroupSyncRead(self, STS_PRESENT_POSITION_L, 4)

    def WritePosEx(  # noqa: N802
        self, servo_id: int, position: int, speed: int, acc: int
    ) -> Tuple[int, int]:
        payload = [
            acc,
            self.sts_lobyte(position),
            self.sts_hibyte(position),
            0,
            0,
            self.sts_lobyte(speed),
            self.sts_hibyte(speed),
        ]
        return self.writeTxRx(servo_id, STS_ACC, len(payload), payload)

    def ReadPos(self, servo_id: int) -> Tuple[int, int, int]:  # noqa: N802
        packet = self.read2ByteTxRx(servo_id, STS_PRESENT_POSITION_L)
        if packet.result == 0:
            value = self.sts_tohost(packet.data[0], 15)
        else:
            value = 0
        return value, packet.result, packet.error

    def ReadSpeed(self, servo_id: int) -> Tuple[int, int, int]:  # noqa: N802
        packet = self.read2ByteTxRx(servo_id, STS_PRESENT_SPEED_L)
        if packet.result == 0:
            value = self.sts_tohost(packet.data[0], 15)
        else:
            value = 0
        return value, packet.result, packet.error

    def ReadPosSpeed(  # noqa: N802
        self, servo_id: int
    ) -> Tuple[int, int, int, int]:
        packet = self.read4ByteTxRx(servo_id, STS_PRESENT_POSITION_L)
        if packet.result == 0:
            combined = packet.data[0]
            position = self.sts_loword(combined)
            speed = self.sts_hiword(combined)
            position = self.sts_tohost(position, 15)
            speed = self.sts_tohost(speed, 15)
        else:
            position = 0
            speed = 0
        return position, speed, packet.result, packet.error

    def ReadMoving(self, servo_id: int) -> Tuple[int, int, int]:  # noqa: N802
        packet = self.read1ByteTxRx(servo_id, STS_MOVING)
        value = packet.data[0] if packet.result == 0 else 0
        return value, packet.result, packet.error

    def SyncWritePosEx(  # noqa: N802
        self, servo_id: int, position: int, speed: int, acc: int
    ) -> bool:
        payload = [
            acc,
            self.sts_lobyte(position),
            self.sts_hibyte(position),
            0,
            0,
            self.sts_lobyte(speed),
            self.sts_hibyte(speed),
        ]
        return self.groupSyncWrite.addParam(servo_id, payload)

    def RegWritePosEx(  # noqa: N802
        self, servo_id: int, position: int, speed: int, acc: int
    ) -> Tuple[int, int]:
        payload = [
            acc,
            self.sts_lobyte(position),
            self.sts_hibyte(position),
            0,
            0,
            self.sts_lobyte(speed),
            self.sts_hibyte(speed),
        ]
        return self.regWriteTxRx(servo_id, STS_ACC, len(payload), payload)

    def RegAction(self) -> int:  # noqa: N802
        return self.action(BROADCAST_ID)

    def WheelMode(self, servo_id: int) -> Tuple[int, int]:  # noqa: N802
        return self.write1ByteTxRx(servo_id, STS_MODE, 1)

    def WriteSpec(  # noqa: N802
        self, servo_id: int, speed: int, acc: int
    ) -> Tuple[int, int]:
        signed_speed = self.sts_toscs(speed, 15)
        payload = [
            acc,
            0,
            0,
            0,
            0,
            self.sts_lobyte(signed_speed),
            self.sts_hibyte(signed_speed),
        ]
        return self.writeTxRx(servo_id, STS_ACC, len(payload), payload)

    def LockEprom(self, servo_id: int) -> Tuple[int, int]:  # noqa: N802
        return self.write1ByteTxRx(servo_id, STS_LOCK, 1)

    def unLockEprom(self, servo_id: int) -> Tuple[int, int]:  # noqa: N802
        return self.write1ByteTxRx(servo_id, STS_LOCK, 0)

    # Telemetry helpers -------------------------------------------------
    def GetBaudrate(self) -> int:  # noqa: N802
        return self.port_handler.getBaudRate()

    def ReadLoad(self, servo_id: int) -> Tuple[float, int, int]:  # noqa: N802
        packet = self.read1ByteTxRx(servo_id, STS_PRESENT_LOAD_L)
        if packet.result == COMM_SUCCESS and packet.data:
            value = packet.data[0] * 0.1
        else:
            value = 0.0
        return value, packet.result, packet.error

    def ReadVoltage(
        self, servo_id: int
    ) -> Tuple[float, int, int]:  # noqa: N802
        packet = self.read1ByteTxRx(servo_id, STS_PRESENT_VOLTAGE)
        if packet.result == COMM_SUCCESS and packet.data:
            value = packet.data[0] * 0.1
        else:
            value = 0.0
        return value, packet.result, packet.error

    def ReadCurrent(
        self, servo_id: int
    ) -> Tuple[float, int, int]:  # noqa: N802
        packet = self.read1ByteTxRx(servo_id, STS_PRESENT_CURRENT_L)
        if packet.result == COMM_SUCCESS and packet.data:
            value = packet.data[0] * 6.5
        else:
            value = 0.0
        return value, packet.result, packet.error

    def ReadTemperature(
        self, servo_id: int
    ) -> Tuple[int, int, int]:  # noqa: N802
        packet = self.read1ByteTxRx(servo_id, STS_PRESENT_TEMPERATURE)
        if packet.result == COMM_SUCCESS and packet.data:
            value = packet.data[0]
        else:
            value = 0
        return value, packet.result, packet.error

    def ReadAccelaration(
        self, servo_id: int
    ) -> Tuple[int, int, int]:  # noqa: N802
        packet = self.read1ByteTxRx(servo_id, STS_ACC)
        if packet.result == COMM_SUCCESS and packet.data:
            value = packet.data[0]
        else:
            value = 0
        return value, packet.result, packet.error

    def ReadMode(self, servo_id: int) -> Tuple[int, int, int]:  # noqa: N802
        packet = self.read1ByteTxRx(servo_id, STS_MODE)
        if packet.result == COMM_SUCCESS and packet.data:
            value = packet.data[0]
        else:
            value = 0
        return value, packet.result, packet.error

    def ReadCorrection(
        self, servo_id: int
    ) -> Tuple[int, int, int]:  # noqa: N802
        packet = self.read2ByteTxRx(servo_id, STS_OFS_L)
        if packet.result == COMM_SUCCESS and packet.data:
            raw = packet.data[0]
            magnitude = raw & 0x7FF
            if raw & 0x0800:
                magnitude = -magnitude
            value = magnitude
        else:
            value = 0
        return value, packet.result, packet.error

    def IsMoving(self, servo_id: int) -> Tuple[bool, int, int]:  # noqa: N802
        packet = self.read1ByteTxRx(servo_id, STS_MOVING)
        if packet.result == COMM_SUCCESS and packet.data:
            value = bool(packet.data[0])
        else:
            value = False
        return value, packet.result, packet.error

    def ReadPosition(
        self, servo_id: int
    ) -> Tuple[int, int, int]:  # noqa: N802
        packet = self.read2ByteTxRx(servo_id, STS_PRESENT_POSITION_L)
        if packet.result == COMM_SUCCESS and packet.data:
            value = self.sts_tohost(packet.data[0], 15)
        else:
            value = 0
        return value, packet.result, packet.error

    def ReadStatus(self, servo_id: int) -> Tuple[int, int, int]:  # noqa: N802
        packet = self.read1ByteTxRx(servo_id, STS_STATUS)
        if packet.result == COMM_SUCCESS and packet.data:
            value = packet.data[0]
        else:
            value = 0
        return value, packet.result, packet.error
