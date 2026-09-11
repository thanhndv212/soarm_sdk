"""Protocol implementation for STServo communication.

This module provides a modernized port of the original
``protocol_packet_handler`` implementation that powers STS/SCS servos. The
class preserves the legacy method names so existing scripts can continue to run
while benefiting from type hints and clearer structure.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Sequence, Tuple

from .port_handler import PortHandler
from .registers import (
    BROADCAST_ID,
    COMM_NOT_AVAILABLE,
    COMM_PORT_BUSY,
    COMM_RX_CORRUPT,
    COMM_RX_FAIL,
    COMM_RX_TIMEOUT,
    COMM_RX_WAITING,
    COMM_SUCCESS,
    COMM_TX_ERROR,
    COMM_TX_FAIL,
    INST_ACTION,
    INST_PING,
    INST_READ,
    INST_REG_WRITE,
    INST_SYNC_READ,
    INST_SYNC_WRITE,
    INST_WRITE,
)

TXPACKET_MAX_LEN = 250
RXPACKET_MAX_LEN = 250

PKT_HEADER0 = 0
PKT_HEADER1 = 1
PKT_ID = 2
PKT_LENGTH = 3
PKT_INSTRUCTION = 4
PKT_ERROR = 4
PKT_PARAMETER0 = 5

ERRBIT_VOLTAGE = 1
ERRBIT_ANGLE = 2
ERRBIT_OVERHEAT = 4
ERRBIT_OVERELE = 8
ERRBIT_OVERLOAD = 32


@dataclass(frozen=True)
class PacketResult:
    """Return data for combined tx/rx operations."""

    data: List[int]
    result: int
    error: int = 0


class ProtocolPacketHandler:
    """Low-level helper that speaks the STServo packet protocol."""

    def __init__(self, port_handler: PortHandler, protocol_end: int) -> None:
        self.port_handler = port_handler
        # Compatibility with legacy attribute casing
        self.portHandler = port_handler
        self.sts_end = protocol_end

    # \-\-\- byte helpers -------------------------------------------------
    def sts_getend(self) -> int:
        return self.sts_end

    def sts_setend(self, value: int) -> None:
        self.sts_end = value

    def sts_tohost(self, value: int, sign_bit: int) -> int:
        if value & (1 << sign_bit):
            return -(value & ~(1 << sign_bit))
        return value

    def sts_toscs(self, value: int, sign_bit: int) -> int:
        if value < 0:
            return (-value) | (1 << sign_bit)
        return value

    def sts_makeword(self, low: int, high: int) -> int:
        if self.sts_end == 0:
            return (low & 0xFF) | ((high & 0xFF) << 8)
        return (high & 0xFF) | ((low & 0xFF) << 8)

    @staticmethod
    def sts_makedword(low_word: int, high_word: int) -> int:
        return (low_word & 0xFFFF) | ((high_word & 0xFFFF) << 16)

    @staticmethod
    def sts_loword(value: int) -> int:
        return value & 0xFFFF

    @staticmethod
    def sts_hiword(value: int) -> int:
        return (value >> 16) & 0xFFFF

    def sts_lobyte(self, value: int) -> int:
        if self.sts_end == 0:
            return value & 0xFF
        return (value >> 8) & 0xFF

    def sts_hibyte(self, value: int) -> int:
        if self.sts_end == 0:
            return (value >> 8) & 0xFF
        return value & 0xFF

    # \-\-\- high-level helpers ------------------------------------------
    @staticmethod
    def getProtocolVersion() -> float:  # noqa: N802 - legacy API
        return 1.0

    @staticmethod
    def getTxRxResult(result: int) -> str:  # noqa: N802
        mapping = {
            COMM_SUCCESS: "[TxRxResult] Communication success!",
            COMM_PORT_BUSY: "[TxRxResult] Port is in use!",
            COMM_TX_FAIL: "[TxRxResult] Failed transmit instruction packet!",
            COMM_RX_FAIL: "[TxRxResult] Failed get status packet from device!",
            COMM_TX_ERROR: "[TxRxResult] Incorrect instruction packet!",
            COMM_RX_WAITING: "[TxRxResult] Now receiving status packet!",
            COMM_RX_TIMEOUT: "[TxRxResult] There is no status packet!",
            COMM_RX_CORRUPT: "[TxRxResult] Incorrect status packet!",
            COMM_NOT_AVAILABLE: (
                "[TxRxResult] Protocol does not support this function!"
            ),
        }
        return mapping.get(result, "")

    @staticmethod
    def getRxPacketError(error: int) -> str:  # noqa: N802
        if error & ERRBIT_VOLTAGE:
            return "[ServoStatus] Input voltage error!"
        if error & ERRBIT_ANGLE:
            return "[ServoStatus] Angle sensor error!"
        if error & ERRBIT_OVERHEAT:
            return "[ServoStatus] Overheat error!"
        if error & ERRBIT_OVERELE:
            return "[ServoStatus] Overcurrent error!"
        if error & ERRBIT_OVERLOAD:
            return "[ServoStatus] Overload error!"
        return ""

    # \-\-\- packet primitives -------------------------------------------
    def txPacket(self, txpacket: Sequence[int]) -> int:  # noqa: N802
        checksum = 0
        total_packet_length = txpacket[PKT_LENGTH] + 4

        if self.port_handler.is_using:
            return COMM_PORT_BUSY
        self.port_handler.is_using = True

        if total_packet_length > TXPACKET_MAX_LEN:
            self.port_handler.is_using = False
            return COMM_TX_ERROR

        packet = bytearray(total_packet_length)
        packet[PKT_HEADER0] = 0xFF
        packet[PKT_HEADER1] = 0xFF
        for idx in range(2, total_packet_length - 1):
            value = txpacket[idx]
            packet[idx] = value
            checksum += value
        packet[total_packet_length - 1] = (~checksum) & 0xFF

        self.port_handler.clearPort()
        written = self.port_handler.writePort(packet)
        if written != total_packet_length:
            self.port_handler.is_using = False
            return COMM_TX_FAIL
        return COMM_SUCCESS

    def rxPacket(self) -> Tuple[List[int], int]:  # noqa: N802
        rxpacket: List[int] = []
        checksum = 0
        rx_length = 0
        wait_length = 6
        result = COMM_TX_FAIL

        while True:
            rxpacket.extend(
                self.port_handler.readPort(wait_length - rx_length)
            )
            rx_length = len(rxpacket)
            if rx_length >= wait_length:
                idx = 0
                for idx in range(rx_length - 1):
                    if rxpacket[idx] == 0xFF and rxpacket[idx + 1] == 0xFF:
                        break
                if idx == 0:
                    if any(
                        (
                            rxpacket[PKT_ID] > 0xFD,
                            rxpacket[PKT_LENGTH] > RXPACKET_MAX_LEN,
                            rxpacket[PKT_ERROR] > 0x7F,
                        )
                    ):
                        del rxpacket[0]
                        rx_length -= 1
                        continue

                    expected_length = rxpacket[PKT_LENGTH] + PKT_LENGTH + 1
                    if wait_length != expected_length:
                        wait_length = expected_length
                        continue

                    if rx_length < wait_length:
                        if self.port_handler.isPacketTimeout():
                            if rx_length == 0:
                                result = COMM_RX_TIMEOUT
                            else:
                                result = COMM_RX_CORRUPT
                            break
                        continue

                    for i in range(2, wait_length - 1):
                        checksum += rxpacket[i]
                    checksum = (~checksum) & 0xFF

                    if rxpacket[wait_length - 1] == checksum:
                        result = COMM_SUCCESS
                    else:
                        result = COMM_RX_CORRUPT
                    break
                else:
                    del rxpacket[0:idx]
                    rx_length -= idx
            else:
                if self.port_handler.isPacketTimeout():
                    if rx_length == 0:
                        result = COMM_RX_TIMEOUT
                    else:
                        result = COMM_RX_CORRUPT
                    break

        self.port_handler.is_using = False
        return rxpacket, result

    def txRxPacket(  # noqa: N802
        self, txpacket: Sequence[int]
    ) -> PacketResult:
        result = self.txPacket(txpacket)
        if result != COMM_SUCCESS:
            return PacketResult([], result)

        if txpacket[PKT_ID] == BROADCAST_ID:
            self.port_handler.is_using = False
            return PacketResult([], result)

        if txpacket[PKT_INSTRUCTION] == INST_READ:
            timeout = txpacket[PKT_PARAMETER0 + 1] + 6
            self.port_handler.setPacketTimeout(timeout)
        else:
            self.port_handler.setPacketTimeout(6)

        while True:
            rxpacket, result = self.rxPacket()
            if result != COMM_SUCCESS or txpacket[PKT_ID] == rxpacket[PKT_ID]:
                break

        if result == COMM_SUCCESS and rxpacket:
            error = rxpacket[PKT_ERROR]
        else:
            error = 0
        return PacketResult(rxpacket, result, error)

    # \-\-\- instruction helpers ----------------------------------------
    def ping(self, sts_id: int) -> PacketResult:
        if sts_id >= BROADCAST_ID:
            return PacketResult([], COMM_NOT_AVAILABLE)

        txpacket = [0] * 6
        txpacket[PKT_ID] = sts_id
        txpacket[PKT_LENGTH] = 2
        txpacket[PKT_INSTRUCTION] = INST_PING

        packet = self.txRxPacket(txpacket)
        if packet.result != COMM_SUCCESS:
            return packet

        packet_read = self.readTxRx(sts_id, 3, 2)
        if packet_read.result == COMM_SUCCESS and len(packet_read.data) >= 2:
            value = self.sts_makeword(packet_read.data[0], packet_read.data[1])
            return PacketResult([value], packet_read.result, packet_read.error)
        return PacketResult([], packet_read.result, packet_read.error)

    def action(self, sts_id: int) -> int:
        txpacket = [0] * 6
        txpacket[PKT_ID] = sts_id
        txpacket[PKT_LENGTH] = 2
        txpacket[PKT_INSTRUCTION] = INST_ACTION
        return self.txRxPacket(txpacket).result

    # -- read helpers ----------------------------------------------------
    def readTx(  # noqa: N802
        self, sts_id: int, address: int, length: int
    ) -> int:
        if sts_id >= BROADCAST_ID:
            return COMM_NOT_AVAILABLE

        txpacket = [0] * 8
        txpacket[PKT_ID] = sts_id
        txpacket[PKT_LENGTH] = 4
        txpacket[PKT_INSTRUCTION] = INST_READ
        txpacket[PKT_PARAMETER0] = address
        txpacket[PKT_PARAMETER0 + 1] = length

        result = self.txPacket(txpacket)
        if result == COMM_SUCCESS:
            self.port_handler.setPacketTimeout(length + 6)
        return result

    def readRx(self, sts_id: int, length: int) -> PacketResult:  # noqa: N802
        data: List[int] = []
        while True:
            rxpacket, result = self.rxPacket()
            if result != COMM_SUCCESS or rxpacket[PKT_ID] == sts_id:
                break
        if result == COMM_SUCCESS and rxpacket[PKT_ID] == sts_id:
            data.extend(rxpacket[PKT_PARAMETER0 : PKT_PARAMETER0 + length])
            error = rxpacket[PKT_ERROR]
            return PacketResult(data, result, error)
        return PacketResult(data, result, 0)

    def readTxRx(  # noqa: N802
        self, sts_id: int, address: int, length: int
    ) -> PacketResult:
        if sts_id >= BROADCAST_ID:
            return PacketResult([], COMM_NOT_AVAILABLE)

        txpacket = [0] * 8
        txpacket[PKT_ID] = sts_id
        txpacket[PKT_LENGTH] = 4
        txpacket[PKT_INSTRUCTION] = INST_READ
        txpacket[PKT_PARAMETER0] = address
        txpacket[PKT_PARAMETER0 + 1] = length

        packet = self.txRxPacket(txpacket)
        if packet.result == COMM_SUCCESS and packet.data:
            data = packet.data[PKT_PARAMETER0 : PKT_PARAMETER0 + length]
            error = packet.data[PKT_ERROR]
            return PacketResult(data, packet.result, error)
        return PacketResult([], packet.result, packet.error)

    def read1ByteTx(self, sts_id: int, address: int) -> int:  # noqa: N802
        return self.readTx(sts_id, address, 1)

    def read1ByteRx(self, sts_id: int) -> PacketResult:  # noqa: N802
        packet = self.readRx(sts_id, 1)
        if packet.result == COMM_SUCCESS and packet.data:
            data = packet.data[0]
        else:
            data = 0
        return PacketResult([data], packet.result, packet.error)

    def read1ByteTxRx(  # noqa: N802
        self, sts_id: int, address: int
    ) -> PacketResult:
        packet = self.readTxRx(sts_id, address, 1)
        if packet.result == COMM_SUCCESS and packet.data:
            data = packet.data[0]
        else:
            data = 0
        return PacketResult([data], packet.result, packet.error)

    def read2ByteTx(self, sts_id: int, address: int) -> int:  # noqa: N802
        return self.readTx(sts_id, address, 2)

    def read2ByteRx(self, sts_id: int) -> PacketResult:  # noqa: N802
        packet = self.readRx(sts_id, 2)
        if packet.result == COMM_SUCCESS and len(packet.data) >= 2:
            value = self.sts_makeword(packet.data[0], packet.data[1])
            return PacketResult([value], packet.result, packet.error)
        return PacketResult([0], packet.result, packet.error)

    def read2ByteTxRx(  # noqa: N802
        self, sts_id: int, address: int
    ) -> PacketResult:
        packet = self.readTxRx(sts_id, address, 2)
        if packet.result == COMM_SUCCESS and len(packet.data) >= 2:
            value = self.sts_makeword(packet.data[0], packet.data[1])
            return PacketResult([value], packet.result, packet.error)
        return PacketResult([0], packet.result, packet.error)

    def read4ByteTx(self, sts_id: int, address: int) -> int:  # noqa: N802
        return self.readTx(sts_id, address, 4)

    def read4ByteRx(self, sts_id: int) -> PacketResult:  # noqa: N802
        packet = self.readRx(sts_id, 4)
        if packet.result == COMM_SUCCESS and len(packet.data) >= 4:
            value = self.sts_makedword(
                self.sts_makeword(packet.data[0], packet.data[1]),
                self.sts_makeword(packet.data[2], packet.data[3]),
            )
            return PacketResult([value], packet.result, packet.error)
        return PacketResult([0], packet.result, packet.error)

    def read4ByteTxRx(  # noqa: N802
        self, sts_id: int, address: int
    ) -> PacketResult:
        packet = self.readTxRx(sts_id, address, 4)
        if packet.result == COMM_SUCCESS and len(packet.data) >= 4:
            value = self.sts_makedword(
                self.sts_makeword(packet.data[0], packet.data[1]),
                self.sts_makeword(packet.data[2], packet.data[3]),
            )
            return PacketResult([value], packet.result, packet.error)
        return PacketResult([0], packet.result, packet.error)

    # -- write helpers ---------------------------------------------------
    def writeTxOnly(
        self, sts_id: int, address: int, length: int, data: Sequence[int]
    ) -> int:  # noqa: N802
        txpacket = [0] * (length + 7)
        txpacket[PKT_ID] = sts_id
        txpacket[PKT_LENGTH] = length + 3
        txpacket[PKT_INSTRUCTION] = INST_WRITE
        txpacket[PKT_PARAMETER0] = address
        end = PKT_PARAMETER0 + 1 + length
        txpacket[PKT_PARAMETER0 + 1 : end] = data[:length]

        result = self.txPacket(txpacket)
        self.port_handler.is_using = False
        return result

    def writeTxRx(
        self, sts_id: int, address: int, length: int, data: Sequence[int]
    ) -> Tuple[int, int]:  # noqa: N802
        txpacket = [0] * (length + 7)
        txpacket[PKT_ID] = sts_id
        txpacket[PKT_LENGTH] = length + 3
        txpacket[PKT_INSTRUCTION] = INST_WRITE
        txpacket[PKT_PARAMETER0] = address
        end = PKT_PARAMETER0 + 1 + length
        txpacket[PKT_PARAMETER0 + 1 : end] = data[:length]

        packet = self.txRxPacket(txpacket)
        return packet.result, packet.error

    def write1ByteTxOnly(  # noqa: N802
        self, sts_id: int, address: int, data: int
    ) -> int:
        return self.writeTxOnly(sts_id, address, 1, [data])

    def write1ByteTxRx(  # noqa: N802
        self, sts_id: int, address: int, data: int
    ) -> Tuple[int, int]:
        return self.writeTxRx(sts_id, address, 1, [data])

    def write2ByteTxOnly(  # noqa: N802
        self, sts_id: int, address: int, data: int
    ) -> int:
        payload = [self.sts_lobyte(data), self.sts_hibyte(data)]
        return self.writeTxOnly(sts_id, address, 2, payload)

    def write2ByteTxRx(  # noqa: N802
        self, sts_id: int, address: int, data: int
    ) -> Tuple[int, int]:
        payload = [self.sts_lobyte(data), self.sts_hibyte(data)]
        return self.writeTxRx(sts_id, address, 2, payload)

    def write4ByteTxOnly(  # noqa: N802
        self, sts_id: int, address: int, data: int
    ) -> int:
        payload = [
            self.sts_lobyte(self.sts_loword(data)),
            self.sts_hibyte(self.sts_loword(data)),
            self.sts_lobyte(self.sts_hiword(data)),
            self.sts_hibyte(self.sts_hiword(data)),
        ]
        return self.writeTxOnly(sts_id, address, 4, payload)

    def write4ByteTxRx(  # noqa: N802
        self, sts_id: int, address: int, data: int
    ) -> Tuple[int, int]:
        payload = [
            self.sts_lobyte(self.sts_loword(data)),
            self.sts_hibyte(self.sts_loword(data)),
            self.sts_lobyte(self.sts_hiword(data)),
            self.sts_hibyte(self.sts_hiword(data)),
        ]
        return self.writeTxRx(sts_id, address, 4, payload)

    def regWriteTxOnly(
        self, sts_id: int, address: int, length: int, data: Sequence[int]
    ) -> int:  # noqa: N802
        txpacket = [0] * (length + 7)
        txpacket[PKT_ID] = sts_id
        txpacket[PKT_LENGTH] = length + 3
        txpacket[PKT_INSTRUCTION] = INST_REG_WRITE
        txpacket[PKT_PARAMETER0] = address
        end = PKT_PARAMETER0 + 1 + length
        txpacket[PKT_PARAMETER0 + 1 : end] = data[:length]

        result = self.txPacket(txpacket)
        self.port_handler.is_using = False
        return result

    def regWriteTxRx(
        self, sts_id: int, address: int, length: int, data: Sequence[int]
    ) -> Tuple[int, int]:  # noqa: N802
        txpacket = [0] * (length + 7)
        txpacket[PKT_ID] = sts_id
        txpacket[PKT_LENGTH] = length + 3
        txpacket[PKT_INSTRUCTION] = INST_REG_WRITE
        txpacket[PKT_PARAMETER0] = address
        end = PKT_PARAMETER0 + 1 + length
        txpacket[PKT_PARAMETER0 + 1 : end] = data[:length]

        packet = self.txRxPacket(txpacket)
        return packet.result, packet.error

    def syncReadTx(
        self,
        start_address: int,
        data_length: int,
        param: Sequence[int],
        param_length: int,
    ) -> int:  # noqa: N802
        txpacket = [0] * (param_length + 8)
        txpacket[PKT_ID] = BROADCAST_ID
        txpacket[PKT_LENGTH] = param_length + 4
        txpacket[PKT_INSTRUCTION] = INST_SYNC_READ
        txpacket[PKT_PARAMETER0] = start_address
        txpacket[PKT_PARAMETER0 + 1] = data_length
        end = PKT_PARAMETER0 + 2 + param_length
        txpacket[PKT_PARAMETER0 + 2 : end] = param[:param_length]
        return self.txPacket(txpacket)

    def syncReadRx(  # noqa: N802
        self, data_length: int, param_length: int
    ) -> Tuple[int, List[int]]:
        wait_length = (6 + data_length) * param_length
        self.port_handler.setPacketTimeout(wait_length)
        rxpacket: List[int] = []
        rx_length = 0
        while True:
            rxpacket.extend(
                self.port_handler.readPort(wait_length - rx_length)
            )
            rx_length = len(rxpacket)
            if rx_length >= wait_length:
                result = COMM_SUCCESS
                break
            if self.port_handler.isPacketTimeout():
                if rx_length == 0:
                    result = COMM_RX_TIMEOUT
                else:
                    result = COMM_RX_CORRUPT
                break
        self.port_handler.is_using = False
        return result, rxpacket

    def syncWriteTxOnly(
        self,
        start_address: int,
        data_length: int,
        param: Sequence[int],
        param_length: int,
    ) -> int:  # noqa: N802
        txpacket = [0] * (param_length + 8)
        txpacket[PKT_ID] = BROADCAST_ID
        txpacket[PKT_LENGTH] = param_length + 4
        txpacket[PKT_INSTRUCTION] = INST_SYNC_WRITE
        txpacket[PKT_PARAMETER0] = start_address
        txpacket[PKT_PARAMETER0 + 1] = data_length
        end = PKT_PARAMETER0 + 2 + param_length
        txpacket[PKT_PARAMETER0 + 2 : end] = param[:param_length]
        return self.txRxPacket(txpacket).result


# Backwards compatible alias -------------------------------------------------
protocol_packet_handler = ProtocolPacketHandler
"""
Legacy name exported for compatibility with older user code.
"""
