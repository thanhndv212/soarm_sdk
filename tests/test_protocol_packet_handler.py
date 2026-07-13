"""Unit tests for soarm_sdk.protocol_packet_handler.

Exercises packet framing and checksum computation against fake port-handler
doubles — no real serial hardware involved.
"""

from __future__ import annotations

from typing import Any, Optional

from soarm_sdk.protocol_packet_handler import (
    PKT_ERROR,
    PKT_ID,
    PKT_INSTRUCTION,
    PKT_LENGTH,
    ProtocolPacketHandler,
)
from soarm_sdk.stservo_def import (
    COMM_PORT_BUSY,
    COMM_RX_CORRUPT,
    COMM_RX_TIMEOUT,
    COMM_SUCCESS,
    COMM_TX_ERROR,
)


class FakeTxPortHandler:
    """Captures what txPacket() would write to the wire."""

    def __init__(self) -> None:
        self.is_using = False
        self.written: Optional[bytes] = None

    def clearPort(self) -> None:
        pass

    def writePort(self, packet) -> int:
        self.written = bytes(packet)
        return len(self.written)


class FakeRxPortHandler:
    """Feeds pre-loaded bytes through readPort(), like a buffered serial read."""

    def __init__(self, buffer: bytes) -> None:
        self._buffer = bytearray(buffer)
        self.is_using = False

    def readPort(self, length: int) -> bytes:
        n = min(length, len(self._buffer))
        chunk = bytes(self._buffer[:n])
        del self._buffer[:n]
        return chunk

    def isPacketTimeout(self) -> bool:
        return len(self._buffer) == 0

    def setPacketTimeout(self, length: int) -> None:
        pass


def _handler(port: Any = None, sts_end: int = 0) -> ProtocolPacketHandler:
    return ProtocolPacketHandler(port, sts_end)


def _checksum(payload_bytes: list[int]) -> int:
    """Same algorithm txPacket/rxPacket use: ~sum(bytes) & 0xFF."""
    return (~sum(payload_bytes)) & 0xFF


def _build_response_packet(
    servo_id: int, error: int, params: list[int]
) -> bytes:
    length_field = 2 + len(params)
    body = [servo_id, length_field, error, *params]
    checksum = _checksum(body)
    return bytes([0xFF, 0xFF, *body, checksum])


# ---------------------------------------------------------------------------
# Byte helpers
# ---------------------------------------------------------------------------


def test_sts_makeword_little_endian():
    handler = _handler(sts_end=0)
    assert handler.sts_makeword(low=0x12, high=0x34) == 0x3412


def test_sts_makeword_big_endian():
    handler = _handler(sts_end=1)
    assert handler.sts_makeword(low=0x12, high=0x34) == 0x1234


def test_sts_tohost_and_toscs_round_trip_negative_value():
    handler = _handler()
    scs = handler.sts_toscs(-5, sign_bit=15)
    assert handler.sts_tohost(scs, sign_bit=15) == -5


def test_sts_tohost_and_toscs_round_trip_positive_value():
    handler = _handler()
    scs = handler.sts_toscs(5, sign_bit=15)
    assert handler.sts_tohost(scs, sign_bit=15) == 5


def test_gettxrxresult_known_and_unknown_codes():
    message = ProtocolPacketHandler.getTxRxResult(COMM_SUCCESS)
    assert "success" in message.lower()
    assert ProtocolPacketHandler.getTxRxResult(-9999) == ""


def test_getrxpacketerror_overheat_bit():
    assert "overheat" in ProtocolPacketHandler.getRxPacketError(4).lower()


def test_getrxpacketerror_no_bits_set():
    assert ProtocolPacketHandler.getRxPacketError(0) == ""


# ---------------------------------------------------------------------------
# txPacket
# ---------------------------------------------------------------------------


def test_txpacket_writes_correct_checksum():
    port = FakeTxPortHandler()
    handler = _handler(port)

    txpacket = [0] * 6
    txpacket[PKT_ID] = 5
    txpacket[PKT_LENGTH] = 2
    txpacket[PKT_INSTRUCTION] = 1  # arbitrary instruction byte

    result = handler.txPacket(txpacket)

    assert result == COMM_SUCCESS
    assert port.written is not None
    assert port.written[0:2] == b"\xff\xff"
    expected_checksum = _checksum([5, 2, 1])
    assert port.written[-1] == expected_checksum
    # On success, txPacket() deliberately leaves is_using=True — the bus is
    # only released by the subsequent rxPacket() call in a full round trip,
    # not by the transmit side alone.
    assert port.is_using is True


def test_txpacket_rejects_port_already_in_use():
    port = FakeTxPortHandler()
    port.is_using = True
    handler = _handler(port)

    result = handler.txPacket([0, 0, 1, 2, 1, 0])

    assert result == COMM_PORT_BUSY
    assert port.written is None


def test_txpacket_rejects_oversized_packet():
    port = FakeTxPortHandler()
    handler = _handler(port)

    txpacket = [0] * 6
    txpacket[PKT_LENGTH] = 255  # total_packet_length = 259 > TXPACKET_MAX_LEN

    result = handler.txPacket(txpacket)

    assert result == COMM_TX_ERROR
    assert port.is_using is False


# ---------------------------------------------------------------------------
# rxPacket
# ---------------------------------------------------------------------------


def test_rxpacket_parses_valid_packet():
    packet = _build_response_packet(servo_id=1, error=0, params=[42])
    handler = _handler(FakeRxPortHandler(packet))

    rxpacket, result = handler.rxPacket()

    assert result == COMM_SUCCESS
    assert rxpacket[PKT_ID] == 1
    assert rxpacket[PKT_ERROR] == 0
    assert rxpacket[5] == 42


def test_rxpacket_detects_corrupt_checksum():
    packet = bytearray(_build_response_packet(servo_id=1, error=0, params=[42]))
    packet[-1] ^= 0xFF  # flip the checksum byte
    handler = _handler(FakeRxPortHandler(bytes(packet)))

    _rxpacket, result = handler.rxPacket()

    assert result == COMM_RX_CORRUPT


def test_rxpacket_resyncs_past_leading_garbage_byte():
    packet = _build_response_packet(servo_id=1, error=0, params=[42])
    handler = _handler(FakeRxPortHandler(b"\x00" + packet))

    rxpacket, result = handler.rxPacket()

    assert result == COMM_SUCCESS
    assert rxpacket[0:2] == [0xFF, 0xFF]
    assert rxpacket[PKT_ID] == 1


def test_rxpacket_times_out_on_empty_buffer():
    handler = _handler(FakeRxPortHandler(b""))

    _rxpacket, result = handler.rxPacket()

    assert result == COMM_RX_TIMEOUT
