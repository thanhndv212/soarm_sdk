"""Synchronized read helper for STServo devices."""

from __future__ import annotations

from typing import Dict, List, Tuple

from .registers import COMM_NOT_AVAILABLE, COMM_RX_CORRUPT, COMM_SUCCESS


class GroupSyncRead:
    """Manage sync-read requests and cached responses."""

    def __init__(self, handler, start_address: int, data_length: int) -> None:
        self.handler = handler
        self.start_address = start_address
        self.data_length = data_length

        self.last_result = False
        self.is_param_changed = False
        self.param: List[int] = []
        self.data_dict: Dict[int, List[int]] = {}

    def makeParam(self) -> None:  # noqa: N802
        if not self.data_dict:
            return
        self.param = list(self.data_dict.keys())
        self.is_param_changed = False

    def addParam(self, servo_id: int) -> bool:  # noqa: N802
        if servo_id in self.data_dict:
            return False
        self.data_dict[servo_id] = []
        self.is_param_changed = True
        return True

    def removeParam(self, servo_id: int) -> None:  # noqa: N802
        if servo_id not in self.data_dict:
            return
        del self.data_dict[servo_id]
        self.is_param_changed = True

    def clearParam(self) -> None:  # noqa: N802
        self.data_dict.clear()
        self.param.clear()
        self.last_result = False
        self.is_param_changed = False

    def txPacket(self) -> int:  # noqa: N802
        if not self.data_dict:
            return COMM_NOT_AVAILABLE
        if self.is_param_changed or not self.param:
            self.makeParam()
        return self.handler.syncReadTx(
            self.start_address,
            self.data_length,
            self.param,
            len(self.data_dict),
        )

    def rxPacket(self) -> int:  # noqa: N802
        if not self.data_dict:
            return COMM_NOT_AVAILABLE

        result, raw = self.handler.syncReadRx(
            self.data_length, len(self.data_dict)
        )
        if result != COMM_SUCCESS:
            self.last_result = False
            return result

        self.last_result = True
        for servo_id in self.data_dict:
            parsed, sub_result = self._readRx(raw, servo_id, self.data_length)
            if sub_result != COMM_SUCCESS:
                self.last_result = False
                self.data_dict[servo_id] = []
            else:
                self.data_dict[servo_id] = parsed
        return result

    def txRxPacket(self) -> int:  # noqa: N802
        result = self.txPacket()
        if result != COMM_SUCCESS:
            return result
        return self.rxPacket()

    def _readRx(
        self, raw: List[int], servo_id: int, data_length: int
    ) -> Tuple[List[int], int]:
        data: List[int] = []
        rx_length = len(raw)
        rx_index = 0
        while (rx_index + 6 + data_length) <= rx_length:
            head = [0x00, 0x00, 0x00]
            while rx_index < rx_length:
                head[2] = head[1]
                head[1] = head[0]
                head[0] = raw[rx_index]
                rx_index += 1
                if head[2] == 0xFF and head[1] == 0xFF and head[0] == servo_id:
                    break
            if (rx_index + 3 + data_length) > rx_length:
                break
            if raw[rx_index] != (data_length + 2):
                rx_index += 1
                continue
            rx_index += 1
            error = raw[rx_index]
            rx_index += 1
            checksum = servo_id + (data_length + 2) + error
            data = [error]
            data.extend(raw[rx_index : rx_index + data_length])
            for _ in range(data_length):
                checksum += raw[rx_index]
                rx_index += 1
            checksum = (~checksum) & 0xFF
            if checksum != raw[rx_index]:
                return [], COMM_RX_CORRUPT
            return data, COMM_SUCCESS
        return [], COMM_RX_CORRUPT

    def isAvailable(
        self, servo_id: int, address: int, data_length: int
    ) -> Tuple[bool, int]:  # noqa: N802
        if servo_id not in self.data_dict:
            return False, 0
        if address < self.start_address:
            return False, 0
        upper = self.start_address + self.data_length - data_length
        if upper < address:
            return False, 0
        snapshot = self.data_dict[servo_id]
        # snapshot is ``[error_byte, *data]``, so the requested field occupies
        # ``snapshot[offset : offset + data_length]``. Checking only the total
        # length (the previous behaviour) answers correctly for a field at the
        # front of the block and wrongly for every field behind it — which was
        # harmless while the only sync-read group was 4 bytes wide, and is not
        # once a caller reads a field at the far end of a longer block.
        offset = address - self.start_address + 1
        if len(snapshot) < offset + data_length:
            return False, 0
        return True, snapshot[0]

    def getData(  # noqa: N802
        self, servo_id: int, address: int, data_length: int
    ) -> int:
        offset = address - self.start_address + 1
        snapshot = self.data_dict[servo_id]
        if data_length == 1:
            return snapshot[offset]
        if data_length == 2:
            return self.handler.sts_makeword(
                snapshot[offset], snapshot[offset + 1]
            )
        if data_length == 4:
            return self.handler.sts_makedword(
                self.handler.sts_makeword(
                    snapshot[offset], snapshot[offset + 1]
                ),
                self.handler.sts_makeword(
                    snapshot[offset + 2], snapshot[offset + 3]
                ),
            )
        return 0
