"""Synchronized write helper for STServo devices."""

from __future__ import annotations

from typing import Dict, List, Sequence

from .stservo_def import COMM_NOT_AVAILABLE


class GroupSyncWrite:
    """Collects data for a broadcast sync-write instruction."""

    def __init__(self, handler, start_address: int, data_length: int) -> None:
        self.handler = handler
        self.start_address = start_address
        self.data_length = data_length

        self.is_param_changed = False
        self.param: List[int] = []
        self.data_dict: Dict[int, List[int]] = {}

    def makeParam(self) -> None:  # noqa: N802
        if not self.data_dict:
            return

        params: List[int] = []
        for servo_id, data in self.data_dict.items():
            if not data:
                return
            params.append(servo_id)
            params.extend(data)
        self.param = params
        self.is_param_changed = False

    def addParam(  # noqa: N802
        self, servo_id: int, data: Sequence[int]
    ) -> bool:
        if servo_id in self.data_dict:
            return False
        if len(data) > self.data_length:
            return False
        self.data_dict[servo_id] = list(data)
        self.is_param_changed = True
        return True

    def removeParam(self, servo_id: int) -> None:  # noqa: N802
        if servo_id not in self.data_dict:
            return
        del self.data_dict[servo_id]
        self.is_param_changed = True

    def changeParam(  # noqa: N802
        self, servo_id: int, data: Sequence[int]
    ) -> bool:
        if servo_id not in self.data_dict:
            return False
        if len(data) > self.data_length:
            return False
        self.data_dict[servo_id] = list(data)
        self.is_param_changed = True
        return True

    def clearParam(self) -> None:  # noqa: N802
        self.data_dict.clear()
        self.param.clear()
        self.is_param_changed = False

    def txPacket(self) -> int:  # noqa: N802
        servo_count = len(self.data_dict)
        if servo_count == 0:
            return COMM_NOT_AVAILABLE

        if self.is_param_changed or not self.param:
            self.makeParam()

        param_length = servo_count * (1 + self.data_length)
        return self.handler.syncWriteTxOnly(
            self.start_address,
            self.data_length,
            self.param,
            param_length,
        )
