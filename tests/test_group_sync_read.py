"""Unit tests for GroupSyncRead availability and field extraction.

The parser hands each servo's response over as ``[error_byte, *data]``.
``isAvailable`` is the guard callers use before ``getData``, so it has to
answer for the *requested field*, not merely for the response as a whole.
"""

from __future__ import annotations

from soarm_sdk.protocol.group_sync_read import GroupSyncRead
from soarm_sdk.protocol.sts import sts


START = 56
LENGTH = 15


def _gsr(snapshot=None, sid=1):
    group = GroupSyncRead(sts(None), START, LENGTH)
    group.addParam(sid)
    if snapshot is not None:
        group.data_dict[sid] = list(snapshot)
    return group


def test_field_at_the_front_is_available_in_a_complete_block():
    group = _gsr([0] + list(range(LENGTH)))
    assert group.isAvailable(1, START, 2) == (True, 0)


def test_field_at_the_far_end_is_available_in_a_complete_block():
    group = _gsr([0] + list(range(LENGTH)))
    assert group.isAvailable(1, START + LENGTH - 2, 2)[0] is True


def test_field_behind_a_truncated_response_is_not_available():
    # Six bytes of data: enough for the field at the front, not for one at the
    # end. Reporting this available let getData walk off the end of the list.
    group = _gsr([0] + [0] * 6)

    assert group.isAvailable(1, START, 2)[0] is True
    assert group.isAvailable(1, START + LENGTH - 2, 2)[0] is False


def test_empty_snapshot_is_never_available():
    group = _gsr([])
    assert group.isAvailable(1, START, 2)[0] is False


def test_unknown_servo_is_never_available():
    group = _gsr([0] + list(range(LENGTH)))
    assert group.isAvailable(99, START, 2)[0] is False


def test_address_below_the_block_start_is_never_available():
    group = _gsr([0] + list(range(LENGTH)))
    assert group.isAvailable(1, START - 1, 2)[0] is False


def test_address_past_the_block_end_is_never_available():
    group = _gsr([0] + list(range(LENGTH)))
    assert group.isAvailable(1, START + LENGTH, 1)[0] is False


def test_get_data_reads_the_requested_field_from_its_offset():
    # data byte i holds value i, so the word at START+2 is (2, 3) little-endian.
    group = _gsr([0] + list(range(LENGTH)))
    assert group.getData(1, START + 2, 2) == (3 << 8) | 2
    assert group.getData(1, START + 6, 1) == 6
