"""Tests for the homing-offset re-centring arithmetic.

The hardware half is a serial conversation; what is testable — and what
actually goes wrong — is the sign-magnitude encoding and the wrap handling
that makes a discontinuous position readable as travel.
"""

from __future__ import annotations

import pytest

from soarm_sdk.calibration.recentre import (
    ENCODER_TICKS,
    _unwrap,
    decode_ofs,
    encode_ofs,
)


@pytest.mark.parametrize(
    "raw, signed",
    [
        (2187, -139),   # shoulder_pan, as read off the arm
        (969, 969),     # shoulder_lift
        (2928, -880),   # elbow_flex
        (147, 147),     # wrist_roll
        (2486, -438),   # gripper
        (0, 0),
    ],
)
def test_decode_matches_the_servos_encoding(raw, signed):
    assert decode_ofs(raw) == signed


@pytest.mark.parametrize("value", [0, 1, -1, 139, -139, 969, -2047, 2047])
def test_encode_decode_round_trips(value):
    assert decode_ofs(encode_ofs(value)) == value


def test_out_of_range_offsets_fold_rather_than_truncate():
    """The offset is modular, so a full turn is a no-op — not an overflow."""
    assert decode_ofs(encode_ofs(100 + ENCODER_TICKS)) == 100
    # 3000 ticks forward is the same frame as 1096 back.
    assert decode_ofs(encode_ofs(3000)) == 3000 - ENCODER_TICKS


def test_unwrap_reads_a_boundary_jump_as_small_motion():
    # 4090 -> 5 is +11 ticks of travel, not -4085.
    assert _unwrap(5 - 4090) == 11
    # 5 -> 4090 is -11.
    assert _unwrap(4090 - 5) == -11


def test_unwrap_leaves_ordinary_motion_alone():
    assert _unwrap(37) == 37
    assert _unwrap(-37) == -37


def test_unwrap_is_antisymmetric():
    for delta in (-4000, -2000, -5, 0, 5, 2000, 4000):
        assert _unwrap(delta) == -_unwrap(-delta)


def test_accumulated_unwrap_recovers_travel_across_the_boundary():
    """A steady sweep through 4095/0 must integrate to its true length."""
    reported = [3900, 3990, 4080, 74, 164, 254]  # +90 ticks per sample
    total = sum(_unwrap(b - a) for a, b in zip(reported, reported[1:]))
    assert total == pytest.approx(450)


def test_centring_offset_follows_the_servos_subtractive_convention():
    """reported = raw - STS_OFS, measured on an STS3215 by stepping the register.

    Pinned because the sign fails silently: the wrong one moves a joint's
    frame away from centre by exactly the amount it should have moved it back.
    """
    from soarm_sdk.calibration.recentre import CENTRE_TICKS

    old_ofs, midpoint = 1341, 3753.0
    new_ofs = old_ofs + round(midpoint - CENTRE_TICKS)
    assert new_ofs == 3046
    # Applying it must put the midpoint at 2048: reported = raw - ofs, and the
    # midpoint's raw value is unchanged by the write.
    raw_midpoint = midpoint + old_ofs
    assert raw_midpoint - new_ofs == CENTRE_TICKS


def test_centring_offset_puts_the_swept_midpoint_at_the_reference():
    """The whole point of Apply: after the write, the midpoint reads target_ref."""
    from soarm_sdk.calibration.recentre import centring_offset

    zero, lo_raw, hi_raw = 1500, 500, 2500
    off, lmin, lmax = centring_offset(zero, lo_raw, hi_raw, 2048)
    # reported = raw - off
    assert zero - off == 2048
    assert (lmin, lmax) == (lo_raw - off, hi_raw - off)


def test_centring_offset_sign_is_not_reversed():
    """A midpoint *below* the reference needs a NEGATIVE offset to come up.

    The reversed sign is silent — it moves the frame by twice the error in
    the wrong direction — so it is pinned rather than left to a comment.
    """
    from soarm_sdk.calibration.recentre import centring_offset

    off, _, _ = centring_offset(zero=1500, pos_min=500, pos_max=2500, target_ref=2048)
    assert off == -548
    off, _, _ = centring_offset(zero=2600, pos_min=1600, pos_max=3600, target_ref=2048)
    assert off == 552


def test_centring_offset_clamps_limits_into_the_encoder():
    from soarm_sdk.calibration.recentre import centring_offset

    _, lmin, lmax = centring_offset(zero=2048, pos_min=-500, pos_max=5000)
    assert (lmin, lmax) == (0, 4095)
