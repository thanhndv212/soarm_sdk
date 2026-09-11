"""Unit tests for soarm_sdk.bus.servo_config.

Covers the pure planning/parsing logic (OperationPlan construction, ID
resolution). Deliberately does not exercise apply_plan()/run_calibration(),
which open a real serial port.

Formerly ``tests/test_calibration.py``, renamed alongside the module's
move from ``soarm_sdk.calibration`` to ``soarm_sdk.bus.servo_config`` —
see that module's docstring for why.
"""

from __future__ import annotations

import argparse

import pytest

from soarm_sdk.bus.servo_config import (
    OperationPlan,
    build_operation_plan,
    collect_final_ids,
    parse_bool_mapping,
    parse_mapping,
    parse_range,
    resolve_id,
)


# ---------------------------------------------------------------------------
# parse_range
# ---------------------------------------------------------------------------


def test_parse_range_valid():
    assert list(parse_range("1-6")) == [1, 2, 3, 4, 5, 6]


def test_parse_range_single_value():
    assert list(parse_range("3-3")) == [3]


@pytest.mark.parametrize("raw", ["abc", "1", "1-2-3", ""])
def test_parse_range_malformed_raises(raw):
    with pytest.raises(ValueError):
        parse_range(raw)


def test_parse_range_start_after_end_raises():
    with pytest.raises(ValueError):
        parse_range("6-1")


def test_parse_range_negative_raises():
    with pytest.raises(ValueError):
        parse_range("-1-5")


# ---------------------------------------------------------------------------
# parse_mapping
# ---------------------------------------------------------------------------


def test_parse_mapping_basic():
    result = parse_mapping(
        ["1:100:200", "2:50:60"], expected_parts=3, label="--test"
    )
    assert result == {1: (100, 200), 2: (50, 60)}


def test_parse_mapping_supports_hex_via_base_zero():
    result = parse_mapping(["1:0x10"], expected_parts=2, label="--test")
    assert result == {1: (16,)}


def test_parse_mapping_wrong_field_count_raises():
    with pytest.raises(ValueError):
        parse_mapping(["1:100"], expected_parts=3, label="--test")


def test_parse_mapping_non_integer_raises():
    with pytest.raises(ValueError):
        parse_mapping(["1:abc"], expected_parts=2, label="--test")


# ---------------------------------------------------------------------------
# parse_bool_mapping
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "state_str,expected",
    [
        ("on", True),
        ("1", True),
        ("true", True),
        ("yes", True),
        ("enable", True),
        ("off", False),
        ("0", False),
        ("false", False),
        ("no", False),
        ("disable", False),
    ],
)
def test_parse_bool_mapping_accepts_known_states(state_str, expected):
    result = parse_bool_mapping([f"11:{state_str}"], "--torque")
    assert result == {11: expected}


def test_parse_bool_mapping_invalid_state_raises():
    with pytest.raises(ValueError):
        parse_bool_mapping(["11:maybe"], "--torque")


def test_parse_bool_mapping_invalid_id_raises():
    with pytest.raises(ValueError):
        parse_bool_mapping(["abc:on"], "--torque")


def test_parse_bool_mapping_wrong_field_count_raises():
    with pytest.raises(ValueError):
        parse_bool_mapping(["11:on:extra"], "--torque")


# ---------------------------------------------------------------------------
# build_operation_plan
# ---------------------------------------------------------------------------


def _make_args(**overrides) -> argparse.Namespace:
    defaults = dict(
        assign_id=[],
        angle_limit=[],
        set_acc=[],
        set_speed=[],
        torque=[],
        set_mode=[],
        set_baud=[],
    )
    defaults.update(overrides)
    return argparse.Namespace(**defaults)


def test_build_operation_plan_empty():
    plan = build_operation_plan(_make_args())
    assert plan == OperationPlan(
        assign_id={},
        angle_limits={},
        acceleration={},
        speed={},
        torque={},
        mode={},
        baud={},
    )


def test_build_operation_plan_populates_all_fields():
    args = _make_args(
        assign_id=["1:10"],
        angle_limit=["10:100:3900"],
        set_acc=["10:50"],
        set_speed=["10:300"],
        torque=["10:on"],
        set_mode=["10:1"],
        set_baud=["10:0"],
    )
    plan = build_operation_plan(args)
    assert plan.assign_id == {1: 10}
    assert plan.angle_limits == {10: (100, 3900)}
    assert plan.acceleration == {10: 50}
    assert plan.speed == {10: 300}
    assert plan.torque == {10: True}
    assert plan.mode == {10: 1}
    assert plan.baud == {10: 0}


# ---------------------------------------------------------------------------
# resolve_id / collect_final_ids
# ---------------------------------------------------------------------------


def test_resolve_id_no_remap_returns_original():
    assert resolve_id({}, 5) == 5


def test_resolve_id_follows_chain():
    remap = {1: 2, 2: 3}
    assert resolve_id(remap, 1) == 3


def test_resolve_id_is_cycle_safe():
    remap = {1: 2, 2: 1}
    # Must terminate rather than looping forever. Walks 1 -> 2 -> 1, then
    # stops because 1 is already in `seen`, returning the last hop taken.
    assert resolve_id(remap, 1) == 1


def test_collect_final_ids_resolves_through_remap():
    plan = OperationPlan(
        assign_id={1: 10},
        angle_limits={1: (0, 100)},
        acceleration={},
        speed={},
        torque={},
        mode={},
        baud={},
    )
    remap = {1: 10}
    assert collect_final_ids(plan, remap) == [10]


def test_collect_final_ids_dedupes_and_sorts():
    plan = OperationPlan(
        assign_id={},
        angle_limits={3: (0, 100)},
        acceleration={1: 50},
        speed={2: 300},
        torque={1: True},
        mode={},
        baud={},
    )
    assert collect_final_ids(plan, {}) == [1, 2, 3]
