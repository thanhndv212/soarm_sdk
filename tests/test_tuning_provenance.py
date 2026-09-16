"""Unit tests for soarm_sdk.tuning.provenance.

``tests/conftest.py`` redirects ``DEFAULT_PROVENANCE_PATH`` for every test in
this suite, so these tests always pass an explicit tmp_path anyway — belt
and suspenders against ever touching a real ``~/.soarm_sdk`` file.
"""

from __future__ import annotations

from soarm_sdk.tuning.gains_io import Gains
from soarm_sdk.tuning.provenance import TuningRecord, append_record, load_records
from soarm_sdk.tuning.search import SearchResult, TrialResult


def _fake_search_result(validated=True, exhausted=False, trial_count=5):
    best = TrialResult(gains=Gains(p=40, d=20, i=2), metrics=None, report=None, cost=0.5)
    return SearchResult(
        best=best,
        trials=tuple([best] * trial_count),
        consecutive_passes_at_best=3,
        validated=validated,
        exhausted=exhausted,
    )


def test_load_records_empty_when_file_missing(tmp_path):
    assert load_records(tmp_path / "does_not_exist.json") == []


def test_append_and_reload_round_trips(tmp_path):
    path = tmp_path / "pid_tuning.json"
    record = TuningRecord.from_search_result(
        arm_id="arm-1",
        servo_id=3,
        joint_name="elbow_flex",
        algorithm="coordinate_descent",
        before=Gains(p=32, d=32, i=0),
        result=_fake_search_result(),
    )
    append_record(record, path)

    loaded = load_records(path)
    assert len(loaded) == 1
    assert loaded[0].arm_id == "arm-1"
    assert loaded[0].servo_id == 3
    assert loaded[0].joint_name == "elbow_flex"
    assert loaded[0].before == Gains(p=32, d=32, i=0)
    assert loaded[0].after == Gains(p=40, d=20, i=2)
    assert loaded[0].validated is True
    assert loaded[0].trial_count == 5


def test_append_preserves_earlier_records(tmp_path):
    path = tmp_path / "pid_tuning.json"
    first = TuningRecord.from_search_result(
        arm_id="arm-1",
        servo_id=1,
        joint_name="shoulder_pan",
        algorithm="coordinate_descent",
        before=Gains(p=32, d=32, i=0),
        result=_fake_search_result(),
    )
    second = TuningRecord.from_search_result(
        arm_id="arm-1",
        servo_id=2,
        joint_name="shoulder_lift",
        algorithm="coordinate_descent",
        before=Gains(p=32, d=32, i=0),
        result=_fake_search_result(validated=False, exhausted=True),
    )
    append_record(first, path)
    append_record(second, path)

    loaded = load_records(path)
    assert [r.servo_id for r in loaded] == [1, 2]
    assert loaded[1].validated is False
    assert loaded[1].exhausted is True
