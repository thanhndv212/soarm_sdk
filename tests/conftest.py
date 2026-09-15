"""Session-wide test safety net: nothing here may touch real hardware state.

A test overwrote the operator's real ``~/.soarm_sdk/calibration.json`` with a
fixture's blank data on 2026-09-14. ``_do_save`` used to refuse before ever
computing a path when a calibration was incomplete, which accidentally kept
``_cal_path``'s fallback to ``DEFAULT_CALIBRATION_PATH`` unreachable from a
test that forgot to set ``ctx.calibration_path``. Removing that refusal (so
an incomplete-but-genuine calibration can still be saved) removed the
accidental guard along with it, and the very next test run reached the real
file with garbage.

The fix is not "remember to set calibration_path in every test" — that is
exactly the discipline that just failed, once, for real. Every test in this
suite gets ``~/.soarm_sdk/calibration.json`` (and the other module that
default-writes to the same path) redirected to an isolated tmp directory
automatically, whether the test knows to ask for it or not.
"""

from __future__ import annotations

import pytest


@pytest.fixture(autouse=True)
def _no_real_calibration_file(tmp_path, monkeypatch):
    """Redirect every default calibration path to this test's own tmp_path.

    Two modules independently default to
    ``Path.home() / ".soarm_sdk" / "calibration.json"`` — the same
    duplicated-convention pattern this codebase has already been bitten by
    once for direction signs. Both are patched here so a test that
    relies on either lands in ``tmp_path``, never in the real file.
    """
    fake = tmp_path / "calibration.json"
    monkeypatch.setattr("soarm_sdk.dashboard.fk.DEFAULT_CALIBRATION_PATH", fake)
    monkeypatch.setattr("soarm_sdk.calibration.sweep_cli.DEFAULT_OUT", fake)
    yield fake
