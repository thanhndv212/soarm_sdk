"""The manual ROM-recording controls in the Homing Wizard, without Viser.

Covers the per-joint Reset button: clearing a bad Record Min/Max should not
require redoing a good one on the same joint, and must never touch another
joint's row.
"""

from __future__ import annotations

import contextlib
import threading
import types


class _Handle:
    """Enough of a Viser GUI handle for a button/markdown/dropdown."""

    def __init__(self, **kw):
        self.content = ""
        self.value = kw.get("initial_value", 0.0)

    def on_click(self, fn):
        self._click = fn
        return fn

    def on_update(self, fn):
        self._update = fn
        return fn

    def click(self):
        self._click(None)


class _Folder:
    def __init__(self, name, log):
        log.append(name)

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        return False


class _Gui:
    """Records every button by label and every markdown by creation order,
    so a test can find a handle without the panel exposing one directly."""

    def __init__(self):
        self.folders: list = []
        self.buttons: dict = {}
        self.markdowns: list = []

    def add_markdown(self, x=""):
        h = _Handle()
        h.content = x
        self.markdowns.append(h)
        return h

    def add_folder(self, name, **_kw):
        return _Folder(name, self.folders)

    def add_number(self, _name, **kw):
        return _Handle(**kw)

    def add_text(self, _name, **kw):
        h = _Handle(**kw)
        h.value = kw.get("initial_value", "")
        return h

    def add_button(self, name, **_kw):
        h = _Handle()
        self.buttons[name] = h
        return h

    def add_slider(self, _name, **kw):
        return _Handle(**kw)

    def add_dropdown(self, _name, options=None, initial_value=None):
        h = _Handle()
        h.value = initial_value
        return h

    def add_checkbox(self, _name, **kw):
        h = _Handle(**kw)
        h.value = kw.get("initial_value", False)
        return h


class _Server:
    def __init__(self):
        self.gui = _Gui()


class _FakeBus:
    """ReadPos returns a distinct, deterministic tick per servo id."""

    def ReadPos(self, sid):
        from soarm_sdk.protocol.registers import COMM_SUCCESS

        return (1000 + sid, COMM_SUCCESS, None)


def _build(joint_ids=(1, 2)):
    from soarm_sdk.dashboard.panels import setup as S

    handles: dict = {}
    ctx = types.SimpleNamespace(
        joint_ids=list(joint_ids),
        parse_ids=lambda s: list(joint_ids),
        lock=threading.Lock(),
        state=types.SimpleNamespace(positions={}, connected=False),
        calibration=None,
    )
    ctx.bus = lambda *a, **k: contextlib.nullcontext(_FakeBus())

    server = _Server()
    S._build_homing(server, ctx, heading=False, handles=handles)
    return server, ctx, handles


def _live_table_handle(server):
    """The man_live_md handle, found by its unique startup content."""
    return next(
        h for h in server.gui.markdowns
        if h.content == "*Waiting for a servo reading…*"
    )


def _row(text: str, label: str) -> str:
    return next(ln for ln in text.split("\n") if ln.startswith(f"| {label}"))


def test_reset_button_exists_for_every_joint():
    server, _ctx, _handles = _build(joint_ids=(1, 2, 3))
    for sid in (1, 2, 3):
        assert f"Reset J{sid}" in server.gui.buttons


def test_reset_clears_both_min_and_max_for_that_joint():
    server, ctx, handles = _build(joint_ids=(1, 2))
    live = _live_table_handle(server)

    server.gui.buttons["Record Min J1"].click()
    server.gui.buttons["Record Max J1"].click()
    handles["homing_tick"](ctx)
    before = _row(live.content, "J1")
    assert "1001" in before  # recorded, not "—"

    server.gui.buttons["Reset J1"].click()
    handles["homing_tick"](ctx)
    after = _row(live.content, "J1")
    assert "| — | — | — |" in after


def test_reset_touches_only_its_own_joint():
    server, ctx, handles = _build(joint_ids=(1, 2))
    live = _live_table_handle(server)

    server.gui.buttons["Record Min J1"].click()
    server.gui.buttons["Record Max J1"].click()
    server.gui.buttons["Record Min J2"].click()
    server.gui.buttons["Record Max J2"].click()
    handles["homing_tick"](ctx)

    server.gui.buttons["Reset J1"].click()
    handles["homing_tick"](ctx)

    assert "| — | — | — |" in _row(live.content, "J1")
    j2_min = _row(live.content, "J2").split("|")[4].strip()
    assert j2_min == "1002"  # unaffected by J1's reset


def test_resetting_a_joint_with_nothing_recorded_is_a_no_op():
    """Clicking Reset before ever recording must not raise."""
    server, ctx, handles = _build(joint_ids=(1,))
    server.gui.buttons["Reset J1"].click()
    handles["homing_tick"](ctx)  # must not raise


def test_reset_after_only_one_end_was_recorded_clears_that_end_too():
    server, ctx, handles = _build(joint_ids=(1,))
    live = _live_table_handle(server)

    def min_col(row: str) -> str:
        return row.split("|")[4].strip()  # | Joint | Live | Angle | Min | ...

    server.gui.buttons["Record Min J1"].click()
    handles["homing_tick"](ctx)
    assert min_col(_row(live.content, "J1")) == "1001"

    server.gui.buttons["Reset J1"].click()
    handles["homing_tick"](ctx)
    assert min_col(_row(live.content, "J1")) == "—"


def test_reset_is_visible_only_in_manual_mode():
    """Grouped with Record Min/Max in _man_controls, so mode-switching
    toggles it the same way — it must not linger visible during an
    automatic sweep."""
    import inspect

    from soarm_sdk.dashboard.panels import setup as S

    src = inspect.getsource(S._build_homing)
    i = src.index("_man_controls = [")
    j = src.index("]", i)
    block = src[i:j]
    assert "man_reset_btns" in block


# -- buttons sit next to the joint they act on, not in a block below -----


def test_each_joints_row_sits_directly_above_its_own_three_buttons():
    """Was: one combined min/max table, then eighteen buttons in a block
    below it with nothing but a servo-id number to say which joint each one
    belongs to. Checked from source, since the mock GUI doesn't preserve
    cross-widget-type creation order."""
    import inspect

    from soarm_sdk.dashboard.panels import setup as S

    src = inspect.getsource(S._build_homing)
    i = src.index("for sid, name in zip(ctx.joint_ids, SOARM100_JOINT_NAMES):\n        man_row_mds")
    j = src.index("man_compute_btn = server.gui.add_button")
    block = src[i:j]
    # Each joint's label is created, then its three buttons, in one loop —
    # not a table followed by a separate loop over every joint again.
    assert block.index("man_row_mds[sid]") < block.index("man_min_btns[sid]")
    assert block.index("man_min_btns[sid]") < block.index("man_max_btns[sid]")
    assert block.index("man_max_btns[sid]") < block.index("man_reset_btns[sid]")


def test_the_row_label_updates_with_the_joint_name_and_current_values():
    server, ctx, handles = _build(joint_ids=(1,))
    row = next(h for h in server.gui.markdowns if h.content.startswith("**J1"))
    assert "shoulder_pan" in row.content
    assert "min —, max —" in row.content

    server.gui.buttons["Record Min J1"].click()
    assert "min 1001" in row.content
    assert "max —" in row.content


def test_resetting_reverts_the_row_label_to_unrecorded():
    server, _ctx, _handles = _build(joint_ids=(1,))
    row = next(h for h in server.gui.markdowns if h.content.startswith("**J1"))
    server.gui.buttons["Record Min J1"].click()
    server.gui.buttons["Record Max J1"].click()
    server.gui.buttons["Reset J1"].click()
    assert row.content.endswith("min —, max —")


def test_the_combined_min_max_table_is_gone():
    """It duplicated man_live_md's own columns with no extra information."""
    import inspect

    from soarm_sdk.dashboard.panels import setup as S

    src = inspect.getsource(S._build_homing)
    assert "| Joint | Min | Max |" not in src


def test_man_status_md_is_now_freeform_not_a_table():
    server, _ctx, _handles = _build(joint_ids=(1,))
    assert server.gui.buttons  # sanity: build succeeded
    # man_status_md starts empty; nothing writes a table into it any more.
    empties = [h for h in server.gui.markdowns if h.content == ""]
    assert empties, "expected an empty freeform status placeholder"


def test_row_labels_are_hidden_during_automatic_sweep_mode():
    import inspect

    from soarm_sdk.dashboard.panels import setup as S

    src = inspect.getsource(S._build_homing)
    i = src.index("_man_controls = [")
    j = src.index("]", i)
    assert "man_row_mds" in src[i:j]
