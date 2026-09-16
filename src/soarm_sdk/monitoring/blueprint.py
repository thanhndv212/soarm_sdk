"""Rerun blueprint layouts for live SO-101 servo telemetry.

Two presets over the same entity-path convention
:class:`~soarm_sdk.robot.telemetry_sinks.RerunSink` already writes to
(``{prefix}/{joint_name}/{field}``):

- :func:`build_by_servo_blueprint` — one panel per servo (6 panels), each
  defaulted to one channel. Rerun has no runtime dropdown widget bound to
  app code; the real equivalent it ships is each ``TimeSeriesView``'s own
  entity/query editor in the Selection Panel — select a panel, edit its
  entity filter, and that swaps which channel *that panel* plots,
  independently of the other five. That editor is the "dropdown" here.
- :func:`build_by_channel_blueprint` — one tab per servo, each tab a grid
  of one panel per telemetry channel for that servo. Clicking a tab *is*
  "select one servo": Tabs is Rerun's native selector control, so this
  preset needs no custom widget at all.

:func:`build_monitor_blueprint` nests both under one top-level ``Tabs`` so
switching preset is also just a click, no relaunch.
"""

from __future__ import annotations

from typing import Sequence, Tuple

import rerun.blueprint as rrb

__all__ = [
    "DEFAULT_FIELDS",
    "build_by_servo_blueprint",
    "build_by_channel_blueprint",
    "build_monitor_blueprint",
]

#: (entity-path field name, human label). Matches every field
#: :class:`RerunSink` logs unconditionally, except the goal/tracking-error
#: pair — those only exist once a command has been sent, and this monitor
#: is passive by design (see ``cli/monitor.py``: ``torque_on_start=False``),
#: so panels for them would sit permanently empty.
DEFAULT_FIELDS: Tuple[Tuple[str, str], ...] = (
    ("position_rad", "Position (rad)"),
    ("velocity_rad_s", "Velocity (rad/s)"),
    ("current_mA", "Current (mA)"),
    ("load_percent", "Load (%)"),
    ("temperature_C", "Temperature (C)"),
    ("voltage_V", "Voltage (V)"),
)


def build_by_servo_blueprint(
    joint_names: Sequence[str],
    *,
    prefix: str = "soarm",
    default_field: str = "position_rad",
    grid_columns: int = 3,
) -> rrb.Grid:
    """One panel per servo, each defaulted to *default_field*."""
    views = [
        rrb.TimeSeriesView(
            origin=f"{prefix}/{name}",
            contents=f"$origin/{default_field}",
            name=name,
        )
        for name in joint_names
    ]
    return rrb.Grid(*views, grid_columns=grid_columns, name="By servo")


def build_by_channel_blueprint(
    joint_names: Sequence[str],
    *,
    prefix: str = "soarm",
    fields: Sequence[Tuple[str, str]] = DEFAULT_FIELDS,
    grid_columns: int = 3,
) -> rrb.Tabs:
    """One tab per servo; each tab is a grid with one panel per channel."""
    tabs = [
        rrb.Grid(
            *[
                rrb.TimeSeriesView(
                    origin=f"{prefix}/{name}",
                    contents=f"$origin/{field}",
                    name=label,
                )
                for field, label in fields
            ],
            grid_columns=grid_columns,
            name=name,
        )
        for name in joint_names
    ]
    return rrb.Tabs(*tabs, name="By telemetry (one servo)")


def build_monitor_blueprint(
    joint_names: Sequence[str],
    *,
    prefix: str = "soarm",
    fields: Sequence[Tuple[str, str]] = DEFAULT_FIELDS,
    default_field: str = "position_rad",
    grid_columns: int = 3,
) -> rrb.Blueprint:
    """Both presets, switchable from one top-level tab bar."""
    root = rrb.Tabs(
        build_by_servo_blueprint(
            joint_names,
            prefix=prefix,
            default_field=default_field,
            grid_columns=grid_columns,
        ),
        build_by_channel_blueprint(
            joint_names,
            prefix=prefix,
            fields=fields,
            grid_columns=grid_columns,
        ),
        name="soarm monitor",
    )
    return rrb.Blueprint(root, collapse_panels=False)
