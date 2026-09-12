#!/usr/bin/env python
"""Run a calibration CLI from a source checkout, without installing first.

    python examples/calibrate.py bus --scan-range 1-6 --ui
    python examples/calibrate.py rom --arm-id thanh_arm

Two jobs live under the word "calibrate", and they are not variants of
each other — which is exactly why they are named modes here rather than
two look-alike launcher scripts a reader has to open to tell apart:

  bus   Servo setup on the wire. Discover servos, assign IDs, set angle
        limits, speed, acceleration, torque, mode and baud rate in their
        EEPROM. Nothing to do with the URDF. Installed as
        ``soarm-calibrate``.

  rom   Drive every joint into both of its mechanical hard stops, measure
        the travel, and write this arm's URDF-frame calibration to
        ``~/.soarm_sdk/calibration.json``. The hard stops are the only
        physical reference a servo and the URDF can both name, which is
        why the calibration is derived from them rather than from a "put
        the arm in the home pose" step. Installed as
        ``soarm-calibrate-rom``.

Neither settles the direction signs — a travel range says how far a joint
moves, not which end is the URDF's lower limit — so ``rom`` writes
``validated: false`` and anything that streams a planned trajectory
should refuse that file until the signs are confirmed on the arm.

There is deliberately no mode for seeding the same calibration offline
from an existing lerobot file: that is ``soarm-seed-calibration``, or
``python -m soarm_sdk.calibration.seed``, whose ``main()`` takes no argv
and so does not forward cleanly through here.

Installing the package (``pip install .``) gives you the console scripts
named above and makes this launcher unnecessary; it exists so the tools
are runnable straight from a checkout, and it puts ``src/`` on the path
to that end.
"""

from __future__ import annotations

import sys
from importlib import import_module
from pathlib import Path

_src_root = Path(__file__).resolve().parents[1] / "src"
if _src_root.is_dir() and str(_src_root) not in sys.path:
    sys.path.insert(0, str(_src_root))

#: mode -> (module providing main(argv), installed console script, one-liner)
MODES: dict[str, tuple[str, str, str]] = {
    "bus": (
        "soarm_sdk.cli.calibrate",
        "soarm-calibrate",
        "servo IDs and EEPROM settings on the wire",
    ),
    "rom": (
        "soarm_sdk.calibration.sweep_cli",
        "soarm-calibrate-rom",
        "measure travel from the hard stops, write the URDF-frame calibration",
    ),
}


def _usage(stream=sys.stdout) -> None:
    print("usage: python examples/calibrate.py <mode> [options]\n", file=stream)
    for name, (_module, script, blurb) in MODES.items():
        print(f"  {name:<5} {blurb}", file=stream)
        print(f"        installed as: {script}", file=stream)
    print(
        "\nPass --help after a mode for its own options, e.g.\n"
        "  python examples/calibrate.py rom --help",
        file=stream,
    )


def main(argv: list[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)

    if not args:
        _usage(sys.stderr)
        return 2
    mode, rest = args[0], args[1:]
    if mode in ("-h", "--help"):
        _usage()
        return 0
    if mode not in MODES:
        print(
            f"unknown mode {mode!r}; expected one of {', '.join(MODES)}\n",
            file=sys.stderr,
        )
        _usage(sys.stderr)
        return 2

    module_name, _script, _blurb = MODES[mode]
    # Imported per mode, not up front: the two pull in different halves of
    # the package, and a mode should not fail to start because the other
    # one's imports are unhappy.
    entry = import_module(module_name).main
    # The sub-CLI builds its own parser, which takes its program name from
    # sys.argv[0] -- left alone, its --help would advertise a command line
    # ("calibrate.py --arm-id ...") that this launcher does not accept,
    # because it drops the mode.
    argv0 = sys.argv[0]
    sys.argv[0] = f"{Path(argv0).name} {mode}"
    try:
        return entry(rest) or 0
    finally:
        sys.argv[0] = argv0


if __name__ == "__main__":
    sys.exit(main())
