#!/usr/bin/env python3
"""Seed a URDF-frame calibration for an arm, offline.

    python -m soarm_sdk.seed_calibration \
        --lerobot ~/.cache/huggingface/lerobot/calibration/robots/so101_follower/thanh_arm.json \
        --out ~/.soarm_sdk/thanh_arm.json

Needs no hardware: it reads the measured travel ranges from an existing
lerobot calibration and combines them with the URDF's joint limits. See
:mod:`soarm_sdk.frame_calibration` for the method and, more importantly,
for why the result is only a seed — the direction signs are assumed, and
the file is written with ``validated: false`` until a physical check
confirms them.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from .frame_calibration import seed_from_lerobot

# so101_new_calib.urdf joint limits, in URDF joint order. These are the
# planner's frame; everything here exists to relate servo ticks to them.
SO101_URDF_LIMITS = {
    "shoulder_pan": (-1.91986, 1.91986),
    "shoulder_lift": (-1.74533, 1.74533),
    "elbow_flex": (-1.69, 1.69),
    "wrist_flex": (-1.65806, 1.65806),
    "wrist_roll": (-2.74385, 2.84121),
    "gripper": (-0.174533, 1.74533),
}

DEFAULT_OUT = Path.home() / ".soarm_sdk" / "calibration.json"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--lerobot", required=True, help="lerobot calibration JSON")
    ap.add_argument("--out", default=str(DEFAULT_OUT), help="where to write")
    ap.add_argument("--arm-id", default=None, help="name for this physical arm")
    args = ap.parse_args()

    cal = seed_from_lerobot(args.lerobot, SO101_URDF_LIMITS, arm_id=args.arm_id)

    print("=" * 72)
    print(f"Seeded URDF-frame calibration for '{cal.arm_id}'")
    print("=" * 72)
    print(f"{'joint':16s}{'zero tick':>11}{'sign':>6}{'seed +/-':>11}{'span ratio':>12}")
    for j in cal.joints:
        flag = "  <- check" if j.suspect else ""
        print(
            f"{j.name:16s}{j.zero_offset_ticks:>11.1f}{j.direction_sign:>+6d}"
            f"{j.seed_residual_rad:>10.3f}r{j.span_ratio:>12.3f}{flag}"
        )

    path = cal.save(args.out)
    print("-" * 72)
    print(f"worst seed uncertainty : {cal.worst_seed_residual_rad:.3f} rad")
    if cal.suspect_joints:
        print(f"span mismatch          : {', '.join(cal.suspect_joints)}")
        print("  measured travel and the URDF disagree about range on these.")
        print("  The gearing is fixed, so the URDF's limits are the approximate")
        print("  side — fine for seeding, not for trusting.")
    print(f"written                : {path}")
    print()
    print("validated: FALSE. The direction signs are assumed +1 and cannot be")
    print("recovered from a travel range. Confirm them on the arm, then call")
    print("RobotCalibration.mark_validated() with what you actually did.")
    print("=" * 72)


if __name__ == "__main__":
    sys.exit(main())
