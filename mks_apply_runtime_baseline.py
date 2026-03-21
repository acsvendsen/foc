#!/usr/bin/env python3
"""Apply the current MKS runtime baseline on official ODrive v0.5.6 firmware.

This is intentionally not a production motion profile writer. It only applies
the current defensible runtime normalization and, optionally, preloads the best
current bare-motor direct-position candidate for manual testing.
"""

from __future__ import annotations

import argparse
import json

import odrive

import common
from mks_axis_characterize import _axis_snapshot, apply_mks_runtime_baseline


BARE_POS_V1 = {
    "current_lim": 2.75,
    "pos_gain": 4.75,
    "vel_gain": 0.10,
    "vel_i_gain": 0.02,
    "vel_limit": 0.45,
}

BARE_POS_FAST1 = {
    "current_lim": 2.75,
    "pos_gain": 4.75,
    "vel_gain": 0.10,
    "vel_i_gain": 0.02,
    "vel_limit": 0.50,
}


def apply_bare_pos_v1(axis) -> None:
    axis.motor.config.current_lim = float(BARE_POS_V1["current_lim"])
    axis.controller.config.pos_gain = float(BARE_POS_V1["pos_gain"])
    axis.controller.config.vel_gain = float(BARE_POS_V1["vel_gain"])
    axis.controller.config.vel_integrator_gain = float(BARE_POS_V1["vel_i_gain"])
    axis.controller.config.vel_limit = float(BARE_POS_V1["vel_limit"])
    axis.controller.config.enable_overspeed_error = False
    axis.controller.config.vel_limit_tolerance = 4.0


def apply_bare_pos_fast1(axis) -> None:
    axis.motor.config.current_lim = float(BARE_POS_FAST1["current_lim"])
    axis.controller.config.pos_gain = float(BARE_POS_FAST1["pos_gain"])
    axis.controller.config.vel_gain = float(BARE_POS_FAST1["vel_gain"])
    axis.controller.config.vel_integrator_gain = float(BARE_POS_FAST1["vel_i_gain"])
    axis.controller.config.vel_limit = float(BARE_POS_FAST1["vel_limit"])
    axis.controller.config.enable_overspeed_error = False
    axis.controller.config.vel_limit_tolerance = 4.0


def run(serial_number: str, axis_index: int, preset: str) -> dict:
    odrv = odrive.find_any(serial_number=str(serial_number).strip(), timeout=10.0)
    axis = getattr(odrv, f"axis{int(axis_index)}")

    apply_mks_runtime_baseline(axis, odrv)

    if preset in ("direct-c1", "bare-pos-v1"):
        apply_bare_pos_v1(axis)
    elif preset == "bare-pos-fast1":
        apply_bare_pos_fast1(axis)

    try:
        common.sync_pos_setpoint(axis, settle_s=0.05, retries=3, verbose=False)
    except Exception:
        pass
    common.force_idle(axis, settle_s=0.05)
    common.clear_errors_all(axis)
    try:
        odrv.clear_errors()
    except Exception:
        pass

    return {
        "board_serial": str(getattr(odrv, "serial_number", "")),
        "axis_index": int(axis_index),
        "preset": preset,
        "snapshot": _axis_snapshot(axis, odrv),
        "axis_report": common.get_axis_error_report(axis),
        "notes": [
            "runtime only; does not save to flash",
            "not a validated motion profile",
            "bare-pos-v1 is for bare-motor passthrough experiments only",
            "legacy preset name direct-c1 maps to bare-pos-v1",
            "bare-pos-fast1 is more aggressive and currently less trusted than bare-pos-v1",
        ],
    }


def main() -> None:
    ap = argparse.ArgumentParser(description="Apply the current MKS runtime baseline")
    ap.add_argument("--serial-number", required=True)
    ap.add_argument("--axis-index", type=int, default=0)
    ap.add_argument(
        "--preset",
        choices=["baseline", "direct-c1", "bare-pos-v1", "bare-pos-fast1"],
        default="baseline",
        help="Optional controller preset after the normalized baseline is applied",
    )
    args = ap.parse_args()

    report = run(args.serial_number, args.axis_index, args.preset)
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
