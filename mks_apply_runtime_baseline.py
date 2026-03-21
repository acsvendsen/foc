#!/usr/bin/env python3
"""Apply the current MKS runtime baseline on official ODrive v0.5.6 firmware.

This is intentionally not a production motion profile writer. It applies the
runtime normalization that currently makes the MKS board behave coherently, and
optionally applies a preset plus manual CLI overrides for bare-motor testing.
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


def apply_overrides(
    axis,
    odrv,
    *,
    encoder_bandwidth=None,
    current_control_bandwidth=None,
    current_lim=None,
    pos_gain=None,
    vel_gain=None,
    vel_i_gain=None,
    vel_limit=None,
    overspeed_error=None,
    vel_limit_tolerance=None,
    dc_max_negative_current=None,
):
    if encoder_bandwidth is not None:
        axis.encoder.config.bandwidth = float(encoder_bandwidth)
    if current_control_bandwidth is not None:
        axis.motor.config.current_control_bandwidth = float(current_control_bandwidth)
    if current_lim is not None:
        axis.motor.config.current_lim = float(current_lim)
    if pos_gain is not None:
        axis.controller.config.pos_gain = float(pos_gain)
    if vel_gain is not None:
        axis.controller.config.vel_gain = float(vel_gain)
    if vel_i_gain is not None:
        axis.controller.config.vel_integrator_gain = float(vel_i_gain)
    if vel_limit is not None:
        axis.controller.config.vel_limit = float(vel_limit)
    if overspeed_error is not None:
        axis.controller.config.enable_overspeed_error = bool(overspeed_error)
    if vel_limit_tolerance is not None:
        axis.controller.config.vel_limit_tolerance = float(vel_limit_tolerance)
    if dc_max_negative_current is not None:
        odrv.config.dc_max_negative_current = float(dc_max_negative_current)


def apply_runtime_baseline(
    serial_number: str,
    axis_index: int,
    preset: str,
    *,
    encoder_bandwidth=None,
    current_control_bandwidth=None,
    current_lim=None,
    pos_gain=None,
    vel_gain=None,
    vel_i_gain=None,
    vel_limit=None,
    overspeed_error=None,
    vel_limit_tolerance=None,
    dc_max_negative_current=None,
) -> dict:
    odrv = odrive.find_any(serial_number=str(serial_number).strip(), timeout=10.0)
    axis = getattr(odrv, f"axis{int(axis_index)}")

    apply_mks_runtime_baseline(axis, odrv)

    if preset in ("direct-c1", "bare-pos-v1"):
        apply_bare_pos_v1(axis)
    elif preset == "bare-pos-fast1":
        apply_bare_pos_fast1(axis)

    apply_overrides(
        axis,
        odrv,
        encoder_bandwidth=encoder_bandwidth,
        current_control_bandwidth=current_control_bandwidth,
        current_lim=current_lim,
        pos_gain=pos_gain,
        vel_gain=vel_gain,
        vel_i_gain=vel_i_gain,
        vel_limit=vel_limit,
        overspeed_error=overspeed_error,
        vel_limit_tolerance=vel_limit_tolerance,
        dc_max_negative_current=dc_max_negative_current,
    )

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
        "applied_overrides": {
            "encoder_bandwidth": encoder_bandwidth,
            "current_control_bandwidth": current_control_bandwidth,
            "current_lim": current_lim,
            "pos_gain": pos_gain,
            "vel_gain": vel_gain,
            "vel_i_gain": vel_i_gain,
            "vel_limit": vel_limit,
            "overspeed_error": overspeed_error,
            "vel_limit_tolerance": vel_limit_tolerance,
            "dc_max_negative_current": dc_max_negative_current,
        },
        "notes": [
            "runtime only; does not save to flash",
            "not a validated motion profile",
            "bare-pos-v1 is for bare-motor passthrough experiments only",
            "legacy preset name direct-c1 maps to bare-pos-v1",
            "bare-pos-fast1 is more aggressive and currently less trusted than bare-pos-v1",
        ],
    }


def run(
    serial_number: str,
    axis_index: int,
    preset: str,
    *,
    encoder_bandwidth=None,
    current_control_bandwidth=None,
    current_lim=None,
    pos_gain=None,
    vel_gain=None,
    vel_i_gain=None,
    vel_limit=None,
    overspeed_error=None,
    vel_limit_tolerance=None,
    dc_max_negative_current=None,
) -> dict:
    """Backward-compatible alias for apply_runtime_baseline()."""
    return apply_runtime_baseline(
        serial_number,
        axis_index,
        preset,
        encoder_bandwidth=encoder_bandwidth,
        current_control_bandwidth=current_control_bandwidth,
        current_lim=current_lim,
        pos_gain=pos_gain,
        vel_gain=vel_gain,
        vel_i_gain=vel_i_gain,
        vel_limit=vel_limit,
        overspeed_error=overspeed_error,
        vel_limit_tolerance=vel_limit_tolerance,
        dc_max_negative_current=dc_max_negative_current,
    )


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
    ap.add_argument("--encoder-bandwidth", type=float, default=None)
    ap.add_argument("--current-control-bandwidth", type=float, default=None)
    ap.add_argument("--current-lim", type=float, default=None)
    ap.add_argument("--pos-gain", type=float, default=None)
    ap.add_argument("--vel-gain", type=float, default=None)
    ap.add_argument("--vel-i-gain", type=float, default=None)
    ap.add_argument("--vel-limit", type=float, default=None)
    ap.add_argument("--overspeed-error", choices=["true", "false"], default=None)
    ap.add_argument("--vel-limit-tolerance", type=float, default=None)
    ap.add_argument("--dc-max-negative-current", type=float, default=None)
    args = ap.parse_args()

    overspeed_error = None
    if args.overspeed_error is not None:
        overspeed_error = (str(args.overspeed_error).strip().lower() == "true")

    report = apply_runtime_baseline(
        args.serial_number,
        args.axis_index,
        args.preset,
        encoder_bandwidth=args.encoder_bandwidth,
        current_control_bandwidth=args.current_control_bandwidth,
        current_lim=args.current_lim,
        pos_gain=args.pos_gain,
        vel_gain=args.vel_gain,
        vel_i_gain=args.vel_i_gain,
        vel_limit=args.vel_limit,
        overspeed_error=overspeed_error,
        vel_limit_tolerance=args.vel_limit_tolerance,
        dc_max_negative_current=args.dc_max_negative_current,
    )
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
