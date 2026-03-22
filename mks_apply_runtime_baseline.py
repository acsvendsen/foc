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
from mks_axis_characterize import _axis_snapshot, _calibration_health, apply_mks_runtime_baseline, resolve_odrv_axis


BARE_POS_V1 = {
    "current_lim": 2.75,
    "pos_gain": 4.75,
    "vel_gain": 0.10,
    "vel_i_gain": 0.02,
    "vel_limit": 0.45,
}

BARE_POS_REPEATABLE_V1 = {
    "current_lim": 2.75,
    "pos_gain": 4.75,
    "vel_gain": 0.10,
    "vel_i_gain": 0.02,
    "vel_limit": 0.44,
}

BARE_POS_REPEATABLE_SOFT_V1 = {
    "current_lim": 2.50,
    "pos_gain": 4.75,
    "vel_gain": 0.10,
    "vel_i_gain": 0.02,
    "vel_limit": 0.45,
}

BARE_POS_TRUSTED_V1 = {
    "current_lim": 2.50,
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

MOUNTED_DIRECT_V1 = {
    "current_lim": 6.0,
    "pos_gain": 4.75,
    "vel_gain": 0.18,
    "vel_i_gain": 0.02,
    "vel_limit": 1.0,
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


def _apply_named_candidate(axis, candidate: dict) -> None:
    axis.motor.config.current_lim = float(candidate["current_lim"])
    axis.controller.config.pos_gain = float(candidate["pos_gain"])
    axis.controller.config.vel_gain = float(candidate["vel_gain"])
    axis.controller.config.vel_integrator_gain = float(candidate["vel_i_gain"])
    axis.controller.config.vel_limit = float(candidate["vel_limit"])
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
    serial_number: str | None = None,
    axis_index: int = 0,
    preset: str = "baseline",
    *,
    odrv=None,
    axis=None,
    timeout_s: float = 10.0,
    reuse_existing_calibration: bool = False,
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
    odrv, axis = resolve_odrv_axis(
        odrv=odrv,
        axis=axis,
        serial_number=serial_number,
        axis_index=axis_index,
        timeout_s=timeout_s,
    )

    baseline_result = apply_mks_runtime_baseline(
        axis,
        odrv,
        reuse_existing_calibration=bool(reuse_existing_calibration),
    )

    if preset in ("direct-c1", "bare-pos-v1"):
        apply_bare_pos_v1(axis)
    elif preset == "bare-pos-trusted-v1":
        _apply_named_candidate(axis, BARE_POS_TRUSTED_V1)
    elif preset == "bare-pos-repeatable-v1":
        _apply_named_candidate(axis, BARE_POS_REPEATABLE_V1)
    elif preset == "bare-pos-repeatable-soft-v1":
        _apply_named_candidate(axis, BARE_POS_REPEATABLE_SOFT_V1)
    elif preset == "bare-pos-fast1":
        apply_bare_pos_fast1(axis)
    elif preset == "mounted-direct-v1":
        _apply_named_candidate(axis, MOUNTED_DIRECT_V1)

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

    snapshot = _axis_snapshot(axis, odrv)
    snapshot_health = _calibration_health(snapshot)
    if not snapshot_health["ok"]:
        raise RuntimeError(
            "MKS runtime baseline degraded after final idle/clear; "
            f"snapshot={json.dumps(snapshot_health, sort_keys=True)}"
        )

    return {
        "board_serial": str(getattr(odrv, "serial_number", "")),
        "axis_index": int(axis_index),
        "preset": preset,
        "snapshot": snapshot,
        "axis_report": common.get_axis_error_report(axis),
        "baseline_result": baseline_result,
        "applied_overrides": {
            "reuse_existing_calibration": bool(reuse_existing_calibration),
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
            "bare-pos-v1 is exploratory and currently not repeatable enough to trust as default",
            "bare-pos-repeatable-v1 is the first repeatable bare-motor candidate found so far",
            "bare-pos-trusted-v1 is the current best-balanced bare-motor MKS preset on the corrected harness",
            "bare-pos-repeatable-soft-v1 trades a little current ceiling for similar stable behavior",
            "legacy preset name direct-c1 maps to bare-pos-v1",
            "bare-pos-fast1 is more aggressive and currently less trusted than bare-pos-v1",
            "mounted-direct-v1 is the current best gearbox-mounted direct-position candidate with motor-side encoder",
            "mounted-direct-v1 still has significant return hysteresis and is not a finished motion profile",
            "mounted-direct-v1 should usually be applied with reuse_existing_calibration=True until mounted recalibration instability is resolved",
        ],
    }


def run(
    serial_number: str | None = None,
    axis_index: int = 0,
    preset: str = "baseline",
    *,
    odrv=None,
    axis=None,
    timeout_s: float = 10.0,
    reuse_existing_calibration: bool = False,
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
        odrv=odrv,
        axis=axis,
        timeout_s=timeout_s,
        reuse_existing_calibration=reuse_existing_calibration,
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
    ap.add_argument("--reuse-existing-calibration", action="store_true")
    ap.add_argument(
        "--preset",
        choices=[
            "baseline",
            "direct-c1",
            "bare-pos-v1",
            "bare-pos-trusted-v1",
            "bare-pos-repeatable-v1",
            "bare-pos-repeatable-soft-v1",
            "bare-pos-fast1",
            "mounted-direct-v1",
        ],
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
        timeout_s=10.0,
        reuse_existing_calibration=args.reuse_existing_calibration,
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
