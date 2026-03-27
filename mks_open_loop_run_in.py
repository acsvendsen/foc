#!/usr/bin/env python3
"""Bounded open-loop run-in helper for mounted MKS axes.

This intentionally uses AXIS_STATE_LOCKIN_SPIN instead of the normal closed-loop
motion path. It is meant for mechanical run-in experiments when the axis can
calibrate but still refuses practical breakaway under closed-loop commands.
"""

from __future__ import annotations

import argparse
import datetime
import json
import math
import time
from pathlib import Path

import odrive
from odrive.enums import AXIS_STATE_IDLE, AXIS_STATE_LOCKIN_SPIN

from mks_apply_runtime_baseline import apply_runtime_baseline
from mks_axis_characterize import _axis_snapshot, resolve_odrv_axis


def _run_lockin_pulse(axis, *, duration_s: float, max_delta_turns: float, max_vel_turns_s: float):
    start_pos = float(getattr(axis.encoder, "pos_estimate", 0.0))
    t0 = time.time()
    peak_vel = 0.0
    peak_iq = 0.0
    samples = []
    abort_reason = None

    axis.requested_state = AXIS_STATE_LOCKIN_SPIN
    while time.time() - t0 < float(duration_s):
        pos = float(getattr(axis.encoder, "pos_estimate", 0.0))
        vel = float(getattr(axis.encoder, "vel_estimate", 0.0))
        iq = float(getattr(axis.motor.current_control, "Iq_measured", 0.0))
        peak_vel = max(peak_vel, abs(vel))
        peak_iq = max(peak_iq, abs(iq))
        samples.append(
            {
                "t": float(time.time() - t0),
                "pos": pos,
                "vel": vel,
                "iq": iq,
                "state": int(getattr(axis, "current_state", 0) or 0),
            }
        )
        axis_err = int(getattr(axis, "error", 0) or 0)
        motor_err = int(getattr(axis.motor, "error", 0) or 0)
        enc_err = int(getattr(axis.encoder, "error", 0) or 0)
        ctrl_err = int(getattr(axis.controller, "error", 0) or 0)
        if axis_err or motor_err or enc_err or ctrl_err:
            abort_reason = {
                "kind": "errors",
                "snapshot": _axis_snapshot(axis),
            }
            break
        if abs(pos - start_pos) > float(max_delta_turns) or abs(vel) > float(max_vel_turns_s):
            abort_reason = {
                "kind": "guard",
                "delta_pos": float(pos - start_pos),
                "vel": float(vel),
                "snapshot": _axis_snapshot(axis),
            }
            break
        time.sleep(0.01)

    axis.requested_state = AXIS_STATE_IDLE
    time.sleep(0.2)
    end_pos = float(getattr(axis.encoder, "pos_estimate", 0.0))
    return {
        "start_pos": start_pos,
        "end_pos": end_pos,
        "delta_pos": float(end_pos - start_pos),
        "peak_vel_abs": float(peak_vel),
        "peak_iq_abs": float(peak_iq),
        "abort_reason": abort_reason,
        "samples_tail": samples[-12:],
    }


def run_open_loop_run_in(
    *,
    serial_number: str | None = None,
    axis_index: int = 0,
    odrv=None,
    axis=None,
    timeout_s: float = 10.0,
    preset: str = "mounted-direct-v3",
    pole_pairs: int = 7,
    calibration_current: float = 2.0,
    encoder_offset_calibration_current: float = 8.0,
    current_lim: float = 8.0,
    cycles: int = 20,
    lockin_current: float = 10.0,
    lockin_vel: float = 10.0,
    lockin_accel: float = 20.0,
    ramp_time: float = 0.4,
    ramp_distance: float = 2.0 * math.pi,
    finish_distance: float = 100.0,
    pulse_duration_s: float = 0.8,
    cooldown_s: float = 0.15,
    min_delta_turns: float = 0.03,
    max_delta_turns: float = 0.75,
    max_vel_turns_s: float = 4.0,
):
    odrv, axis = resolve_odrv_axis(
        odrv=odrv,
        axis=axis,
        serial_number=serial_number,
        axis_index=axis_index,
        timeout_s=timeout_s,
    )

    result = {
        "serial_number": str(getattr(odrv, "serial_number", serial_number)),
        "axis_index": int(axis_index),
        "requested": {
            "preset": str(preset),
            "pole_pairs": int(pole_pairs),
            "calibration_current": float(calibration_current),
            "encoder_offset_calibration_current": float(encoder_offset_calibration_current),
            "current_lim": float(current_lim),
            "cycles": int(cycles),
            "lockin_current": float(lockin_current),
            "lockin_vel": float(lockin_vel),
            "lockin_accel": float(lockin_accel),
            "ramp_time": float(ramp_time),
            "ramp_distance": float(ramp_distance),
            "finish_distance": float(finish_distance),
            "pulse_duration_s": float(pulse_duration_s),
            "cooldown_s": float(cooldown_s),
            "min_delta_turns": float(min_delta_turns),
            "max_delta_turns": float(max_delta_turns),
            "max_vel_turns_s": float(max_vel_turns_s),
        },
        "cycles_run": [],
    }

    prev_cfg = None
    try:
        result["before"] = _axis_snapshot(axis, odrv)
        baseline = apply_runtime_baseline(
            odrv=odrv,
            axis=axis,
            preset=preset,
            pole_pairs=int(pole_pairs),
            calibration_current=float(calibration_current),
            encoder_offset_calibration_current=float(encoder_offset_calibration_current),
        )
        result["baseline"] = baseline
        axis.motor.config.current_lim = float(current_lim)

        cfg = axis.config.general_lockin
        prev_cfg = {
            "current": float(cfg.current),
            "ramp_time": float(cfg.ramp_time),
            "ramp_distance": float(cfg.ramp_distance),
            "accel": float(cfg.accel),
            "vel": float(cfg.vel),
            "finish_distance": float(cfg.finish_distance),
            "finish_on_vel": bool(cfg.finish_on_vel),
            "finish_on_distance": bool(cfg.finish_on_distance),
        }
        cfg.current = float(lockin_current)
        cfg.ramp_time = float(ramp_time)
        cfg.accel = float(lockin_accel)
        cfg.vel = float(lockin_vel)
        cfg.ramp_distance = float(ramp_distance)
        cfg.finish_distance = float(finish_distance)
        cfg.finish_on_vel = False
        cfg.finish_on_distance = False

        low_motion_count = 0
        total_delta = 0.0
        for idx in range(int(cycles)):
            pulse = _run_lockin_pulse(
                axis,
                duration_s=float(pulse_duration_s),
                max_delta_turns=float(max_delta_turns),
                max_vel_turns_s=float(max_vel_turns_s),
            )
            pulse["cycle_index"] = int(idx + 1)
            result["cycles_run"].append(pulse)
            total_delta += float(pulse["delta_pos"])

            if pulse["abort_reason"] is not None:
                result["abort_reason"] = {
                    "kind": "pulse_abort",
                    "cycle_index": int(idx + 1),
                    "detail": pulse["abort_reason"],
                }
                break

            if abs(float(pulse["delta_pos"])) < float(min_delta_turns):
                low_motion_count += 1
            else:
                low_motion_count = 0
            if low_motion_count >= 2:
                result["abort_reason"] = {
                    "kind": "low_motion",
                    "cycle_index": int(idx + 1),
                    "delta_pos": float(pulse["delta_pos"]),
                }
                break
            time.sleep(float(cooldown_s))

        result["total_delta_turns"] = float(total_delta)
    finally:
        try:
            axis.requested_state = AXIS_STATE_IDLE
        except Exception:
            pass
        try:
            if prev_cfg is not None:
                cfg = axis.config.general_lockin
                cfg.current = float(prev_cfg["current"])
                cfg.ramp_time = float(prev_cfg["ramp_time"])
                cfg.ramp_distance = float(prev_cfg["ramp_distance"])
                cfg.accel = float(prev_cfg["accel"])
                cfg.vel = float(prev_cfg["vel"])
                cfg.finish_distance = float(prev_cfg["finish_distance"])
                cfg.finish_on_vel = bool(prev_cfg["finish_on_vel"])
                cfg.finish_on_distance = bool(prev_cfg["finish_on_distance"])
        except Exception:
            pass
        time.sleep(0.1)
        result["after"] = _axis_snapshot(axis, odrv)

    return result


def main():
    parser = argparse.ArgumentParser(description="Bounded open-loop run-in using AXIS_STATE_LOCKIN_SPIN.")
    parser.add_argument("--serial-number", default=None)
    parser.add_argument("--axis-index", type=int, default=0)
    parser.add_argument("--timeout-s", type=float, default=10.0)
    parser.add_argument("--preset", default="mounted-direct-v3")
    parser.add_argument("--pole-pairs", type=int, default=7)
    parser.add_argument("--calibration-current", type=float, default=2.0)
    parser.add_argument("--encoder-offset-calibration-current", type=float, default=8.0)
    parser.add_argument("--current-lim", type=float, default=8.0)
    parser.add_argument("--cycles", type=int, default=20)
    parser.add_argument("--lockin-current", type=float, default=10.0)
    parser.add_argument("--lockin-vel", type=float, default=10.0)
    parser.add_argument("--lockin-accel", type=float, default=20.0)
    parser.add_argument("--ramp-time", type=float, default=0.4)
    parser.add_argument("--ramp-distance", type=float, default=(2.0 * math.pi))
    parser.add_argument("--finish-distance", type=float, default=100.0)
    parser.add_argument("--pulse-duration-s", type=float, default=0.8)
    parser.add_argument("--cooldown-s", type=float, default=0.15)
    parser.add_argument("--min-delta-turns", type=float, default=0.03)
    parser.add_argument("--max-delta-turns", type=float, default=0.75)
    parser.add_argument("--max-vel-turns-s", type=float, default=4.0)
    parser.add_argument("--json-out", default=None)
    args = parser.parse_args()

    result = run_open_loop_run_in(
        serial_number=args.serial_number,
        axis_index=args.axis_index,
        timeout_s=args.timeout_s,
        preset=args.preset,
        pole_pairs=args.pole_pairs,
        calibration_current=args.calibration_current,
        encoder_offset_calibration_current=args.encoder_offset_calibration_current,
        current_lim=args.current_lim,
        cycles=args.cycles,
        lockin_current=args.lockin_current,
        lockin_vel=args.lockin_vel,
        lockin_accel=args.lockin_accel,
        ramp_time=args.ramp_time,
        ramp_distance=args.ramp_distance,
        finish_distance=args.finish_distance,
        pulse_duration_s=args.pulse_duration_s,
        cooldown_s=args.cooldown_s,
        min_delta_turns=args.min_delta_turns,
        max_delta_turns=args.max_delta_turns,
        max_vel_turns_s=args.max_vel_turns_s,
    )

    if args.json_out:
        out_path = Path(args.json_out)
    else:
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = Path("logs") / f"mks_open_loop_run_in_{stamp}.json"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(result, indent=2, sort_keys=True))
    print(json.dumps({"json_out": str(out_path), **result}, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
