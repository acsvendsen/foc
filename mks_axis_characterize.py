#!/usr/bin/env python3
"""MKS ODrive axis bring-up and characterization helper.

Purpose:
- apply the current best runtime-only MKS normalization
- fingerprint the board/axis
- verify torque authority
- probe bare-motor direct position authority without pretending we already have
  a usable gearbox profile

This is intentionally a go/no-go diagnostic, not an autotuner.
"""

import argparse
import datetime
import json
import math
import os
import sys
import time

import odrive
from odrive.enums import AXIS_STATE_FULL_CALIBRATION_SEQUENCE, CONTROL_MODE_POSITION_CONTROL
from odrive.enums import INPUT_MODE_PASSTHROUGH

import common


CANDIDATE_PRESETS = {
    "conservative": {
        "current_lim": 2.5,
        "pos_gain": 4.5,
        "vel_gain": 0.09,
        "vel_i_gain": 0.0,
        "vel_limit": 0.35,
    },
    "bare-pos-repeatable-v1": {
        "current_lim": 2.75,
        "pos_gain": 4.75,
        "vel_gain": 0.10,
        "vel_i_gain": 0.02,
        "vel_limit": 0.44,
    },
    "bare-pos-repeatable-soft-v1": {
        "current_lim": 2.50,
        "pos_gain": 4.75,
        "vel_gain": 0.10,
        "vel_i_gain": 0.02,
        "vel_limit": 0.45,
    },
    "bare-pos-v1": {
        "current_lim": 2.75,
        "pos_gain": 4.75,
        "vel_gain": 0.10,
        "vel_i_gain": 0.02,
        "vel_limit": 0.45,
    },
    "bare-pos-fast1": {
        "current_lim": 2.75,
        "pos_gain": 4.75,
        "vel_gain": 0.10,
        "vel_i_gain": 0.02,
        "vel_limit": 0.50,
    },
}


def _safe_float(value, default=None):
    try:
        return float(value)
    except Exception:
        return default


def _safe_int(value, default=None):
    try:
        return int(value)
    except Exception:
        return default


def _axis_snapshot(axis, odrv=None):
    return {
        "serial_number": (None if odrv is None else str(getattr(odrv, "serial_number", ""))),
        "hw_version_major": (None if odrv is None else _safe_int(getattr(odrv, "hw_version_major", None))),
        "hw_version_minor": (None if odrv is None else _safe_int(getattr(odrv, "hw_version_minor", None))),
        "fw_version_major": (None if odrv is None else _safe_int(getattr(odrv, "fw_version_major", None))),
        "fw_version_minor": (None if odrv is None else _safe_int(getattr(odrv, "fw_version_minor", None))),
        "fw_version_revision": (None if odrv is None else _safe_int(getattr(odrv, "fw_version_revision", None))),
        "state": _safe_int(getattr(axis, "current_state", None)),
        "axis_err": _safe_int(getattr(axis, "error", 0), 0),
        "motor_err": _safe_int(getattr(axis.motor, "error", 0), 0),
        "enc_err": _safe_int(getattr(axis.encoder, "error", 0), 0),
        "ctrl_err": _safe_int(getattr(axis.controller, "error", 0), 0),
        "motor_is_calibrated": bool(getattr(axis.motor, "is_calibrated", False)),
        "enc_ready": bool(getattr(axis.encoder, "is_ready", False)),
        "motor_direction": _safe_int(common.get_configured_direction(axis)),
        "pos_est": _safe_float(getattr(axis.encoder, "pos_estimate", None)),
        "vel_est": _safe_float(getattr(axis.encoder, "vel_estimate", None)),
        "shadow_count": _safe_int(getattr(axis.encoder, "shadow_count", None)),
        "iq_set": _safe_float(getattr(axis.motor.current_control, "Iq_setpoint", None)),
        "iq_meas": _safe_float(getattr(axis.motor.current_control, "Iq_measured", None)),
        "current_lim": _safe_float(getattr(axis.motor.config, "current_lim", None)),
        "calibration_current": _safe_float(getattr(axis.motor.config, "calibration_current", None)),
        "torque_constant": _safe_float(getattr(axis.motor.config, "torque_constant", None)),
        "phase_resistance": _safe_float(getattr(axis.motor.config, "phase_resistance", None)),
        "phase_inductance": _safe_float(getattr(axis.motor.config, "phase_inductance", None)),
        "requested_current_range": _safe_float(getattr(axis.motor.config, "requested_current_range", None)),
        "current_control_bandwidth": _safe_float(getattr(axis.motor.config, "current_control_bandwidth", None)),
        "resistance_calib_max_voltage": _safe_float(getattr(axis.motor.config, "resistance_calib_max_voltage", None)),
        "control_mode": _safe_int(getattr(axis.controller.config, "control_mode", None)),
        "input_mode": _safe_int(getattr(axis.controller.config, "input_mode", None)),
        "pos_gain": _safe_float(getattr(axis.controller.config, "pos_gain", None)),
        "vel_gain": _safe_float(getattr(axis.controller.config, "vel_gain", None)),
        "vel_i_gain": _safe_float(getattr(axis.controller.config, "vel_integrator_gain", None)),
        "vel_limit": _safe_float(getattr(axis.controller.config, "vel_limit", None)),
        "vel_limit_tolerance": _safe_float(getattr(axis.controller.config, "vel_limit_tolerance", None)),
        "enable_overspeed_error": bool(getattr(axis.controller.config, "enable_overspeed_error", False)),
        "trap_vel": _safe_float(getattr(axis.trap_traj.config, "vel_limit", None)),
        "trap_acc": _safe_float(getattr(axis.trap_traj.config, "accel_limit", None)),
        "trap_dec": _safe_float(getattr(axis.trap_traj.config, "decel_limit", None)),
        "encoder_cpr": _safe_int(getattr(axis.encoder.config, "cpr", None)),
        "encoder_bandwidth": _safe_float(getattr(axis.encoder.config, "bandwidth", None)),
        "encoder_interp": bool(getattr(axis.encoder.config, "enable_phase_interpolation", False)),
    }


def _connect(serial_number, axis_index, timeout_s):
    odrv = odrive.find_any(
        serial_number=(None if not str(serial_number or "").strip() else str(serial_number).strip()),
        timeout=float(timeout_s),
    )
    axis = getattr(odrv, f"axis{int(axis_index)}")
    return odrv, axis


def resolve_odrv_axis(
    *,
    odrv=None,
    axis=None,
    serial_number=None,
    axis_index=0,
    timeout_s=10.0,
):
    """Resolve a board/axis from an existing handle or by discovery.

    Preferred in odrivetool: pass ``odrv=odrv0`` or ``axis=odrv0.axis0`` to
    avoid a second USB discovery pass.
    """
    if axis is not None:
        return odrv, axis
    if odrv is not None:
        return odrv, getattr(odrv, f"axis{int(axis_index)}")
    return _connect(serial_number, axis_index, timeout_s)


def _calibration_health(snapshot):
    errors = []
    if not bool(snapshot.get("motor_is_calibrated", False)):
        errors.append("motor_not_calibrated")
    if not bool(snapshot.get("enc_ready", False)):
        errors.append("encoder_not_ready")
    for key in ("axis_err", "motor_err", "enc_err", "ctrl_err"):
        try:
            if int(snapshot.get(key, 0) or 0) != 0:
                errors.append(f"{key}={hex(int(snapshot.get(key, 0) or 0))}")
        except Exception:
            pass

    phase_resistance = _safe_float(snapshot.get("phase_resistance"))
    phase_inductance = _safe_float(snapshot.get("phase_inductance"))

    if phase_resistance is None or (not math.isfinite(phase_resistance)):
        errors.append("phase_resistance_not_finite")
    elif not (0.05 <= phase_resistance <= 0.5):
        errors.append(f"phase_resistance_out_of_range={phase_resistance}")

    if phase_inductance is None or (not math.isfinite(phase_inductance)):
        errors.append("phase_inductance_not_finite")
    elif not (1e-05 <= phase_inductance <= 1e-03):
        errors.append(f"phase_inductance_out_of_range={phase_inductance}")

    return {
        "ok": (len(errors) == 0),
        "errors": errors,
        "snapshot": snapshot,
    }


def apply_mks_runtime_baseline(
    axis,
    odrv=None,
    *,
    encoder_bandwidth=200.0,
    current_control_bandwidth=300.0,
    dc_max_negative_current=-5.0,
    baseline_current_lim=6.0,
    calibration_current=6.0,
    overspeed_error=False,
    vel_limit_tolerance=4.0,
    baseline_pos_gain=4.5,
    baseline_vel_gain=0.09,
    baseline_vel_i_gain=0.0,
    baseline_vel_limit=0.35,
):
    """Apply the current best runtime-only MKS normalization.

    This is still a bare-motor characterization baseline, not a motion profile.
    """
    common.clear_errors_all(axis)
    if odrv is not None:
        try:
            odrv.clear_errors()
        except Exception:
            pass
        try:
            odrv.config.dc_max_negative_current = float(dc_max_negative_current)
        except Exception:
            pass
    common.force_idle(axis, settle_s=0.05)
    common.set_encoder(axis, cpr=1024, bandwidth=float(encoder_bandwidth), interp=True, use_index=False, encoder_source="INC_ENCODER0")
    axis.motor.config.current_control_bandwidth = float(current_control_bandwidth)
    axis.motor.config.current_lim = float(baseline_current_lim)
    axis.motor.config.calibration_current = float(calibration_current)
    calibration_attempts = []
    last_health = None
    for attempt in range(1, 4):
        is_calibrated, enc_ready, axis_err = common.calibrate(axis)
        snap = _axis_snapshot(axis, odrv)
        health = _calibration_health(snap)
        health["attempt"] = int(attempt)
        health["calibrate_result"] = {
            "motor_is_calibrated": bool(is_calibrated),
            "enc_ready": bool(enc_ready),
            "axis_err": int(axis_err),
        }
        calibration_attempts.append(health)
        last_health = health
        if health["ok"]:
            break
        common.force_idle(axis, settle_s=0.05)
        common.clear_errors_all(axis)
        if odrv is not None:
            try:
                odrv.clear_errors()
            except Exception:
                pass
    if not last_health or not last_health["ok"]:
        raise RuntimeError(
            "MKS baseline calibration produced invalid electrical parameters; "
            f"attempts={json.dumps(calibration_attempts, sort_keys=True)}"
        )
    common.clear_errors_all(axis)
    if odrv is not None:
        try:
            odrv.clear_errors()
        except Exception:
            pass
    axis.controller.config.enable_overspeed_error = bool(overspeed_error)
    axis.controller.config.vel_limit_tolerance = float(vel_limit_tolerance)
    axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
    axis.controller.config.pos_gain = float(baseline_pos_gain)
    axis.controller.config.vel_gain = float(baseline_vel_gain)
    axis.controller.config.vel_integrator_gain = float(baseline_vel_i_gain)
    axis.controller.config.vel_limit = float(baseline_vel_limit)
    try:
        common.sync_pos_setpoint(axis, settle_s=0.05, retries=3, verbose=False)
    except Exception:
        pass
    final_snapshot = _axis_snapshot(axis, odrv)
    final_health = _calibration_health(final_snapshot)
    if not final_health["ok"]:
        raise RuntimeError(
            "MKS baseline post-calibration state became invalid after sync/config; "
            f"final={json.dumps(final_health, sort_keys=True)}"
        )
    return {
        "calibration_attempts": calibration_attempts,
        "final_snapshot": final_snapshot,
        "final_health": final_health,
    }


def build_candidate(
    preset: str,
    *,
    current_lim=None,
    pos_gain=None,
    vel_gain=None,
    vel_i_gain=None,
    vel_limit=None,
):
    cfg = dict(CANDIDATE_PRESETS[str(preset)])
    if current_lim is not None:
        cfg["current_lim"] = float(current_lim)
    if pos_gain is not None:
        cfg["pos_gain"] = float(pos_gain)
    if vel_gain is not None:
        cfg["vel_gain"] = float(vel_gain)
    if vel_i_gain is not None:
        cfg["vel_i_gain"] = float(vel_i_gain)
    if vel_limit is not None:
        cfg["vel_limit"] = float(vel_limit)
    return cfg


def _position_passthrough_trial(
    axis,
    delta_turns,
    current_lim,
    pos_gain,
    vel_gain,
    vel_i_gain,
    vel_limit,
    hold_s=1.0,
    abort_abs_turns=0.5,
):
    common.clear_errors_all(axis)
    common.force_idle(axis, settle_s=0.05)
    axis.motor.config.current_lim = float(current_lim)
    axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
    axis.controller.config.pos_gain = float(pos_gain)
    axis.controller.config.vel_gain = float(vel_gain)
    axis.controller.config.vel_integrator_gain = float(vel_i_gain)
    axis.controller.config.vel_limit = float(vel_limit)
    axis.controller.config.enable_overspeed_error = False
    axis.controller.config.vel_limit_tolerance = 4.0

    if not common.ensure_closed_loop(axis, timeout_s=2.0, clear_first=False, pre_sync=True, retries=2):
        return {"ok": False, "error": "failed_closed_loop"}

    try:
        common.sync_pos_setpoint(axis, settle_s=0.05, retries=3, verbose=False)
    except Exception:
        pass

    p0 = float(getattr(axis.encoder, "pos_estimate", 0.0))
    q0 = int(getattr(axis.encoder, "shadow_count", 0))
    tgt = p0 + float(delta_turns)
    peak_vel = 0.0
    peak_iq = 0.0
    err = None
    aborted = False

    try:
        axis.controller.input_pos = float(tgt)
        t_end = time.time() + float(hold_s)
        while time.time() < t_end:
            common.assert_no_errors(axis, label="mks_axis_characterize")
            p = float(getattr(axis.encoder, "pos_estimate", p0))
            v = float(getattr(axis.encoder, "vel_estimate", 0.0))
            iq = float(getattr(axis.motor.current_control, "Iq_measured", 0.0))
            peak_vel = max(peak_vel, abs(v))
            peak_iq = max(peak_iq, abs(iq))
            if abs(p - p0) > float(abort_abs_turns):
                aborted = True
                err = f"runaway_abs_dev>{abort_abs_turns}"
                break
            time.sleep(0.01)
    except Exception as exc:
        err = str(exc)

    p1 = float(getattr(axis.encoder, "pos_estimate", p0))
    q1 = int(getattr(axis.encoder, "shadow_count", q0))
    try:
        axis.controller.input_pos = float(p0)
        time.sleep(0.10)
    except Exception:
        pass
    common.force_idle(axis, settle_s=0.05)
    return {
        "ok": err is None and not aborted,
        "error": err,
        "delta_cmd": float(delta_turns),
        "dp": float(p1 - p0),
        "dq": int(q1 - q0),
        "track_ratio": float((p1 - p0) / float(delta_turns)),
        "peak_vel": float(peak_vel),
        "peak_iq": float(peak_iq),
        "end_snap": _axis_snapshot(axis),
    }


def characterize_mks_axis(
    serial_number=None,
    axis_index=0,
    timeout_s=10.0,
    odrv=None,
    axis=None,
    *,
    candidate_preset="bare-pos-repeatable-v1",
    candidate_current_lim=None,
    candidate_pos_gain=None,
    candidate_vel_gain=None,
    candidate_vel_i_gain=None,
    candidate_vel_limit=None,
    hold_s=1.0,
    abort_abs_turns=0.5,
    encoder_bandwidth=200.0,
    current_control_bandwidth=300.0,
    dc_max_negative_current=-5.0,
    baseline_current_lim=6.0,
    calibration_current=6.0,
    overspeed_error=False,
    vel_limit_tolerance=4.0,
    baseline_pos_gain=4.5,
    baseline_vel_gain=0.09,
    baseline_vel_i_gain=0.0,
    baseline_vel_limit=0.35,
):
    odrv, axis = resolve_odrv_axis(
        odrv=odrv,
        axis=axis,
        serial_number=serial_number,
        axis_index=axis_index,
        timeout_s=timeout_s,
    )
    report = {
        "timestamp": datetime.datetime.now().isoformat(),
        "board_serial": str(getattr(odrv, "serial_number", "")),
        "axis_index": int(axis_index),
        "fingerprint_before": _axis_snapshot(axis, odrv),
    }

    apply_mks_runtime_baseline(
        axis,
        odrv,
        encoder_bandwidth=encoder_bandwidth,
        current_control_bandwidth=current_control_bandwidth,
        dc_max_negative_current=dc_max_negative_current,
        baseline_current_lim=baseline_current_lim,
        calibration_current=calibration_current,
        overspeed_error=overspeed_error,
        vel_limit_tolerance=vel_limit_tolerance,
        baseline_pos_gain=baseline_pos_gain,
        baseline_vel_gain=baseline_vel_gain,
        baseline_vel_i_gain=baseline_vel_i_gain,
        baseline_vel_limit=baseline_vel_limit,
    )
    report["after_baseline"] = _axis_snapshot(axis, odrv)

    report["torque_probe"] = common.torque_authority_ramp_probe(
        axis,
        torque_targets_nm=(0.02, -0.02, 0.04, -0.04),
        current_lim=6.0,
        vel_limit=0.5,
        ramp_s=0.12,
        dwell_s=0.12,
        settle_s=0.06,
        dt=0.01,
        min_motion_turns=8e-4,
        min_motion_counts=2,
        iq_set_gate_a=0.25,
        iq_meas_gate_a=0.05,
        track_ratio_min=0.35,
        sign_match_min=0.65,
        vel_abort_turns_s=6.0,
        leave_idle=False,
        collect_samples=False,
        verbose=False,
    )

    # Best current bare-motor candidate found so far: still characterization-only.
    candidate = build_candidate(
        candidate_preset,
        current_lim=candidate_current_lim,
        pos_gain=candidate_pos_gain,
        vel_gain=candidate_vel_gain,
        vel_i_gain=candidate_vel_i_gain,
        vel_limit=candidate_vel_limit,
    )
    report["candidate"] = dict(candidate)
    report["pass_plus_005"] = _position_passthrough_trial(axis, +0.05, hold_s=hold_s, abort_abs_turns=abort_abs_turns, **candidate)
    report["pass_minus_005"] = _position_passthrough_trial(axis, -0.05, hold_s=hold_s, abort_abs_turns=abort_abs_turns, **candidate)
    report["pass_plus_010"] = _position_passthrough_trial(axis, +0.10, hold_s=hold_s, abort_abs_turns=abort_abs_turns, **candidate)
    report["pass_minus_010"] = _position_passthrough_trial(axis, -0.10, hold_s=hold_s, abort_abs_turns=abort_abs_turns, **candidate)
    report["pass_plus_020"] = _position_passthrough_trial(axis, +0.20, hold_s=hold_s, abort_abs_turns=abort_abs_turns, **candidate)
    report["pass_minus_020"] = _position_passthrough_trial(axis, -0.20, hold_s=hold_s, abort_abs_turns=abort_abs_turns, **candidate)

    small_ok = bool(report["pass_plus_005"].get("ok")) and bool(report["pass_minus_005"].get("ok"))
    medium_ok = bool(report["pass_plus_010"].get("ok")) and bool(report["pass_minus_010"].get("ok"))
    large_ok = bool(report["pass_plus_020"].get("ok")) and bool(report["pass_minus_020"].get("ok"))

    ratio_fields = [
        "pass_plus_005",
        "pass_minus_005",
        "pass_plus_010",
        "pass_minus_010",
        "pass_plus_020",
        "pass_minus_020",
    ]
    ratios = [
        abs(float(report[name].get("track_ratio", 0.0) or 0.0))
        for name in ratio_fields
        if bool(report[name].get("ok"))
    ]
    report["verdict"] = {
        "baseline_ready": bool(report["after_baseline"].get("enc_ready")) and bool(report["after_baseline"].get("motor_is_calibrated")),
        "torque_authority_ok": bool(report["torque_probe"].get("ok")),
        "fault_free_small_steps": bool(small_ok),
        "fault_free_medium_steps": bool(medium_ok),
        "fault_free_large_steps": bool(large_ok),
        "max_abs_track_ratio": (None if not ratios else float(max(ratios))),
        "min_abs_track_ratio": (None if not ratios else float(min(ratios))),
        "go_for_motion_profiles": False,
        "recommended_next_step": (
            "raise_position_authority_without_faults"
            if (small_ok and medium_ok and large_ok)
            else "revisit_runtime_baseline"
        ),
        "why_not_profile_ready": "bare-motor position path still under-tracks significantly even when fault-free",
    }

    common.clear_errors_all(axis)
    common.force_idle(axis, settle_s=0.05)
    report["final_state"] = _axis_snapshot(axis, odrv)
    return report


def run_axis_characterization(
    serial_number=None,
    axis_index=0,
    timeout_s=10.0,
    **kwargs,
):
    """Explicit alias for characterize_mks_axis() for REPL use."""
    return characterize_mks_axis(
        serial_number=serial_number,
        axis_index=axis_index,
        timeout_s=timeout_s,
        **kwargs,
    )


def main():
    p = argparse.ArgumentParser(description="Apply MKS runtime baseline and characterize position authority.")
    p.add_argument("--serial-number", default="", help="Optional board serial. Blank uses first found.")
    p.add_argument("--axis-index", type=int, default=0, help="Axis index. Default: 0")
    p.add_argument("--timeout-s", type=float, default=10.0, help="Connect timeout in seconds.")
    p.add_argument("--out", default="", help="Optional JSON output path.")
    p.add_argument("--candidate-preset", choices=sorted(CANDIDATE_PRESETS.keys()), default="bare-pos-repeatable-v1")
    p.add_argument("--candidate-current-lim", type=float, default=None)
    p.add_argument("--candidate-pos-gain", type=float, default=None)
    p.add_argument("--candidate-vel-gain", type=float, default=None)
    p.add_argument("--candidate-vel-i-gain", type=float, default=None)
    p.add_argument("--candidate-vel-limit", type=float, default=None)
    p.add_argument("--hold-s", type=float, default=1.0)
    p.add_argument("--abort-abs-turns", type=float, default=0.5)
    p.add_argument("--encoder-bandwidth", type=float, default=200.0)
    p.add_argument("--current-control-bandwidth", type=float, default=300.0)
    p.add_argument("--dc-max-negative-current", type=float, default=-5.0)
    p.add_argument("--baseline-current-lim", type=float, default=6.0)
    p.add_argument("--calibration-current", type=float, default=6.0)
    p.add_argument("--overspeed-error", choices=["true", "false"], default="false")
    p.add_argument("--vel-limit-tolerance", type=float, default=4.0)
    p.add_argument("--baseline-pos-gain", type=float, default=4.5)
    p.add_argument("--baseline-vel-gain", type=float, default=0.09)
    p.add_argument("--baseline-vel-i-gain", type=float, default=0.0)
    p.add_argument("--baseline-vel-limit", type=float, default=0.35)
    args = p.parse_args()

    overspeed_error = (str(args.overspeed_error).strip().lower() == "true")

    res = characterize_mks_axis(
        serial_number=(str(args.serial_number).strip() or None),
        axis_index=int(args.axis_index),
        timeout_s=float(args.timeout_s),
        candidate_preset=str(args.candidate_preset),
        candidate_current_lim=args.candidate_current_lim,
        candidate_pos_gain=args.candidate_pos_gain,
        candidate_vel_gain=args.candidate_vel_gain,
        candidate_vel_i_gain=args.candidate_vel_i_gain,
        candidate_vel_limit=args.candidate_vel_limit,
        hold_s=float(args.hold_s),
        abort_abs_turns=float(args.abort_abs_turns),
        encoder_bandwidth=float(args.encoder_bandwidth),
        current_control_bandwidth=float(args.current_control_bandwidth),
        dc_max_negative_current=float(args.dc_max_negative_current),
        baseline_current_lim=float(args.baseline_current_lim),
        calibration_current=float(args.calibration_current),
        overspeed_error=bool(overspeed_error),
        vel_limit_tolerance=float(args.vel_limit_tolerance),
        baseline_pos_gain=float(args.baseline_pos_gain),
        baseline_vel_gain=float(args.baseline_vel_gain),
        baseline_vel_i_gain=float(args.baseline_vel_i_gain),
        baseline_vel_limit=float(args.baseline_vel_limit),
    )

    out_path = str(args.out or "").strip()
    if out_path:
        with open(os.path.abspath(out_path), "w", encoding="utf-8") as f:
            json.dump(res, f, indent=2, sort_keys=True)
    print(json.dumps(res, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
