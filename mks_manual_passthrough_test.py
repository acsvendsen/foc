#!/usr/bin/env python3
"""Direct MKS position-passthrough tester on normalized v0.5.6 firmware.

This is intentionally not a motion-profile runner. It validates whether a
runtime baseline and a small set of direct position controller settings behave
coherently on the bare motor before we create any real MKS profiles.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import time
from pathlib import Path

import odrive
from odrive.enums import CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_PASSTHROUGH

import common
from mks_axis_characterize import (
    CANDIDATE_PRESETS,
    apply_mks_runtime_baseline,
    build_candidate,
    clear_local_errors,
    neutralize_controller_idle_state,
    resolve_odrv_axis,
)


DEFAULT_CANDIDATES = [
    {"name": "mks_bare_pos_conservative", **CANDIDATE_PRESETS["conservative"]},
    {"name": "mks_bare_pos_trusted_v1", **CANDIDATE_PRESETS["bare-pos-trusted-v1"]},
    {"name": "mks_bare_pos_repeatable_v1", **CANDIDATE_PRESETS["bare-pos-repeatable-v1"]},
    {"name": "mks_bare_pos_repeatable_soft_v1", **CANDIDATE_PRESETS["bare-pos-repeatable-soft-v1"]},
    {"name": "mks_bare_pos_v1", **CANDIDATE_PRESETS["bare-pos-v1"]},
    {"name": "mks_bare_pos_fast1", **CANDIDATE_PRESETS["bare-pos-fast1"]},
]


def _safe_float(v, default=None):
    try:
        return float(v)
    except Exception:
        return default


def _prepare_candidate(odrv, axis, candidate):
    clear_local_errors(axis, odrv=odrv, settle_s=0.05)
    try:
        odrv.clear_errors()
    except Exception:
        pass
    axis.motor.config.current_lim = float(candidate["current_lim"])
    axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
    axis.controller.config.pos_gain = float(candidate["pos_gain"])
    axis.controller.config.vel_gain = float(candidate["vel_gain"])
    axis.controller.config.vel_integrator_gain = float(candidate.get("vel_i_gain", 0.0))
    axis.controller.config.vel_limit = float(candidate["vel_limit"])
    axis.controller.config.enable_overspeed_error = False
    axis.controller.config.vel_limit_tolerance = 4.0
    if not common.ensure_closed_loop(axis, timeout_s=2.0, clear_first=False, pre_sync=True, retries=2):
        return False
    try:
        common.sync_pos_setpoint(axis, settle_s=0.05, retries=3, verbose=False)
    except Exception:
        pass
    return True


def _candidate_list(
    *,
    candidate_preset=None,
    candidate_current_lim=None,
    candidate_pos_gain=None,
    candidate_vel_gain=None,
    candidate_vel_i_gain=None,
    candidate_vel_limit=None,
):
    if (
        candidate_preset is None
        and candidate_current_lim is None
        and candidate_pos_gain is None
        and candidate_vel_gain is None
        and candidate_vel_i_gain is None
        and candidate_vel_limit is None
    ):
        return [dict(candidate) for candidate in DEFAULT_CANDIDATES]

    preset = str(candidate_preset or "bare-pos-repeatable-v1")
    candidate = build_candidate(
        preset,
        current_lim=candidate_current_lim,
        pos_gain=candidate_pos_gain,
        vel_gain=candidate_vel_gain,
        vel_i_gain=candidate_vel_i_gain,
        vel_limit=candidate_vel_limit,
    )
    return [{"name": f"mks_{preset}_custom", **candidate}]


def _trial(odrv, axis, candidate, delta_turns, hold_s=0.40, abort_abs_turns=0.25):
    ok_cl = _prepare_candidate(odrv, axis, candidate)
    if not ok_cl:
        return {"ok": False, "error": "closed_loop_failed", "delta_cmd": float(delta_turns)}

    p0 = float(getattr(axis.encoder, "pos_estimate", 0.0))
    q0 = int(getattr(axis.encoder, "shadow_count", 0))
    tgt = p0 + float(delta_turns)
    peak_vel = 0.0
    peak_iq_set = 0.0
    peak_iq_meas = 0.0
    err = None

    axis.controller.input_pos = float(tgt)
    t_end = time.time() + float(hold_s)
    while time.time() < t_end:
        ax = int(getattr(axis, "error", 0))
        mo = int(getattr(axis.motor, "error", 0))
        en = int(getattr(axis.encoder, "error", 0))
        ct = int(getattr(axis.controller, "error", 0))
        oe = int(getattr(odrv, "error", 0))
        p = float(getattr(axis.encoder, "pos_estimate", p0))
        v = float(getattr(axis.encoder, "vel_estimate", 0.0))
        iq_set = float(getattr(axis.motor.current_control, "Iq_setpoint", 0.0))
        iq_meas = float(getattr(axis.motor.current_control, "Iq_measured", 0.0))
        peak_vel = max(peak_vel, abs(v))
        peak_iq_set = max(peak_iq_set, abs(iq_set))
        peak_iq_meas = max(peak_iq_meas, abs(iq_meas))
        if any([ax, mo, en, ct, oe]):
            err = f"axis={hex(ax)} motor={hex(mo)} enc={hex(en)} ctrl={hex(ct)} odrv={hex(oe)}"
            break
        if abs(p - p0) > float(abort_abs_turns):
            err = f"runaway_abs_dev>{abort_abs_turns}"
            break
        time.sleep(0.01)

    p1 = float(getattr(axis.encoder, "pos_estimate", p0))
    q1 = int(getattr(axis.encoder, "shadow_count", q0))
    try:
        axis.controller.input_pos = float(p0)
        time.sleep(0.10)
        p_return = float(getattr(axis.encoder, "pos_estimate", p0))
    except Exception:
        p_return = None
    common.force_idle(axis, settle_s=0.05)
    neutralize_controller_idle_state(axis)

    dp = float(p1 - p0)
    return {
        "ok": err is None,
        "error": err,
        "delta_cmd": float(delta_turns),
        "start_pos": float(p0),
        "end_pos": float(p1),
        "dp": float(dp),
        "dq": int(q1 - q0),
        "track_ratio": (None if float(delta_turns) == 0.0 else float(dp / float(delta_turns))),
        "peak_vel": float(peak_vel),
        "peak_iq_set": float(peak_iq_set),
        "peak_iq_meas": float(peak_iq_meas),
        "return_residual": (None if p_return is None else float(p_return - p0)),
        "axis_report": common.get_axis_error_report(axis),
    }


def _trial_with_return_control(
    odrv,
    axis,
    candidate,
    delta_turns,
    *,
    hold_s=0.40,
    abort_abs_turns=0.25,
    return_wait_s=0.10,
    return_settle_tol=None,
    return_settle_timeout=None,
):
    ok_cl = _prepare_candidate(odrv, axis, candidate)
    if not ok_cl:
        return {"ok": False, "error": "closed_loop_failed", "delta_cmd": float(delta_turns)}

    p0 = float(getattr(axis.encoder, "pos_estimate", 0.0))
    q0 = int(getattr(axis.encoder, "shadow_count", 0))
    tgt = p0 + float(delta_turns)
    peak_vel = 0.0
    peak_iq_set = 0.0
    peak_iq_meas = 0.0
    err = None

    axis.controller.input_pos = float(tgt)
    t_end = time.time() + float(hold_s)
    while time.time() < t_end:
        ax = int(getattr(axis, "error", 0))
        mo = int(getattr(axis.motor, "error", 0))
        en = int(getattr(axis.encoder, "error", 0))
        ct = int(getattr(axis.controller, "error", 0))
        oe = int(getattr(odrv, "error", 0))
        p = float(getattr(axis.encoder, "pos_estimate", p0))
        v = float(getattr(axis.encoder, "vel_estimate", 0.0))
        iq_set = float(getattr(axis.motor.current_control, "Iq_setpoint", 0.0))
        iq_meas = float(getattr(axis.motor.current_control, "Iq_measured", 0.0))
        peak_vel = max(peak_vel, abs(v))
        peak_iq_set = max(peak_iq_set, abs(iq_set))
        peak_iq_meas = max(peak_iq_meas, abs(iq_meas))
        if any([ax, mo, en, ct, oe]):
            err = f"axis={hex(ax)} motor={hex(mo)} enc={hex(en)} ctrl={hex(ct)} odrv={hex(oe)}"
            break
        if abs(p - p0) > float(abort_abs_turns):
            err = f"runaway_abs_dev>{abort_abs_turns}"
            break
        time.sleep(0.01)

    p1 = float(getattr(axis.encoder, "pos_estimate", p0))
    q1 = int(getattr(axis.encoder, "shadow_count", q0))
    return_residual = None
    return_settled = None
    try:
        axis.controller.input_pos = float(p0)
        if return_settle_tol is not None:
            deadline = time.time() + float(
                return_settle_timeout if return_settle_timeout is not None else max(return_wait_s, 0.10)
            )
            settled = False
            while time.time() < deadline:
                pr = float(getattr(axis.encoder, "pos_estimate", p0))
                if abs(pr - p0) <= float(return_settle_tol):
                    settled = True
                    break
                time.sleep(0.01)
            p_return = float(getattr(axis.encoder, "pos_estimate", p0))
            return_residual = float(p_return - p0)
            return_settled = bool(settled)
        else:
            time.sleep(float(return_wait_s))
            p_return = float(getattr(axis.encoder, "pos_estimate", p0))
            return_residual = float(p_return - p0)
    except Exception:
        pass
    common.force_idle(axis, settle_s=0.05)
    neutralize_controller_idle_state(axis)

    dp = float(p1 - p0)
    return {
        "ok": err is None,
        "error": err,
        "delta_cmd": float(delta_turns),
        "start_pos": float(p0),
        "end_pos": float(p1),
        "dp": float(dp),
        "dq": int(q1 - q0),
        "track_ratio": (None if float(delta_turns) == 0.0 else float(dp / float(delta_turns))),
        "peak_vel": float(peak_vel),
        "peak_iq_set": float(peak_iq_set),
        "peak_iq_meas": float(peak_iq_meas),
        "return_residual": return_residual,
        "return_settled": return_settled,
        "axis_report": common.get_axis_error_report(axis),
    }


def _score_candidate(results):
    # Higher is better. Reward non-faulting motion and symmetry, penalize overshoot/runaway.
    score = 0.0
    for r in results:
        if not r.get("ok"):
            score -= 100.0
            continue
        cmd = abs(float(r["delta_cmd"]))
        dp = abs(float(r["dp"]))
        ratio = 0.0 if cmd == 0.0 else (dp / cmd)
        score += 50.0 * min(ratio, 1.0)
        if ratio > 1.5:
            score -= 40.0 * (ratio - 1.5)
    by_mag = {}
    for r in results:
        by_mag.setdefault(abs(float(r["delta_cmd"])), []).append(r)
    for mag, rs in by_mag.items():
        if len(rs) == 2 and all(r.get("ok") for r in rs):
            a, b = rs
            sym = abs(abs(float(a["dp"])) - abs(float(b["dp"])))
            score -= 200.0 * sym
    return float(score)


def run_manual_passthrough_test(
    serial_number: str | None = None,
    axis_index: int = 0,
    out_path: str | None = None,
    *,
    odrv=None,
    axis=None,
    timeout_s: float = 10.0,
    candidate_preset: str | None = None,
    candidate_current_lim=None,
    candidate_pos_gain=None,
    candidate_vel_gain=None,
    candidate_vel_i_gain=None,
    candidate_vel_limit=None,
    steps=None,
    hold_s: float = 0.40,
    abort_abs_turns: float = 0.25,
    encoder_bandwidth: float = 200.0,
    current_control_bandwidth: float = 300.0,
    dc_max_negative_current: float = -5.0,
    baseline_current_lim: float = 6.0,
    calibration_current: float = 6.0,
    overspeed_error: bool = False,
    vel_limit_tolerance: float = 4.0,
    baseline_pos_gain: float = 4.5,
    baseline_vel_gain: float = 0.09,
    baseline_vel_i_gain: float = 0.0,
    baseline_vel_limit: float = 0.35,
    apply_baseline: bool = True,
    return_wait_s: float = 0.10,
    return_settle_tol=None,
    return_settle_timeout=None,
):
    odrv, axis = resolve_odrv_axis(
        odrv=odrv,
        axis=axis,
        serial_number=serial_number,
        axis_index=axis_index,
        timeout_s=timeout_s,
    )

    report = {
        "timestamp": dt.datetime.now().isoformat(),
        "board_serial": str(getattr(odrv, "serial_number", "")),
        "axis_index": int(axis_index),
        "firmware": [
            int(getattr(odrv, "fw_version_major", 0) or 0),
            int(getattr(odrv, "fw_version_minor", 0) or 0),
            int(getattr(odrv, "fw_version_revision", 0) or 0),
        ],
        "baseline_before": common.get_axis_error_report(axis),
    }

    baseline_result = None
    if bool(apply_baseline):
        baseline_result = apply_mks_runtime_baseline(
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
    report["baseline_after"] = common.get_axis_error_report(axis)
    report["baseline_result"] = baseline_result
    report["dc_max_negative_current"] = _safe_float(getattr(odrv.config, "dc_max_negative_current", None))
    report["baseline_config"] = {
        "apply_baseline": bool(apply_baseline),
        "encoder_bandwidth": float(encoder_bandwidth),
        "current_control_bandwidth": float(current_control_bandwidth),
        "dc_max_negative_current": float(dc_max_negative_current),
        "baseline_current_lim": float(baseline_current_lim),
        "calibration_current": float(calibration_current),
        "overspeed_error": bool(overspeed_error),
        "vel_limit_tolerance": float(vel_limit_tolerance),
        "baseline_pos_gain": float(baseline_pos_gain),
        "baseline_vel_gain": float(baseline_vel_gain),
        "baseline_vel_i_gain": float(baseline_vel_i_gain),
        "baseline_vel_limit": float(baseline_vel_limit),
        "return_wait_s": float(return_wait_s),
        "return_settle_tol": return_settle_tol,
        "return_settle_timeout": return_settle_timeout,
    }

    step_list = list(steps if steps is not None else [+0.05, -0.05, +0.10, -0.10, +0.15, -0.15, +0.20, -0.20])
    candidate_reports = []
    for candidate in _candidate_list(
        candidate_preset=candidate_preset,
        candidate_current_lim=candidate_current_lim,
        candidate_pos_gain=candidate_pos_gain,
        candidate_vel_gain=candidate_vel_gain,
        candidate_vel_i_gain=candidate_vel_i_gain,
        candidate_vel_limit=candidate_vel_limit,
    ):
        trials = []
        for delta in step_list:
            trials.append(
                _trial_with_return_control(
                    odrv,
                    axis,
                    candidate,
                    delta,
                    hold_s=hold_s,
                    abort_abs_turns=abort_abs_turns,
                    return_wait_s=return_wait_s,
                    return_settle_tol=return_settle_tol,
                    return_settle_timeout=return_settle_timeout,
                )
            )
        cand = dict(candidate)
        cand["trials"] = trials
        cand["score"] = _score_candidate(trials)
        candidate_reports.append(cand)

    candidate_reports.sort(key=lambda c: c["score"], reverse=True)
    report["candidates"] = candidate_reports
    report["best_candidate"] = candidate_reports[0] if candidate_reports else None

    clear_local_errors(axis, odrv=odrv, settle_s=0.05)
    common.force_idle(axis, settle_s=0.05)
    neutralize_controller_idle_state(axis)
    try:
        odrv.clear_errors()
    except Exception:
        pass
    report["final_state"] = common.get_axis_error_report(axis)

    if out_path:
        out = Path(out_path).resolve()
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        report["report_saved"] = str(out)

    return report


def run(serial_number: str, axis_index: int, out_path: str):
    """Backward-compatible alias for run_manual_passthrough_test()."""
    report = run_manual_passthrough_test(serial_number, axis_index, out_path=out_path)
    print(json.dumps(report, indent=2, sort_keys=True))
    if report.get("report_saved"):
        print(f"report_saved={report['report_saved']}")
    return report


def _parse_steps(value: str):
    items = []
    for raw in str(value).split(","):
        s = raw.strip()
        if not s:
            continue
        items.append(float(s))
    return items


def main():
    ap = argparse.ArgumentParser(description="Direct MKS manual passthrough test on normalized v0.5.6 firmware")
    ap.add_argument("--serial-number", required=True)
    ap.add_argument("--axis-index", type=int, default=0)
    ap.add_argument("--out", required=True)
    ap.add_argument("--candidate-preset", choices=sorted(CANDIDATE_PRESETS.keys()), default=None)
    ap.add_argument("--candidate-current-lim", type=float, default=None)
    ap.add_argument("--candidate-pos-gain", type=float, default=None)
    ap.add_argument("--candidate-vel-gain", type=float, default=None)
    ap.add_argument("--candidate-vel-i-gain", type=float, default=None)
    ap.add_argument("--candidate-vel-limit", type=float, default=None)
    ap.add_argument("--steps", default="", help="Comma-separated turn steps, e.g. 0.05,-0.05,0.10,-0.10")
    ap.add_argument("--hold-s", type=float, default=0.40)
    ap.add_argument("--abort-abs-turns", type=float, default=0.25)
    ap.add_argument("--encoder-bandwidth", type=float, default=200.0)
    ap.add_argument("--current-control-bandwidth", type=float, default=300.0)
    ap.add_argument("--dc-max-negative-current", type=float, default=-5.0)
    ap.add_argument("--baseline-current-lim", type=float, default=6.0)
    ap.add_argument("--calibration-current", type=float, default=6.0)
    ap.add_argument("--overspeed-error", choices=["true", "false"], default="false")
    ap.add_argument("--vel-limit-tolerance", type=float, default=4.0)
    ap.add_argument("--baseline-pos-gain", type=float, default=4.5)
    ap.add_argument("--baseline-vel-gain", type=float, default=0.09)
    ap.add_argument("--baseline-vel-i-gain", type=float, default=0.0)
    ap.add_argument("--baseline-vel-limit", type=float, default=0.35)
    ap.add_argument("--skip-baseline", action="store_true")
    ap.add_argument("--return-wait-s", type=float, default=0.10)
    ap.add_argument("--return-settle-tol", type=float, default=None)
    ap.add_argument("--return-settle-timeout", type=float, default=None)
    args = ap.parse_args()
    overspeed_error = (str(args.overspeed_error).strip().lower() == "true")
    steps = _parse_steps(args.steps) if str(args.steps).strip() else None
    report = run_manual_passthrough_test(
        args.serial_number,
        args.axis_index,
        out_path=args.out,
        candidate_preset=args.candidate_preset,
        candidate_current_lim=args.candidate_current_lim,
        candidate_pos_gain=args.candidate_pos_gain,
        candidate_vel_gain=args.candidate_vel_gain,
        candidate_vel_i_gain=args.candidate_vel_i_gain,
        candidate_vel_limit=args.candidate_vel_limit,
        steps=steps,
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
        apply_baseline=(not bool(args.skip_baseline)),
        return_wait_s=float(args.return_wait_s),
        return_settle_tol=args.return_settle_tol,
        return_settle_timeout=args.return_settle_timeout,
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    if report.get("report_saved"):
        print(f"report_saved={report['report_saved']}")


if __name__ == "__main__":
    main()
