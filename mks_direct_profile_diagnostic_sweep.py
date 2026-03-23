#!/usr/bin/env python3
"""Diagnostic-first sweep for the shaky direct-position profile family.

Purpose:
- stop treating the current direct-position presets as normal motion profiles
- quantify the shared foundation problem with explicit diagnostics
- rank cautious candidate variants using measured hold-shake and move-hunting
  behavior instead of blind gain guessing

This is intentionally a characterization harness. It does not write flash
configuration and it does not claim the winning candidate is production-ready.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import statistics
import time
from pathlib import Path

from odrive.enums import CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_PASSTHROUGH

import common
from mks_apply_runtime_baseline import apply_runtime_baseline
from mks_axis_characterize import (
    CANDIDATE_PRESETS,
    clear_local_errors,
    neutralize_controller_idle_state,
    resolve_odrv_axis,
)


DEFAULT_MOUNTED_CANDIDATES = [
    {"name": "mounted-direct-v1", **CANDIDATE_PRESETS["mounted-direct-v1"]},
    {"name": "mounted-direct-v2", **CANDIDATE_PRESETS["mounted-direct-v2"]},
    {"name": "mounted-direct-v3", **CANDIDATE_PRESETS["mounted-direct-v3"]},
    {
        "name": "mounted-direct-soft-v4-exp",
        "current_lim": 6.0,
        "pos_gain": 4.25,
        "vel_gain": 0.30,
        "vel_i_gain": 0.01,
        "vel_limit": 1.0,
    },
    {
        "name": "mounted-direct-damped-exp",
        "current_lim": 6.0,
        "pos_gain": 3.75,
        "vel_gain": 0.28,
        "vel_i_gain": 0.0,
        "vel_limit": 0.90,
    },
]


def _rms(values: list[float]) -> float:
    if not values:
        return 0.0
    return math.sqrt(sum(v * v for v in values) / float(len(values)))


def _sign_with_deadband(value: float, *, deadband: float) -> int:
    if value > float(deadband):
        return 1
    if value < -float(deadband):
        return -1
    return 0


def _candidate_list(mode: str) -> list[dict]:
    if str(mode).strip().lower() == "mounted":
        return [dict(row) for row in DEFAULT_MOUNTED_CANDIDATES]
    raise ValueError(f"Unsupported sweep mode '{mode}'")


def _apply_candidate(axis, odrv, candidate: dict) -> None:
    clear_local_errors(axis, odrv=odrv, settle_s=0.05)
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
        raise RuntimeError("closed_loop_failed")
    common.sync_pos_setpoint(axis, settle_s=0.05, retries=3, verbose=False)


def _sample_hold_metrics(axis, target_turns: float, *, duration_s: float, dt_s: float) -> dict:
    positions = []
    velocities = []
    iq_sets = []
    iq_meas = []
    errs = []
    vel_sign_flips = 0
    prev_vel_sign = 0
    err_sign_flips = 0
    prev_err_sign = 0
    deadline = time.time() + float(duration_s)
    fault = None

    while time.time() < deadline:
        ax = int(getattr(axis, "error", 0))
        mo = int(getattr(axis.motor, "error", 0))
        en = int(getattr(axis.encoder, "error", 0))
        ct = int(getattr(axis.controller, "error", 0))
        if any([ax, mo, en, ct]):
            fault = f"axis={hex(ax)} motor={hex(mo)} enc={hex(en)} ctrl={hex(ct)}"
            break

        pos = float(getattr(axis.encoder, "pos_estimate", target_turns))
        vel = float(getattr(axis.encoder, "vel_estimate", 0.0))
        iq_s = float(getattr(axis.motor.current_control, "Iq_setpoint", 0.0))
        iq_m = float(getattr(axis.motor.current_control, "Iq_measured", 0.0))
        err = float(target_turns - pos)

        positions.append(float(pos))
        velocities.append(float(vel))
        iq_sets.append(float(iq_s))
        iq_meas.append(float(iq_m))
        errs.append(float(err))

        vel_sign = _sign_with_deadband(float(vel), deadband=0.03)
        if prev_vel_sign != 0 and vel_sign != 0 and vel_sign != prev_vel_sign:
            vel_sign_flips += 1
        if vel_sign != 0:
            prev_vel_sign = vel_sign

        err_sign = _sign_with_deadband(float(err), deadband=0.004)
        if prev_err_sign != 0 and err_sign != 0 and err_sign != prev_err_sign:
            err_sign_flips += 1
        if err_sign != 0:
            prev_err_sign = err_sign

        time.sleep(float(dt_s))

    pos_span = 0.0 if not positions else float(max(positions) - min(positions))
    err_span = 0.0 if not errs else float(max(errs) - min(errs))
    metrics = {
        "ok": fault is None,
        "error": fault,
        "samples": int(len(positions)),
        "duration_s": float(duration_s),
        "pos_span_turns": float(pos_span),
        "pos_rms_turns": float(_rms([p - statistics.mean(positions) for p in positions])) if len(positions) >= 2 else 0.0,
        "err_peak_abs_turns": float(max((abs(v) for v in errs), default=0.0)),
        "err_span_turns": float(err_span),
        "vel_rms_turns_s": float(_rms(velocities)),
        "vel_peak_abs_turns_s": float(max((abs(v) for v in velocities), default=0.0)),
        "vel_sign_flips": int(vel_sign_flips),
        "err_sign_flips": int(err_sign_flips),
        "iq_set_rms_a": float(_rms(iq_sets)),
        "iq_meas_rms_a": float(_rms(iq_meas)),
        "iq_meas_peak_abs_a": float(max((abs(v) for v in iq_meas), default=0.0)),
    }

    if not metrics["ok"]:
        metrics["classification"] = "faulted"
    elif metrics["pos_span_turns"] <= 0.006 and metrics["vel_sign_flips"] <= 1:
        metrics["classification"] = "stable_hold"
    elif metrics["pos_span_turns"] <= 0.020 and metrics["vel_sign_flips"] <= 3:
        metrics["classification"] = "mild_hold_wave"
    else:
        metrics["classification"] = "shaky_hold"

    return metrics


def _direct_move_hunting_probe(
    odrv,
    axis,
    *,
    delta_turns: float,
    move_window_s: float,
    final_hold_s: float,
    dt_s: float,
    abort_abs_turns: float,
) -> dict:
    start_pos = float(getattr(axis.encoder, "pos_estimate", 0.0))
    target = float(start_pos + float(delta_turns))
    cmd_sign = 1 if float(delta_turns) >= 0.0 else -1
    axis.controller.input_pos = float(target)

    positions = [float(start_pos)]
    velocities = []
    iq_meas = []
    errs = []
    active_vel_sign_flips = 0
    prev_vel_sign = 0
    backtrack_turns = 0.0
    monotonic_good = 0
    monotonic_total = 0
    fault = None
    aborted = False
    active_err_threshold = max(0.03, 0.15 * abs(float(delta_turns)))
    deadline = time.time() + float(move_window_s)

    while time.time() < deadline:
        ax = int(getattr(axis, "error", 0))
        mo = int(getattr(axis.motor, "error", 0))
        en = int(getattr(axis.encoder, "error", 0))
        ct = int(getattr(axis.controller, "error", 0))
        oe = int(getattr(odrv, "error", 0))
        if any([ax, mo, en, ct, oe]):
            fault = f"axis={hex(ax)} motor={hex(mo)} enc={hex(en)} ctrl={hex(ct)} odrv={hex(oe)}"
            break

        pos = float(getattr(axis.encoder, "pos_estimate", start_pos))
        vel = float(getattr(axis.encoder, "vel_estimate", 0.0))
        iq = float(getattr(axis.motor.current_control, "Iq_measured", 0.0))
        err = float(target - pos)

        prev_pos = positions[-1]
        dpos = float(pos - prev_pos)
        positions.append(float(pos))
        velocities.append(float(vel))
        iq_meas.append(float(iq))
        errs.append(float(err))

        monotonic_total += 1
        if (float(dpos) * float(cmd_sign)) >= -1e-5:
            monotonic_good += 1
        if (float(dpos) * float(cmd_sign)) < 0.0:
            backtrack_turns += abs(float(dpos))

        vel_sign = _sign_with_deadband(float(vel), deadband=0.05)
        if abs(float(err)) >= float(active_err_threshold):
            if prev_vel_sign != 0 and vel_sign != 0 and vel_sign != prev_vel_sign:
                active_vel_sign_flips += 1
            if vel_sign != 0:
                prev_vel_sign = vel_sign

        if abs(float(pos - start_pos)) > float(abort_abs_turns):
            aborted = True
            fault = f"runaway_abs_dev>{abort_abs_turns}"
            break

        time.sleep(float(dt_s))

    end_pos = float(getattr(axis.encoder, "pos_estimate", start_pos))
    move_metrics = {
        "ok": fault is None,
        "error": fault,
        "aborted": bool(aborted),
        "start_pos": float(start_pos),
        "target": float(target),
        "end_pos": float(end_pos),
        "delta_cmd": float(delta_turns),
        "dp": float(end_pos - start_pos),
        "track_ratio": (None if float(delta_turns) == 0.0 else float((end_pos - start_pos) / float(delta_turns))),
        "final_error_turns": float(target - end_pos),
        "final_error_abs_turns": abs(float(target - end_pos)),
        "peak_vel_abs_turns_s": float(max((abs(v) for v in velocities), default=0.0)),
        "vel_rms_turns_s": float(_rms(velocities)),
        "iq_meas_peak_abs_a": float(max((abs(v) for v in iq_meas), default=0.0)),
        "monotonic_fraction": (0.0 if monotonic_total == 0 else float(monotonic_good) / float(monotonic_total)),
        "backtrack_turns": float(backtrack_turns),
        "active_vel_sign_flips": int(active_vel_sign_flips),
        "active_err_threshold_turns": float(active_err_threshold),
    }

    hold_metrics = _sample_hold_metrics(axis, float(target), duration_s=float(final_hold_s), dt_s=float(dt_s))
    move_metrics["final_hold"] = hold_metrics
    if not move_metrics["ok"]:
        move_metrics["classification"] = "faulted"
    elif move_metrics["monotonic_fraction"] >= 0.98 and move_metrics["backtrack_turns"] <= 0.01 and int(move_metrics["active_vel_sign_flips"]) <= 1:
        move_metrics["classification"] = "smooth_travel"
    elif move_metrics["monotonic_fraction"] >= 0.90 and move_metrics["backtrack_turns"] <= 0.05 and int(move_metrics["active_vel_sign_flips"]) <= 4:
        move_metrics["classification"] = "mild_travel_wave"
    else:
        move_metrics["classification"] = "hunting_travel"

    return move_metrics


def _low_step_sign_probe(
    odrv,
    axis,
    candidate: dict,
    *,
    step_turns: float,
    hold_s: float,
    dt_s: float,
) -> dict:
    try:
        _apply_candidate(axis, odrv, candidate)
    except Exception as exc:
        return {"ok": False, "classification": "closed_loop_failed", "error": str(exc)}
    try:
        return common.position_sign_probe(
            axis=axis,
            step_turns=float(step_turns),
            hold_s=float(hold_s),
            dt=float(dt_s),
            current_lim=float(candidate["current_lim"]),
            pos_gain=float(candidate["pos_gain"]),
            vel_gain=float(candidate["vel_gain"]),
            vel_i_gain=float(candidate.get("vel_i_gain", 0.0)),
            vel_limit=float(candidate["vel_limit"]),
            verbose=False,
        )
    finally:
        common.force_idle(axis, settle_s=0.05)
        neutralize_controller_idle_state(axis)


def _score_candidate(sign_probe: dict, move_probe: dict) -> float:
    if not bool(move_probe.get("ok")):
        return -1000.0

    hold = dict(move_probe.get("final_hold") or {})
    score = 100.0
    if not bool(sign_probe.get("ok")):
        score -= 40.0
    score += 18.0 * max(0.0, min(1.0, float(move_probe.get("track_ratio") or 0.0)))
    score -= 70.0 * float(move_probe.get("final_error_abs_turns") or 0.0)
    score -= 60.0 * float(hold.get("pos_span_turns") or 0.0)
    score -= 15.0 * float(hold.get("vel_rms_turns_s") or 0.0)
    score -= 7.5 * float(move_probe.get("backtrack_turns") or 0.0)
    score -= 4.0 * float(move_probe.get("active_vel_sign_flips") or 0.0)
    if str(hold.get("classification") or "") == "shaky_hold":
        score -= 18.0
    if str(move_probe.get("classification") or "") == "hunting_travel":
        score -= 18.0
    return float(score)


def _evaluate_candidate(
    odrv,
    axis,
    *,
    axis_index: int,
    baseline_preset: str,
    candidate: dict,
    sign_step_turns: float,
    sign_hold_s: float,
    move_delta_turns: float,
    move_window_s: float,
    final_hold_s: float,
    dt_s: float,
    abort_abs_turns: float,
) -> dict:
    baseline = apply_runtime_baseline(
        odrv=odrv,
        axis_index=int(axis_index),
        preset=str(baseline_preset),
        reuse_existing_calibration=True,
    )

    sign_probe = _low_step_sign_probe(
        odrv,
        axis,
        candidate,
        step_turns=float(sign_step_turns),
        hold_s=float(sign_hold_s),
        dt_s=float(dt_s),
    )

    _apply_candidate(axis, odrv, candidate)
    common.sync_pos_setpoint(axis, settle_s=0.05, retries=3, verbose=False)
    move_probe = _direct_move_hunting_probe(
        odrv,
        axis,
        delta_turns=float(move_delta_turns),
        move_window_s=float(move_window_s),
        final_hold_s=float(final_hold_s),
        dt_s=float(dt_s),
        abort_abs_turns=float(abort_abs_turns),
    )
    common.force_idle(axis, settle_s=0.05)
    neutralize_controller_idle_state(axis)

    score = _score_candidate(sign_probe, move_probe)
    return {
        "candidate": dict(candidate),
        "baseline_snapshot": baseline.get("snapshot"),
        "sign_probe": sign_probe,
        "move_probe": move_probe,
        "score": float(score),
    }


def run_direct_profile_diagnostic_sweep(
    *,
    odrv=None,
    axis=None,
    serial_number=None,
    axis_index=0,
    mode="mounted",
    candidates=None,
    baseline_preset="baseline",
    sign_step_turns=0.05,
    sign_hold_s=0.25,
    move_delta_turns=2.0,
    move_window_s=3.0,
    final_hold_s=1.5,
    dt_s=0.01,
    abort_abs_turns=3.0,
    out_path=None,
) -> dict:
    odrv, axis = resolve_odrv_axis(
        odrv=odrv,
        axis=axis,
        serial_number=serial_number,
        axis_index=axis_index,
        timeout_s=10.0,
    )

    report = {
        "ts": dt.datetime.now().isoformat(),
        "board_serial": str(getattr(odrv, "serial_number", "")),
        "axis_index": int(axis_index),
        "mode": str(mode),
        "baseline_preset": str(baseline_preset),
        "config": {
            "sign_step_turns": float(sign_step_turns),
            "sign_hold_s": float(sign_hold_s),
            "move_delta_turns": float(move_delta_turns),
            "move_window_s": float(move_window_s),
            "final_hold_s": float(final_hold_s),
            "dt_s": float(dt_s),
            "abort_abs_turns": float(abort_abs_turns),
        },
        "candidates": [],
        "best_candidate": None,
    }

    sweep_candidates = [dict(row) for row in (candidates if candidates is not None else _candidate_list(str(mode)))]
    for candidate in sweep_candidates:
        rec = _evaluate_candidate(
            odrv,
            axis,
            axis_index=int(axis_index),
            baseline_preset=str(baseline_preset),
            candidate=dict(candidate),
            sign_step_turns=float(sign_step_turns),
            sign_hold_s=float(sign_hold_s),
            move_delta_turns=float(move_delta_turns),
            move_window_s=float(move_window_s),
            final_hold_s=float(final_hold_s),
            dt_s=float(dt_s),
            abort_abs_turns=float(abort_abs_turns),
        )
        report["candidates"].append(rec)

    ranked = sorted(
        list(report["candidates"]),
        key=lambda row: float(row.get("score") or -1e9),
        reverse=True,
    )
    report["best_candidate"] = (ranked[0] if ranked else None)

    if out_path is not None:
        path = Path(str(out_path)).expanduser().resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, indent=2) + "\n")
        report["save_path"] = str(path)

    return report


def main() -> None:
    ap = argparse.ArgumentParser(description="Diagnostic-first sweep for the shaky direct-position profile family")
    ap.add_argument("--serial-number", default="", help="Optional runtime board serial")
    ap.add_argument("--axis-index", type=int, default=0)
    ap.add_argument("--mode", default="mounted", choices=["mounted"])
    ap.add_argument("--baseline-preset", default="baseline")
    ap.add_argument("--sign-step-turns", type=float, default=0.05)
    ap.add_argument("--sign-hold-s", type=float, default=0.25)
    ap.add_argument("--move-delta-turns", type=float, default=2.0)
    ap.add_argument("--move-window-s", type=float, default=3.0)
    ap.add_argument("--final-hold-s", type=float, default=1.5)
    ap.add_argument("--dt-s", type=float, default=0.01)
    ap.add_argument("--abort-abs-turns", type=float, default=3.0)
    ap.add_argument("--out", default="", help="Optional JSON output path")
    args = ap.parse_args()

    report = run_direct_profile_diagnostic_sweep(
        serial_number=(str(args.serial_number).strip() or None),
        axis_index=int(args.axis_index),
        mode=str(args.mode),
        baseline_preset=str(args.baseline_preset),
        sign_step_turns=float(args.sign_step_turns),
        sign_hold_s=float(args.sign_hold_s),
        move_delta_turns=float(args.move_delta_turns),
        move_window_s=float(args.move_window_s),
        final_hold_s=float(args.final_hold_s),
        dt_s=float(args.dt_s),
        abort_abs_turns=float(args.abort_abs_turns),
        out_path=(str(args.out).strip() or None),
    )
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
