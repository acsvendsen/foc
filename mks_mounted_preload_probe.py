#!/usr/bin/env python3
"""Mounted MKS preload / one-sided repeatability probe.

Purpose:
- compare direct target approach vs forced approach-from-below / from-above
- quantify final target error and spread under mounted gearbox hysteresis
- detect whether repeated cycling improves error ("run in") or just plateaus

This is intentionally a direct-position probe, not a trap-profile runner.
"""

from __future__ import annotations

import argparse
import json
import statistics
import time
from pathlib import Path

from odrive.enums import CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_PASSTHROUGH

import common
from mks_apply_runtime_baseline import apply_runtime_baseline
from mks_axis_characterize import build_candidate, clear_local_errors, neutralize_controller_idle_state, resolve_odrv_axis


def _parse_floats(value: str):
    return [float(part.strip()) for part in str(value).split(",") if str(part).strip()]


def _prepare_candidate(odrv, axis, candidate):
    clear_local_errors(axis, odrv=odrv, settle_s=0.05)
    axis.motor.config.current_lim = float(candidate["current_lim"])
    axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
    axis.controller.config.pos_gain = float(candidate["pos_gain"])
    axis.controller.config.vel_gain = float(candidate["vel_gain"])
    axis.controller.config.vel_integrator_gain = float(candidate["vel_i_gain"])
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


def _move_and_observe(odrv, axis, target, *, hold_s, abort_abs_turns):
    start = float(getattr(axis.encoder, "pos_estimate", 0.0))
    axis.controller.input_pos = float(target)
    deadline = time.time() + float(hold_s)
    peak_vel = 0.0
    peak_iq_set = 0.0
    peak_iq_meas = 0.0
    err = None
    while time.time() < deadline:
        ax = int(getattr(axis, "error", 0))
        mo = int(getattr(axis.motor, "error", 0))
        en = int(getattr(axis.encoder, "error", 0))
        ct = int(getattr(axis.controller, "error", 0))
        oe = int(getattr(odrv, "error", 0))
        pos = float(getattr(axis.encoder, "pos_estimate", start))
        vel = float(getattr(axis.encoder, "vel_estimate", 0.0))
        iq_set = float(getattr(axis.motor.current_control, "Iq_setpoint", 0.0))
        iq_meas = float(getattr(axis.motor.current_control, "Iq_measured", 0.0))
        peak_vel = max(peak_vel, abs(vel))
        peak_iq_set = max(peak_iq_set, abs(iq_set))
        peak_iq_meas = max(peak_iq_meas, abs(iq_meas))
        if any([ax, mo, en, ct, oe]):
            err = f"axis={hex(ax)} motor={hex(mo)} enc={hex(en)} ctrl={hex(ct)} odrv={hex(oe)}"
            break
        if abs(pos - start) > float(abort_abs_turns):
            err = f"runaway_abs_dev>{abort_abs_turns}"
            break
        time.sleep(0.01)
    end = float(getattr(axis.encoder, "pos_estimate", start))
    return {
        "ok": err is None,
        "error": err,
        "start_pos": start,
        "end_pos": end,
        "dp": float(end - start),
        "peak_vel": float(peak_vel),
        "peak_iq_set": float(peak_iq_set),
        "peak_iq_meas": float(peak_iq_meas),
    }


def _run_trial(
    odrv,
    axis,
    *,
    base_target,
    delta_turns,
    candidate,
    approach_mode,
    approach_offset_turns,
    pre_hold_s,
    final_hold_s,
    return_hold_s,
    abort_abs_turns,
):
    if not _prepare_candidate(odrv, axis, candidate):
        return {"ok": False, "error": "closed_loop_failed", "approach_mode": approach_mode, "delta_cmd": delta_turns}

    target = float(base_target) + float(delta_turns)
    sign = 1.0 if float(delta_turns) >= 0.0 else -1.0
    pre_target = None
    if str(approach_mode) == "from_below":
        pre_target = float(target) - abs(float(approach_offset_turns))
    elif str(approach_mode) == "from_above":
        pre_target = float(target) + abs(float(approach_offset_turns))

    trial = {
        "ok": True,
        "error": None,
        "approach_mode": str(approach_mode),
        "delta_cmd": float(delta_turns),
        "base_target": float(base_target),
        "target": float(target),
        "pre_target": pre_target,
        "stages": [],
    }

    if pre_target is not None:
        pre = _move_and_observe(odrv, axis, pre_target, hold_s=pre_hold_s, abort_abs_turns=abort_abs_turns)
        pre["stage"] = "preload"
        trial["stages"].append(pre)
        if not pre["ok"]:
            trial["ok"] = False
            trial["error"] = pre["error"]

    final = _move_and_observe(odrv, axis, target, hold_s=final_hold_s, abort_abs_turns=abort_abs_turns)
    final["stage"] = "target"
    trial["stages"].append(final)
    if not final["ok"] and trial["error"] is None:
        trial["ok"] = False
        trial["error"] = final["error"]

    end_pos = float(getattr(axis.encoder, "pos_estimate", target))
    trial["end_pos"] = end_pos
    trial["final_error"] = float(end_pos - float(target))
    trial["final_error_abs"] = abs(float(trial["final_error"]))
    trial["peak_iq_set"] = max((float(stage.get("peak_iq_set", 0.0)) for stage in trial["stages"]), default=0.0)
    trial["peak_iq_meas"] = max((float(stage.get("peak_iq_meas", 0.0)) for stage in trial["stages"]), default=0.0)
    trial["approach_sign"] = int(sign)

    ret = _move_and_observe(odrv, axis, base_target, hold_s=return_hold_s, abort_abs_turns=abort_abs_turns)
    ret["stage"] = "return"
    trial["stages"].append(ret)
    trial["return_pos"] = float(getattr(axis.encoder, "pos_estimate", base_target))
    trial["return_residual"] = float(trial["return_pos"] - float(base_target))
    trial["axis_report"] = common.get_axis_error_report(axis)

    common.force_idle(axis, settle_s=0.05)
    neutralize_controller_idle_state(axis)
    return trial


def _summary_for_trials(trials):
    ok_trials = [trial for trial in trials if bool(trial.get("ok"))]
    errs = [float(trial["final_error"]) for trial in ok_trials if trial.get("final_error") is not None]
    abs_errs = [abs(float(trial["final_error"])) for trial in ok_trials if trial.get("final_error") is not None]
    return_residuals = [float(trial["return_residual"]) for trial in ok_trials if trial.get("return_residual") is not None]
    peak_iq = [float(trial["peak_iq_meas"]) for trial in ok_trials if trial.get("peak_iq_meas") is not None]
    early = abs_errs[:2]
    late = abs_errs[-2:] if len(abs_errs) >= 2 else abs_errs
    return {
        "completed": len(trials),
        "ok_runs": len(ok_trials),
        "hard_faults": sum(1 for trial in trials if not bool(trial.get("ok"))),
        "mean_err": (statistics.mean(errs) if errs else None),
        "mean_abs_err": (statistics.mean(abs_errs) if abs_errs else None),
        "spread": ((max(errs) - min(errs)) if errs else None),
        "stdev_abs_err": (statistics.pstdev(abs_errs) if len(abs_errs) > 1 else 0.0 if abs_errs else None),
        "return_residual_mean": (statistics.mean(return_residuals) if return_residuals else None),
        "return_residual_abs_mean": (statistics.mean(abs(v) for v in return_residuals) if return_residuals else None),
        "peak_iq_meas_mean": (statistics.mean(peak_iq) if peak_iq else None),
        "early_mean_abs_err": (statistics.mean(early) if early else None),
        "late_mean_abs_err": (statistics.mean(late) if late else None),
        "run_in_improvement_abs_err": (
            None if (not early or not late) else float(statistics.mean(early) - statistics.mean(late))
        ),
    }


def run_mounted_preload_probe(
    *,
    odrv=None,
    axis=None,
    serial_number=None,
    axis_index=0,
    candidate_preset="mounted-direct-v3",
    approach_modes=None,
    target_deltas=None,
    approach_offset_turns=0.10,
    cycles=5,
    pre_hold_s=0.70,
    final_hold_s=0.90,
    return_hold_s=0.90,
    abort_abs_turns=0.90,
    out_path=None,
):
    approach_modes = list(approach_modes if approach_modes is not None else ["direct", "from_below", "from_above"])
    target_deltas = list(target_deltas if target_deltas is not None else [0.25, -0.25])
    odrv, axis = resolve_odrv_axis(
        odrv=odrv,
        axis=axis,
        serial_number=serial_number,
        axis_index=axis_index,
        timeout_s=10.0,
    )
    baseline = apply_runtime_baseline(
        odrv=odrv,
        axis_index=axis_index,
        preset=str(candidate_preset),
        reuse_existing_calibration=True,
    )
    candidate = build_candidate(str(candidate_preset))
    base_target = float(getattr(axis.encoder, "pos_estimate", 0.0))
    report = {
        "board_serial": str(getattr(odrv, "serial_number", "")),
        "axis_index": int(axis_index),
        "candidate_preset": str(candidate_preset),
        "candidate": dict(candidate),
        "base_target": float(base_target),
        "approach_offset_turns": float(approach_offset_turns),
        "cycles": int(cycles),
        "pre_hold_s": float(pre_hold_s),
        "final_hold_s": float(final_hold_s),
        "return_hold_s": float(return_hold_s),
        "abort_abs_turns": float(abort_abs_turns),
        "baseline_snapshot": baseline.get("snapshot"),
        "cases": [],
    }

    for delta_turns in target_deltas:
        for approach_mode in approach_modes:
            trials = []
            for idx in range(int(cycles)):
                trial = _run_trial(
                    odrv,
                    axis,
                    base_target=base_target,
                    delta_turns=float(delta_turns),
                    candidate=candidate,
                    approach_mode=str(approach_mode),
                    approach_offset_turns=float(approach_offset_turns),
                    pre_hold_s=float(pre_hold_s),
                    final_hold_s=float(final_hold_s),
                    return_hold_s=float(return_hold_s),
                    abort_abs_turns=float(abort_abs_turns),
                )
                trial["cycle_index"] = idx + 1
                trials.append(trial)
                print(
                    json.dumps(
                        {
                            "delta_cmd": float(delta_turns),
                            "approach_mode": str(approach_mode),
                            "cycle_index": idx + 1,
                            "ok": trial.get("ok"),
                            "final_error": trial.get("final_error"),
                            "return_residual": trial.get("return_residual"),
                        },
                        indent=2,
                    ),
                    flush=True,
                )
            report["cases"].append(
                {
                    "delta_cmd": float(delta_turns),
                    "approach_mode": str(approach_mode),
                    "summary": _summary_for_trials(trials),
                    "trials": trials,
                }
            )

    if out_path:
        Path(out_path).write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(f"report_saved={Path(out_path).resolve()}", flush=True)
    return report


def main():
    ap = argparse.ArgumentParser(description="Mounted MKS preload / one-sided repeatability probe")
    ap.add_argument("--serial-number", default="", help="Optional board serial. Blank uses first found.")
    ap.add_argument("--axis-index", type=int, default=0)
    ap.add_argument("--candidate-preset", default="mounted-direct-v3")
    ap.add_argument("--approach-modes", default="direct,from_below,from_above")
    ap.add_argument("--target-deltas", default="0.25,-0.25")
    ap.add_argument("--approach-offset-turns", type=float, default=0.10)
    ap.add_argument("--cycles", type=int, default=5)
    ap.add_argument("--pre-hold-s", type=float, default=0.70)
    ap.add_argument("--final-hold-s", type=float, default=0.90)
    ap.add_argument("--return-hold-s", type=float, default=0.90)
    ap.add_argument("--abort-abs-turns", type=float, default=0.90)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    run_mounted_preload_probe(
        serial_number=(str(args.serial_number).strip() or None),
        axis_index=int(args.axis_index),
        candidate_preset=str(args.candidate_preset),
        approach_modes=[part.strip() for part in str(args.approach_modes).split(",") if part.strip()],
        target_deltas=_parse_floats(args.target_deltas),
        approach_offset_turns=float(args.approach_offset_turns),
        cycles=int(args.cycles),
        pre_hold_s=float(args.pre_hold_s),
        final_hold_s=float(args.final_hold_s),
        return_hold_s=float(args.return_hold_s),
        abort_abs_turns=float(args.abort_abs_turns),
        out_path=args.out,
    )


if __name__ == "__main__":
    main()
