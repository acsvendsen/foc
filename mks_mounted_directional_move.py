#!/usr/bin/env python3
"""Direction-aware mounted MKS direct-position move helper.

This encodes the measured preload rule for the mounted 25:1 gearbox path with
motor-side encoder:

- positive targets: approach from above
- negative targets: approach from below

This is a direct-position helper only. It is not a trap/profile runner.
"""

from __future__ import annotations

import argparse
import json
import time

import common
from mks_apply_runtime_baseline import apply_runtime_baseline
from mks_axis_characterize import build_candidate, neutralize_controller_idle_state, resolve_odrv_axis
from mks_mounted_preload_rules import choose_directional_approach, select_directional_preload_offset
from mks_mounted_preload_probe import _move_and_observe, _prepare_candidate


def _move_to_target_direct(
    odrv,
    axis,
    target,
    *,
    timeout_s,
    abort_abs_turns,
    target_tolerance_turns,
    target_vel_tolerance_turns_s,
    dt=0.01,
):
    start = float(getattr(axis.encoder, "pos_estimate", 0.0))
    axis.controller.input_pos = float(target)
    deadline = time.time() + max(0.05, float(timeout_s))
    peak_vel = 0.0
    peak_iq_set = 0.0
    peak_iq_meas = 0.0
    err = None
    reached = False
    reach_t = None

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
        if (
            abs(float(target) - float(pos)) <= float(target_tolerance_turns)
            and abs(float(vel)) <= float(target_vel_tolerance_turns_s)
        ):
            reached = True
            reach_t = float(time.time())
            break
        time.sleep(max(0.005, float(dt)))

    end = float(getattr(axis.encoder, "pos_estimate", start))
    if err is None and not reached:
        err = (
            "target_not_reached_within_timeout "
            f"(timeout_s={float(timeout_s):.3f} final_err={float(target) - float(end):+.6f}t)"
        )

    return {
        "ok": (err is None),
        "error": err,
        "start_pos": float(start),
        "end_pos": float(end),
        "dp": float(end - start),
        "peak_vel": float(peak_vel),
        "peak_iq_set": float(peak_iq_set),
        "peak_iq_meas": float(peak_iq_meas),
        "target": float(target),
        "reached": bool(reached),
        "reach_time_s": (None if reach_t is None else float(reach_t - (deadline - max(0.05, float(timeout_s))))),
        "final_error": float(float(target) - float(end)),
        "final_error_abs": abs(float(float(target) - float(end))),
    }


def run_direct_move(
    *,
    odrv=None,
    axis=None,
    serial_number=None,
    axis_index=0,
    candidate_preset="bare-direct-smooth-v1",
    delta_turns=None,
    target_turns=None,
    timeout_s=8.0,
    final_hold_s=1.0,
    return_to_start=False,
    return_hold_s=0.90,
    abort_abs_turns=1.80,
    target_tolerance_turns=0.03,
    target_vel_tolerance_turns_s=0.20,
):
    if delta_turns is None and target_turns is None:
        raise ValueError("run_direct_move requires delta_turns or target_turns")

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
    if not _prepare_candidate(odrv, axis, candidate):
        raise RuntimeError("closed_loop_failed")

    start_pos = float(getattr(axis.encoder, "pos_estimate", 0.0))
    target = float(target_turns) if target_turns is not None else float(start_pos + float(delta_turns))
    delta = float(target - start_pos)

    final = _move_to_target_direct(
        odrv,
        axis,
        target,
        timeout_s=float(timeout_s),
        abort_abs_turns=abort_abs_turns,
        target_tolerance_turns=target_tolerance_turns,
        target_vel_tolerance_turns_s=target_vel_tolerance_turns_s,
    )
    final["stage"] = "target"
    if not final["ok"]:
        raise RuntimeError(f"target_failed: {final['error']}")

    if float(final_hold_s) > 0.0:
        hold = _move_and_observe(odrv, axis, target, hold_s=final_hold_s, abort_abs_turns=abort_abs_turns)
        hold["stage"] = "hold"
    else:
        hold = None

    end_pos = float(getattr(axis.encoder, "pos_estimate", target))
    result = {
        "board_serial": str(getattr(odrv, "serial_number", "")),
        "axis_index": int(axis_index),
        "candidate_preset": str(candidate_preset),
        "candidate": dict(candidate),
        "baseline_snapshot": baseline.get("snapshot"),
        "start_pos": float(start_pos),
        "target": float(target),
        "delta_cmd": float(delta),
        "approach_mode": "direct",
        "approach_offset_turns": 0.0,
        "pre_target": None,
        "end_pos": float(end_pos),
        "final_error": float(end_pos - float(target)),
        "final_error_abs": abs(float(end_pos - float(target))),
        "stages": ([final] + ([hold] if hold is not None else [])),
    }

    if bool(return_to_start):
        ret = _move_and_observe(odrv, axis, start_pos, hold_s=return_hold_s, abort_abs_turns=abort_abs_turns)
        ret["stage"] = "return"
        result["return"] = ret
        result["return_pos"] = float(getattr(axis.encoder, "pos_estimate", start_pos))
        result["return_residual"] = float(result["return_pos"] - float(start_pos))

    common.force_idle(axis, settle_s=0.05)
    neutralize_controller_idle_state(axis)
    result["axis_report"] = common.get_axis_error_report(axis)
    return result


def run_directional_move(
    *,
    odrv=None,
    axis=None,
    serial_number=None,
    axis_index=0,
    candidate_preset="mounted-direct-v3",
    delta_turns=None,
    target_turns=None,
    approach_offset_turns=None,
    timeout_s=8.0,
    pre_hold_s=0.70,
    final_hold_s=0.90,
    return_to_start=False,
    return_hold_s=0.90,
    abort_abs_turns=0.90,
    target_tolerance_turns=0.03,
    target_vel_tolerance_turns_s=0.20,
):
    if delta_turns is None and target_turns is None:
        raise ValueError("run_directional_move requires delta_turns or target_turns")

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
    if not _prepare_candidate(odrv, axis, candidate):
        raise RuntimeError("closed_loop_failed")

    start_pos = float(getattr(axis.encoder, "pos_estimate", 0.0))
    target = float(target_turns) if target_turns is not None else float(start_pos + float(delta_turns))
    delta = float(target - start_pos)
    approach_mode = choose_directional_approach(delta)
    if approach_offset_turns is None:
        approach_offset_turns = select_directional_preload_offset(delta)

    pre_target = None
    if approach_mode == "from_above":
        pre_target = float(target) + abs(float(approach_offset_turns))
    elif approach_mode == "from_below":
        pre_target = float(target) - abs(float(approach_offset_turns))

    stages = []
    if pre_target is not None:
        pre_dist = abs(float(pre_target) - float(start_pos))
        final_dist_est = abs(float(target) - float(pre_target))
        total_dist = max(1e-6, float(pre_dist + final_dist_est))
        pre_timeout_s = max(0.75, float(timeout_s) * float(pre_dist / total_dist))
        pre = _move_to_target_direct(
            odrv,
            axis,
            pre_target,
            timeout_s=pre_timeout_s,
            abort_abs_turns=abort_abs_turns,
            target_tolerance_turns=target_tolerance_turns,
            target_vel_tolerance_turns_s=target_vel_tolerance_turns_s,
        )
        pre["stage"] = "preload"
        stages.append(pre)
        if not pre["ok"]:
            raise RuntimeError(f"preload_failed: {pre['error']}")
        if float(pre_hold_s) > 0.0:
            pre_hold = _move_and_observe(odrv, axis, pre_target, hold_s=pre_hold_s, abort_abs_turns=abort_abs_turns)
            pre_hold["stage"] = "preload_hold"
            stages.append(pre_hold)

    cur_pos = float(getattr(axis.encoder, "pos_estimate", start_pos))
    final_dist = abs(float(target) - float(cur_pos))
    direct_dist = abs(float(target) - float(start_pos))
    if pre_target is None:
        final_timeout_s = float(timeout_s)
    else:
        final_timeout_s = max(0.75, float(timeout_s) * float(final_dist / max(1e-6, direct_dist + abs(float(pre_target) - float(start_pos)))))
    final = _move_to_target_direct(
        odrv,
        axis,
        target,
        timeout_s=final_timeout_s,
        abort_abs_turns=abort_abs_turns,
        target_tolerance_turns=target_tolerance_turns,
        target_vel_tolerance_turns_s=target_vel_tolerance_turns_s,
    )
    final["stage"] = "target"
    stages.append(final)
    if not final["ok"]:
        raise RuntimeError(f"target_failed: {final['error']}")

    if float(final_hold_s) > 0.0:
        hold = _move_and_observe(odrv, axis, target, hold_s=final_hold_s, abort_abs_turns=abort_abs_turns)
        hold["stage"] = "hold"
        stages.append(hold)

    end_pos = float(getattr(axis.encoder, "pos_estimate", target))
    result = {
        "board_serial": str(getattr(odrv, "serial_number", "")),
        "axis_index": int(axis_index),
        "candidate_preset": str(candidate_preset),
        "candidate": dict(candidate),
        "baseline_snapshot": baseline.get("snapshot"),
        "start_pos": float(start_pos),
        "target": float(target),
        "delta_cmd": float(delta),
        "approach_mode": str(approach_mode),
        "approach_offset_turns": float(approach_offset_turns),
        "pre_target": pre_target,
        "end_pos": float(end_pos),
        "final_error": float(end_pos - float(target)),
        "final_error_abs": abs(float(end_pos - float(target))),
        "stages": stages,
    }

    if bool(return_to_start):
        ret = _move_and_observe(odrv, axis, start_pos, hold_s=return_hold_s, abort_abs_turns=abort_abs_turns)
        ret["stage"] = "return"
        result["return"] = ret
        result["return_pos"] = float(getattr(axis.encoder, "pos_estimate", start_pos))
        result["return_residual"] = float(result["return_pos"] - float(start_pos))

    common.force_idle(axis, settle_s=0.05)
    neutralize_controller_idle_state(axis)
    result["axis_report"] = common.get_axis_error_report(axis)
    return result


def main():
    ap = argparse.ArgumentParser(description="Direction-aware mounted MKS direct-position move helper")
    ap.add_argument("--serial-number", default="", help="Optional board serial")
    ap.add_argument("--axis-index", type=int, default=0)
    ap.add_argument("--candidate-preset", default="mounted-direct-v3")
    ap.add_argument("--delta-turns", type=float, default=None)
    ap.add_argument("--target-turns", type=float, default=None)
    ap.add_argument("--approach-offset-turns", type=float, default=None)
    ap.add_argument("--pre-hold-s", type=float, default=0.70)
    ap.add_argument("--final-hold-s", type=float, default=0.90)
    ap.add_argument("--return-to-start", action="store_true")
    ap.add_argument("--return-hold-s", type=float, default=0.90)
    ap.add_argument("--abort-abs-turns", type=float, default=0.90)
    args = ap.parse_args()

    res = run_directional_move(
        serial_number=(str(args.serial_number).strip() or None),
        axis_index=int(args.axis_index),
        candidate_preset=str(args.candidate_preset),
        delta_turns=args.delta_turns,
        target_turns=args.target_turns,
        approach_offset_turns=(None if args.approach_offset_turns is None else float(args.approach_offset_turns)),
        pre_hold_s=float(args.pre_hold_s),
        final_hold_s=float(args.final_hold_s),
        return_to_start=bool(args.return_to_start),
        return_hold_s=float(args.return_hold_s),
        abort_abs_turns=float(args.abort_abs_turns),
    )
    print(json.dumps(res, indent=2))


if __name__ == "__main__":
    main()
