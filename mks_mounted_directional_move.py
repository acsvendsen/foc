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


def _read_position_loop_config(axis):
    return {
        "pos_gain": float(getattr(axis.controller.config, "pos_gain", 0.0)),
        "vel_gain": float(getattr(axis.controller.config, "vel_gain", 0.0)),
        "vel_i_gain": float(getattr(axis.controller.config, "vel_integrator_gain", 0.0)),
        "vel_limit": float(getattr(axis.controller.config, "vel_limit", 0.0)),
    }


def _apply_position_loop_config(axis, *, pos_gain=None, vel_gain=None, vel_i_gain=None, vel_limit=None):
    if pos_gain is not None:
        axis.controller.config.pos_gain = float(pos_gain)
    if vel_gain is not None:
        axis.controller.config.vel_gain = float(vel_gain)
    if vel_i_gain is not None:
        axis.controller.config.vel_integrator_gain = float(vel_i_gain)
    if vel_limit is not None:
        axis.controller.config.vel_limit = float(vel_limit)


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
    commanded_span = abs(float(target) - float(start))
    effective_abort_abs_turns = max(
        float(abort_abs_turns),
        float(commanded_span) + max(0.25, 0.20 * float(commanded_span)),
    )
    axis.controller.input_pos = float(target)
    deadline = time.time() + max(0.05, float(timeout_s))
    peak_vel = 0.0
    peak_iq_set = 0.0
    peak_iq_meas = 0.0
    err = None
    reached = False
    reach_t = None
    peak_track_err = 0.0
    monotonic_good = 0
    monotonic_total = 0
    backtrack_turns = 0.0
    active_vel_sign_flips = 0
    prev_vel_sign = 0
    active_err_threshold = max(0.03, 0.15 * abs(float(target) - float(start)))
    prev_pos = float(start)
    move_sign = 1 if (float(target) - float(start)) > 0.0 else (-1 if (float(target) - float(start)) < 0.0 else 0)

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
        track_err = float(target) - float(pos)
        peak_track_err = max(float(peak_track_err), abs(float(track_err)))
        if any([ax, mo, en, ct, oe]):
            err = f"axis={hex(ax)} motor={hex(mo)} enc={hex(en)} ctrl={hex(ct)} odrv={hex(oe)}"
            break
        if abs(pos - start) > float(effective_abort_abs_turns):
            err = f"runaway_abs_dev>{effective_abort_abs_turns:.6f}"
            break

        dpos = float(pos - prev_pos)
        monotonic_total += 1
        if move_sign == 0 or (float(dpos) * float(move_sign)) >= -1e-5:
            monotonic_good += 1
        if move_sign != 0 and (float(dpos) * float(move_sign)) < 0.0:
            backtrack_turns += abs(float(dpos))
        prev_pos = float(pos)

        if abs(float(track_err)) >= float(active_err_threshold):
            vel_sign = 1 if float(vel) > 0.05 else (-1 if float(vel) < -0.05 else 0)
            if prev_vel_sign != 0 and vel_sign != 0 and vel_sign != prev_vel_sign:
                active_vel_sign_flips += 1
            if vel_sign != 0:
                prev_vel_sign = vel_sign
        if (
            abs(float(track_err)) <= float(target_tolerance_turns)
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
        "effective_abort_abs_turns": float(effective_abort_abs_turns),
        "reached": bool(reached),
        "reach_time_s": (None if reach_t is None else float(reach_t - (deadline - max(0.05, float(timeout_s))))),
        "final_error": float(float(target) - float(end)),
        "final_error_abs": abs(float(float(target) - float(end))),
        "peak_track_err": float(peak_track_err),
        "monotonic_fraction": (0.0 if monotonic_total == 0 else float(monotonic_good) / float(monotonic_total)),
        "backtrack_turns": float(backtrack_turns),
        "active_vel_sign_flips": int(active_vel_sign_flips),
    }


def _slew_to_target_direct(
    odrv,
    axis,
    target,
    *,
    timeout_s,
    abort_abs_turns,
    command_vel_turns_s,
    handoff_window_turns,
    target_tolerance_turns,
    target_vel_tolerance_turns_s,
    dt=0.01,
):
    start = float(getattr(axis.encoder, "pos_estimate", 0.0))
    commanded_span = abs(float(target) - float(start))
    effective_abort_abs_turns = max(
        float(abort_abs_turns),
        float(commanded_span) + max(0.25, 0.20 * float(commanded_span)),
    )
    cmd_pos = float(start)
    axis.controller.input_pos = float(cmd_pos)
    deadline = time.time() + max(0.05, float(timeout_s))
    peak_vel = 0.0
    peak_iq_set = 0.0
    peak_iq_meas = 0.0
    err = None
    reached = False
    handoff_done = False
    reach_t = None
    peak_track_err = 0.0
    monotonic_good = 0
    monotonic_total = 0
    backtrack_turns = 0.0
    active_vel_sign_flips = 0
    prev_vel_sign = 0
    active_err_threshold = max(0.03, 0.15 * abs(float(target) - float(start)))
    prev_pos = float(start)
    command_step = max(1e-5, float(command_vel_turns_s) * max(0.005, float(dt)))

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
        track_err = float(target) - float(pos)
        peak_track_err = max(float(peak_track_err), abs(float(track_err)))

        if any([ax, mo, en, ct, oe]):
            err = f"axis={hex(ax)} motor={hex(mo)} enc={hex(en)} ctrl={hex(ct)} odrv={hex(oe)}"
            break
        if abs(pos - start) > float(effective_abort_abs_turns):
            err = f"runaway_abs_dev>{effective_abort_abs_turns:.6f}"
            break

        dpos = float(pos - prev_pos)
        move_sign = 1 if (float(target) - float(start)) > 0.0 else (-1 if (float(target) - float(start)) < 0.0 else 0)
        monotonic_total += 1
        if move_sign == 0 or (float(dpos) * float(move_sign)) >= -1e-5:
            monotonic_good += 1
        if move_sign != 0 and (float(dpos) * float(move_sign)) < 0.0:
            backtrack_turns += abs(float(dpos))
        prev_pos = float(pos)

        if abs(float(track_err)) >= float(active_err_threshold):
            vel_sign = 1 if float(vel) > 0.05 else (-1 if float(vel) < -0.05 else 0)
            if prev_vel_sign != 0 and vel_sign != 0 and vel_sign != prev_vel_sign:
                active_vel_sign_flips += 1
            if vel_sign != 0:
                prev_vel_sign = vel_sign

        if (
            abs(float(track_err)) <= float(target_tolerance_turns)
            and abs(float(vel)) <= float(target_vel_tolerance_turns_s)
        ):
            reached = True
            reach_t = float(time.time())
            break

        if not handoff_done:
            remaining_cmd = float(target) - float(cmd_pos)
            if abs(float(remaining_cmd)) <= float(handoff_window_turns):
                cmd_pos = float(target)
                handoff_done = True
            else:
                cmd_pos += max(-float(command_step), min(float(command_step), float(remaining_cmd)))
        else:
            cmd_pos = float(target)
        axis.controller.input_pos = float(cmd_pos)
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
        "effective_abort_abs_turns": float(effective_abort_abs_turns),
        "reached": bool(reached),
        "reach_time_s": (None if reach_t is None else float(reach_t - (deadline - max(0.05, float(timeout_s))))),
        "final_error": float(float(target) - float(end)),
        "final_error_abs": abs(float(float(target) - float(end))),
        "command_vel_turns_s": float(command_vel_turns_s),
        "handoff_window_turns": float(handoff_window_turns),
        "peak_track_err": float(peak_track_err),
        "monotonic_fraction": (0.0 if monotonic_total == 0 else float(monotonic_good) / float(monotonic_total)),
        "backtrack_turns": float(backtrack_turns),
        "active_vel_sign_flips": int(active_vel_sign_flips),
    }


def _restore_position_passthrough(axis, input_pos=None):
    pos_ctrl_mode = int(getattr(common, "CONTROL_MODE_POSITION_CONTROL", 3))
    passthrough = int(getattr(common, "INPUT_MODE_PASSTHROUGH", 1))
    try:
        axis.controller.config.control_mode = int(pos_ctrl_mode)
    except Exception:
        pass
    try:
        axis.controller.config.input_mode = int(passthrough)
    except Exception:
        pass
    try:
        axis.controller.input_vel = 0.0
    except Exception:
        pass
    try:
        axis.controller.input_pos = float(
            getattr(axis.encoder, "pos_estimate", 0.0) if input_pos is None else input_pos
        )
    except Exception:
        pass


def _velocity_travel_to_target(
    odrv,
    axis,
    target,
    *,
    timeout_s,
    abort_abs_turns,
    command_vel_turns_s,
    handoff_window_turns,
    dt=0.01,
):
    start = float(getattr(axis.encoder, "pos_estimate", 0.0))
    commanded_span = abs(float(target) - float(start))
    effective_abort_abs_turns = max(
        float(abort_abs_turns),
        float(commanded_span) + max(0.25, 0.20 * float(commanded_span)),
    )
    vel_ctrl_mode = int(getattr(common, "CONTROL_MODE_VELOCITY_CONTROL", 2))
    passthrough = int(getattr(common, "INPUT_MODE_PASSTHROUGH", 1))
    try:
        axis.controller.config.control_mode = int(vel_ctrl_mode)
    except Exception:
        pass
    try:
        axis.controller.config.input_mode = int(passthrough)
    except Exception:
        pass
    try:
        axis.controller.config.vel_limit = max(
            float(getattr(axis.controller.config, "vel_limit", 0.0)),
            abs(float(command_vel_turns_s)) * 1.5,
        )
    except Exception:
        pass

    deadline = time.time() + max(0.05, float(timeout_s))
    peak_vel = 0.0
    peak_iq_set = 0.0
    peak_iq_meas = 0.0
    err = None
    reached = False
    handoff_done = False
    reach_t = None
    peak_track_err = 0.0
    monotonic_good = 0
    monotonic_total = 0
    backtrack_turns = 0.0
    active_vel_sign_flips = 0
    prev_vel_sign = 0
    active_err_threshold = max(0.03, 0.15 * abs(float(target) - float(start)))
    prev_pos = float(start)
    move_sign = 1 if (float(target) - float(start)) > 0.0 else (-1 if (float(target) - float(start)) < 0.0 else 0)

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
        track_err = float(target) - float(pos)
        peak_track_err = max(float(peak_track_err), abs(float(track_err)))

        if any([ax, mo, en, ct, oe]):
            err = f"axis={hex(ax)} motor={hex(mo)} enc={hex(en)} ctrl={hex(ct)} odrv={hex(oe)}"
            break
        if abs(pos - start) > float(effective_abort_abs_turns):
            err = f"runaway_abs_dev>{effective_abort_abs_turns:.6f}"
            break

        dpos = float(pos - prev_pos)
        monotonic_total += 1
        if move_sign == 0 or (float(dpos) * float(move_sign)) >= -1e-5:
            monotonic_good += 1
        if move_sign != 0 and (float(dpos) * float(move_sign)) < 0.0:
            backtrack_turns += abs(float(dpos))
        prev_pos = float(pos)

        if abs(float(track_err)) >= float(active_err_threshold):
            vel_sign = 1 if float(vel) > 0.05 else (-1 if float(vel) < -0.05 else 0)
            if prev_vel_sign != 0 and vel_sign != 0 and vel_sign != prev_vel_sign:
                active_vel_sign_flips += 1
            if vel_sign != 0:
                prev_vel_sign = vel_sign

        remaining = float(target) - float(pos)
        if abs(float(remaining)) <= float(handoff_window_turns):
            reached = True
            handoff_done = True
            reach_t = float(time.time())
            break

        abs_remaining = abs(float(remaining))
        slowdown_window = max(float(handoff_window_turns) * 3.0, 0.15)
        ramp = min(1.0, abs_remaining / max(1e-6, slowdown_window))
        cmd_abs = max(0.12, abs(float(command_vel_turns_s)) * ramp)
        axis.controller.input_vel = (1.0 if float(remaining) > 0.0 else -1.0) * float(cmd_abs)
        time.sleep(max(0.003, float(dt)))

    try:
        axis.controller.input_vel = 0.0
    except Exception:
        pass
    _restore_position_passthrough(axis)

    end = float(getattr(axis.encoder, "pos_estimate", start))
    if err is None and not handoff_done:
        err = (
            "handoff_window_not_reached_within_timeout "
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
        "effective_abort_abs_turns": float(effective_abort_abs_turns),
        "reached": bool(reached),
        "reach_time_s": (None if reach_t is None else float(reach_t - (deadline - max(0.05, float(timeout_s))))),
        "final_error": float(float(target) - float(end)),
        "final_error_abs": abs(float(float(target) - float(end))),
        "command_vel_turns_s": float(command_vel_turns_s),
        "handoff_window_turns": float(handoff_window_turns),
        "peak_track_err": float(peak_track_err),
        "monotonic_fraction": (0.0 if monotonic_total == 0 else float(monotonic_good) / float(monotonic_total)),
        "backtrack_turns": float(backtrack_turns),
        "active_vel_sign_flips": int(active_vel_sign_flips),
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


def run_directional_slew_move(
    *,
    odrv=None,
    axis=None,
    serial_number=None,
    axis_index=0,
    candidate_preset="mounted-direct-v3",
    delta_turns=None,
    target_turns=None,
    approach_offset_turns=None,
    timeout_s=10.0,
    pre_hold_s=0.25,
    final_hold_s=0.90,
    return_to_start=False,
    return_hold_s=0.90,
    abort_abs_turns=0.90,
    target_tolerance_turns=0.03,
    target_vel_tolerance_turns_s=0.20,
    command_vel_turns_s=0.35,
    handoff_window_turns=0.10,
    command_dt=0.01,
    travel_pos_gain=None,
    travel_vel_gain=None,
    travel_vel_i_gain=None,
    travel_vel_limit=None,
):
    if delta_turns is None and target_turns is None:
        raise ValueError("run_directional_slew_move requires delta_turns or target_turns")

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
    final_loop_cfg = _read_position_loop_config(axis)
    travel_loop_cfg = dict(final_loop_cfg)
    if any(v is not None for v in (travel_pos_gain, travel_vel_gain, travel_vel_i_gain, travel_vel_limit)):
        if travel_pos_gain is not None:
            travel_loop_cfg["pos_gain"] = float(travel_pos_gain)
        if travel_vel_gain is not None:
            travel_loop_cfg["vel_gain"] = float(travel_vel_gain)
        if travel_vel_i_gain is not None:
            travel_loop_cfg["vel_i_gain"] = float(travel_vel_i_gain)
        if travel_vel_limit is not None:
            travel_loop_cfg["vel_limit"] = float(travel_vel_limit)
        _apply_position_loop_config(axis, **travel_loop_cfg)

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
    direct_dist = max(1e-6, abs(float(target) - float(start_pos)))
    if pre_target is not None:
        pre_dist = abs(float(pre_target) - float(start_pos))
        pre_timeout_s = max(
            0.75,
            float(timeout_s) * float(pre_dist / max(1e-6, direct_dist + abs(float(pre_target) - float(target)))),
            float(pre_dist / max(1e-6, float(command_vel_turns_s))) + 0.75,
        )
        pre = _slew_to_target_direct(
            odrv,
            axis,
            pre_target,
            timeout_s=pre_timeout_s,
            abort_abs_turns=abort_abs_turns,
            command_vel_turns_s=command_vel_turns_s,
            handoff_window_turns=handoff_window_turns,
            target_tolerance_turns=target_tolerance_turns,
            target_vel_tolerance_turns_s=target_vel_tolerance_turns_s,
            dt=command_dt,
        )
        pre["stage"] = "preload_travel"
        stages.append(pre)
        if not pre["ok"]:
            raise RuntimeError(f"preload_failed: {pre['error']}")
        if float(pre_hold_s) > 0.0:
            pre_hold = _move_and_observe(odrv, axis, pre_target, hold_s=pre_hold_s, abort_abs_turns=abort_abs_turns)
            pre_hold["stage"] = "preload_hold"
            stages.append(pre_hold)

    cur_pos = float(getattr(axis.encoder, "pos_estimate", start_pos))
    final_dist = abs(float(target) - float(cur_pos))
    final_timeout_s = max(
        0.75,
        float(timeout_s) * float(final_dist / max(1e-6, direct_dist + (0.0 if pre_target is None else abs(float(pre_target) - float(start_pos))))),
        float(final_dist / max(1e-6, float(command_vel_turns_s))) + 0.75,
    )
    final = _slew_to_target_direct(
        odrv,
        axis,
        target,
        timeout_s=final_timeout_s,
        abort_abs_turns=abort_abs_turns,
        command_vel_turns_s=command_vel_turns_s,
        handoff_window_turns=handoff_window_turns,
        target_tolerance_turns=target_tolerance_turns,
        target_vel_tolerance_turns_s=target_vel_tolerance_turns_s,
        dt=command_dt,
    )
    final["stage"] = "target_travel"
    stages.append(final)
    if not final["ok"]:
        raise RuntimeError(f"target_failed: {final['error']}")

    _apply_position_loop_config(axis, **final_loop_cfg)
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
        "command_vel_turns_s": float(command_vel_turns_s),
        "handoff_window_turns": float(handoff_window_turns),
        "travel_loop_cfg": dict(travel_loop_cfg),
        "final_loop_cfg": dict(final_loop_cfg),
        "stages": stages,
    }

    if bool(return_to_start):
        _apply_position_loop_config(axis, **travel_loop_cfg)
        ret = _slew_to_target_direct(
            odrv,
            axis,
            start_pos,
            timeout_s=max(0.75, float(abs(float(start_pos) - float(end_pos)) / max(1e-6, float(command_vel_turns_s))) + 0.75),
            abort_abs_turns=abort_abs_turns,
            command_vel_turns_s=command_vel_turns_s,
            handoff_window_turns=handoff_window_turns,
            target_tolerance_turns=target_tolerance_turns,
            target_vel_tolerance_turns_s=target_vel_tolerance_turns_s,
            dt=command_dt,
        )
        ret["stage"] = "return_travel"
        result["return"] = ret
        result["return_pos"] = float(getattr(axis.encoder, "pos_estimate", start_pos))
        result["return_residual"] = float(result["return_pos"] - float(start_pos))
        _apply_position_loop_config(axis, **final_loop_cfg)

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


def run_directional_velocity_travel_move(
    *,
    odrv=None,
    axis=None,
    serial_number=None,
    axis_index=0,
    candidate_preset="mounted-direct-v3",
    delta_turns=None,
    target_turns=None,
    approach_offset_turns=None,
    timeout_s=10.0,
    pre_hold_s=0.10,
    final_hold_s=0.90,
    return_to_start=False,
    return_hold_s=0.90,
    abort_abs_turns=3.00,
    target_tolerance_turns=0.03,
    target_vel_tolerance_turns_s=0.20,
    command_vel_turns_s=4.00,
    handoff_window_turns=0.35,
    command_dt=0.01,
):
    if delta_turns is None and target_turns is None:
        raise ValueError("run_directional_velocity_travel_move requires delta_turns or target_turns")

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
    final_loop_cfg = _read_position_loop_config(axis)

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
    direct_dist = max(1e-6, abs(float(target) - float(start_pos)))
    if pre_target is not None:
        pre_dist = abs(float(pre_target) - float(start_pos))
        pre_timeout_s = max(
            0.75,
            float(timeout_s) * float(pre_dist / max(1e-6, direct_dist + abs(float(pre_target) - float(target)))),
            float(pre_dist / max(1e-6, float(command_vel_turns_s))) + 0.75,
        )
        pre = _velocity_travel_to_target(
            odrv,
            axis,
            pre_target,
            timeout_s=pre_timeout_s,
            abort_abs_turns=abort_abs_turns,
            command_vel_turns_s=command_vel_turns_s,
            handoff_window_turns=handoff_window_turns,
            dt=command_dt,
        )
        pre["stage"] = "preload_travel"
        stages.append(pre)
        if not pre["ok"]:
            raise RuntimeError(f"preload_failed: {pre['error']}")
        _apply_position_loop_config(axis, **final_loop_cfg)
        if float(pre_hold_s) > 0.0:
            pre_hold = _move_and_observe(odrv, axis, pre_target, hold_s=pre_hold_s, abort_abs_turns=abort_abs_turns)
            pre_hold["stage"] = "preload_hold"
            stages.append(pre_hold)

    cur_pos = float(getattr(axis.encoder, "pos_estimate", start_pos))
    final_dist = abs(float(target) - float(cur_pos))
    final_timeout_s = max(
        0.75,
        float(timeout_s) * float(final_dist / max(1e-6, direct_dist + (0.0 if pre_target is None else abs(float(pre_target) - float(start_pos))))),
        float(final_dist / max(1e-6, float(command_vel_turns_s))) + 0.75,
    )
    travel = _velocity_travel_to_target(
        odrv,
        axis,
        target,
        timeout_s=final_timeout_s,
        abort_abs_turns=abort_abs_turns,
        command_vel_turns_s=command_vel_turns_s,
        handoff_window_turns=handoff_window_turns,
        dt=command_dt,
    )
    travel["stage"] = "target_travel"
    stages.append(travel)
    if not travel["ok"]:
        raise RuntimeError(f"target_failed: {travel['error']}")

    _apply_position_loop_config(axis, **final_loop_cfg)
    if float(final_hold_s) > 0.0:
        hold = _move_and_observe(
            odrv,
            axis,
            target,
            hold_s=final_hold_s,
            abort_abs_turns=abort_abs_turns,
        )
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
        "command_vel_turns_s": float(command_vel_turns_s),
        "handoff_window_turns": float(handoff_window_turns),
        "final_loop_cfg": dict(final_loop_cfg),
        "stages": stages,
    }

    if bool(return_to_start):
        ret = _velocity_travel_to_target(
            odrv,
            axis,
            start_pos,
            timeout_s=max(0.75, float(abs(float(start_pos) - float(end_pos)) / max(1e-6, float(command_vel_turns_s))) + 0.75),
            abort_abs_turns=abort_abs_turns,
            command_vel_turns_s=command_vel_turns_s,
            handoff_window_turns=handoff_window_turns,
            dt=command_dt,
        )
        ret["stage"] = "return_travel"
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
