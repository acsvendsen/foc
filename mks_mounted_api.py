#!/usr/bin/env python3
"""Import-friendly mounted MKS API.

This module is the preferred entrypoint for mounted gearbox work from
odrivetool/IPython or small local scripts. It wraps the current best mounted
runtime preset and the measured directional-preload move rule.
"""

from __future__ import annotations

from mks_apply_runtime_baseline import apply_runtime_baseline
from mks_direct_profile_diagnostic_sweep import run_direct_profile_diagnostic_sweep
from mks_axis_characterize import resolve_odrv_axis
from mks_mounted_absolute_target_probe import run_mounted_absolute_target_probe
from mks_mounted_directional_move import run_directional_move, run_directional_slew_move
from mks_mounted_slew_compare import run_mounted_slew_compare
from mks_mounted_directional_validation import run_mounted_directional_validation
from mks_mounted_preload_probe import run_mounted_preload_probe


DEFAULT_MOUNTED_PRESET = "mounted-direct-v3"


def connect_mounted(*, odrv=None, axis=None, serial_number=None, axis_index=0, timeout_s=10.0):
    """Resolve a live mounted MKS board/axis handle."""
    return resolve_odrv_axis(
        odrv=odrv,
        axis=axis,
        serial_number=serial_number,
        axis_index=axis_index,
        timeout_s=timeout_s,
    )


def apply_mounted_preset(
    *,
    odrv=None,
    axis=None,
    serial_number=None,
    axis_index=0,
    preset=DEFAULT_MOUNTED_PRESET,
):
    """Apply the current mounted runtime preset using existing calibration."""
    return apply_runtime_baseline(
        odrv=odrv,
        axis=axis,
        serial_number=serial_number,
        axis_index=axis_index,
        preset=str(preset),
        reuse_existing_calibration=True,
    )


def mounted_move(
    *,
    odrv=None,
    axis=None,
    serial_number=None,
    axis_index=0,
    delta_turns=None,
    target_turns=None,
    preset=DEFAULT_MOUNTED_PRESET,
    approach_offset_turns=None,
    pre_hold_s=0.70,
    final_hold_s=0.90,
    return_to_start=False,
    return_hold_s=0.90,
    abort_abs_turns=0.90,
):
    """Run one mounted direct-position move with the directional preload rule."""
    return run_directional_move(
        odrv=odrv,
        axis=axis,
        serial_number=serial_number,
        axis_index=axis_index,
        candidate_preset=str(preset),
        delta_turns=delta_turns,
        target_turns=target_turns,
        approach_offset_turns=(None if approach_offset_turns is None else float(approach_offset_turns)),
        pre_hold_s=float(pre_hold_s),
        final_hold_s=float(final_hold_s),
        return_to_start=bool(return_to_start),
        return_hold_s=float(return_hold_s),
        abort_abs_turns=float(abort_abs_turns),
    )


def mounted_slew_move(
    *,
    odrv=None,
    axis=None,
    serial_number=None,
    axis_index=0,
    delta_turns=None,
    target_turns=None,
    preset=DEFAULT_MOUNTED_PRESET,
    approach_offset_turns=None,
    pre_hold_s=0.25,
    final_hold_s=0.90,
    return_to_start=False,
    return_hold_s=0.90,
    abort_abs_turns=3.0,
    command_vel_turns_s=0.30,
    handoff_window_turns=0.10,
    command_dt=0.01,
    travel_pos_gain=None,
    travel_vel_gain=None,
    travel_vel_i_gain=None,
    travel_vel_limit=None,
):
    """Run one mounted direct-position move with directional preload and slewed travel."""
    return run_directional_slew_move(
        odrv=odrv,
        axis=axis,
        serial_number=serial_number,
        axis_index=axis_index,
        candidate_preset=str(preset),
        delta_turns=delta_turns,
        target_turns=target_turns,
        approach_offset_turns=(None if approach_offset_turns is None else float(approach_offset_turns)),
        pre_hold_s=float(pre_hold_s),
        final_hold_s=float(final_hold_s),
        return_to_start=bool(return_to_start),
        return_hold_s=float(return_hold_s),
        abort_abs_turns=float(abort_abs_turns),
        command_vel_turns_s=float(command_vel_turns_s),
        handoff_window_turns=float(handoff_window_turns),
        command_dt=float(command_dt),
        travel_pos_gain=(None if travel_pos_gain is None else float(travel_pos_gain)),
        travel_vel_gain=(None if travel_vel_gain is None else float(travel_vel_gain)),
        travel_vel_i_gain=(None if travel_vel_i_gain is None else float(travel_vel_i_gain)),
        travel_vel_limit=(None if travel_vel_limit is None else float(travel_vel_limit)),
    )


def mounted_slew_compare(
    *,
    odrv=None,
    axis=None,
    serial_number=None,
    axis_index=0,
    preset=DEFAULT_MOUNTED_PRESET,
    delta_turns=2.0,
    cycles=2,
    pre_hold_s=0.25,
    final_hold_s=0.90,
    abort_abs_turns=3.0,
    timeout_s=12.0,
    command_vel_turns_s=0.30,
    handoff_window_turns=0.10,
    command_dt=0.01,
    out_path=None,
):
    """Compare direct-step versus slew-limited mounted travel on the same preset."""
    return run_mounted_slew_compare(
        odrv=odrv,
        axis=axis,
        serial_number=serial_number,
        axis_index=axis_index,
        candidate_preset=str(preset),
        delta_turns=float(delta_turns),
        cycles=int(cycles),
        pre_hold_s=float(pre_hold_s),
        final_hold_s=float(final_hold_s),
        abort_abs_turns=float(abort_abs_turns),
        timeout_s=float(timeout_s),
        command_vel_turns_s=float(command_vel_turns_s),
        handoff_window_turns=float(handoff_window_turns),
        command_dt=float(command_dt),
        out_path=out_path,
    )


def mounted_preload_sweep(
    *,
    odrv=None,
    axis=None,
    serial_number=None,
    axis_index=0,
    preset=DEFAULT_MOUNTED_PRESET,
    target_deltas=None,
    approach_modes=None,
    approach_offset_turns=None,
    cycles=3,
    pre_hold_s=0.70,
    final_hold_s=0.90,
    return_hold_s=0.90,
    abort_abs_turns=0.90,
    out_path=None,
):
    """Run the mounted preload probe using the current mounted preset."""
    return run_mounted_preload_probe(
        odrv=odrv,
        axis=axis,
        serial_number=serial_number,
        axis_index=axis_index,
        candidate_preset=str(preset),
        target_deltas=target_deltas,
        approach_modes=approach_modes,
        approach_offset_turns=(None if approach_offset_turns is None else float(approach_offset_turns)),
        cycles=int(cycles),
        pre_hold_s=float(pre_hold_s),
        final_hold_s=float(final_hold_s),
        return_hold_s=float(return_hold_s),
        abort_abs_turns=float(abort_abs_turns),
        out_path=out_path,
    )


def mounted_directional_validation(
    *,
    odrv=None,
    axis=None,
    serial_number=None,
    axis_index=0,
    preset=DEFAULT_MOUNTED_PRESET,
    start_offsets=None,
    target_deltas=None,
    approach_modes=None,
    approach_offset_turns=None,
    cycles=3,
    pre_hold_s=0.70,
    final_hold_s=0.90,
    return_hold_s=0.90,
    abort_abs_turns=0.90,
    out_path=None,
):
    """Validate the mounted directional rule across shifted starting positions."""
    return run_mounted_directional_validation(
        odrv=odrv,
        axis=axis,
        serial_number=serial_number,
        axis_index=axis_index,
        preset=str(preset),
        start_offsets=start_offsets,
        target_deltas=target_deltas,
        approach_modes=approach_modes,
        approach_offset_turns=(None if approach_offset_turns is None else float(approach_offset_turns)),
        cycles=int(cycles),
        pre_hold_s=float(pre_hold_s),
        final_hold_s=float(final_hold_s),
        return_hold_s=float(return_hold_s),
        abort_abs_turns=float(abort_abs_turns),
        out_path=out_path,
    )


def mounted_absolute_target_probe(
    *,
    odrv=None,
    axis=None,
    serial_number=None,
    axis_index=0,
    preset=DEFAULT_MOUNTED_PRESET,
    start_offsets=None,
    target_offsets=None,
    cycles=2,
    pre_hold_s=0.70,
    final_hold_s=0.90,
    abort_abs_turns=0.90,
    out_path=None,
):
    """Probe absolute mounted target moves using the current directional rule."""
    return run_mounted_absolute_target_probe(
        odrv=odrv,
        axis=axis,
        serial_number=serial_number,
        axis_index=axis_index,
        preset=str(preset),
        start_offsets=start_offsets,
        target_offsets=target_offsets,
        cycles=int(cycles),
        pre_hold_s=float(pre_hold_s),
        final_hold_s=float(final_hold_s),
        abort_abs_turns=float(abort_abs_turns),
        out_path=out_path,
    )


def mounted_diagnostic_sweep(
    *,
    odrv=None,
    axis=None,
    serial_number=None,
    axis_index=0,
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
):
    """Run the diagnostic-first mounted direct-profile sweep."""
    return run_direct_profile_diagnostic_sweep(
        odrv=odrv,
        axis=axis,
        serial_number=serial_number,
        axis_index=axis_index,
        mode="mounted",
        candidates=candidates,
        baseline_preset=str(baseline_preset),
        sign_step_turns=float(sign_step_turns),
        sign_hold_s=float(sign_hold_s),
        move_delta_turns=float(move_delta_turns),
        move_window_s=float(move_window_s),
        final_hold_s=float(final_hold_s),
        dt_s=float(dt_s),
        abort_abs_turns=float(abort_abs_turns),
        out_path=out_path,
    )


def mounted_plan():
    """Return the current mounted plan and operating rule as plain data."""
    return {
        "current_best_preset": DEFAULT_MOUNTED_PRESET,
        "experimental_runtime_presets": [
            "mounted-direct-soft-v4-exp",
        ],
        "experimental_move_profiles": [
            "mks_mounted_direct_slew_v1_exp",
            "mks_mounted_direct_slew_staged_v2_exp",
        ],
        "directional_rule": {
            "positive_delta": "from_above",
            "negative_delta": "from_below",
        },
        "preload_offset_rule": "if abs(delta_turns) >= 0.90: 0.20 else clamp(0.30 * abs(delta_turns), 0.10, 0.15)",
        "known_boundaries": [
            "trap/operator path is still not usable on the mounted MKS gearbox path",
            "motor-side encoder plus printed harmonic-drive hysteresis still limits return precision",
            "the shared direct-position foundation can still shake/hunt; use diagnostics before promoting new presets",
        ],
        "next_steps": [
            "run mounted_diagnostic_sweep() before promoting any direct-position preset",
            "use directional preload for mounted direct-position tests",
            "use mounted_slew_move() to test whether shaped travel reduces hunting during long moves",
            "use mounted_slew_compare() to compare direct-step versus slew-limited travel on the same preset",
            "use mounted absolute target probes to map the usable travel envelope",
            "only then consider backend/UI integration of the mounted directional move path",
        ],
    }
