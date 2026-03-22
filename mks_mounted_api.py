#!/usr/bin/env python3
"""Import-friendly mounted MKS API.

This module is the preferred entrypoint for mounted gearbox work from
odrivetool/IPython or small local scripts. It wraps the current best mounted
runtime preset and the measured directional-preload move rule.
"""

from __future__ import annotations

from mks_apply_runtime_baseline import apply_runtime_baseline
from mks_axis_characterize import resolve_odrv_axis
from mks_mounted_directional_move import run_directional_move
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
    approach_offset_turns=0.10,
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
        approach_offset_turns=float(approach_offset_turns),
        pre_hold_s=float(pre_hold_s),
        final_hold_s=float(final_hold_s),
        return_to_start=bool(return_to_start),
        return_hold_s=float(return_hold_s),
        abort_abs_turns=float(abort_abs_turns),
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
    approach_offset_turns=0.10,
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
        approach_offset_turns=float(approach_offset_turns),
        cycles=int(cycles),
        pre_hold_s=float(pre_hold_s),
        final_hold_s=float(final_hold_s),
        return_hold_s=float(return_hold_s),
        abort_abs_turns=float(abort_abs_turns),
        out_path=out_path,
    )


def mounted_plan():
    """Return the current mounted plan and operating rule as plain data."""
    return {
        "current_best_preset": DEFAULT_MOUNTED_PRESET,
        "directional_rule": {
            "positive_delta": "from_above",
            "negative_delta": "from_below",
        },
        "known_boundaries": [
            "trap/operator path is still not usable on the mounted MKS gearbox path",
            "motor-side encoder plus printed harmonic-drive hysteresis still limits return precision",
        ],
        "next_steps": [
            "use directional preload for mounted direct-position tests",
            "sweep across more starting positions and target magnitudes",
            "only then consider backend/UI integration of the mounted directional move path",
        ],
    }
