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
    pre_hold_s=0.70,
    final_hold_s=0.90,
    return_to_start=False,
    return_hold_s=0.90,
    abort_abs_turns=0.90,
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
        pre = _move_and_observe(odrv, axis, pre_target, hold_s=pre_hold_s, abort_abs_turns=abort_abs_turns)
        pre["stage"] = "preload"
        stages.append(pre)
        if not pre["ok"]:
            raise RuntimeError(f"preload_failed: {pre['error']}")

    final = _move_and_observe(odrv, axis, target, hold_s=final_hold_s, abort_abs_turns=abort_abs_turns)
    final["stage"] = "target"
    stages.append(final)
    if not final["ok"]:
        raise RuntimeError(f"target_failed: {final['error']}")

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
