#!/usr/bin/env python3
"""Validate the mounted directional preload rule across starting positions."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from mks_apply_runtime_baseline import apply_runtime_baseline
from mks_axis_characterize import resolve_odrv_axis
from mks_mounted_directional_move import run_directional_move
from mks_mounted_preload_probe import run_mounted_preload_probe


def _parse_floats(value: str):
    return [float(part.strip()) for part in str(value).split(",") if part.strip()]


def _best_by_delta(cases):
    grouped = {}
    for case in cases:
        grouped.setdefault(float(case["delta_cmd"]), []).append(case)
    best = {}
    for delta, items in grouped.items():
        ranked = sorted(
            items,
            key=lambda item: float(item.get("summary", {}).get("mean_abs_err") or 1e9),
        )
        winner = ranked[0]
        best[str(delta)] = {
            "approach_mode": winner["approach_mode"],
            "mean_abs_err": winner.get("summary", {}).get("mean_abs_err"),
            "mean_err": winner.get("summary", {}).get("mean_err"),
            "spread": winner.get("summary", {}).get("spread"),
        }
    return best


def run_mounted_directional_validation(
    *,
    odrv=None,
    axis=None,
    serial_number=None,
    axis_index=0,
    preset="mounted-direct-v3",
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
    start_offsets = list(start_offsets if start_offsets is not None else [0.0, 0.50, -0.50])
    target_deltas = list(target_deltas if target_deltas is not None else [0.10, -0.10, 0.25, -0.25, 0.50, -0.50])
    approach_modes = list(approach_modes if approach_modes is not None else ["direct", "from_below", "from_above"])

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
        preset=str(preset),
        reuse_existing_calibration=True,
    )
    initial_base = float(getattr(axis.encoder, "pos_estimate", 0.0))

    report = {
        "board_serial": str(getattr(odrv, "serial_number", "")),
        "axis_index": int(axis_index),
        "preset": str(preset),
        "initial_base": float(initial_base),
        "start_offsets": [float(v) for v in start_offsets],
        "target_deltas": [float(v) for v in target_deltas],
        "approach_modes": [str(v) for v in approach_modes],
        "approach_offset_turns": (
            None if approach_offset_turns is None else float(approach_offset_turns)
        ),
        "cycles": int(cycles),
        "baseline_snapshot": baseline.get("snapshot"),
        "positions": [],
    }

    for start_offset in start_offsets:
        target_base = float(initial_base + float(start_offset))
        move_to_base = run_directional_move(
            odrv=odrv,
            axis_index=axis_index,
            target_turns=float(target_base),
            candidate_preset=str(preset),
            approach_offset_turns=(None if approach_offset_turns is None else float(approach_offset_turns)),
            pre_hold_s=float(pre_hold_s),
            final_hold_s=float(final_hold_s),
            return_to_start=False,
            abort_abs_turns=float(abort_abs_turns),
        )
        actual_base = float(getattr(axis.encoder, "pos_estimate", target_base))
        probe = run_mounted_preload_probe(
            odrv=odrv,
            axis=axis,
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
        )
        record = {
            "start_offset": float(start_offset),
            "target_base": float(target_base),
            "move_to_base": {
                "approach_mode": move_to_base.get("approach_mode"),
                "final_error": move_to_base.get("final_error"),
            },
            "actual_base": float(actual_base),
            "base_error": float(actual_base - target_base),
            "cases": probe.get("cases", []),
            "best_by_delta": _best_by_delta(probe.get("cases", [])),
        }
        report["positions"].append(record)
        print(
            json.dumps(
                {
                    "start_offset": float(start_offset),
                    "base_error": record["base_error"],
                    "best_by_delta": record["best_by_delta"],
                },
                indent=2,
            ),
            flush=True,
        )

    if out_path:
        Path(out_path).write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(f"report_saved={Path(out_path).resolve()}", flush=True)
    return report


def main():
    ap = argparse.ArgumentParser(description="Validate mounted directional preload rule across starting positions")
    ap.add_argument("--serial-number", default="")
    ap.add_argument("--axis-index", type=int, default=0)
    ap.add_argument("--preset", default="mounted-direct-v3")
    ap.add_argument("--start-offsets", default="0.0,0.50,-0.50")
    ap.add_argument("--target-deltas", default="0.10,-0.10,0.25,-0.25,0.50,-0.50")
    ap.add_argument("--approach-modes", default="direct,from_below,from_above")
    ap.add_argument("--approach-offset-turns", type=float, default=None)
    ap.add_argument("--cycles", type=int, default=3)
    ap.add_argument("--pre-hold-s", type=float, default=0.70)
    ap.add_argument("--final-hold-s", type=float, default=0.90)
    ap.add_argument("--return-hold-s", type=float, default=0.90)
    ap.add_argument("--abort-abs-turns", type=float, default=0.90)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    run_mounted_directional_validation(
        serial_number=(str(args.serial_number).strip() or None),
        axis_index=int(args.axis_index),
        preset=str(args.preset),
        start_offsets=_parse_floats(args.start_offsets),
        target_deltas=_parse_floats(args.target_deltas),
        approach_modes=[part.strip() for part in str(args.approach_modes).split(",") if part.strip()],
        approach_offset_turns=(None if args.approach_offset_turns is None else float(args.approach_offset_turns)),
        cycles=int(args.cycles),
        pre_hold_s=float(args.pre_hold_s),
        final_hold_s=float(args.final_hold_s),
        return_hold_s=float(args.return_hold_s),
        abort_abs_turns=float(args.abort_abs_turns),
        out_path=args.out,
    )


if __name__ == "__main__":
    main()
