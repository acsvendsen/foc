#!/usr/bin/env python3
"""Probe mounted MKS directional-preload performance on absolute targets.

This treats the mounted directional move helper as the intended primitive and
measures outbound/restore accuracy across multiple starting positions and
absolute target positions relative to the current anchor.
"""

from __future__ import annotations

import argparse
import json
import statistics
from pathlib import Path

from mks_apply_runtime_baseline import apply_runtime_baseline
from mks_axis_characterize import resolve_odrv_axis
from mks_mounted_directional_move import run_directional_move


def _parse_floats(value: str):
    return [float(part.strip()) for part in str(value).split(",") if part.strip()]


def _summarize_moves(moves):
    ok = [move for move in moves if move.get("ok", True)]
    errs = [float(move.get("final_error", 0.0)) for move in ok if move.get("final_error") is not None]
    abs_errs = [abs(v) for v in errs]
    return {
        "runs": len(moves),
        "ok_runs": len(ok),
        "mean_err": (statistics.mean(errs) if errs else None),
        "mean_abs_err": (statistics.mean(abs_errs) if abs_errs else None),
        "spread": ((max(errs) - min(errs)) if errs else None),
        "max_abs_err": (max(abs_errs) if abs_errs else None),
    }


def run_mounted_absolute_target_probe(
    *,
    odrv=None,
    axis=None,
    serial_number=None,
    axis_index=0,
    preset="mounted-direct-v3",
    start_offsets=None,
    target_offsets=None,
    cycles=2,
    pre_hold_s=0.70,
    final_hold_s=0.90,
    abort_abs_turns=0.90,
    out_path=None,
):
    start_offsets = list(start_offsets if start_offsets is not None else [0.0, 0.50, -0.50])
    target_offsets = list(target_offsets if target_offsets is not None else [-0.50, -0.25, 0.25, 0.50])

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
    anchor = float(getattr(axis.encoder, "pos_estimate", 0.0))

    report = {
        "board_serial": str(getattr(odrv, "serial_number", "")),
        "axis_index": int(axis_index),
        "preset": str(preset),
        "anchor": float(anchor),
        "start_offsets": [float(v) for v in start_offsets],
        "target_offsets": [float(v) for v in target_offsets],
        "cycles": int(cycles),
        "pre_hold_s": float(pre_hold_s),
        "final_hold_s": float(final_hold_s),
        "abort_abs_turns": float(abort_abs_turns),
        "baseline_snapshot": baseline.get("snapshot"),
        "pairs": [],
    }

    for start_offset in start_offsets:
        start_target = float(anchor + float(start_offset))
        start_establish = run_directional_move(
            odrv=odrv,
            axis_index=axis_index,
            target_turns=start_target,
            candidate_preset=str(preset),
            return_to_start=False,
            pre_hold_s=float(pre_hold_s),
            final_hold_s=float(final_hold_s),
            abort_abs_turns=float(abort_abs_turns),
        )
        actual_start = float(getattr(axis.encoder, "pos_estimate", start_target))

        for target_offset in target_offsets:
            target_abs = float(anchor + float(target_offset))
            if abs(target_abs - start_target) < 1e-9:
                continue

            outbound_moves = []
            restore_moves = []
            for idx in range(int(cycles)):
                if idx > 0:
                    run_directional_move(
                        odrv=odrv,
                        axis_index=axis_index,
                        target_turns=start_target,
                        candidate_preset=str(preset),
                        return_to_start=False,
                        pre_hold_s=float(pre_hold_s),
                        final_hold_s=float(final_hold_s),
                        abort_abs_turns=float(abort_abs_turns),
                    )
                outbound = run_directional_move(
                    odrv=odrv,
                    axis_index=axis_index,
                    target_turns=target_abs,
                    candidate_preset=str(preset),
                    return_to_start=False,
                    pre_hold_s=float(pre_hold_s),
                    final_hold_s=float(final_hold_s),
                    abort_abs_turns=float(abort_abs_turns),
                )
                outbound["cycle_index"] = idx + 1
                outbound_moves.append(outbound)

                restore = run_directional_move(
                    odrv=odrv,
                    axis_index=axis_index,
                    target_turns=start_target,
                    candidate_preset=str(preset),
                    return_to_start=False,
                    pre_hold_s=float(pre_hold_s),
                    final_hold_s=float(final_hold_s),
                    abort_abs_turns=float(abort_abs_turns),
                )
                restore["cycle_index"] = idx + 1
                restore_moves.append(restore)

            pair = {
                "start_offset": float(start_offset),
                "target_offset": float(target_offset),
                "start_target": float(start_target),
                "target_abs": float(target_abs),
                "start_establish": {
                    "approach_mode": start_establish.get("approach_mode"),
                    "final_error": start_establish.get("final_error"),
                },
                "actual_start": float(actual_start),
                "base_error": float(actual_start - start_target),
                "outbound": outbound_moves,
                "restore": restore_moves,
                "outbound_summary": _summarize_moves(outbound_moves),
                "restore_summary": _summarize_moves(restore_moves),
            }
            report["pairs"].append(pair)
            print(
                json.dumps(
                    {
                        "start_offset": float(start_offset),
                        "target_offset": float(target_offset),
                        "outbound_mean_abs_err": pair["outbound_summary"]["mean_abs_err"],
                        "restore_mean_abs_err": pair["restore_summary"]["mean_abs_err"],
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
    ap = argparse.ArgumentParser(description="Probe mounted directional preload performance on absolute targets")
    ap.add_argument("--serial-number", default="")
    ap.add_argument("--axis-index", type=int, default=0)
    ap.add_argument("--preset", default="mounted-direct-v3")
    ap.add_argument("--start-offsets", default="0.0,0.50,-0.50")
    ap.add_argument("--target-offsets", default="-0.50,-0.25,0.25,0.50")
    ap.add_argument("--cycles", type=int, default=2)
    ap.add_argument("--pre-hold-s", type=float, default=0.70)
    ap.add_argument("--final-hold-s", type=float, default=0.90)
    ap.add_argument("--abort-abs-turns", type=float, default=0.90)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    run_mounted_absolute_target_probe(
        serial_number=(str(args.serial_number).strip() or None),
        axis_index=int(args.axis_index),
        preset=str(args.preset),
        start_offsets=_parse_floats(args.start_offsets),
        target_offsets=_parse_floats(args.target_offsets),
        cycles=int(args.cycles),
        pre_hold_s=float(args.pre_hold_s),
        final_hold_s=float(args.final_hold_s),
        abort_abs_turns=float(args.abort_abs_turns),
        out_path=args.out,
    )


if __name__ == "__main__":
    main()
