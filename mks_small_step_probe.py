#!/usr/bin/env python3
"""Isolate small-step behavior on the bare-motor MKS path.

This compares:
1. fresh-baseline single-step trials for +step and -step
2. single-baseline ordered sequences

The goal is to separate intrinsic directional weakness from
sequence/operating-point sensitivity.
"""

from __future__ import annotations

import argparse
import json
import statistics
from pathlib import Path

from mks_apply_runtime_baseline import apply_runtime_baseline
from mks_axis_characterize import resolve_odrv_axis
from mks_manual_passthrough_test import run_manual_passthrough_test


def _summarize_reports(reports):
    ratios = []
    residuals = []
    settled = 0
    for report in reports:
        trial = report["best_candidate"]["trials"][0]
        if trial.get("track_ratio") is not None:
            ratios.append(float(trial["track_ratio"]))
        if trial.get("return_residual") is not None:
            residuals.append(float(trial["return_residual"]))
        if trial.get("return_settled") is True:
            settled += 1
    return {
        "n": len(reports),
        "all_ok_runs": sum(
            1 for report in reports if all(bool(t.get("ok")) for t in report["best_candidate"]["trials"])
        ),
        "track_ratio_mean": (statistics.mean(ratios) if ratios else None),
        "track_ratio_min": (min(ratios) if ratios else None),
        "track_ratio_max": (max(ratios) if ratios else None),
        "track_ratio_stdev": (statistics.pstdev(ratios) if len(ratios) > 1 else 0.0 if ratios else None),
        "return_residual_mean": (statistics.mean(residuals) if residuals else None),
        "return_residual_max_abs": (max((abs(v) for v in residuals), default=None)),
        "return_settled_count": settled,
    }


def _single_step_fresh_baseline_runs(
    *,
    odrv,
    axis_index,
    candidate_preset,
    candidate_current_lim,
    candidate_pos_gain,
    candidate_vel_gain,
    candidate_vel_i_gain,
    candidate_vel_limit,
    step,
    repeats,
    hold_s,
    abort_abs_turns,
    return_settle_tol,
    return_settle_timeout,
):
    runs = []
    for idx in range(int(repeats)):
        baseline = apply_runtime_baseline(
            odrv=odrv,
            axis_index=axis_index,
            preset=candidate_preset,
            current_lim=candidate_current_lim,
            pos_gain=candidate_pos_gain,
            vel_gain=candidate_vel_gain,
            vel_i_gain=candidate_vel_i_gain,
            vel_limit=candidate_vel_limit,
        )
        report = run_manual_passthrough_test(
            odrv=odrv,
            axis_index=axis_index,
            candidate_preset=candidate_preset,
            candidate_current_lim=candidate_current_lim,
            candidate_pos_gain=candidate_pos_gain,
            candidate_vel_gain=candidate_vel_gain,
            candidate_vel_i_gain=candidate_vel_i_gain,
            candidate_vel_limit=candidate_vel_limit,
            steps=[step],
            hold_s=hold_s,
            abort_abs_turns=abort_abs_turns,
            apply_baseline=False,
            return_settle_tol=return_settle_tol,
            return_settle_timeout=return_settle_timeout,
        )
        runs.append(
            {
                "repeat_index": idx + 1,
                "baseline_snapshot": baseline.get("snapshot"),
                "best_candidate": report["best_candidate"],
            }
        )
    return {
        "mode": "fresh_baseline_single_step",
        "step": float(step),
        "repeats": int(repeats),
        "runs": runs,
        "summary": _summarize_reports(runs),
    }


def _single_baseline_sequence_runs(
    *,
    odrv,
    axis_index,
    candidate_preset,
    candidate_current_lim,
    candidate_pos_gain,
    candidate_vel_gain,
    candidate_vel_i_gain,
    candidate_vel_limit,
    sequence_name,
    steps,
    repeats,
    hold_s,
    abort_abs_turns,
    return_settle_tol,
    return_settle_timeout,
):
    apply_runtime_baseline(
        odrv=odrv,
        axis_index=axis_index,
        preset=candidate_preset,
        current_lim=candidate_current_lim,
        pos_gain=candidate_pos_gain,
        vel_gain=candidate_vel_gain,
        vel_i_gain=candidate_vel_i_gain,
        vel_limit=candidate_vel_limit,
    )

    runs = []
    for idx in range(int(repeats)):
        report = run_manual_passthrough_test(
            odrv=odrv,
            axis_index=axis_index,
            candidate_preset=candidate_preset,
            candidate_current_lim=candidate_current_lim,
            candidate_pos_gain=candidate_pos_gain,
            candidate_vel_gain=candidate_vel_gain,
            candidate_vel_i_gain=candidate_vel_i_gain,
            candidate_vel_limit=candidate_vel_limit,
            steps=steps,
            hold_s=hold_s,
            abort_abs_turns=abort_abs_turns,
            apply_baseline=False,
            return_settle_tol=return_settle_tol,
            return_settle_timeout=return_settle_timeout,
        )
        runs.append(
            {
                "repeat_index": idx + 1,
                "best_candidate": report["best_candidate"],
            }
        )

    return {
        "mode": "single_baseline_sequence",
        "sequence_name": sequence_name,
        "steps": [float(v) for v in steps],
        "repeats": int(repeats),
        "runs": runs,
    }


def run_small_step_probe(
    *,
    serial_number=None,
    axis_index=0,
    candidate_preset="bare-pos-repeatable-v1",
    candidate_current_lim=None,
    candidate_pos_gain=None,
    candidate_vel_gain=None,
    candidate_vel_i_gain=None,
    candidate_vel_limit=None,
    step=0.05,
    fresh_repeats=3,
    sequence_repeats=3,
    hold_s=0.90,
    abort_abs_turns=0.25,
    return_settle_tol=0.005,
    return_settle_timeout=1.5,
    out_path=None,
):
    odrv, _ = resolve_odrv_axis(
        serial_number=serial_number,
        axis_index=axis_index,
        timeout_s=10.0,
    )

    report = {
        "board_serial": str(getattr(odrv, "serial_number", "")),
        "axis_index": int(axis_index),
        "candidate": {
            "preset": candidate_preset,
            "current_lim": candidate_current_lim,
            "pos_gain": candidate_pos_gain,
            "vel_gain": candidate_vel_gain,
            "vel_i_gain": candidate_vel_i_gain,
            "vel_limit": candidate_vel_limit,
        },
        "step": float(step),
        "hold_s": float(hold_s),
        "abort_abs_turns": float(abort_abs_turns),
        "return_settle_tol": float(return_settle_tol),
        "return_settle_timeout": float(return_settle_timeout),
        "fresh_plus": _single_step_fresh_baseline_runs(
            odrv=odrv,
            axis_index=axis_index,
            candidate_preset=candidate_preset,
            candidate_current_lim=candidate_current_lim,
            candidate_pos_gain=candidate_pos_gain,
            candidate_vel_gain=candidate_vel_gain,
            candidate_vel_i_gain=candidate_vel_i_gain,
            candidate_vel_limit=candidate_vel_limit,
            step=abs(float(step)),
            repeats=fresh_repeats,
            hold_s=hold_s,
            abort_abs_turns=abort_abs_turns,
            return_settle_tol=return_settle_tol,
            return_settle_timeout=return_settle_timeout,
        ),
        "fresh_minus": _single_step_fresh_baseline_runs(
            odrv=odrv,
            axis_index=axis_index,
            candidate_preset=candidate_preset,
            candidate_current_lim=candidate_current_lim,
            candidate_pos_gain=candidate_pos_gain,
            candidate_vel_gain=candidate_vel_gain,
            candidate_vel_i_gain=candidate_vel_i_gain,
            candidate_vel_limit=candidate_vel_limit,
            step=-abs(float(step)),
            repeats=fresh_repeats,
            hold_s=hold_s,
            abort_abs_turns=abort_abs_turns,
            return_settle_tol=return_settle_tol,
            return_settle_timeout=return_settle_timeout,
        ),
        "sequence_forward": _single_baseline_sequence_runs(
            odrv=odrv,
            axis_index=axis_index,
            candidate_preset=candidate_preset,
            candidate_current_lim=candidate_current_lim,
            candidate_pos_gain=candidate_pos_gain,
            candidate_vel_gain=candidate_vel_gain,
            candidate_vel_i_gain=candidate_vel_i_gain,
            candidate_vel_limit=candidate_vel_limit,
            sequence_name="forward",
            steps=[+abs(float(step)), -abs(float(step)), +0.10, -0.10],
            repeats=sequence_repeats,
            hold_s=hold_s,
            abort_abs_turns=abort_abs_turns,
            return_settle_tol=return_settle_tol,
            return_settle_timeout=return_settle_timeout,
        ),
        "sequence_reverse": _single_baseline_sequence_runs(
            odrv=odrv,
            axis_index=axis_index,
            candidate_preset=candidate_preset,
            candidate_current_lim=candidate_current_lim,
            candidate_pos_gain=candidate_pos_gain,
            candidate_vel_gain=candidate_vel_gain,
            candidate_vel_i_gain=candidate_vel_i_gain,
            candidate_vel_limit=candidate_vel_limit,
            sequence_name="reverse",
            steps=[-abs(float(step)), +abs(float(step)), -0.10, +0.10],
            repeats=sequence_repeats,
            hold_s=hold_s,
            abort_abs_turns=abort_abs_turns,
            return_settle_tol=return_settle_tol,
            return_settle_timeout=return_settle_timeout,
        ),
    }

    if out_path:
        out = Path(out_path).resolve()
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        report["report_saved"] = str(out)

    return report


def main():
    ap = argparse.ArgumentParser(description="Probe MKS bare-motor small-step asymmetry")
    ap.add_argument("--serial-number", default="", help="Optional board serial. Blank uses first found.")
    ap.add_argument("--axis-index", type=int, default=0)
    ap.add_argument("--candidate-preset", default="bare-pos-repeatable-v1")
    ap.add_argument("--candidate-current-lim", type=float, default=None)
    ap.add_argument("--candidate-pos-gain", type=float, default=None)
    ap.add_argument("--candidate-vel-gain", type=float, default=None)
    ap.add_argument("--candidate-vel-i-gain", type=float, default=None)
    ap.add_argument("--candidate-vel-limit", type=float, default=None)
    ap.add_argument("--step", type=float, default=0.05)
    ap.add_argument("--fresh-repeats", type=int, default=3)
    ap.add_argument("--sequence-repeats", type=int, default=3)
    ap.add_argument("--hold-s", type=float, default=0.90)
    ap.add_argument("--abort-abs-turns", type=float, default=0.25)
    ap.add_argument("--return-settle-tol", type=float, default=0.005)
    ap.add_argument("--return-settle-timeout", type=float, default=1.5)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    report = run_small_step_probe(
        serial_number=(str(args.serial_number).strip() or None),
        axis_index=int(args.axis_index),
        candidate_preset=str(args.candidate_preset),
        candidate_current_lim=args.candidate_current_lim,
        candidate_pos_gain=args.candidate_pos_gain,
        candidate_vel_gain=args.candidate_vel_gain,
        candidate_vel_i_gain=args.candidate_vel_i_gain,
        candidate_vel_limit=args.candidate_vel_limit,
        step=float(args.step),
        fresh_repeats=int(args.fresh_repeats),
        sequence_repeats=int(args.sequence_repeats),
        hold_s=float(args.hold_s),
        abort_abs_turns=float(args.abort_abs_turns),
        return_settle_tol=float(args.return_settle_tol),
        return_settle_timeout=float(args.return_settle_timeout),
        out_path=args.out,
    )
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
