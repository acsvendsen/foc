#!/usr/bin/env python3
"""Mounted MKS direct-position hysteresis probe.

This is for the gearbox-mounted path only. It assumes:
- motor-side encoder
- mounted harmonic drive / compliance
- known-good existing calibration that should be reused

The goal is to compare mounted direct-position candidates on:
1. outbound authority (track ratio)
2. return residual / hysteresis
3. repeatability

This is intentionally not a trap-profile runner.
"""

from __future__ import annotations

import argparse
import json
import statistics
from pathlib import Path

from mks_apply_runtime_baseline import apply_runtime_baseline
from mks_axis_characterize import CANDIDATE_PRESETS, resolve_odrv_axis
from mks_manual_passthrough_test import run_manual_passthrough_test


DEFAULT_CANDIDATES = [
    {"name": "mounted-direct-v2", **CANDIDATE_PRESETS["mounted-direct-v2"]},
    {"name": "mounted-direct-v3", **CANDIDATE_PRESETS["mounted-direct-v3"]},
]


def _parse_steps(value: str):
    items = []
    for raw in str(value).split(","):
        s = raw.strip()
        if not s:
            continue
        items.append(float(s))
    return items


def _summarize_candidate(runs, steps):
    score_vals = [float(run["score"]) for run in runs]
    metrics = {}
    return_mean_by_step = {}
    for step in steps:
        ratios = []
        residuals = []
        settled = []
        peak_iq_set = []
        peak_iq_meas = []
        oks = []
        for run in runs:
            trial = next(t for t in run["trials"] if abs(float(t.get("delta_cmd", 0.0)) - float(step)) < 1e-9)
            oks.append(bool(trial.get("ok")))
            if trial.get("track_ratio") is not None:
                ratios.append(float(trial["track_ratio"]))
            if trial.get("return_residual") is not None:
                residuals.append(float(trial["return_residual"]))
            settled.append(bool(trial.get("return_settled")))
            if trial.get("peak_iq_set") is not None:
                peak_iq_set.append(float(trial["peak_iq_set"]))
            if trial.get("peak_iq_meas") is not None:
                peak_iq_meas.append(float(trial["peak_iq_meas"]))
        metrics[str(step)] = {
            "n": len(runs),
            "all_ok": all(oks) if oks else False,
            "track_ratio_mean": (statistics.mean(ratios) if ratios else None),
            "track_ratio_min": (min(ratios) if ratios else None),
            "track_ratio_max": (max(ratios) if ratios else None),
            "track_ratio_stdev": (statistics.pstdev(ratios) if len(ratios) > 1 else 0.0 if ratios else None),
            "return_residual_mean": (statistics.mean(residuals) if residuals else None),
            "return_residual_abs_mean": (statistics.mean(abs(v) for v in residuals) if residuals else None),
            "return_residual_abs_max": (max((abs(v) for v in residuals), default=None)),
            "return_settled_rate": ((sum(1 for v in settled if v) / len(settled)) if settled else None),
            "all_return_settled": all(settled) if settled else False,
            "peak_iq_set_mean": (statistics.mean(peak_iq_set) if peak_iq_set else None),
            "peak_iq_meas_mean": (statistics.mean(peak_iq_meas) if peak_iq_meas else None),
        }
        return_mean_by_step[float(step)] = metrics[str(step)]["return_residual_mean"]

    directional_bias = {}
    for step in steps:
        step_f = float(step)
        if step_f <= 0.0:
            continue
        pos_key = float(step_f)
        neg_key = float(-step_f)
        pos_mean = return_mean_by_step.get(pos_key)
        neg_mean = return_mean_by_step.get(neg_key)
        directional_bias[str(step_f)] = {
            "positive_return_residual_mean": pos_mean,
            "negative_return_residual_mean": neg_mean,
            "residual_mean_difference": (
                None if (pos_mean is None or neg_mean is None) else float(pos_mean - neg_mean)
            ),
            "residual_abs_mean_sum": (
                None if (pos_mean is None or neg_mean is None) else float(abs(pos_mean) + abs(neg_mean))
            ),
        }
    return {
        "all_ok_runs": sum(1 for run in runs if all(bool(t.get("ok")) for t in run["trials"])),
        "score_mean": (statistics.mean(score_vals) if score_vals else None),
        "score_min": (min(score_vals) if score_vals else None),
        "score_max": (max(score_vals) if score_vals else None),
        "score_stdev": (statistics.pstdev(score_vals) if len(score_vals) > 1 else 0.0 if score_vals else None),
        "metrics": metrics,
        "directional_bias": directional_bias,
    }


def run_mounted_hysteresis_probe(
    *,
    odrv=None,
    axis=None,
    serial_number=None,
    axis_index=0,
    repeats=3,
    steps=None,
    hold_s=0.90,
    abort_abs_turns=0.75,
    return_settle_tol=0.005,
    return_settle_timeout=1.5,
    out_path=None,
    candidates=None,
):
    steps = list(steps if steps is not None else [0.25, -0.25, 0.50, -0.50])
    candidates = list(candidates if candidates is not None else DEFAULT_CANDIDATES)
    odrv, axis = resolve_odrv_axis(
        odrv=odrv,
        axis=axis,
        serial_number=serial_number,
        axis_index=axis_index,
        timeout_s=10.0,
    )

    report = {
        "board_serial": str(getattr(odrv, "serial_number", "")),
        "axis_index": int(axis_index),
        "mode": "mounted_direct_hysteresis_probe",
        "repeats": int(repeats),
        "steps": [float(step) for step in steps],
        "hold_s": float(hold_s),
        "abort_abs_turns": float(abort_abs_turns),
        "return_settle_tol": float(return_settle_tol) if return_settle_tol is not None else None,
        "return_settle_timeout": float(return_settle_timeout) if return_settle_timeout is not None else None,
        "reuse_existing_calibration": True,
        "candidates": [],
    }

    if out_path:
        out_path = str(Path(out_path).resolve())
        partial_path = f"{out_path}.partial"
    else:
        partial_path = None

    def save_partial():
        if partial_path:
            Path(partial_path).write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    for candidate in candidates:
        runs = []
        for idx in range(int(repeats)):
            baseline = apply_runtime_baseline(
                odrv=odrv,
                axis_index=axis_index,
                preset="mounted-direct-v1",
                reuse_existing_calibration=True,
                current_lim=candidate["current_lim"],
            )
            res = run_manual_passthrough_test(
                odrv=odrv,
                axis=axis,
                axis_index=axis_index,
                candidate_preset="mounted-direct-v1",
                candidate_current_lim=candidate["current_lim"],
                candidate_pos_gain=candidate["pos_gain"],
                candidate_vel_gain=candidate["vel_gain"],
                candidate_vel_i_gain=candidate["vel_i_gain"],
                candidate_vel_limit=candidate["vel_limit"],
                steps=steps,
                hold_s=hold_s,
                abort_abs_turns=abort_abs_turns,
                apply_baseline=False,
                return_settle_tol=return_settle_tol,
                return_settle_timeout=return_settle_timeout,
            )
            run = dict(res["best_candidate"])
            run["repeat_index"] = idx + 1
            run["baseline_snapshot"] = baseline.get("snapshot")
            runs.append(run)
            report["candidates"] = [c for c in report["candidates"] if c["name"] != candidate["name"]]
            report["candidates"].append({"name": candidate["name"], "config": candidate, "runs": runs})
            save_partial()
            print(json.dumps({
                "candidate": candidate["name"],
                "repeat": idx + 1,
                "score": run["score"],
                "trials": [
                    {
                        "delta_cmd": t.get("delta_cmd"),
                        "ok": t.get("ok"),
                        "error": t.get("error"),
                        "track_ratio": t.get("track_ratio"),
                        "return_residual": t.get("return_residual"),
                        "return_settled": t.get("return_settled"),
                    }
                    for t in run["trials"]
                ],
            }, indent=2), flush=True)

        summary = _summarize_candidate(runs, steps)
        record = {
            "name": candidate["name"],
            "config": candidate,
            **summary,
            "runs": runs,
        }
        report["candidates"] = [c for c in report["candidates"] if c["name"] != candidate["name"]]
        report["candidates"].append(record)
        save_partial()
        print(json.dumps({
            "candidate": candidate["name"],
            "summary": {
                "all_ok_runs": record["all_ok_runs"],
                "score_mean": record["score_mean"],
                "metrics": record["metrics"],
            },
        }, indent=2), flush=True)

    if out_path:
        Path(out_path).write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(f"report_saved={out_path}", flush=True)
    return report


def main():
    ap = argparse.ArgumentParser(description="Mounted MKS direct-position hysteresis probe")
    ap.add_argument("--serial-number", default="", help="Optional board serial. Blank uses first found.")
    ap.add_argument("--axis-index", type=int, default=0)
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--steps", default="0.25,-0.25,0.50,-0.50")
    ap.add_argument("--hold-s", type=float, default=0.90)
    ap.add_argument("--abort-abs-turns", type=float, default=0.75)
    ap.add_argument("--return-settle-tol", type=float, default=0.005)
    ap.add_argument("--return-settle-timeout", type=float, default=1.5)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    run_mounted_hysteresis_probe(
        serial_number=(str(args.serial_number).strip() or None),
        axis_index=int(args.axis_index),
        repeats=int(args.repeats),
        steps=_parse_steps(args.steps),
        hold_s=float(args.hold_s),
        abort_abs_turns=float(args.abort_abs_turns),
        return_settle_tol=float(args.return_settle_tol),
        return_settle_timeout=float(args.return_settle_timeout),
        out_path=args.out,
    )


if __name__ == "__main__":
    main()
