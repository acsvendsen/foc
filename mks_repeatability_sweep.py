#!/usr/bin/env python3
"""Repeatability sweep for bare-motor MKS direct-position candidates.

This keeps one live ODrive handle and re-applies the normalized runtime
baseline before each candidate repeat. It is meant to separate "looked good
once" from "survives repeated fresh baseline cycles".
"""

from __future__ import annotations

import argparse
import json
import statistics
from pathlib import Path

from mks_apply_runtime_baseline import apply_runtime_baseline
from mks_manual_passthrough_test import run_manual_passthrough_test
from mks_axis_characterize import resolve_odrv_axis


DEFAULT_CANDIDATES = [
    {"name": "v044", "current_lim": 2.75, "pos_gain": 4.75, "vel_gain": 0.10, "vel_i_gain": 0.02, "vel_limit": 0.44},
    {"name": "v043", "current_lim": 2.75, "pos_gain": 4.75, "vel_gain": 0.10, "vel_i_gain": 0.02, "vel_limit": 0.43},
    {"name": "ilim25_v045", "current_lim": 2.50, "pos_gain": 4.75, "vel_gain": 0.10, "vel_i_gain": 0.02, "vel_limit": 0.45},
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
    ok_runs = [all(bool(t.get("ok")) for t in run["trials"]) for run in runs]
    score_vals = [float(r["score"]) for r in runs]
    metrics = {}
    for step in steps:
        vals = []
        for run in runs:
            trial = next(t for t in run["trials"] if t.get("delta_cmd") == step)
            if trial.get("ok") and trial.get("track_ratio") is not None:
                vals.append(float(trial["track_ratio"]))
        metrics[str(step)] = {
            "n": len(vals),
            "mean": (statistics.mean(vals) if vals else None),
            "min": (min(vals) if vals else None),
            "max": (max(vals) if vals else None),
            "stdev": (statistics.pstdev(vals) if len(vals) > 1 else 0.0 if vals else None),
        }
    return {
        "all_ok_runs": sum(ok_runs),
        "score_mean": statistics.mean(score_vals),
        "score_min": min(score_vals),
        "score_max": max(score_vals),
        "score_stdev": (statistics.pstdev(score_vals) if len(score_vals) > 1 else 0.0),
        "metrics": metrics,
    }


def run_repeatability_sweep(
    *,
    serial_number=None,
    axis_index=0,
    repeats=3,
    steps=None,
    hold_s=0.90,
    abort_abs_turns=0.25,
    return_settle_tol=0.005,
    return_settle_timeout=1.5,
    out_path=None,
    candidates=None,
):
    steps = list(steps if steps is not None else [0.05, -0.05, 0.10, -0.10])
    candidates = list(candidates if candidates is not None else DEFAULT_CANDIDATES)
    odrv, axis = resolve_odrv_axis(
        serial_number=serial_number,
        axis_index=axis_index,
        timeout_s=10.0,
    )

    report = {
        "repeats": int(repeats),
        "steps": steps,
        "hold_s": float(hold_s),
        "return_settle_tol": float(return_settle_tol) if return_settle_tol is not None else None,
        "return_settle_timeout": float(return_settle_timeout) if return_settle_timeout is not None else None,
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

    baseline = apply_runtime_baseline(
        odrv=odrv,
        axis_index=axis_index,
        preset="bare-pos-v1",
    )
    report["board_serial"] = baseline.get("board_serial")
    report["initial_baseline"] = baseline
    save_partial()

    for candidate in candidates:
        runs = []
        for idx in range(int(repeats)):
            baseline = apply_runtime_baseline(
                odrv=odrv,
                axis_index=axis_index,
                preset="bare-pos-v1",
            )
            res = run_manual_passthrough_test(
                odrv=odrv,
                axis=axis,
                axis_index=axis_index,
                candidate_preset="bare-pos-v1",
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
    ap = argparse.ArgumentParser(description="Repeatability sweep for MKS bare-motor direct-position candidates")
    ap.add_argument("--serial-number", default="", help="Optional board serial. Blank uses first found.")
    ap.add_argument("--axis-index", type=int, default=0)
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--steps", default="0.05,-0.05,0.10,-0.10")
    ap.add_argument("--hold-s", type=float, default=0.90)
    ap.add_argument("--abort-abs-turns", type=float, default=0.25)
    ap.add_argument("--return-settle-tol", type=float, default=0.005)
    ap.add_argument("--return-settle-timeout", type=float, default=1.5)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    run_repeatability_sweep(
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
