#!/usr/bin/env python3
"""Repeatability runner for bare-motor MKS passthrough candidates.

This script keeps one live ODrive handle open, reruns the runtime baseline,
then executes repeated passthrough trials for one or more candidate configs.
It saves partial progress after each repeat so long runs are not lost.
"""

from __future__ import annotations

import argparse
import json
import statistics
from pathlib import Path

from mks_axis_characterize import resolve_odrv_axis
from mks_manual_passthrough_test import run_manual_passthrough_test


DEFAULT_CANDIDATES = [
    {
        "name": "v044",
        "candidate_current_lim": 2.75,
        "candidate_pos_gain": 4.75,
        "candidate_vel_gain": 0.10,
        "candidate_vel_i_gain": 0.02,
        "candidate_vel_limit": 0.44,
    },
    {
        "name": "v043",
        "candidate_current_lim": 2.75,
        "candidate_pos_gain": 4.75,
        "candidate_vel_gain": 0.10,
        "candidate_vel_i_gain": 0.02,
        "candidate_vel_limit": 0.43,
    },
    {
        "name": "ilim25_v045",
        "candidate_current_lim": 2.50,
        "candidate_pos_gain": 4.75,
        "candidate_vel_gain": 0.10,
        "candidate_vel_i_gain": 0.02,
        "candidate_vel_limit": 0.45,
    },
]


def _parse_steps(value: str):
    items = []
    for raw in str(value).split(","):
        s = raw.strip()
        if not s:
            continue
        items.append(float(s))
    return items


def _summarize_runs(runs, steps):
    ok_runs = [all(bool(t.get("ok")) for t in run["trials"]) for run in runs]
    scores = [float(run["score"]) for run in runs]
    metrics = {}
    for step in steps:
        vals = []
        for run in runs:
            trial = next((t for t in run["trials"] if t.get("delta_cmd") == step), None)
            if trial and trial.get("ok") and trial.get("track_ratio") is not None:
                vals.append(float(trial["track_ratio"]))
        metrics[str(step)] = {
            "n": len(vals),
            "mean": (statistics.mean(vals) if vals else None),
            "stdev": (statistics.pstdev(vals) if len(vals) > 1 else 0.0 if vals else None),
            "min": (min(vals) if vals else None),
            "max": (max(vals) if vals else None),
        }
    return {
        "all_ok_runs": sum(ok_runs),
        "score_mean": statistics.mean(scores),
        "score_stdev": (statistics.pstdev(scores) if len(scores) > 1 else 0.0),
        "score_min": min(scores),
        "score_max": max(scores),
        "metrics": metrics,
    }


def run_repeatability_sweep(
    *,
    serial_number=None,
    axis_index=0,
    timeout_s=10.0,
    repeats=3,
    steps=None,
    hold_s=0.90,
    abort_abs_turns=0.25,
    candidates=None,
    out_path=None,
):
    odrv, _ = resolve_odrv_axis(
        serial_number=serial_number,
        axis_index=axis_index,
        timeout_s=timeout_s,
    )
    steps = list(steps if steps is not None else [0.05, -0.05, 0.10, -0.10])
    candidates = list(candidates if candidates is not None else DEFAULT_CANDIDATES)

    report = {
        "board_serial": str(getattr(odrv, "serial_number", "")),
        "axis_index": int(axis_index),
        "repeats": int(repeats),
        "steps": steps,
        "hold_s": float(hold_s),
        "abort_abs_turns": float(abort_abs_turns),
        "candidates": [],
    }

    out = None
    if out_path:
        out = Path(out_path).resolve()
        out.parent.mkdir(parents=True, exist_ok=True)

    for candidate in candidates:
        runs = []
        candidate_report = {
            "name": candidate["name"],
            "config": dict(candidate),
            "runs": runs,
        }
        report["candidates"].append(candidate_report)

        for i in range(int(repeats)):
            result = run_manual_passthrough_test(
                odrv=odrv,
                axis_index=axis_index,
                candidate_preset="bare-pos-v1",
                candidate_current_lim=candidate.get("candidate_current_lim"),
                candidate_pos_gain=candidate.get("candidate_pos_gain"),
                candidate_vel_gain=candidate.get("candidate_vel_gain"),
                candidate_vel_i_gain=candidate.get("candidate_vel_i_gain"),
                candidate_vel_limit=candidate.get("candidate_vel_limit"),
                steps=steps,
                hold_s=float(hold_s),
                abort_abs_turns=float(abort_abs_turns),
            )
            best = result["best_candidate"]
            runs.append(
                {
                    "repeat": i + 1,
                    "score": best["score"],
                    "trials": [
                        {
                            "delta_cmd": t.get("delta_cmd"),
                            "ok": t.get("ok"),
                            "error": t.get("error"),
                            "track_ratio": t.get("track_ratio"),
                        }
                        for t in best["trials"]
                    ],
                }
            )
            candidate_report["summary"] = _summarize_runs(runs, steps)
            if out is not None:
                out.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    if out is not None:
        report["report_saved"] = str(out)
        out.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report


def main():
    ap = argparse.ArgumentParser(description="Repeatability sweep for bare-motor MKS candidates")
    ap.add_argument("--serial-number", default="")
    ap.add_argument("--axis-index", type=int, default=0)
    ap.add_argument("--timeout-s", type=float, default=10.0)
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--steps", default="0.05,-0.05,0.10,-0.10")
    ap.add_argument("--hold-s", type=float, default=0.90)
    ap.add_argument("--abort-abs-turns", type=float, default=0.25)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    report = run_repeatability_sweep(
        serial_number=(str(args.serial_number).strip() or None),
        axis_index=int(args.axis_index),
        timeout_s=float(args.timeout_s),
        repeats=int(args.repeats),
        steps=_parse_steps(args.steps),
        hold_s=float(args.hold_s),
        abort_abs_turns=float(args.abort_abs_turns),
        out_path=args.out,
    )
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
