#!/usr/bin/env python3
"""A/B compare mounted direct-step versus slew-limited travel.

This is a diagnostic helper, not a production move path. It uses the same
candidate preset underneath and only changes the move law:

- direct-step directional preload
- slew-limited directional preload
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import time
from pathlib import Path

from mks_mounted_directional_move import run_directional_move, run_directional_slew_move


def _summarize(label: str, res: dict) -> dict:
    stages = {str(s.get("stage")): s for s in (res.get("stages") or [])}
    target_stage = stages.get("target") or stages.get("target_travel") or {}
    hold_stage = stages.get("hold") or {}
    return {
        "label": str(label),
        "approach_mode": res.get("approach_mode"),
        "delta_cmd": res.get("delta_cmd"),
        "final_error_abs": res.get("final_error_abs"),
        "target_stage": {
            "stage": target_stage.get("stage"),
            "ok": target_stage.get("ok"),
            "final_error_abs": target_stage.get("final_error_abs"),
            "peak_vel": target_stage.get("peak_vel"),
            "monotonic_fraction": target_stage.get("monotonic_fraction"),
            "backtrack_turns": target_stage.get("backtrack_turns"),
            "active_vel_sign_flips": target_stage.get("active_vel_sign_flips"),
            "peak_track_err": target_stage.get("peak_track_err"),
        },
        "hold_stage": {
            "classification": hold_stage.get("classification"),
            "pos_span_turns": hold_stage.get("pos_span_turns"),
            "vel_peak_abs_turns_s": hold_stage.get("vel_peak_abs_turns_s"),
        },
    }


def run_mounted_slew_compare(
    *,
    odrv=None,
    axis=None,
    serial_number=None,
    axis_index=0,
    candidate_preset="mounted-direct-v3",
    delta_turns=2.0,
    cycles=2,
    pre_hold_s=0.25,
    final_hold_s=0.90,
    abort_abs_turns=3.0,
    timeout_s=12.0,
    command_vel_turns_s=0.30,
    handoff_window_turns=0.10,
    command_dt=0.01,
    travel_pos_gain=None,
    travel_vel_gain=None,
    travel_vel_i_gain=None,
    travel_vel_limit=None,
    out_path=None,
):
    report = {
        "serial_number": (None if serial_number is None else str(serial_number)),
        "axis_index": int(axis_index),
        "candidate_preset": str(candidate_preset),
        "delta_turns": float(delta_turns),
        "cycles": int(cycles),
        "command_vel_turns_s": float(command_vel_turns_s),
        "handoff_window_turns": float(handoff_window_turns),
        "travel_pos_gain": (None if travel_pos_gain is None else float(travel_pos_gain)),
        "travel_vel_gain": (None if travel_vel_gain is None else float(travel_vel_gain)),
        "travel_vel_i_gain": (None if travel_vel_i_gain is None else float(travel_vel_i_gain)),
        "travel_vel_limit": (None if travel_vel_limit is None else float(travel_vel_limit)),
        "tests": [],
        "ts": dt.datetime.now().isoformat(timespec="seconds"),
    }

    for cycle in range(max(1, int(cycles))):
        for label, fn, signed_delta in [
            ("direct_pos", run_directional_move, abs(float(delta_turns))),
            ("direct_neg", run_directional_move, -abs(float(delta_turns))),
            ("slew_pos", run_directional_slew_move, abs(float(delta_turns))),
            ("slew_neg", run_directional_slew_move, -abs(float(delta_turns))),
        ]:
            kwargs = dict(
                odrv=odrv,
                axis=axis,
                serial_number=serial_number,
                axis_index=int(axis_index),
                candidate_preset=str(candidate_preset),
                delta_turns=float(signed_delta),
                timeout_s=float(timeout_s),
                pre_hold_s=float(pre_hold_s),
                final_hold_s=float(final_hold_s),
                return_to_start=False,
                abort_abs_turns=float(abort_abs_turns),
            )
            if fn is run_directional_slew_move:
                kwargs.update(
                    command_vel_turns_s=float(command_vel_turns_s),
                    handoff_window_turns=float(handoff_window_turns),
                    command_dt=float(command_dt),
                    travel_pos_gain=(None if travel_pos_gain is None else float(travel_pos_gain)),
                    travel_vel_gain=(None if travel_vel_gain is None else float(travel_vel_gain)),
                    travel_vel_i_gain=(None if travel_vel_i_gain is None else float(travel_vel_i_gain)),
                    travel_vel_limit=(None if travel_vel_limit is None else float(travel_vel_limit)),
                )
            res = fn(**kwargs)
            report["tests"].append(_summarize(f"{label}_cycle{cycle + 1}", res))
            time.sleep(0.25)

    if out_path:
        path = Path(out_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, indent=2))

    return report


def main():
    ap = argparse.ArgumentParser(description="Compare mounted direct-step versus slew-limited travel")
    ap.add_argument("--serial-number", default="", help="Optional board serial")
    ap.add_argument("--axis-index", type=int, default=0)
    ap.add_argument("--candidate-preset", default="mounted-direct-v3")
    ap.add_argument("--delta-turns", type=float, default=2.0)
    ap.add_argument("--cycles", type=int, default=2)
    ap.add_argument("--pre-hold-s", type=float, default=0.25)
    ap.add_argument("--final-hold-s", type=float, default=0.90)
    ap.add_argument("--abort-abs-turns", type=float, default=3.0)
    ap.add_argument("--timeout-s", type=float, default=12.0)
    ap.add_argument("--command-vel-turns-s", type=float, default=0.30)
    ap.add_argument("--handoff-window-turns", type=float, default=0.10)
    ap.add_argument("--command-dt", type=float, default=0.01)
    ap.add_argument("--travel-pos-gain", type=float, default=None)
    ap.add_argument("--travel-vel-gain", type=float, default=None)
    ap.add_argument("--travel-vel-i-gain", type=float, default=None)
    ap.add_argument("--travel-vel-limit", type=float, default=None)
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    res = run_mounted_slew_compare(
        serial_number=(str(args.serial_number).strip() or None),
        axis_index=int(args.axis_index),
        candidate_preset=str(args.candidate_preset),
        delta_turns=float(args.delta_turns),
        cycles=int(args.cycles),
        pre_hold_s=float(args.pre_hold_s),
        final_hold_s=float(args.final_hold_s),
        abort_abs_turns=float(args.abort_abs_turns),
        timeout_s=float(args.timeout_s),
        command_vel_turns_s=float(args.command_vel_turns_s),
        handoff_window_turns=float(args.handoff_window_turns),
        command_dt=float(args.command_dt),
        travel_pos_gain=(None if args.travel_pos_gain is None else float(args.travel_pos_gain)),
        travel_vel_gain=(None if args.travel_vel_gain is None else float(args.travel_vel_gain)),
        travel_vel_i_gain=(None if args.travel_vel_i_gain is None else float(args.travel_vel_i_gain)),
        travel_vel_limit=(None if args.travel_vel_limit is None else float(args.travel_vel_limit)),
        out_path=(str(args.out).strip() or None),
    )
    print(json.dumps(res, indent=2))


if __name__ == "__main__":
    main()
