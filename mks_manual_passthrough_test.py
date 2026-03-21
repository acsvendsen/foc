#!/usr/bin/env python3
"""Direct MKS position-passthrough tester on normalized v0.5.6 firmware.

This is intentionally not a motion-profile runner. It validates whether a
runtime baseline and a small set of direct position controller settings behave
coherently on the bare motor before we create any real MKS profiles.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import time
from pathlib import Path

import odrive
from odrive.enums import AXIS_STATE_FULL_CALIBRATION_SEQUENCE, CONTROL_MODE_POSITION_CONTROL
from odrive.enums import INPUT_MODE_PASSTHROUGH

import common


DEFAULT_CANDIDATES = [
    {"name": "mks_bare_pos_conservative", "current_lim": 2.5, "pos_gain": 4.5, "vel_gain": 0.09, "vel_i_gain": 0.0, "vel_limit": 0.35},
    {"name": "mks_bare_pos_v1", "current_lim": 2.75, "pos_gain": 4.75, "vel_gain": 0.10, "vel_i_gain": 0.02, "vel_limit": 0.45},
    {"name": "mks_bare_pos_fast1", "current_lim": 2.75, "pos_gain": 4.75, "vel_gain": 0.10, "vel_i_gain": 0.02, "vel_limit": 0.50},
]


def _safe_float(v, default=None):
    try:
        return float(v)
    except Exception:
        return default


def _apply_runtime_baseline(odrv, axis):
    common.clear_errors_all(axis)
    try:
        odrv.clear_errors()
    except Exception:
        pass
    try:
        odrv.config.dc_max_negative_current = -5.0
    except Exception:
        pass
    common.force_idle(axis, settle_s=0.05)
    common.set_encoder(axis, cpr=1024, bandwidth=200, interp=True, use_index=False, encoder_source="INC_ENCODER0")
    axis.motor.config.current_control_bandwidth = 300.0
    axis.motor.config.current_lim = 6.0
    axis.motor.config.calibration_current = 6.0
    axis.controller.config.enable_overspeed_error = False
    axis.controller.config.vel_limit_tolerance = 4.0
    axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
    axis.controller.config.pos_gain = 4.5
    axis.controller.config.vel_gain = 0.09
    axis.controller.config.vel_integrator_gain = 0.0
    axis.controller.config.vel_limit = 0.35
    common.calibrate(axis)
    common.clear_errors_all(axis)
    try:
        odrv.clear_errors()
    except Exception:
        pass
    try:
        common.sync_pos_setpoint(axis, settle_s=0.05, retries=3, verbose=False)
    except Exception:
        pass
    return common.get_axis_error_report(axis)


def _prepare_candidate(odrv, axis, candidate):
    common.clear_errors_all(axis)
    try:
        odrv.clear_errors()
    except Exception:
        pass
    axis.motor.config.current_lim = float(candidate["current_lim"])
    axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
    axis.controller.config.pos_gain = float(candidate["pos_gain"])
    axis.controller.config.vel_gain = float(candidate["vel_gain"])
    axis.controller.config.vel_integrator_gain = float(candidate.get("vel_i_gain", 0.0))
    axis.controller.config.vel_limit = float(candidate["vel_limit"])
    axis.controller.config.enable_overspeed_error = False
    axis.controller.config.vel_limit_tolerance = 4.0
    if not common.ensure_closed_loop(axis, timeout_s=2.0, clear_first=False, pre_sync=True, retries=2):
        return False
    try:
        common.sync_pos_setpoint(axis, settle_s=0.05, retries=3, verbose=False)
    except Exception:
        pass
    return True


def _trial(odrv, axis, candidate, delta_turns, hold_s=0.40, abort_abs_turns=0.25):
    ok_cl = _prepare_candidate(odrv, axis, candidate)
    if not ok_cl:
        return {"ok": False, "error": "closed_loop_failed", "delta_cmd": float(delta_turns)}

    p0 = float(getattr(axis.encoder, "pos_estimate", 0.0))
    q0 = int(getattr(axis.encoder, "shadow_count", 0))
    tgt = p0 + float(delta_turns)
    peak_vel = 0.0
    peak_iq_set = 0.0
    peak_iq_meas = 0.0
    err = None

    axis.controller.input_pos = float(tgt)
    t_end = time.time() + float(hold_s)
    while time.time() < t_end:
        ax = int(getattr(axis, "error", 0))
        mo = int(getattr(axis.motor, "error", 0))
        en = int(getattr(axis.encoder, "error", 0))
        ct = int(getattr(axis.controller, "error", 0))
        oe = int(getattr(odrv, "error", 0))
        p = float(getattr(axis.encoder, "pos_estimate", p0))
        v = float(getattr(axis.encoder, "vel_estimate", 0.0))
        iq_set = float(getattr(axis.motor.current_control, "Iq_setpoint", 0.0))
        iq_meas = float(getattr(axis.motor.current_control, "Iq_measured", 0.0))
        peak_vel = max(peak_vel, abs(v))
        peak_iq_set = max(peak_iq_set, abs(iq_set))
        peak_iq_meas = max(peak_iq_meas, abs(iq_meas))
        if any([ax, mo, en, ct, oe]):
            err = f"axis={hex(ax)} motor={hex(mo)} enc={hex(en)} ctrl={hex(ct)} odrv={hex(oe)}"
            break
        if abs(p - p0) > float(abort_abs_turns):
            err = f"runaway_abs_dev>{abort_abs_turns}"
            break
        time.sleep(0.01)

    p1 = float(getattr(axis.encoder, "pos_estimate", p0))
    q1 = int(getattr(axis.encoder, "shadow_count", q0))
    try:
        axis.controller.input_pos = float(p0)
        time.sleep(0.10)
    except Exception:
        pass
    common.force_idle(axis, settle_s=0.05)

    dp = float(p1 - p0)
    return {
        "ok": err is None,
        "error": err,
        "delta_cmd": float(delta_turns),
        "start_pos": float(p0),
        "end_pos": float(p1),
        "dp": float(dp),
        "dq": int(q1 - q0),
        "track_ratio": (None if float(delta_turns) == 0.0 else float(dp / float(delta_turns))),
        "peak_vel": float(peak_vel),
        "peak_iq_set": float(peak_iq_set),
        "peak_iq_meas": float(peak_iq_meas),
        "axis_report": common.get_axis_error_report(axis),
    }


def _score_candidate(results):
    # Higher is better. Reward non-faulting motion and symmetry, penalize overshoot/runaway.
    score = 0.0
    for r in results:
        if not r.get("ok"):
            score -= 100.0
            continue
        cmd = abs(float(r["delta_cmd"]))
        dp = abs(float(r["dp"]))
        ratio = 0.0 if cmd == 0.0 else (dp / cmd)
        score += 50.0 * min(ratio, 1.0)
        if ratio > 1.5:
            score -= 40.0 * (ratio - 1.5)
    by_mag = {}
    for r in results:
        by_mag.setdefault(abs(float(r["delta_cmd"])), []).append(r)
    for mag, rs in by_mag.items():
        if len(rs) == 2 and all(r.get("ok") for r in rs):
            a, b = rs
            sym = abs(abs(float(a["dp"])) - abs(float(b["dp"])))
            score -= 200.0 * sym
    return float(score)


def run(serial_number: str, axis_index: int, out_path: str):
    odrv = odrive.find_any(serial_number=str(serial_number).strip(), timeout=10.0)
    axis = getattr(odrv, f"axis{int(axis_index)}")

    report = {
        "timestamp": dt.datetime.now().isoformat(),
        "board_serial": str(getattr(odrv, "serial_number", "")),
        "axis_index": int(axis_index),
        "firmware": [
            int(getattr(odrv, "fw_version_major", 0) or 0),
            int(getattr(odrv, "fw_version_minor", 0) or 0),
            int(getattr(odrv, "fw_version_revision", 0) or 0),
        ],
        "baseline_before": common.get_axis_error_report(axis),
    }

    report["baseline_after"] = _apply_runtime_baseline(odrv, axis)
    report["dc_max_negative_current"] = _safe_float(getattr(odrv.config, "dc_max_negative_current", None))

    step_list = [+0.05, -0.05, +0.10, -0.10, +0.15, -0.15, +0.20, -0.20]
    candidate_reports = []
    for candidate in DEFAULT_CANDIDATES:
        trials = []
        for delta in step_list:
            trials.append(_trial(odrv, axis, candidate, delta))
        cand = dict(candidate)
        cand["trials"] = trials
        cand["score"] = _score_candidate(trials)
        candidate_reports.append(cand)

    candidate_reports.sort(key=lambda c: c["score"], reverse=True)
    report["candidates"] = candidate_reports
    report["best_candidate"] = candidate_reports[0] if candidate_reports else None

    common.clear_errors_all(axis)
    common.force_idle(axis, settle_s=0.05)
    try:
        odrv.clear_errors()
    except Exception:
        pass
    report["final_state"] = common.get_axis_error_report(axis)

    out = Path(out_path).resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    print(f"report_saved={out}")


def main():
    ap = argparse.ArgumentParser(description="Direct MKS manual passthrough test on normalized v0.5.6 firmware")
    ap.add_argument("--serial-number", required=True)
    ap.add_argument("--axis-index", type=int, default=0)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    run(args.serial_number, args.axis_index, args.out)


if __name__ == "__main__":
    main()
