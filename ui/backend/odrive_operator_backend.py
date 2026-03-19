#!/usr/bin/env python3
from __future__ import annotations

import argparse
import contextlib
import io
import json
import math
import os
import sys
import threading
import time
import traceback
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import odrive  # type: ignore

import common  # type: ignore
DEFAULT_KV_EST = 140.0
DEFAULT_LINE_LINE_R_OHM = 0.30
DEFAULT_GEAR_RATIO = 25.0
DEFAULT_PROFILE = "gearbox_output_continuous_quiet_20260309"
DEFAULT_CONNECT_TIMEOUT_S = 3.0


def _clean_json(value: Any) -> Any:
    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(k): _clean_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_clean_json(v) for v in value]
    return str(value)


def _normalize_snapshot(snapshot: dict[str, Any] | None) -> dict[str, Any]:
    out = dict(snapshot or {})
    for key in ("enc_ready", "enc_use_index", "enc_index_found"):
        if key in out and out.get(key) is not None:
            out[key] = bool(out.get(key))
    return out


def _connect(axis_index: int, timeout_s: float):
    try:
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            odrv = odrive.find_any(timeout=float(timeout_s))
    except Exception as exc:
        msg = str(exc).strip() or (
            f"ODrive connection failed within {float(timeout_s):.2f}s. "
            "The device may be disconnected or already in use by another program."
        )
        raise RuntimeError(msg) from exc
    if odrv is None:
        raise RuntimeError(f"No ODrive found within {float(timeout_s):.2f}s")
    axis_name = f"axis{int(axis_index)}"
    try:
        axis = getattr(odrv, axis_name)
    except AttributeError as exc:
        raise RuntimeError(f"Device does not expose {axis_name}") from exc
    # common.clear_errors_all() re-fetches the device; keep it on the same live object.
    common._ODRV0_FALLBACK = odrv
    return odrv, axis


def _device_info(odrv: Any, axis_index: int) -> dict[str, Any]:
    serial_number = None
    try:
        serial_number = str(getattr(odrv, "serial_number"))
    except Exception:
        serial_number = None
    return {
        "axis_index": int(axis_index),
        "serial_number": serial_number,
        "vbus_voltage": _clean_json(getattr(odrv, "vbus_voltage", None)),
    }


def _continuous_profiles() -> list[str]:
    return [row["name"] for row in _continuous_profile_records()]


def _continuous_profile_records() -> list[dict[str, Any]]:
    profiles_path = _profiles_path()
    data = json.loads(profiles_path.read_text())
    profiles = dict(data.get("profiles") or {})
    rows: list[dict[str, Any]] = []
    for name, payload in profiles.items():
        if ("continuous_kwargs" in (payload or {})) or ("continuous" in str(name)):
            prof = dict(payload or {})
            rows.append({
                "name": str(name),
                "notes": str(prof.get("notes") or ""),
                "limitations": list(prof.get("limitations") or []),
                "source": str(prof.get("source") or ""),
            })
    return sorted(rows, key=lambda row: str(row.get("name") or ""))


def _profiles_path() -> Path:
    return REPO_ROOT / "logs" / "stable_diagnostic_profiles_latest.json"


def _load_continuous_move_kwargs(profile_name: str) -> dict[str, Any]:
    data = json.loads(_profiles_path().read_text())
    profiles = dict(data.get("profiles") or {})
    key = str(profile_name).strip()
    if key not in profiles:
        raise ValueError(f"Unknown continuous profile '{profile_name}'")
    prof = dict(profiles.get(key) or {})
    suite = dict(prof.get("suite_kwargs") or {})
    step = dict(suite.get("step_kwargs") or {})
    extra = dict(prof.get("continuous_kwargs") or {})
    reanchor = extra.get("quiet_hold_reanchor_err_turns", 0.035)
    return {
        "timeout_s": float(extra.get("timeout_s", 8.0)),
        "min_delta_turns": float(extra.get("min_delta_turns", 0.0015)),
        "settle_s": float(extra.get("settle_s", 0.08)),
        "vel_limit": float(suite.get("vel_limit", 0.40)),
        "vel_limit_tolerance": float(suite.get("vel_limit_tolerance", 4.0)),
        "enable_overspeed_error": bool(suite.get("enable_overspeed_error", False)),
        "trap_vel": float(suite.get("trap_vel", 0.28)),
        "trap_acc": float(suite.get("trap_acc", 0.32)),
        "trap_dec": float(suite.get("trap_dec", 0.32)),
        "current_lim": float(suite.get("current_lim", 6.5)),
        "pos_gain": float(suite.get("pos_gain", 12.0)),
        "vel_gain": float(suite.get("vel_gain", 0.22)),
        "vel_i_gain": float(suite.get("vel_i_gain", 0.0)),
        "target_tolerance_turns": float(step.get("target_tolerance_turns", 0.03)),
        "target_vel_tolerance_turns_s": float(step.get("target_vel_tolerance_turns_s", 0.20)),
        "quiet_hold_enable": bool(extra.get("quiet_hold_enable", True)),
        "quiet_hold_s": float(extra.get("quiet_hold_s", 0.06)),
        "quiet_hold_pos_gain_scale": float(extra.get("quiet_hold_pos_gain_scale", 0.45)),
        "quiet_hold_vel_gain_scale": float(extra.get("quiet_hold_vel_gain_scale", 0.70)),
        "quiet_hold_vel_i_gain": float(extra.get("quiet_hold_vel_i_gain", 0.0)),
        "quiet_hold_vel_limit_scale": float(extra.get("quiet_hold_vel_limit_scale", 0.50)),
        "quiet_hold_persist": bool(extra.get("quiet_hold_persist", True)),
        "quiet_hold_reanchor_err_turns": (None if reanchor is None else float(reanchor)),
        "fail_to_idle": bool(extra.get("fail_to_idle", False)),
    }


def _trap_move_time_est(distance_turns: float, trap_vel: float, trap_acc: float, trap_dec: float) -> float:
    s = abs(float(distance_turns))
    if s <= 0.0:
        return 0.0
    v = max(1e-9, abs(float(trap_vel)))
    a = max(1e-9, abs(float(trap_acc)))
    d = max(1e-9, abs(float(trap_dec)))
    s_acc = (v * v) / (2.0 * a)
    s_dec = (v * v) / (2.0 * d)
    s_ramp = s_acc + s_dec
    if s <= s_ramp:
        vp = math.sqrt(max(0.0, 2.0 * s / ((1.0 / a) + (1.0 / d))))
        return (vp / a) + (vp / d)
    return (v / a) + (v / d) + ((s - s_ramp) / v)


def _move_to_angle_continuous(
    axis: Any,
    *,
    angle_deg: float,
    angle_space: str,
    profile_name: str,
    gear_ratio: float,
    zero_turns_motor: float | None,
    relative_to_current: bool,
    timeout_s: float | None,
    sample_hook=None,
) -> dict[str, Any]:
    cfg = _load_continuous_move_kwargs(profile_name=profile_name)
    start_turns_motor = float(getattr(axis.encoder, "pos_estimate", 0.0))
    base_turns_motor = (
        start_turns_motor
        if (bool(relative_to_current) or (zero_turns_motor is None))
        else float(zero_turns_motor)
    )
    space = str(angle_space).strip().lower()
    if space == "motor":
        target_turns_motor = float(base_turns_motor) + (float(angle_deg) / 360.0)
    elif space == "gearbox_output":
        target_turns_motor = float(base_turns_motor) + ((float(angle_deg) / 360.0) * float(gear_ratio))
    else:
        raise ValueError(f"Unsupported angle_space '{angle_space}'")
    if timeout_s is not None:
        cfg["timeout_s"] = float(timeout_s)
    else:
        move_dist = abs(float(target_turns_motor) - float(start_turns_motor))
        est_total = _trap_move_time_est(
            move_dist,
            float(cfg["trap_vel"]),
            float(cfg["trap_acc"]),
            float(cfg["trap_dec"]),
        ) + float(cfg["settle_s"]) + 0.75
        cfg["timeout_s"] = max(float(cfg["timeout_s"]), float(est_total))
    raw = common.move_to_pos_strict(
        axis,
        float(target_turns_motor),
        use_trap_traj=True,
        timeout_s=float(cfg["timeout_s"]),
        min_delta_turns=float(cfg["min_delta_turns"]),
        settle_s=float(cfg["settle_s"]),
        vel_limit=float(cfg["vel_limit"]),
        vel_limit_tolerance=float(cfg["vel_limit_tolerance"]),
        enable_overspeed_error=bool(cfg["enable_overspeed_error"]),
        trap_vel=float(cfg["trap_vel"]),
        trap_acc=float(cfg["trap_acc"]),
        trap_dec=float(cfg["trap_dec"]),
        current_lim=float(cfg["current_lim"]),
        pos_gain=float(cfg["pos_gain"]),
        vel_gain=float(cfg["vel_gain"]),
        vel_i_gain=float(cfg["vel_i_gain"]),
        require_target_reached=True,
        target_tolerance_turns=float(cfg["target_tolerance_turns"]),
        target_vel_tolerance_turns_s=float(cfg["target_vel_tolerance_turns_s"]),
        quiet_hold_enable=bool(cfg["quiet_hold_enable"]),
        quiet_hold_s=float(cfg["quiet_hold_s"]),
        quiet_hold_pos_gain_scale=float(cfg["quiet_hold_pos_gain_scale"]),
        quiet_hold_vel_gain_scale=float(cfg["quiet_hold_vel_gain_scale"]),
        quiet_hold_vel_i_gain=float(cfg["quiet_hold_vel_i_gain"]),
        quiet_hold_vel_limit_scale=float(cfg["quiet_hold_vel_limit_scale"]),
        quiet_hold_persist=bool(cfg["quiet_hold_persist"]),
        quiet_hold_reanchor_err_turns=(
            None
            if cfg.get("quiet_hold_reanchor_err_turns") is None
            else float(cfg["quiet_hold_reanchor_err_turns"])
        ),
        fail_to_idle=bool(cfg["fail_to_idle"]),
        sample_hook=sample_hook,
    )
    out = dict(raw or {})
    out["profile_name"] = str(profile_name)
    out["angle_space"] = str(angle_space)
    out["angle_deg"] = float(angle_deg)
    out["gear_ratio"] = float(gear_ratio)
    out["start_turns_motor"] = float(start_turns_motor)
    out["zero_turns_motor"] = float(base_turns_motor)
    out["target_turns_motor"] = float(target_turns_motor)
    return out


def _apply_continuous_profile(axis: Any, cfg: dict[str, Any]) -> None:
    try:
        axis.motor.config.current_lim = float(cfg["current_lim"])
    except Exception:
        pass
    try:
        margin = max(1.0, float(cfg["current_lim"]) * 0.33)
        axis.motor.config.current_lim_margin = float(margin)
    except Exception:
        pass
    try:
        axis.controller.config.control_mode = common.CONTROL_MODE_POSITION_CONTROL
        axis.controller.config.input_mode = common.INPUT_MODE_TRAP_TRAJ
        axis.controller.config.pos_gain = float(cfg["pos_gain"])
        axis.controller.config.vel_gain = float(cfg["vel_gain"])
        axis.controller.config.vel_integrator_gain = float(cfg["vel_i_gain"])
        axis.controller.config.vel_limit = float(cfg["vel_limit"])
        if hasattr(axis.controller.config, "vel_limit_tolerance"):
            axis.controller.config.vel_limit_tolerance = float(cfg["vel_limit_tolerance"])
        if hasattr(axis.controller.config, "enable_overspeed_error"):
            axis.controller.config.enable_overspeed_error = bool(cfg["enable_overspeed_error"])
    except Exception:
        pass
    try:
        axis.trap_traj.config.vel_limit = float(cfg["trap_vel"])
        axis.trap_traj.config.accel_limit = float(cfg["trap_acc"])
        axis.trap_traj.config.decel_limit = float(cfg["trap_dec"])
    except Exception:
        pass


def _target_turns_motor_for_angle(
    axis: Any,
    *,
    angle_deg: float,
    angle_space: str,
    gear_ratio: float,
    zero_turns_motor: float | None,
    relative_to_current: bool,
) -> dict[str, Any]:
    start_turns_motor = float(getattr(axis.encoder, "pos_estimate", 0.0))
    base_turns_motor = (
        start_turns_motor
        if (bool(relative_to_current) or (zero_turns_motor is None))
        else float(zero_turns_motor)
    )
    space = str(angle_space).strip().lower()
    if space == "motor":
        target_turns_motor = float(base_turns_motor) + (float(angle_deg) / 360.0)
    elif space == "gearbox_output":
        target_turns_motor = float(base_turns_motor) + ((float(angle_deg) / 360.0) * float(gear_ratio))
    else:
        raise ValueError(f"Unsupported angle_space '{angle_space}'")
    return {
        "start_turns_motor": float(start_turns_motor),
        "zero_turns_motor": float(base_turns_motor),
        "target_turns_motor": float(target_turns_motor),
    }


def _issue_follow_angle_target(
    axis: Any,
    *,
    angle_deg: float,
    angle_space: str,
    profile_name: str,
    gear_ratio: float,
    zero_turns_motor: float | None,
    relative_to_current: bool,
) -> dict[str, Any]:
    cfg = _load_continuous_move_kwargs(profile_name=profile_name)
    state_before = int(getattr(axis, "current_state", 0) or 0)
    _apply_continuous_profile(axis, cfg)

    if state_before != int(common.AXIS_STATE_CLOSED_LOOP_CONTROL):
        if not bool(common.ensure_closed_loop(axis, timeout_s=2.0)):
            raise RuntimeError(f"Failed to enter CLOSED_LOOP_CONTROL. snapshot={common._snapshot_motion(axis)}")
        _apply_continuous_profile(axis, cfg)
        try:
            axis.controller.input_pos = float(getattr(axis.encoder, "pos_estimate", 0.0))
        except Exception:
            pass
        if not bool(common.sync_pos_setpoint(axis, settle_s=0.03, retries=2, verbose=False)):
            raise RuntimeError("Failed to synchronize input_pos/pos_setpoint before live follow.")

    tgt = _target_turns_motor_for_angle(
        axis,
        angle_deg=float(angle_deg),
        angle_space=str(angle_space),
        gear_ratio=float(gear_ratio),
        zero_turns_motor=zero_turns_motor,
        relative_to_current=bool(relative_to_current),
    )
    target_turns_motor = float(tgt["target_turns_motor"])
    try:
        if hasattr(axis.controller, "move_to_pos"):
            axis.controller.move_to_pos(float(target_turns_motor))
        else:
            axis.controller.input_pos = float(target_turns_motor)
    except Exception as exc:
        raise RuntimeError(f"Failed to issue live follow target: {exc}") from exc

    return {
        "issued": True,
        "profile_name": str(profile_name),
        "angle_space": str(angle_space),
        "angle_deg": float(angle_deg),
        "gear_ratio": float(gear_ratio),
        "start_turns_motor": float(tgt["start_turns_motor"]),
        "zero_turns_motor": float(tgt["zero_turns_motor"]),
        "target_turns_motor": float(target_turns_motor),
        "state_before": int(state_before),
        "state_after": int(getattr(axis, "current_state", 0) or 0),
        "input_pos_after": _clean_json(getattr(axis.controller, "input_pos", None)),
    }


def _status_bundle(axis: Any, *, kv_est: float | None, line_line_r_ohm: float | None) -> dict[str, Any]:
    diagnosis = common.diagnose_axis_state(
        axis=axis,
        mounted=True,
        verbose=False,
        kv_est=kv_est,
        line_line_r_ohm=line_line_r_ohm,
    )
    fact_sheet = common.motor_fact_sheet(
        axis=axis,
        kv_est=kv_est,
        line_line_r_ohm=line_line_r_ohm,
        verbose=False,
    )
    snapshot = _normalize_snapshot((diagnosis.get("report") or {}).get("snapshot") or {})
    axis_err_names = list((diagnosis.get("report") or {}).get("axis_err_names") or [])
    motor_err_names = list((diagnosis.get("report") or {}).get("motor_err_names") or [])
    enc_err_names = list((diagnosis.get("report") or {}).get("enc_err_names") or [])
    ctrl_err_names = list((diagnosis.get("report") or {}).get("ctrl_err_names") or [])
    state = int(snapshot.get("state") or 0)
    startup_ready = "startup ready" in list(diagnosis.get("verdicts") or [])
    latched_error = bool(axis_err_names or motor_err_names or enc_err_names or ctrl_err_names)
    capabilities = {
        "can_startup": bool(state != int(common.AXIS_STATE_FULL_CALIBRATION_SEQUENCE)),
        "can_idle": True,
        "can_clear_errors": True,
        "can_diagnose": True,
        "can_fact_sheet": True,
        "can_move_continuous": bool(startup_ready and not latched_error),
        "can_move_continuous_aggressive": bool(startup_ready and not latched_error),
        "can_capture_zero_here": bool(startup_ready),
        "startup_ready": bool(startup_ready),
        "armed": bool(state == int(common.AXIS_STATE_CLOSED_LOOP_CONTROL)),
        "idle": bool(state == int(common.AXIS_STATE_IDLE)),
        "has_latched_errors": bool(latched_error),
    }
    return {
        "snapshot": _clean_json(snapshot),
        "diagnosis": _clean_json(diagnosis),
        "fact_sheet": _clean_json(fact_sheet),
        "capabilities": _clean_json(capabilities),
        "available_profiles": _continuous_profiles(),
        "available_profile_details": _continuous_profile_records(),
    }


def _telemetry_bundle(axis: Any) -> dict[str, Any]:
    snapshot = _normalize_snapshot(common._snapshot_motion(axis) or {})
    axis_err = int(snapshot.get("axis_err") or 0)
    motor_err = int(snapshot.get("motor_err") or 0)
    enc_err = int(snapshot.get("enc_err") or 0)
    ctrl_err = int(snapshot.get("ctrl_err") or 0)
    latched_error = bool(axis_err or motor_err or enc_err or ctrl_err)
    state = int(snapshot.get("state") or 0)
    startup_ready = bool(snapshot.get("enc_ready")) and not bool(latched_error)
    capabilities = {
        "can_startup": bool(state != int(common.AXIS_STATE_FULL_CALIBRATION_SEQUENCE)),
        "can_idle": True,
        "can_clear_errors": True,
        "can_diagnose": True,
        "can_fact_sheet": True,
        "can_move_continuous": bool(startup_ready),
        "can_move_continuous_aggressive": bool(startup_ready),
        "can_capture_zero_here": bool(startup_ready),
        "startup_ready": bool(startup_ready),
        "armed": bool(state == int(common.AXIS_STATE_CLOSED_LOOP_CONTROL)),
        "idle": bool(state == int(common.AXIS_STATE_IDLE)),
        "has_latched_errors": bool(latched_error),
    }
    return {
        "snapshot": _clean_json(snapshot),
        "diagnosis": None,
        "fact_sheet": None,
        "capabilities": _clean_json(capabilities),
        "available_profiles": _continuous_profiles(),
        "available_profile_details": _continuous_profile_records(),
    }


def _telemetry_result_from_status(status: dict[str, Any]) -> dict[str, Any]:
    snap = dict(status.get("snapshot") or {})
    return {
        "pos_est": snap.get("pos_est"),
        "vel_est": snap.get("vel_est"),
        "Iq_meas": snap.get("Iq_meas"),
        "input_pos": snap.get("input_pos"),
        "tracking_err_turns": (
            None
            if (snap.get("input_pos") is None or snap.get("pos_est") is None)
            else (float(snap.get("input_pos")) - float(snap.get("pos_est")))
        ),
    }


def _mark_motion_capabilities(status: dict[str, Any], *, motion_active: bool) -> dict[str, Any]:
    out = dict(status or {})
    capabilities = dict(out.get("capabilities") or {})
    capabilities["motion_active"] = bool(motion_active)
    if bool(motion_active):
        capabilities["can_move_continuous"] = False
        capabilities["can_move_continuous_aggressive"] = False
        capabilities["can_startup"] = False
        capabilities["can_diagnose"] = False
        capabilities["can_fact_sheet"] = False
        capabilities["can_capture_zero_here"] = False
    out["capabilities"] = _clean_json(capabilities)
    return out


def _result_envelope(*, ok: bool, action: str, device: dict[str, Any] | None = None, message: str | None = None,
                     status: dict[str, Any] | None = None, result: dict[str, Any] | None = None,
                     error: dict[str, Any] | None = None, request_id: str | None = None) -> dict[str, Any]:
    out = {
        "ok": bool(ok),
        "action": str(action),
        "request_id": (None if request_id is None else str(request_id)),
        "timestamp_s": time.time(),
        "device": _clean_json(device),
        "message": str(message) if message else None,
        "snapshot": None,
        "diagnosis": None,
        "fact_sheet": None,
        "capabilities": None,
        "available_profiles": _continuous_profiles(),
        "available_profile_details": _continuous_profile_records(),
        "result": _clean_json(result) if result is not None else None,
        "error": _clean_json(error) if error is not None else None,
    }
    if status:
        out["snapshot"] = status.get("snapshot")
        out["diagnosis"] = status.get("diagnosis")
        out["fact_sheet"] = status.get("fact_sheet")
        out["capabilities"] = status.get("capabilities")
        out["available_profiles"] = status.get("available_profiles") or out["available_profiles"]
        out["available_profile_details"] = status.get("available_profile_details") or out["available_profile_details"]
    return _clean_json(out)


def _json_text(payload: dict[str, Any], *, pretty: bool) -> str:
    if pretty:
        return json.dumps(payload, indent=2, sort_keys=False)
    return json.dumps(payload, separators=(",", ":"), sort_keys=False)


def _handle_status(axis: Any, args: argparse.Namespace) -> tuple[str, dict[str, Any], str, dict[str, Any] | None]:
    status = _status_bundle(axis, kv_est=args.kv_est, line_line_r_ohm=args.line_line_r_ohm)
    return "status", status, "Status refreshed", None


def _handle_diagnose(axis: Any, args: argparse.Namespace) -> tuple[str, dict[str, Any], str, dict[str, Any] | None]:
    status = _status_bundle(axis, kv_est=args.kv_est, line_line_r_ohm=args.line_line_r_ohm)
    return "diagnose", status, "Diagnosis refreshed", {"diagnosis": status.get("diagnosis")}


def _handle_fact_sheet(axis: Any, args: argparse.Namespace) -> tuple[str, dict[str, Any], str, dict[str, Any] | None]:
    status = _status_bundle(axis, kv_est=args.kv_est, line_line_r_ohm=args.line_line_r_ohm)
    return "fact-sheet", status, "Motor fact sheet refreshed", {"fact_sheet": status.get("fact_sheet")}


def _handle_idle(axis: Any, args: argparse.Namespace) -> tuple[str, dict[str, Any], str, dict[str, Any] | None]:
    axis.requested_state = common.AXIS_STATE_IDLE
    time.sleep(max(0.05, float(args.settle_s)))
    status = _status_bundle(axis, kv_est=args.kv_est, line_line_r_ohm=args.line_line_r_ohm)
    return "idle", status, "Axis set to IDLE", {"requested_state": int(common.AXIS_STATE_IDLE)}


def _handle_clear_errors(axis: Any, args: argparse.Namespace) -> tuple[str, dict[str, Any], str, dict[str, Any] | None]:
    common.clear_errors_all(axis=axis, settle_s=float(args.settle_s))
    status = _status_bundle(axis, kv_est=args.kv_est, line_line_r_ohm=args.line_line_r_ohm)
    return "clear-errors", status, "Errors cleared and axis idled", {"settle_s": float(args.settle_s)}


def _handle_startup(axis: Any, args: argparse.Namespace) -> tuple[str, dict[str, Any], str, dict[str, Any] | None]:
    axis.requested_state = common.AXIS_STATE_IDLE
    common.clear_errors_all(axis=axis, settle_s=float(args.settle_s))
    axis.requested_state = common.AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    wait_ok = bool(common.wait_idle(axis, timeout_s=float(args.timeout_s)))
    status = _status_bundle(axis, kv_est=args.kv_est, line_line_r_ohm=args.line_line_r_ohm)
    startup_ready = bool((status.get("capabilities") or {}).get("startup_ready"))
    ok = bool(wait_ok and startup_ready)
    message = "Full calibration sequence completed" if ok else "Startup did not reach a move-ready state"
    result = {
        "wait_idle_ok": bool(wait_ok),
        "startup_ready": bool(startup_ready),
        "requested_state": int(common.AXIS_STATE_FULL_CALIBRATION_SEQUENCE),
        "timeout_s": float(args.timeout_s),
    }
    return "startup", status, message, result


def _handle_move_continuous(axis: Any, args: argparse.Namespace) -> tuple[str, dict[str, Any], str, dict[str, Any] | None]:
    result = _move_to_angle_continuous(
        axis,
        angle_deg=float(args.angle_deg),
        angle_space=str(args.angle_space),
        profile_name=str(args.profile_name),
        gear_ratio=float(args.gear_ratio),
        zero_turns_motor=(None if args.zero_turns_motor is None else float(args.zero_turns_motor)),
        relative_to_current=bool(args.relative_to_current),
        timeout_s=(None if args.timeout_s is None else float(args.timeout_s)),
    )
    status = _status_bundle(axis, kv_est=args.kv_est, line_line_r_ohm=args.line_line_r_ohm)
    return "move-continuous", status, "Continuous move completed", {"move": result}


def _handle_follow_angle(axis: Any, args: argparse.Namespace) -> tuple[str, dict[str, Any], str, dict[str, Any] | None]:
    result = _issue_follow_angle_target(
        axis,
        angle_deg=float(args.angle_deg),
        angle_space=str(args.angle_space),
        profile_name=str(args.profile_name),
        gear_ratio=float(args.gear_ratio),
        zero_turns_motor=(None if args.zero_turns_motor is None else float(args.zero_turns_motor)),
        relative_to_current=bool(args.relative_to_current),
    )
    status = _status_bundle(axis, kv_est=args.kv_est, line_line_r_ohm=args.line_line_r_ohm)
    return "follow-angle", status, "Live follow target issued", {"follow": result}


def _handle_telemetry(axis: Any, args: argparse.Namespace) -> tuple[str, dict[str, Any], str, dict[str, Any] | None]:
    status = _telemetry_bundle(axis)
    result = _telemetry_result_from_status(status)
    return "telemetry", status, "Telemetry refreshed", result


def _parser(*, exit_on_error: bool = True) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Thin JSON backend for the Robot operator console",
        exit_on_error=exit_on_error,
    )
    parser.add_argument("action", choices=[
        "status",
        "diagnose",
        "fact-sheet",
        "idle",
        "clear-errors",
        "startup",
        "move-continuous",
        "move-continuous-async",
        "motion-status",
        "follow-angle",
        "telemetry",
        "profiles",
        "serve",
    ])
    parser.add_argument("--axis-index", type=int, default=0)
    parser.add_argument("--connect-timeout-s", type=float, default=DEFAULT_CONNECT_TIMEOUT_S)
    parser.add_argument("--kv-est", type=float, default=DEFAULT_KV_EST)
    parser.add_argument("--line-line-r-ohm", type=float, default=DEFAULT_LINE_LINE_R_OHM)
    parser.add_argument("--settle-s", type=float, default=0.15)
    parser.add_argument("--timeout-s", type=float)
    parser.add_argument("--debug", action="store_true")

    parser.add_argument("--angle-deg", type=float)
    parser.add_argument("--angle-space", choices=["gearbox_output", "motor"], default="gearbox_output")
    parser.add_argument("--profile-name", default=DEFAULT_PROFILE)
    parser.add_argument("--gear-ratio", type=float, default=DEFAULT_GEAR_RATIO)
    parser.add_argument("--zero-turns-motor", type=float)
    parser.add_argument("--relative-to-current", action="store_true")
    return parser


def _parse_request_args(action: str, arguments: list[str]) -> argparse.Namespace:
    parser = _parser(exit_on_error=False)
    try:
        args = parser.parse_args([str(action), *[str(item) for item in arguments]])
    except Exception as exc:
        raise ValueError(str(exc)) from exc

    if args.action in {"move-continuous", "move-continuous-async", "follow-angle"} and args.angle_deg is None:
        raise ValueError("--angle-deg is required for move-continuous/move-continuous-async/follow-angle")
    return args


def _execute_action(args: argparse.Namespace, odrv: Any, axis: Any, device: dict[str, Any]) -> tuple[int, dict[str, Any]]:
    if args.action == "status":
        action, status, message, result = _handle_status(axis, args)
    elif args.action == "diagnose":
        action, status, message, result = _handle_diagnose(axis, args)
    elif args.action == "fact-sheet":
        action, status, message, result = _handle_fact_sheet(axis, args)
    elif args.action == "idle":
        action, status, message, result = _handle_idle(axis, args)
    elif args.action == "clear-errors":
        action, status, message, result = _handle_clear_errors(axis, args)
    elif args.action == "startup":
        action, status, message, result = _handle_startup(axis, args)
    elif args.action == "move-continuous":
        action, status, message, result = _handle_move_continuous(axis, args)
    elif args.action == "move-continuous-async":
        raise RuntimeError("move-continuous-async is only supported in serve mode")
    elif args.action == "motion-status":
        raise RuntimeError("motion-status is only supported in serve mode")
    elif args.action == "follow-angle":
        action, status, message, result = _handle_follow_angle(axis, args)
    elif args.action == "telemetry":
        action, status, message, result = _handle_telemetry(axis, args)
    else:
        raise RuntimeError(f"Unsupported action '{args.action}'")

    ok = True
    if action == "startup":
        ok = bool((result or {}).get("startup_ready"))
    payload = _result_envelope(
        ok=ok,
        action=action,
        device=device,
        message=message,
        status=status,
        result=result,
    )
    return (0 if ok else 2), payload


class _PersistentServer:
    def __init__(self):
        self._odrv = None
        self._axis = None
        self._device = None
        self._axis_index = None
        self._motion_lock = threading.Lock()
        self._motion_thread = None
        self._motion_latest_status = None
        self._motion_latest_result = None
        self._motion_error = None
        self._motion_started_s = None
        self._motion_completed_s = None

    def _reset_connection(self) -> None:
        self._odrv = None
        self._axis = None
        self._device = None
        self._axis_index = None

    def _ensure_connection(self, args: argparse.Namespace) -> tuple[Any, Any, dict[str, Any]]:
        axis_index = int(args.axis_index)
        if self._axis is not None and self._odrv is not None and self._axis_index == axis_index:
            return self._odrv, self._axis, dict(self._device or {})
        self._odrv, self._axis = _connect(axis_index=axis_index, timeout_s=float(args.connect_timeout_s))
        self._axis_index = axis_index
        self._device = _device_info(self._odrv, axis_index)
        return self._odrv, self._axis, dict(self._device or {})

    def _motion_active(self) -> bool:
        return bool(self._motion_thread is not None and self._motion_thread.is_alive())

    def _motion_snapshot_status(self) -> dict[str, Any] | None:
        with self._motion_lock:
            if self._motion_latest_status is None:
                return None
            return dict(self._motion_latest_status)

    def _motion_status_payload(self) -> dict[str, Any]:
        with self._motion_lock:
            active = bool(self._motion_thread is not None and self._motion_thread.is_alive())
            return {
                "active": bool(active),
                "started_s": self._motion_started_s,
                "completed_s": self._motion_completed_s,
                "latest_result": _clean_json(self._motion_latest_result),
                "error": _clean_json(self._motion_error),
            }

    def _publish_motion_sample(self, sample: dict[str, Any]) -> None:
        snapshot = _normalize_snapshot(sample or {})
        status = {
            "snapshot": _clean_json(snapshot),
            "diagnosis": None,
            "fact_sheet": None,
            "capabilities": {
                "can_startup": False,
                "can_idle": False,
                "can_clear_errors": False,
                "can_diagnose": False,
                "can_fact_sheet": False,
                "can_move_continuous": False,
                "can_move_continuous_aggressive": False,
                "can_capture_zero_here": False,
                "startup_ready": bool(snapshot.get("enc_ready")) and not bool(
                    int(snapshot.get("axis_err") or 0)
                    or int(snapshot.get("motor_err") or 0)
                    or int(snapshot.get("enc_err") or 0)
                    or int(snapshot.get("ctrl_err") or 0)
                ),
                "armed": bool(int(snapshot.get("state") or 0) == int(common.AXIS_STATE_CLOSED_LOOP_CONTROL)),
                "idle": bool(int(snapshot.get("state") or 0) == int(common.AXIS_STATE_IDLE)),
                "has_latched_errors": bool(
                    int(snapshot.get("axis_err") or 0)
                    or int(snapshot.get("motor_err") or 0)
                    or int(snapshot.get("enc_err") or 0)
                    or int(snapshot.get("ctrl_err") or 0)
                ),
                "motion_active": True,
            },
            "available_profiles": _continuous_profiles(),
            "available_profile_details": _continuous_profile_records(),
        }
        with self._motion_lock:
            self._motion_latest_status = status

    def _run_background_move(self, args: argparse.Namespace) -> None:
        try:
            _, axis, _ = self._ensure_connection(args)
            result = _move_to_angle_continuous(
                axis,
                angle_deg=float(args.angle_deg),
                angle_space=str(args.angle_space),
                profile_name=str(args.profile_name),
                gear_ratio=float(args.gear_ratio),
                zero_turns_motor=(None if args.zero_turns_motor is None else float(args.zero_turns_motor)),
                relative_to_current=bool(args.relative_to_current),
                timeout_s=(None if args.timeout_s is None else float(args.timeout_s)),
                sample_hook=self._publish_motion_sample,
            )
            final_status = _status_bundle(axis, kv_est=args.kv_est, line_line_r_ohm=args.line_line_r_ohm)
            final_status = _mark_motion_capabilities(final_status, motion_active=False)
            with self._motion_lock:
                self._motion_latest_status = final_status
                self._motion_latest_result = {"move": _clean_json(result)}
                self._motion_error = None
                self._motion_completed_s = time.time()
        except Exception as exc:
            status = None
            if self._axis is not None:
                try:
                    status = _status_bundle(self._axis, kv_est=args.kv_est, line_line_r_ohm=args.line_line_r_ohm)
                    status = _mark_motion_capabilities(status, motion_active=False)
                except Exception:
                    status = None
            with self._motion_lock:
                self._motion_latest_status = status
                self._motion_latest_result = None
                self._motion_error = {"type": exc.__class__.__name__, "message": str(exc)}
                self._motion_completed_s = time.time()

    def _start_background_move(self, args: argparse.Namespace) -> dict[str, Any]:
        if self._motion_active():
            raise RuntimeError("A background continuous move is already active.")
        _, axis, _ = self._ensure_connection(args)
        initial_status = _status_bundle(axis, kv_est=args.kv_est, line_line_r_ohm=args.line_line_r_ohm)
        initial_status = _mark_motion_capabilities(initial_status, motion_active=True)
        with self._motion_lock:
            self._motion_latest_status = initial_status
            self._motion_latest_result = None
            self._motion_error = None
            self._motion_started_s = time.time()
            self._motion_completed_s = None
        self._motion_thread = threading.Thread(
            target=self._run_background_move,
            args=(args,),
            name="focui-background-move",
            daemon=True,
        )
        self._motion_thread.start()
        return {
            "accepted": True,
            "motion_active": True,
            "profile_name": str(args.profile_name),
            "angle_deg": float(args.angle_deg),
            "angle_space": str(args.angle_space),
        }

    def handle_request(self, request: dict[str, Any]) -> dict[str, Any]:
        request_id = None if request.get("id") is None else str(request.get("id"))
        action = str(request.get("action") or "").strip()
        arguments = list(request.get("arguments") or [])
        parsed_args = None

        if action == "shutdown":
            return _result_envelope(
                ok=True,
                action="shutdown",
                device=_clean_json(self._device),
                message="Backend server shutting down",
                result={"shutdown": True},
                request_id=request_id,
            )

        try:
            parsed_args = _parse_request_args(action, arguments)
            if parsed_args.action == "serve":
                raise ValueError("'serve' is reserved for backend startup")

            if parsed_args.action == "profiles":
                payload = _result_envelope(
                    ok=True,
                    action="profiles",
                    device=None,
                    message="Loaded continuous-move profiles",
                    status=None,
                    result={"profiles": _continuous_profile_records()},
                    request_id=request_id,
                )
                return payload

            if parsed_args.action == "move-continuous-async":
                _, axis, device = self._ensure_connection(parsed_args)
                status = _mark_motion_capabilities(
                    _status_bundle(axis, kv_est=parsed_args.kv_est, line_line_r_ohm=parsed_args.line_line_r_ohm),
                    motion_active=True,
                )
                result = self._start_background_move(parsed_args)
                return _result_envelope(
                    ok=True,
                    action="move-continuous-async",
                    device=device,
                    message="Background continuous move started",
                    status=status,
                    result=result,
                    request_id=request_id,
                )

            if parsed_args.action == "motion-status":
                status = self._motion_snapshot_status()
                if status is None:
                    _, axis, device = self._ensure_connection(parsed_args)
                    status = _mark_motion_capabilities(
                        _telemetry_bundle(axis),
                        motion_active=False,
                    )
                else:
                    status = _mark_motion_capabilities(status, motion_active=self._motion_active())
                    device = dict(self._device or {})
                return _result_envelope(
                    ok=True,
                    action="motion-status",
                    device=device,
                    message="Background motion status refreshed",
                    status=status,
                    result=self._motion_status_payload(),
                    request_id=request_id,
                )

            if parsed_args.action == "telemetry" and self._motion_active():
                status = self._motion_snapshot_status()
                if status is None:
                    _, axis, _ = self._ensure_connection(parsed_args)
                    status = _mark_motion_capabilities(_telemetry_bundle(axis), motion_active=True)
                else:
                    status = _mark_motion_capabilities(status, motion_active=True)
                return _result_envelope(
                    ok=True,
                    action="telemetry",
                    device=_clean_json(self._device),
                    message="Telemetry refreshed",
                    status=status,
                    result=_telemetry_result_from_status(status),
                    request_id=request_id,
                )

            if self._motion_active() and parsed_args.action not in {"telemetry", "motion-status"}:
                raise RuntimeError("Background continuous move is active. Wait for completion before running this action.")

            odrv, axis, device = self._ensure_connection(parsed_args)
            _, payload = _execute_action(parsed_args, odrv, axis, device)
            payload["request_id"] = request_id
            if self._motion_active():
                payload = _mark_motion_capabilities(payload, motion_active=True)
            return payload

        except Exception as exc:
            status = None
            if self._axis is not None:
                try:
                    status = _status_bundle(
                        self._axis,
                        kv_est=(DEFAULT_KV_EST if parsed_args is None else parsed_args.kv_est),
                        line_line_r_ohm=(DEFAULT_LINE_LINE_R_OHM if parsed_args is None else parsed_args.line_line_r_ohm),
                    )
                except Exception:
                    status = None
                    self._reset_connection()
            error = {
                "type": exc.__class__.__name__,
                "message": str(exc),
            }
            payload = _result_envelope(
                ok=False,
                action=(action or "unknown"),
                device=_clean_json(self._device),
                message=str(exc),
                status=status,
                error=error,
                request_id=request_id,
            )
            return payload


def _serve_forever() -> int:
    server = _PersistentServer()
    ready_payload = _result_envelope(
        ok=True,
        action="serve",
        device=None,
        message="Backend server ready",
        result={"server_ready": True, "pid": int(getattr(os, "getpid")())},
    )
    print(_json_text(ready_payload, pretty=False), flush=True)

    for raw_line in sys.stdin:
        line = str(raw_line).strip()
        if not line:
            continue
        try:
            request = json.loads(line)
            if not isinstance(request, dict):
                raise ValueError("Request must be a JSON object")
        except Exception as exc:
            payload = _result_envelope(
                ok=False,
                action="invalid-request",
                device=None,
                message=str(exc),
                error={"type": exc.__class__.__name__, "message": str(exc)},
            )
            print(_json_text(payload, pretty=False), flush=True)
            continue

        payload = server.handle_request(request)
        print(_json_text(payload, pretty=False), flush=True)
        if str(request.get("action") or "") == "shutdown":
            break
    return 0


def main() -> int:
    parser = _parser()
    args = parser.parse_args()

    if args.action == "serve":
        return _serve_forever()

    if args.action == "profiles":
        payload = _result_envelope(
            ok=True,
            action="profiles",
            device=None,
            message="Loaded continuous-move profiles",
            status=None,
            result={"profiles": _continuous_profile_records()},
        )
        print(_json_text(payload, pretty=True))
        return 0

    try:
        args = _parse_request_args(args.action, sys.argv[2:])
    except Exception as exc:
        parser.error(str(exc))

    odrv = None
    axis = None
    device = None

    try:
        odrv, axis = _connect(axis_index=int(args.axis_index), timeout_s=float(args.connect_timeout_s))
        device = _device_info(odrv, int(args.axis_index))
        exit_code, payload = _execute_action(args, odrv, axis, device)
        print(_json_text(payload, pretty=True))
        return exit_code

    except Exception as exc:
        status = None
        if axis is not None:
            try:
                status = _status_bundle(axis, kv_est=args.kv_est, line_line_r_ohm=args.line_line_r_ohm)
            except Exception:
                status = None
        error = {
            "type": exc.__class__.__name__,
            "message": str(exc),
        }
        if args.debug:
            error["traceback"] = traceback.format_exc()
        payload = _result_envelope(
            ok=False,
            action=args.action,
            device=device,
            message=str(exc),
            status=status,
            error=error,
        )
        print(_json_text(payload, pretty=True))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
