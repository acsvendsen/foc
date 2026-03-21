#!/usr/bin/env python3
from __future__ import annotations

import argparse
import contextlib
import datetime
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
from odrive.device_manager import get_device_manager  # type: ignore
from odrive.libodrive import DeviceType  # type: ignore

import common  # type: ignore
DEFAULT_KV_EST = 140.0
DEFAULT_LINE_LINE_R_OHM = 0.30
DEFAULT_GEAR_RATIO = 25.0
DEFAULT_PROFILE = "gearbox_output_continuous_quiet_20260309"
DEFAULT_CONNECT_TIMEOUT_S = 3.0
MAX_STAGE_LOG_CHARS = 6000


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


def _truncate_text(text: str | None, max_chars: int = MAX_STAGE_LOG_CHARS) -> str | None:
    if text is None:
        return None
    s = str(text)
    if len(s) <= int(max_chars):
        return s
    tail = max(0, int(max_chars) - 64)
    return s[:tail] + "\n...<truncated>..."


def _save_axis_configuration(axis: Any) -> None:
    parent = getattr(axis, "_parent", None)
    if parent is None:
        raise RuntimeError("Axis parent device is unavailable; cannot save configuration")
    parent.save_configuration()


def _run_direction_validation_contract(
    axis: Any,
    *,
    cycles: int,
    cmd_delta_turns: float,
    current_lim: float,
) -> dict[str, Any]:
    validate_cycles = max(2, min(4, int(cycles)))
    validate_delta = max(0.02, float(cmd_delta_turns) * 3.0)
    validate_current_lim = max(4.0, min(float(current_lim), 8.0))
    result = common.hardware_sign_consistency_validation(
        axis=axis,
        cycles=int(validate_cycles),
        cmd_delta_turns=float(validate_delta),
        current_lim=float(validate_current_lim),
        pos_gain=10.0,
        vel_gain=0.18,
        vel_i_gain=0.0,
        vel_limit=0.35,
        settle_between_s=0.12,
        run_preflight=False,
        run_jump_vs_slip=False,
        save_path=None,
        verbose=False,
    )
    summary = dict((result or {}).get("summary") or {})
    return {
        "ok": bool((result or {}).get("ok")),
        "classification": str((result or {}).get("classification") or "unknown"),
        "interpretation": str((result or {}).get("interpretation") or ""),
        "cycles": int(validate_cycles),
        "cmd_delta_turns": float(validate_delta),
        "current_lim": float(validate_current_lim),
        "summary": {
            "sign_ok": int(summary.get("sign_ok", 0)),
            "sign_inverted": int(summary.get("sign_inverted", 0)),
            "errors": int(summary.get("errors", 0)),
            "counts_by_classification": _clean_json(summary.get("counts_by_classification") or {}),
        },
    }


def _normalize_snapshot(snapshot: dict[str, Any] | None) -> dict[str, Any]:
    out = dict(snapshot or {})
    for key in ("enc_ready", "enc_use_index", "enc_index_found"):
        if key in out and out.get(key) is not None:
            out[key] = bool(out.get(key))
    return out


def _connect(axis_index: int, timeout_s: float, serial_number: str | None = None):
    try:
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            serials = None if not str(serial_number or "").strip() else str(serial_number).strip()
            odrv = odrive.find_any(serial_number=serials, timeout=float(timeout_s))
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


def _discover_runtime_devices(wait_s: float = 0.25) -> list[dict[str, Any]]:
    manager = get_device_manager()
    deadline = time.time() + max(0.05, float(wait_s))
    while time.time() < deadline:
        runtime_devices = [
            dev for dev in list(manager.devices)
            if getattr(getattr(dev, "info", None), "device_type", None) == DeviceType.RUNTIME
        ]
        if runtime_devices:
            break
        time.sleep(0.05)
    rows: list[dict[str, Any]] = []
    for dev in list(manager.devices):
        info = getattr(dev, "info", None)
        if getattr(info, "device_type", None) != DeviceType.RUNTIME:
            continue
        rows.append({
            "serial_number": str(getattr(info, "serial_number", "")),
            "manufacturer": _clean_json(getattr(info, "manufacturer", None)),
            "product": _clean_json(getattr(info, "product", None)),
        })
    rows.sort(key=lambda row: str(row.get("serial_number") or ""))
    return rows


def _write_json_atomic(path: Path, payload: dict[str, Any]) -> None:
    tmp = path.with_name(f"{path.name}.tmp")
    tmp.write_text(json.dumps(payload, indent=2, sort_keys=False) + "\n")
    os.replace(tmp, path)


def _continuous_profile_editor_payload(profile_name: str) -> dict[str, Any]:
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
        "name": key,
        "notes": str(prof.get("notes") or ""),
        "source": str(prof.get("source") or ""),
        "load_mode": str(prof.get("load_mode") or "loaded"),
        "require_repeatability": bool(prof.get("require_repeatability", False)),
        "stop_on_frame_jump": bool(prof.get("stop_on_frame_jump", True)),
        "stop_on_hard_fault": bool(prof.get("stop_on_hard_fault", True)),
        "limitations": list(prof.get("limitations") or []),
        "current_lim": float(suite.get("current_lim", 6.5)),
        "enable_overspeed_error": bool(suite.get("enable_overspeed_error", False)),
        "pos_gain": float(suite.get("pos_gain", 12.0)),
        "vel_gain": float(suite.get("vel_gain", 0.22)),
        "vel_i_gain": float(suite.get("vel_i_gain", 0.0)),
        "trap_vel": float(suite.get("trap_vel", 0.28)),
        "trap_acc": float(suite.get("trap_acc", 0.32)),
        "trap_dec": float(suite.get("trap_dec", 0.32)),
        "vel_limit": float(suite.get("vel_limit", 0.40)),
        "vel_limit_tolerance": float(suite.get("vel_limit_tolerance", 4.0)),
        "stiction_kick_nm": float(suite.get("stiction_kick_nm", 0.0)),
        "target_tolerance_turns": float(step.get("target_tolerance_turns", 0.03)),
        "target_vel_tolerance_turns_s": float(step.get("target_vel_tolerance_turns_s", 0.20)),
        "timeout_s": float(extra.get("timeout_s", 8.0)),
        "min_delta_turns": float(extra.get("min_delta_turns", 0.0015)),
        "settle_s": float(extra.get("settle_s", 0.08)),
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


def _save_continuous_profile_editor_payload(profile_payload: dict[str, Any]) -> dict[str, Any]:
    p = _profiles_path()
    data = json.loads(p.read_text())
    profiles = dict(data.get("profiles") or {})

    name = str(profile_payload.get("name") or "").strip()
    if not name:
        raise ValueError("Profile name is required")

    existing = dict(profiles.get(name) or {})
    limitations = [str(item).strip() for item in list(profile_payload.get("limitations") or []) if str(item).strip()]

    suite = {
        "current_lim": float(profile_payload.get("current_lim", 6.5)),
        "enable_overspeed_error": bool(profile_payload.get("enable_overspeed_error", False)),
        "pos_gain": float(profile_payload.get("pos_gain", 12.0)),
        "vel_gain": float(profile_payload.get("vel_gain", 0.22)),
        "vel_i_gain": float(profile_payload.get("vel_i_gain", 0.0)),
        "trap_vel": float(profile_payload.get("trap_vel", 0.28)),
        "trap_acc": float(profile_payload.get("trap_acc", 0.32)),
        "trap_dec": float(profile_payload.get("trap_dec", 0.32)),
        "vel_limit": float(profile_payload.get("vel_limit", 0.40)),
        "vel_limit_tolerance": float(profile_payload.get("vel_limit_tolerance", 4.0)),
        "stiction_kick_nm": float(profile_payload.get("stiction_kick_nm", 0.0)),
        "step_kwargs": {
            "target_tolerance_turns": float(profile_payload.get("target_tolerance_turns", 0.03)),
            "target_vel_tolerance_turns_s": float(profile_payload.get("target_vel_tolerance_turns_s", 0.20)),
        },
    }
    continuous = {
        "timeout_s": float(profile_payload.get("timeout_s", 8.0)),
        "min_delta_turns": float(profile_payload.get("min_delta_turns", 0.0015)),
        "settle_s": float(profile_payload.get("settle_s", 0.08)),
        "quiet_hold_enable": bool(profile_payload.get("quiet_hold_enable", True)),
        "quiet_hold_s": float(profile_payload.get("quiet_hold_s", 0.06)),
        "quiet_hold_pos_gain_scale": float(profile_payload.get("quiet_hold_pos_gain_scale", 0.45)),
        "quiet_hold_vel_gain_scale": float(profile_payload.get("quiet_hold_vel_gain_scale", 0.70)),
        "quiet_hold_vel_i_gain": float(profile_payload.get("quiet_hold_vel_i_gain", 0.0)),
        "quiet_hold_vel_limit_scale": float(profile_payload.get("quiet_hold_vel_limit_scale", 0.50)),
        "quiet_hold_persist": bool(profile_payload.get("quiet_hold_persist", True)),
        "quiet_hold_reanchor_err_turns": (
            None if profile_payload.get("quiet_hold_reanchor_err_turns") in (None, "")
            else float(profile_payload.get("quiet_hold_reanchor_err_turns"))
        ),
        "fail_to_idle": bool(profile_payload.get("fail_to_idle", False)),
    }

    rec = {
        "profile_name": name,
        "load_mode": str(profile_payload.get("load_mode") or existing.get("load_mode") or "loaded"),
        "source": str(profile_payload.get("source") or existing.get("source") or "focui_manual_editor"),
        "notes": str(profile_payload.get("notes") or ""),
        "require_repeatability": bool(profile_payload.get("require_repeatability", False)),
        "stop_on_frame_jump": bool(profile_payload.get("stop_on_frame_jump", True)),
        "stop_on_hard_fault": bool(profile_payload.get("stop_on_hard_fault", True)),
        "suite_kwargs": suite,
        "continuous_kwargs": continuous,
        "validated_targets_deg": [],
        "limitations": limitations,
    }

    profiles[name] = rec
    data["profiles"] = profiles
    data["updated_at"] = datetime.datetime.now().isoformat()
    _write_json_atomic(p, data)
    return {
        "ok": True,
        "profile_name": name,
        "profiles_path": str(p),
        "record": rec,
        "editor": _continuous_profile_editor_payload(name),
    }


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
    fail_to_idle_override: bool | None = None,
    sample_hook=None,
) -> dict[str, Any]:
    cfg = _load_continuous_move_kwargs(profile_name=profile_name)
    if fail_to_idle_override is not None:
        cfg["fail_to_idle"] = bool(fail_to_idle_override)
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
        "available_profiles": None,
        "available_profile_details": None,
    }


def _telemetry_result_from_status(status: dict[str, Any]) -> dict[str, Any]:
    snap = dict(status.get("snapshot") or {})
    iq_meas = snap.get("Iq_meas")
    tc = snap.get("tc")
    return {
        "timestamp_s": time.time(),
        "pos_est": snap.get("pos_est"),
        "vel_est": snap.get("vel_est"),
        "Iq_meas": iq_meas,
        "input_pos": snap.get("input_pos"),
        "tracking_err_turns": (
            None
            if (snap.get("input_pos") is None or snap.get("pos_est") is None)
            else (float(snap.get("input_pos")) - float(snap.get("pos_est")))
        ),
        "estimated_motor_torque_nm": (
            None
            if (iq_meas is None or tc is None)
            else (float(iq_meas) * float(tc))
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
                     profile_editor: dict[str, Any] | None = None,
                     error: dict[str, Any] | None = None, request_id: str | None = None,
                     include_catalog: bool = True) -> dict[str, Any]:
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
        "available_profiles": (_continuous_profiles() if include_catalog else None),
        "available_profile_details": (_continuous_profile_records() if include_catalog else None),
        "profile_editor": _clean_json(profile_editor) if profile_editor is not None else None,
        "result": _clean_json(result) if result is not None else None,
        "error": _clean_json(error) if error is not None else None,
    }
    if status:
        out["snapshot"] = status.get("snapshot")
        out["diagnosis"] = status.get("diagnosis")
        out["fact_sheet"] = status.get("fact_sheet")
        out["capabilities"] = status.get("capabilities")
        if include_catalog:
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


def _handle_set_motor_direction(axis: Any, args: argparse.Namespace) -> tuple[str, dict[str, Any], str, dict[str, Any] | None]:
    desired_direction = int(args.direction)
    if desired_direction not in (-1, 1):
        raise ValueError("--direction must be either -1 or 1")

    state_before = int(getattr(axis, "current_state", 0) or 0)
    if state_before != int(common.AXIS_STATE_IDLE):
        try:
            axis.requested_state = common.AXIS_STATE_IDLE
        except Exception:
            pass
        time.sleep(max(0.05, float(args.settle_s)))

    try:
        axis.motor.config.direction = int(desired_direction)
    except Exception as exc:
        raise RuntimeError(f"Failed to set motor direction: {exc}") from exc

    time.sleep(max(0.02, min(0.10, float(args.settle_s))))
    status = _status_bundle(axis, kv_est=args.kv_est, line_line_r_ohm=args.line_line_r_ohm)
    applied_direction = ((status.get("snapshot") or {}).get("motor_direction"))
    message = f"Motor direction set to {int(desired_direction):+d} (runtime only; not saved)"
    result = {
        "requested_direction": int(desired_direction),
        "applied_direction": (None if applied_direction is None else int(applied_direction)),
        "persisted": False,
        "state_before": int(state_before),
        "state_after": int(getattr(axis, "current_state", 0) or 0),
    }
    return "set-motor-direction", status, message, result


def _handle_auto_direction_contract(axis: Any, args: argparse.Namespace) -> tuple[str, dict[str, Any], str, dict[str, Any] | None]:
    state_before = int(getattr(axis, "current_state", 0) or 0)
    try:
        current_lim = float(getattr(axis.motor.config, "current_lim", 8.0))
    except Exception:
        current_lim = 8.0

    result = common.auto_direction_contract(
        axis=axis,
        candidate_directions=(-1, 1),
        cycles=max(2, int(getattr(args, "cycles", 4) or 4)),
        cmd_delta_turns=float(getattr(args, "cmd_delta_turns", 0.01) or 0.01),
        current_lim=max(4.0, min(float(current_lim), 10.0)),
        pos_gain=16.0,
        vel_gain=0.24,
        vel_i_gain=0.0,
        vel_limit=0.6,
        settle_between_s=0.12,
        run_jump_vs_slip=False,
        jump_hold_s=3.0,
        persist=False,
        save_path=None,
        verbose=False,
    )
    validation = _run_direction_validation_contract(
        axis,
        cycles=max(2, int(getattr(args, "cycles", 4) or 4)),
        cmd_delta_turns=float(getattr(args, "cmd_delta_turns", 0.01) or 0.01),
        current_lim=max(4.0, min(float(current_lim), 10.0)),
    )
    status = _status_bundle(axis, kv_est=args.kv_est, line_line_r_ohm=args.line_line_r_ohm)
    selected = int((result or {}).get("selected_direction", (status.get("snapshot") or {}).get("motor_direction") or 1))
    auto_ok = bool((result or {}).get("ok"))
    ok = bool(auto_ok and validation.get("ok"))
    persisted = False
    persist_error = None
    if ok and bool(getattr(args, "persist", False)):
        try:
            _save_axis_configuration(axis)
            persisted = True
        except Exception as exc:
            persist_error = str(exc)
            ok = False
    message = (
        f"Auto direction selected {selected:+d} and validated"
        if ok else
        f"Auto direction selected {selected:+d} but validation is not trustworthy; inspect result before using it"
    )
    summary = dict((result or {}).get("summary") or {})
    payload = {
        "selected_direction": int(selected),
        "confidence": _clean_json((result or {}).get("confidence")),
        "ok": bool(ok),
        "auto_contract_ok": bool(auto_ok),
        "trust_result": bool(ok),
        "persisted": bool(persisted),
        "trust_reason": (
            "Selected direction passed the stronger post-selection sign contract."
            if ok else
            "The selected direction did not survive stronger post-selection sign validation."
        ),
        "winner_classification": summary.get("winner_classification"),
        "winner_sign_ok": summary.get("winner_sign_ok"),
        "winner_sign_inverted": summary.get("winner_sign_inverted"),
        "runner_up_score_margin": summary.get("runner_up_score_margin"),
        "validation_ok": bool(validation.get("ok")),
        "validation_classification": validation.get("classification"),
        "validation_interpretation": validation.get("interpretation"),
        "validation_summary": validation.get("summary"),
        "validation_config": {
            "cycles": validation.get("cycles"),
            "cmd_delta_turns": validation.get("cmd_delta_turns"),
            "current_lim": validation.get("current_lim"),
        },
        "state_before": int(state_before),
        "state_after": int(getattr(axis, "current_state", 0) or 0),
        "persist_error": persist_error,
        "raw": _clean_json(result),
    }
    return "auto-direction-contract", status, message, payload


def _guided_motor_calibration(axis: Any) -> dict[str, Any]:
    try:
        is_cal = bool(getattr(axis.motor, "is_calibrated", False))
    except Exception:
        is_cal = False
    try:
        phase_r = float(getattr(axis.motor.config, "phase_resistance", 0.0))
    except Exception:
        phase_r = 0.0
    try:
        phase_l = float(getattr(axis.motor.config, "phase_inductance", 0.0))
    except Exception:
        phase_l = 0.0

    if bool(is_cal) or ((phase_r > 0.0) and (phase_l > 0.0)):
        return {
            "ok": True,
            "skipped": True,
            "message": "Motor already has valid calibration data.",
            "phase_resistance": float(phase_r),
            "phase_inductance": float(phase_l),
            "is_calibrated": bool(is_cal),
        }

    try:
        current_lim = float(getattr(axis.motor.config, "current_lim", 10.0))
    except Exception:
        current_lim = 10.0
    attempts = []
    sweep = []
    for candidate in [
        max(2.0, min(current_lim * 0.70, current_lim - 0.5)),
        max(2.0, min(current_lim * 0.85, current_lim - 0.5)),
        max(2.0, min(current_lim * 0.95, current_lim - 0.5)),
    ]:
        rounded = round(float(candidate), 3)
        if rounded not in [round(float(x), 3) for x in sweep]:
            sweep.append(float(candidate))

    last_error = None
    for calibration_current in sweep:
        common.clear_errors_all(axis=axis, settle_s=0.10)
        try:
            axis.motor.config.calibration_current = float(calibration_current)
        except Exception:
            pass
        axis.requested_state = common.AXIS_STATE_MOTOR_CALIBRATION
        wait_ok = bool(common.wait_idle(axis, timeout_s=12.0))
        try:
            phase_r = float(getattr(axis.motor.config, "phase_resistance", 0.0))
        except Exception:
            phase_r = 0.0
        try:
            phase_l = float(getattr(axis.motor.config, "phase_inductance", 0.0))
        except Exception:
            phase_l = 0.0
        try:
            is_cal = bool(getattr(axis.motor, "is_calibrated", False))
        except Exception:
            is_cal = False
        axis_err = int(getattr(axis, "error", 0) or 0)
        motor_err = int(getattr(axis.motor, "error", 0) or 0)
        attempt = {
            "calibration_current": float(calibration_current),
            "wait_idle_ok": bool(wait_ok),
            "axis_err": int(axis_err),
            "motor_err": int(motor_err),
            "phase_resistance": float(phase_r),
            "phase_inductance": float(phase_l),
            "is_calibrated": bool(is_cal),
        }
        attempts.append(attempt)
        if bool(wait_ok) and int(axis_err) == 0 and int(motor_err) == 0 and bool(is_cal) and (phase_r > 0.0) and (phase_l > 0.0):
            return {
                "ok": True,
                "skipped": False,
                "message": "Motor calibration completed.",
                "attempts": attempts,
                "phase_resistance": float(phase_r),
                "phase_inductance": float(phase_l),
                "is_calibrated": bool(is_cal),
            }
        last_error = f"axis_err=0x{axis_err:x} motor_err=0x{motor_err:x}"
        common.clear_errors_all(axis=axis, settle_s=0.08)

    return {
        "ok": False,
        "skipped": False,
        "message": "Motor calibration failed.",
        "attempts": attempts,
        "error": last_error,
        "phase_resistance": float(phase_r),
        "phase_inductance": float(phase_l),
        "is_calibrated": bool(is_cal),
    }


def _capture_noisy_call(fn, *args, **kwargs) -> tuple[Any, str]:
    out_buf = io.StringIO()
    err_buf = io.StringIO()
    try:
        with contextlib.redirect_stdout(out_buf), contextlib.redirect_stderr(err_buf):
            result = fn(*args, **kwargs)
    except Exception as exc:
        combined = (out_buf.getvalue() or "") + (("\n" + err_buf.getvalue()) if err_buf.getvalue() else "")
        try:
            setattr(exc, "_captured_logs", _truncate_text(combined))
        except Exception:
            pass
        raise
    combined = (out_buf.getvalue() or "") + (("\n" + err_buf.getvalue()) if err_buf.getvalue() else "")
    return result, _truncate_text(combined)


def _handle_guided_bringup(axis: Any, args: argparse.Namespace) -> tuple[str, dict[str, Any], str, dict[str, Any] | None]:
    stages: list[dict[str, Any]] = []

    def add_stage(name: str, ok: bool, message: str, *, skipped: bool = False,
                  details: dict[str, Any] | None = None, logs: str | None = None) -> None:
        stages.append({
            "name": str(name),
            "ok": bool(ok),
            "skipped": bool(skipped),
            "message": str(message),
            "details": _clean_json(details or {}),
            "logs": _truncate_text(logs),
        })

    common.clear_errors_all(axis=axis, settle_s=float(args.settle_s))
    try:
        axis.requested_state = common.AXIS_STATE_IDLE
    except Exception:
        pass
    time.sleep(max(0.05, float(args.settle_s)))
    add_stage(
        "clear_idle",
        True,
        "Axis idled and errors cleared.",
        details={"snapshot": _clean_json(common._snapshot_motion(axis))},
    )

    motor_stage = _guided_motor_calibration(axis)
    add_stage(
        "motor_calibration",
        bool(motor_stage.get("ok")),
        str(motor_stage.get("message") or "Motor calibration stage finished."),
        skipped=bool(motor_stage.get("skipped", False)),
        details={k: v for k, v in dict(motor_stage).items() if k not in {"ok", "message", "skipped"}},
    )
    if not bool(motor_stage.get("ok")):
        status = _status_bundle(axis, kv_est=args.kv_est, line_line_r_ohm=args.line_line_r_ohm)
        return "guided-bringup", status, "Guided bring-up failed at motor calibration", {
            "ok": False,
            "failed_stage": "motor_calibration",
            "stages": stages,
        }

    try:
        cpr = int(getattr(axis.encoder.config, "cpr", 1024))
    except Exception:
        cpr = 1024
    try:
        bandwidth = float(getattr(axis.encoder.config, "bandwidth", 10.0))
    except Exception:
        bandwidth = 10.0
    try:
        interp = bool(getattr(axis.encoder.config, "enable_phase_interpolation", False))
    except Exception:
        interp = False

    try:
        _, preflight_logs = _capture_noisy_call(
            common.preflight_encoder,
            axis,
            cpr=cpr,
            bandwidth=bandwidth,
            interp=interp,
        )
        add_stage(
            "encoder_preflight",
            True,
            "Encoder preflight passed.",
            details={"cpr": int(cpr), "bandwidth": float(bandwidth), "interp": bool(interp)},
            logs=preflight_logs,
        )
    except Exception as exc:
        add_stage(
            "encoder_preflight",
            False,
            str(exc),
            details={"cpr": int(cpr), "bandwidth": float(bandwidth), "interp": bool(interp)},
            logs=_truncate_text(getattr(exc, "_captured_logs", None)),
        )
        status = _status_bundle(axis, kv_est=args.kv_est, line_line_r_ohm=args.line_line_r_ohm)
        return "guided-bringup", status, "Guided bring-up failed at encoder preflight", {
            "ok": False,
            "failed_stage": "encoder_preflight",
            "stages": stages,
        }

    startup_result = common.move_startup_contract(
        axis,
        startup_mode="guarded",
        require_index=None,
        run_index_search_on_recover=None,
        require_encoder_ready=True,
        timeout_s=3.0,
        sync_settle_s=0.03,
        stability_observe_s=0.25,
        stability_dt=0.02,
    )
    add_stage(
        "startup_contract",
        bool(startup_result.get("ok")),
        ("Startup contract passed." if bool(startup_result.get("ok")) else str(startup_result.get("error") or "Startup contract failed.")),
        details={
            "ok": bool(startup_result.get("ok")),
            "mode": startup_result.get("mode"),
            "require_index": startup_result.get("require_index"),
            "use_index": startup_result.get("use_index"),
            "enc_ready_start": startup_result.get("enc_ready_start"),
            "index_found_start": startup_result.get("index_found_start"),
            "closed_loop_ok": startup_result.get("closed_loop_ok"),
            "sync_ok": startup_result.get("sync_ok"),
            "stability_probe": startup_result.get("stability_probe"),
            "error": startup_result.get("error"),
        },
    )
    if not bool(startup_result.get("ok")):
        status = _status_bundle(axis, kv_est=args.kv_est, line_line_r_ohm=args.line_line_r_ohm)
        return "guided-bringup", status, "Guided bring-up failed at startup contract", {
            "ok": False,
            "failed_stage": "startup_contract",
            "stages": stages,
        }

    auto_dir_result = common.auto_direction_contract(
        axis=axis,
        candidate_directions=(-1, 1),
        cycles=max(2, int(getattr(args, "cycles", 4) or 4)),
        cmd_delta_turns=float(getattr(args, "cmd_delta_turns", 0.01) or 0.01),
        current_lim=max(6.0, min(float(getattr(axis.motor.config, "current_lim", 8.0)), 10.0)),
        pos_gain=16.0,
        vel_gain=0.24,
        vel_i_gain=0.0,
        vel_limit=0.6,
        settle_between_s=0.12,
        run_jump_vs_slip=False,
        jump_hold_s=3.0,
        persist=False,
        save_path=None,
        verbose=False,
    )
    add_stage(
        "auto_direction_contract",
        bool(auto_dir_result.get("ok")),
        ("Auto direction passed." if bool(auto_dir_result.get("ok")) else "Auto direction returned low confidence or inconsistent sign behavior."),
        details={
            "ok": bool(auto_dir_result.get("ok")),
            "selected_direction": auto_dir_result.get("selected_direction"),
            "confidence": auto_dir_result.get("confidence"),
            "persisted": auto_dir_result.get("persisted"),
            "summary": auto_dir_result.get("summary"),
        },
    )
    if not bool(auto_dir_result.get("ok")):
        status = _status_bundle(axis, kv_est=args.kv_est, line_line_r_ohm=args.line_line_r_ohm)
        return "guided-bringup", status, "Guided bring-up failed at auto direction", {
            "ok": False,
            "failed_stage": "auto_direction_contract",
            "stages": stages,
        }

    direction_validation = _run_direction_validation_contract(
        axis,
        cycles=max(2, int(getattr(args, "cycles", 4) or 4)),
        cmd_delta_turns=float(getattr(args, "cmd_delta_turns", 0.01) or 0.01),
        current_lim=max(6.0, min(float(getattr(axis.motor.config, "current_lim", 8.0)), 10.0)),
    )
    add_stage(
        "direction_validation_contract",
        bool(direction_validation.get("ok")),
        (
            "Direction validation passed."
            if bool(direction_validation.get("ok"))
            else "Selected direction failed the stronger post-selection sign validation."
        ),
        details={
            "ok": bool(direction_validation.get("ok")),
            "classification": direction_validation.get("classification"),
            "interpretation": direction_validation.get("interpretation"),
            "summary": direction_validation.get("summary"),
            "config": {
                "cycles": direction_validation.get("cycles"),
                "cmd_delta_turns": direction_validation.get("cmd_delta_turns"),
                "current_lim": direction_validation.get("current_lim"),
            },
        },
    )
    if not bool(direction_validation.get("ok")):
        status = _status_bundle(axis, kv_est=args.kv_est, line_line_r_ohm=args.line_line_r_ohm)
        return "guided-bringup", status, "Guided bring-up failed at direction validation", {
            "ok": False,
            "failed_stage": "direction_validation_contract",
            "stages": stages,
        }

    sign_probe = common.position_sign_probe(
        axis=axis,
        step_turns=max(0.02, float(getattr(args, "cmd_delta_turns", 0.01) or 0.01) * 3.0),
        hold_s=0.20,
        dt=0.01,
        current_lim=min(6.0, max(4.0, float(getattr(axis.motor.config, "current_lim", 6.0)))),
        pos_gain=10.0,
        vel_gain=0.18,
        vel_i_gain=0.0,
        vel_limit=0.35,
        verbose=False,
    )
    add_stage(
        "position_sign_probe",
        bool(sign_probe.get("ok")),
        ("Position sign probe passed." if bool(sign_probe.get("ok")) else str(sign_probe.get("classification") or "Position sign probe failed.")),
        details={
            "ok": bool(sign_probe.get("ok")),
            "classification": sign_probe.get("classification"),
            "motor_direction": sign_probe.get("motor_direction"),
            "step_count": len(list(sign_probe.get("steps") or [])),
            "steps": sign_probe.get("steps"),
        },
    )
    if not bool(sign_probe.get("ok")):
        status = _status_bundle(axis, kv_est=args.kv_est, line_line_r_ohm=args.line_line_r_ohm)
        return "guided-bringup", status, "Guided bring-up failed at position sign probe", {
            "ok": False,
            "failed_stage": "position_sign_probe",
            "stages": stages,
        }

    persisted = False
    persist_error = None
    if bool(getattr(args, "persist", False)):
        try:
            _save_axis_configuration(axis)
            persisted = True
        except Exception as exc:
            persist_error = str(exc)
            add_stage(
                "persist_detected_direction",
                False,
                "Direction passed validation but could not be saved.",
                details={"error": persist_error},
            )
            status = _status_bundle(axis, kv_est=args.kv_est, line_line_r_ohm=args.line_line_r_ohm)
            return "guided-bringup", status, "Guided bring-up failed while saving detected direction", {
                "ok": False,
                "failed_stage": "persist_detected_direction",
                "stages": stages,
            }
        add_stage(
            "persist_detected_direction",
            True,
            "Detected direction saved to controller configuration.",
            details={"persisted": True},
        )

    status = _status_bundle(axis, kv_est=args.kv_est, line_line_r_ohm=args.line_line_r_ohm)
    return "guided-bringup", status, "Guided bring-up passed", {
        "ok": True,
        "failed_stage": None,
        "persisted": bool(persisted),
        "persist_error": persist_error,
        "stages": stages,
    }


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
        fail_to_idle_override=(True if bool(getattr(args, "release_after_move", False)) else None),
    )
    status = _status_bundle(axis, kv_est=args.kv_est, line_line_r_ohm=args.line_line_r_ohm)
    return "move-continuous", status, "Continuous move completed", {"move": result}


def _handle_move_two_axes_synced(
    args: argparse.Namespace,
) -> tuple[str, dict[str, Any], dict[str, Any], str, dict[str, Any] | None]:
    axis_a_index = int(args.axis_a_index)
    axis_b_index = int(args.axis_b_index)
    serial_a = str(getattr(args, "serial_a", "") or "").strip() or None
    serial_b = str(getattr(args, "serial_b", "") or "").strip() or serial_a
    if axis_a_index == axis_b_index and serial_a == serial_b:
        raise ValueError("Axis A and Axis B must be different for a synced move unless they target different board serials")

    odrv_a, axis_a = _connect(
        axis_index=axis_a_index,
        timeout_s=float(args.connect_timeout_s),
        serial_number=serial_a,
    )
    if serial_b == serial_a:
        odrv_b = odrv_a
        try:
            axis_b = getattr(odrv_b, f"axis{axis_b_index}")
        except AttributeError as exc:
            raise RuntimeError(f"Device does not expose axis{axis_b_index}") from exc
    else:
        odrv_b, axis_b = _connect(
            axis_index=axis_b_index,
            timeout_s=float(args.connect_timeout_s),
            serial_number=serial_b,
        )

    profile_name_a = str(getattr(args, "profile_a_name", None) or args.profile_name)
    profile_name_b = str(getattr(args, "profile_b_name", None) or args.profile_name)
    cfg_a = _load_continuous_move_kwargs(profile_name=profile_name_a)
    cfg_b = _load_continuous_move_kwargs(profile_name=profile_name_b)
    if bool(getattr(args, "release_after_move", False)):
        cfg_a["fail_to_idle"] = True
        cfg_b["fail_to_idle"] = True

    _apply_continuous_profile(axis_a, cfg_a)
    _apply_continuous_profile(axis_b, cfg_b)

    tgt_a = _target_turns_motor_for_angle(
        axis_a,
        angle_deg=float(args.angle_a_deg),
        angle_space=str(args.angle_space),
        gear_ratio=float(args.gear_ratio_a),
        zero_turns_motor=(None if args.zero_a_turns_motor is None else float(args.zero_a_turns_motor)),
        relative_to_current=bool(args.relative_to_current),
    )
    tgt_b = _target_turns_motor_for_angle(
        axis_b,
        angle_deg=float(args.angle_b_deg),
        angle_space=str(args.angle_space),
        gear_ratio=float(args.gear_ratio_b),
        zero_turns_motor=(None if args.zero_b_turns_motor is None else float(args.zero_b_turns_motor)),
        relative_to_current=bool(args.relative_to_current),
    )

    if args.timeout_s is not None:
        timeout_s = float(args.timeout_s)
    else:
        move_dist_a = abs(float(tgt_a["target_turns_motor"]) - float(tgt_a["start_turns_motor"]))
        move_dist_b = abs(float(tgt_b["target_turns_motor"]) - float(tgt_b["start_turns_motor"]))
        est_total = max(
            _trap_move_time_est(move_dist_a, float(cfg_a["trap_vel"]), float(cfg_a["trap_acc"]), float(cfg_a["trap_dec"])),
            _trap_move_time_est(move_dist_b, float(cfg_b["trap_vel"]), float(cfg_b["trap_acc"]), float(cfg_b["trap_dec"])),
        ) + max(float(cfg_a["settle_s"]), float(cfg_b["settle_s"])) + 0.75
        timeout_s = max(float(cfg_a["timeout_s"]), float(cfg_b["timeout_s"]), float(est_total))

    move_result = common.move_axes_absolute_synced(
        [
            {
                "axis": axis_a,
                "target": float(tgt_a["target_turns_motor"]),
                "name": f"axis{axis_a_index}",
                "use_trap_traj": True,
                "trap_vel": float(cfg_a["trap_vel"]),
                "trap_acc": float(cfg_a["trap_acc"]),
                "trap_dec": float(cfg_a["trap_dec"]),
                "max_error_turns": float(cfg_a["target_tolerance_turns"]),
                "max_vel_turns_s": float(cfg_a["target_vel_tolerance_turns_s"]),
            },
            {
                "axis": axis_b,
                "target": float(tgt_b["target_turns_motor"]),
                "name": f"axis{axis_b_index}",
                "use_trap_traj": True,
                "trap_vel": float(cfg_b["trap_vel"]),
                "trap_acc": float(cfg_b["trap_acc"]),
                "trap_dec": float(cfg_b["trap_dec"]),
                "max_error_turns": float(cfg_b["target_tolerance_turns"]),
                "max_vel_turns_s": float(cfg_b["target_vel_tolerance_turns_s"]),
            },
        ],
        use_trap_traj=True,
        require_absolute=False,
        require_index=False,
        trap_vel=max(float(cfg_a["trap_vel"]), float(cfg_b["trap_vel"])),
        trap_acc=max(float(cfg_a["trap_acc"]), float(cfg_b["trap_acc"])),
        trap_dec=max(float(cfg_a["trap_dec"]), float(cfg_b["trap_dec"])),
        timeout_s=float(timeout_s),
        settle_s=max(float(cfg_a["settle_s"]), float(cfg_b["settle_s"])),
        max_error_turns=max(float(cfg_a["target_tolerance_turns"]), float(cfg_b["target_tolerance_turns"])),
        max_vel_turns_s=max(float(cfg_a["target_vel_tolerance_turns_s"]), float(cfg_b["target_vel_tolerance_turns_s"])),
    )

    if bool(getattr(args, "release_after_move", False)):
        try:
            axis_a.requested_state = common.AXIS_STATE_IDLE
        except Exception:
            pass
        try:
            axis_b.requested_state = common.AXIS_STATE_IDLE
        except Exception:
            pass
        time.sleep(max(0.05, float(args.settle_s)))

    status = _status_bundle(axis_a, kv_est=args.kv_est, line_line_r_ohm=args.line_line_r_ohm)
    device = _device_info(odrv_a, axis_a_index)
    return "move-two-axes-synced", device, status, "Synced two-axis move completed", {
        "profile_name": str(args.profile_name),
        "profile_a_name": profile_name_a,
        "profile_b_name": profile_name_b,
        "angle_space": str(args.angle_space),
        "release_after_move": bool(getattr(args, "release_after_move", False)),
        "axis_a": {
            "axis_index": axis_a_index,
            "serial_number": _clean_json(getattr(odrv_a, "serial_number", None)),
            "angle_deg": float(args.angle_a_deg),
            "gear_ratio": float(args.gear_ratio_a),
            "start_turns_motor": float(tgt_a["start_turns_motor"]),
            "zero_turns_motor": float(tgt_a["zero_turns_motor"]),
            "target_turns_motor": float(tgt_a["target_turns_motor"]),
        },
        "axis_b": {
            "axis_index": axis_b_index,
            "serial_number": _clean_json(getattr(odrv_b, "serial_number", None)),
            "angle_deg": float(args.angle_b_deg),
            "gear_ratio": float(args.gear_ratio_b),
            "start_turns_motor": float(tgt_b["start_turns_motor"]),
            "zero_turns_motor": float(tgt_b["zero_turns_motor"]),
            "target_turns_motor": float(tgt_b["target_turns_motor"]),
        },
        "move": move_result,
    }


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
        "discover-boards",
        "status",
        "diagnose",
        "fact-sheet",
        "set-motor-direction",
        "auto-direction-contract",
        "guided-bringup",
        "idle",
        "clear-errors",
        "startup",
        "move-continuous",
        "move-two-axes-synced",
        "move-continuous-async",
        "motion-status",
        "follow-angle",
        "telemetry",
        "stream-subscribe",
        "stream-unsubscribe",
        "profiles",
        "profile-config",
        "save-profile",
        "serve",
    ])
    parser.add_argument("--axis-index", type=int, default=0)
    parser.add_argument("--serial-number")
    parser.add_argument("--connect-timeout-s", type=float, default=DEFAULT_CONNECT_TIMEOUT_S)
    parser.add_argument("--kv-est", type=float, default=DEFAULT_KV_EST)
    parser.add_argument("--line-line-r-ohm", type=float, default=DEFAULT_LINE_LINE_R_OHM)
    parser.add_argument("--settle-s", type=float, default=0.15)
    parser.add_argument("--timeout-s", type=float)
    parser.add_argument("--debug", action="store_true")

    parser.add_argument("--angle-deg", type=float)
    parser.add_argument("--angle-a-deg", type=float)
    parser.add_argument("--angle-b-deg", type=float)
    parser.add_argument("--angle-space", choices=["gearbox_output", "motor"], default="gearbox_output")
    parser.add_argument("--profile-name", default=DEFAULT_PROFILE)
    parser.add_argument("--profile-a-name")
    parser.add_argument("--profile-b-name")
    parser.add_argument("--gear-ratio", type=float, default=DEFAULT_GEAR_RATIO)
    parser.add_argument("--gear-ratio-a", type=float, default=DEFAULT_GEAR_RATIO)
    parser.add_argument("--gear-ratio-b", type=float, default=DEFAULT_GEAR_RATIO)
    parser.add_argument("--zero-turns-motor", type=float)
    parser.add_argument("--zero-a-turns-motor", type=float)
    parser.add_argument("--zero-b-turns-motor", type=float)
    parser.add_argument("--axis-a-index", type=int, default=0)
    parser.add_argument("--axis-b-index", type=int, default=1)
    parser.add_argument("--serial-a")
    parser.add_argument("--serial-b")
    parser.add_argument("--relative-to-current", action="store_true")
    parser.add_argument("--release-after-move", action="store_true")
    parser.add_argument("--interval-ms", type=int, default=40)
    parser.add_argument("--profile-json")
    parser.add_argument("--direction", type=int)
    parser.add_argument("--cycles", type=int, default=4)
    parser.add_argument("--cmd-delta-turns", type=float, default=0.01)
    parser.add_argument("--persist", action="store_true")
    return parser


def _parse_request_args(action: str, arguments: list[str]) -> argparse.Namespace:
    parser = _parser(exit_on_error=False)
    try:
        args = parser.parse_args([str(action), *[str(item) for item in arguments]])
    except Exception as exc:
        raise ValueError(str(exc)) from exc

    if args.action in {"move-continuous", "move-continuous-async", "follow-angle"} and args.angle_deg is None:
        raise ValueError("--angle-deg is required for move-continuous/move-continuous-async/follow-angle")
    if args.action == "move-two-axes-synced" and (args.angle_a_deg is None or args.angle_b_deg is None):
        raise ValueError("--angle-a-deg and --angle-b-deg are required for move-two-axes-synced")
    if args.action == "set-motor-direction" and args.direction is None:
        raise ValueError("--direction is required for set-motor-direction")
    if args.action == "profile-config" and not str(args.profile_name or "").strip():
        raise ValueError("--profile-name is required for profile-config")
    if args.action == "save-profile" and not str(args.profile_json or "").strip():
        raise ValueError("--profile-json is required for save-profile")
    return args


def _execute_action(args: argparse.Namespace, odrv: Any, axis: Any, device: dict[str, Any]) -> tuple[int, dict[str, Any]]:
    if args.action == "status":
        action, status, message, result = _handle_status(axis, args)
    elif args.action == "diagnose":
        action, status, message, result = _handle_diagnose(axis, args)
    elif args.action == "fact-sheet":
        action, status, message, result = _handle_fact_sheet(axis, args)
    elif args.action == "set-motor-direction":
        action, status, message, result = _handle_set_motor_direction(axis, args)
    elif args.action == "auto-direction-contract":
        action, status, message, result = _handle_auto_direction_contract(axis, args)
    elif args.action == "guided-bringup":
        action, status, message, result = _handle_guided_bringup(axis, args)
    elif args.action == "idle":
        action, status, message, result = _handle_idle(axis, args)
    elif args.action == "clear-errors":
        action, status, message, result = _handle_clear_errors(axis, args)
    elif args.action == "startup":
        action, status, message, result = _handle_startup(axis, args)
    elif args.action == "move-continuous":
        action, status, message, result = _handle_move_continuous(axis, args)
    elif args.action == "move-two-axes-synced":
        action, device, status, message, result = _handle_move_two_axes_synced(args)
        payload = _result_envelope(
            ok=True,
            action=action,
            device=device,
            message=message,
            status=status,
            result=result,
            include_catalog=True,
        )
        return 0, payload
    elif args.action == "move-continuous-async":
        raise RuntimeError("move-continuous-async is only supported in serve mode")
    elif args.action == "motion-status":
        raise RuntimeError("motion-status is only supported in serve mode")
    elif args.action == "follow-angle":
        action, status, message, result = _handle_follow_angle(axis, args)
    elif args.action == "telemetry":
        action, status, message, result = _handle_telemetry(axis, args)
    elif args.action in {"stream-subscribe", "stream-unsubscribe"}:
        raise RuntimeError(f"{args.action} is only supported in serve mode")
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
        include_catalog=(action != "telemetry"),
    )
    return (0 if ok else 2), payload


class _PersistentServer:
    def __init__(self, emit_payload):
        self._odrv = None
        self._axis = None
        self._device = None
        self._axis_index = None
        self._serial_number = None
        self._emit_payload = emit_payload
        self._motion_lock = threading.Lock()
        self._motion_thread = None
        self._motion_latest_status = None
        self._motion_latest_result = None
        self._motion_error = None
        self._motion_started_s = None
        self._motion_completed_s = None
        self._stream_lock = threading.Lock()
        self._stream_enabled = False
        self._stream_interval_s = 0.04
        self._stream_args = None
        self._stream_stop = threading.Event()
        self._stream_thread = threading.Thread(
            target=self._stream_loop,
            name="focui-telemetry-stream",
            daemon=True,
        )
        self._stream_thread.start()

    def stop(self) -> None:
        self._stream_stop.set()
        try:
            if self._stream_thread.is_alive():
                self._stream_thread.join(timeout=0.5)
        except Exception:
            pass

    def _reset_connection(self) -> None:
        self._odrv = None
        self._axis = None
        self._device = None
        self._axis_index = None
        self._serial_number = None

    def _ensure_connection(self, args: argparse.Namespace) -> tuple[Any, Any, dict[str, Any]]:
        axis_index = int(args.axis_index)
        serial_number = str(getattr(args, "serial_number", "") or "").strip() or None
        if (
            self._axis is not None
            and self._odrv is not None
            and self._axis_index == axis_index
            and self._serial_number == serial_number
        ):
            return self._odrv, self._axis, dict(self._device or {})
        self._odrv, self._axis = _connect(
            axis_index=axis_index,
            timeout_s=float(args.connect_timeout_s),
            serial_number=serial_number,
        )
        self._axis_index = axis_index
        self._serial_number = serial_number
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

    def _emit_event(self, *, action: str, message: str, status: dict[str, Any] | None = None,
                    result: dict[str, Any] | None = None, error: dict[str, Any] | None = None) -> None:
        payload = _result_envelope(
            ok=(error is None),
            action=action,
            device=_clean_json(self._device),
            message=message,
            status=status,
            result=result,
            error=error,
            request_id=None,
            include_catalog=False,
        )
        self._emit_payload(payload)

    def _emit_graph_event(self, *, graph_sample: dict[str, Any]) -> None:
        payload = {
            "ok": True,
            "action": "stream-graph",
            "request_id": None,
            "timestamp_s": time.time(),
            "device": None,
            "message": None,
            "snapshot": None,
            "diagnosis": None,
            "fact_sheet": None,
            "capabilities": None,
            "available_profiles": None,
            "available_profile_details": None,
            "profile_editor": None,
            "graph_sample": _clean_json(graph_sample),
            "result": None,
            "error": None,
        }
        self._emit_payload(payload)

    def _stream_enabled_now(self) -> bool:
        with self._stream_lock:
            return bool(self._stream_enabled)

    def _stream_loop(self) -> None:
        while not self._stream_stop.is_set():
            with self._stream_lock:
                enabled = bool(self._stream_enabled)
                interval_s = float(self._stream_interval_s)
                args = self._stream_args

            if not enabled:
                self._stream_stop.wait(0.05)
                continue

            if args is None:
                self._stream_stop.wait(0.05)
                continue

            if self._motion_active():
                self._stream_stop.wait(0.02)
                continue

            try:
                _, axis, _ = self._ensure_connection(args)
                status = _mark_motion_capabilities(_telemetry_bundle(axis), motion_active=False)
                self._emit_graph_event(graph_sample=_telemetry_result_from_status(status))
            except Exception as exc:
                self._emit_event(
                    action="stream-telemetry",
                    message=str(exc),
                    status=None,
                    result=None,
                    error={"type": exc.__class__.__name__, "message": str(exc)},
                )
                self._reset_connection()
                self._stream_stop.wait(max(0.2, interval_s))
                continue

            self._stream_stop.wait(max(0.02, interval_s))

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
            "available_profiles": None,
            "available_profile_details": None,
        }
        with self._motion_lock:
            self._motion_latest_status = status
        if self._stream_enabled_now():
            self._emit_graph_event(graph_sample=_telemetry_result_from_status(status))

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
                fail_to_idle_override=(True if bool(getattr(args, "release_after_move", False)) else None),
                sample_hook=self._publish_motion_sample,
            )
            final_status = _status_bundle(axis, kv_est=args.kv_est, line_line_r_ohm=args.line_line_r_ohm)
            final_status = _mark_motion_capabilities(final_status, motion_active=False)
            with self._motion_lock:
                self._motion_latest_status = final_status
                self._motion_latest_result = {"move": _clean_json(result)}
                self._motion_error = None
                self._motion_completed_s = time.time()
            if self._stream_enabled_now():
                self._emit_event(
                    action="stream-motion-status",
                    message="Background move completed",
                    status=final_status,
                    result=self._motion_status_payload(),
                )
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
            if self._stream_enabled_now():
                self._emit_event(
                    action="stream-motion-status",
                    message=str(exc),
                    status=status,
                    result=self._motion_status_payload(),
                    error={"type": exc.__class__.__name__, "message": str(exc)},
                )

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

            if parsed_args.action == "discover-boards":
                devices = _discover_runtime_devices()
                return _result_envelope(
                    ok=True,
                    action="discover-boards",
                    device=None,
                    message=f"Discovered {len(devices)} ODrive runtime device(s)",
                    status=None,
                    result={"devices": devices},
                    request_id=request_id,
                )

            if parsed_args.action == "profile-config":
                editor = _continuous_profile_editor_payload(str(parsed_args.profile_name))
                payload = _result_envelope(
                    ok=True,
                    action="profile-config",
                    device=None,
                    message=f"Loaded profile editor for {parsed_args.profile_name}",
                    status=None,
                    profile_editor=editor,
                    result={"profile_name": str(parsed_args.profile_name)},
                    request_id=request_id,
                )
                return payload

            if parsed_args.action == "save-profile":
                try:
                    profile_payload = json.loads(str(parsed_args.profile_json))
                except Exception as exc:
                    raise ValueError(f"Invalid --profile-json payload: {exc}") from exc
                if not isinstance(profile_payload, dict):
                    raise ValueError("--profile-json must decode to an object")
                saved = _save_continuous_profile_editor_payload(profile_payload)
                payload = _result_envelope(
                    ok=True,
                    action="save-profile",
                    device=None,
                    message=f"Saved continuous profile {saved['profile_name']}",
                    status=None,
                    profile_editor=saved.get("editor"),
                    result={
                        "profile_name": saved.get("profile_name"),
                        "profiles_path": saved.get("profiles_path"),
                        "record": saved.get("record"),
                    },
                    request_id=request_id,
                )
                return payload

            if parsed_args.action == "stream-subscribe":
                _, axis, device = self._ensure_connection(parsed_args)
                status = _mark_motion_capabilities(
                    _telemetry_bundle(axis),
                    motion_active=self._motion_active(),
                )
                with self._stream_lock:
                    self._stream_enabled = True
                    self._stream_interval_s = max(0.02, float(parsed_args.interval_ms) / 1000.0)
                    self._stream_args = parsed_args
                return _result_envelope(
                    ok=True,
                    action="stream-subscribe",
                    device=device,
                    message="Telemetry stream enabled",
                    status=status,
                    result={"streaming": True, "interval_ms": int(parsed_args.interval_ms)},
                    request_id=request_id,
                )

            if parsed_args.action == "stream-unsubscribe":
                with self._stream_lock:
                    self._stream_enabled = False
                status = None
                if self._axis is not None:
                    status = _mark_motion_capabilities(
                        _telemetry_bundle(self._axis),
                        motion_active=self._motion_active(),
                    )
                return _result_envelope(
                    ok=True,
                    action="stream-unsubscribe",
                    device=_clean_json(self._device),
                    message="Telemetry stream disabled",
                    status=status,
                    result={"streaming": False},
                    request_id=request_id,
                )

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
                    include_catalog=False,
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
                    include_catalog=False,
                )

            if self._motion_active() and parsed_args.action not in {"telemetry", "motion-status"}:
                raise RuntimeError("Background continuous move is active. Wait for completion before running this action.")

            if parsed_args.action == "move-two-axes-synced":
                action_name, device, status, message, result = _handle_move_two_axes_synced(parsed_args)
                payload = _result_envelope(
                    ok=True,
                    action=action_name,
                    device=device,
                    message=message,
                    status=status,
                    result=result,
                    request_id=request_id,
                    include_catalog=True,
                )
                if self._motion_active():
                    payload = _mark_motion_capabilities(payload, motion_active=True)
                return payload

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
    emit_lock = threading.Lock()

    def _emit_payload(payload: dict[str, Any]) -> None:
        text = _json_text(payload, pretty=False)
        with emit_lock:
            print(text, flush=True)

    server = _PersistentServer(_emit_payload)
    ready_payload = _result_envelope(
        ok=True,
        action="serve",
        device=None,
        message="Backend server ready",
        result={"server_ready": True, "pid": int(getattr(os, "getpid")())},
    )
    _emit_payload(ready_payload)

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
            _emit_payload(payload)
            continue

        payload = server.handle_request(request)
        _emit_payload(payload)
        if str(request.get("action") or "") == "shutdown":
            break
    server.stop()
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

    if args.action == "profile-config":
        try:
            args = _parse_request_args(args.action, sys.argv[2:])
        except Exception as exc:
            parser.error(str(exc))
        payload = _result_envelope(
            ok=True,
            action="profile-config",
            device=None,
            message=f"Loaded profile editor for {args.profile_name}",
            status=None,
            profile_editor=_continuous_profile_editor_payload(str(args.profile_name)),
            result={"profile_name": str(args.profile_name)},
        )
        print(_json_text(payload, pretty=True))
        return 0

    if args.action == "save-profile":
        try:
            args = _parse_request_args(args.action, sys.argv[2:])
        except Exception as exc:
            parser.error(str(exc))
        try:
            profile_payload = json.loads(str(args.profile_json))
        except Exception as exc:
            parser.error(f"Invalid --profile-json payload: {exc}")
        if not isinstance(profile_payload, dict):
            parser.error("--profile-json must decode to an object")
        saved = _save_continuous_profile_editor_payload(profile_payload)
        payload = _result_envelope(
            ok=True,
            action="save-profile",
            device=None,
            message=f"Saved continuous profile {saved['profile_name']}",
            status=None,
            profile_editor=saved.get("editor"),
            result={
                "profile_name": saved.get("profile_name"),
                "profiles_path": saved.get("profiles_path"),
                "record": saved.get("record"),
            },
        )
        print(_json_text(payload, pretty=True))
        return 0

    if args.action == "discover-boards":
        try:
            args = _parse_request_args(args.action, sys.argv[2:])
        except Exception as exc:
            parser.error(str(exc))
        devices = _discover_runtime_devices()
        payload = _result_envelope(
            ok=True,
            action="discover-boards",
            device=None,
            message=f"Discovered {len(devices)} ODrive runtime device(s)",
            status=None,
            result={"devices": devices},
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
        if args.action == "move-two-axes-synced":
            action_name, device, status, message, result = _handle_move_two_axes_synced(args)
            payload = _result_envelope(
                ok=True,
                action=action_name,
                device=device,
                message=message,
                status=status,
                result=result,
                include_catalog=True,
            )
            exit_code = 0
        else:
            odrv, axis = _connect(
                axis_index=int(args.axis_index),
                timeout_s=float(args.connect_timeout_s),
                serial_number=(str(getattr(args, "serial_number", "") or "").strip() or None),
            )
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
