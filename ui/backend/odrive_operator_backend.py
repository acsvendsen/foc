#!/usr/bin/env python3
from __future__ import annotations

import argparse
import contextlib
import io
import json
import math
import sys
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
    snapshot = dict((diagnosis.get("report") or {}).get("snapshot") or {})
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


def _result_envelope(*, ok: bool, action: str, device: dict[str, Any] | None = None, message: str | None = None,
                     status: dict[str, Any] | None = None, result: dict[str, Any] | None = None,
                     error: dict[str, Any] | None = None) -> dict[str, Any]:
    out = {
        "ok": bool(ok),
        "action": str(action),
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


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Thin JSON backend for the Robot operator console")
    parser.add_argument("action", choices=[
        "status",
        "diagnose",
        "fact-sheet",
        "idle",
        "clear-errors",
        "startup",
        "move-continuous",
        "profiles",
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


def main() -> int:
    parser = _parser()
    args = parser.parse_args()

    if args.action == "profiles":
        payload = _result_envelope(
            ok=True,
            action="profiles",
            device=None,
            message="Loaded continuous-move profiles",
            status=None,
            result={"profiles": _continuous_profile_records()},
        )
        print(json.dumps(payload, indent=2, sort_keys=False))
        return 0

    if args.action == "move-continuous" and args.angle_deg is None:
        parser.error("--angle-deg is required for move-continuous")

    odrv = None
    axis = None
    device = None

    try:
        odrv, axis = _connect(axis_index=int(args.axis_index), timeout_s=float(args.connect_timeout_s))
        device = _device_info(odrv, int(args.axis_index))

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
        print(json.dumps(payload, indent=2, sort_keys=False))
        return 0 if ok else 2

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
        print(json.dumps(payload, indent=2, sort_keys=False))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
