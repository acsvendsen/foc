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
from mks_mounted_directional_move import run_directional_move, run_directional_slew_move, run_directional_velocity_travel_move, run_direct_move, run_velocity_point_to_point_move  # type: ignore
from mks_apply_runtime_baseline import apply_runtime_baseline  # type: ignore
try:
    from output_sensor_bridge import get_output_sensor_snapshot_from_env  # type: ignore
except Exception as _output_sensor_import_exc:  # pragma: no cover - optional dependency path
    _OUTPUT_SENSOR_IMPORT_ERROR = str(_output_sensor_import_exc)

    def get_output_sensor_snapshot_from_env(*, axis_motor_turns: float | None = None, gear_ratio: float | None = None):  # type: ignore
        return None
else:
    _OUTPUT_SENSOR_IMPORT_ERROR = None

DEFAULT_KV_EST = 140.0
DEFAULT_LINE_LINE_R_OHM = 0.30
DEFAULT_GEAR_RATIO = 25.0
DEFAULT_PROFILE = "gearbox_output_continuous_quiet_20260309"
DEFAULT_MKS_STARTUP_PROFILE = "mks_mounted_direct_preload_v3"
DEFAULT_CONNECT_TIMEOUT_S = 3.0
MAX_STAGE_LOG_CHARS = 6000


class MoveResultError(RuntimeError):
    def __init__(self, message: str, *, result: dict[str, Any] | None = None) -> None:
        super().__init__(message)
        self.result = result


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
    raw = str(serial_number or "").strip()
    queries: list[str | None] = [None] if not raw else [raw]
    upper = raw.upper()
    try:
        if raw:
            if raw.isdigit():
                queries.append(format(int(raw), "X"))
            elif all(ch in "0123456789ABCDEF" for ch in upper):
                queries.append(str(int(upper, 16)))
    except Exception:
        pass
    seen: set[str | None] = set()
    errors: list[str] = []
    odrv = None
    for query in queries:
        key = None if query is None else str(query).strip()
        if key in seen:
            continue
        seen.add(key)
        try:
            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                odrv = odrive.find_any(serial_number=query, timeout=float(timeout_s))
            break
        except Exception as exc:
            errors.append(f"query={query!r} error={exc!r}")
    if odrv is None:
        msg = (
            f"ODrive connection failed within {float(timeout_s):.2f}s. "
            "The device may be disconnected or already in use by another program."
        )
        if errors:
            msg += f" attempts=[{' ; '.join(errors)}]"
        raise RuntimeError(msg)
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


def _output_sensor_payload(
    *,
    axis_motor_turns: float | None = None,
    gear_ratio: float | None = None,
) -> dict[str, Any] | None:
    configured_port = str(os.getenv("ROBOT_OUTPUT_SENSOR_PORT", "") or "").strip()
    if not configured_port and _OUTPUT_SENSOR_IMPORT_ERROR is None:
        return None
    if _OUTPUT_SENSOR_IMPORT_ERROR is not None:
        if not configured_port:
            return None
        return {
            "configured": True,
            "connected": False,
            "healthy": False,
            "port": configured_port,
            "baudrate": _clean_json(os.getenv("ROBOT_OUTPUT_SENSOR_BAUD")),
            "last_error": f"output sensor bridge import failed: {_OUTPUT_SENSOR_IMPORT_ERROR}",
        }
    try:
        payload = get_output_sensor_snapshot_from_env(
            axis_motor_turns=axis_motor_turns,
            gear_ratio=(float(gear_ratio) if gear_ratio is not None else float(DEFAULT_GEAR_RATIO)),
        )
    except Exception as exc:
        return {
            "configured": bool(configured_port),
            "connected": False,
            "healthy": False,
            "port": configured_port,
            "baudrate": _clean_json(os.getenv("ROBOT_OUTPUT_SENSOR_BAUD")),
            "last_error": str(exc),
        }
    return _clean_json(payload)


def _continuous_profiles() -> list[str]:
    return [row["name"] for row in _continuous_profile_records()]


def _builtin_continuous_profiles() -> dict[str, dict[str, Any]]:
    return {
        "mks_bare_direct_trusted_v1": {
            "profile_name": "mks_bare_direct_trusted_v1",
            "load_mode": "mks_direct_position",
            "source": "codex_builtin_mks_profile",
            "experimental": True,
            "foundation_validated": False,
            "notes": (
                "Experimental cautious bare-motor direct-position characterization preset for short one-shot moves. "
                "This is not a validated precision foundation and not a trap/operator profile."
            ),
            "require_repeatability": False,
            "stop_on_frame_jump": True,
            "stop_on_hard_fault": True,
            "suite_kwargs": {
                "current_lim": 2.50,
                "enable_overspeed_error": False,
                "pos_gain": 4.75,
                "vel_gain": 0.10,
                "vel_i_gain": 0.02,
                "trap_vel": 0.45,
                "trap_acc": 0.45,
                "trap_dec": 0.45,
                "vel_limit": 0.45,
                "vel_limit_tolerance": 4.0,
                "stiction_kick_nm": 0.0,
                "step_kwargs": {
                    "target_tolerance_turns": 0.03,
                    "target_vel_tolerance_turns_s": 0.20,
                },
            },
            "continuous_kwargs": {
                "move_mode": "mks_direct_position",
                "candidate_preset": "bare-pos-trusted-v1",
                "reuse_existing_calibration": True,
                "live_follow_supported": False,
                "final_hold_s": 0.90,
                "abort_abs_turns": 0.60,
                "timeout_s": 8.0,
                "min_delta_turns": 0.0015,
                "settle_s": 0.08,
                "quiet_hold_enable": False,
                "quiet_hold_s": 0.0,
                "quiet_hold_pos_gain_scale": 1.0,
                "quiet_hold_vel_gain_scale": 1.0,
                "quiet_hold_vel_i_gain": 0.0,
                "quiet_hold_vel_limit_scale": 1.0,
                "quiet_hold_persist": False,
                "quiet_hold_reanchor_err_turns": None,
                "fail_to_idle": True,
            },
            "validated_targets_deg": [],
            "limitations": [
                "Bare-motor / unmounted-gearbox characterization only.",
                "Direct-position path only; trap/operator move path is not validated here.",
                "Best suited to short cautious moves; it is intentionally soft on longer travel.",
                "Experimental: this direct-control family can shake/hunt on multiple boards.",
                "Live follow is disabled for this profile.",
            ],
        },
        "mks_bare_direct_smooth_v1": {
            "profile_name": "mks_bare_direct_smooth_v1",
            "load_mode": "mks_direct_position",
            "source": "codex_builtin_mks_profile",
            "experimental": True,
            "foundation_validated": False,
            "notes": (
                "Experimental smoother bare-motor direct-position preset for longer one-shot moves. "
                "This backs off position stiffness to reduce the soft travel-hunting wave, but it is not a validated precision foundation."
            ),
            "require_repeatability": False,
            "stop_on_frame_jump": True,
            "stop_on_hard_fault": True,
            "suite_kwargs": {
                "current_lim": 6.0,
                "enable_overspeed_error": False,
                "pos_gain": 3.75,
                "vel_gain": 0.28,
                "vel_i_gain": 0.0,
                "trap_vel": 1.0,
                "trap_acc": 1.0,
                "trap_dec": 1.0,
                "vel_limit": 1.0,
                "vel_limit_tolerance": 4.0,
                "stiction_kick_nm": 0.0,
                "step_kwargs": {
                    "target_tolerance_turns": 0.03,
                    "target_vel_tolerance_turns_s": 0.20,
                },
            },
            "continuous_kwargs": {
                "move_mode": "mks_direct_position",
                "candidate_preset": "bare-direct-smooth-v1",
                "reuse_existing_calibration": True,
                "live_follow_supported": False,
                "final_hold_s": 1.00,
                "abort_abs_turns": 2.50,
                "timeout_s": 10.0,
                "min_delta_turns": 0.0015,
                "settle_s": 0.10,
                "quiet_hold_enable": False,
                "quiet_hold_s": 0.0,
                "quiet_hold_pos_gain_scale": 1.0,
                "quiet_hold_vel_gain_scale": 1.0,
                "quiet_hold_vel_i_gain": 0.0,
                "quiet_hold_vel_limit_scale": 1.0,
                "quiet_hold_persist": False,
                "quiet_hold_reanchor_err_turns": None,
                "fail_to_idle": True,
            },
            "validated_targets_deg": [],
            "limitations": [
                "Bare-motor / unmounted-gearbox path only.",
                "Direct-position only; trap/operator move path is not validated here.",
                "Designed for smoother longer one-shot moves, not maximum stiffness.",
                "Experimental: the shared direct-control method can still shake/hunt.",
                "Live follow is disabled for this profile.",
            ],
        },
        "mks_mounted_direct_preload_v3": {
            "profile_name": "mks_mounted_direct_preload_v3",
            "load_mode": "mks_direct_position",
            "source": "codex_builtin_mks_profile",
            "experimental": True,
            "foundation_validated": False,
            "notes": (
                "Experimental higher-authority mounted direct-position preset with directional preload. "
                "Positive deltas approach from above, negative deltas from below. "
                "Use this only for characterization; longer moves can excite a soft wave and it is not a validated precision foundation."
            ),
            "require_repeatability": False,
            "stop_on_frame_jump": True,
            "stop_on_hard_fault": True,
            "suite_kwargs": {
                "current_lim": 6.0,
                "enable_overspeed_error": False,
                "pos_gain": 4.75,
                "vel_gain": 0.30,
                "vel_i_gain": 0.01,
                "trap_vel": 1.0,
                "trap_acc": 1.0,
                "trap_dec": 1.0,
                "vel_limit": 1.0,
                "vel_limit_tolerance": 4.0,
                "stiction_kick_nm": 0.0,
                "step_kwargs": {
                    "target_tolerance_turns": 0.03,
                    "target_vel_tolerance_turns_s": 0.20,
                },
            },
            "continuous_kwargs": {
                "move_mode": "mks_directional_direct",
                "candidate_preset": "mounted-direct-v3",
                "reuse_existing_calibration": True,
                "pole_pairs": 7,
                "calibration_current": 2.0,
                "encoder_offset_calibration_current": 8.0,
                "live_follow_supported": False,
                "pre_hold_s": 0.70,
                "final_hold_s": 0.90,
                "abort_abs_turns": 0.90,
                "timeout_s": 8.0,
                "min_delta_turns": 0.0015,
                "settle_s": 0.08,
                "quiet_hold_enable": False,
                "quiet_hold_s": 0.0,
                "quiet_hold_pos_gain_scale": 1.0,
                "quiet_hold_vel_gain_scale": 1.0,
                "quiet_hold_vel_i_gain": 0.0,
                "quiet_hold_vel_limit_scale": 1.0,
                "quiet_hold_persist": False,
                "quiet_hold_reanchor_err_turns": None,
                "fail_to_idle": True,
            },
            "validated_targets_deg": [],
            "limitations": [
                "Mounted direct-position path only; trap/operator move path is still a no-go.",
                "Longer moves can excite a low-frequency compliance/hysteresis wave.",
                "Motor-side encoder only; output precision is limited by gearbox hysteresis/compliance.",
                "Experimental: the shared direct-control family can shake/hunt on multiple boards.",
                "Live follow is disabled for this profile.",
            ],
        },
        "mks_mounted_direct_preload_coarse_v1_exp": {
            "profile_name": "mks_mounted_direct_preload_coarse_v1_exp",
            "load_mode": "mks_direct_position",
            "source": "codex_builtin_mks_profile",
            "experimental": True,
            "foundation_validated": False,
            "notes": (
                "Experimental mounted direct-position profile for the newer MKS motor/gearbox combination evaluated on 2026-03-25. "
                "It reuses the mounted-direct-v3 candidate and the same directional preload rule, but it is positioned explicitly as a coarse-motion "
                "evaluation path rather than a precision profile. Positive deltas approach from above, negative deltas from below. "
                "Current evidence says it is usable for medium moves, while very small moves remain too coarse for precision arm work."
            ),
            "require_repeatability": False,
            "stop_on_frame_jump": True,
            "stop_on_hard_fault": True,
            "suite_kwargs": {
                "current_lim": 6.0,
                "enable_overspeed_error": False,
                "pos_gain": 4.75,
                "vel_gain": 0.30,
                "vel_i_gain": 0.01,
                "trap_vel": 1.0,
                "trap_acc": 1.0,
                "trap_dec": 1.0,
                "vel_limit": 1.0,
                "vel_limit_tolerance": 4.0,
                "stiction_kick_nm": 0.0,
                "step_kwargs": {
                    "target_tolerance_turns": 0.03,
                    "target_vel_tolerance_turns_s": 0.20,
                },
            },
            "continuous_kwargs": {
                "move_mode": "mks_directional_direct",
                "candidate_preset": "mounted-direct-v3",
                "reuse_existing_calibration": True,
                "pole_pairs": 7,
                "calibration_current": 2.0,
                "encoder_offset_calibration_current": 8.0,
                "live_follow_supported": False,
                "pre_hold_s": 0.70,
                "final_hold_s": 0.90,
                "abort_abs_turns": 0.90,
                "timeout_s": 8.0,
                "min_delta_turns": 0.0015,
                "settle_s": 0.08,
                "quiet_hold_enable": False,
                "quiet_hold_s": 0.0,
                "quiet_hold_pos_gain_scale": 1.0,
                "quiet_hold_vel_gain_scale": 1.0,
                "quiet_hold_vel_i_gain": 0.0,
                "quiet_hold_vel_limit_scale": 1.0,
                "quiet_hold_persist": False,
                "quiet_hold_reanchor_err_turns": None,
                "fail_to_idle": True,
            },
            "validated_targets_deg": [],
            "limitations": [
                "Mounted direct-position path only; trap/operator move path is still a no-go.",
                "Use the built-in directional preload rule: positive deltas from above, negative deltas from below.",
                "This profile is for coarse mounted motion evaluation, not precision arm control.",
                "On the current new hardware, very small moves remain too coarse: roughly +0.15 motor turns on the positive side and -0.10 motor turns on the negative side are the first reasonably repeatable region.",
                "Motor-side encoder only; output precision is still limited by gearbox hysteresis/compliance.",
                "Live follow is disabled for this profile.",
            ],
        },
        "mks_mounted_direct_simple_v1_exp": {
            "profile_name": "mks_mounted_direct_simple_v1_exp",
            "load_mode": "mks_direct_position",
            "source": "codex_builtin_mks_profile",
            "experimental": True,
            "foundation_validated": False,
            "notes": (
                "Experimental mounted direct-position profile without directional preload. "
                "This exists only to isolate whether the preload stage is the blocker on the current MKS motor/gearbox setup."
            ),
            "require_repeatability": False,
            "stop_on_frame_jump": True,
            "stop_on_hard_fault": True,
            "suite_kwargs": {
                "current_lim": 6.0,
                "enable_overspeed_error": False,
                "pos_gain": 4.75,
                "vel_gain": 0.30,
                "vel_i_gain": 0.01,
                "trap_vel": 1.0,
                "trap_acc": 1.0,
                "trap_dec": 1.0,
                "vel_limit": 1.0,
                "vel_limit_tolerance": 4.0,
                "stiction_kick_nm": 0.0,
                "step_kwargs": {
                    "target_tolerance_turns": 0.03,
                    "target_vel_tolerance_turns_s": 0.20,
                },
            },
            "continuous_kwargs": {
                "move_mode": "mks_direct_position",
                "candidate_preset": "mounted-direct-v3",
                "reuse_existing_calibration": True,
                "pole_pairs": 7,
                "calibration_current": 2.0,
                "encoder_offset_calibration_current": 8.0,
                "live_follow_supported": False,
                "final_hold_s": 0.90,
                "abort_abs_turns": 2.00,
                "timeout_s": 8.0,
                "min_delta_turns": 0.0015,
                "settle_s": 0.08,
                "quiet_hold_enable": False,
                "quiet_hold_s": 0.0,
                "quiet_hold_pos_gain_scale": 1.0,
                "quiet_hold_vel_gain_scale": 1.0,
                "quiet_hold_vel_i_gain": 0.0,
                "quiet_hold_vel_limit_scale": 1.0,
                "quiet_hold_persist": False,
                "quiet_hold_reanchor_err_turns": None,
                "fail_to_idle": True,
            },
            "validated_targets_deg": [],
            "limitations": [
                "Mounted direct-position path only; no preload stage.",
                "Isolation profile only; use it to tell whether preload is the blocker.",
                "Motor-side encoder only; output precision is limited by gearbox hysteresis/compliance.",
                "Live follow is disabled for this profile.",
            ],
        },
        "mks_mounted_trap_v1_exp": {
            "profile_name": "mks_mounted_trap_v1_exp",
            "load_mode": "mks_trap_position",
            "source": "codex_builtin_mks_profile",
            "experimental": True,
            "foundation_validated": False,
            "notes": (
                "Experimental mounted trap-trajectory profile for the current MKS motor/gearbox setup. "
                "This uses the standard trap trajectory path again, so trap vel/acc/dec are active. "
                "Use it to test whether acceleration-shaped motion is more usable than the direct MKS move family."
            ),
            "require_repeatability": False,
            "stop_on_frame_jump": True,
            "stop_on_hard_fault": True,
            "suite_kwargs": {
                "current_lim": 8.0,
                "enable_overspeed_error": False,
                "pos_gain": 4.25,
                "vel_gain": 0.22,
                "vel_i_gain": 0.0,
                "trap_vel": 1.5,
                "trap_acc": 1.0,
                "trap_dec": 1.0,
                "vel_limit": 2.0,
                "vel_limit_tolerance": 4.0,
                "stiction_kick_nm": 0.0,
                "step_kwargs": {
                    "target_tolerance_turns": 0.04,
                    "target_vel_tolerance_turns_s": 0.20,
                },
            },
            "continuous_kwargs": {
                "move_mode": "trap_strict",
                "pole_pairs": 7,
                "calibration_current": 2.0,
                "encoder_offset_calibration_current": 8.0,
                "live_follow_supported": True,
                "timeout_s": 10.0,
                "min_delta_turns": 0.0015,
                "settle_s": 0.12,
                "quiet_hold_enable": False,
                "quiet_hold_s": 0.0,
                "quiet_hold_pos_gain_scale": 1.0,
                "quiet_hold_vel_gain_scale": 1.0,
                "quiet_hold_vel_i_gain": 0.0,
                "quiet_hold_vel_limit_scale": 1.0,
                "quiet_hold_persist": False,
                "quiet_hold_reanchor_err_turns": None,
                "fail_to_idle": True,
            },
            "validated_targets_deg": [],
            "limitations": [
                "Experimental mounted trap/trajectory path only; not a validated precision foundation.",
                "Trap vel/acc/dec are active on this profile and can be edited/forked meaningfully.",
                "Assumes the axis is already startup-ready; this profile itself does not run the direct MKS staged startup helper.",
                "Motor-side encoder only; output precision is still limited by gearbox hysteresis/compliance.",
            ],
        },
        "mks_mounted_trap_v2_soft_exp": {
            "profile_name": "mks_mounted_trap_v2_soft_exp",
            "load_mode": "mks_trap_position",
            "source": "codex_builtin_mks_profile",
            "experimental": True,
            "foundation_validated": False,
            "notes": (
                "Experimental very soft mounted trap-trajectory profile for the current MKS motor/gearbox setup. "
                "This backs off trap velocity and especially trap acceleration/deceleration to test whether the mounted combo can move at all under a gentler acceleration-shaped path."
            ),
            "require_repeatability": False,
            "stop_on_frame_jump": True,
            "stop_on_hard_fault": True,
            "suite_kwargs": {
                "current_lim": 8.0,
                "enable_overspeed_error": False,
                "pos_gain": 3.75,
                "vel_gain": 0.18,
                "vel_i_gain": 0.0,
                "trap_vel": 0.60,
                "trap_acc": 0.25,
                "trap_dec": 0.25,
                "vel_limit": 1.0,
                "vel_limit_tolerance": 4.0,
                "stiction_kick_nm": 0.0,
                "step_kwargs": {
                    "target_tolerance_turns": 0.04,
                    "target_vel_tolerance_turns_s": 0.20,
                },
            },
            "continuous_kwargs": {
                "move_mode": "trap_strict",
                "pole_pairs": 7,
                "calibration_current": 2.0,
                "encoder_offset_calibration_current": 8.0,
                "live_follow_supported": True,
                "timeout_s": 14.0,
                "min_delta_turns": 0.0015,
                "settle_s": 0.15,
                "quiet_hold_enable": False,
                "quiet_hold_s": 0.0,
                "quiet_hold_pos_gain_scale": 1.0,
                "quiet_hold_vel_gain_scale": 1.0,
                "quiet_hold_vel_i_gain": 0.0,
                "quiet_hold_vel_limit_scale": 1.0,
                "quiet_hold_persist": False,
                "quiet_hold_reanchor_err_turns": None,
                "fail_to_idle": True,
            },
            "validated_targets_deg": [],
            "limitations": [
                "Experimental mounted trap/trajectory path only; not a validated precision foundation.",
                "Much softer than mks_mounted_trap_v1_exp; intended only to answer whether the mounted combo can move under a gentle trap law at all.",
                "Trap vel/acc/dec are active on this profile and can be edited/forked meaningfully.",
                "Assumes the axis is already startup-ready; this profile itself does not run the direct MKS staged startup helper.",
                "Motor-side encoder only; output precision is still limited by gearbox hysteresis/compliance.",
            ],
        },
        "mks_mounted_direct_slew_v1_exp": {
            "profile_name": "mks_mounted_direct_slew_v1_exp",
            "load_mode": "mks_direct_position",
            "source": "codex_builtin_mks_profile",
            "experimental": True,
            "foundation_validated": False,
            "notes": (
                "Experimental conservative mounted direct-position preset that shapes long travel by slewing the commanded position "
                "instead of issuing one large step. This is the slower baseline for testing whether move-law shaping helps at all."
            ),
            "require_repeatability": False,
            "stop_on_frame_jump": True,
            "stop_on_hard_fault": True,
            "suite_kwargs": {
                "current_lim": 6.0,
                "enable_overspeed_error": False,
                "pos_gain": 4.75,
                "vel_gain": 0.30,
                "vel_i_gain": 0.01,
                "trap_vel": 1.0,
                "trap_acc": 1.0,
                "trap_dec": 1.0,
                "vel_limit": 1.0,
                "vel_limit_tolerance": 4.0,
                "stiction_kick_nm": 0.0,
                "step_kwargs": {
                    "target_tolerance_turns": 0.03,
                    "target_vel_tolerance_turns_s": 0.20,
                },
            },
            "continuous_kwargs": {
                "move_mode": "mks_directional_slew_direct",
                "candidate_preset": "mounted-direct-v3",
                "reuse_existing_calibration": True,
                "calibration_current": 2.0,
                "encoder_offset_calibration_current": 8.0,
                "live_follow_supported": False,
                "pre_hold_s": 0.25,
                "final_hold_s": 0.90,
                "abort_abs_turns": 3.00,
                "timeout_s": 12.0,
                "command_vel_turns_s": 0.30,
                "handoff_window_turns": 0.10,
                "command_dt": 0.01,
                "min_delta_turns": 0.0015,
                "settle_s": 0.08,
                "quiet_hold_enable": False,
                "quiet_hold_s": 0.0,
                "quiet_hold_pos_gain_scale": 1.0,
                "quiet_hold_vel_gain_scale": 1.0,
                "quiet_hold_vel_i_gain": 0.0,
                "quiet_hold_vel_limit_scale": 1.0,
                "quiet_hold_persist": False,
                "quiet_hold_reanchor_err_turns": None,
                "fail_to_idle": True,
            },
            "validated_targets_deg": [],
            "limitations": [
                "Mounted direct-position path only; trap/operator move path is still a no-go.",
                "Experimental conservative shaped-travel variant intended to reduce hunting during long moves.",
                "Motor-side encoder only; output precision is still limited by gearbox hysteresis/compliance.",
                "Experimental: this is a move-law experiment, not a validated precision foundation.",
                "Live follow is disabled for this profile.",
            ],
        },
        "mks_mounted_direct_slew_staged_v2_exp": {
            "profile_name": "mks_mounted_direct_slew_staged_v2_exp",
            "load_mode": "mks_direct_position",
            "source": "codex_builtin_mks_profile",
            "experimental": True,
            "foundation_validated": False,
            "notes": (
                "Experimental mounted direct-position preset with faster slew-shaped travel and softer travel-only gains. "
                "This is intended to test the classic underdamped step-response hypothesis without throwing away the stronger final settle. "
                "At a 25:1 gearbox ratio, the shaped travel command is roughly 36 output deg/s instead of the earlier very slow bring-up rate."
            ),
            "require_repeatability": False,
            "stop_on_frame_jump": True,
            "stop_on_hard_fault": True,
            "suite_kwargs": {
                "current_lim": 6.0,
                "enable_overspeed_error": False,
                "pos_gain": 4.75,
                "vel_gain": 0.30,
                "vel_i_gain": 0.01,
                "trap_vel": 1.0,
                "trap_acc": 1.0,
                "trap_dec": 1.0,
                "vel_limit": 1.0,
                "vel_limit_tolerance": 4.0,
                "stiction_kick_nm": 0.0,
                "step_kwargs": {
                    "target_tolerance_turns": 0.03,
                    "target_vel_tolerance_turns_s": 0.20,
                },
            },
            "continuous_kwargs": {
                "move_mode": "mks_directional_slew_direct",
                "candidate_preset": "mounted-direct-v3",
                "reuse_existing_calibration": True,
                "calibration_current": 2.0,
                "encoder_offset_calibration_current": 8.0,
                "live_follow_supported": False,
                "pre_hold_s": 0.15,
                "final_hold_s": 0.90,
                "abort_abs_turns": 3.00,
                "timeout_s": 12.0,
                "command_vel_turns_s": 2.50,
                "handoff_window_turns": 0.20,
                "command_dt": 0.01,
                "travel_pos_gain": 2.75,
                "travel_vel_gain": 0.22,
                "travel_vel_i_gain": 0.0,
                "travel_vel_limit": 3.00,
                "min_delta_turns": 0.0015,
                "settle_s": 0.08,
                "quiet_hold_enable": False,
                "quiet_hold_s": 0.0,
                "quiet_hold_pos_gain_scale": 1.0,
                "quiet_hold_vel_gain_scale": 1.0,
                "quiet_hold_vel_i_gain": 0.0,
                "quiet_hold_vel_limit_scale": 1.0,
                "quiet_hold_persist": False,
                "quiet_hold_reanchor_err_turns": None,
                "fail_to_idle": True,
            },
            "validated_targets_deg": [],
            "limitations": [
                "Mounted direct-position path only; trap/operator move path is still a no-go.",
                "Experimental faster shaped-travel variant with softer travel-only gains and stronger final settle gains.",
                "Motor-side encoder only; output precision is still limited by gearbox hysteresis/compliance.",
                "If this still shakes badly, the problem is below simple outer-loop move-law shaping.",
                "Live follow is disabled for this profile.",
            ],
        },
        "mks_mounted_velocity_travel_v1_exp": {
            "profile_name": "mks_mounted_velocity_travel_v1_exp",
            "load_mode": "mks_direct_position",
            "source": "codex_builtin_mks_profile",
            "experimental": True,
            "foundation_validated": False,
            "notes": (
                "Experimental mounted profile that uses velocity-led travel with direct final settle. "
                "This exists because the direct-position travel family can trade smoothness against speed, but not achieve both cleanly."
            ),
            "require_repeatability": False,
            "stop_on_frame_jump": True,
            "stop_on_hard_fault": True,
            "suite_kwargs": {
                "current_lim": 6.0,
                "enable_overspeed_error": False,
                "pos_gain": 4.75,
                "vel_gain": 0.30,
                "vel_i_gain": 0.01,
                "trap_vel": 1.0,
                "trap_acc": 1.0,
                "trap_dec": 1.0,
                "vel_limit": 1.0,
                "vel_limit_tolerance": 4.0,
                "stiction_kick_nm": 0.0,
                "step_kwargs": {
                    "target_tolerance_turns": 0.03,
                    "target_vel_tolerance_turns_s": 0.20,
                },
            },
            "continuous_kwargs": {
                "move_mode": "mks_directional_velocity_travel_direct",
                "candidate_preset": "mounted-direct-v3",
                "reuse_existing_calibration": True,
                "calibration_current": 2.0,
                "encoder_offset_calibration_current": 8.0,
                "live_follow_supported": False,
                "pre_hold_s": 0.10,
                "final_hold_s": 0.90,
                "abort_abs_turns": 3.00,
                "timeout_s": 12.0,
                "command_vel_turns_s": 4.00,
                "handoff_window_turns": 0.35,
                "command_dt": 0.01,
                "min_delta_turns": 0.0015,
                "settle_s": 0.08,
                "quiet_hold_enable": False,
                "quiet_hold_s": 0.0,
                "quiet_hold_pos_gain_scale": 1.0,
                "quiet_hold_vel_gain_scale": 1.0,
                "quiet_hold_vel_i_gain": 0.0,
                "quiet_hold_vel_limit_scale": 1.0,
                "quiet_hold_persist": False,
                "quiet_hold_reanchor_err_turns": None,
                "fail_to_idle": True,
            },
            "validated_targets_deg": [],
            "limitations": [
                "Mounted experimental path only; this is a travel-law experiment, not a validated precision foundation.",
                "Uses velocity-led travel and direct final settle to test whether travel hunting is mainly caused by position chasing.",
                "Motor-side encoder only; output precision is still limited by gearbox hysteresis/compliance.",
                "Live follow is disabled for this profile.",
            ],
        },
        "mks_mounted_velocity_point_to_point_v1_exp": {
            "profile_name": "mks_mounted_velocity_point_to_point_v1_exp",
            "load_mode": "mks_direct_position",
            "source": "codex_builtin_mks_profile",
            "experimental": True,
            "foundation_validated": False,
            "notes": (
                "Experimental mounted profile that stays in velocity control until it is actually near the target, "
                "then uses direct position hold only for the final tiny capture. "
                "This is the first serious attempt to build point-to-point motion on top of the now-working velocity layer."
            ),
            "require_repeatability": False,
            "stop_on_frame_jump": True,
            "stop_on_hard_fault": True,
            "suite_kwargs": {
                "current_lim": 8.0,
                "enable_overspeed_error": False,
                "pos_gain": 4.75,
                "vel_gain": 0.30,
                "vel_i_gain": 0.01,
                "trap_vel": 1.0,
                "trap_acc": 1.0,
                "trap_dec": 1.0,
                "vel_limit": 1.0,
                "vel_limit_tolerance": 4.0,
                "stiction_kick_nm": 0.0,
                "step_kwargs": {
                    "target_tolerance_turns": 0.03,
                    "target_vel_tolerance_turns_s": 0.20,
                },
            },
            "continuous_kwargs": {
                "move_mode": "mks_velocity_point_to_point_direct",
                "candidate_preset": "mounted-direct-v3",
                "reuse_existing_calibration": True,
                "calibration_current": 2.0,
                "encoder_offset_calibration_current": 8.0,
                "live_follow_supported": False,
                "final_hold_s": 0.90,
                "abort_abs_turns": 3.00,
                "timeout_s": 12.0,
                "command_vel_turns_s": 1.00,
                "handoff_window_turns": 0.15,
                "command_dt": 0.01,
                "min_delta_turns": 0.0015,
                "settle_s": 0.08,
                "quiet_hold_enable": False,
                "quiet_hold_s": 0.0,
                "quiet_hold_pos_gain_scale": 1.0,
                "quiet_hold_vel_gain_scale": 1.0,
                "quiet_hold_vel_i_gain": 0.0,
                "quiet_hold_vel_limit_scale": 1.0,
                "quiet_hold_persist": False,
                "quiet_hold_reanchor_err_turns": None,
                "fail_to_idle": True,
            },
            "validated_targets_deg": [],
            "limitations": [
                "Mounted experimental path only; this is not a validated precision foundation yet.",
                "Velocity-led point-to-point move that avoids the old early handoff into dead direct-position travel.",
                "Current board testing says handoff at 0.15 turns is the least-bad setting so far, but final hold still misses by roughly 0.05 to 0.08 motor turns on 5 to 10 degree output moves.",
                "Motor-side encoder only; output precision is still limited by gearbox hysteresis/compliance.",
                "Live follow is disabled for this profile.",
            ],
        },
        "mks_mounted_direct_preload_soft_v4_exp": {
            "profile_name": "mks_mounted_direct_preload_soft_v4_exp",
            "load_mode": "mks_direct_position",
            "source": "codex_builtin_mks_profile",
            "experimental": True,
            "foundation_validated": False,
            "notes": (
                "Experimental softer mounted direct-position preset for MKS A/B testing. "
                "It keeps the same directional preload move path as mounted-direct-v3 but backs off position stiffness. "
                "Use it only for characterization; it is not promoted as the default MKS mounted preset."
            ),
            "require_repeatability": False,
            "stop_on_frame_jump": True,
            "stop_on_hard_fault": True,
            "suite_kwargs": {
                "current_lim": 6.0,
                "enable_overspeed_error": False,
                "pos_gain": 4.25,
                "vel_gain": 0.30,
                "vel_i_gain": 0.01,
                "trap_vel": 1.0,
                "trap_acc": 1.0,
                "trap_dec": 1.0,
                "vel_limit": 1.0,
                "vel_limit_tolerance": 4.0,
                "stiction_kick_nm": 0.0,
                "step_kwargs": {
                    "target_tolerance_turns": 0.03,
                    "target_vel_tolerance_turns_s": 0.20,
                },
            },
            "continuous_kwargs": {
                "move_mode": "mks_directional_direct",
                "candidate_preset": "mounted-direct-soft-v4-exp",
                "reuse_existing_calibration": True,
                "calibration_current": 2.0,
                "encoder_offset_calibration_current": 8.0,
                "live_follow_supported": False,
                "pre_hold_s": 0.70,
                "final_hold_s": 0.90,
                "abort_abs_turns": 0.90,
                "timeout_s": 8.0,
                "min_delta_turns": 0.0015,
                "settle_s": 0.08,
                "quiet_hold_enable": False,
                "quiet_hold_s": 0.0,
                "quiet_hold_pos_gain_scale": 1.0,
                "quiet_hold_vel_gain_scale": 1.0,
                "quiet_hold_vel_i_gain": 0.0,
                "quiet_hold_vel_limit_scale": 1.0,
                "quiet_hold_persist": False,
                "quiet_hold_reanchor_err_turns": None,
                "fail_to_idle": True,
            },
            "validated_targets_deg": [],
            "limitations": [
                "Experimental MKS-only mounted direct-position variant for A/B testing.",
                "Mounted direct-position path only; trap/operator move path is still a no-go.",
                "Softer than mks_mounted_direct_preload_v3; may reduce some hold metrics while preserving the same shared hunting-travel foundation.",
                "Experimental: the shared direct-control family can shake/hunt on multiple boards.",
                "Live follow is disabled for this profile.",
            ],
        },
        "mks_mounted_direct_preload_long_v1": {
            "profile_name": "mks_mounted_direct_preload_long_v1",
            "load_mode": "mks_direct_position",
            "source": "codex_builtin_mks_profile",
            "experimental": True,
            "foundation_validated": False,
            "notes": (
                "Experimental derated mounted direct-position preset for longer one-shot moves. "
                "This backs off authority to reduce the soft low-frequency wave, but it is not a validated precision foundation."
            ),
            "require_repeatability": False,
            "stop_on_frame_jump": True,
            "stop_on_hard_fault": True,
            "suite_kwargs": {
                "current_lim": 6.0,
                "enable_overspeed_error": False,
                "pos_gain": 4.25,
                "vel_gain": 0.22,
                "vel_i_gain": 0.0,
                "trap_vel": 0.60,
                "trap_acc": 0.60,
                "trap_dec": 0.60,
                "vel_limit": 0.60,
                "vel_limit_tolerance": 4.0,
                "stiction_kick_nm": 0.0,
                "step_kwargs": {
                    "target_tolerance_turns": 0.03,
                    "target_vel_tolerance_turns_s": 0.20,
                },
            },
            "continuous_kwargs": {
                "move_mode": "mks_directional_direct",
                "candidate_preset": "mounted-direct-v3",
                "reuse_existing_calibration": True,
                "calibration_current": 2.0,
                "encoder_offset_calibration_current": 8.0,
                "live_follow_supported": False,
                "pre_hold_s": 0.70,
                "final_hold_s": 1.10,
                "abort_abs_turns": 0.90,
                "timeout_s": 10.0,
                "min_delta_turns": 0.0015,
                "settle_s": 0.10,
                "quiet_hold_enable": False,
                "quiet_hold_s": 0.0,
                "quiet_hold_pos_gain_scale": 1.0,
                "quiet_hold_vel_gain_scale": 1.0,
                "quiet_hold_vel_i_gain": 0.0,
                "quiet_hold_vel_limit_scale": 1.0,
                "quiet_hold_persist": False,
                "quiet_hold_reanchor_err_turns": None,
                "fail_to_idle": True,
            },
            "validated_targets_deg": [],
            "limitations": [
                "Experimental derated mounted direct-position profile for longer one-shot moves.",
                "Mounted direct-position path only; trap/operator move path is still a no-go.",
                "Lower authority than mks_mounted_direct_preload_v3; expect slower travel and more static error.",
                "Motor-side encoder only; output precision is limited by gearbox hysteresis/compliance.",
                "Experimental: the shared direct-control family can shake/hunt on multiple boards.",
                "Live follow is disabled for this profile.",
            ],
        },
    }


def _continuous_profile_catalog() -> dict[str, dict[str, Any]]:
    data = json.loads(_profiles_path().read_text())
    profiles = dict(_builtin_continuous_profiles())
    profiles.update(dict(data.get("profiles") or {}))
    return profiles


def _continuous_profile_records() -> list[dict[str, Any]]:
    profiles = _continuous_profile_catalog()
    rows: list[dict[str, Any]] = []
    for name, payload in profiles.items():
        if ("continuous_kwargs" in (payload or {})) or ("continuous" in str(name)):
            prof = dict(payload or {})
            rows.append({
                "name": str(name),
                "notes": str(prof.get("notes") or ""),
                "limitations": list(prof.get("limitations") or []),
                "source": str(prof.get("source") or ""),
                "experimental": bool(prof.get("experimental", False)),
                "foundation_validated": bool(prof.get("foundation_validated", False)),
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
    profiles = _continuous_profile_catalog()
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
        "experimental": bool(prof.get("experimental", False)),
        "foundation_validated": bool(prof.get("foundation_validated", False)),
        "load_mode": str(prof.get("load_mode") or "loaded"),
        "move_mode": str(extra.get("move_mode") or "trap_strict"),
        "candidate_preset": (None if not str(extra.get("candidate_preset") or "").strip() else str(extra.get("candidate_preset"))),
        "reuse_existing_calibration": bool(extra.get("reuse_existing_calibration", False)),
        "pole_pairs": (None if extra.get("pole_pairs") is None else int(extra.get("pole_pairs"))),
        "calibration_current": (None if extra.get("calibration_current") is None else float(extra.get("calibration_current"))),
        "encoder_offset_calibration_current": (
            None if extra.get("encoder_offset_calibration_current") is None else float(extra.get("encoder_offset_calibration_current"))
        ),
        "live_follow_supported": bool(extra.get("live_follow_supported", True)),
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
        "pre_hold_s": (None if extra.get("pre_hold_s") is None else float(extra.get("pre_hold_s"))),
        "final_hold_s": (None if extra.get("final_hold_s") is None else float(extra.get("final_hold_s"))),
        "abort_abs_turns": (None if extra.get("abort_abs_turns") is None else float(extra.get("abort_abs_turns"))),
        "command_vel_turns_s": (None if extra.get("command_vel_turns_s") is None else float(extra.get("command_vel_turns_s"))),
        "handoff_window_turns": (None if extra.get("handoff_window_turns") is None else float(extra.get("handoff_window_turns"))),
        "command_dt": (None if extra.get("command_dt") is None else float(extra.get("command_dt"))),
        "travel_pos_gain": (None if extra.get("travel_pos_gain") is None else float(extra.get("travel_pos_gain"))),
        "travel_vel_gain": (None if extra.get("travel_vel_gain") is None else float(extra.get("travel_vel_gain"))),
        "travel_vel_i_gain": (None if extra.get("travel_vel_i_gain") is None else float(extra.get("travel_vel_i_gain"))),
        "travel_vel_limit": (None if extra.get("travel_vel_limit") is None else float(extra.get("travel_vel_limit"))),
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

    existing = dict((_continuous_profile_catalog().get(name) or {}))
    limitations = [str(item).strip() for item in list(profile_payload.get("limitations") or []) if str(item).strip()]

    existing_suite = dict(existing.get("suite_kwargs") or {})
    existing_step = dict(existing_suite.get("step_kwargs") or {})
    existing_continuous = dict(existing.get("continuous_kwargs") or {})

    step_kwargs = dict(existing_step)
    step_kwargs.update({
        "target_tolerance_turns": float(profile_payload.get("target_tolerance_turns", 0.03)),
        "target_vel_tolerance_turns_s": float(profile_payload.get("target_vel_tolerance_turns_s", 0.20)),
    })

    suite = dict(existing_suite)
    suite.update({
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
        "step_kwargs": step_kwargs,
    })
    continuous = dict(existing_continuous)
    continuous.update({
        "move_mode": str(profile_payload.get("move_mode") or existing_continuous.get("move_mode") or "trap_strict"),
        "candidate_preset": (
            None
            if profile_payload.get("candidate_preset") in (None, "")
            else str(profile_payload.get("candidate_preset"))
        ),
        "reuse_existing_calibration": bool(profile_payload.get("reuse_existing_calibration", existing_continuous.get("reuse_existing_calibration", False))),
        "pole_pairs": (
            None if profile_payload.get("pole_pairs") in (None, "")
            else int(profile_payload.get("pole_pairs"))
        ),
        "calibration_current": (
            None if profile_payload.get("calibration_current") in (None, "")
            else float(profile_payload.get("calibration_current"))
        ),
        "encoder_offset_calibration_current": (
            None if profile_payload.get("encoder_offset_calibration_current") in (None, "")
            else float(profile_payload.get("encoder_offset_calibration_current"))
        ),
        "live_follow_supported": bool(profile_payload.get("live_follow_supported", existing_continuous.get("live_follow_supported", True))),
        "timeout_s": float(profile_payload.get("timeout_s", 8.0)),
        "min_delta_turns": float(profile_payload.get("min_delta_turns", 0.0015)),
        "settle_s": float(profile_payload.get("settle_s", 0.08)),
        "pre_hold_s": (
            None if profile_payload.get("pre_hold_s") in (None, "")
            else float(profile_payload.get("pre_hold_s"))
        ),
        "final_hold_s": (
            None if profile_payload.get("final_hold_s") in (None, "")
            else float(profile_payload.get("final_hold_s"))
        ),
        "abort_abs_turns": (
            None if profile_payload.get("abort_abs_turns") in (None, "")
            else float(profile_payload.get("abort_abs_turns"))
        ),
        "command_vel_turns_s": (
            None if profile_payload.get("command_vel_turns_s") in (None, "")
            else float(profile_payload.get("command_vel_turns_s"))
        ),
        "handoff_window_turns": (
            None if profile_payload.get("handoff_window_turns") in (None, "")
            else float(profile_payload.get("handoff_window_turns"))
        ),
        "command_dt": (
            None if profile_payload.get("command_dt") in (None, "")
            else float(profile_payload.get("command_dt"))
        ),
        "travel_pos_gain": (
            None if profile_payload.get("travel_pos_gain") in (None, "")
            else float(profile_payload.get("travel_pos_gain"))
        ),
        "travel_vel_gain": (
            None if profile_payload.get("travel_vel_gain") in (None, "")
            else float(profile_payload.get("travel_vel_gain"))
        ),
        "travel_vel_i_gain": (
            None if profile_payload.get("travel_vel_i_gain") in (None, "")
            else float(profile_payload.get("travel_vel_i_gain"))
        ),
        "travel_vel_limit": (
            None if profile_payload.get("travel_vel_limit") in (None, "")
            else float(profile_payload.get("travel_vel_limit"))
        ),
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
    })

    rec = {
        "profile_name": name,
        "load_mode": str(profile_payload.get("load_mode") or existing.get("load_mode") or "loaded"),
        "source": str(profile_payload.get("source") or existing.get("source") or "focui_manual_editor"),
        "experimental": bool(profile_payload.get("experimental", existing.get("experimental", False))),
        "foundation_validated": bool(profile_payload.get("foundation_validated", existing.get("foundation_validated", False))),
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
    profiles = _continuous_profile_catalog()
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
        "move_mode": str(extra.get("move_mode") or "trap_strict"),
        "candidate_preset": str(extra.get("candidate_preset") or ""),
        "reuse_existing_calibration": bool(extra.get("reuse_existing_calibration", False)),
        "pole_pairs": (None if extra.get("pole_pairs") is None else int(extra.get("pole_pairs"))),
        "calibration_current": (None if extra.get("calibration_current") is None else float(extra.get("calibration_current"))),
        "encoder_offset_calibration_current": (
            None if extra.get("encoder_offset_calibration_current") is None else float(extra.get("encoder_offset_calibration_current"))
        ),
        "live_follow_supported": bool(extra.get("live_follow_supported", True)),
        "pre_hold_s": float(extra.get("pre_hold_s", 0.70)),
        "final_hold_s": float(extra.get("final_hold_s", 0.90)),
        "abort_abs_turns": float(extra.get("abort_abs_turns", 0.90)),
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


def _move_stage_by_name(stages: list[dict[str, Any]], *names: str) -> dict[str, Any] | None:
    wanted = {str(name).strip().lower() for name in names if str(name).strip()}
    for stage in stages:
        if str((stage or {}).get("stage") or "").strip().lower() in wanted:
            return dict(stage or {})
    return None


def _classify_travel_stage(stage: dict[str, Any]) -> str | None:
    if not stage:
        return None
    if not bool(stage.get("ok", True)):
        return "faulted"
    span = max(
        abs(float(stage.get("target", 0.0)) - float(stage.get("start_pos", 0.0))),
        abs(float(stage.get("dp", 0.0))),
        1e-6,
    )
    monotonic = float(stage.get("monotonic_fraction", 1.0))
    backtrack = abs(float(stage.get("backtrack_turns", 0.0)))
    flips = int(stage.get("active_vel_sign_flips", 0))
    err_abs = abs(float(stage.get("final_error_abs", 0.0)))
    backtrack_ratio = backtrack / span
    err_ratio = err_abs / span

    if monotonic >= 0.98 and backtrack_ratio <= 0.01 and flips == 0 and err_ratio <= 0.10:
        return "clean_travel"
    if monotonic >= 0.93 and backtrack_ratio <= 0.03 and flips <= 1 and err_ratio <= 0.18:
        return "mild_wave"
    if monotonic >= 0.82 and backtrack_ratio <= 0.08 and flips <= 2 and err_ratio <= 0.30:
        return "wavy_travel"
    return "hunting_travel"


def _travel_diagnostics_from_move(
    raw: dict[str, Any] | None,
    *,
    move_mode: str,
    angle_space: str,
    gear_ratio: float,
) -> dict[str, Any] | None:
    if not isinstance(raw, dict):
        return None
    stages = [dict(stage or {}) for stage in list(raw.get("stages") or []) if isinstance(stage, dict)]
    target_stage = _move_stage_by_name(stages, "target_travel", "target")
    if not target_stage:
        return None

    duration_s = target_stage.get("reach_time_s")
    duration_s = None if duration_s in (None, "") else float(duration_s)
    span_turns = abs(
        float(target_stage.get("target", 0.0)) - float(target_stage.get("start_pos", 0.0))
    )
    achieved_avg_turns_s = None
    if duration_s is not None and duration_s > 1e-6:
        achieved_avg_turns_s = abs(float(target_stage.get("dp", 0.0))) / float(duration_s)
    commanded_turns_s = target_stage.get("command_vel_turns_s")
    commanded_turns_s = None if commanded_turns_s in (None, "") else abs(float(commanded_turns_s))
    achieved_fraction = None
    if (
        achieved_avg_turns_s is not None
        and commanded_turns_s is not None
        and commanded_turns_s > 1e-6
    ):
        achieved_fraction = float(achieved_avg_turns_s) / float(commanded_turns_s)

    final_error_abs_turns = abs(float(target_stage.get("final_error_abs", 0.0)))
    if str(move_mode).strip().lower() == "mks_velocity_point_to_point_direct":
        final_error_abs_turns = abs(float(raw.get("final_error_abs", final_error_abs_turns)))

    out = {
        "move_mode": str(move_mode),
        "stage": str(target_stage.get("stage") or ""),
        "travel_classification": _classify_travel_stage(target_stage),
        "span_turns": float(span_turns),
        "duration_s": duration_s,
        "commanded_turns_s": commanded_turns_s,
        "achieved_avg_turns_s": achieved_avg_turns_s,
        "achieved_fraction_of_commanded": achieved_fraction,
        "peak_turns_s": float(target_stage.get("peak_vel", 0.0)),
        "monotonic_fraction": float(target_stage.get("monotonic_fraction", 0.0)),
        "backtrack_turns": float(target_stage.get("backtrack_turns", 0.0)),
        "vel_sign_flips": int(target_stage.get("active_vel_sign_flips", 0)),
        "final_error_abs_turns": float(final_error_abs_turns),
    }

    if str(move_mode).strip().lower() == "mks_velocity_point_to_point_direct":
        for key in (
            "handoff_reached",
            "handoff_error_turns",
            "handoff_time_s",
            "capture_start_error_turns",
            "capture_end_error_turns",
            "capture_improvement_turns",
            "failure_stage",
            "phase_summary",
        ):
            value = raw.get(key)
            if value is not None:
                out[key] = value

    if str(angle_space).strip().lower() == "gearbox_output" and float(gear_ratio) > 0.0:
        scale = 360.0 / float(gear_ratio)
        if commanded_turns_s is not None:
            out["commanded_output_deg_s"] = float(commanded_turns_s) * scale
        if achieved_avg_turns_s is not None:
            out["achieved_avg_output_deg_s"] = float(achieved_avg_turns_s) * scale
        out["peak_output_deg_s"] = float(out["peak_turns_s"]) * scale
        out["final_error_abs_output_deg"] = float(out["final_error_abs_turns"]) * scale
        for key, output_key in (
            ("handoff_error_turns", "handoff_error_output_deg"),
            ("capture_start_error_turns", "capture_start_error_output_deg"),
            ("capture_end_error_turns", "capture_end_error_output_deg"),
            ("capture_improvement_turns", "capture_improvement_output_deg"),
        ):
            value = out.get(key)
            if value is not None:
                out[output_key] = float(value) * scale
    return out


def _candidate_override_from_cfg(cfg: dict[str, Any]) -> dict[str, float]:
    out: dict[str, float] = {}
    for key in ("current_lim", "pos_gain", "vel_gain", "vel_i_gain", "vel_limit"):
        value = cfg.get(key)
        if value is None:
            continue
        out[key] = float(value)
    return out


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
    speed_scale: float | None = None,
    fail_to_idle_override: bool | None = None,
    sample_hook=None,
) -> dict[str, Any]:
    cfg = _load_continuous_move_kwargs(profile_name=profile_name)
    move_mode = str(cfg.get("move_mode") or "trap_strict").strip().lower()
    candidate_override = _candidate_override_from_cfg(cfg)
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
    elif move_mode == "trap_strict":
        move_dist = abs(float(target_turns_motor) - float(start_turns_motor))
        est_total = _trap_move_time_est(
            move_dist,
            float(cfg["trap_vel"]),
            float(cfg["trap_acc"]),
            float(cfg["trap_dec"]),
        ) + float(cfg["settle_s"]) + 0.75
        cfg["timeout_s"] = max(float(cfg["timeout_s"]), float(est_total))
    if move_mode == "mks_direct_position":
        raw = run_direct_move(
            axis=axis,
            candidate_preset=(str(cfg.get("candidate_preset") or profile_name)),
            candidate_override=candidate_override,
            reuse_existing_calibration=bool(cfg.get("reuse_existing_calibration", True)),
            pole_pairs=cfg.get("pole_pairs"),
            calibration_current=cfg.get("calibration_current"),
            encoder_offset_calibration_current=cfg.get("encoder_offset_calibration_current"),
            target_turns=float(target_turns_motor),
            timeout_s=float(cfg["timeout_s"]),
            final_hold_s=float(cfg.get("final_hold_s", 0.90)),
            return_to_start=False,
            abort_abs_turns=float(cfg.get("abort_abs_turns", 0.90)),
            target_tolerance_turns=float(cfg["target_tolerance_turns"]),
            target_vel_tolerance_turns_s=float(cfg["target_vel_tolerance_turns_s"]),
        )
        out = dict(raw or {})
        out["move_mode"] = move_mode
        out["profile_name"] = str(profile_name)
        out["angle_space"] = str(angle_space)
        out["angle_deg"] = float(angle_deg)
        out["gear_ratio"] = float(gear_ratio)
        out["start_turns_motor"] = float(start_turns_motor)
        out["zero_turns_motor"] = float(base_turns_motor)
        out["target_turns_motor"] = float(target_turns_motor)
        out["travel_diagnostics"] = _travel_diagnostics_from_move(
            raw,
            move_mode=move_mode,
            angle_space=space,
            gear_ratio=float(gear_ratio),
        )
        return out
    if move_mode == "mks_directional_direct":
        raw = run_directional_move(
            axis=axis,
            candidate_preset=(str(cfg.get("candidate_preset") or profile_name)),
            candidate_override=candidate_override,
            reuse_existing_calibration=bool(cfg.get("reuse_existing_calibration", True)),
            pole_pairs=cfg.get("pole_pairs"),
            calibration_current=cfg.get("calibration_current"),
            encoder_offset_calibration_current=cfg.get("encoder_offset_calibration_current"),
            target_turns=float(target_turns_motor),
            timeout_s=float(cfg["timeout_s"]),
            pre_hold_s=float(cfg.get("pre_hold_s", 0.70)),
            final_hold_s=float(cfg.get("final_hold_s", 0.90)),
            return_to_start=False,
            abort_abs_turns=float(cfg.get("abort_abs_turns", 0.90)),
            target_tolerance_turns=float(cfg["target_tolerance_turns"]),
            target_vel_tolerance_turns_s=float(cfg["target_vel_tolerance_turns_s"]),
        )
        out = dict(raw or {})
        out["move_mode"] = move_mode
        out["profile_name"] = str(profile_name)
        out["angle_space"] = str(angle_space)
        out["angle_deg"] = float(angle_deg)
        out["gear_ratio"] = float(gear_ratio)
        out["start_turns_motor"] = float(start_turns_motor)
        out["zero_turns_motor"] = float(base_turns_motor)
        out["target_turns_motor"] = float(target_turns_motor)
        out["travel_diagnostics"] = _travel_diagnostics_from_move(
            raw,
            move_mode=move_mode,
            angle_space=space,
            gear_ratio=float(gear_ratio),
        )
        return out
    if move_mode == "mks_directional_velocity_travel_direct":
        if speed_scale is not None:
            scale = max(0.05, float(speed_scale))
            base_command_vel = float(cfg.get("command_vel_turns_s", 0.30))
            base_command_dt = float(cfg.get("command_dt", 0.01))
            base_handoff_window = float(cfg.get("handoff_window_turns", 0.10))
            cfg["command_vel_turns_s"] = max(0.05, base_command_vel * scale)
            cfg["command_dt"] = max(
                0.003,
                base_command_dt / max(1.0, min(scale, 4.0)),
            )
            # Keep the velocity-led handoff from growing with speed.
            # The earlier version made 2.0x/3.0x start braking sooner and
            # spend more time in the slower direct-settle phase.
            cfg["handoff_window_turns"] = max(
                0.20,
                min(
                    base_handoff_window,
                    base_handoff_window / max(1.0, min(scale, 4.0) ** 0.25),
                ),
            )
        raw = run_directional_velocity_travel_move(
            axis=axis,
            candidate_preset=(str(cfg.get("candidate_preset") or profile_name)),
            candidate_override=candidate_override,
            reuse_existing_calibration=bool(cfg.get("reuse_existing_calibration", True)),
            pole_pairs=cfg.get("pole_pairs"),
            calibration_current=cfg.get("calibration_current"),
            encoder_offset_calibration_current=cfg.get("encoder_offset_calibration_current"),
            target_turns=float(target_turns_motor),
            timeout_s=float(cfg["timeout_s"]),
            pre_hold_s=float(cfg.get("pre_hold_s", 0.10)),
            final_hold_s=float(cfg.get("final_hold_s", 0.90)),
            return_to_start=False,
            abort_abs_turns=float(cfg.get("abort_abs_turns", 3.00)),
            target_tolerance_turns=float(cfg["target_tolerance_turns"]),
            target_vel_tolerance_turns_s=float(cfg["target_vel_tolerance_turns_s"]),
            command_vel_turns_s=float(cfg.get("command_vel_turns_s", 4.00)),
            handoff_window_turns=float(cfg.get("handoff_window_turns", 0.35)),
            command_dt=float(cfg.get("command_dt", 0.01)),
        )
        out = dict(raw or {})
        out["move_mode"] = move_mode
        out["profile_name"] = str(profile_name)
        out["angle_space"] = str(angle_space)
        out["angle_deg"] = float(angle_deg)
        out["gear_ratio"] = float(gear_ratio)
        out["start_turns_motor"] = float(start_turns_motor)
        out["zero_turns_motor"] = float(base_turns_motor)
        out["target_turns_motor"] = float(target_turns_motor)
        out["speed_scale"] = None if speed_scale is None else float(speed_scale)
        out["effective_command_vel_turns_s"] = float(cfg.get("command_vel_turns_s", 4.00))
        out["effective_command_dt"] = float(cfg.get("command_dt", 0.01))
        out["effective_handoff_window_turns"] = float(cfg.get("handoff_window_turns", 0.35))
        out["travel_diagnostics"] = _travel_diagnostics_from_move(
            raw,
            move_mode=move_mode,
            angle_space=space,
            gear_ratio=float(gear_ratio),
        )
        return out
    if move_mode == "mks_velocity_point_to_point_direct":
        if speed_scale is not None:
            scale = max(0.05, float(speed_scale))
            base_command_vel = float(cfg.get("command_vel_turns_s", 1.00))
            base_command_dt = float(cfg.get("command_dt", 0.01))
            base_slowdown_window = float(cfg.get("handoff_window_turns", 0.20))
            cfg["command_vel_turns_s"] = max(0.05, base_command_vel * scale)
            cfg["command_dt"] = max(
                0.003,
                base_command_dt / max(1.0, min(scale, 4.0)),
            )
            cfg["handoff_window_turns"] = max(
                0.06,
                min(
                    base_slowdown_window * max(1.0, min(scale, 4.0) ** 0.5),
                    base_slowdown_window * 2.0,
                ),
            )
        raw = run_velocity_point_to_point_move(
            axis=axis,
            candidate_preset=(str(cfg.get("candidate_preset") or profile_name)),
            candidate_override=candidate_override,
            reuse_existing_calibration=bool(cfg.get("reuse_existing_calibration", True)),
            pole_pairs=cfg.get("pole_pairs"),
            calibration_current=cfg.get("calibration_current"),
            encoder_offset_calibration_current=cfg.get("encoder_offset_calibration_current"),
            target_turns=float(target_turns_motor),
            timeout_s=float(cfg["timeout_s"]),
            final_hold_s=float(cfg.get("final_hold_s", 0.90)),
            return_to_start=False,
            abort_abs_turns=float(cfg.get("abort_abs_turns", 3.00)),
            target_tolerance_turns=float(cfg["target_tolerance_turns"]),
            target_vel_tolerance_turns_s=float(cfg["target_vel_tolerance_turns_s"]),
            command_vel_turns_s=float(cfg.get("command_vel_turns_s", 1.00)),
            handoff_window_turns=float(cfg.get("handoff_window_turns", 0.20)),
            command_dt=float(cfg.get("command_dt", 0.01)),
        )
        out = dict(raw or {})
        out["move_mode"] = move_mode
        out["profile_name"] = str(profile_name)
        out["angle_space"] = str(angle_space)
        out["angle_deg"] = float(angle_deg)
        out["gear_ratio"] = float(gear_ratio)
        out["start_turns_motor"] = float(start_turns_motor)
        out["zero_turns_motor"] = float(base_turns_motor)
        out["target_turns_motor"] = float(target_turns_motor)
        out["speed_scale"] = None if speed_scale is None else float(speed_scale)
        out["effective_command_vel_turns_s"] = float(cfg.get("command_vel_turns_s", 1.00))
        out["effective_command_dt"] = float(cfg.get("command_dt", 0.01))
        out["effective_handoff_window_turns"] = float(cfg.get("handoff_window_turns", 0.20))
        out["travel_diagnostics"] = _travel_diagnostics_from_move(
            raw,
            move_mode=move_mode,
            angle_space=space,
            gear_ratio=float(gear_ratio),
        )
        if not bool(out.get("ok", True)):
            failure_stage = str(out.get("failure_stage") or "move_failed")
            detail = str(out.get("error") or "velocity point-to-point move failed")
            raise MoveResultError(f"{failure_stage}: {detail}", result=out)
        return out
    if move_mode == "mks_directional_slew_direct":
        if speed_scale is not None:
            scale = max(0.05, float(speed_scale))
            base_command_vel = float(cfg.get("command_vel_turns_s", 0.30))
            base_command_dt = float(cfg.get("command_dt", 0.01))
            base_handoff_window = float(cfg.get("handoff_window_turns", 0.10))
            cfg["command_vel_turns_s"] = max(
                0.05,
                base_command_vel * scale,
            )
            # Keep the per-update command step from exploding as speed rises.
            cfg["command_dt"] = max(
                0.003,
                base_command_dt / max(1.0, min(scale, 4.0)),
            )
            # Hand off earlier at higher speed so the final settle takes over sooner.
            cfg["handoff_window_turns"] = max(
                base_handoff_window,
                base_handoff_window * min(max(scale, 1.0) ** 0.5, 2.0),
            )
            # Soften travel-only position stiffness at higher speed to reduce hunting.
            if cfg.get("travel_pos_gain") is not None:
                cfg["travel_pos_gain"] = max(
                    1.5,
                    float(cfg.get("travel_pos_gain")) / max(1.0, min(scale, 4.0) ** 0.5),
                )
            base_travel_vel_limit = cfg.get("travel_vel_limit", cfg.get("vel_limit", 1.0))
            cfg["travel_vel_limit"] = max(
                abs(float(cfg["command_vel_turns_s"])) * 1.2,
                float(base_travel_vel_limit) * scale,
            )
        raw = run_directional_slew_move(
            axis=axis,
            candidate_preset=(str(cfg.get("candidate_preset") or profile_name)),
            candidate_override=candidate_override,
            reuse_existing_calibration=bool(cfg.get("reuse_existing_calibration", True)),
            pole_pairs=cfg.get("pole_pairs"),
            calibration_current=cfg.get("calibration_current"),
            encoder_offset_calibration_current=cfg.get("encoder_offset_calibration_current"),
            target_turns=float(target_turns_motor),
            timeout_s=float(cfg["timeout_s"]),
            pre_hold_s=float(cfg.get("pre_hold_s", 0.25)),
            final_hold_s=float(cfg.get("final_hold_s", 0.90)),
            return_to_start=False,
            abort_abs_turns=float(cfg.get("abort_abs_turns", 3.00)),
            target_tolerance_turns=float(cfg["target_tolerance_turns"]),
            target_vel_tolerance_turns_s=float(cfg["target_vel_tolerance_turns_s"]),
            command_vel_turns_s=float(cfg.get("command_vel_turns_s", 0.30)),
            handoff_window_turns=float(cfg.get("handoff_window_turns", 0.10)),
            command_dt=float(cfg.get("command_dt", 0.01)),
            travel_pos_gain=cfg.get("travel_pos_gain"),
            travel_vel_gain=cfg.get("travel_vel_gain"),
            travel_vel_i_gain=cfg.get("travel_vel_i_gain"),
            travel_vel_limit=cfg.get("travel_vel_limit"),
        )
        out = dict(raw or {})
        out["move_mode"] = move_mode
        out["profile_name"] = str(profile_name)
        out["angle_space"] = str(angle_space)
        out["angle_deg"] = float(angle_deg)
        out["gear_ratio"] = float(gear_ratio)
        out["start_turns_motor"] = float(start_turns_motor)
        out["zero_turns_motor"] = float(base_turns_motor)
        out["target_turns_motor"] = float(target_turns_motor)
        out["speed_scale"] = None if speed_scale is None else float(speed_scale)
        out["effective_command_vel_turns_s"] = float(cfg.get("command_vel_turns_s", 0.30))
        out["effective_command_dt"] = float(cfg.get("command_dt", 0.01))
        out["effective_handoff_window_turns"] = float(cfg.get("handoff_window_turns", 0.10))
        out["effective_travel_pos_gain"] = (
            None if cfg.get("travel_pos_gain") is None else float(cfg.get("travel_pos_gain"))
        )
        out["effective_travel_vel_limit"] = float(cfg.get("travel_vel_limit", cfg.get("vel_limit", 1.0)))
        out["travel_diagnostics"] = _travel_diagnostics_from_move(
            raw,
            move_mode=move_mode,
            angle_space=space,
            gear_ratio=float(gear_ratio),
        )
        return out
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
    move_mode = str(cfg.get("move_mode") or "trap_strict").strip().lower()
    if move_mode != "trap_strict":
        raise RuntimeError(
            f"Live follow is not supported for profile '{profile_name}'. "
            "Use one-shot directional direct moves instead."
        )
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
        "output_sensor": _output_sensor_payload(
            axis_motor_turns=(
                None if snapshot.get("pos_est") is None else float(snapshot.get("pos_est"))
            ),
            gear_ratio=float(DEFAULT_GEAR_RATIO),
        ),
        "available_profiles": _continuous_profiles(),
        "available_profile_details": _continuous_profile_records(),
    }


def _status_error_name_map(status: dict[str, Any] | None) -> dict[str, list[str]]:
    report = dict(((status or {}).get("diagnosis") or {}).get("report") or {})
    return {
        "axis": [str(item) for item in (report.get("axis_err_names") or []) if str(item).strip()],
        "motor": [str(item) for item in (report.get("motor_err_names") or []) if str(item).strip()],
        "encoder": [str(item) for item in (report.get("enc_err_names") or []) if str(item).strip()],
        "controller": [str(item) for item in (report.get("ctrl_err_names") or []) if str(item).strip()],
    }


def _status_error_summary(status: dict[str, Any] | None) -> str | None:
    parts: list[str] = []
    names = _status_error_name_map(status)
    if names["axis"]:
        parts.append(f"axis: {', '.join(names['axis'])}")
    if names["motor"]:
        parts.append(f"motor: {', '.join(names['motor'])}")
    if names["encoder"]:
        parts.append(f"encoder: {', '.join(names['encoder'])}")
    if names["controller"]:
        parts.append(f"controller: {', '.join(names['controller'])}")
    if parts:
        return " | ".join(parts)
    snapshot = dict((status or {}).get("snapshot") or {})
    if any(int(snapshot.get(key) or 0) != 0 for key in ("axis_err", "motor_err", "enc_err", "ctrl_err")):
        return (
            f"axis={int(snapshot.get('axis_err') or 0)}, "
            f"motor={int(snapshot.get('motor_err') or 0)}, "
            f"encoder={int(snapshot.get('enc_err') or 0)}, "
            f"controller={int(snapshot.get('ctrl_err') or 0)}"
        )
    return None


def _status_has_encoder_mismatch(status: dict[str, Any] | None) -> bool:
    names = _status_error_name_map(status)
    combined = set(names["axis"] + names["encoder"])
    return bool(
        "AXIS_ERROR_ENCODER_FAILED" in combined
        or "ENCODER_ERROR_CPR_POLEPAIRS_MISMATCH" in combined
    )


def _status_readiness_view(status: dict[str, Any] | None) -> dict[str, Any]:
    snapshot = dict((status or {}).get("snapshot") or {})
    capabilities = dict((status or {}).get("capabilities") or {})
    return {
        "state": int(snapshot.get("state") or 0),
        "startup_ready": bool(capabilities.get("startup_ready")),
        "idle": bool(capabilities.get("idle")),
        "armed": bool(capabilities.get("armed")),
        "motion_active": bool(capabilities.get("motion_active")),
        "has_latched_errors": bool(capabilities.get("has_latched_errors")),
        "enc_ready": bool(snapshot.get("enc_ready")),
        "index_found": bool(snapshot.get("enc_index_found")),
        "pos_est": _clean_json(snapshot.get("pos_est")),
        "error_summary": _status_error_summary(status),
    }


def _status_make_ready_blocker(status: dict[str, Any] | None) -> str | None:
    snapshot = dict((status or {}).get("snapshot") or {})
    capabilities = dict((status or {}).get("capabilities") or {})
    if bool(capabilities.get("motion_active")):
        return "motion_active"
    if bool(capabilities.get("has_latched_errors")):
        return "latched_errors"
    if not bool(capabilities.get("startup_ready")):
        return "startup_not_ready"
    if not bool(snapshot.get("enc_ready")):
        return "encoder_not_ready"
    if not bool(capabilities.get("idle")):
        return "not_idle"
    return None


def _preferred_mks_startup_profile_name() -> str:
    profiles = _continuous_profile_catalog()
    if DEFAULT_MKS_STARTUP_PROFILE in profiles:
        return DEFAULT_MKS_STARTUP_PROFILE
    mounted = next((name for name in profiles if str(name).startswith("mks_mounted_")), None)
    if mounted:
        return str(mounted)
    any_mks = next((name for name in profiles if str(name).startswith("mks_")), None)
    return "" if any_mks is None else str(any_mks)


def _repair_startup_from_profile(axis: Any, *, profile_name: str) -> dict[str, Any]:
    cfg = _load_continuous_move_kwargs(profile_name=profile_name)
    baseline = apply_runtime_baseline(
        axis=axis,
        preset=str(cfg.get("candidate_preset") or profile_name),
        reuse_existing_calibration=False,
        pole_pairs=cfg.get("pole_pairs"),
        baseline_current_lim=float(cfg.get("current_lim", 6.0)),
        current_lim=float(cfg.get("current_lim", 6.0)),
        calibration_current=cfg.get("calibration_current"),
        encoder_offset_calibration_current=cfg.get("encoder_offset_calibration_current"),
        pos_gain=cfg.get("pos_gain"),
        vel_gain=cfg.get("vel_gain"),
        vel_i_gain=cfg.get("vel_i_gain"),
        vel_limit=cfg.get("vel_limit"),
        overspeed_error=cfg.get("enable_overspeed_error"),
        vel_limit_tolerance=cfg.get("vel_limit_tolerance"),
    )
    return {
        "profile_name": str(profile_name),
        "candidate_preset": str(cfg.get("candidate_preset") or profile_name),
        "baseline": _clean_json(baseline),
    }


def _choose_make_ready_repair_profile(
    *,
    requested_profile_name: str,
    fallback_profile_name: str,
    seen_encoder_mismatch: bool,
) -> tuple[str | None, str]:
    profiles = _continuous_profile_catalog()
    requested = str(requested_profile_name or "").strip()
    fallback = str(fallback_profile_name or "").strip()

    if requested.startswith("mks_") and requested in profiles:
        return requested, "requested_mks_profile"
    if seen_encoder_mismatch and fallback.startswith("mks_") and fallback in profiles:
        return fallback, "fallback_after_encoder_mismatch"
    if seen_encoder_mismatch:
        preferred = _preferred_mks_startup_profile_name()
        if preferred:
            return preferred, "preferred_mks_fallback_after_encoder_mismatch"
    return None, "generic_startup"


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
        "output_sensor": _output_sensor_payload(
            axis_motor_turns=(
                None if snapshot.get("pos_est") is None else float(snapshot.get("pos_est"))
            ),
            gear_ratio=float(DEFAULT_GEAR_RATIO),
        ),
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
    fallback_output_sensor = _output_sensor_payload(gear_ratio=float(DEFAULT_GEAR_RATIO))
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
        "output_sensor": fallback_output_sensor,
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
        out["output_sensor"] = status.get("output_sensor")
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


def _handle_repair_startup_from_profile(axis: Any, args: argparse.Namespace) -> tuple[str, dict[str, Any], str, dict[str, Any] | None]:
    profile_name = str(args.profile_name or "").strip()
    if not profile_name:
        raise ValueError("--profile-name is required for repair-startup-from-profile")
    if not profile_name.startswith("mks_"):
        raise RuntimeError(
            f"repair-startup-from-profile only supports the MKS profile family right now; got '{profile_name}'"
        )

    repair = _repair_startup_from_profile(axis, profile_name=profile_name)
    status = _status_bundle(axis, kv_est=args.kv_est, line_line_r_ohm=args.line_line_r_ohm)
    startup_ready = bool((status.get("capabilities") or {}).get("startup_ready"))
    message = (
        f"Profile startup repair completed for {profile_name}"
        if startup_ready else
        f"Profile startup repair did not reach a move-ready state for {profile_name}"
    )
    return "repair-startup-from-profile", status, message, {
        "profile_name": profile_name,
        "startup_ready": startup_ready,
        "candidate_preset": repair.get("candidate_preset"),
        "baseline": repair.get("baseline"),
    }


def _handle_make_ready(axis: Any, args: argparse.Namespace) -> tuple[str, dict[str, Any], str, dict[str, Any] | None]:
    requested_profile_name = str(args.profile_name or "").strip()
    fallback_profile_name = str(getattr(args, "mks_fallback_profile_name", "") or "").strip()
    max_passes = max(1, min(8, int(getattr(args, "repair_passes", 5) or 5)))
    timeout_s = 30.0 if args.timeout_s is None else float(args.timeout_s)
    seen_encoder_mismatch = False
    attempts: list[dict[str, Any]] = []
    repair_profile_used: str | None = None
    repair_reason_used: str | None = None
    final_status: dict[str, Any] | None = None

    for iteration in range(1, max_passes + 1):
        before = _status_bundle(axis, kv_est=args.kv_est, line_line_r_ohm=args.line_line_r_ohm)
        final_status = before
        if _status_has_encoder_mismatch(before):
            seen_encoder_mismatch = True
        blocker = _status_make_ready_blocker(before)
        step: dict[str, Any] = {
            "iteration": int(iteration),
            "before": _status_readiness_view(before),
            "blocker": blocker,
        }
        if blocker is None:
            step["action"] = "ready"
            attempts.append(_clean_json(step))
            break
        if blocker == "motion_active":
            step["action"] = "blocked_motion_active"
            attempts.append(_clean_json(step))
            break

        if blocker == "latched_errors":
            common.clear_errors_all(axis=axis, settle_s=float(args.settle_s))
            step["action"] = "clear-errors"
        elif blocker in {"startup_not_ready", "encoder_not_ready"}:
            repair_profile_name, repair_reason = _choose_make_ready_repair_profile(
                requested_profile_name=requested_profile_name,
                fallback_profile_name=fallback_profile_name,
                seen_encoder_mismatch=seen_encoder_mismatch,
            )
            step["repair_reason"] = str(repair_reason)
            if repair_profile_name:
                repair = _repair_startup_from_profile(axis, profile_name=repair_profile_name)
                repair_profile_used = str(repair_profile_name)
                repair_reason_used = str(repair_reason)
                step["action"] = "repair-startup-from-profile"
                step["repair_profile_name"] = str(repair_profile_name)
                step["candidate_preset"] = repair.get("candidate_preset")
            else:
                axis.requested_state = common.AXIS_STATE_IDLE
                common.clear_errors_all(axis=axis, settle_s=float(args.settle_s))
                axis.requested_state = common.AXIS_STATE_FULL_CALIBRATION_SEQUENCE
                wait_ok = bool(common.wait_idle(axis, timeout_s=float(timeout_s)))
                step["action"] = "startup"
                step["wait_idle_ok"] = bool(wait_ok)
                step["timeout_s"] = float(timeout_s)
        elif blocker == "not_idle":
            axis.requested_state = common.AXIS_STATE_IDLE
            time.sleep(max(0.05, float(args.settle_s)))
            step["action"] = "idle"
        else:
            step["action"] = "no-op"

        after = _status_bundle(axis, kv_est=args.kv_est, line_line_r_ohm=args.line_line_r_ohm)
        final_status = after
        if _status_has_encoder_mismatch(after):
            seen_encoder_mismatch = True
        step["after"] = _status_readiness_view(after)
        step["resolved"] = (_status_make_ready_blocker(after) is None)
        attempts.append(_clean_json(step))
        if step["resolved"]:
            break

    if final_status is None:
        final_status = _status_bundle(axis, kv_est=args.kv_est, line_line_r_ohm=args.line_line_r_ohm)

    final_blocker = _status_make_ready_blocker(final_status)
    final_ready = (final_blocker is None)
    final_error_summary = _status_error_summary(final_status)
    final_view = _status_readiness_view(final_status)
    if final_ready:
        message = "Axis is ready, idle, and clear of latched errors"
    elif final_blocker == "motion_active":
        message = "Make-ready is blocked by an active background move"
    elif final_error_summary:
        message = f"Make-ready stopped with {final_error_summary}"
    else:
        message = f"Make-ready stopped with blocker: {str(final_blocker or 'unknown')}"

    return "make-ready", final_status, message, {
        "requested_profile_name": requested_profile_name,
        "fallback_profile_name": fallback_profile_name,
        "repair_profile_name": repair_profile_used,
        "repair_reason": repair_reason_used,
        "seen_encoder_mismatch": bool(seen_encoder_mismatch),
        "final_ready": bool(final_ready),
        "final_blocker": final_blocker,
        "final_error_summary": final_error_summary,
        "final_state": final_view,
        "attempts": _clean_json(attempts),
    }


def _handle_set_requested_state(axis: Any, args: argparse.Namespace) -> tuple[str, dict[str, Any], str, dict[str, Any] | None]:
    requested_state = int(args.requested_state)
    if bool(getattr(args, "clear_first", False)):
        common.clear_errors_all(axis=axis, settle_s=float(args.settle_s))
    axis.requested_state = int(requested_state)

    wait_idle_ok = None
    wait_state_ok = None
    timeout_s = (None if args.timeout_s is None else float(args.timeout_s))
    if bool(getattr(args, "wait_idle", False)) and timeout_s is not None:
        wait_idle_ok = bool(common.wait_idle(axis, timeout_s=float(timeout_s)))
    elif bool(getattr(args, "wait_state", False)) and timeout_s is not None:
        wait_state_ok = bool(
            common.wait_state(
                axis,
                int(requested_state),
                timeout_s=float(timeout_s),
                poll_s=0.05,
                feed_watchdog=False,
            )
        )

    status = _status_bundle(axis, kv_est=args.kv_est, line_line_r_ohm=args.line_line_r_ohm)
    message = f"Requested axis state {int(requested_state)}"
    result = {
        "requested_state": int(requested_state),
        "wait_idle_ok": wait_idle_ok,
        "wait_state_ok": wait_state_ok,
        "timeout_s": timeout_s,
        "state_after": int(getattr(axis, "current_state", 0) or 0),
    }
    return "set-requested-state", status, message, result


def _handle_command_position(axis: Any, args: argparse.Namespace) -> tuple[str, dict[str, Any], str, dict[str, Any] | None]:
    turns = float(args.turns)
    relative = bool(getattr(args, "relative", False))
    timeout_s = (None if args.timeout_s is None else float(args.timeout_s))
    release_after = bool(getattr(args, "release_after_command", False))
    if release_after and timeout_s is None:
        raise ValueError("--timeout-s is required when --release-after-command is used")

    clear_first = bool(getattr(args, "clear_first", False))
    if clear_first:
        common.clear_errors_all(axis=axis, settle_s=float(args.settle_s))
    if not common.ensure_closed_loop(axis, timeout_s=3.0, clear_first=False, pre_sync=True, retries=2):
        raise RuntimeError("Failed to enter CLOSED_LOOP_CONTROL before direct position command")

    axis.controller.config.control_mode = common.CONTROL_MODE_POSITION_CONTROL
    axis.controller.config.input_mode = common.INPUT_MODE_PASSTHROUGH
    common.sync_pos_setpoint(axis, settle_s=0.05, retries=3, verbose=False)

    start_pos = float(getattr(axis.encoder, "pos_estimate", 0.0))
    target_turns = (start_pos + float(turns)) if relative else float(turns)
    axis.controller.input_pos = float(target_turns)

    target_tol = max(1e-4, float(getattr(args, "target_tolerance_turns", 0.01) or 0.01))
    vel_tol = max(1e-4, float(getattr(args, "target_vel_tolerance_turns_s", 0.20) or 0.20))
    reached = None
    if timeout_s is not None:
        deadline = time.time() + max(0.05, float(timeout_s))
        reached = False
        while time.time() < deadline:
            pos = float(getattr(axis.encoder, "pos_estimate", 0.0))
            vel = abs(float(getattr(axis.encoder, "vel_estimate", 0.0)))
            if abs(float(target_turns) - float(pos)) <= float(target_tol) and vel <= float(vel_tol):
                reached = True
                break
            time.sleep(0.02)
        if release_after:
            axis.requested_state = common.AXIS_STATE_IDLE
            time.sleep(max(0.02, float(args.settle_s)))

    status = _status_bundle(axis, kv_est=args.kv_est, line_line_r_ohm=args.line_line_r_ohm)
    end_pos = (status.get("snapshot") or {}).get("pos_est")
    result = {
        "mode": "direct_position",
        "relative": bool(relative),
        "requested_turns": float(turns),
        "target_turns": float(target_turns),
        "start_pos_turns": float(start_pos),
        "end_pos_turns": _clean_json(end_pos),
        "delta_turns": (None if end_pos is None else float(end_pos) - float(start_pos)),
        "timeout_s": timeout_s,
        "target_tolerance_turns": float(target_tol),
        "target_vel_tolerance_turns_s": float(vel_tol),
        "release_after_command": bool(release_after),
        "reached_target": reached,
    }
    return "command-position", status, "Direct position command sent", result


def _axis_output_sensor_sample(axis: Any, *, gear_ratio: float) -> dict[str, Any]:
    try:
        motor_turns = float(getattr(axis.encoder, "pos_estimate", 0.0))
    except Exception:
        motor_turns = None
    payload = _output_sensor_payload(axis_motor_turns=motor_turns, gear_ratio=float(gear_ratio))
    return dict(payload or {})


def _axis_velocity_run_sample(axis: Any, *, gear_ratio: float) -> dict[str, Any]:
    sample = {
        "motor_turns": _clean_json(getattr(axis.encoder, "pos_estimate", None)),
        "motor_vel_turns_s": _clean_json(getattr(axis.encoder, "vel_estimate", None)),
    }
    sensor = _axis_output_sensor_sample(axis, gear_ratio=float(gear_ratio))
    sample["output_sensor"] = sensor
    sample["output_turns"] = sensor.get("output_turns")
    sample["output_vel_turns_s"] = sensor.get("output_vel_turns_s")
    sample["lag_output_turns"] = sensor.get("compliance_lag_output_turns")
    return sample


def _velocity_run_result_from_samples(
    *,
    mode: str,
    turns_per_second: float,
    duration_s: float | None,
    release_after: bool,
    start_sample: dict[str, Any],
    end_sample: dict[str, Any],
    peak_motor_vel_turns_s: float,
    peak_output_vel_turns_s: float,
    furthest_output_delta_turns: float | None = None,
    max_abs_output_delta_turns: float | None = None,
    extra: dict[str, Any] | None = None,
) -> dict[str, Any]:
    start_motor_turns = start_sample.get("motor_turns")
    end_motor_turns = end_sample.get("motor_turns")
    start_output_turns = start_sample.get("output_turns")
    end_output_turns = end_sample.get("output_turns")
    start_output_vel = start_sample.get("output_vel_turns_s")
    end_output_vel = end_sample.get("output_vel_turns_s")
    start_lag = start_sample.get("lag_output_turns")
    end_lag = end_sample.get("lag_output_turns")
    result = {
        "mode": str(mode),
        "turns_per_second": float(turns_per_second),
        "duration_s": (None if duration_s is None else float(duration_s)),
        "release_after_command": bool(release_after),
        "start_pos_turns": _clean_json(start_motor_turns),
        "end_pos_turns": _clean_json(end_motor_turns),
        "delta_turns": (
            None
            if start_motor_turns is None or end_motor_turns is None
            else float(end_motor_turns) - float(start_motor_turns)
        ),
        "peak_vel_turns_s": float(peak_motor_vel_turns_s),
        "completed_duration": (duration_s is not None),
        "start_output_turns": _clean_json(start_output_turns),
        "end_output_turns": _clean_json(end_output_turns),
        "output_delta_turns": (
            None
            if start_output_turns is None or end_output_turns is None
            else float(end_output_turns) - float(start_output_turns)
        ),
        "start_output_vel_turns_s": _clean_json(start_output_vel),
        "end_output_vel_turns_s": _clean_json(end_output_vel),
        "peak_output_vel_turns_s": float(peak_output_vel_turns_s),
        "furthest_output_delta_turns": _clean_json(furthest_output_delta_turns),
        "max_abs_output_delta_turns": _clean_json(max_abs_output_delta_turns),
        "start_lag_output_turns": _clean_json(start_lag),
        "end_lag_output_turns": _clean_json(end_lag),
        "lag_delta_output_turns": (
            None
            if start_lag is None or end_lag is None
            else float(end_lag) - float(start_lag)
        ),
    }
    if extra:
        result.update(_clean_json(extra))
    return result


def _handle_command_velocity(axis: Any, args: argparse.Namespace) -> tuple[str, dict[str, Any], str, dict[str, Any] | None]:
    turns_per_second = float(args.turns_per_second)
    duration_s = (None if args.duration_s is None else float(args.duration_s))
    release_after = bool(getattr(args, "release_after_command", False))
    if release_after and duration_s is None:
        raise ValueError("--duration-s is required when --release-after-command is used")
    gear_ratio = float(getattr(args, "gear_ratio", DEFAULT_GEAR_RATIO) or DEFAULT_GEAR_RATIO)

    clear_first = bool(getattr(args, "clear_first", False))
    if clear_first:
        common.clear_errors_all(axis=axis, settle_s=float(args.settle_s))
    if not common.ensure_closed_loop(axis, timeout_s=3.0, clear_first=False, pre_sync=False, retries=2):
        raise RuntimeError("Failed to enter CLOSED_LOOP_CONTROL before direct velocity command")

    prev_control = int(getattr(axis.controller.config, "control_mode", common.CONTROL_MODE_POSITION_CONTROL))
    prev_input = int(getattr(axis.controller.config, "input_mode", common.INPUT_MODE_PASSTHROUGH))
    vel_control_mode = int(getattr(common, "CONTROL_MODE_VELOCITY_CONTROL", 2))

    start_sample = _axis_velocity_run_sample(axis, gear_ratio=gear_ratio)
    peak_vel = 0.0
    peak_output_vel = 0.0
    furthest_output_delta_turns = None
    max_abs_output_delta_turns = 0.0
    start_output_turns = start_sample.get("output_turns")
    try:
        axis.controller.config.control_mode = vel_control_mode
        axis.controller.config.input_mode = common.INPUT_MODE_PASSTHROUGH
        axis.controller.input_vel = float(turns_per_second)
        if duration_s is not None:
            deadline = time.time() + max(0.01, float(duration_s))
            while time.time() < deadline:
                sample = _axis_velocity_run_sample(axis, gear_ratio=gear_ratio)
                peak_vel = max(peak_vel, abs(float(sample.get("motor_vel_turns_s") or 0.0)))
                peak_output_vel = max(peak_output_vel, abs(float(sample.get("output_vel_turns_s") or 0.0)))
                current_output_turns = sample.get("output_turns")
                if start_output_turns is not None and current_output_turns is not None:
                    output_delta = float(current_output_turns) - float(start_output_turns)
                    if abs(output_delta) >= float(max_abs_output_delta_turns):
                        max_abs_output_delta_turns = abs(output_delta)
                        furthest_output_delta_turns = output_delta
                time.sleep(0.02)
            axis.controller.input_vel = 0.0
            time.sleep(max(0.02, float(args.settle_s)))
            if release_after:
                axis.requested_state = common.AXIS_STATE_IDLE
                time.sleep(max(0.02, float(args.settle_s)))
    finally:
        if duration_s is not None:
            try:
                axis.controller.config.control_mode = int(prev_control)
            except Exception:
                pass
            try:
                axis.controller.config.input_mode = int(prev_input)
            except Exception:
                pass

    end_sample = _axis_velocity_run_sample(axis, gear_ratio=gear_ratio)
    status = _status_bundle(axis, kv_est=args.kv_est, line_line_r_ohm=args.line_line_r_ohm)
    status["output_sensor"] = _axis_output_sensor_sample(axis, gear_ratio=gear_ratio)
    result = _velocity_run_result_from_samples(
        mode="direct_velocity",
        turns_per_second=float(turns_per_second),
        duration_s=duration_s,
        release_after=bool(release_after),
        start_sample=start_sample,
        end_sample=end_sample,
        peak_motor_vel_turns_s=float(peak_vel),
        peak_output_vel_turns_s=float(peak_output_vel),
        furthest_output_delta_turns=_clean_json(furthest_output_delta_turns),
        max_abs_output_delta_turns=float(max_abs_output_delta_turns),
    )
    return "command-velocity", status, "Direct velocity command sent", result


def _handle_command_velocity_assist(axis: Any, args: argparse.Namespace) -> tuple[str, dict[str, Any], str, dict[str, Any] | None]:
    turns_per_second = float(args.turns_per_second)
    duration_s = (None if args.duration_s is None else float(args.duration_s))
    if duration_s is None or duration_s <= 0.0:
        raise ValueError("--duration-s is required for command-velocity-assist")
    release_after = bool(getattr(args, "release_after_command", False))
    gear_ratio = float(getattr(args, "gear_ratio", DEFAULT_GEAR_RATIO) or DEFAULT_GEAR_RATIO)
    breakaway_floor_turns_s = float(getattr(args, "breakaway_floor_turns_s", 0.0) or 0.0)
    kick_max_duration_s = float(getattr(args, "kick_max_duration_s", 0.20) or 0.20)
    breakaway_output_turns = float(getattr(args, "breakaway_output_turns", 0.0015) or 0.0015)
    if abs(turns_per_second) <= 1e-6:
        raise ValueError("--turns-per-second must be nonzero for command-velocity-assist")

    clear_first = bool(getattr(args, "clear_first", False))
    if clear_first:
        common.clear_errors_all(axis=axis, settle_s=float(args.settle_s))
    if not common.ensure_closed_loop(axis, timeout_s=3.0, clear_first=False, pre_sync=False, retries=2):
        raise RuntimeError("Failed to enter CLOSED_LOOP_CONTROL before assisted velocity command")

    prev_control = int(getattr(axis.controller.config, "control_mode", common.CONTROL_MODE_POSITION_CONTROL))
    prev_input = int(getattr(axis.controller.config, "input_mode", common.INPUT_MODE_PASSTHROUGH))
    vel_control_mode = int(getattr(common, "CONTROL_MODE_VELOCITY_CONTROL", 2))

    start_sample = _axis_velocity_run_sample(axis, gear_ratio=gear_ratio)
    start_output_turns = start_sample.get("output_turns")
    target_sign = 1.0 if turns_per_second >= 0.0 else -1.0
    assist_floor = max(abs(turns_per_second), abs(breakaway_floor_turns_s))
    kick_turns_per_second = target_sign * assist_floor
    max_kick_duration = max(0.02, min(float(duration_s), float(kick_max_duration_s)))
    peak_vel = 0.0
    peak_output_vel = 0.0
    furthest_output_delta_turns = None
    max_abs_output_delta_turns = 0.0
    kick_duration_actual = 0.0
    breakaway_detected = False
    breakaway_detected_at_s = None
    sustain_duration_actual = 0.0

    try:
        axis.controller.config.control_mode = vel_control_mode
        axis.controller.config.input_mode = common.INPUT_MODE_PASSTHROUGH

        overall_started = time.time()
        overall_deadline = overall_started + float(duration_s)
        axis.controller.input_vel = float(kick_turns_per_second)
        kick_deadline = min(overall_deadline, overall_started + max_kick_duration)
        while time.time() < kick_deadline:
            sample = _axis_velocity_run_sample(axis, gear_ratio=gear_ratio)
            peak_vel = max(peak_vel, abs(float(sample.get("motor_vel_turns_s") or 0.0)))
            peak_output_vel = max(peak_output_vel, abs(float(sample.get("output_vel_turns_s") or 0.0)))
            current_output_turns = sample.get("output_turns")
            if start_output_turns is not None and current_output_turns is not None:
                output_delta = float(current_output_turns) - float(start_output_turns)
                if abs(output_delta) >= float(max_abs_output_delta_turns):
                    max_abs_output_delta_turns = abs(output_delta)
                    furthest_output_delta_turns = output_delta
                if abs(output_delta) >= float(breakaway_output_turns) and ((output_delta >= 0.0) == (turns_per_second >= 0.0)):
                    breakaway_detected = True
                    breakaway_detected_at_s = max(0.0, time.time() - overall_started)
                    break
            time.sleep(0.02)

        kick_duration_actual = max(0.0, min(float(duration_s), time.time() - overall_started))
        remaining_duration = max(0.0, overall_deadline - time.time())

        if breakaway_detected and remaining_duration > 0.0:
            axis.controller.input_vel = float(turns_per_second)
            sustain_started = time.time()
            while time.time() < overall_deadline:
                sample = _axis_velocity_run_sample(axis, gear_ratio=gear_ratio)
                peak_vel = max(peak_vel, abs(float(sample.get("motor_vel_turns_s") or 0.0)))
                peak_output_vel = max(peak_output_vel, abs(float(sample.get("output_vel_turns_s") or 0.0)))
                current_output_turns = sample.get("output_turns")
                if start_output_turns is not None and current_output_turns is not None:
                    output_delta = float(current_output_turns) - float(start_output_turns)
                    if abs(output_delta) >= float(max_abs_output_delta_turns):
                        max_abs_output_delta_turns = abs(output_delta)
                        furthest_output_delta_turns = output_delta
                time.sleep(0.02)
            sustain_duration_actual = max(0.0, time.time() - sustain_started)

        axis.controller.input_vel = 0.0
        time.sleep(max(0.02, float(args.settle_s)))
        if release_after:
            axis.requested_state = common.AXIS_STATE_IDLE
            time.sleep(max(0.02, float(args.settle_s)))
    finally:
        try:
            axis.controller.config.control_mode = int(prev_control)
        except Exception:
            pass
        try:
            axis.controller.config.input_mode = int(prev_input)
        except Exception:
            pass

    end_sample = _axis_velocity_run_sample(axis, gear_ratio=gear_ratio)
    status = _status_bundle(axis, kv_est=args.kv_est, line_line_r_ohm=args.line_line_r_ohm)
    status["output_sensor"] = _axis_output_sensor_sample(axis, gear_ratio=gear_ratio)
    result = _velocity_run_result_from_samples(
        mode="direct_velocity_assist",
        turns_per_second=float(turns_per_second),
        duration_s=float(duration_s),
        release_after=bool(release_after),
        start_sample=start_sample,
        end_sample=end_sample,
        peak_motor_vel_turns_s=float(peak_vel),
        peak_output_vel_turns_s=float(peak_output_vel),
        furthest_output_delta_turns=_clean_json(furthest_output_delta_turns),
        max_abs_output_delta_turns=float(max_abs_output_delta_turns),
        extra={
            "assist_used": True,
            "assist_floor_turns_s": float(assist_floor),
            "kick_turns_per_second": float(kick_turns_per_second),
            "kick_max_duration_s": float(max_kick_duration),
            "kick_duration_s_actual": float(kick_duration_actual),
            "breakaway_output_turns_threshold": float(breakaway_output_turns),
            "breakaway_detected": bool(breakaway_detected),
            "breakaway_detected_at_s": _clean_json(breakaway_detected_at_s),
            "sustain_duration_s_actual": float(sustain_duration_actual),
        },
    )
    message = "Assisted velocity command completed"
    if not breakaway_detected:
        message = "Assisted velocity command ended without output breakaway"
    return "command-velocity-assist", status, message, result


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
        applied_path = common.set_configured_direction(axis, int(desired_direction))
    except Exception as exc:
        raise RuntimeError(f"Failed to set motor direction: {exc}") from exc

    time.sleep(max(0.02, min(0.10, float(args.settle_s))))
    status = _status_bundle(axis, kv_est=args.kv_est, line_line_r_ohm=args.line_line_r_ohm)
    applied_direction = ((status.get("snapshot") or {}).get("motor_direction"))
    message = f"Motor direction set to {int(desired_direction):+d} (runtime only; not saved)"
    result = {
        "requested_direction": int(desired_direction),
        "applied_direction": (None if applied_direction is None else int(applied_direction)),
        "applied_path": str(applied_path),
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


def _guided_trap_micro_move(axis: Any) -> dict[str, Any]:
    base = float(getattr(axis.encoder, "pos_estimate", 0.0))
    delta = 0.02
    move_cfg = {
        "use_trap_traj": True,
        "timeout_s": 2.0,
        "min_delta_turns": 0.003,
        "settle_s": 0.08,
        "vel_limit": 0.45,
        "vel_limit_tolerance": 1.0,
        "enable_overspeed_error": False,
        "trap_vel": 0.30,
        "trap_acc": 0.20,
        "trap_dec": 0.20,
        "current_lim": 4.0,
        "pos_gain": 6.0,
        "vel_gain": 0.12,
        "vel_i_gain": 0.0,
        "fail_to_idle": True,
        "require_target_reached": True,
        "target_tolerance_turns": 0.01,
        "target_vel_tolerance_turns_s": 0.10,
        "quiet_hold_enable": False,
        "quiet_hold_s": 0.0,
        "quiet_hold_persist": False,
        "quiet_hold_reanchor_err_turns": None,
        "require_start_sync": True,
        "start_sync_tol_turns": 0.01,
        "abort_on_reverse_motion": True,
        "reverse_motion_eps_turns": 0.002,
        "reverse_motion_confirm_samples": 2,
    }
    forward = common.move_to_pos_strict(axis, base + delta, **move_cfg)
    reverse = common.move_to_pos_strict(axis, base, **move_cfg)
    return {
        "ok": True,
        "message": "Trap micro-move passed.",
        "config": _clean_json(move_cfg),
        "forward": {
            "start": float(forward.get("start", base)),
            "target": float(forward.get("target", base + delta)),
            "end": float(forward.get("end", base)),
            "err": float(forward.get("err", 0.0)),
            "duration_s": float(forward.get("duration_s", 0.0)),
            "peak_abs_vel": float(forward.get("peak_abs_vel", 0.0)),
        },
        "reverse": {
            "start": float(reverse.get("start", base + delta)),
            "target": float(reverse.get("target", base)),
            "end": float(reverse.get("end", base)),
            "err": float(reverse.get("err", 0.0)),
            "duration_s": float(reverse.get("duration_s", 0.0)),
            "peak_abs_vel": float(reverse.get("peak_abs_vel", 0.0)),
        },
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

    try:
        trap_probe = _guided_trap_micro_move(axis)
        add_stage(
            "trap_micro_move_contract",
            True,
            "Trap-trajectory micro-move passed.",
            details=trap_probe,
        )
    except Exception as exc:
        add_stage(
            "trap_micro_move_contract",
            False,
            f"Trap-trajectory micro-move failed: {exc}",
            details={"error": str(exc)},
        )
        status = _status_bundle(axis, kv_est=args.kv_est, line_line_r_ohm=args.line_line_r_ohm)
        return "guided-bringup", status, "Guided bring-up failed at trap micro-move", {
            "ok": False,
            "failed_stage": "trap_micro_move_contract",
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
        speed_scale=(None if getattr(args, "speed_scale", None) is None else float(args.speed_scale)),
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
    if str(cfg_a.get("move_mode") or "trap_strict").strip().lower() != "trap_strict":
        raise RuntimeError(
            f"Profile '{profile_name_a}' uses the mounted MKS directional direct path and is not supported in synced trap moves."
        )
    if str(cfg_b.get("move_mode") or "trap_strict").strip().lower() != "trap_strict":
        raise RuntimeError(
            f"Profile '{profile_name_b}' uses the mounted MKS directional direct path and is not supported in synced trap moves."
        )
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
        "set-requested-state",
        "command-position",
        "command-velocity",
        "command-velocity-assist",
        "set-motor-direction",
        "auto-direction-contract",
        "guided-bringup",
        "idle",
        "clear-errors",
        "startup",
        "make-ready",
        "repair-startup-from-profile",
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
    parser.add_argument("--speed-scale", type=float)
    parser.add_argument("--interval-ms", type=int, default=40)
    parser.add_argument("--profile-json")
    parser.add_argument("--direction", type=int)
    parser.add_argument("--cycles", type=int, default=4)
    parser.add_argument("--cmd-delta-turns", type=float, default=0.01)
    parser.add_argument("--persist", action="store_true")
    parser.add_argument("--requested-state", type=int)
    parser.add_argument("--wait-idle", action="store_true")
    parser.add_argument("--wait-state", action="store_true")
    parser.add_argument("--clear-first", action="store_true")
    parser.add_argument("--turns", type=float)
    parser.add_argument("--relative", action="store_true")
    parser.add_argument("--turns-per-second", type=float)
    parser.add_argument("--duration-s", type=float)
    parser.add_argument("--release-after-command", action="store_true")
    parser.add_argument("--breakaway-floor-turns-s", type=float)
    parser.add_argument("--kick-max-duration-s", type=float, default=0.20)
    parser.add_argument("--breakaway-output-turns", type=float, default=0.0015)
    parser.add_argument("--target-tolerance-turns", type=float, default=0.01)
    parser.add_argument("--target-vel-tolerance-turns-s", type=float, default=0.20)
    parser.add_argument("--mks-fallback-profile-name", default=DEFAULT_MKS_STARTUP_PROFILE)
    parser.add_argument("--repair-passes", type=int, default=5)
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
    if args.action == "set-requested-state" and args.requested_state is None:
        raise ValueError("--requested-state is required for set-requested-state")
    if args.action == "command-position" and args.turns is None:
        raise ValueError("--turns is required for command-position")
    if args.action == "command-velocity" and args.turns_per_second is None:
        raise ValueError("--turns-per-second is required for command-velocity")
    if args.action == "command-velocity-assist" and args.turns_per_second is None:
        raise ValueError("--turns-per-second is required for command-velocity-assist")
    if args.action == "profile-config" and not str(args.profile_name or "").strip():
        raise ValueError("--profile-name is required for profile-config")
    if args.action == "repair-startup-from-profile" and not str(args.profile_name or "").strip():
        raise ValueError("--profile-name is required for repair-startup-from-profile")
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
    elif args.action == "set-requested-state":
        action, status, message, result = _handle_set_requested_state(axis, args)
    elif args.action == "command-position":
        action, status, message, result = _handle_command_position(axis, args)
    elif args.action == "command-velocity":
        action, status, message, result = _handle_command_velocity(axis, args)
    elif args.action == "command-velocity-assist":
        action, status, message, result = _handle_command_velocity_assist(axis, args)
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
    elif args.action == "make-ready":
        action, status, message, result = _handle_make_ready(axis, args)
    elif args.action == "repair-startup-from-profile":
        action, status, message, result = _handle_repair_startup_from_profile(axis, args)
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
    elif action == "make-ready":
        ok = bool((result or {}).get("final_ready"))
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

    def _emit_graph_event(self, *, graph_sample: dict[str, Any], status: dict[str, Any] | None = None) -> None:
        payload = {
            "ok": True,
            "action": "stream-graph",
            "request_id": None,
            "timestamp_s": time.time(),
            "device": None,
            "message": None,
            "snapshot": None if status is None else status.get("snapshot"),
            "diagnosis": None if status is None else status.get("diagnosis"),
            "fact_sheet": None if status is None else status.get("fact_sheet"),
            "capabilities": None if status is None else status.get("capabilities"),
            "output_sensor": None if status is None else status.get("output_sensor"),
            "available_profiles": None if status is None else status.get("available_profiles"),
            "available_profile_details": None if status is None else status.get("available_profile_details"),
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
                self._emit_graph_event(
                    graph_sample=_telemetry_result_from_status(status),
                    status=status,
                )
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
            "output_sensor": _output_sensor_payload(
                axis_motor_turns=(
                    None if snapshot.get("pos_est") is None else float(snapshot.get("pos_est"))
                ),
                gear_ratio=float(DEFAULT_GEAR_RATIO),
            ),
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
            self._emit_graph_event(
                graph_sample=_telemetry_result_from_status(status),
                status=status,
            )

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
                speed_scale=(None if getattr(args, "speed_scale", None) is None else float(args.speed_scale)),
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
            latest_result = None
            exc_result = getattr(exc, "result", None)
            if isinstance(exc_result, dict):
                latest_result = {"move": _clean_json(exc_result)}
            with self._motion_lock:
                self._motion_latest_status = status
                self._motion_latest_result = latest_result
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
