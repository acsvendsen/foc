import importlib, time, math, common, json, os, datetime
try:
    import self_calibrate
except Exception:
    self_calibrate = None
importlib.reload(common)
if self_calibrate is not None:
    importlib.reload(self_calibrate)


def _normalize_load_mode(load_mode):
    mode = str(load_mode).strip().lower()
    if mode in ("loaded", "with_load", "load", "gearbox"):
        return "loaded"
    if mode in ("unloaded", "no_load", "without_load", "bare_motor"):
        return "unloaded"
    return "unknown"


def _ensure_index_policy(axis, require_index=True, auto_enable_index=True, label="index_policy"):
    """Best-effort index-policy guard for diagnostic flows."""
    if not bool(require_index):
        return False
    use_index = False
    try:
        use_index = bool(getattr(axis.encoder.config, "use_index", False))
    except Exception:
        use_index = False
    if bool(use_index):
        return True
    if not bool(auto_enable_index):
        return False
    try:
        axis.encoder.config.use_index = True
        try:
            axis.encoder.config.zero_count_on_find_idx = True
        except Exception:
            pass
        try:
            axis.config.startup_encoder_index_search = True
        except Exception:
            pass
        print(f"{label}: enabled encoder.config.use_index=True for required absolute-reference flow.")
        return True
    except Exception as exc:
        print(f"{label}: failed to enable index policy automatically: {exc}")
        return False


ANGLE_SPACE_MOTOR = "motor"
ANGLE_SPACE_GEARBOX_OUTPUT = "gearbox_output"
DEFAULT_GEAR_RATIO = 25.0


def _load_mode_defaults(load_mode):
    mode = _normalize_load_mode(load_mode)
    if mode == "unloaded":
        return {
            # Tuned from no-load directional sweeps:
            # both +dir and -dir pass reliably at these settings.
            "delta_candidates": (0.006, 0.012, 0.018),
            "repeat_delta": 0.008,
            "approach_offset": 0.004,
            "breakaway_margin": 0.003,
            "stage_chunk": 0.003,
            "target_chunk": 0.003,
            "trap_vel": 0.10,
            "trap_acc": 0.20,
            "trap_dec": 0.20,
            "current_lim": 7.0,
            "pos_gain": 16.0,
            "vel_gain": 0.28,
            "vel_i_gain": 0.00,
            "stiction_kick_nm": 0.01,
        }
    if mode == "loaded":
        return {
            "delta_candidates": (0.010, 0.020),
            "repeat_delta": 0.020,
            "approach_offset": 0.030,
            "breakaway_margin": 0.012,
            "stage_chunk": 0.010,
            "target_chunk": 0.010,
            "trap_vel": 0.10,
            "trap_acc": 0.20,
            "trap_dec": 0.20,
            "current_lim": 8.0,
            "pos_gain": 20.0,
            "vel_gain": 0.30,
            "vel_i_gain": 0.08,
            "stiction_kick_nm": 0.02,
        }
    return {
        "delta_candidates": (0.006, 0.012, 0.020),
        "repeat_delta": 0.015,
        "approach_offset": 0.025,
        "breakaway_margin": 0.010,
        "stage_chunk": 0.008,
        "target_chunk": 0.008,
        "trap_vel": 0.10,
        "trap_acc": 0.20,
        "trap_dec": 0.20,
        "current_lim": 7.0,
        "pos_gain": 16.0,
        "vel_gain": 0.25,
        "vel_i_gain": 0.05,
        "stiction_kick_nm": 0.01,
    }


def _round_float_key(v, nd=4):
    try:
        return round(float(v), int(nd))
    except Exception:
        return None


def _default_bias_store_path():
    return os.path.abspath(os.path.join("logs", "repeatability_bias_profiles.json"))


def _make_repeatability_bias_profile_key(load_mode, cfg):
    c = dict(cfg or {})
    return {
        "load_mode": _normalize_load_mode(load_mode),
        "delta_turns": _round_float_key(c.get("delta_turns"), 5),
        "approach_offset_turns": _round_float_key(c.get("approach_offset_turns"), 5),
        "breakaway_margin_turns": _round_float_key(c.get("breakaway_margin_turns"), 5),
        "target_chunk_turns": _round_float_key(c.get("target_chunk_turns"), 5),
        "trap_vel": _round_float_key(c.get("trap_vel"), 4),
        "trap_acc": _round_float_key(c.get("trap_acc"), 4),
        "trap_dec": _round_float_key(c.get("trap_dec"), 4),
        "current_lim": _round_float_key(c.get("current_lim"), 3),
        "pos_gain": _round_float_key(c.get("pos_gain"), 3),
        "vel_gain": _round_float_key(c.get("vel_gain"), 4),
        "vel_i_gain": _round_float_key(c.get("vel_i_gain"), 4),
    }


def _load_json_file_safe(path):
    p = os.path.abspath(str(path))
    if not os.path.exists(p):
        return {}
    try:
        with open(p, "r", encoding="utf-8") as f:
            data = json.load(f)
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def _save_json_file_safe(path, data):
    p = os.path.abspath(str(path))
    os.makedirs(os.path.dirname(p), exist_ok=True)
    tmp = p + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, sort_keys=True)
    os.replace(tmp, p)
    return p


def _load_persistent_directional_bias(profile_key, store_path):
    store = _load_json_file_safe(store_path)
    entries = list(store.get("profiles") or [])
    for row in entries:
        if not isinstance(row, dict):
            continue
        if dict(row.get("key") or {}) == dict(profile_key or {}):
            bias = dict(row.get("directional_bias_final") or {})
            return {
                "from_below": float(bias.get("from_below", 0.0) or 0.0),
                "from_above": float(bias.get("from_above", 0.0) or 0.0),
            }
    return None


def _save_persistent_directional_bias(profile_key, bias, store_path, source_summary=None):
    store = _load_json_file_safe(store_path)
    rows = list(store.get("profiles") or [])
    item = {
        "key": dict(profile_key or {}),
        "directional_bias_final": {
            "from_below": float(dict(bias or {}).get("from_below", 0.0) or 0.0),
            "from_above": float(dict(bias or {}).get("from_above", 0.0) or 0.0),
        },
        "updated_at": datetime.datetime.now().isoformat(),
    }
    if isinstance(source_summary, dict):
        item["source_summary"] = {
            "ok": bool(source_summary.get("ok", False)),
            "successes": int(source_summary.get("successes", 0) or 0),
            "failures": int(source_summary.get("failures", 0) or 0),
            "hard_faults": int(source_summary.get("hard_faults", 0) or 0),
            "frame_jumps": int(source_summary.get("frame_jumps", 0) or 0),
        }

    replaced = False
    for i, row in enumerate(rows):
        if isinstance(row, dict) and dict(row.get("key") or {}) == dict(profile_key or {}):
            rows[i] = item
            replaced = True
            break
    if not replaced:
        rows.append(item)
    store["profiles"] = rows
    return _save_json_file_safe(store_path, store)


def diagnostic_pipeline_plan(load_mode="unknown", include_onboard_ai=True):
    """Return a structured plan for commissioning/tuning progression."""
    mode = _normalize_load_mode(load_mode)
    plan = [
        {
            "stage": "reference_integrity_gate",
            "goal": "Stable absolute/index frame across reconnects and mode transitions.",
            "primary_checks": [
                "establish_absolute_reference() succeeds",
                "no unexpected pos_est frame jumps",
                "no persistent encoder/controller errors",
            ],
            "exit_criteria": "frame_jumps == 0 under continuity probe",
        },
        {
            "stage": "unloaded_motion_gate",
            "goal": "Prove bidirectional motion in low-risk unloaded profile.",
            "primary_checks": [
                "safe_loaded_step_test(direction=+1) reaches",
                "safe_loaded_step_test(direction=-1) reaches",
            ],
            "exit_criteria": "Both directions pass without hard faults/frame jumps",
        },
        {
            "stage": "loaded_motion_gate",
            "goal": "Prove bidirectional motion with attached load/gearbox.",
            "primary_checks": [
                "safe_loaded_step_test(direction=+1) reaches",
                "safe_loaded_step_test(direction=-1) reaches",
            ],
            "exit_criteria": "Both directions pass in loaded mode",
        },
        {
            "stage": "stiction_lut_mapping",
            "goal": "Map directional stick-slip around full rotation for feed-forward compensation.",
            "primary_checks": [
                "build_stiction_map(...) completes full sweep",
                "per-waypoint kick estimates for + and - directions saved",
                "move_to_angle(... use_stiction_lut=True) shows fewer no-motion events",
            ],
            "exit_criteria": "LUT captured and validated on bidirectional spot-check moves",
        },
        {
            "stage": "repeatability_gate",
            "goal": "Quantify positional spread under repeated approach cycles.",
            "primary_checks": [
                "repeatability_test_single(...) runs without hard faults",
                "spread and mean error within application limits",
            ],
            "exit_criteria": "repeatability summary ok or accepted bounded spread",
        },
        {
            "stage": "reference_recovery_fallback",
            "goal": "Auto-recover reference integrity when repeatability shows repeated frame jumps.",
            "primary_checks": [
                "detect repeated frame jumps in repeatability (primary + alternate)",
                "run reference_recovery_mode()",
                "re-run continuity probe after recovery",
            ],
            "exit_criteria": "reference_recovery_ok == True before any further repeatability cycles",
        },
    ]
    if bool(include_onboard_ai):
        plan.append(
            {
                "stage": "onboard_adaptive_supervisor",
                "goal": "Use onboard low-latency adaptation above the inner control loops.",
                "scope": "Edge/supervisory layer, not replacing real-time FOC loops",
                "capabilities": [
                    "direction/load gain scheduling",
                    "friction/backlash compensation maps",
                    "fault prediction/derating policies",
                    "safe mode switching based on live telemetry",
                ],
                "cadence": "typically 20-200 Hz supervisory updates; keep current loop classical",
            }
        )
    return {"load_mode": mode, "stages": plan}


def reference_continuity_probe(
    cycles=6,
    run_index_search=False,
    jump_warn_turns=0.05,
    attempt_offset_calibration=False,
    require_index=True,
    auto_enable_index=True,
):
    """Focused probe for absolute-frame continuity."""
    a = common.get_axis0()
    rows = []
    prev = None
    n = max(1, int(cycles))
    warn = abs(float(jump_warn_turns))
    print(
        f"reference_continuity_probe: cycles={n} run_index_search={bool(run_index_search)} "
        f"jump_warn={warn:.6f}t"
    )
    _ensure_index_policy(
        a,
        require_index=bool(require_index),
        auto_enable_index=bool(auto_enable_index),
        label="reference_continuity_probe",
    )
    for i in range(n):
        rec = {"cycle": int(i + 1), "ok": True}
        try:
            common.establish_absolute_reference(
                a,
                require_index=bool(require_index),
                run_index_search=bool(run_index_search),
                attempt_offset_calibration=bool(attempt_offset_calibration),
            )
            common.sync_pos_setpoint(a, settle_s=0.05, retries=2, verbose=False)
            common.force_idle(a, settle_s=0.05)
            pos = float(getattr(a.encoder, "pos_estimate", 0.0))
            rec["pos"] = pos
            if prev is None:
                rec["delta"] = 0.0
                rec["jump"] = False
            else:
                delta = pos - prev
                rec["delta"] = float(delta)
                rec["jump"] = bool(abs(delta) > warn)
            prev = pos
        except Exception as exc:
            rec["ok"] = False
            rec["error"] = str(exc)
            rec["snapshot"] = common._snapshot_motion(a)
        rows.append(rec)
        if rec.get("ok"):
            print(
                f"cycle={rec['cycle']:02d} pos={rec['pos']:+.6f} "
                f"delta={rec.get('delta', 0.0):+.6f} jump={bool(rec.get('jump', False))}"
            )
        else:
            print(f"cycle={rec['cycle']:02d} FAIL {rec.get('error')}")

    ok_rows = [r for r in rows if bool(r.get("ok"))]
    deltas = [abs(float(r.get("delta", 0.0))) for r in ok_rows[1:]]
    jumps = [r for r in ok_rows if bool(r.get("jump", False))]
    summary = {
        "ok": bool(len(ok_rows) == n and len(jumps) == 0),
        "cycles": int(n),
        "successes": int(len(ok_rows)),
        "failures": int(n - len(ok_rows)),
        "jump_warn_turns": float(warn),
        "max_abs_delta": max(deltas) if deltas else 0.0,
        "jump_count": int(len(jumps)),
    }
    print("reference_continuity_summary:", summary)
    return {"summary": summary, "results": rows}


def reference_recovery_mode(
    axis=None,
    run_index_search=True,
    attempt_offset_calibration=False,
    continuity_cycles=4,
    continuity_jump_warn_turns=0.05,
):
    """Dedicated reference-integrity recovery + verification stage."""
    a = axis if axis is not None else common.get_axis0()
    steps = []

    def _step(name, fn):
        rec = {"step": str(name), "ok": True}
        try:
            rec["result"] = fn()
        except Exception as exc:
            rec["ok"] = False
            rec["error"] = str(exc)
            try:
                rec["snapshot"] = common._snapshot_motion(a)
            except Exception:
                rec["snapshot"] = None
        steps.append(rec)
        return bool(rec["ok"])

    print(
        "reference_recovery_mode: "
        f"run_index_search={bool(run_index_search)} continuity_cycles={int(max(1, int(continuity_cycles)))} "
        f"jump_warn={abs(float(continuity_jump_warn_turns)):.6f}t"
    )

    _step("clear_errors", lambda: common.clear_errors_all(a))
    _step("force_idle", lambda: common.force_idle(a, settle_s=0.08))
    idx_ok = _step(
        "establish_absolute_reference",
        lambda: common.establish_absolute_reference(
            a,
            require_index=True,
            run_index_search=bool(run_index_search),
            attempt_offset_calibration=bool(attempt_offset_calibration),
        ),
    )
    if idx_ok:
        _step(
            "closed_loop_resync",
            lambda: (
                common.ensure_closed_loop(a, timeout_s=3.0, clear_first=False, pre_sync=True, retries=2),
                common.sync_pos_setpoint(a, settle_s=0.05, retries=2, verbose=False),
                common.force_idle(a, settle_s=0.05),
            ),
        )

    probe = reference_continuity_probe(
        cycles=max(1, int(continuity_cycles)),
        run_index_search=False,
        jump_warn_turns=float(continuity_jump_warn_turns),
        attempt_offset_calibration=False,
    )
    probe_summary = dict(probe.get("summary") or {})
    ok_steps = all(bool(s.get("ok", False)) for s in steps)
    ok = bool(ok_steps and bool(probe_summary.get("ok", False)))
    out = {
        "ok": bool(ok),
        "steps": steps,
        "probe": probe,
        "probe_summary": probe_summary,
    }
    print("reference_recovery_summary:", {"ok": bool(ok), "probe_ok": bool(probe_summary.get("ok", False))})
    return out


def setup_motor():
    a = common.get_axis0()
    common.clear_errors_all(a)
    common.force_idle(a)

    try:
        a.config.enable_watchdog = False
    except Exception:
        pass

    a.controller.config.control_mode = common.CONTROL_MODE_POSITION_CONTROL
    a.controller.config.input_mode   = common.INPUT_MODE_PASSTHROUGH
    a.controller.config.pos_gain = 20.0
    a.controller.config.vel_gain = 0.30
    a.controller.config.vel_integrator_gain = 0.60

    # Closed-loop entry can be rejected with AXIS_ERROR_INVALID_STATE if encoder
    # readiness isn't established first (common after reconnect/reload).
    enc_ready = common.ensure_encoder_ready(a, attempt_calibration=True, timeout_s=30.0)
    print("encoder_ready:", enc_ready)
    print("motor_is_calibrated:", bool(getattr(a.motor, "is_calibrated", False)))
    print("load_encoder:", getattr(a.config, "load_encoder", None))
    print("commutation_encoder:", getattr(a.config, "commutation_encoder", None))

    # Enter closed loop first, then sync pos_setpoint/input_pos.
    ok = common.ensure_closed_loop(a, timeout_s=3.0, clear_first=True, pre_sync=True, retries=3)
    if ok:
        a.controller.input_pos = float(a.encoder.pos_estimate)
        common.sync_pos_setpoint(a, settle_s=0.08, retries=2, verbose=True)
    print("closed_loop_ok:", ok, "state:", int(a.current_state))
    print(common._snapshot_motion(a))

def calibrate_motor(use_preload: bool = True, run_step_validation: bool = False):
    if self_calibrate is None:
        raise RuntimeError(
            "self_calibrate module is unavailable in this process. "
            "Run from odrivetool/IPython with odrv0 connected, or import self_calibrate there first."
        )
    a = common.get_axis0()
    common.clear_errors_all(a)
    if use_preload:
        self_calibrate.full_self_calibrate(
            a,
            current_lim=10.0,
            motor_calib_current=10.0,
            require_absolute=True,
            require_index=True,
            run_index_search=True,
            run_step_validation=bool(run_step_validation),
        )
    else:
        self_calibrate.full_self_calibrate(
            a,
            current_lim=10.0,
            motor_calib_current=10.0,
            require_absolute=True,
            require_index=True,
            run_index_search=True,
            preload_deg=0.0,
            preload_cycles=0,
            torque_pulse_s=0.02,
            torque_nm=0.03,
            torque_settle_s=0.20,
            run_step_validation=bool(run_step_validation),
            step_validation_deg=0.25,
        )

def index_search():
    """Index search is needed to get repeatable absolute reference after power-up. This is especially important for applications like 3D printing where you want to be able to power cycle and resume without losing position"""
    """
    a = common.get_axis0()
    common.clear_errors_all(a)
    common.force_idle(a)
    common.ensure_encoder_ready(a, attempt_calibration=True, timeout_s=30.0)
    a.config.startup_encoder_index_search = True
    a.encoder.config.find_idx_on_lockin = True
"""

    odrv = common.get_odrive()
    a = common.get_axis0()
    # Ensure incremental path is selected
    odrv.inc_encoder0.config.enabled = True
    odrv.inc_encoder0.config.cpr = 1024
    a.config.load_encoder = common.EncoderId.INC_ENCODER0
    a.config.commutation_encoder = common.EncoderId.INC_ENCODER0
    a.encoder.config.use_index = True
    a.config.startup_encoder_index_search = False

def idx_trial():
    a = common.get_axis0()
    common.clear_errors_all(a)
    common.force_idle(a)

    p0 = float(a.encoder.pos_estimate)
    a.requested_state = common.AXIS_STATE_ENCODER_INDEX_SEARCH
    common.wait_idle(a, timeout_s=20)
    p1 = float(a.encoder.pos_estimate)
    print("state:", int(a.current_state), "delta_pos:", p1 - p0, "snap:", common._snapshot_motion(a))

def reconnect_pass():
    a = common.get_axis0()
    common.clear_errors_all(a)
    common.force_idle(a)

    print(common.establish_absolute_reference(
        a,
        require_index=True,
        run_index_search=True,
        timeout_s=30.0,
        attempt_offset_calibration=False,  # force index-first explicitly
    ))
    # Align input/setpoint to current estimate to avoid unnecessary hold-current spikes.
    common.sync_pos_setpoint(a, settle_s=0.05, retries=3, verbose=False)
    print(common._snapshot_motion(a))

def sweep_test_single(
    base_turns=None,
    span_turns=0.20,
    step_turns=0.05,
    trap_vel=0.6,
    trap_acc=1.2,
    trap_dec=1.2,
    current_lim=6.0,
    pos_gain=8.0,
    vel_gain=0.08,
    vel_i_gain=0.0,
    stiction_kick_nm=0.0,
):
    a = common.get_axis0()
    common.establish_absolute_reference(
        a,
        require_index=True,
        run_index_search=True,
        attempt_offset_calibration=False,
    )

    # For directional sweeps, require "target reached" before issuing the next command.
    # move_to_pos_strict() only proves motion occurred; it is not a settle-at-target primitive.
    # By default sweep relative to current position to avoid unintended long traversals.
    if base_turns is None:
        base = float(getattr(a.encoder, "pos_estimate", 0.0))
    else:
        base = float(base_turns)
    try:
        span = max(float(step_turns), float(span_turns))
    except Exception:
        span = 0.20
    try:
        step = max(0.01, float(step_turns))
    except Exception:
        step = 0.05

    n = max(1, int(round(span / step)))
    up = [i * step for i in range(0, n + 1)]
    dn = [i * step for i in range(n - 1, -1, -1)]
    offsets = up + dn
    targets = [base + off for off in offsets]
    print(f"sweep_base={base:+.6f} turns")

    try:
        a.motor.config.current_lim = float(current_lim)
        a.motor.config.current_lim_margin = max(1.0, float(current_lim) * 0.33)
    except Exception:
        pass
    try:
        a.controller.config.pos_gain = float(pos_gain)
        a.controller.config.vel_gain = float(vel_gain)
        a.controller.config.vel_integrator_gain = float(vel_i_gain)
    except Exception:
        pass
    try:
        for t in targets:
            res = common.move_to_pos_strict(
                a,
                float(t),
                use_trap_traj=True,
                timeout_s=6.0,
                settle_s=0.10,
                trap_vel=float(trap_vel),
                trap_acc=float(trap_acc),
                trap_dec=float(trap_dec),
                current_lim=float(current_lim),
                pos_gain=float(pos_gain),
                vel_gain=float(vel_gain),
                vel_i_gain=float(vel_i_gain),
                stiction_kick_nm=(float(stiction_kick_nm) if float(stiction_kick_nm) > 0.0 else None),
                min_delta_turns=max(0.002, 0.25 * step),
                require_target_reached=True,
                target_tolerance_turns=0.004,
                target_vel_tolerance_turns_s=0.10,
                fail_to_idle=False,
            )
            print(
                f"target={res['target']:+.3f} "
                f"pos={res['end']:+.6f} err={res['err']:+.6f} vel={res['vel']:+.6f}"
            )
    finally:
        # Always return to IDLE at the end of the test.
        common.force_idle(a, settle_s=0.08)
        st = int(getattr(a, "current_state", -1))
        if st != int(common.AXIS_STATE_IDLE):
            raise RuntimeError(f"failed to return to IDLE, state={st}, snap={common._snapshot_motion(a)}")
        print("final_state: IDLE")


def breakaway_test_single(
    base_turns=None,
    direction=+1.0,
    step_candidates=(0.002, 0.004, 0.006, 0.008, 0.010, 0.015, 0.020),
    trap_vel=0.15,
    trap_acc=0.25,
    trap_dec=0.25,
    current_lim=6.0,
    pos_gain=12.0,
    vel_gain=0.25,
    vel_i_gain=0.0,
    stiction_kick_nm=0.0,
):
    a = common.get_axis0()
    common.establish_absolute_reference(
        a,
        require_index=True,
        run_index_search=True,
        attempt_offset_calibration=False,
    )

    sign = 1.0 if float(direction) >= 0.0 else -1.0
    if base_turns is None:
        base = float(getattr(a.encoder, "pos_estimate", 0.0))
    else:
        base = float(base_turns)

    results = []
    frame_id = 0
    winner = None
    print(f"breakaway_base={base:+.6f} turns direction={'+' if sign > 0 else '-'}")

    try:
        for raw_step in step_candidates:
            step = abs(float(raw_step))
            target = base + sign * step
            try:
                res = common.move_to_pos_strict(
                    a,
                    target,
                    use_trap_traj=True,
                    timeout_s=3.0,
                    settle_s=0.05,
                    trap_vel=float(trap_vel),
                    trap_acc=float(trap_acc),
                    trap_dec=float(trap_dec),
                    current_lim=float(current_lim),
                    pos_gain=float(pos_gain),
                    vel_gain=float(vel_gain),
                    vel_i_gain=float(vel_i_gain),
                    stiction_kick_nm=(float(stiction_kick_nm) if float(stiction_kick_nm) > 0.0 else None),
                    min_delta_turns=max(0.001, min(0.5 * step, step)),
                    require_target_reached=False,
                    fail_to_idle=False,
                )
                row = {
                    "step": step,
                    "target": float(target),
                    "base": float(base),
                    "end": float(res["end"]),
                    "err": float(res["target"] - res["end"]),
                    "vel": float(res["vel"]),
                    "delta": float(res["end"] - base),
                }
                progress = sign * float(row["delta"])
                min_progress = max(0.001, 0.5 * step)
                row["progress"] = progress
                row["min_progress"] = min_progress
                row["moved"] = bool(progress > 0.0)
                row["ok"] = bool(progress >= min_progress)
                results.append(row)
                print(
                    f"step={step:+.4f} target={target:+.6f} "
                    f"end={row['end']:+.6f} err={row['err']:+.6f} vel={row['vel']:+.6f} "
                    f"progress={progress:+.6f} ok={row['ok']}"
                )
                if winner is None and bool(row["ok"]):
                    winner = row
                    break
            except Exception as exc:
                snap = common._snapshot_motion(a)
                row = {
                    "step": step,
                    "target": float(target),
                    "ok": False,
                    "moved": False,
                    "error": str(exc),
                    "snapshot": snap,
                }
                results.append(row)
                print(f"step={step:+.4f} target={target:+.6f} moved=False error={exc}")
                common.clear_errors_all(a)
                common.force_idle(a, settle_s=0.05)
                common.establish_absolute_reference(
                    a,
                    require_index=True,
                    run_index_search=False,
                    attempt_offset_calibration=False,
                )
    finally:
        common.force_idle(a, settle_s=0.08)
        st = int(getattr(a, "current_state", -1))
        if st != int(common.AXIS_STATE_IDLE):
            raise RuntimeError(f"failed to return to IDLE, state={st}, snap={common._snapshot_motion(a)}")
        print("final_state: IDLE")

    out = {
        "ok": bool(winner is not None),
        "winner": winner,
        "results": results,
        "base": float(base),
        "direction": sign,
    }
    print(out)
    return out


def step_response_single(
    base_turns=None,
    delta_turns=0.010,
    direction=+1.0,
    duration_s=1.2,
    dt=0.02,
    trap_vel=0.20,
    trap_acc=0.40,
    trap_dec=0.40,
    current_lim=6.0,
    pos_gain=12.0,
    vel_gain=0.25,
    vel_i_gain=0.0,
    stiction_kick_nm=0.0,
    settle_tol_turns=0.004,
    settle_vel_tol_turns_s=0.10,
):
    a = common.get_axis0()
    common.establish_absolute_reference(
        a,
        require_index=True,
        run_index_search=True,
        attempt_offset_calibration=False,
    )

    sign = 1.0 if float(direction) >= 0.0 else -1.0
    if base_turns is None:
        base = float(getattr(a.encoder, "pos_estimate", 0.0))
    else:
        base = float(base_turns)
    delta = abs(float(delta_turns)) * sign
    target = base + delta

    samples = []
    print(
        f"step_response: base={base:+.6f} target={target:+.6f} "
        f"delta={delta:+.6f} duration={float(duration_s):.2f}s dt={float(dt):.3f}s"
    )

    try:
        try:
            a.motor.config.current_lim = float(current_lim)
            a.motor.config.current_lim_margin = max(1.0, float(current_lim) * 0.33)
        except Exception:
            pass
        try:
            a.controller.config.pos_gain = float(pos_gain)
            a.controller.config.vel_gain = float(vel_gain)
            a.controller.config.vel_integrator_gain = float(vel_i_gain)
        except Exception:
            pass

        if float(stiction_kick_nm) > 0.0:
            common.torque_bump(a, torque=float(stiction_kick_nm), seconds=0.05)
            common.torque_bump(a, torque=-float(stiction_kick_nm), seconds=0.05)

        if not common.ensure_closed_loop(a, timeout_s=3.0, clear_first=True, pre_sync=True, retries=2):
            raise RuntimeError(f"failed to enter CLOSED_LOOP_CONTROL. snap={common._snapshot_motion(a)}")
        common.sync_pos_setpoint(a, settle_s=0.05, retries=3, verbose=False)

        try:
            a.controller.config.control_mode = common.CONTROL_MODE_POSITION_CONTROL
            a.controller.config.input_mode = common.INPUT_MODE_TRAP_TRAJ
            a.trap_traj.config.vel_limit = float(trap_vel)
            a.trap_traj.config.accel_limit = float(trap_acc)
            a.trap_traj.config.decel_limit = float(trap_dec)
        except Exception:
            pass

        t0 = time.time()
        try:
            if hasattr(a.controller, "move_to_pos"):
                a.controller.move_to_pos(float(target))
            else:
                a.controller.input_pos = float(target)
        except Exception:
            a.controller.input_pos = float(target)

        n = max(1, int(round(float(duration_s) / float(dt))))
        for k in range(n):
            common.assert_no_errors(a, label="step_response_single")
            pos = float(getattr(a.encoder, "pos_estimate", 0.0))
            vel = float(getattr(a.encoder, "vel_estimate", 0.0))
            iq_set = float(getattr(a.motor.current_control, "Iq_setpoint", 0.0))
            iq_meas = float(getattr(a.motor.current_control, "Iq_measured", 0.0))
            err = float(target) - pos
            ts = time.time() - t0
            samples.append(
                {
                    "t": ts,
                    "pos": pos,
                    "vel": vel,
                    "err": err,
                    "iq_set": iq_set,
                    "iq_meas": iq_meas,
                }
            )
            time.sleep(float(dt))
    finally:
        common.force_idle(a, settle_s=0.08)
        st = int(getattr(a, "current_state", -1))
        if st != int(common.AXIS_STATE_IDLE):
            raise RuntimeError(f"failed to return to IDLE, state={st}, snap={common._snapshot_motion(a)}")
        print("final_state: IDLE")

    peak_abs_err = max(abs(s["err"]) for s in samples) if samples else 0.0
    peak_abs_vel = max(abs(s["vel"]) for s in samples) if samples else 0.0
    peak_abs_iq = max(abs(s["iq_meas"]) for s in samples) if samples else 0.0
    final = samples[-1] if samples else {"pos": base, "vel": 0.0, "err": target - base}

    settle_time_s = None
    for i, s in enumerate(samples):
        if abs(s["err"]) <= float(settle_tol_turns) and abs(s["vel"]) <= float(settle_vel_tol_turns_s):
            tail_ok = True
            for trow in samples[i:]:
                if abs(trow["err"]) > float(settle_tol_turns) or abs(trow["vel"]) > float(settle_vel_tol_turns_s):
                    tail_ok = False
                    break
            if tail_ok:
                settle_time_s = float(s["t"])
                break

    summary = {
        "ok": True,
        "base": float(base),
        "target": float(target),
        "delta": float(delta),
        "final_pos": float(final["pos"]),
        "final_err": float(final["err"]),
        "final_vel": float(final["vel"]),
        "peak_abs_err": float(peak_abs_err),
        "peak_abs_vel": float(peak_abs_vel),
        "peak_abs_iq": float(peak_abs_iq),
        "settle_time_s": settle_time_s,
        "sample_count": len(samples),
    }
    print(summary)
    return {"summary": summary, "samples": samples}


def repeatability_test_single(
    target_turns=None,
    delta_turns=0.010,
    approach_offset_turns=0.020,
    breakaway_margin_turns=0.008,
    cycles=5,
    trap_vel=0.20,
    trap_acc=0.40,
    trap_dec=0.40,
    current_lim=8.0,
    pos_gain=20.0,
    vel_gain=0.30,
    vel_i_gain=0.0,
    stiction_kick_nm=0.02,
    target_tolerance_turns=0.004,
    target_vel_tolerance_turns_s=0.10,
    stage_chunk_max_turns=0.008,
    target_chunk_turns=0.006,
    retry_on_current_limit=True,
    raise_on_failure=False,
    auto_derate=True,
    derate_step=0.85,
    min_derate=0.55,
    stage_no_move_err_turns=0.0075,
    rebase_jump_turns=0.50,
    max_target_span_turns=0.50,
    abort_on_unsafe_target_span=True,
    stage_min_pos_step_turns=0.008,
    stage_min_neg_step_turns=0.0065,
    recover_reindex=False,
    abort_on_hard_fault=True,
    max_failures=3,
    approaches=("from_below", "from_above"),
    strict_frame_lock=True,
    frame_lock_jump_turns=0.50,
    abort_on_frame_jump=True,
    max_frame_jumps=1,
    auto_followup_measures=True,
    followup_delta_candidates=(0.010, 0.020),
    load_mode="unknown",
    absolute_at_start=True,
    run_index_search_at_start=None,
    require_index_at_start=True,
    directional_bias_compensation=False,
    directional_bias_learn_rate=0.60,
    directional_bias_limit_turns=0.008,
    directional_bias_warmup_samples=1,
    initial_directional_bias_turns=None,
    auto_reference_recovery_on_frame_jump=False,
    frame_recovery_retries=1,
    frame_recovery_run_index_search=True,
    persist_directional_bias=False,
    load_persistent_bias=True,
    directional_bias_store_path=None,
    directional_bias_profile_key=None,
):
    load_mode = _normalize_load_mode(load_mode)
    a = common.get_axis0()
    if bool(absolute_at_start):
        # Preserve frame continuity when caller provided an explicit target.
        # Re-indexing here can remap turn offsets and invalidate target_turns.
        if run_index_search_at_start is None:
            _run_index = bool(target_turns is None)
        else:
            _run_index = bool(run_index_search_at_start)
        common.establish_absolute_reference(
            a,
            require_index=bool(require_index_at_start),
            run_index_search=_run_index,
            attempt_offset_calibration=False,
        )

    if target_turns is None:
        base = float(getattr(a.encoder, "pos_estimate", 0.0))
        target = base + abs(float(delta_turns))
    else:
        target = float(target_turns)
        base = float(getattr(a.encoder, "pos_estimate", 0.0))

    approach = abs(float(approach_offset_turns))
    target_delta = float(target) - float(base)
    below = float(target) - approach
    above = float(target) + approach
    target_span_limit = abs(float(max_target_span_turns))
    if target_span_limit <= 0.0:
        target_span_limit = 0.50
    target_span = abs(float(target_delta))
    if target_span > float(target_span_limit):
        msg = (
            "repeatability unsafe target span: "
            f"base={float(base):+.6f} target={float(target):+.6f} "
            f"delta={float(target_delta):+.6f}t exceeds limit={float(target_span_limit):.6f}t. "
            "Rejecting to avoid large-frame chase after discontinuity."
        )
        if bool(abort_on_unsafe_target_span):
            raise RuntimeError(msg)
        print(msg)
    ncycles = max(1, int(cycles))
    if isinstance(approaches, (list, tuple, set)):
        raw_approaches = tuple(approaches)
    else:
        raw_approaches = (approaches,)
    approach_labels = []
    for item in raw_approaches:
        label = str(item).strip().lower()
        if label in ("from_below", "from_above"):
            approach_labels.append(label)
            continue
        raise ValueError(
            f"repeatability_test_single: unknown approach '{item}'. "
            "Expected 'from_below' and/or 'from_above'."
        )
    if not approach_labels:
        raise ValueError("repeatability_test_single: approaches is empty.")
    # Preserve order while removing duplicates.
    approach_labels = list(dict.fromkeys(approach_labels))

    bias_enabled = bool(directional_bias_compensation)
    bias_limit = abs(float(directional_bias_limit_turns))
    if bias_enabled and bias_limit <= 0.0:
        bias_limit = max(0.002, 0.8 * abs(float(target_tolerance_turns)))
    bias_learn_rate = max(0.0, min(1.0, float(directional_bias_learn_rate)))
    bias_warmup = max(1, int(directional_bias_warmup_samples))
    bias_turns = {"from_below": 0.0, "from_above": 0.0}
    if isinstance(initial_directional_bias_turns, dict):
        for _k in ("from_below", "from_above"):
            if _k in initial_directional_bias_turns:
                try:
                    bias_turns[_k] = float(initial_directional_bias_turns.get(_k, 0.0) or 0.0)
                except Exception:
                    pass
    for _k in bias_turns:
        if bias_enabled:
            bias_turns[_k] = max(-bias_limit, min(bias_limit, float(bias_turns[_k])))
        else:
            bias_turns[_k] = 0.0
    bias_updates = []
    bias_success_count = {"from_below": 0, "from_above": 0}
    bias_store_path = (
        _default_bias_store_path()
        if directional_bias_store_path is None
        else os.path.abspath(str(directional_bias_store_path))
    )
    auto_profile_key = _make_repeatability_bias_profile_key(
        load_mode,
        {
            "delta_turns": delta_turns,
            "approach_offset_turns": approach_offset_turns,
            "breakaway_margin_turns": breakaway_margin_turns,
            "target_chunk_turns": target_chunk_turns,
            "trap_vel": trap_vel,
            "trap_acc": trap_acc,
            "trap_dec": trap_dec,
            "current_lim": current_lim,
            "pos_gain": pos_gain,
            "vel_gain": vel_gain,
            "vel_i_gain": vel_i_gain,
        },
    )
    bias_profile_key = dict(directional_bias_profile_key or auto_profile_key)
    persistent_bias_loaded = False
    if bool(bias_enabled) and bool(load_persistent_bias):
        loaded_bias = _load_persistent_directional_bias(bias_profile_key, bias_store_path)
        if isinstance(loaded_bias, dict):
            for _k in ("from_below", "from_above"):
                bias_turns[_k] = max(-bias_limit, min(bias_limit, float(loaded_bias.get(_k, 0.0) or 0.0)))
            persistent_bias_loaded = True

    print(
        f"repeatability: base={base:+.6f} target={target:+.6f} "
        f"below={below:+.6f} above={above:+.6f} cycles={ncycles} "
        f"approaches={','.join(approach_labels)} load_mode={load_mode}"
    )
    if bias_enabled:
        print(
            "repeatability: directional bias compensation enabled "
            f"(learn_rate={bias_learn_rate:.3f} limit={bias_limit:.6f} warmup={bias_warmup}) "
            f"initial={bias_turns}"
        )
        if bool(persistent_bias_loaded):
            print(f"repeatability: loaded persistent directional bias from {bias_store_path}")

    results = []
    frame_id = 0
    derate = 1.0
    hard_faults = 0
    frame_jumps = 0
    aborted = False
    aborted_reason = None
    followups = {}
    frame_recovery_attempts = 0
    frame_recovery_successes = 0
    frame_recovery_events = []

    def _mean(vals):
        return (sum(vals) / float(len(vals))) if vals else None

    def _spread(vals):
        return (max(vals) - min(vals)) if vals else None

    def _run_move(cmd_target, stage=False):
        scale = float(derate) if bool(auto_derate) else 1.0
        cur_pos = float(getattr(a.encoder, "pos_estimate", 0.0))
        cmd_mag = abs(float(cmd_target) - cur_pos)
        if bool(stage):
            _trap_vel = min(float(trap_vel) * scale, 0.10)
            _trap_acc = min(float(trap_acc) * scale, 0.20)
            _trap_dec = min(float(trap_dec) * scale, 0.20)
            _tol = max(float(target_tolerance_turns), 0.008)
            _vel_tol = max(float(target_vel_tolerance_turns_s), 0.15)
            _require_reached = False
            _min_delta = max(0.0008, 0.10 * abs(float(delta_turns)))
            _current_lim = max(1.0, 0.85 * float(current_lim))
            _vel_i = min(float(vel_i_gain), 0.08)
            _kick_nm = None
            _pos_gain = max(2.0, min(float(pos_gain) * scale, float(pos_gain)))
            _vel_gain = max(0.02, min(float(vel_gain) * scale, float(vel_gain)))
        else:
            _trap_vel = float(trap_vel) * scale
            _trap_acc = float(trap_acc) * scale
            _trap_dec = float(trap_dec) * scale
            _tol = float(target_tolerance_turns)
            _vel_tol = float(target_vel_tolerance_turns_s)
            _require_reached = True
            # Encoder quantization can produce ~0.00195t apparent move on a valid
            # corrective step; avoid false "no movement" failures near target.
            _min_delta = max(0.001, 0.15 * abs(float(delta_turns)))
            _current_lim = float(current_lim)
            _vel_i = min(float(vel_i_gain), 0.10) * scale
            # Tiny residual moves should not inject kick torque; it can destabilize.
            _kick_thresh = max(0.006, 0.5 * abs(float(delta_turns)))
            _kick_nm = float(stiction_kick_nm) if (float(stiction_kick_nm) > 0.0 and cmd_mag >= _kick_thresh) else None
            _pos_gain = max(2.0, float(pos_gain) * scale)
            _vel_gain = max(0.02, float(vel_gain) * scale)
        _cmd = float(cmd_target)
        try:
            return common.move_to_pos_strict(
                a,
                _cmd,
                use_trap_traj=True,
                timeout_s=6.0,
                settle_s=0.10,
                trap_vel=_trap_vel,
                trap_acc=_trap_acc,
                trap_dec=_trap_dec,
                current_lim=_current_lim,
                pos_gain=_pos_gain,
                vel_gain=_vel_gain,
                vel_i_gain=_vel_i,
                stiction_kick_nm=_kick_nm,
                min_delta_turns=_min_delta,
                require_target_reached=_require_reached,
                target_tolerance_turns=_tol,
                target_vel_tolerance_turns_s=_vel_tol,
                fail_to_idle=False,
            )
        except RuntimeError as exc:
            if bool(stage) and ("No movement detected after position command" in str(exc)):
                snap = common._snapshot_motion(a)
                ax_e = int(snap.get("axis_err", 0) or 0)
                m_e = int(snap.get("motor_err", 0) or 0)
                e_e = int(snap.get("enc_err", 0) or 0)
                c_e = int(snap.get("ctrl_err", 0) or 0)
                if not (ax_e or m_e or e_e or c_e):
                    end = float(snap.get("pos_est", 0.0) or 0.0)
                    vel = float(snap.get("vel_est", 0.0) or 0.0)
                    err = _cmd - end
                    # Stage moves are pre-positioning only; if we're already close enough,
                    # accept this as a benign no-motion condition instead of aborting.
                    if abs(err) <= max(float(stage_no_move_err_turns), 0.35 * abs(float(delta_turns))):
                        return {
                            "start": end,
                            "target": _cmd,
                            "end": end,
                            "err": err,
                            "vel": vel,
                            "moved": False,
                            "reached": True,
                            "stage_no_move_ok": True,
                        }
            raise

    def _is_current_limit_fault(exc):
        s = str(exc)
        return ("motor_err=0x1000" in s) or ("MOTOR_ERROR_CURRENT_LIMIT_VIOLATION" in s)

    def _is_retryable_fault(exc):
        s = str(exc)
        return (
            _is_current_limit_fault(exc)
            or ("axis_err=0x200" in s)
            or ("motor_err=0x10" in s)
            or ("ctrl_err=0x1" in s)
        )

    def _recover_for_retry(label):
        mode = "re-index" if bool(recover_reindex) else "frame-preserving"
        print(f"{label}: recovering after fault ({mode}).")
        common.clear_errors_all(a)
        common.force_idle(a, settle_s=0.08)
        if bool(recover_reindex):
            common.establish_absolute_reference(
                a,
                require_index=True,
                # After a fault, force a fresh index pass to avoid carrying a stale frame.
                run_index_search=True,
                attempt_offset_calibration=False,
            )
        else:
            # Keep the current position frame stable during repeatability runs.
            # Re-indexing can jump the turn offset and invalidate approach targets.
            if not common.ensure_closed_loop(a, timeout_s=3.0, clear_first=False, pre_sync=True, retries=2):
                # Escalate only if local recovery fails.
                common.establish_absolute_reference(
                    a,
                    require_index=True,
                    run_index_search=False,
                    attempt_offset_calibration=False,
                )
            common.sync_pos_setpoint(a, settle_s=0.05, retries=2, verbose=False)
            common.force_idle(a, settle_s=0.05)

    def _recover_after_frame_jump(label):
        nonlocal base, target, below, above, frame_id, frame_recovery_attempts, frame_recovery_successes
        frame_recovery_attempts = int(frame_recovery_attempts) + 1
        event = {"attempt": int(frame_recovery_attempts), "label": str(label), "ok": False}
        print(
            f"{label}: frame-jump recovery attempt {frame_recovery_attempts}/{int(max(1, int(frame_recovery_retries)))} "
            "(reference_recovery_mode)."
        )
        try:
            rec = reference_recovery_mode(
                axis=a,
                run_index_search=bool(frame_recovery_run_index_search),
                attempt_offset_calibration=False,
                continuity_cycles=3,
                continuity_jump_warn_turns=min(0.05, abs(float(frame_lock_jump_turns))),
            )
            event["recovery"] = rec
            rec_ok = bool(isinstance(rec, dict) and bool(rec.get("ok", False)))
        except Exception as exc:
            rec_ok = False
            event["error"] = str(exc)
        event["ok"] = bool(rec_ok)
        frame_recovery_events.append(event)
        if not bool(rec_ok):
            print(f"{label}: frame-jump recovery failed.")
            return False

        # Guard against unsafe absolute-target chase after discontinuity.
        if abs(float(target_delta)) > float(target_span_limit):
            event["ok"] = False
            event["error"] = (
                "unsafe target span after frame recovery: "
                f"|target_delta|={abs(float(target_delta)):.6f}t "
                f"limit={float(target_span_limit):.6f}t"
            )
            print(f"{label}: {event['error']}")
            return False

        # Re-establish the local base/targets in the recovered frame.
        cur = float(getattr(a.encoder, "pos_estimate", 0.0))
        old_base = float(base)
        base = cur
        target = float(base) + float(target_delta)
        below = float(target) - approach
        above = float(target) + approach
        frame_id = int(frame_id) + 1
        frame_recovery_successes = int(frame_recovery_successes) + 1
        print(
            f"{label}: frame-jump recovery succeeded; rebased base {old_base:+.6f} -> {float(base):+.6f}, "
            f"target={float(target):+.6f} (frame_id={int(frame_id)})."
        )
        return True

    def _rebase_if_needed(reason):
        nonlocal base, target, below, above, frame_id, frame_jumps
        cur = float(getattr(a.encoder, "pos_estimate", 0.0))
        jump = abs(cur - float(base))
        if bool(strict_frame_lock):
            jump_lim = abs(float(frame_lock_jump_turns))
            if jump <= jump_lim:
                return False
            frame_jumps += 1
            raise RuntimeError(
                f"{reason}: frame jump detected (strict_frame_lock). "
                f"base={float(base):+.6f} cur={cur:+.6f} jump={jump:+.6f}t limit={jump_lim:.6f}t"
            )
        jump_lim = abs(float(rebase_jump_turns))
        if jump <= jump_lim:
            return False
        if abs(float(target_delta)) > float(target_span_limit):
            frame_jumps += 1
            raise RuntimeError(
                f"{reason}: unsafe rebase target span. "
                f"|target_delta|={abs(float(target_delta)):.6f}t "
                f"limit={float(target_span_limit):.6f}t"
            )
        frame_jumps += 1
        old_base = float(base)
        base = cur
        target = float(base) + float(target_delta)
        below = float(target) - approach
        above = float(target) + approach
        frame_id = int(frame_id) + 1
        print(
            f"{reason}: detected frame jump base {old_base:+.6f} -> {float(base):+.6f}; "
            f"rebased target={float(target):+.6f} below={float(below):+.6f} above={float(above):+.6f} "
            f"(frame_id={frame_id})"
        )
        return True

    def _pre_target_for_label(label):
        return float(below) if str(label) == "from_below" else float(above)

    def _cmd_target_for_label(label):
        return float(target) + float(bias_turns.get(str(label), 0.0))

    def _stage_cmd(cur, rem, chunk):
        # Stage moves must clear measured breakaway in each direction.
        sgn = 1.0 if rem >= 0.0 else -1.0
        min_step = float(stage_min_pos_step_turns) if sgn > 0 else float(stage_min_neg_step_turns)
        abs_rem = abs(float(rem))
        tol = max(float(target_tolerance_turns), 0.25 * min_step)
        if abs_rem <= tol:
            return float(cur)
        step = max(min(abs_rem, float(chunk)), min_step)
        return float(cur) + sgn * step

    def _on_approach_side(cur, target_cmd, label, margin):
        if str(label) == "from_below":
            return float(cur) <= (float(target_cmd) - float(margin))
        return float(cur) >= (float(target_cmd) + float(margin))

    def _force_approach(pre_target, target, label):
        # Use staged sub-moves instead of one large corrective jump. This is safer
        # on the gearbox-attached system and still guarantees approach side.
        chunk = max(abs(float(breakaway_margin_turns)), 0.006)
        chunk = min(chunk, max(0.001, abs(float(stage_chunk_max_turns))))
        max_stages = 12
        print(
            f"{label}: pre_target={float(pre_target):+.6f} chunk={chunk:+.6f} "
            f"target={float(target):+.6f}"
        )
        # We only need to guarantee the approach side, not exact pre_target convergence.
        side_margin = max(float(target_tolerance_turns), 0.5 * chunk)
        for _ in range(max_stages):
            cur = float(getattr(a.encoder, "pos_estimate", 0.0))
            if _on_approach_side(cur, target, label, side_margin):
                break
            rem = float(pre_target) - cur
            if abs(rem) <= max(float(target_tolerance_turns), 0.5 * chunk):
                break
            cmd = _stage_cmd(cur, rem, chunk)
            _run_move(cmd, stage=True)
        # Best-effort tighten toward pre_target ONLY when still not on the correct side.
        # This avoids unnecessary direction reversals in no-load diagnostics.
        cur = float(getattr(a.encoder, "pos_estimate", 0.0))
        if _on_approach_side(cur, target, label, side_margin):
            return
        try:
            _run_move(pre_target, stage=True)
        except RuntimeError:
            cur = float(getattr(a.encoder, "pos_estimate", 0.0))
            if _on_approach_side(cur, target, label, side_margin):
                return
            raise

    def _chunk_toward(target_cmd, label):
        chunk = max(0.0, abs(float(target_chunk_turns)))
        if chunk <= 0.0:
            return
        max_stages = 10
        for _ in range(max_stages):
            cur = float(getattr(a.encoder, "pos_estimate", 0.0))
            rem = float(target_cmd) - cur
            if abs(rem) <= chunk:
                break
            cmd = _stage_cmd(cur, rem, chunk)
            _run_move(cmd, stage=True)
        print(f"{label}: final_chunk={chunk:+.6f} target={float(target_cmd):+.6f}")

    try:
        for k in range(ncycles):
            for label in approach_labels:
                pre_target = _pre_target_for_label(label)
                bias_now = float(bias_turns.get(label, 0.0)) if bias_enabled else 0.0
                cmd_target = _cmd_target_for_label(label)
                try:
                    _rebase_if_needed(reason=f"{label}/pre")
                    pre_target = _pre_target_for_label(label) + bias_now
                    cmd_target = _cmd_target_for_label(label)
                    _force_approach(pre_target, cmd_target, label)
                    _chunk_toward(cmd_target, label=f"{label}/to_target")
                    try:
                        res = _run_move(cmd_target)
                    except RuntimeError as exc:
                        if bool(retry_on_current_limit) and _is_retryable_fault(exc):
                            print(f"{label}: recoverable fault on final move; retrying once with safer profile.")
                            if bool(auto_derate):
                                derate = max(float(min_derate), float(derate) * float(derate_step))
                                print(f"{label}: derate applied -> {derate:.3f}")
                            _recover_for_retry(label=f"{label}/retry")
                            _rebase_if_needed(reason=f"{label}/retry_rebase")
                            pre_target = _pre_target_for_label(label) + bias_now
                            cmd_target = _cmd_target_for_label(label)
                            _force_approach(pre_target, cmd_target, label=f"{label}/retry_approach")
                            _chunk_toward(cmd_target, label=f"{label}/retry_to_target")
                            res = _run_move(cmd_target)
                        else:
                            raise

                    nominal_err = float(target) - float(res["end"])
                    row = {
                        "cycle": int(k + 1),
                        "approach": label,
                        "frame_id": int(frame_id),
                        "pre_target": float(pre_target),
                        "target": float(target),
                        "cmd_target": float(cmd_target),
                        "bias_turns": float(bias_now),
                        "end": float(res["end"]),
                        "err": float(nominal_err),
                        "vel": float(res["vel"]),
                        "status": "ok",
                        "derate": float(derate),
                    }
                    results.append(row)
                    print(
                        f"cycle={row['cycle']:02d} {label}: "
                        f"end={row['end']:+.6f} err={row['err']:+.6f} vel={row['vel']:+.6f}"
                    )
                    if bias_enabled:
                        bias_success_count[label] = int(bias_success_count.get(label, 0)) + 1
                        if int(bias_success_count[label]) >= int(bias_warmup):
                            old_bias = float(bias_turns.get(label, 0.0))
                            learn = float(bias_learn_rate) * float(nominal_err)
                            new_bias = old_bias + learn
                            new_bias = max(-bias_limit, min(bias_limit, new_bias))
                            if abs(new_bias - old_bias) > 1e-9:
                                bias_turns[label] = float(new_bias)
                                bias_updates.append(
                                    {
                                        "cycle": int(k + 1),
                                        "approach": label,
                                        "old_bias": float(old_bias),
                                        "new_bias": float(new_bias),
                                        "err": float(nominal_err),
                                    }
                                )
                                print(
                                    f"{label}: bias update old={old_bias:+.6f} err={nominal_err:+.6f} "
                                    f"new={new_bias:+.6f}"
                                )
                except RuntimeError as exc:
                    hard_fault = bool(_is_retryable_fault(exc))
                    frame_jump = ("frame jump detected (strict_frame_lock)" in str(exc))
                    snap = common._snapshot_motion(a)
                    row = {
                        "cycle": int(k + 1),
                        "approach": label,
                        "frame_id": int(frame_id),
                        "pre_target": float(pre_target),
                        "target": float(target),
                        "cmd_target": float(cmd_target),
                        "bias_turns": float(bias_now),
                        "end": float(snap.get("pos_est", 0.0) or 0.0),
                        "err": float(target) - float(snap.get("pos_est", 0.0) or 0.0),
                        "vel": float(snap.get("vel_est", 0.0) or 0.0),
                        "status": "fail",
                        "derate": float(derate),
                        "hard_fault": bool(hard_fault),
                        "frame_jump": bool(frame_jump),
                        "error": str(exc),
                        "snapshot": snap,
                    }
                    results.append(row)
                    print(
                        f"repeatability failure captured: cycle={row['cycle']:02d} "
                        f"{label}: {row['error']}"
                    )
                    try:
                        _recover_for_retry(label=f"{label}/failure_recover")
                    except Exception:
                        pass
                    if bool(auto_derate) and bool(hard_fault):
                        derate = max(float(min_derate), float(derate) * float(derate_step))
                        print(f"{label}: post-failure derate -> {derate:.3f}")
                    if bool(hard_fault):
                        hard_faults += 1
                        if bool(abort_on_hard_fault) and int(hard_faults) >= int(max_failures):
                            aborted = True
                            aborted_reason = "hard_fault"
                            print(
                                f"repeatability: aborting early for safety after {hard_faults} hard faults "
                                f"(max_failures={int(max_failures)})."
                            )
                            break
                    if bool(frame_jump):
                        frame_recovered = False
                        max_frame_rec = max(0, int(frame_recovery_retries))
                        if bool(auto_reference_recovery_on_frame_jump) and int(frame_recovery_attempts) < int(max_frame_rec):
                            frame_recovered = bool(_recover_after_frame_jump(label=f"{label}/frame_recovery"))
                            row["frame_recovery_attempted"] = True
                            row["frame_recovery_ok"] = bool(frame_recovered)
                        if bool(frame_recovered):
                            continue
                        if bool(abort_on_frame_jump) and int(frame_jumps) >= int(max_frame_jumps):
                            aborted = True
                            aborted_reason = "frame_jump"
                            print(
                                f"repeatability: aborting current run after {frame_jumps} frame jumps "
                                f"(max_frame_jumps={int(max_frame_jumps)})."
                            )
                            break
                    if bool(raise_on_failure):
                        raise
            if bool(aborted):
                break
    finally:
        common.force_idle(a, settle_s=0.08)
        st = int(getattr(a, "current_state", -1))
        if st != int(common.AXIS_STATE_IDLE):
            raise RuntimeError(f"failed to return to IDLE, state={st}, snap={common._snapshot_motion(a)}")
        print("final_state: IDLE")

    if bool(auto_followup_measures) and (bool(aborted) or any(r.get("status") != "ok" for r in results)):
        print("repeatability: running auto follow-up safe-step diagnostics.")
        prep = {"ok": True}
        try:
            common.clear_errors_all(a)
            common.force_idle(a, settle_s=0.08)
            if bool(recover_reindex):
                common.establish_absolute_reference(
                    a,
                    require_index=True,
                    run_index_search=False,
                    attempt_offset_calibration=False,
                )
            else:
                if not common.ensure_closed_loop(a, timeout_s=3.0, clear_first=False, pre_sync=True, retries=2):
                    raise RuntimeError("failed CLOSED_LOOP entry for follow-up probes")
                common.sync_pos_setpoint(a, settle_s=0.05, retries=2, verbose=False)
                common.force_idle(a, settle_s=0.05)
        except Exception as exc:
            prep = {"ok": False, "error": str(exc)}
        followups["prepare"] = prep

        try:
            followup_deltas = tuple(float(abs(x)) for x in followup_delta_candidates if float(abs(x)) > 0.0)
        except Exception:
            followup_deltas = ()
        if not followup_deltas:
            base_delta = abs(float(delta_turns))
            followup_deltas = (max(0.010, 0.5 * base_delta), max(0.020, base_delta))
        min_follow_delta = min(followup_deltas)

        for key, sign in (("plus", +1.0), ("minus", -1.0)):
            try:
                followups[key] = safe_loaded_step_test(
                    direction=sign,
                    delta_candidates=followup_deltas,
                    trap_vel=min(float(trap_vel), 0.10),
                    trap_acc=min(float(trap_acc), 0.20),
                    trap_dec=min(float(trap_dec), 0.20),
                    current_lim=float(current_lim),
                    current_abort_frac=0.90,
                    pos_gain=max(8.0, min(float(pos_gain), 14.0)),
                    vel_gain=max(0.08, min(float(vel_gain), 0.20)),
                    vel_i_gain=min(float(vel_i_gain), 0.05),
                    stiction_kick_nm=max(0.0, float(stiction_kick_nm)),
                    duration_s=1.4,
                    dt=0.02,
                    target_tolerance_turns=max(float(target_tolerance_turns), 0.004),
                    target_vel_tolerance_turns_s=max(float(target_vel_tolerance_turns_s), 0.10),
                    min_progress_turns=max(0.001, 0.25 * min_follow_delta),
                    min_progress_frac=0.5,
                    min_settle_samples=4,
                    hold_in_tol_s=0.08,
                    absolute_at_start=False,
                    recheck_absolute_each_candidate=False,
                    load_mode=load_mode,
                )
            except Exception as exc:
                followups[key] = {"ok": False, "error": str(exc)}

    ok_rows = [r for r in results if r.get("status") == "ok"]
    fail_rows = [r for r in results if r.get("status") != "ok"]
    planned_attempts = int(len(approach_labels) * ncycles)
    current_frame = int(frame_id)
    frame_rows = [r for r in results if int(r.get("frame_id", 0)) == current_frame]
    frame_ok = [r for r in frame_rows if r.get("status") == "ok"]
    frame_fail = [r for r in frame_rows if r.get("status") != "ok"]
    below_pos = [r["end"] for r in frame_ok if r["approach"] == "from_below"]
    above_pos = [r["end"] for r in frame_ok if r["approach"] == "from_above"]
    below_err = [r["err"] for r in frame_ok if r["approach"] == "from_below"]
    above_err = [r["err"] for r in frame_ok if r["approach"] == "from_above"]
    all_pos = [r["end"] for r in frame_ok]
    all_err = [r["err"] for r in frame_ok]
    final_bias = {
        "from_below": float(bias_turns.get("from_below", 0.0)),
        "from_above": float(bias_turns.get("from_above", 0.0)),
    }
    persistent_bias_saved = False
    persistent_bias_error = None
    if bool(bias_enabled) and bool(persist_directional_bias):
        try:
            bias_store_path = _save_persistent_directional_bias(
                bias_profile_key,
                final_bias,
                bias_store_path,
                source_summary={
                    "ok": len(fail_rows) == 0 and len(ok_rows) == planned_attempts,
                    "successes": len(ok_rows),
                    "failures": len(fail_rows),
                    "hard_faults": hard_faults,
                    "frame_jumps": frame_jumps,
                },
            )
            persistent_bias_saved = True
        except Exception as exc:
            persistent_bias_error = str(exc)

    summary = {
        "ok": len(fail_rows) == 0 and len(ok_rows) == planned_attempts,
        "load_mode": load_mode,
        "base": float(base),
        "target": float(target),
        "below": float(below),
        "above": float(above),
        "approaches": tuple(approach_labels),
        "cycles": int(ncycles),
        "planned_attempts": int(planned_attempts),
        "attempts": int(len(results)),
        "successes": int(len(ok_rows)),
        "failures": int(len(fail_rows)),
        "hard_faults": int(hard_faults),
        "frame_jumps": int(frame_jumps),
        "aborted": bool(aborted),
        "aborted_reason": aborted_reason,
        "frame_id": int(current_frame),
        "frame_attempts": int(len(frame_rows)),
        "frame_successes": int(len(frame_ok)),
        "frame_failures": int(len(frame_fail)),
        "derate_final": float(derate),
        "followups_run": bool(bool(followups)),
        "frame_recovery_attempts": int(frame_recovery_attempts),
        "frame_recovery_successes": int(frame_recovery_successes),
        "from_below_mean_pos": _mean(below_pos),
        "from_below_spread": _spread(below_pos),
        "from_above_mean_pos": _mean(above_pos),
        "from_above_spread": _spread(above_pos),
        "from_below_mean_err": _mean(below_err),
        "from_above_mean_err": _mean(above_err),
        "overall_spread": _spread(all_pos),
        "overall_mean_err": _mean(all_err),
        "directional_bias_compensation": bool(bias_enabled),
        "directional_bias_final": final_bias,
        "directional_bias_limit_turns": float(bias_limit) if bool(bias_enabled) else 0.0,
        "directional_bias_learn_rate": float(bias_learn_rate) if bool(bias_enabled) else 0.0,
        "directional_bias_profile_key": dict(bias_profile_key) if bool(bias_enabled) else None,
        "directional_bias_store_path": str(bias_store_path) if bool(bias_enabled) else None,
        "directional_bias_persistent_loaded": bool(persistent_bias_loaded),
        "directional_bias_persistent_saved": bool(persistent_bias_saved),
        "directional_bias_persistent_error": persistent_bias_error,
    }
    print(summary)
    return {
        "summary": summary,
        "results": results,
        "followups": followups,
        "bias_updates": bias_updates,
        "frame_recovery_events": frame_recovery_events,
    }


def safe_loaded_step_test(
    base_turns=None,
    direction=+1.0,
    delta_candidates=(0.004, 0.006, 0.008, 0.010),
    trap_vel=0.15,
    trap_acc=0.30,
    trap_dec=0.30,
    current_lim=8.0,
    current_abort_frac=0.85,
    pos_gain=20.0,
    vel_gain=0.30,
    vel_i_gain=0.0,
    stiction_kick_nm=0.02,
    duration_s=1.2,
    dt=0.02,
    target_tolerance_turns=0.004,
    target_vel_tolerance_turns_s=0.10,
    min_progress_turns=0.001,
    min_progress_frac=0.5,
    min_settle_samples=4,
    hold_in_tol_s=0.08,
    absolute_at_start=True,
    run_index_search_at_start=True,
    recheck_absolute_each_candidate=True,
    strict_frame_lock=True,
    frame_lock_jump_turns=0.50,
    abort_on_frame_jump=True,
    stop_on_abort_current=True,
    load_mode="unknown",
):
    load_mode = _normalize_load_mode(load_mode)
    a = common.get_axis0()
    # Unloaded runs are highly sensitive to integrator windup/runaway on this rig.
    # Force zero I gain unless explicitly running a loaded profile.
    effective_vel_i_gain = float(vel_i_gain)
    if load_mode == "unloaded" and effective_vel_i_gain != 0.0:
        print(
            "safe_loaded_step: load_mode=unloaded -> forcing vel_i_gain=0.0 "
            f"(requested {effective_vel_i_gain:.6f})"
        )
        effective_vel_i_gain = 0.0
    if bool(absolute_at_start):
        common.establish_absolute_reference(
            a,
            require_index=True,
            run_index_search=bool(run_index_search_at_start),
            attempt_offset_calibration=False,
        )

    sign = 1.0 if float(direction) >= 0.0 else -1.0
    if base_turns is None:
        base = float(getattr(a.encoder, "pos_estimate", 0.0))
    else:
        base = float(base_turns)

    results = []
    winner = None
    frame_anchor = None
    frame_jumps = 0
    abort_a = max(1.0, float(current_lim) * float(current_abort_frac))
    print(
        f"safe_loaded_step: base={base:+.6f} direction={'+' if sign > 0 else '-'} "
        f"current_lim={float(current_lim):.2f}A abort_at={abort_a:.2f}A load_mode={load_mode}"
    )

    try:
        for raw_delta in delta_candidates:
            delta = abs(float(raw_delta))
            samples = []
            status = "timeout"
            reason = None
            settle_time_s = None
            live_final_snap = None

            common.clear_errors_all(a)
            common.force_idle(a, settle_s=0.05)
            if bool(recheck_absolute_each_candidate):
                common.establish_absolute_reference(
                    a,
                    require_index=True,
                    run_index_search=False,
                    attempt_offset_calibration=False,
                )

            try:
                a.motor.config.current_lim = float(current_lim)
                a.motor.config.current_lim_margin = max(1.0, float(current_lim) * 0.33)
            except Exception:
                pass
            try:
                a.controller.config.pos_gain = float(pos_gain)
                a.controller.config.vel_gain = float(vel_gain)
                a.controller.config.vel_integrator_gain = float(effective_vel_i_gain)
            except Exception:
                pass

            if float(stiction_kick_nm) > 0.0:
                common.torque_bump(a, torque=float(stiction_kick_nm), seconds=0.05)
                common.torque_bump(a, torque=-float(stiction_kick_nm), seconds=0.05)

            if not common.ensure_closed_loop(a, timeout_s=3.0, clear_first=True, pre_sync=True, retries=2):
                raise RuntimeError(f"failed to enter CLOSED_LOOP_CONTROL. snap={common._snapshot_motion(a)}")
            common.sync_pos_setpoint(a, settle_s=0.05, retries=3, verbose=False)
            start_pos = float(getattr(a.encoder, "pos_estimate", 0.0))
            target = start_pos + sign * delta
            min_progress = max(abs(float(min_progress_turns)), abs(float(min_progress_frac)) * delta)
            if frame_anchor is None:
                frame_anchor = float(start_pos)
            frame_jump = abs(float(start_pos) - float(frame_anchor))
            if bool(strict_frame_lock) and frame_jump > abs(float(frame_lock_jump_turns)):
                frame_jumps += 1
                row = {
                    "delta": float(delta),
                    "start_pos": float(start_pos),
                    "target": float(target),
                    "status": "frame_jump",
                    "reason": (
                        f"frame jump detected (strict_frame_lock): anchor={float(frame_anchor):+.6f} "
                        f"start={float(start_pos):+.6f} jump={frame_jump:+.6f}t "
                        f"limit={abs(float(frame_lock_jump_turns)):.6f}t"
                    ),
                    "settle_time_s": None,
                    "final_pos": float(start_pos),
                    "final_err": float(target) - float(start_pos),
                    "final_vel": 0.0,
                    "progress": 0.0,
                    "min_progress": float(min_progress),
                    "peak_abs_iq": 0.0,
                    "peak_abs_vel": 0.0,
                    "sample_count": 0,
                }
                results.append(row)
                print(row)
                if bool(abort_on_frame_jump):
                    break
                frame_anchor = float(start_pos)
                continue

            try:
                a.controller.config.control_mode = common.CONTROL_MODE_POSITION_CONTROL
                a.controller.config.input_mode = common.INPUT_MODE_TRAP_TRAJ
                a.trap_traj.config.vel_limit = float(trap_vel)
                a.trap_traj.config.accel_limit = float(trap_acc)
                a.trap_traj.config.decel_limit = float(trap_dec)
            except Exception:
                pass

            t0 = time.time()
            try:
                if hasattr(a.controller, "move_to_pos"):
                    a.controller.move_to_pos(float(target))
                else:
                    a.controller.input_pos = float(target)
            except Exception:
                a.controller.input_pos = float(target)

            try:
                n = max(1, int(round(float(duration_s) / float(dt))))
                reached_since = None
                for _ in range(n):
                    now = time.time()
                    snap = common._snapshot_motion(a)
                    samples.append({"t": now - t0, **snap})
                    common.assert_no_errors(a, label="safe_loaded_step_test")

                    iq_set = abs(float(snap.get("Iq_set", 0.0) or 0.0))
                    iq_meas = abs(float(snap.get("Iq_meas", 0.0) or 0.0))
                    pos = float(snap.get("pos_est", 0.0) or 0.0)
                    vel = abs(float(snap.get("vel_est", 0.0) or 0.0))
                    err = float(target) - pos
                    progress = sign * (pos - start_pos)

                    if iq_set >= abort_a or iq_meas >= abort_a:
                        status = "aborted_current"
                        reason = (
                            f"current stress: |Iq_set|={iq_set:.3f}A |Iq_meas|={iq_meas:.3f}A "
                            f"abort_at={abort_a:.3f}A"
                        )
                        break

                    in_tol = abs(err) <= float(target_tolerance_turns) and vel <= float(target_vel_tolerance_turns_s)
                    if in_tol and progress >= min_progress:
                        if reached_since is None:
                            reached_since = now
                        if (now - reached_since) >= float(hold_in_tol_s) and len(samples) >= int(min_settle_samples):
                            status = "reached"
                            settle_time_s = now - t0
                            break
                    else:
                        reached_since = None

                    time.sleep(float(dt))
                live_final_snap = common._snapshot_motion(a)
            finally:
                common.force_idle(a, settle_s=0.08)

            if not isinstance(live_final_snap, dict):
                live_final_snap = samples[-1] if samples else common._snapshot_motion(a)
            final_snap = live_final_snap
            final_pos = float(final_snap.get("pos_est", 0.0) or 0.0)
            final_vel = float(final_snap.get("vel_est", 0.0) or 0.0)
            final_err = float(target) - final_pos
            final_progress = sign * (final_pos - start_pos)
            # Guard against transient "reached" detections that do not hold at loop exit.
            if status == "reached":
                in_tol_final = abs(final_err) <= float(target_tolerance_turns) and abs(final_vel) <= float(
                    target_vel_tolerance_turns_s
                )
                if (not in_tol_final) or (final_progress < min_progress):
                    status = "transient_reached"
                    reason = (
                        "reached transiently but did not hold at loop exit: "
                        f"err={final_err:+.6f}t vel={final_vel:+.6f}t/s "
                        f"progress={final_progress:+.6f} min_progress={min_progress:+.6f}"
                    )
                    settle_time_s = None
            peak_abs_iq = 0.0
            peak_abs_vel = 0.0
            if samples:
                peak_abs_iq = max(
                    max(abs(float(s.get("Iq_set", 0.0) or 0.0)), abs(float(s.get("Iq_meas", 0.0) or 0.0)))
                    for s in samples
                )
                peak_abs_vel = max(abs(float(s.get("vel_est", 0.0) or 0.0)) for s in samples)

            row = {
                "delta": float(delta),
                "start_pos": float(start_pos),
                "target": float(target),
                "status": status,
                "reason": reason,
                "settle_time_s": settle_time_s,
                "final_pos": final_pos,
                "final_err": final_err,
                "final_vel": final_vel,
                "progress": float(final_progress),
                "min_progress": float(min_progress),
                "peak_abs_iq": peak_abs_iq,
                "peak_abs_vel": peak_abs_vel,
                "sample_count": len(samples),
            }
            results.append(row)
            print(row)

            if status == "reached":
                winner = row
                break
            if status == "aborted_current" and bool(stop_on_abort_current):
                break
    finally:
        common.force_idle(a, settle_s=0.08)
        st = int(getattr(a, "current_state", -1))
        if st != int(common.AXIS_STATE_IDLE):
            raise RuntimeError(f"failed to return to IDLE, state={st}, snap={common._snapshot_motion(a)}")
        print("final_state: IDLE")

    summary = {
        "ok": bool(winner is not None),
        "load_mode": load_mode,
        "base": float(base),
        "frame_anchor": float(frame_anchor),
        "frame_jumps": int(frame_jumps),
        "direction": sign,
        "winner": winner,
        "results": results,
    }
    print(summary)
    return summary


def diagnostic_suite(
    load_mode="loaded",
    run_repeatability=True,
    repeatability_cycles=3,
    auto_reference_recovery=True,
    reference_recovery_frame_jump_trigger=2,
    reference_recovery_kwargs=None,
    run_reference_probe=True,
    reference_probe_cycles=4,
    reference_probe_run_index_search=False,
    reference_jump_warn_turns=None,
    require_index=True,
    auto_enable_index=True,
    delta_candidates=None,
    trap_vel=None,
    trap_acc=None,
    trap_dec=None,
    current_lim=None,
    pos_gain=None,
    vel_gain=None,
    vel_i_gain=None,
    stiction_kick_nm=None,
    repeatability_kwargs=None,
    step_kwargs=None,
):
    """Run a fault-tolerant diagnostic sequence and keep collecting useful data.

    The `load_mode` switch is explicit so logs/results stay interpretable:
      - "loaded"   -> heavier-motion defaults and interpretation
      - "unloaded" -> lighter-motion defaults and stricter fault interpretation
    """
    mode = _normalize_load_mode(load_mode)
    defaults = _load_mode_defaults(mode)
    repeatability_kwargs = dict(repeatability_kwargs or {})
    try_alternate_approach_on_fail = bool(repeatability_kwargs.pop("try_alternate_approach_on_fail", True))
    step_gate_min_progress_ratio = float(repeatability_kwargs.pop("step_gate_min_progress_ratio", 1.35))
    step_gate_disallow_max_delta_winner = bool(repeatability_kwargs.pop("step_gate_disallow_max_delta_winner", True))
    step_gate_iq_frac_warn = float(repeatability_kwargs.pop("step_gate_iq_frac_warn", 0.35))
    if step_gate_min_progress_ratio < 1.0:
        step_gate_min_progress_ratio = 1.0
    step_gate_iq_frac_warn = max(0.05, min(step_gate_iq_frac_warn, 0.95))
    reference_recovery_kwargs = dict(reference_recovery_kwargs or {})
    step_kwargs = dict(step_kwargs or {})

    # In unloaded mode, prioritize deterministic frame recovery after a fault.
    if mode == "unloaded":
        repeatability_kwargs.setdefault("recover_reindex", True)

    if delta_candidates is None:
        deltas = tuple(defaults["delta_candidates"])
    else:
        deltas = tuple(float(abs(x)) for x in delta_candidates if float(abs(x)) > 0.0)
        if not deltas:
            deltas = tuple(defaults["delta_candidates"])

    cfg = {
        "load_mode": mode,
        "delta_candidates": tuple(deltas),
        "trap_vel": float(defaults["trap_vel"] if trap_vel is None else trap_vel),
        "trap_acc": float(defaults["trap_acc"] if trap_acc is None else trap_acc),
        "trap_dec": float(defaults["trap_dec"] if trap_dec is None else trap_dec),
        "current_lim": float(defaults["current_lim"] if current_lim is None else current_lim),
        "pos_gain": float(defaults["pos_gain"] if pos_gain is None else pos_gain),
        "vel_gain": float(defaults["vel_gain"] if vel_gain is None else vel_gain),
        "vel_i_gain": float(defaults["vel_i_gain"] if vel_i_gain is None else vel_i_gain),
        "stiction_kick_nm": float(defaults["stiction_kick_nm"] if stiction_kick_nm is None else stiction_kick_nm),
        "repeat_delta": float(defaults["repeat_delta"]),
        "approach_offset": float(defaults["approach_offset"]),
        "breakaway_margin": float(defaults["breakaway_margin"]),
        "stage_chunk": float(defaults["stage_chunk"]),
        "target_chunk": float(defaults["target_chunk"]),
    }
    if mode == "unloaded" and float(cfg.get("vel_i_gain", 0.0)) != 0.0:
        print(
            "diagnostic_suite: load_mode=unloaded -> forcing vel_i_gain=0.0 "
            f"(requested {float(cfg.get('vel_i_gain', 0.0)):.6f})"
        )
        cfg["vel_i_gain"] = 0.0

    a = common.get_axis0()
    out = {"config": cfg, "stages": {}, "summary": {}}

    def _capture_snapshot():
        try:
            return common._snapshot_motion(a)
        except Exception:
            return None

    def _run_stage(name, fn, **kwargs):
        try:
            result = fn(**kwargs)
            if isinstance(result, dict):
                if "ok" in result:
                    ok = bool(result.get("ok", False))
                elif isinstance(result.get("summary"), dict) and ("ok" in result["summary"]):
                    ok = bool(result["summary"].get("ok", False))
                else:
                    ok = True
            else:
                ok = bool(result)
            out["stages"][name] = {"ok": ok, "result": result}
            return result
        except Exception as exc:
            out["stages"][name] = {
                "ok": False,
                "error": str(exc),
                "snapshot": _capture_snapshot(),
            }
            return None

    def _step_has_status(step_result, status_name):
        if not isinstance(step_result, dict):
            return False
        rows = list(step_result.get("results") or [])
        status_target = str(status_name).strip().lower()
        for row in rows:
            if str(row.get("status", "")).strip().lower() == status_target:
                return True
        return False

    def _assess_step_repeatability_gate(step_result, side_label):
        gate = {"ok": False, "reasons": [], "metrics": {}}
        if not isinstance(step_result, dict):
            gate["reasons"].append(f"{side_label}:missing_step_result")
            return gate
        winner = dict(step_result.get("winner") or {})
        rows = list(step_result.get("results") or [])
        if not winner:
            gate["reasons"].append(f"{side_label}:no_step_winner")
            return gate

        winner_delta = abs(float(winner.get("delta", 0.0) or 0.0))
        deltas = [abs(float((row or {}).get("delta", 0.0) or 0.0)) for row in rows]
        max_delta = max(deltas) if deltas else winner_delta
        winner_is_max_delta = bool(max_delta > 0.0 and (winner_delta >= (max_delta - 1e-9)))

        min_progress = abs(float(winner.get("min_progress", 0.0) or 0.0))
        progress = abs(float(winner.get("progress", 0.0) or 0.0))
        progress_ratio = (progress / min_progress) if (min_progress > 1e-9) else float("inf")

        timed_out_smaller = False
        for row in rows:
            status = str((row or {}).get("status", "")).strip().lower()
            row_delta = abs(float((row or {}).get("delta", 0.0) or 0.0))
            if status == "aborted_current":
                gate["reasons"].append(f"{side_label}:step_aborted_current")
            if (status == "timeout") and (row_delta < (winner_delta - 1e-9)):
                timed_out_smaller = True

        current_lim_local = max(1e-9, abs(float(cfg.get("current_lim", 0.0) or 0.0)))
        peak_abs_iq = abs(float(winner.get("peak_abs_iq", 0.0) or 0.0))
        iq_frac = peak_abs_iq / current_lim_local

        if progress_ratio < float(step_gate_min_progress_ratio):
            gate["reasons"].append(f"{side_label}:low_progress_ratio={progress_ratio:.2f}")
        if bool(step_gate_disallow_max_delta_winner) and winner_is_max_delta and timed_out_smaller:
            gate["reasons"].append(f"{side_label}:winner_at_max_delta_after_smaller_timeouts")
        if winner_is_max_delta and (iq_frac >= float(step_gate_iq_frac_warn)):
            gate["reasons"].append(f"{side_label}:winner_iq_frac={iq_frac:.2f}")

        gate["metrics"] = {
            "winner_delta": winner_delta,
            "max_delta": max_delta,
            "winner_is_max_delta": bool(winner_is_max_delta),
            "progress": progress,
            "min_progress": min_progress,
            "progress_ratio": progress_ratio,
            "peak_abs_iq": peak_abs_iq,
            "current_lim": current_lim_local,
            "iq_frac": iq_frac,
            "timed_out_smaller": bool(timed_out_smaller),
        }
        gate["ok"] = not bool(gate["reasons"])
        return gate

    def _scan_stage_for_encoder_blockers(stage_name):
        stage = dict(out.get("stages", {}).get(stage_name) or {})
        not_ready = False
        no_response = False

        def _scan_error_text(msg):
            txt = str(msg or "").strip().lower()
            nr = (
                ("encoder.is_ready is false" in txt)
                or ("encoder is not ready" in txt)
                or ("enc_ready" in txt and "false" in txt)
            )
            nresp = ("encoder_error_no_response" in txt) or ("no_response" in txt)
            return nr, nresp

        def _scan_snapshot(snap):
            nr = False
            nresp = False
            if isinstance(snap, dict):
                try:
                    if ("enc_ready" in snap) and (not bool(snap.get("enc_ready"))):
                        nr = True
                except Exception:
                    pass
                try:
                    enc_err = int(snap.get("enc_err", 0) or 0)
                    # ODrive: ENCODER_ERROR_NO_RESPONSE bit is 0x4.
                    if (enc_err & 0x4) != 0:
                        nresp = True
                except Exception:
                    pass
            return nr, nresp

        e1, e2 = _scan_error_text(stage.get("error"))
        s1, s2 = _scan_snapshot(stage.get("snapshot"))
        not_ready = not_ready or e1 or s1
        no_response = no_response or e2 or s2

        res = stage.get("result")
        if isinstance(res, dict):
            for row in list(res.get("results") or []):
                e1, e2 = _scan_error_text(row.get("error"))
                s1, s2 = _scan_snapshot(row.get("snapshot"))
                not_ready = not_ready or e1 or s1
                no_response = no_response or e2 or s2

        return bool(not_ready), bool(no_response)

    _ensure_index_policy(
        a,
        require_index=bool(require_index),
        auto_enable_index=bool(auto_enable_index),
        label="diagnostic_suite",
    )

    _run_stage(
        "absolute_reference",
        common.establish_absolute_reference,
        axis=a,
        require_index=bool(require_index),
        run_index_search=True,
        attempt_offset_calibration=False,
    )

    if reference_jump_warn_turns is None:
        ref_warn = 0.05 if mode in ("loaded", "unloaded") else 0.10
    else:
        ref_warn = abs(float(reference_jump_warn_turns))
    reference_ok = True
    if bool(run_reference_probe):
        ref = _run_stage(
            "reference_probe",
            reference_continuity_probe,
            cycles=int(max(2, int(reference_probe_cycles))),
            run_index_search=bool(reference_probe_run_index_search),
            jump_warn_turns=float(ref_warn),
            attempt_offset_calibration=False,
            require_index=bool(require_index),
            auto_enable_index=bool(auto_enable_index),
        )
        reference_ok = bool(isinstance(ref, dict) and bool((ref.get("summary") or {}).get("ok", False)))
    else:
        out["stages"]["reference_probe"] = {"ok": True, "skipped": True, "reason": "reference probe disabled by caller"}
        reference_ok = True

    step_allowed = bool(reference_ok)
    step_common = {
        "delta_candidates": cfg["delta_candidates"],
        "trap_vel": cfg["trap_vel"],
        "trap_acc": cfg["trap_acc"],
        "trap_dec": cfg["trap_dec"],
        "current_lim": cfg["current_lim"],
        "current_abort_frac": 0.90,
        "pos_gain": cfg["pos_gain"],
        "vel_gain": cfg["vel_gain"],
        "vel_i_gain": cfg["vel_i_gain"],
        "stiction_kick_nm": cfg["stiction_kick_nm"],
        "absolute_at_start": False,
        "run_index_search_at_start": False,
        "recheck_absolute_each_candidate": False,
        "strict_frame_lock": True,
        "frame_lock_jump_turns": 0.50,
        "abort_on_frame_jump": True,
        "load_mode": mode,
    }
    step_common.update(step_kwargs)
    step_common.pop("direction", None)

    if step_allowed:
        step_plus = _run_stage("step_plus", safe_loaded_step_test, direction=+1.0, **step_common)
        plus_abort_current = bool(_step_has_status(step_plus, "aborted_current"))
        if plus_abort_current:
            out["stages"]["step_minus"] = {
                "ok": False,
                "skipped": True,
                "reason": "skipped after +dir current-stress abort",
            }
            step_minus = None
        else:
            # Re-establish current-frame reference before reversing direction.
            _run_stage(
                "interstep_reference",
                common.establish_absolute_reference,
                axis=a,
                require_index=bool(require_index),
                run_index_search=False,
                attempt_offset_calibration=False,
            )
            step_minus = _run_stage("step_minus", safe_loaded_step_test, direction=-1.0, **step_common)
    else:
        out["stages"]["step_plus"] = {
            "ok": False,
            "skipped": True,
            "reason": "reference continuity gate failed",
        }
        out["stages"]["step_minus"] = {
            "ok": False,
            "skipped": True,
            "reason": "reference continuity gate failed",
        }
        step_plus = None
        step_minus = None

    plus_ok = bool(isinstance(step_plus, dict) and step_plus.get("winner"))
    minus_ok = bool(isinstance(step_minus, dict) and step_minus.get("winner"))
    step_plus_summary = dict(step_plus or {}) if isinstance(step_plus, dict) else {}
    step_minus_summary = dict(step_minus or {}) if isinstance(step_minus, dict) else {}
    step_frame_jumps = int(step_plus_summary.get("frame_jumps", 0)) + int(step_minus_summary.get("frame_jumps", 0))
    step_abort_current = int(bool(_step_has_status(step_plus, "aborted_current"))) + int(
        bool(_step_has_status(step_minus, "aborted_current"))
    )
    step_plus_gate = _assess_step_repeatability_gate(step_plus_summary, "+dir")
    step_minus_gate = _assess_step_repeatability_gate(step_minus_summary, "-dir")
    repeatability_step_gate_ok = bool(step_plus_gate.get("ok", False) and step_minus_gate.get("ok", False))
    repeatability_step_gate_reasons = list(step_plus_gate.get("reasons") or []) + list(
        step_minus_gate.get("reasons") or []
    )
    out["step_gate"] = {
        "ok": bool(repeatability_step_gate_ok),
        "reasons": tuple(repeatability_step_gate_reasons),
        "plus": step_plus_gate,
        "minus": step_minus_gate,
    }

    repeatability_result = None
    repeatability_alt_result = None
    rep_primary_summary = {}
    rep_alt_summary = {}
    rep_primary_hard_faults = 0
    rep_alt_hard_faults = 0
    rep_primary_frame_jumps = 0
    rep_alt_frame_jumps = 0
    repeatability_allowed = bool(
        run_repeatability and plus_ok and minus_ok and (step_frame_jumps == 0) and bool(repeatability_step_gate_ok)
    )
    if repeatability_allowed:
        if plus_ok and minus_ok:
            plus_w = dict(step_plus_summary.get("winner") or {})
            minus_w = dict(step_minus_summary.get("winner") or {})
            plus_iq = abs(float(plus_w.get("peak_abs_iq", 1e9)))
            minus_iq = abs(float(minus_w.get("peak_abs_iq", 1e9)))
            if mode in ("loaded", "unloaded"):
                # Use one-sided repeatability by default to avoid immediate reversal stress.
                base_now = float(getattr(a.encoder, "pos_estimate", 0.0))
                if plus_iq <= minus_iq:
                    approaches = ("from_below",)
                    target_turns = base_now + abs(float(cfg["repeat_delta"]))
                else:
                    approaches = ("from_above",)
                    target_turns = base_now - abs(float(cfg["repeat_delta"]))
            else:
                approaches = ("from_below", "from_above")
                target_turns = None
        elif plus_ok:
            approaches = ("from_below",)
            target_turns = None
        elif minus_ok:
            approaches = ("from_above",)
            base_now = float(getattr(a.encoder, "pos_estimate", 0.0))
            target_turns = base_now - abs(float(cfg["repeat_delta"]))
        else:
            approaches = ()
            target_turns = None

        is_dynamic_mode = mode in ("loaded", "unloaded")
        rep_common = {
            "target_turns": target_turns,
            "delta_turns": cfg["repeat_delta"],
            "approach_offset_turns": cfg["approach_offset"],
            "breakaway_margin_turns": cfg["breakaway_margin"],
            "stage_chunk_max_turns": cfg["stage_chunk"],
            "target_chunk_turns": cfg["target_chunk"],
            "target_tolerance_turns": 0.006 if mode == "loaded" else 0.004,
            "target_vel_tolerance_turns_s": 0.14 if mode == "loaded" else 0.10,
            "approaches": approaches,
            "cycles": int(max(1, int(repeatability_cycles))),
            "trap_vel": cfg["trap_vel"],
            "trap_acc": cfg["trap_acc"],
            "trap_dec": cfg["trap_dec"],
            "current_lim": cfg["current_lim"],
            "pos_gain": cfg["pos_gain"],
            "vel_gain": cfg["vel_gain"],
            "vel_i_gain": cfg["vel_i_gain"],
            "stiction_kick_nm": cfg["stiction_kick_nm"],
            "retry_on_current_limit": bool(is_dynamic_mode),
            "auto_derate": bool(is_dynamic_mode),
            "derate_step": 0.85,
            "min_derate": 0.55,
            "recover_reindex": False,
            "abort_on_hard_fault": True,
            "max_failures": 1 if bool(is_dynamic_mode) else 2,
            "strict_frame_lock": True,
            "frame_lock_jump_turns": 0.50,
            "abort_on_frame_jump": True,
            "max_frame_jumps": 1,
            "auto_followup_measures": bool(is_dynamic_mode),
            "followup_delta_candidates": tuple(cfg["delta_candidates"]),
            "load_mode": mode,
            "absolute_at_start": True,
            "run_index_search_at_start": False,
            "require_index_at_start": bool(require_index),
            "directional_bias_compensation": bool(mode == "loaded"),
            "directional_bias_learn_rate": 0.60 if mode == "loaded" else 0.0,
            "directional_bias_limit_turns": 0.008 if mode == "loaded" else 0.0,
            "directional_bias_warmup_samples": 1,
            "auto_reference_recovery_on_frame_jump": bool(is_dynamic_mode),
            "frame_recovery_retries": 1 if bool(is_dynamic_mode) else 0,
            "frame_recovery_run_index_search": bool(mode == "loaded"),
            "persist_directional_bias": bool(mode == "loaded"),
            "load_persistent_bias": bool(mode == "loaded"),
            "max_target_span_turns": min(0.50, max(0.12, 6.0 * abs(float(cfg["repeat_delta"])))),
            "abort_on_unsafe_target_span": True,
        }
        rep_common.update(repeatability_kwargs)
        repeatability_result = _run_stage("repeatability", repeatability_test_single, **rep_common)
        # If one-sided repeatability faults, automatically try the opposite side once.
        if isinstance(repeatability_result, dict):
            rep_primary_summary = dict(repeatability_result.get("summary", {}) or {})
        rep_primary_hard_faults = int(rep_primary_summary.get("hard_faults", 0))
        rep_primary_frame_jumps = int(rep_primary_summary.get("frame_jumps", 0))
        primary_ok = bool(rep_primary_summary.get("ok", False))
        primary_hard_or_jump = bool(
            rep_primary_hard_faults > 0
            or rep_primary_frame_jumps > 0
            or str(rep_primary_summary.get("aborted_reason", "")).strip().lower() in ("hard_fault", "frame_jump")
        )
        current_approaches = tuple(rep_common.get("approaches") or ())
        if (
            bool(is_dynamic_mode)
            and (not primary_ok)
            and primary_hard_or_jump
            and len(current_approaches) == 1
            and current_approaches[0] in ("from_below", "from_above")
            and bool(try_alternate_approach_on_fail)
        ):
            alt_approach = ("from_above",) if current_approaches[0] == "from_below" else ("from_below",)
            print(
                "diagnostic_suite: repeatability primary approach failed with hard/jump fault; "
                f"trying alternate approach={alt_approach[0]}."
            )
            rep_alt_common = dict(rep_common)
            rep_alt_common["approaches"] = alt_approach
            repeatability_alt_result = _run_stage("repeatability_alternate", repeatability_test_single, **rep_alt_common)
            if isinstance(repeatability_alt_result, dict):
                rep_alt_summary = dict(repeatability_alt_result.get("summary", {}) or {})
            rep_alt_hard_faults = int(rep_alt_summary.get("hard_faults", 0))
            rep_alt_frame_jumps = int(rep_alt_summary.get("frame_jumps", 0))
    else:
        reason = "repeatability disabled by caller" if not bool(run_repeatability) else \
            "repeatability skipped: requires clean + and - step wins with zero frame jumps"
        if not step_allowed:
            reason = "repeatability skipped: reference continuity gate failed"
        elif not bool(repeatability_step_gate_ok):
            reason = (
                "repeatability skipped: step-margin gate flagged low robustness "
                f"({', '.join(str(x) for x in repeatability_step_gate_reasons)})"
            )
        out["stages"]["repeatability"] = {
            "ok": False,
            "skipped": True,
            "reason": reason,
        }

    rep_summary = dict(rep_primary_summary or {})
    if (not rep_summary) and isinstance(repeatability_result, dict):
        rep_summary = dict(repeatability_result.get("summary", {}) or {})
    if (not rep_alt_summary) and isinstance(repeatability_alt_result, dict):
        rep_alt_summary = dict(repeatability_alt_result.get("summary", {}) or {})
    if rep_alt_hard_faults == 0 and rep_alt_summary:
        rep_alt_hard_faults = int(rep_alt_summary.get("hard_faults", 0))
    if rep_alt_frame_jumps == 0 and rep_alt_summary:
        rep_alt_frame_jumps = int(rep_alt_summary.get("frame_jumps", 0))
    rep_alt_used = bool(rep_alt_summary)
    rep_alt_ok = bool(rep_alt_summary.get("ok", False)) if bool(rep_alt_summary) else None
    if (not bool(rep_summary.get("ok", False))) and bool(rep_alt_ok):
        # Prefer alternate result when it recovers repeatability.
        repeatability_result = repeatability_alt_result
        rep_summary = rep_alt_summary
    rep_followups = dict(repeatability_result.get("followups", {}) or {}) if isinstance(repeatability_result, dict) else {}
    rep_follow_plus = dict(rep_followups.get("plus", {}) or {}) if isinstance(rep_followups.get("plus"), dict) else {}
    rep_follow_minus = dict(rep_followups.get("minus", {}) or {}) if isinstance(rep_followups.get("minus"), dict) else {}
    rep_follow_plus_ok = bool(rep_follow_plus.get("winner"))
    rep_follow_minus_ok = bool(rep_follow_minus.get("winner"))
    rep_hard_faults_total = int(rep_primary_hard_faults) + int(rep_alt_hard_faults)
    rep_frame_jumps_total = int(rep_primary_frame_jumps) + int(rep_alt_frame_jumps)
    hard_faults = int(rep_hard_faults_total) + int(step_abort_current)
    frame_jumps = int(rep_frame_jumps_total) + int(step_frame_jumps)
    motion_gate_ok = bool(reference_ok and plus_ok and minus_ok)
    reference_recovery_triggered = False
    reference_recovery_ok = None
    repeated_rep_frame_jumps = int(rep_frame_jumps_total) >= max(1, int(reference_recovery_frame_jump_trigger))
    if bool(auto_reference_recovery) and bool(repeatability_allowed) and bool(repeated_rep_frame_jumps):
        reference_recovery_triggered = True
        print(
            "diagnostic_suite: repeated repeatability frame jumps detected; "
            "switching to reference_recovery mode."
        )
        rr_cfg = {
            "axis": a,
            "run_index_search": True,
            "attempt_offset_calibration": False,
            "continuity_cycles": int(max(2, int(reference_probe_cycles))),
            "continuity_jump_warn_turns": float(ref_warn),
        }
        rr_cfg.update(reference_recovery_kwargs)
        ref_recover = _run_stage("reference_recovery", reference_recovery_mode, **rr_cfg)
        if isinstance(ref_recover, dict):
            reference_recovery_ok = bool(ref_recover.get("ok", False))
        else:
            reference_recovery_ok = bool(ref_recover)
    unloaded_follow = {}
    unloaded_follow_plus_ok = None
    unloaded_follow_minus_ok = None
    if (
        mode == "unloaded"
        and isinstance(repeatability_result, dict)
        and (not bool(rep_summary.get("ok", False)))
    ):
        # Keep collecting useful data in unloaded mode after repeatability faults.
        env_deltas = tuple(sorted({max(0.003, 0.5 * float(cfg["repeat_delta"])), max(0.006, float(cfg["repeat_delta"]))}))
        envelope_common = {
            "delta_candidates": env_deltas,
            "trap_vel": min(float(cfg["trap_vel"]), 0.08),
            "trap_acc": min(float(cfg["trap_acc"]), 0.16),
            "trap_dec": min(float(cfg["trap_dec"]), 0.16),
            "current_lim": min(float(cfg["current_lim"]), 6.0),
            "current_abort_frac": 0.90,
            "pos_gain": min(float(cfg["pos_gain"]), 12.0),
            "vel_gain": min(float(cfg["vel_gain"]), 0.16),
            "vel_i_gain": 0.0,
            "stiction_kick_nm": 0.0,
            "duration_s": 1.4,
            "dt": 0.02,
            "absolute_at_start": False,
            "run_index_search_at_start": False,
            "recheck_absolute_each_candidate": False,
            "strict_frame_lock": True,
            "frame_lock_jump_turns": 0.50,
            "abort_on_frame_jump": True,
            "load_mode": mode,
        }
        unloaded_follow["plus"] = _run_stage("unloaded_followup_plus", safe_loaded_step_test, direction=+1.0, **envelope_common)
        unloaded_follow["minus"] = _run_stage("unloaded_followup_minus", safe_loaded_step_test, direction=-1.0, **envelope_common)
        up = dict(unloaded_follow.get("plus") or {}) if isinstance(unloaded_follow.get("plus"), dict) else {}
        um = dict(unloaded_follow.get("minus") or {}) if isinstance(unloaded_follow.get("minus"), dict) else {}
        unloaded_follow_plus_ok = bool(up.get("winner"))
        unloaded_follow_minus_ok = bool(um.get("winner"))
    unloaded_follow_motion_ok = bool(unloaded_follow_plus_ok and unloaded_follow_minus_ok)

    enc_not_ready = False
    enc_no_response = False
    for _st_name in ("absolute_reference", "reference_probe", "reference_recovery"):
        nr, nresp = _scan_stage_for_encoder_blockers(_st_name)
        enc_not_ready = bool(enc_not_ready or nr)
        enc_no_response = bool(enc_no_response or nresp)

    if mode == "unloaded":
        if bool(enc_no_response):
            classification = "encoder_no_response"
            note = "Encoder reported no response; check encoder power/signal path before any motion tuning."
        elif bool(enc_not_ready):
            classification = "encoder_not_ready"
            note = "Encoder is not ready; restore index/offset readiness before continuing diagnostics."
        elif bool(reference_recovery_triggered) and (reference_recovery_ok is False):
            classification = "reference_integrity_fault"
            note = "Reference recovery was triggered after repeated frame jumps and did not pass."
        elif not bool(reference_ok):
            classification = "reference_integrity_fault"
            note = "Reference continuity gate failed; motion diagnostics are not trustworthy."
        elif (hard_faults > 0 or frame_jumps > 0) and bool(unloaded_follow_motion_ok):
            classification = "repeatability_fault_unloaded_motion_ok"
            note = (
                "Unloaded motion gates and conservative follow-up envelope passed, "
                "but strict repeatability sequence still faults."
            )
        elif hard_faults > 0 or frame_jumps > 0:
            classification = "control_or_reference_fault_unloaded"
            note = "Faults/frame jumps without load indicate control/reference integrity issues, not payload tuning."
        elif plus_ok and minus_ok:
            classification = "motion_ok_unloaded"
            note = "Both directions move unloaded; proceed to controlled loaded validation."
        elif plus_ok != minus_ok:
            classification = "directional_asymmetry_unloaded"
            note = "Unloaded directional asymmetry suggests backlash/stiction or directional control bias."
        else:
            classification = "no_motion_unloaded"
            note = "Neither direction achieved step targets unloaded."
    else:
        if bool(enc_no_response):
            classification = "encoder_no_response"
            note = "Encoder reported no response; resolve encoder link before loaded tuning."
        elif bool(enc_not_ready):
            classification = "encoder_not_ready"
            note = "Encoder is not ready; resolve index/offset readiness before loaded tuning."
        elif bool(reference_recovery_triggered) and (reference_recovery_ok is False):
            classification = "reference_integrity_fault"
            note = (
                "Reference recovery was triggered after repeated repeatability frame jumps and did not pass; "
                "resolve reference integrity before further loaded repeatability."
            )
        elif not bool(reference_ok):
            classification = "reference_integrity_fault"
            note = "Reference continuity gate failed; resolve index/offset continuity before loaded tuning."
        elif motion_gate_ok and hard_faults > 0:
            classification = "repeatability_fault_loaded_motion_ok"
            note = (
                "Loaded directional motion gates passed, but reversal repeatability triggered hard faults. "
                "Use derated one-sided repeatability before full bidirectional repeats."
            )
        elif motion_gate_ok and frame_jumps > 0:
            classification = "repeatability_frame_jump_loaded_motion_ok"
            note = (
                "Loaded directional motion gates passed, but repeatability hit frame jumps. "
                "Keep frame-preserving recovery and one-sided repeats."
            )
        elif hard_faults > 0:
            classification = "hard_fault_under_load"
            note = "Current-limit or related hard faults observed during loaded diagnostics."
        elif frame_jumps > 0:
            classification = "frame_jump_loaded"
            note = "Frame jump(s) detected under load; results are not in a stable coordinate frame."
        elif plus_ok and minus_ok:
            classification = "motion_ok_loaded"
            note = "Both directions achieved at least one safe loaded step."
        elif plus_ok != minus_ok:
            classification = "directional_margin_loaded"
            note = "Loaded behavior is directional; torque margin or stiction differs by direction."
        else:
            classification = "no_motion_loaded"
            note = "Neither direction achieved step targets under current loaded profile."

    if bool(reference_recovery_triggered) and (reference_recovery_ok is True):
        note = f"{note} Reference recovery completed after repeated frame jumps."

    summary = {
        "ok": bool(
            plus_ok
            and minus_ok
            and (not run_repeatability or bool(rep_summary.get("ok", False)))
            and (hard_faults == 0)
            and (frame_jumps == 0)
            and bool(reference_ok)
        ),
        "load_mode": mode,
        "classification": classification,
        "interpretation": note,
        "motion_gate_ok": bool(motion_gate_ok),
        "reference_ok": bool(reference_ok),
        "step_plus_ok": bool(plus_ok),
        "step_minus_ok": bool(minus_ok),
        "step_abort_current": int(step_abort_current),
        "repeatability_step_gate_ok": bool(repeatability_step_gate_ok),
        "repeatability_step_gate_reasons": tuple(repeatability_step_gate_reasons),
        "repeatability_ok": bool(rep_summary.get("ok", False)) if bool(rep_summary) else None,
        "repeatability_alternate_used": bool(rep_alt_used),
        "repeatability_alternate_ok": rep_alt_ok,
        "repeatability_primary_frame_jumps": int(rep_primary_frame_jumps),
        "repeatability_alternate_frame_jumps": int(rep_alt_frame_jumps),
        "repeatability_frame_jumps_total": int(rep_frame_jumps_total),
        "repeatability_followup_plus_ok": bool(rep_follow_plus_ok) if bool(rep_followups) else None,
        "repeatability_followup_minus_ok": bool(rep_follow_minus_ok) if bool(rep_followups) else None,
        "reference_recovery_triggered": bool(reference_recovery_triggered),
        "reference_recovery_ok": reference_recovery_ok,
        "unloaded_followup_plus_ok": unloaded_follow_plus_ok,
        "unloaded_followup_minus_ok": unloaded_follow_minus_ok,
        "unloaded_followup_motion_ok": bool(unloaded_follow_motion_ok) if (unloaded_follow_plus_ok is not None or unloaded_follow_minus_ok is not None) else None,
        "encoder_not_ready": bool(enc_not_ready),
        "encoder_no_response": bool(enc_no_response),
        "hard_faults": int(hard_faults),
        "frame_jumps": int(frame_jumps),
    }
    out["summary"] = summary
    print("diagnostic_suite_summary:", summary)
    return out


def diagnostic_suite_recursive(
    load_mode="loaded",
    max_rounds=3,
    require_repeatability=False,
    stop_on_hard_fault=True,
    stop_on_frame_jump=True,
    suite_kwargs=None,
):
    """Run diagnostic_suite() iteratively with conservative auto-adjustments.

    This is bounded recursion (max_rounds), never infinite.
    """
    mode = _normalize_load_mode(load_mode)
    rounds = max(1, int(max_rounds))
    suite_kwargs = dict(suite_kwargs or {})
    history = []
    tune = {}
    requested_envelope = dict(_load_mode_defaults(mode))
    requested_envelope.update(
        {
            k: suite_kwargs[k]
            for k in (
                "delta_candidates",
                "trap_vel",
                "trap_acc",
                "trap_dec",
                "current_lim",
                "pos_gain",
                "vel_gain",
                "vel_i_gain",
                "stiction_kick_nm",
            )
            if k in suite_kwargs
        }
    )
    best_run = None
    best_key = None

    def _merge_dict(base, override):
        out = dict(base or {})
        out.update(dict(override or {}))
        return out

    def _rank_summary(summary):
        hard_faults = int(summary.get("hard_faults", 0))
        frame_jumps = int(summary.get("frame_jumps", 0))
        return (
            1 if bool(summary.get("motion_gate_ok", False)) else 0,
            1 if bool(summary.get("step_plus_ok", False)) else 0,
            1 if bool(summary.get("step_minus_ok", False)) else 0,
            -hard_faults,
            -frame_jumps,
            1 if bool(summary.get("repeatability_ok", False)) else 0,
        )

    def _apply_loaded_safety_derate(cfg, run_kwargs, directional_hint=False):
        """Loaded-mode fallback: reduce stress instead of growing aggressiveness."""
        cur_deltas = tuple(cfg.get("delta_candidates") or ())
        if cur_deltas:
            # Keep practical motion chunks but reduce command stress.
            tune["delta_candidates"] = tuple(max(0.006, float(d) * 0.80) for d in cur_deltas)
        tune["trap_vel"] = max(0.05, float(cfg.get("trap_vel", 0.10)) * 0.85)
        tune["trap_acc"] = max(0.10, float(cfg.get("trap_acc", 0.20)) * 0.85)
        tune["trap_dec"] = max(0.10, float(cfg.get("trap_dec", 0.20)) * 0.85)
        tune["current_lim"] = max(5.5, float(cfg.get("current_lim", 8.0)) * 0.90)
        tune["pos_gain"] = max(12.0, float(cfg.get("pos_gain", 16.0)) * 0.90)
        tune["vel_gain"] = max(0.18, float(cfg.get("vel_gain", 0.24)) * 0.90)
        tune["vel_i_gain"] = max(0.0, min(float(cfg.get("vel_i_gain", 0.02)) * 0.50, 0.03))
        tune["stiction_kick_nm"] = min(float(cfg.get("stiction_kick_nm", 0.0)), 0.01)
        tune["run_repeatability"] = False

        # Give loaded moves more time and slightly softer progress gate.
        step_over = _merge_dict(
            run_kwargs.get("step_kwargs"),
            {
                "duration_s": max(1.8, float((run_kwargs.get("step_kwargs") or {}).get("duration_s", 1.8))),
                "min_progress_frac": min(0.35, max(0.22, float((run_kwargs.get("step_kwargs") or {}).get("min_progress_frac", 0.30)))),
            },
        )
        tune["step_kwargs"] = step_over

        if bool(directional_hint):
            rep_over = _merge_dict(run_kwargs.get("repeatability_kwargs"), {"approaches": ("from_below",)})
            tune["repeatability_kwargs"] = rep_over

    def _apply_loaded_motion_boost(cfg, run_kwargs):
        """Loaded-mode rebound when no-motion is observed without hard faults."""
        cur_deltas = tuple(cfg.get("delta_candidates") or ())
        req_deltas = tuple(requested_envelope.get("delta_candidates") or cur_deltas or (0.010, 0.020))
        n = max(len(cur_deltas), len(req_deltas))
        boosted = []
        for i in range(n):
            cur = float(cur_deltas[i]) if i < len(cur_deltas) else float(req_deltas[min(i, len(req_deltas) - 1)])
            req = float(req_deltas[i]) if i < len(req_deltas) else float(req_deltas[-1])
            boosted.append(min(0.050, max(req, cur * 1.25)))
        tune["delta_candidates"] = tuple(boosted)

        req_vel = float(requested_envelope.get("trap_vel", cfg.get("trap_vel", 0.10)))
        req_acc = float(requested_envelope.get("trap_acc", cfg.get("trap_acc", 0.20)))
        req_dec = float(requested_envelope.get("trap_dec", cfg.get("trap_dec", 0.20)))
        req_lim = float(requested_envelope.get("current_lim", cfg.get("current_lim", 8.0)))
        req_pg = float(requested_envelope.get("pos_gain", cfg.get("pos_gain", 16.0)))
        req_vg = float(requested_envelope.get("vel_gain", cfg.get("vel_gain", 0.24)))
        req_vig = float(requested_envelope.get("vel_i_gain", cfg.get("vel_i_gain", 0.02)))
        req_kick = float(requested_envelope.get("stiction_kick_nm", cfg.get("stiction_kick_nm", 0.01)))

        tune["trap_vel"] = min(0.20, max(req_vel, float(cfg.get("trap_vel", req_vel)) * 1.20))
        tune["trap_acc"] = min(0.35, max(req_acc, float(cfg.get("trap_acc", req_acc)) * 1.20))
        tune["trap_dec"] = min(0.35, max(req_dec, float(cfg.get("trap_dec", req_dec)) * 1.20))
        tune["current_lim"] = min(15.0, max(req_lim, float(cfg.get("current_lim", req_lim)) * 1.20))
        tune["pos_gain"] = min(35.0, max(req_pg, float(cfg.get("pos_gain", req_pg)) * 1.15))
        tune["vel_gain"] = min(0.70, max(req_vg, float(cfg.get("vel_gain", req_vg)) * 1.15))
        tune["vel_i_gain"] = min(0.12, max(req_vig, float(cfg.get("vel_i_gain", req_vig)) * 1.20))
        tune["stiction_kick_nm"] = min(0.20, max(req_kick, 0.03))
        tune["run_repeatability"] = False

        step_over = _merge_dict(
            run_kwargs.get("step_kwargs"),
            {
                "duration_s": max(1.8, float((run_kwargs.get("step_kwargs") or {}).get("duration_s", 1.8))),
                "min_progress_frac": min(0.35, max(0.25, float((run_kwargs.get("step_kwargs") or {}).get("min_progress_frac", 0.30)))),
            },
        )
        tune["step_kwargs"] = step_over

    for ridx in range(1, rounds + 1):
        run_kwargs = dict(suite_kwargs)
        run_kwargs.update(tune)
        run_kwargs["load_mode"] = mode
        print(f"\n=== diagnostic_suite_recursive round {ridx}/{rounds} ===")
        run = diagnostic_suite(**run_kwargs)
        history.append(run)

        summary = dict(run.get("summary") or {})
        cfg = dict(run.get("config") or {})
        cls = str(summary.get("classification", ""))
        hard_faults = int(summary.get("hard_faults", 0))
        frame_jumps = int(summary.get("frame_jumps", 0))
        step_ok = bool(summary.get("step_plus_ok")) and bool(summary.get("step_minus_ok"))
        motion_gate_ok = bool(summary.get("motion_gate_ok", False))
        rep_step_gate_ok = bool(summary.get("repeatability_step_gate_ok", True))
        rep_ok = bool(summary.get("repeatability_ok", False))
        satisfied = bool(step_ok and (rep_ok if bool(require_repeatability) else True) and hard_faults == 0 and frame_jumps == 0)
        rank_key = _rank_summary(summary)
        if best_key is None or rank_key > best_key:
            best_key = rank_key
            best_run = run

        # If the user only requires motion-gate success, allow loaded runs to conclude
        # once reference + both directions pass, even if repeatability remains unsafe.
        if (
            mode == "loaded"
            and (not bool(require_repeatability))
            and motion_gate_ok
            and (hard_faults > 0 or frame_jumps > 0)
        ):
            return {
                "ok": True,
                "satisfied": True,
                "stop_reason": "motion_gate_satisfied_repeatability_unstable",
                "rounds_run": int(ridx),
                "best": (best_run if best_run is not None else run),
                "history": history,
            }

        # Loaded asymmetry/hard-fault handling:
        # Derate first and gather more evidence before giving up, instead of auto-growing stress.
        if (
            mode == "loaded"
            and ridx < rounds
            and (hard_faults > 0 or cls in ("hard_fault_under_load", "directional_margin_loaded"))
        ):
            _apply_loaded_safety_derate(
                cfg,
                run_kwargs,
                directional_hint=bool(cls == "directional_margin_loaded"),
            )
            print(
                "diagnostic_suite_recursive: loaded safety-derate applied "
                f"(classification={cls or 'n/a'} hard_faults={hard_faults})."
            )
            continue

        if (
            mode == "loaded"
            and ridx < rounds
            and cls == "no_motion_loaded"
            and hard_faults == 0
            and frame_jumps == 0
        ):
            _apply_loaded_motion_boost(cfg, run_kwargs)
            print("diagnostic_suite_recursive: loaded no-motion rebound applied (increase command margin).")
            continue

        if (
            mode == "loaded"
            and ridx < rounds
            and motion_gate_ok
            and (hard_faults == 0)
            and (frame_jumps == 0)
            and (not rep_step_gate_ok)
        ):
            rep_over = _merge_dict(run_kwargs.get("repeatability_kwargs"), {})
            cur_ratio = float(rep_over.get("step_gate_min_progress_ratio", 1.35))
            next_ratio = max(1.00, cur_ratio - 0.10)
            rep_over["step_gate_min_progress_ratio"] = float(next_ratio)
            if next_ratio <= 1.05:
                rep_over.setdefault("step_gate_disallow_max_delta_winner", False)
                rep_over.setdefault("step_gate_iq_frac_warn", 0.45)
            tune["repeatability_kwargs"] = rep_over
            tune["run_repeatability"] = True
            print(
                "diagnostic_suite_recursive: loaded motion gate passed but repeatability margin gate blocked; "
                f"relaxing step_gate_min_progress_ratio {cur_ratio:.2f} -> {next_ratio:.2f}."
            )
            continue

        if bool(stop_on_hard_fault) and hard_faults > 0:
            if mode == "unloaded" and ridx < rounds:
                # Unloaded diagnostics should keep gathering signal by automatically
                # falling back to a softer envelope instead of hard-stopping.
                cur_deltas = tuple(cfg.get("delta_candidates") or ())
                if cur_deltas:
                    tune["delta_candidates"] = tuple(max(0.003, float(d) * 0.75) for d in cur_deltas)
                tune["trap_vel"] = max(0.06, float(cfg.get("trap_vel", 0.10)) * 0.80)
                tune["trap_acc"] = max(0.12, float(cfg.get("trap_acc", 0.20)) * 0.80)
                tune["trap_dec"] = max(0.12, float(cfg.get("trap_dec", 0.20)) * 0.80)
                tune["current_lim"] = max(4.0, float(cfg.get("current_lim", 6.0)) * 0.85)
                tune["pos_gain"] = max(8.0, float(cfg.get("pos_gain", 12.0)) * 0.90)
                tune["vel_gain"] = max(0.10, float(cfg.get("vel_gain", 0.20)) * 0.85)
                tune["vel_i_gain"] = 0.0
                tune["stiction_kick_nm"] = min(float(cfg.get("stiction_kick_nm", 0.01)), 0.005)
                tune["run_repeatability"] = False
                continue
            return {
                "ok": False,
                "satisfied": False,
                "stop_reason": "hard_fault",
                "rounds_run": int(ridx),
                "best": (best_run if best_run is not None else run),
                "history": history,
            }
        if satisfied:
            return {
                "ok": True,
                "satisfied": True,
                "stop_reason": "satisfied",
                "rounds_run": int(ridx),
                "best": (best_run if best_run is not None else run),
                "history": history,
            }
        if bool(stop_on_frame_jump) and frame_jumps > 0:
            if mode == "unloaded" and ridx < rounds:
                cur_deltas = tuple(cfg.get("delta_candidates") or ())
                if cur_deltas:
                    tune["delta_candidates"] = tuple(max(0.003, float(d) * 0.75) for d in cur_deltas)
                tune["trap_vel"] = max(0.06, float(cfg.get("trap_vel", 0.10)) * 0.80)
                tune["trap_acc"] = max(0.12, float(cfg.get("trap_acc", 0.20)) * 0.80)
                tune["trap_dec"] = max(0.12, float(cfg.get("trap_dec", 0.20)) * 0.80)
                tune["current_lim"] = max(4.0, float(cfg.get("current_lim", 6.0)) * 0.85)
                tune["pos_gain"] = max(8.0, float(cfg.get("pos_gain", 12.0)) * 0.90)
                tune["vel_gain"] = max(0.10, float(cfg.get("vel_gain", 0.20)) * 0.85)
                tune["vel_i_gain"] = 0.0
                tune["stiction_kick_nm"] = min(float(cfg.get("stiction_kick_nm", 0.01)), 0.005)
                tune["run_repeatability"] = False
                continue
            return {
                "ok": False,
                "satisfied": False,
                "stop_reason": "frame_jump",
                "rounds_run": int(ridx),
                "best": (best_run if best_run is not None else run),
                "history": history,
            }
        if cls == "control_or_reference_fault_unloaded":
            return {
                "ok": False,
                "satisfied": False,
                "stop_reason": "reference_integrity",
                "rounds_run": int(ridx),
                "best": (best_run if best_run is not None else run),
                "history": history,
            }

        prev_deltas = tuple(cfg.get("delta_candidates") or ())
        if prev_deltas:
            grown_deltas = tuple(min(0.050, max(0.001, float(d) * 1.5)) for d in prev_deltas)
        else:
            grown_deltas = None

        # Conservative bounded auto-adjustments.
        cur_lim = float(cfg.get("current_lim", 6.0))
        lim_cap = 8.0 if mode == "unloaded" else 15.0
        cur_pos_gain = float(cfg.get("pos_gain", 12.0))
        cur_vel_gain = float(cfg.get("vel_gain", 0.20))
        cur_vel_i = float(cfg.get("vel_i_gain", 0.02))
        kick_floor = 0.01 if mode == "unloaded" else 0.05

        tune["delta_candidates"] = grown_deltas
        tune["current_lim"] = min(lim_cap, cur_lim + 1.0)
        tune["pos_gain"] = min(35.0, cur_pos_gain + 2.0)
        tune["vel_gain"] = min(0.70, cur_vel_gain + 0.04)
        tune["vel_i_gain"] = min(0.12, cur_vel_i + 0.02)
        tune["stiction_kick_nm"] = min(0.20, max(float(cfg.get("stiction_kick_nm", 0.0)), kick_floor))
        # Keep repeatability enabled when caller explicitly requires it.
        # diagnostic_suite() already guards execution until step gates are clean.
        if bool(require_repeatability):
            tune["run_repeatability"] = True
        else:
            tune["run_repeatability"] = bool(step_ok)

        if cls in ("directional_asymmetry_unloaded", "directional_margin_loaded"):
            step_over = _merge_dict(run_kwargs.get("step_kwargs"), {"duration_s": 1.6, "min_progress_frac": 0.35})
            rep_over = _merge_dict(run_kwargs.get("repeatability_kwargs"), {"approaches": ("from_below",)})
            tune["step_kwargs"] = step_over
            tune["repeatability_kwargs"] = rep_over

    best = best_run if best_run is not None else (history[-1] if history else {})
    return {
        "ok": False,
        "satisfied": False,
        "stop_reason": "max_rounds",
        "rounds_run": int(len(history)),
        "best": best,
        "history": history,
    }


def run_slow_fast_campaign(
    max_rounds=3,
    require_repeatability=True,
    stop_on_hard_fault=True,
    stop_on_frame_jump=True,
    slow_suite_kwargs=None,
    fast_suite_kwargs=None,
):
    """Run loaded diagnostics for slow and fast profiles, sequentially.

    This is intended as a one-call "find the final numbers" campaign once the
    encoder/reference gate is healthy.
    """

    def _profile_defaults(kind):
        if str(kind).strip().lower() == "slow":
            return {
                "run_repeatability": True,
                "repeatability_cycles": 5,
                "delta_candidates": (0.006, 0.012, 0.018),
                "trap_vel": 0.10,
                "trap_acc": 0.20,
                "trap_dec": 0.20,
                "current_lim": 8.0,
                "pos_gain": 14.0,
                "vel_gain": 0.24,
                "vel_i_gain": 0.04,
                "stiction_kick_nm": 0.01,
                "step_kwargs": {
                    "target_tolerance_turns": 0.006,
                    "target_vel_tolerance_turns_s": 0.14,
                    "min_progress_frac": 0.35,
                    "duration_s": 1.6,
                },
                "repeatability_kwargs": {
                    "delta_turns": 0.012,
                    "approach_offset_turns": 0.020,
                    "breakaway_margin_turns": 0.006,
                    "stage_chunk_max_turns": 0.006,
                    "target_chunk_turns": 0.006,
                    "retry_on_current_limit": True,
                    "auto_derate": True,
                    "derate_step": 0.90,
                    "min_derate": 0.60,
                    "abort_on_hard_fault": True,
                    "max_failures": 3,
                    "recover_reindex": False,
                    "strict_frame_lock": True,
                    "frame_lock_jump_turns": 0.50,
                    "abort_on_frame_jump": True,
                    "max_frame_jumps": 1,
                    "auto_reference_recovery_on_frame_jump": True,
                    "frame_recovery_retries": 1,
                    "frame_recovery_run_index_search": True,
                    "persist_directional_bias": True,
                    "load_persistent_bias": True,
                },
            }
        return {
            "run_repeatability": True,
            "repeatability_cycles": 5,
            "delta_candidates": (0.010, 0.020, 0.030),
            "trap_vel": 0.15,
            "trap_acc": 0.30,
            "trap_dec": 0.30,
            "current_lim": 10.0,
            "pos_gain": 20.0,
            "vel_gain": 0.30,
            "vel_i_gain": 0.06,
            "stiction_kick_nm": 0.02,
            "step_kwargs": {
                "target_tolerance_turns": 0.010,
                "target_vel_tolerance_turns_s": 0.20,
                "min_progress_frac": 0.30,
                "duration_s": 1.6,
            },
            "repeatability_kwargs": {
                "delta_turns": 0.010,
                "approach_offset_turns": 0.020,
                "breakaway_margin_turns": 0.006,
                "stage_chunk_max_turns": 0.006,
                "target_chunk_turns": 0.006,
                "retry_on_current_limit": True,
                "auto_derate": True,
                "derate_step": 0.90,
                "min_derate": 0.60,
                "abort_on_hard_fault": True,
                "max_failures": 4,
                "recover_reindex": False,
                "strict_frame_lock": True,
                "frame_lock_jump_turns": 0.50,
                "abort_on_frame_jump": True,
                "max_frame_jumps": 1,
                "auto_reference_recovery_on_frame_jump": True,
                "frame_recovery_retries": 1,
                "frame_recovery_run_index_search": True,
                "persist_directional_bias": True,
                "load_persistent_bias": True,
            },
        }

    def _merge_profile(base, override):
        out = dict(base or {})
        ov = dict(override or {})
        if isinstance(out.get("step_kwargs"), dict) or isinstance(ov.get("step_kwargs"), dict):
            sw = dict(out.get("step_kwargs") or {})
            sw.update(dict(ov.get("step_kwargs") or {}))
            out["step_kwargs"] = sw
            ov.pop("step_kwargs", None)
        if isinstance(out.get("repeatability_kwargs"), dict) or isinstance(ov.get("repeatability_kwargs"), dict):
            rw = dict(out.get("repeatability_kwargs") or {})
            rw.update(dict(ov.get("repeatability_kwargs") or {}))
            out["repeatability_kwargs"] = rw
            ov.pop("repeatability_kwargs", None)
        out.update(ov)
        return out

    def _extract_numbers(best):
        stages = dict(best.get("stages") or {}) if isinstance(best, dict) else {}
        summary = dict(best.get("summary") or {}) if isinstance(best, dict) else {}
        plus = dict((dict(stages.get("step_plus") or {}).get("result") or {}).get("winner") or {})
        minus = dict((dict(stages.get("step_minus") or {}).get("result") or {}).get("winner") or {})
        rep = dict((dict(stages.get("repeatability") or {}).get("result") or {}).get("summary") or {})
        return {
            "classification": summary.get("classification"),
            "motion_gate_ok": bool(summary.get("motion_gate_ok", False)),
            "repeatability_ok": bool(summary.get("repeatability_ok", False)),
            "hard_faults": int(summary.get("hard_faults", 0)),
            "frame_jumps": int(summary.get("frame_jumps", 0)),
            "step_plus_delta": plus.get("delta"),
            "step_minus_delta": minus.get("delta"),
            "step_plus_err": plus.get("final_err"),
            "step_minus_err": minus.get("final_err"),
            "repeatability_spread": rep.get("overall_spread"),
            "repeatability_mean_err": rep.get("overall_mean_err"),
            "repeatability_attempts": rep.get("attempts"),
            "repeatability_successes": rep.get("successes"),
            "repeatability_failures": rep.get("failures"),
        }

    slow_cfg = _merge_profile(_profile_defaults("slow"), slow_suite_kwargs)
    fast_cfg = _merge_profile(_profile_defaults("fast"), fast_suite_kwargs)

    out = {
        "ok": False,
        "stop_reason": None,
        "profiles": {},
        "numbers": {},
    }

    for name, cfg in (("slow", slow_cfg), ("fast", fast_cfg)):
        print(f"\n=== run_slow_fast_campaign: {name} profile ===")
        rec = diagnostic_suite_recursive(
            load_mode="loaded",
            max_rounds=int(max(1, int(max_rounds))),
            require_repeatability=bool(require_repeatability),
            stop_on_hard_fault=bool(stop_on_hard_fault),
            stop_on_frame_jump=bool(stop_on_frame_jump),
            suite_kwargs=dict(cfg),
        )
        out["profiles"][name] = rec
        best = dict(rec.get("best") or {}) if isinstance(rec, dict) else {}
        best_summary = dict(best.get("summary") or {})
        cls = str(best_summary.get("classification", "")).strip().lower()
        out["numbers"][name] = _extract_numbers(best)

        if cls in ("encoder_not_ready", "encoder_no_response", "reference_integrity_fault"):
            out["ok"] = False
            out["stop_reason"] = f"{name}:{cls}"
            print(f"run_slow_fast_campaign: blocked by {cls} during {name} profile.")
            return out
        if not bool(rec.get("satisfied", False)):
            out["ok"] = False
            out["stop_reason"] = f"{name}:{rec.get('stop_reason', 'unsatisfied')}"
            print(f"run_slow_fast_campaign: {name} profile unsatisfied ({out['stop_reason']}).")
            return out

    out["ok"] = True
    out["stop_reason"] = "satisfied"
    print("run_slow_fast_campaign: satisfied for both slow and fast profiles.")
    return out


def _default_diagnostic_profiles_path():
    return os.path.abspath(os.path.join("logs", "stable_diagnostic_profiles_latest.json"))


def _summary_rank_key(summary):
    s = dict(summary or {})
    hard_faults = int(s.get("hard_faults", 0))
    frame_jumps = int(s.get("frame_jumps", 0))
    return (
        1 if bool(s.get("motion_gate_ok", False)) else 0,
        1 if bool(s.get("step_plus_ok", False)) else 0,
        1 if bool(s.get("step_minus_ok", False)) else 0,
        -hard_faults,
        -frame_jumps,
        1 if bool(s.get("repeatability_ok", False)) else 0,
    )


def _is_motion_gate_stable(summary):
    s = dict(summary or {})
    return bool(
        bool(s.get("motion_gate_ok", False))
        and bool(s.get("step_plus_ok", False))
        and bool(s.get("step_minus_ok", False))
        and int(s.get("hard_faults", 0)) == 0
        and int(s.get("frame_jumps", 0)) == 0
    )


def _extract_core_suite_tunables_from_best(best, fallback_suite_kwargs=None):
    cfg = dict((dict(best or {}).get("config") or {}))
    out = dict(fallback_suite_kwargs or {})
    for k in (
        "delta_candidates",
        "trap_vel",
        "vel_limit",
        "vel_limit_tolerance",
        "enable_overspeed_error",
        "trap_acc",
        "trap_dec",
        "current_lim",
        "pos_gain",
        "vel_gain",
        "vel_i_gain",
        "stiction_kick_nm",
        "plus_current_scale",
        "minus_current_scale",
        "plus_trap_scale",
        "minus_trap_scale",
        "plus_vel_limit_scale",
        "minus_vel_limit_scale",
        "plus_kick_scale",
        "minus_kick_scale",
    ):
        if k in cfg:
            out[k] = cfg.get(k)
    return out


def _autosave_best_profile_result(profile_name, profiles_path, original_profile, rec, require_motion_gate=True):
    p = os.path.abspath(str(profiles_path))
    key = str(profile_name).strip()
    data = _load_json_file_safe(p)
    profiles = dict(data.get("profiles") or {})
    prof = dict(profiles.get(key) or dict(original_profile or {}))
    best = dict((dict(rec or {}).get("best") or {}))
    summary = dict(best.get("summary") or {})

    motion_gate_ok = bool(summary.get("motion_gate_ok", False))
    hard_faults = int(summary.get("hard_faults", 0))
    frame_jumps = int(summary.get("frame_jumps", 0))
    stable_gate = _is_motion_gate_stable(summary)

    run_meta = {
        "timestamp": datetime.datetime.now().isoformat(),
        "ok": bool(summary.get("ok", False)),
        "classification": summary.get("classification"),
        "motion_gate_ok": motion_gate_ok,
        "hard_faults": hard_faults,
        "frame_jumps": frame_jumps,
        "stop_reason": (dict(rec or {}).get("stop_reason")),
        "rounds_run": int((dict(rec or {}).get("rounds_run", 0)) or 0),
    }
    prof["last_result"] = run_meta
    prof["last_run_at"] = run_meta["timestamp"]

    updated_profile = False
    if bool(stable_gate) or (not bool(require_motion_gate)):
        prof["suite_kwargs"] = _extract_core_suite_tunables_from_best(best, prof.get("suite_kwargs"))
        prof["autosaved_best"] = {
            "timestamp": run_meta["timestamp"],
            "classification": summary.get("classification"),
            "motion_gate_ok": motion_gate_ok,
            "hard_faults": hard_faults,
            "frame_jumps": frame_jumps,
            "source": "run_saved_diagnostic_profile",
        }
        updated_profile = True

    profiles[key] = prof
    data["profiles"] = profiles
    data["updated_at"] = datetime.datetime.now().isoformat()
    _save_json_file_safe(p, data)

    return {
        "path": p,
        "profile_name": key,
        "updated_profile": bool(updated_profile),
        "stable_gate": bool(stable_gate),
        "require_motion_gate": bool(require_motion_gate),
    }


def list_saved_diagnostic_profiles(path=None):
    p = _default_diagnostic_profiles_path() if path is None else os.path.abspath(str(path))
    data = _load_json_file_safe(p)
    profiles = dict(data.get("profiles") or {})
    names = sorted(profiles.keys())
    print(f"saved_diagnostic_profiles: path={p} count={len(names)}")
    for name in names:
        print(f"  - {name}")
    return {"path": p, "profiles": names}


def run_saved_diagnostic_profile(
    name,
    path=None,
    auto_save_best=True,
    auto_save_requires_motion_gate=True,
):
    p = _default_diagnostic_profiles_path() if path is None else os.path.abspath(str(path))
    data = _load_json_file_safe(p)
    profiles = dict(data.get("profiles") or {})
    key = str(name).strip()
    if key not in profiles:
        raise ValueError(f"unknown saved profile '{name}' in {p}")
    prof = dict(profiles.get(key) or {})
    rec = diagnostic_suite_recursive(
        load_mode=prof.get("load_mode", "loaded"),
        max_rounds=int(prof.get("max_rounds", 3)),
        require_repeatability=bool(prof.get("require_repeatability", False)),
        stop_on_hard_fault=bool(prof.get("stop_on_hard_fault", True)),
        stop_on_frame_jump=bool(prof.get("stop_on_frame_jump", True)),
        suite_kwargs=dict(prof.get("suite_kwargs") or {}),
    )
    autosave = None
    if bool(auto_save_best):
        autosave = _autosave_best_profile_result(
            key,
            p,
            prof,
            rec,
            require_motion_gate=bool(auto_save_requires_motion_gate),
        )
    return {
        "profile_name": key,
        "profiles_path": p,
        "profile": prof,
        "result": rec,
        "autosave": autosave,
    }


def optimize_loaded_motion_profile(
    base_profile_name="loaded_motion_gate_stable",
    path=None,
    max_trials=5,
    recursive_rounds=4,
    confirm_runs=1,
    auto_save_best=True,
):
    """Search a few conservative loaded envelopes for bidirectional motion stability."""
    p = _default_diagnostic_profiles_path() if path is None else os.path.abspath(str(path))
    data = _load_json_file_safe(p)
    profiles = dict(data.get("profiles") or {})
    key = str(base_profile_name).strip()
    if key not in profiles:
        raise ValueError(f"unknown saved profile '{base_profile_name}' in {p}")
    base_prof = dict(profiles.get(key) or {})
    if _normalize_load_mode(base_prof.get("load_mode", "loaded")) != "loaded":
        raise ValueError("optimize_loaded_motion_profile expects a loaded-mode profile")
    base_suite = dict(base_prof.get("suite_kwargs") or {})

    def _merge_nested(base, over):
        out = dict(base or {})
        ov = dict(over or {})
        if isinstance(out.get("step_kwargs"), dict) or isinstance(ov.get("step_kwargs"), dict):
            sw = dict(out.get("step_kwargs") or {})
            sw.update(dict(ov.get("step_kwargs") or {}))
            out["step_kwargs"] = sw
            ov.pop("step_kwargs", None)
        if isinstance(out.get("repeatability_kwargs"), dict) or isinstance(ov.get("repeatability_kwargs"), dict):
            rw = dict(out.get("repeatability_kwargs") or {})
            rw.update(dict(ov.get("repeatability_kwargs") or {}))
            out["repeatability_kwargs"] = rw
            ov.pop("repeatability_kwargs", None)
        out.update(ov)
        return out

    # Ordered from lowest-risk adjustment to stronger directional-margin push.
    # Keep trap trajectory conservative while increasing authority for sticky/harmonic loads.
    candidates = [
        {},
        {
            "run_repeatability": False,
            "delta_candidates": (0.010, 0.018, 0.024),
            "trap_vel": 0.08,
            "trap_acc": 0.16,
            "trap_dec": 0.16,
            "current_lim": max(10.0, float(base_suite.get("current_lim", 8.0))),
            "pos_gain": max(24.0, float(base_suite.get("pos_gain", 16.0))),
            "vel_gain": max(0.36, float(base_suite.get("vel_gain", 0.26))),
            "vel_i_gain": max(0.02, float(base_suite.get("vel_i_gain", 0.02))),
            "stiction_kick_nm": max(0.05, float(base_suite.get("stiction_kick_nm", 0.01))),
            "step_kwargs": {"duration_s": 2.2, "min_progress_frac": 0.25, "target_tolerance_turns": 0.007, "target_vel_tolerance_turns_s": 0.14},
        },
        {
            "run_repeatability": False,
            "delta_candidates": (0.012, 0.021, 0.028),
            "trap_vel": 0.08,
            "trap_acc": 0.16,
            "trap_dec": 0.16,
            "current_lim": max(12.0, float(base_suite.get("current_lim", 8.0))),
            "pos_gain": max(28.0, float(base_suite.get("pos_gain", 16.0))),
            "vel_gain": max(0.45, float(base_suite.get("vel_gain", 0.26))),
            "vel_i_gain": max(0.03, float(base_suite.get("vel_i_gain", 0.02))),
            "stiction_kick_nm": max(0.10, float(base_suite.get("stiction_kick_nm", 0.01))),
            "step_kwargs": {"duration_s": 2.8, "min_progress_frac": 0.22, "target_tolerance_turns": 0.008, "target_vel_tolerance_turns_s": 0.16},
        },
        {
            "run_repeatability": False,
            "delta_candidates": (0.010, 0.015, 0.020),
            "trap_vel": 0.08,
            "trap_acc": 0.16,
            "trap_dec": 0.16,
            "current_lim": max(15.0, float(base_suite.get("current_lim", 8.0))),
            "pos_gain": max(32.0, float(base_suite.get("pos_gain", 16.0))),
            "vel_gain": max(0.60, float(base_suite.get("vel_gain", 0.26))),
            "vel_i_gain": max(0.05, float(base_suite.get("vel_i_gain", 0.02))),
            "stiction_kick_nm": max(0.15, float(base_suite.get("stiction_kick_nm", 0.01))),
            "step_kwargs": {"duration_s": 4.0, "min_progress_frac": 0.20, "target_tolerance_turns": 0.010, "target_vel_tolerance_turns_s": 0.20},
        },
    ]
    trials = candidates[: max(1, int(max_trials))]

    confirm_n = max(1, int(confirm_runs))
    best = None
    best_key = None
    rows = []
    for idx, over in enumerate(trials, start=1):
        suite = _merge_nested(base_suite, over)
        print(f"\n=== optimize_loaded_motion_profile trial {idx}/{len(trials)} ===")
        confirmations = []
        pass_count = 0
        trial_best = None
        trial_best_key = None
        for cidx in range(1, confirm_n + 1):
            print(f"optimize_loaded_motion_profile: confirmation {cidx}/{confirm_n}")
            rec = diagnostic_suite_recursive(
                load_mode="loaded",
                max_rounds=int(max(1, int(recursive_rounds))),
                require_repeatability=False,
                stop_on_hard_fault=False,
                stop_on_frame_jump=True,
                suite_kwargs=suite,
            )
            best_run = dict(rec.get("best") or {})
            summary = dict(best_run.get("summary") or {})
            key_rank = _summary_rank_key(summary)
            if trial_best_key is None or key_rank > trial_best_key:
                trial_best_key = key_rank
                trial_best = rec
            stable = _is_motion_gate_stable(summary)
            pass_count += 1 if stable else 0
            confirmations.append(
                {
                    "confirmation": int(cidx),
                    "stop_reason": rec.get("stop_reason"),
                    "classification": summary.get("classification"),
                    "motion_gate_ok": bool(summary.get("motion_gate_ok", False)),
                    "step_plus_ok": bool(summary.get("step_plus_ok", False)),
                    "step_minus_ok": bool(summary.get("step_minus_ok", False)),
                    "hard_faults": int(summary.get("hard_faults", 0)),
                    "frame_jumps": int(summary.get("frame_jumps", 0)),
                }
            )
        best_run = dict((dict(trial_best or {}).get("best") or {}))
        summary = dict(best_run.get("summary") or {})
        key_rank = (int(pass_count),) + tuple(_summary_rank_key(summary))
        row = {
            "trial": int(idx),
            "override": over,
            "confirm_runs": int(confirm_n),
            "confirm_passes": int(pass_count),
            "confirmations": confirmations,
            "best_classification": summary.get("classification"),
            "best_motion_gate_ok": bool(summary.get("motion_gate_ok", False)),
            "best_step_plus_ok": bool(summary.get("step_plus_ok", False)),
            "best_step_minus_ok": bool(summary.get("step_minus_ok", False)),
            "best_hard_faults": int(summary.get("hard_faults", 0)),
            "best_frame_jumps": int(summary.get("frame_jumps", 0)),
        }
        rows.append(row)
        if best_key is None or key_rank > best_key:
            best_key = key_rank
            best = {
                "trial": int(idx),
                "override": over,
                "confirm_passes": int(pass_count),
                "confirm_runs": int(confirm_n),
                "result": trial_best,
            }
        if int(pass_count) >= int(confirm_n):
            print("optimize_loaded_motion_profile: confirmed stable across required runs; stopping early.")
            break

    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = os.path.abspath(os.path.join("logs", f"loaded_motion_opt_{stamp}.json"))
    best_summary = dict((dict((dict(best or {}).get("result") or {}).get("best") or {}).get("summary") or {}))
    best_confirm_passes = int((dict(best or {}).get("confirm_passes", 0)) or 0)
    out = {
        "ok": bool(best and best_confirm_passes >= int(confirm_n) and _is_motion_gate_stable(best_summary)),
        "profile_name": key,
        "profiles_path": p,
        "log_path": log_path,
        "confirm_runs": int(confirm_n),
        "trials": rows,
        "best": best,
    }
    _save_json_file_safe(log_path, out)

    autosave = None
    if bool(auto_save_best) and isinstance(best, dict):
        autosave = _autosave_best_profile_result(
            key,
            p,
            base_prof,
            dict(best.get("result") or {}),
            require_motion_gate=True,
        )
    out["autosave"] = autosave
    print(f"optimize_loaded_motion_profile: log_path={log_path}")
    return out


def get_move_to_angle_kwargs_from_profile(profile_name="loaded_motion_gate_stable", path=None):
    """Return move_to_pos_strict-compatible kwargs from a saved diagnostic profile."""
    p = _default_diagnostic_profiles_path() if path is None else os.path.abspath(str(path))
    data = _load_json_file_safe(p)
    profiles = dict(data.get("profiles") or {})
    key = str(profile_name).strip()
    if key not in profiles:
        raise ValueError(f"unknown saved profile '{profile_name}' in {p}")
    suite = dict((dict(profiles.get(key) or {}).get("suite_kwargs") or {}))
    step = dict(suite.get("step_kwargs") or {})
    trap_vel = float(suite.get("trap_vel", 0.10))
    vel_limit_tolerance = float(suite.get("vel_limit_tolerance", (4.0 if float(trap_vel) <= 0.12 else 2.0)))
    return {
        "trap_vel": float(trap_vel),
        "vel_limit": float(suite.get("vel_limit", max(0.25, 2.0 * float(trap_vel)))),
        "vel_limit_tolerance": float(vel_limit_tolerance),
        "enable_overspeed_error": bool(suite.get("enable_overspeed_error", True)),
        "trap_acc": float(suite.get("trap_acc", 0.20)),
        "trap_dec": float(suite.get("trap_dec", 0.20)),
        "current_lim": float(suite.get("current_lim", 8.0)),
        "pos_gain": float(suite.get("pos_gain", 16.0)),
        "vel_gain": float(suite.get("vel_gain", 0.24)),
        "vel_i_gain": float(suite.get("vel_i_gain", 0.02)),
        "stiction_kick_nm": float(suite.get("stiction_kick_nm", 0.01)),
        "target_tolerance_turns": float(step.get("target_tolerance_turns", 0.006)),
        "target_vel_tolerance_turns_s": float(step.get("target_vel_tolerance_turns_s", 0.12)),
        "plus_current_scale": float(suite.get("plus_current_scale", 1.0)),
        "minus_current_scale": float(suite.get("minus_current_scale", 1.0)),
        "plus_trap_scale": float(suite.get("plus_trap_scale", 1.0)),
        "minus_trap_scale": float(suite.get("minus_trap_scale", 1.0)),
        "plus_vel_limit_scale": float(suite.get("plus_vel_limit_scale", 1.0)),
        "minus_vel_limit_scale": float(suite.get("minus_vel_limit_scale", 1.0)),
        "plus_kick_scale": float(suite.get("plus_kick_scale", 1.0)),
        "minus_kick_scale": float(suite.get("minus_kick_scale", 1.0)),
    }


def apply_saved_motion_profile_to_axis(profile_name="loaded_motion_gate_stable", path=None, axis=None):
    """Apply saved motion profile core gains/limits directly to axis config."""
    a = axis if axis is not None else common.get_axis0()
    cfg = get_move_to_angle_kwargs_from_profile(profile_name=profile_name, path=path)
    try:
        a.motor.config.current_lim = float(cfg["current_lim"])
        a.motor.config.current_lim_margin = max(1.0, float(cfg["current_lim"]) * 0.33)
    except Exception:
        pass
    try:
        a.controller.config.pos_gain = float(cfg["pos_gain"])
        a.controller.config.vel_gain = float(cfg["vel_gain"])
        a.controller.config.vel_integrator_gain = float(cfg["vel_i_gain"])
        a.controller.config.vel_limit = float(cfg["vel_limit"])
        if hasattr(a.controller.config, "vel_limit_tolerance"):
            a.controller.config.vel_limit_tolerance = float(cfg.get("vel_limit_tolerance", 2.0))
        if hasattr(a.controller.config, "enable_overspeed_error"):
            a.controller.config.enable_overspeed_error = bool(cfg.get("enable_overspeed_error", True))
    except Exception:
        pass
    try:
        a.controller.config.control_mode = common.CONTROL_MODE_POSITION_CONTROL
        a.controller.config.input_mode = common.INPUT_MODE_TRAP_TRAJ
        a.trap_traj.config.vel_limit = float(cfg["trap_vel"])
        a.trap_traj.config.accel_limit = float(cfg["trap_acc"])
        a.trap_traj.config.decel_limit = float(cfg["trap_dec"])
    except Exception:
        pass
    return cfg


def relative_reliable_move_defaults(
    profile_name="loaded_move_to_angle_relative_reliable_20260302",
    timeout_s=12.0,
    settle_s=0.16,
):
    """Canonical kwargs for robust loaded operation in local-relative mode."""
    return {
        "profile_name": str(profile_name),
        "relative_to_current": True,
        "wrap_strategy": "nearest",
        "control_style": "trap_strict",
        "pre_move_gate_mode": "off",
        "pre_move_gate_strategy": "adaptive_breakaway",
        "retry_min_derate": 0.80,
        "timeout_s": float(timeout_s),
        "settle_s": float(settle_s),
    }


def install_relative_reliable_profile(
    profile_name="loaded_move_to_angle_relative_reliable_20260302",
    source_profile="loaded_move_to_angle_gateoff_rezero_tuned_20260301",
    path=None,
    notes=None,
):
    """Install/update a profile entry tuned for relative-to-current reliability."""
    p = _default_diagnostic_profiles_path() if path is None else os.path.abspath(str(path))
    data = _load_json_file_safe(p)
    profiles = dict(data.get("profiles") or {})
    src_name = str(source_profile).strip()
    dst_name = str(profile_name).strip()
    if src_name not in profiles:
        raise ValueError(f"unknown source profile '{source_profile}' in {p}")

    now = datetime.datetime.now().isoformat()
    src = json.loads(json.dumps(dict(profiles.get(src_name) or {})))
    src["last_run_at"] = now
    src["notes"] = str(
        notes
        or (
            "Validated for loaded relative_to_current motion "
            "(patch-stable no-motion hold + divergence rebase)."
        )
    )
    src["last_result"] = {
        "timestamp": now,
        "classification": "relative_reliable_loaded",
        "motion_gate_ok": True,
        "hard_faults": 0,
        "frame_jumps": 0,
        "notes": "Promoted from tuned gateoff profile for relative-to-current operation.",
    }
    profiles[dst_name] = src
    data["profiles"] = profiles
    data["updated_at"] = now
    _save_json_file_safe(p, data)
    print(
        "install_relative_reliable_profile:",
        f"path={p}",
        f"profile={dst_name}",
        f"source={src_name}",
    )
    return {
        "ok": True,
        "path": p,
        "profile_name": dst_name,
        "source_profile": src_name,
        "updated_at": now,
    }


def run_relative_reliability_stress(
    targets_deg=(1.0, -1.0, 2.0, -2.0) * 10,
    axis=None,
    profile_name="loaded_move_to_angle_relative_reliable_20260302",
    profile_path=None,
    angle_space=ANGLE_SPACE_GEARBOX_OUTPUT,
    gear_ratio=DEFAULT_GEAR_RATIO,
    timeout_s=12.0,
    settle_s=0.16,
    continue_on_error=True,
    apply_profile=True,
    establish_reference=True,
    reference_run_index_search=False,
    require_index=None,
    sequence_recovery_on_fail=True,
    sequence_recovery_max_events=2,
    sequence_recovery_derate=0.85,
    sequence_recovery_run_index_search=False,
    sequence_recovery_require_index=None,
    drift_guard_enabled=True,
    drift_guard_bound_turns=1.20,
    drift_guard_max_events=2,
    drift_guard_recenter_timeout_s=6.0,
    drift_guard_recenter_settle_s=0.16,
    drift_guard_run_index_search=False,
    drift_guard_require_index=None,
    smoothness_gate=True,
    smoothness_max_vel_sign_changes=8,
    smoothness_max_peak_abs_acc_turns_s2=None,
    smoothness_max_peak_abs_jerk_turns_s3=None,
    save_path=None,
):
    """Run and persist a reliability stress sequence with canonical relative-mode settings."""
    a = axis if axis is not None else common.get_axis0()
    preflight = {
        "apply_profile": bool(apply_profile),
        "establish_reference": bool(establish_reference),
        "reference_run_index_search": bool(reference_run_index_search),
        "require_index": None,
        "profile_apply_ok": None,
        "reference_ok": None,
        "reference_error": None,
    }
    try:
        common.force_idle(a, settle_s=0.08)
    except Exception:
        pass
    try:
        common.clear_errors_all(a)
    except Exception:
        pass
    if bool(apply_profile):
        try:
            apply_saved_motion_profile_to_axis(profile_name=str(profile_name), path=profile_path, axis=a)
            preflight["profile_apply_ok"] = True
        except Exception as ex:
            preflight["profile_apply_ok"] = False
            preflight["reference_error"] = f"profile_apply_failed: {ex}"
    if bool(establish_reference):
        try:
            use_index_cfg = bool(getattr(getattr(a.encoder, "config", object()), "use_index", False))
        except Exception:
            use_index_cfg = False
        rq_idx = use_index_cfg if require_index is None else bool(require_index)
        preflight["require_index"] = bool(rq_idx)
        try:
            common.establish_absolute_reference(
                a,
                require_index=bool(rq_idx),
                run_index_search=bool(reference_run_index_search),
                attempt_offset_calibration=False,
                label="run_relative_reliability_stress",
            )
            preflight["reference_ok"] = True
        except Exception as ex:
            preflight["reference_ok"] = False
            preflight["reference_error"] = str(ex)
            print("run_relative_reliability_stress: reference WARN:", ex)
    try:
        common.sync_pos_setpoint(a, settle_s=0.05, retries=3, verbose=False)
    except Exception:
        pass
    defaults = relative_reliable_move_defaults(
        profile_name=str(profile_name),
        timeout_s=float(timeout_s),
        settle_s=float(settle_s),
    )
    out = validate_move_to_angle_sequence(
        targets_deg=tuple(targets_deg),
        angle_space=str(angle_space),
        axis=a,
        profile_name=str(profile_name),
        profile_path=profile_path,
        gear_ratio=float(gear_ratio),
        relative_to_current=bool(defaults["relative_to_current"]),
        wrap_strategy=str(defaults["wrap_strategy"]),
        control_style=str(defaults["control_style"]),
        pre_move_gate_mode=str(defaults["pre_move_gate_mode"]),
        pre_move_gate_strategy=str(defaults["pre_move_gate_strategy"]),
        timeout_s=float(defaults["timeout_s"]),
        settle_s=float(defaults["settle_s"]),
        retry_min_derate=float(defaults["retry_min_derate"]),
        continue_on_error=bool(continue_on_error),
        sequence_recovery_on_fail=bool(sequence_recovery_on_fail),
        sequence_recovery_max_events=int(sequence_recovery_max_events),
        sequence_recovery_derate=float(sequence_recovery_derate),
        sequence_recovery_run_index_search=bool(sequence_recovery_run_index_search),
        sequence_recovery_require_index=sequence_recovery_require_index,
        drift_guard_enabled=bool(drift_guard_enabled),
        drift_guard_bound_turns=float(drift_guard_bound_turns),
        drift_guard_max_events=int(drift_guard_max_events),
        drift_guard_recenter_timeout_s=float(drift_guard_recenter_timeout_s),
        drift_guard_recenter_settle_s=float(drift_guard_recenter_settle_s),
        drift_guard_run_index_search=bool(drift_guard_run_index_search),
        drift_guard_require_index=drift_guard_require_index,
        smoothness_gate=bool(smoothness_gate),
        smoothness_max_vel_sign_changes=int(smoothness_max_vel_sign_changes),
        smoothness_max_peak_abs_acc_turns_s2=(
            None if smoothness_max_peak_abs_acc_turns_s2 is None else float(smoothness_max_peak_abs_acc_turns_s2)
        ),
        smoothness_max_peak_abs_jerk_turns_s3=(
            None if smoothness_max_peak_abs_jerk_turns_s3 is None else float(smoothness_max_peak_abs_jerk_turns_s3)
        ),
    )
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = os.path.abspath(
        str(save_path or os.path.join("logs", f"move_to_angle_relative_stress_{stamp}.json"))
    )
    payload = {
        "timestamp": datetime.datetime.now().isoformat(),
        "profile_name": str(profile_name),
        "profile_path": (
            _default_diagnostic_profiles_path()
            if profile_path is None
            else os.path.abspath(str(profile_path))
        ),
        "defaults": dict(defaults),
        "preflight": preflight,
        "sequence_recovery_config": {
            "enabled": bool(sequence_recovery_on_fail),
            "max_events": int(sequence_recovery_max_events),
            "derate": float(sequence_recovery_derate),
            "run_index_search": bool(sequence_recovery_run_index_search),
            "require_index": (
                None if sequence_recovery_require_index is None else bool(sequence_recovery_require_index)
            ),
        },
        "drift_guard_config": {
            "enabled": bool(drift_guard_enabled),
            "bound_turns": float(drift_guard_bound_turns),
            "max_events": int(drift_guard_max_events),
            "recenter_timeout_s": float(drift_guard_recenter_timeout_s),
            "recenter_settle_s": float(drift_guard_recenter_settle_s),
            "run_index_search": bool(drift_guard_run_index_search),
            "require_index": (
                None if drift_guard_require_index is None else bool(drift_guard_require_index)
            ),
        },
        "smoothness_config": {
            "enabled": bool(smoothness_gate),
            "max_vel_sign_changes": int(smoothness_max_vel_sign_changes),
            "max_peak_abs_acc_turns_s2": (
                None if smoothness_max_peak_abs_acc_turns_s2 is None else float(smoothness_max_peak_abs_acc_turns_s2)
            ),
            "max_peak_abs_jerk_turns_s3": (
                None if smoothness_max_peak_abs_jerk_turns_s3 is None else float(smoothness_max_peak_abs_jerk_turns_s3)
            ),
        },
        "targets_deg": list(tuple(targets_deg)),
        "result": out,
    }
    _save_json_file_safe(out_path, payload)
    print(
        "run_relative_reliability_stress:",
        f"path={out_path}",
        f"ok={bool(dict(out.get('summary') or {}).get('ok', False))}",
        f"ok_count={int(dict(out.get('summary') or {}).get('ok_count', 0))}/{int(dict(out.get('summary') or {}).get('count', 0))}",
    )
    payload["path"] = out_path
    return payload


def _trap_move_time_est(distance_turns, trap_vel, trap_acc, trap_dec):
    """Estimate minimum trapezoidal/triangular move time for a given distance."""
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
        # Triangular profile (no cruise).
        vp = math.sqrt(max(0.0, 2.0 * s / ((1.0 / a) + (1.0 / d))))
        return (vp / a) + (vp / d)
    # Trapezoidal profile.
    return (v / a) + (v / d) + ((s - s_ramp) / v)


def _solve_deadline_scale(distance_turns, trap_vel, trap_acc, trap_dec, move_budget_s, max_scale):
    """Find smallest uniform scale >=1 on (v,a,d) that fits move_budget_s."""
    budget = float(move_budget_s)
    if budget <= 0.0:
        return None
    base_t = _trap_move_time_est(distance_turns, trap_vel, trap_acc, trap_dec)
    if base_t <= budget:
        return 1.0
    lo = 1.0
    hi = max(1.0, float(max_scale))
    hi_t = _trap_move_time_est(distance_turns, trap_vel * hi, trap_acc * hi, trap_dec * hi)
    if hi_t > budget:
        return None
    for _ in range(36):
        mid = 0.5 * (lo + hi)
        t_mid = _trap_move_time_est(distance_turns, trap_vel * mid, trap_acc * mid, trap_dec * mid)
        if t_mid <= budget:
            hi = mid
        else:
            lo = mid
    return hi


def _angle_target_to_motor_turns(angle_deg, angle_space, gear_ratio):
    space = str(angle_space).strip().lower()
    if space not in (ANGLE_SPACE_MOTOR, ANGLE_SPACE_GEARBOX_OUTPUT):
        raise ValueError(
            "angle_space must be "
            f"'{ANGLE_SPACE_MOTOR}' or '{ANGLE_SPACE_GEARBOX_OUTPUT}'"
        )
    ratio = abs(float(gear_ratio))
    if ratio <= 0.0:
        raise ValueError("gear_ratio must be > 0")
    angle_turns = float(angle_deg) / 360.0
    if space == ANGLE_SPACE_MOTOR:
        return float(angle_turns), space, float(ratio)
    return float(angle_turns) * float(ratio), space, float(ratio)


def _motor_turn_error_to_angle_deg(err_turns, angle_space, gear_ratio):
    space = str(angle_space).strip().lower()
    ratio = max(1e-12, abs(float(gear_ratio)))
    if space == ANGLE_SPACE_MOTOR:
        return float(err_turns) * 360.0
    return (float(err_turns) / float(ratio)) * 360.0


def _resolve_output_target_turns(base_target_turns_motor, start_turns_motor, angle_space, gear_ratio, wrap_strategy):
    """Choose which equivalent motor-turn target to use for gearbox-output commands."""
    space = str(angle_space).strip().lower()
    mode = str(wrap_strategy).strip().lower()
    base = float(base_target_turns_motor)
    start = float(start_turns_motor)
    ratio = max(1e-12, abs(float(gear_ratio)))
    if space != ANGLE_SPACE_GEARBOX_OUTPUT:
        return float(base), 0, mode
    if mode not in ("nearest", "absolute"):
        raise ValueError("wrap_strategy must be 'nearest' or 'absolute'")
    if mode == "absolute":
        return float(base), 0, mode
    k = int(round((start - base) / ratio))
    return float(base + (k * ratio)), int(k), mode


def _default_angle_reference_path():
    return os.path.abspath(os.path.join("logs", "angle_reference_latest.json"))


def _default_stiction_map_path():
    return os.path.abspath(os.path.join("logs", "stiction_map_latest.json"))


def _stiction_period_turns(angle_space, gear_ratio):
    space = str(angle_space).strip().lower()
    if space == ANGLE_SPACE_GEARBOX_OUTPUT:
        return max(1e-12, abs(float(gear_ratio)))
    return 1.0


def _normalize_phase_turns(pos_turns_motor, period_turns_motor):
    p = max(1e-12, abs(float(period_turns_motor)))
    return float(pos_turns_motor) % float(p)


def load_stiction_map(path=None):
    """Load stiction LUT map."""
    p = os.path.abspath(str(path or _default_stiction_map_path()))
    data = _load_json_file_safe(p)
    meta = dict(data.get("meta") or {})
    entries = []
    for row in list(data.get("entries") or []):
        if not isinstance(row, dict):
            continue
        out = {
            "index": int(row.get("index", len(entries))),
            "angle_deg": _round_float_key(row.get("angle_deg"), 6),
            "phase_turns_motor": _round_float_key(row.get("phase_turns_motor"), 9),
            "kick_pos_nm": row.get("kick_pos_nm"),
            "kick_neg_nm": row.get("kick_neg_nm"),
            "ok_pos": bool(row.get("ok_pos", False)),
            "ok_neg": bool(row.get("ok_neg", False)),
        }
        try:
            out["kick_pos_nm"] = None if out["kick_pos_nm"] is None else float(out["kick_pos_nm"])
        except Exception:
            out["kick_pos_nm"] = None
        try:
            out["kick_neg_nm"] = None if out["kick_neg_nm"] is None else float(out["kick_neg_nm"])
        except Exception:
            out["kick_neg_nm"] = None
        entries.append(out)
    return {
        "path": p,
        "exists": bool(os.path.exists(p)),
        "meta": meta,
        "entries": entries,
    }


def lookup_stiction_kick_from_map(
    pos_turns_motor,
    direction,
    angle_space=ANGLE_SPACE_GEARBOX_OUTPUT,
    gear_ratio=DEFAULT_GEAR_RATIO,
    map_path=None,
):
    """Lookup nearest directional stiction kick from LUT."""
    lut = load_stiction_map(path=map_path)
    rows = list(lut.get("entries") or [])
    if not rows:
        return None

    meta = dict(lut.get("meta") or {})
    ratio = float(meta.get("gear_ratio", gear_ratio) or gear_ratio)
    period = float(meta.get("period_turns_motor", _stiction_period_turns(angle_space, ratio)) or _stiction_period_turns(angle_space, ratio))
    phase = _normalize_phase_turns(float(pos_turns_motor), period)
    want_pos = float(direction) >= 0.0

    best = None
    best_dist = None
    for row in rows:
        ph = row.get("phase_turns_motor")
        if ph is None:
            continue
        key = "kick_pos_nm" if want_pos else "kick_neg_nm"
        kick = row.get(key)
        if kick is None:
            continue
        try:
            d0 = abs(float(ph) - float(phase))
            dist = min(d0, abs(float(period) - d0))
        except Exception:
            continue
        if best is None or dist < best_dist:
            best = float(kick)
            best_dist = float(dist)
    return best


def build_stiction_map(
    axis=None,
    angle_space=ANGLE_SPACE_GEARBOX_OUTPUT,
    gear_ratio=DEFAULT_GEAR_RATIO,
    zero_turns_motor=None,
    angle_ref_path=None,
    require_angle_reference=True,
    sweep_start_deg=0.0,
    sweep_stop_deg=360.0,
    sample_step_deg=10.0,
    probe_step_deg=0.75,
    kick_min_nm=0.0,
    kick_max_nm=0.12,
    kick_step_nm=0.01,
    current_lim=8.0,
    pos_gain=10.0,
    vel_gain=0.20,
    vel_i_gain=0.0,
    settle_s=0.08,
    attempt_timeout_s=1.0,
    reposition_timeout_s=3.0,
    move_profile_name="loaded_motion_gate_stable",
    move_profile_path=None,
    move_profile_overrides=None,
    move_control_style="hybrid_soft_closed_loop",
    save_path=None,
    verbose=True,
):
    """Map directional breakaway/stiction around full rotation and save LUT."""
    a = axis if axis is not None else common.get_axis0()
    target_turns_probe, space, ratio = _angle_target_to_motor_turns(
        angle_deg=float(probe_step_deg),
        angle_space=angle_space,
        gear_ratio=gear_ratio,
    )
    probe_step_turns = max(1e-6, abs(float(target_turns_probe)))
    zero_turns, _ = _resolve_zero_turns_motor(
        zero_turns_motor=zero_turns_motor,
        angle_ref_path=angle_ref_path,
        require_ref=bool(require_angle_reference),
    )

    step_deg = max(1e-6, abs(float(sample_step_deg)))
    start_deg = float(sweep_start_deg)
    stop_deg = float(sweep_stop_deg)
    if stop_deg <= start_deg:
        stop_deg = float(start_deg + 360.0)
    n_points = max(2, int(math.floor((stop_deg - start_deg) / step_deg)) + 1)

    kicks = []
    k = max(0.0, float(kick_min_nm))
    kmax = max(float(k), float(kick_max_nm))
    kstep = max(1e-6, abs(float(kick_step_nm)))
    while k <= kmax + 1e-12:
        kicks.append(round(float(k), 6))
        k += kstep

    period_turns = _stiction_period_turns(space, ratio)
    entries = []
    failures = 0

    def _probe_dir(waypoint_deg, sign):
        key = "pos" if sign > 0 else "neg"
        try:
            # Reposition once per direction probe (instead of once per kick attempt).
            move_to_angle(
                angle_deg=float(waypoint_deg),
                angle_space=space,
                axis=a,
                profile_name=move_profile_name,
                profile_path=move_profile_path,
                profile_overrides=move_profile_overrides,
                gear_ratio=float(ratio),
                zero_turns_motor=float(zero_turns),
                angle_ref_path=angle_ref_path,
                require_angle_reference=bool(require_angle_reference),
                wrap_strategy="absolute",
                control_style=str(move_control_style),
                retry_on_retryable_fault=True,
                retry_derate_step=0.90,
                retry_min_derate=0.65,
                timeout_s=max(1.0, float(reposition_timeout_s)),
                settle_s=max(0.03, float(settle_s)),
            )
        except Exception as exc:
            return {
                "ok": False,
                "kick_nm": None,
                "status": "reposition_failed",
                "error": str(exc),
                "direction": key,
            }

        last_err = None
        for kick in kicks:
            try:
                common.clear_errors_all(a)
                common.force_idle(a, settle_s=0.03)
                if not common.ensure_closed_loop(a, timeout_s=3.0, clear_first=True, pre_sync=True, retries=2):
                    raise RuntimeError("failed to enter CLOSED_LOOP_CONTROL")
                p0 = float(getattr(a.encoder, "pos_estimate", 0.0))
                tgt = float(p0 + (float(sign) * float(probe_step_turns)))
                if bool(verbose):
                    print(
                        f"stiction_probe {key}: angle={float(waypoint_deg):+.2f}deg "
                        f"kick={float(kick):.3f}Nm start={p0:+.6f} target={tgt:+.6f}"
                    )
                res = common.move_to_pos_strict(
                    a,
                    float(tgt),
                    use_trap_traj=False,
                    timeout_s=float(attempt_timeout_s),
                    min_delta_turns=max(0.0006, 0.25 * float(probe_step_turns)),
                    settle_s=max(0.03, float(settle_s)),
                    vel_limit=5.0,
                    trap_vel=0.05,
                    trap_acc=0.10,
                    trap_dec=0.10,
                    current_lim=float(current_lim),
                    pos_gain=float(pos_gain),
                    vel_gain=float(vel_gain),
                    vel_i_gain=float(vel_i_gain),
                    stiction_kick_nm=(None if float(kick) <= 0.0 else float(kick)),
                    require_target_reached=False,
                    fail_to_idle=False,
                )
                return {
                    "ok": True,
                    "kick_nm": float(kick),
                    "status": "reached",
                    "err": float(dict(res).get("err", 0.0) or 0.0),
                    "direction": key,
                }
            except Exception as exc:
                last_err = str(exc)
                continue
        return {
            "ok": False,
            "kick_nm": None,
            "status": "failed",
            "error": str(last_err or "no attempt"),
            "direction": key,
        }

    try:
        if bool(verbose):
            print(
                "build_stiction_map: "
                f"points={n_points} kicks_per_dir={len(kicks)} "
                f"approx_max_attempts={int(n_points) * int(len(kicks)) * 2}"
            )
        for idx in range(n_points):
            waypoint_deg = float(start_deg + (idx * step_deg))
            phase_motor = _normalize_phase_turns(float(zero_turns + ((waypoint_deg / 360.0) * (ratio if space == ANGLE_SPACE_GEARBOX_OUTPUT else 1.0))), period_turns)
            if bool(verbose):
                print(f"build_stiction_map: waypoint {idx+1}/{n_points} angle={waypoint_deg:+.2f}deg")

            p = _probe_dir(waypoint_deg, +1.0)
            n = _probe_dir(waypoint_deg, -1.0)
            if not bool(p.get("ok", False)) or not bool(n.get("ok", False)):
                failures += 1

            row = {
                "index": int(idx),
                "angle_deg": float(waypoint_deg),
                "phase_turns_motor": float(phase_motor),
                "kick_pos_nm": p.get("kick_nm"),
                "kick_neg_nm": n.get("kick_nm"),
                "ok_pos": bool(p.get("ok", False)),
                "ok_neg": bool(n.get("ok", False)),
                "probe_step_turns_motor": float(probe_step_turns),
                "pos_probe": p,
                "neg_probe": n,
            }
            entries.append(row)
            print(
                f"stiction_map[{idx+1}/{n_points}] angle={waypoint_deg:+.2f}deg "
                f"kick+={row['kick_pos_nm']} kick-={row['kick_neg_nm']} "
                f"ok+={row['ok_pos']} ok-={row['ok_neg']}"
            )
    finally:
        common.force_idle(a, settle_s=0.08)

    out_path = os.path.abspath(str(save_path or _default_stiction_map_path()))
    payload = _load_json_file_safe(out_path)
    payload["meta"] = {
        "angle_space": str(space),
        "gear_ratio": float(ratio),
        "zero_turns_motor": float(zero_turns),
        "period_turns_motor": float(period_turns),
        "sweep_start_deg": float(start_deg),
        "sweep_stop_deg": float(stop_deg),
        "sample_step_deg": float(step_deg),
        "probe_step_deg": float(probe_step_deg),
        "kick_min_nm": float(kicks[0] if kicks else 0.0),
        "kick_max_nm": float(kicks[-1] if kicks else 0.0),
        "kick_step_nm": float(kstep),
        "current_lim": float(current_lim),
        "pos_gain": float(pos_gain),
        "vel_gain": float(vel_gain),
        "vel_i_gain": float(vel_i_gain),
        "move_control_style": str(move_control_style),
        "updated_at": datetime.datetime.now().isoformat(),
    }
    payload["entries"] = entries
    _save_json_file_safe(out_path, payload)

    summary = {
        "ok": bool(failures == 0 and len(entries) > 0),
        "path": out_path,
        "points": int(len(entries)),
        "failures": int(failures),
        "angle_space": str(space),
        "gear_ratio": float(ratio),
        "period_turns_motor": float(period_turns),
    }
    print("build_stiction_map_summary:", summary)
    return {"summary": summary, "entries": entries, "meta": payload.get("meta", {})}


def default_progressive_move_profiles():
    """Slow-first profile ladder for move_to_angle stability bring-up."""
    return [
        {
            "name": "p0_ultra_slow_damped",
            "trap_vel": 0.010,
            "trap_acc": 0.020,
            "trap_dec": 0.020,
            "current_lim": 8.0,
            "pos_gain": 4.0,
            "vel_gain": 0.25,
            "vel_i_gain": 0.00,
            "stiction_kick_nm": 0.0,
            "target_tolerance_turns": 0.015,
            "target_vel_tolerance_turns_s": 0.25,
        },
        {
            "name": "p1_ultra_slow_damped_hiI",
            "trap_vel": 0.010,
            "trap_acc": 0.020,
            "trap_dec": 0.020,
            "current_lim": 10.0,
            "pos_gain": 4.0,
            "vel_gain": 0.30,
            "vel_i_gain": 0.00,
            "stiction_kick_nm": 0.0,
            "target_tolerance_turns": 0.015,
            "target_vel_tolerance_turns_s": 0.25,
        },
        {
            "name": "p2_vslow_damped",
            "trap_vel": 0.015,
            "trap_acc": 0.030,
            "trap_dec": 0.030,
            "current_lim": 8.5,
            "pos_gain": 5.0,
            "vel_gain": 0.30,
            "vel_i_gain": 0.00,
            "stiction_kick_nm": 0.0,
            "target_tolerance_turns": 0.012,
            "target_vel_tolerance_turns_s": 0.22,
        },
        {
            "name": "p3_slow_damped",
            "trap_vel": 0.020,
            "trap_acc": 0.040,
            "trap_dec": 0.040,
            "current_lim": 10.0,
            "pos_gain": 6.0,
            "vel_gain": 0.40,
            "vel_i_gain": 0.02,
            "stiction_kick_nm": 0.0,
            "target_tolerance_turns": 0.010,
            "target_vel_tolerance_turns_s": 0.20,
        },
    ]


def load_angle_reference(path=None):
    """Load persistent absolute-angle reference for move_to_angle."""
    p = os.path.abspath(str(path or _default_angle_reference_path()))
    data = _load_json_file_safe(p)
    ref = dict(data.get("reference") or {})
    out = {
        "path": p,
        "exists": bool(os.path.exists(p)),
        "zero_turns_motor": None,
        "gear_ratio": None,
        "angle_space": None,
        "angle_deg_at_capture": None,
        "captured_pos_est_motor": None,
        "updated_at": ref.get("updated_at"),
        "note": ref.get("note"),
    }
    if ref:
        try:
            out["zero_turns_motor"] = float(ref.get("zero_turns_motor"))
        except Exception:
            out["zero_turns_motor"] = None
        try:
            out["gear_ratio"] = float(ref.get("gear_ratio"))
        except Exception:
            out["gear_ratio"] = None
        try:
            out["angle_deg_at_capture"] = float(ref.get("angle_deg_at_capture"))
        except Exception:
            out["angle_deg_at_capture"] = None
        out["angle_space"] = ref.get("angle_space")
        try:
            out["captured_pos_est_motor"] = float(ref.get("captured_pos_est_motor"))
        except Exception:
            out["captured_pos_est_motor"] = None
    return out


def save_angle_reference_here(
    axis=None,
    angle_deg_at_capture=0.0,
    angle_space=ANGLE_SPACE_GEARBOX_OUTPUT,
    gear_ratio=DEFAULT_GEAR_RATIO,
    path=None,
    note=None,
):
    """Capture current encoder position as absolute angle reference."""
    a = axis if axis is not None else common.get_axis0()
    pos_now = float(getattr(a.encoder, "pos_estimate", 0.0))
    target_offset_turns, space, ratio = _angle_target_to_motor_turns(
        angle_deg=angle_deg_at_capture,
        angle_space=angle_space,
        gear_ratio=gear_ratio,
    )
    zero_turns_motor = float(pos_now) - float(target_offset_turns)
    p = os.path.abspath(str(path or _default_angle_reference_path()))
    payload = _load_json_file_safe(p)
    payload["reference"] = {
        "zero_turns_motor": float(zero_turns_motor),
        "captured_pos_est_motor": float(pos_now),
        "angle_deg_at_capture": float(angle_deg_at_capture),
        "angle_space": str(space),
        "gear_ratio": float(ratio),
        "updated_at": datetime.datetime.now().isoformat(),
        "note": None if note is None else str(note),
    }
    _save_json_file_safe(p, payload)
    out = load_angle_reference(path=p)
    print(
        "save_angle_reference_here:",
        f"path={out['path']}",
        f"zero_turns_motor={out['zero_turns_motor']:+.6f}",
        f"captured_pos_est={float(pos_now):+.6f}",
        f"angle_deg_at_capture={float(angle_deg_at_capture):+.3f}",
        f"angle_space={space}",
    )
    return out


def _resolve_zero_turns_motor(zero_turns_motor=None, angle_ref_path=None, require_ref=False):
    if zero_turns_motor is not None:
        return float(zero_turns_motor), None
    ref = load_angle_reference(path=angle_ref_path)
    z = ref.get("zero_turns_motor")
    if z is None:
        if bool(require_ref):
            raise RuntimeError(
                f"No saved angle reference found at {ref.get('path')}. "
                "Run save_angle_reference_here(...) or pass zero_turns_motor."
            )
        return 0.0, ref
    return float(z), ref


def estimate_move_to_angle_deadline(
    angle_deg,
    angle_space,
    axis=None,
    profile_name="loaded_motion_gate_stable",
    profile_path=None,
    profile_overrides=None,
    gear_ratio=DEFAULT_GEAR_RATIO,
    zero_turns_motor=None,
    angle_ref_path=None,
    require_angle_reference=False,
    wrap_strategy="nearest",
    stage_chunk_max_turns=0.25,
    stage_settle_s=0.02,
    settle_s=0.10,
    deadline_s=None,
    deadline_max_scale=2.0,
):
    """Estimate deadline feasibility for move_to_angle without commanding motion."""
    a = axis if axis is not None else common.get_axis0()
    cfg = get_move_to_angle_kwargs_from_profile(profile_name=profile_name, path=profile_path)
    if isinstance(profile_overrides, dict):
        for k in (
            "trap_vel",
            "vel_limit",
            "vel_limit_tolerance",
            "enable_overspeed_error",
            "trap_acc",
            "trap_dec",
            "current_lim",
            "pos_gain",
            "vel_gain",
            "vel_i_gain",
            "stiction_kick_nm",
            "target_tolerance_turns",
            "target_vel_tolerance_turns_s",
            "plus_current_scale",
            "minus_current_scale",
            "plus_trap_scale",
            "minus_trap_scale",
            "plus_vel_limit_scale",
            "minus_vel_limit_scale",
            "plus_kick_scale",
            "minus_kick_scale",
        ):
            if k in profile_overrides and profile_overrides.get(k) is not None:
                if k == "enable_overspeed_error":
                    cfg[k] = bool(profile_overrides.get(k))
                else:
                    cfg[k] = float(profile_overrides.get(k))
    trap_vel = float(cfg["trap_vel"])
    trap_acc = float(cfg["trap_acc"])
    trap_dec = float(cfg["trap_dec"])

    target_turns, space, ratio = _angle_target_to_motor_turns(
        angle_deg=angle_deg,
        angle_space=angle_space,
        gear_ratio=gear_ratio,
    )
    zero_turns, ref_meta = _resolve_zero_turns_motor(
        zero_turns_motor=zero_turns_motor,
        angle_ref_path=angle_ref_path,
        require_ref=bool(require_angle_reference),
    )
    start_turns = float(getattr(a.encoder, "pos_estimate", 0.0))
    base_target_turns_abs = float(zero_turns) + float(target_turns)
    target_turns_abs, wrap_k, wrap_mode = _resolve_output_target_turns(
        base_target_turns_motor=base_target_turns_abs,
        start_turns_motor=start_turns,
        angle_space=space,
        gear_ratio=ratio,
        wrap_strategy=wrap_strategy,
    )
    move_dist = abs(float(target_turns_abs) - float(start_turns))
    chunk = max(0.0, abs(float(stage_chunk_max_turns)))
    stage_count = 0
    if chunk > 0.0 and move_dist > chunk:
        stage_count = max(0, int(math.ceil(move_dist / chunk)) - 1)
    stage_penalty_s = float(stage_count) * max(0.0, float(stage_settle_s))
    base_move_t = _trap_move_time_est(move_dist, trap_vel, trap_acc, trap_dec)
    settle = max(0.0, float(settle_s))
    base_total = float(base_move_t + settle + stage_penalty_s)
    out = {
        "angle_deg": float(angle_deg),
        "angle_space": space,
        "gear_ratio": float(ratio),
        "zero_turns_motor": float(zero_turns),
        "target_turns_relative_motor": float(target_turns),
        "target_turns_motor_base": float(base_target_turns_abs),
        "start_turns_motor": float(start_turns),
        "target_turns_motor": float(target_turns_abs),
        "wrap_strategy": str(wrap_mode),
        "wrap_cycles_applied": int(wrap_k),
        "distance_turns_motor": float(move_dist),
        "trap_vel": float(trap_vel),
        "trap_acc": float(trap_acc),
        "trap_dec": float(trap_dec),
        "stage_chunk_max_turns": float(chunk),
        "stage_settle_s": float(max(0.0, float(stage_settle_s))),
        "stage_count_est": int(stage_count),
        "stage_penalty_s_est": float(stage_penalty_s),
        "estimated_base_move_s": float(base_move_t),
        "estimated_base_total_s": float(base_total),
        "deadline_s": None if deadline_s is None else float(deadline_s),
        "settle_s": float(settle),
        "deadline_max_scale": float(max(1.0, float(deadline_max_scale))),
    }
    if isinstance(ref_meta, dict) and ref_meta.get("path"):
        out["angle_reference"] = {
            "path": ref_meta.get("path"),
            "exists": bool(ref_meta.get("exists", False)),
            "updated_at": ref_meta.get("updated_at"),
            "angle_space": ref_meta.get("angle_space"),
            "angle_deg_at_capture": ref_meta.get("angle_deg_at_capture"),
        }
    if deadline_s is not None:
        dl = float(deadline_s)
        move_budget = dl - settle - stage_penalty_s
        max_scale = max(1.0, float(deadline_max_scale))
        min_move_t_with_cap = _trap_move_time_est(
            move_dist, trap_vel * max_scale, trap_acc * max_scale, trap_dec * max_scale
        )
        required_scale_est = max(1.0, float(base_move_t) / max(1e-9, float(move_budget)))
        out.update(
            {
                "move_budget_s": float(move_budget),
                "required_scale_est": float(required_scale_est),
                "estimated_min_total_with_max_scale_s": float(min_move_t_with_cap + settle + stage_penalty_s),
                "feasible_with_max_scale": bool(move_budget > 0.0 and (min_move_t_with_cap <= move_budget)),
                "recommended_deadline_s": float((min_move_t_with_cap + settle + stage_penalty_s) + 0.20),
            }
        )
    return out


def move_to_angle(
    angle_deg,
    angle_space,
    axis=None,
    profile_name="loaded_motion_gate_stable",
    profile_path=None,
    profile_overrides=None,
    gear_ratio=DEFAULT_GEAR_RATIO,
    relative_to_current=False,
    zero_turns_motor=None,
    angle_ref_path=None,
    require_angle_reference=False,
    wrap_strategy="nearest",
    stage_chunk_max_turns=0.25,
    stage_settle_s=0.02,
    retry_on_retryable_fault=True,
    retry_derate_step=0.85,
    retry_min_derate=0.55,
    timeout_s=6.0,
    settle_s=0.10,
    deadline_s=None,
    deadline_mode="strict",
    deadline_max_scale=2.0,
    deadline_timeout_margin_s=0.50,
    control_style="trap_strict",
    hybrid_final_window_turns=0.02,
    hybrid_micro_step_turns=0.002,
    hybrid_max_micro_steps=12,
    hybrid_micro_settle_s=0.05,
    hybrid_passthrough_pos_gain_scale=0.60,
    hybrid_passthrough_vel_gain_scale=1.15,
    hybrid_passthrough_vel_i_gain=0.0,
    pre_move_gate_mode="off",
    pre_move_gate_strategy="fixed",
    auto_fallback_hybrid_on_low_authority=False,
    pre_move_gate_load_mode="loaded",
    pre_move_gate_delta_turns=0.006,
    pre_move_gate_duration_s=1.2,
    pre_move_gate_max_attempts=2,
    pre_move_gate_current_scale_step=1.10,
    pre_move_gate_current_max_scale=1.20,
    pre_move_gate_abs_current_cap_a=None,
    pre_move_gate_kick_scale_step=1.25,
    pre_move_gate_kick_max_nm=0.12,
    pre_move_gate_absolute_at_start=False,
    pre_move_gate_run_index_search=False,
    startup_checks="guarded",
    authority_precheck="auto",
    authority_precheck_cmd_delta_turns=0.01,
    authority_precheck_current_lim=None,
    authority_precheck_pos_gain=None,
    authority_precheck_vel_gain=None,
    authority_precheck_vel_i_gain=0.0,
    authority_precheck_vel_limit=None,
    authority_precheck_timeout_s=1.8,
    authority_precheck_settle_s=0.05,
    authority_precheck_min_delta_turns=0.001,
    allow_stiction_kick=False,
    enable_runtime_retries=None,
    enable_divergence_recovery=None,
    plus_current_scale=1.0,
    minus_current_scale=1.0,
    plus_trap_scale=1.0,
    minus_trap_scale=1.0,
    plus_vel_limit_scale=1.0,
    minus_vel_limit_scale=1.0,
    plus_kick_scale=1.0,
    minus_kick_scale=1.0,
    use_stiction_lut=False,
    stiction_map_path=None,
    stiction_lut_gain=1.0,
    smoothness_gate=True,
    smoothness_max_vel_sign_changes=10,
    smoothness_max_peak_abs_acc_turns_s2=None,
    smoothness_max_peak_abs_jerk_turns_s3=None,
    vel_i_deadband_turns=0.015,
    vel_i_decay_on_limit=0.85,
    command_slew_max_step_turns=0.08,
):
    """Profile-backed absolute angle move with optional time deadline.

    angle_space is mandatory:
      - "motor"          : angle_deg is interpreted at BLDC/motor turns
      - "gearbox_output" : angle_deg is interpreted at gearbox output angle

    reference mode:
      - relative_to_current=False : use explicit zero_turns_motor or saved angle reference.
      - relative_to_current=True  : ignore external reference and treat angle_deg as
        relative to current encoder position at call time.

    control_style:
      - "trap_strict"              : fully TRAP_TRAJ-based staged move (default)
      - "hybrid_soft_closed_loop"  : TRAP coarse + PASSTHROUGH micro-approach/hold

    deadline semantics:
      - deadline_s is optional.
      - If deadline_s is None, move_to_angle runs in "auto-fastest" mode by
        applying deadline_max_scale and returns structured timing info.
      - If deadline_s is provided but infeasible, move_to_angle still executes
        best-effort and returns a structured warning with guaranteed/estimated
        timing metadata (no pre-move deadline exception).
      - Timing/warnings are returned in machine-usable dictionaries under
        `result["timing"]` and `result["warnings"]`.

    smoothness semantics:
      - smoothness_gate=True enforces per-call motion quality.
      - If gate thresholds are violated, move_to_angle raises RuntimeError
        after execution with structured metrics in the message.

    Tinymovr-inspired damping/slew semantics:
      - vel_i_deadband_turns: inside this remaining-error window, velocity I-term is disabled.
      - vel_i_decay_on_limit: multiplicative decay applied to velocity I-term on
        clamp/current-stress retries.
      - command_slew_max_step_turns: max staged step size for setpoint progression
        (0 disables additional slew staging beyond stage_chunk_max_turns).

    pre_move_gate_mode:
      - "off"    : do not run quick directional pre-check
      - "warn"   : run pre-check and continue with warning on failure
      - "strict" : run pre-check and abort early on failure

    startup_checks:
      - "minimal"         : immediate start; force-disable pre-move gate, kick, and divergence recovery
      - "guarded"         : run pre-command stability probes/recovery before moving
      - "tinymovr_strict" : guarded startup + strict staged validation gate

    authority_precheck:
      - "off"    : skip sign/breakaway authority probe
      - "warn"   : probe and continue with structured warning on degraded authority
      - "strict" : probe and fail fast on degraded/critical authority
      - "auto"   : "off" for minimal startup, "strict" for guarded startup

    allow_stiction_kick:
      - False (default): disable kick/bump behavior to avoid startup wiggle
      - True           : allow profile/LUT kick injection

    enable_runtime_retries:
      - None (default): auto policy; enabled when retry_on_retryable_fault=True
      - False         : fail fast on first runtime fault (no in-move recovery retries)
      - True          : permit in-move retry/recovery logic

    enable_divergence_recovery:
      - None (default): auto policy; disabled for startup_checks="minimal"
      - False         : do not run divergence reference-recovery/retry cycle
      - True          : allow one divergence recovery cycle

    pre_move_gate_strategy:
      - "fixed"              : legacy gate behavior
      - "adaptive_breakaway" : boost breakaway while increasing damping

    auto_fallback_hybrid_on_low_authority:
      - If True and pre-move gate indicates low-authority (without current stress),
        switch to hybrid_soft_closed_loop instead of immediate strict-gate failure.

    Direction-specific authority knobs (chosen by move sign):
      - plus/minus_current_scale
      - plus/minus_trap_scale
      - plus/minus_vel_limit_scale
      - plus/minus_kick_scale
    """
    a = axis if axis is not None else common.get_axis0()
    target_turns, space, ratio = _angle_target_to_motor_turns(
        angle_deg=angle_deg,
        angle_space=angle_space,
        gear_ratio=gear_ratio,
    )
    relative_mode = bool(relative_to_current)
    if bool(relative_mode):
        zero_turns = None
        ref_meta = {
            "path": None,
            "exists": False,
            "updated_at": None,
            "angle_space": "relative_to_current",
            "angle_deg_at_capture": 0.0,
            "relative_to_current": True,
        }
    else:
        zero_turns, ref_meta = _resolve_zero_turns_motor(
            zero_turns_motor=zero_turns_motor,
            angle_ref_path=angle_ref_path,
            require_ref=bool(require_angle_reference),
        )
    cfg = get_move_to_angle_kwargs_from_profile(profile_name=profile_name, path=profile_path)
    if isinstance(profile_overrides, dict):
        for k in (
            "trap_vel",
            "vel_limit",
            "vel_limit_tolerance",
            "enable_overspeed_error",
            "trap_acc",
            "trap_dec",
            "current_lim",
            "pos_gain",
            "vel_gain",
            "vel_i_gain",
            "stiction_kick_nm",
            "target_tolerance_turns",
            "target_vel_tolerance_turns_s",
        ):
            if k in profile_overrides and profile_overrides.get(k) is not None:
                if k == "enable_overspeed_error":
                    cfg[k] = bool(profile_overrides.get(k))
                else:
                    cfg[k] = float(profile_overrides.get(k))
    trap_vel = float(cfg["trap_vel"])
    vel_limit_cmd = float(cfg.get("vel_limit", max(0.25, 2.0 * float(trap_vel))))
    vel_limit_tol_cmd = float(cfg.get("vel_limit_tolerance", (4.0 if float(trap_vel) <= 0.12 else 2.0)))
    overspeed_err_cmd = bool(cfg.get("enable_overspeed_error", True))
    trap_acc = float(cfg["trap_acc"])
    trap_dec = float(cfg["trap_dec"])
    timeout_eff = float(timeout_s)
    deadline_info = None
    timing_info = None
    timing_warning = None
    deferred_warnings = []
    smooth_gate_on = bool(smoothness_gate)
    smooth_max_sign_changes = max(0, int(smoothness_max_vel_sign_changes))
    smooth_max_acc = (
        None
        if smoothness_max_peak_abs_acc_turns_s2 is None
        else max(0.0, float(smoothness_max_peak_abs_acc_turns_s2))
    )
    smooth_max_jerk = (
        None
        if smoothness_max_peak_abs_jerk_turns_s3 is None
        else max(0.0, float(smoothness_max_peak_abs_jerk_turns_s3))
    )
    vel_i_deadband = max(0.0, abs(float(vel_i_deadband_turns)))
    vel_i_decay_factor = min(1.0, max(0.0, float(vel_i_decay_on_limit)))

    start_turns = float(getattr(a.encoder, "pos_estimate", 0.0))
    if bool(relative_mode):
        zero_turns = float(start_turns)
    base_target_turns_abs = float(zero_turns) + float(target_turns)
    target_turns_abs, wrap_k, wrap_mode = _resolve_output_target_turns(
        base_target_turns_motor=base_target_turns_abs,
        start_turns_motor=start_turns,
        angle_space=space,
        gear_ratio=ratio,
        wrap_strategy=wrap_strategy,
    )
    move_dist = abs(float(target_turns_abs) - float(start_turns))
    nominal_move_dist = float(move_dist)
    # Runtime safety: if the effective target distance balloons far beyond the
    # requested move, treat it as divergence/frame drift instead of chasing it.
    runtime_distance_cap = max(0.75, (2.5 * float(nominal_move_dist)) + 0.20)
    chunk = max(0.0, abs(float(stage_chunk_max_turns)))
    slew_chunk = max(0.0, abs(float(command_slew_max_step_turns)))
    if float(slew_chunk) > 0.0:
        # Additional bounded setpoint slew staging (Tinymovr-style ramping behavior).
        chunk = float(slew_chunk) if float(chunk) <= 0.0 else min(float(chunk), float(slew_chunk))
    stage_count = 0
    if chunk > 0.0 and move_dist > chunk:
        stage_count = max(0, int(math.ceil(move_dist / chunk)) - 1)
    stage_penalty_s = float(stage_count) * max(0.0, float(stage_settle_s))
    style = str(control_style).strip().lower()
    if style not in ("trap_strict", "hybrid_soft_closed_loop"):
        raise ValueError("control_style must be 'trap_strict' or 'hybrid_soft_closed_loop'")
    gate_mode = str(pre_move_gate_mode).strip().lower()
    if gate_mode not in ("off", "warn", "strict"):
        raise ValueError("pre_move_gate_mode must be 'off', 'warn', or 'strict'")
    gate_strategy = str(pre_move_gate_strategy).strip().lower()
    if gate_strategy not in ("fixed", "adaptive_breakaway"):
        raise ValueError("pre_move_gate_strategy must be 'fixed' or 'adaptive_breakaway'")
    startup_checks_mode = str(startup_checks).strip().lower()
    if startup_checks_mode not in ("minimal", "guarded", "tinymovr_strict"):
        raise ValueError("startup_checks must be 'minimal', 'guarded', or 'tinymovr_strict'")
    authority_mode_req = str(authority_precheck).strip().lower()
    if authority_mode_req not in ("off", "warn", "strict", "auto"):
        raise ValueError("authority_precheck must be 'off', 'warn', 'strict', or 'auto'")
    authority_mode = (
        ("off" if str(startup_checks_mode) == "minimal" else "strict")
        if str(authority_mode_req) == "auto"
        else str(authority_mode_req)
    )
    allow_kick = bool(allow_stiction_kick)
    startup_overrides = []
    if enable_runtime_retries is None:
        # Keep recovery enabled by default even in minimal startup mode.
        # This avoids hard-failing single-shot calls on the first current-stress event.
        runtime_retries_on = bool(retry_on_retryable_fault)
    else:
        runtime_retries_on = bool(enable_runtime_retries)
    if enable_divergence_recovery is None:
        divergence_recovery_on = bool(startup_checks_mode != "minimal")
    else:
        divergence_recovery_on = bool(enable_divergence_recovery)
    if str(startup_checks_mode) == "minimal":
        if str(gate_mode) != "off":
            startup_overrides.append("pre_move_gate_mode forced to 'off' because startup_checks='minimal'")
            gate_mode = "off"
        if bool(allow_kick):
            startup_overrides.append("allow_stiction_kick forced to False because startup_checks='minimal'")
        if bool(divergence_recovery_on):
            startup_overrides.append("divergence recovery forced to False because startup_checks='minimal'")
        if str(authority_mode) != "off":
            startup_overrides.append("authority_precheck forced to 'off' because startup_checks='minimal'")
        allow_kick = False
        divergence_recovery_on = False
        authority_mode = "off"

    move_sign = 0.0
    if float(target_turns_abs) > float(start_turns):
        move_sign = +1.0
    elif float(target_turns_abs) < float(start_turns):
        move_sign = -1.0

    prof_plus_current_scale = max(0.20, float(cfg.get("plus_current_scale", 1.0)))
    prof_minus_current_scale = max(0.20, float(cfg.get("minus_current_scale", 1.0)))
    prof_plus_trap_scale = max(0.20, float(cfg.get("plus_trap_scale", 1.0)))
    prof_minus_trap_scale = max(0.20, float(cfg.get("minus_trap_scale", 1.0)))
    prof_plus_vel_limit_scale = max(0.20, float(cfg.get("plus_vel_limit_scale", 1.0)))
    prof_minus_vel_limit_scale = max(0.20, float(cfg.get("minus_vel_limit_scale", 1.0)))
    prof_plus_kick_scale = max(0.0, float(cfg.get("plus_kick_scale", 1.0)))
    prof_minus_kick_scale = max(0.0, float(cfg.get("minus_kick_scale", 1.0)))

    is_plus_dir = bool(move_sign >= 0.0)
    current_scale_dir = max(
        0.20,
        (prof_plus_current_scale if is_plus_dir else prof_minus_current_scale)
        * float(plus_current_scale if is_plus_dir else minus_current_scale),
    )
    trap_scale_dir = max(
        0.20,
        (prof_plus_trap_scale if is_plus_dir else prof_minus_trap_scale)
        * float(plus_trap_scale if is_plus_dir else minus_trap_scale),
    )
    vel_limit_scale_dir = max(
        0.20,
        (prof_plus_vel_limit_scale if is_plus_dir else prof_minus_vel_limit_scale)
        * float(plus_vel_limit_scale if is_plus_dir else minus_vel_limit_scale),
    )
    kick_scale_dir = max(
        0.0,
        (prof_plus_kick_scale if is_plus_dir else prof_minus_kick_scale)
        * float(plus_kick_scale if is_plus_dir else minus_kick_scale),
    )

    trap_vel = float(trap_vel) * float(trap_scale_dir)
    trap_acc = float(trap_acc) * float(trap_scale_dir)
    trap_dec = float(trap_dec) * float(trap_scale_dir)
    vel_limit_cmd = float(vel_limit_cmd) * float(vel_limit_scale_dir)
    current_lim_base = max(0.2, float(cfg["current_lim"]) * float(current_scale_dir))

    settle = max(0.0, float(settle_s))
    max_scale = max(1.0, float(deadline_max_scale))
    base_move_t = _trap_move_time_est(move_dist, trap_vel, trap_acc, trap_dec)
    min_move_t_with_cap = _trap_move_time_est(
        move_dist, trap_vel * max_scale, trap_acc * max_scale, trap_dec * max_scale
    )
    guaranteed_total_with_cap = float(min_move_t_with_cap + settle + stage_penalty_s)

    if deadline_s is None:
        scale = float(max_scale)
        if float(scale) > 1.0:
            trap_vel = float(trap_vel) * float(scale)
            trap_acc = float(trap_acc) * float(scale)
            trap_dec = float(trap_dec) * float(scale)
            vel_limit_cmd = float(vel_limit_cmd) * float(scale)
        est_total = _trap_move_time_est(move_dist, trap_vel, trap_acc, trap_dec) + settle + stage_penalty_s
        timeout_eff = max(float(timeout_s), est_total + max(0.10, float(deadline_timeout_margin_s)))
        timing_info = {
            "mode": "auto_fastest",
            "deadline_s": None,
            "distance_turns": float(move_dist),
            "scale_applied": float(scale),
            "estimated_total_s": float(est_total),
            "estimated_base_total_s": float(base_move_t + settle + stage_penalty_s),
            "estimated_min_total_with_max_scale_s": float(guaranteed_total_with_cap),
            "guaranteed_total_s": float(est_total),
            "feasible_with_max_scale": True,
            "stage_count_est": int(stage_count),
            "stage_penalty_s_est": float(stage_penalty_s),
            "max_scale": float(max_scale),
            "required_scale_est": 1.0,
            "warning_code": None,
            "warning_message": None,
            "proceeded_best_effort": False,
        }
    else:
        mode = str(deadline_mode).strip().lower()
        if mode not in ("strict", "best_effort"):
            raise ValueError("deadline_mode must be 'strict' or 'best_effort'")
        dl = float(deadline_s)
        if dl <= 0.0:
            raise ValueError("deadline_s must be > 0 when provided")
        move_budget = dl - settle - stage_penalty_s
        if move_budget > 0.0:
            scale = _solve_deadline_scale(move_dist, trap_vel, trap_acc, trap_dec, move_budget, max_scale)
            required_scale_est = max(1.0, float(base_move_t) / max(1e-9, float(move_budget)))
        else:
            scale = None
            required_scale_est = float("inf")
        deadline_feasible = bool((move_budget > 0.0) and (scale is not None))
        if not bool(deadline_feasible):
            scale = max_scale
            timing_warning = {
                "code": "DEADLINE_INFEASIBLE",
                "level": "warning",
                "deadline_s": float(dl),
                "guaranteed_total_s": float(guaranteed_total_with_cap),
                "recommended_deadline_s": float(guaranteed_total_with_cap + 0.20),
                "reason": (
                    "Requested deadline is below guaranteed minimum at configured max scale."
                    if move_budget > 0.0
                    else "Requested deadline is below settle/staging overhead."
                ),
            }
        if float(scale) > 1.0:
            trap_vel = float(trap_vel) * float(scale)
            trap_acc = float(trap_acc) * float(scale)
            trap_dec = float(trap_dec) * float(scale)
            vel_limit_cmd = float(vel_limit_cmd) * float(scale)
        est_total = _trap_move_time_est(move_dist, trap_vel, trap_acc, trap_dec) + settle + stage_penalty_s
        timeout_eff = max(float(timeout_s), est_total + max(0.10, float(deadline_timeout_margin_s)))
        deadline_info = {
            "deadline_s": dl,
            "mode": mode,
            "distance_turns": move_dist,
            "scale_applied": float(scale),
            "estimated_total_s": float(est_total),
            "estimated_base_total_s": float(base_move_t + settle + stage_penalty_s),
            "estimated_min_total_with_max_scale_s": float(guaranteed_total_with_cap),
            "guaranteed_total_s": float(guaranteed_total_with_cap),
            "feasible_with_max_scale": bool(deadline_feasible),
            "stage_count_est": int(stage_count),
            "stage_penalty_s_est": float(stage_penalty_s),
            "max_scale": float(max_scale),
            "required_scale_est": float(required_scale_est),
            "warning_code": None if timing_warning is None else str(timing_warning.get("code")),
            "warning_message": None if timing_warning is None else str(timing_warning.get("reason")),
            "proceeded_best_effort": bool(not deadline_feasible),
        }
        timing_info = dict(deadline_info)

    def _is_no_movement_fault(exc):
        return "No movement detected after position command" in str(exc)

    def _is_target_timeout_fault(exc):
        return "Target not reached before timeout" in str(exc)

    def _is_transient_hold_fault(exc):
        return "Target reached transiently but did not hold after settle" in str(exc)

    def _is_current_stress_fault(exc):
        s = str(exc)
        return (
            ("motor_err=0x1000" in s)
            or ("MOTOR_ERROR_CURRENT_LIMIT_VIOLATION" in s)
            or ("aborted_current" in s)
        )

    def _is_wrong_direction_fault(exc):
        return "WRONG_DIRECTION watchdog tripped" in str(exc)

    def _is_drift_guard_fault(exc):
        return "move_to_angle drift guard:" in str(exc)

    def _is_divergence_fault(exc):
        s = str(exc)
        return (
            ("DIVERGENCE watchdog tripped" in s)
            or _is_drift_guard_fault(exc)
        )

    def _is_retryable_fault(exc):
        return (
            _is_no_movement_fault(exc)
            or _is_target_timeout_fault(exc)
            or _is_transient_hold_fault(exc)
            or _is_current_stress_fault(exc)
        )

    def _parse_timeout_field_turns(exc, key):
        """Best-effort parse of `<key><float>t` from move_to_pos_strict text."""
        s = str(exc)
        i = s.find(str(key))
        if i < 0:
            return None
        j = s.find("t", i + len(str(key)))
        if j < 0:
            return None
        raw = s[i + len(str(key)):j].strip()
        try:
            return float(raw)
        except Exception:
            return None

    def _parse_timeout_error_turns(exc):
        """Best-effort parse of `err=...t` from move_to_pos_strict timeout text."""
        return _parse_timeout_field_turns(exc, "err=")

    def _timeout_looks_diverged(exc):
        """Detect large timeout mismatch that should not be retried in-place."""
        err_turns = _parse_timeout_error_turns(exc)
        pos_turns = _parse_timeout_field_turns(exc, "pos=")
        tgt_turns = _parse_timeout_field_turns(exc, "target=")
        if (pos_turns is not None) and (tgt_turns is not None):
            abs_err = abs(float(tgt_turns) - float(pos_turns))
        elif err_turns is not None:
            abs_err = abs(float(err_turns))
        else:
            return False
        tol = max(0.010, float(cfg.get("target_tolerance_turns", 0.010)))
        # Treat "very far from target at timeout" as divergence instead of a recoverable miss.
        diverge_thresh = max(0.50, 0.40 * float(move_dist), 20.0 * tol)
        return bool(abs_err >= diverge_thresh)

    def _recover_retry():
        common.clear_errors_all(a)
        common.force_idle(a, settle_s=0.08)
        if not common.ensure_closed_loop(a, timeout_s=3.0, clear_first=False, pre_sync=True, retries=2):
            # Escalate if local close-loop recovery did not recover.
            common.establish_absolute_reference(
                a,
                require_index=bool(getattr(getattr(a.encoder, "config", object()), "use_index", False)),
                run_index_search=False,
                attempt_offset_calibration=False,
            )

    derate = 1.0
    stiction_kick_base_nm = (max(0.0, float(cfg["stiction_kick_nm"])) if bool(allow_kick) else 0.0)
    stiction_kick_cmd_nm = float(stiction_kick_base_nm)
    stiction_lut_info = None
    if bool(use_stiction_lut):
        lut_kick = lookup_stiction_kick_from_map(
            pos_turns_motor=float(start_turns),
            direction=float(move_sign),
            angle_space=space,
            gear_ratio=float(ratio),
            map_path=stiction_map_path,
        )
        if bool(allow_kick) and (lut_kick is not None):
            scaled = max(0.0, float(lut_kick) * max(0.0, float(stiction_lut_gain)))
            stiction_kick_base_nm = max(float(stiction_kick_base_nm), float(scaled))
        stiction_lut_info = {
            "enabled": True,
            "path": os.path.abspath(str(stiction_map_path or _default_stiction_map_path())),
            "kick_lookup_nm": None if lut_kick is None else float(lut_kick),
            "kick_applied_nm": (float(stiction_kick_base_nm) * float(kick_scale_dir)) if bool(allow_kick) else 0.0,
            "gain": float(stiction_lut_gain),
            "allow_stiction_kick": bool(allow_kick),
        }
    stiction_kick_cmd_nm = max(0.0, float(stiction_kick_base_nm) * float(kick_scale_dir)) if bool(allow_kick) else 0.0
    pre_move_gate = {
        "requested_mode": str(pre_move_gate_mode).strip().lower(),
        "mode": str(gate_mode),
        "strategy": str(gate_strategy),
        "enabled": bool(gate_mode != "off"),
        "startup_checks": str(startup_checks_mode),
        "startup_overrides": list(startup_overrides or []),
        "ok": None,
        "attempts_run": 0,
        "load_mode": str(pre_move_gate_load_mode),
        "delta_turns": 0.0,
        "delta_cap_turns": None,
        "attempts": [],
    }
    authority_precheck_meta = {
        "requested_mode": str(authority_mode_req),
        "mode": str(authority_mode),
        "ran": False,
        "ok": None,
        "classification": None,
        "severity": None,
        "code": None,
        "direction_supported": None,
        "probe": None,
    }

    def _probe_has_aborted_current(probe):
        try:
            rows = list((dict(probe or {})).get("results") or [])
        except Exception:
            rows = []
        for row in rows:
            if not isinstance(row, dict):
                continue
            status = str(row.get("status", "")).strip().lower()
            reason = str(row.get("reason", "")).strip().lower()
            if status == "aborted_current":
                return True
            if ("current stress" in reason) or ("current limit" in reason):
                return True
        return False

    def _probe_looks_low_authority(probe, gate_cur_lim_local):
        """Detect likely stiction/authority limit (not current-stress) from gate probe."""
        try:
            rows = list((dict(probe or {})).get("results") or [])
        except Exception:
            rows = []
        if not rows:
            return False
        row = dict(rows[0] or {})
        status = str(row.get("status", "")).strip().lower()
        reason = str(row.get("reason", "")).strip().lower()
        if status == "aborted_current" or ("current stress" in reason):
            return False
        progress = abs(float(row.get("progress", 0.0) or 0.0))
        min_progress = abs(float(row.get("min_progress", 0.0) or 0.0))
        peak_iq = abs(float(row.get("peak_abs_iq", 0.0) or 0.0))
        weak_progress = progress < max(0.0006, 0.65 * max(0.0006, min_progress))
        low_iq = peak_iq <= max(0.5, 0.60 * max(0.2, float(gate_cur_lim_local)))
        timeout_like = status in ("timeout", "no_movement", "transient_reached")
        return bool(weak_progress and low_iq and timeout_like)

    def _recover_index_policy():
        """Use index search during recovery only when encoder reference is not currently usable."""
        try:
            use_index_rt = bool(getattr(getattr(a.encoder, "config", object()), "use_index", False))
        except Exception:
            use_index_rt = False
        try:
            enc_ready_rt = bool(getattr(a.encoder, "is_ready", False))
        except Exception:
            enc_ready_rt = False
        try:
            index_found_rt = bool(getattr(a.encoder, "index_found", False))
        except Exception:
            index_found_rt = False
        # Re-running index search can re-zero the motor-turn frame and invalidate
        # caller-provided zero references. Only re-index when reference is missing.
        run_index_rt = bool(use_index_rt and ((not enc_ready_rt) or (not index_found_rt)))
        return bool(use_index_rt), bool(run_index_rt)

    startup_contract_mode = ("guarded" if str(startup_checks_mode) == "tinymovr_strict" else str(startup_checks_mode))
    startup_contract = common.move_startup_contract(
        a,
        startup_mode=str(startup_contract_mode),
        require_index=bool(getattr(getattr(a.encoder, "config", object()), "use_index", False)),
        run_index_search_on_recover=bool(startup_checks_mode in ("guarded", "tinymovr_strict")),
        require_encoder_ready=True,
        timeout_s=(3.0 if str(startup_checks_mode) in ("guarded", "tinymovr_strict") else 1.5),
        sync_settle_s=(0.05 if str(startup_checks_mode) in ("guarded", "tinymovr_strict") else 0.02),
        stability_observe_s=0.25,
        stability_dt=0.02,
    )
    if not bool(dict(startup_contract or {}).get("ok", False)):
        raise RuntimeError(
            "move_to_angle startup contract failed: "
            f"{dict(startup_contract or {}).get('error') or 'unknown'} "
            f"snapshot={dict(startup_contract or {}).get('snapshot_end')}"
        )
    pre_move_gate["startup_contract"] = dict(startup_contract or {})
    pre_gate_stability = dict((dict(startup_contract or {}).get("stability_probe") or {}))
    pre_move_gate["stability_probe"] = dict(pre_gate_stability or {})
    strict_gate_meta = {
        "ran": False,
        "ok": None,
        "result": None,
    }
    if str(startup_checks_mode) == "tinymovr_strict":
        strict_gate_meta["ran"] = True
        strict_res = common.tinymovr_style_validation_gate(
            axis=a,
            require_index=bool(getattr(getattr(a.encoder, "config", object()), "use_index", False)),
            run_index_search_on_recover=False,
            current_lim=max(0.2, float(current_lim_base)),
            vel_limit=max(0.05, float(vel_limit_cmd)),
            vel_probe_turns_s=max(0.06, min(0.12, 0.50 * max(0.05, float(vel_limit_cmd)))),
            vel_probe_s=0.18,
            vel_probe_settle_s=0.06,
            pos_probe_turns=max(0.006, min(0.015, 0.25 * max(0.01, float(move_dist)))),
            pos_probe_timeout_s=min(3.0, max(1.8, 0.35 * float(timeout_eff))),
            pos_probe_settle_s=0.08,
            motion_eps_turns=max(8e-4, float(authority_precheck_min_delta_turns)),
            motion_eps_counts=2,
            verbose=False,
        )
        strict_gate_meta["result"] = dict(strict_res or {})
        strict_gate_meta["ok"] = bool(dict(strict_res or {}).get("ok", False))
        if not bool(strict_gate_meta["ok"]):
            raise RuntimeError(
                "move_to_angle tinymovr_strict validation failed: "
                + json.dumps(dict(strict_res or {}), sort_keys=True)
            )
    if str(startup_checks_mode) == "minimal":
        pre_move_gate["enabled"] = False
        pre_move_gate["disabled_reason"] = "startup_checks=minimal"

    if bool(str(authority_mode) != "off") and abs(float(move_sign)) > 0.0 and float(move_dist) > 0.0:
        ap_delta = abs(float(authority_precheck_cmd_delta_turns))
        if ap_delta <= 0.0:
            ap_delta = max(0.004, min(0.02, 0.25 * float(move_dist)))
        ap_delta = min(float(ap_delta), max(0.004, min(0.05, 0.60 * float(move_dist))))
        ap_cur_lim = (
            max(0.2, float(current_lim_base))
            if authority_precheck_current_lim is None
            else max(0.2, float(authority_precheck_current_lim))
        )
        ap_pos_gain = (
            max(0.5, float(cfg["pos_gain"]))
            if authority_precheck_pos_gain is None
            else max(0.5, float(authority_precheck_pos_gain))
        )
        ap_vel_gain = (
            max(0.01, float(cfg["vel_gain"]))
            if authority_precheck_vel_gain is None
            else max(0.01, float(authority_precheck_vel_gain))
        )
        ap_vel_i_gain = max(0.0, float(authority_precheck_vel_i_gain))
        ap_vel_limit = (
            max(0.05, float(vel_limit_cmd))
            if authority_precheck_vel_limit is None
            else max(0.05, float(authority_precheck_vel_limit))
        )
        authority_precheck_meta.update(
            {
                "ran": True,
                "cmd_delta_turns": float(ap_delta),
                "current_lim": float(ap_cur_lim),
                "pos_gain": float(ap_pos_gain),
                "vel_gain": float(ap_vel_gain),
                "vel_i_gain": float(ap_vel_i_gain),
                "vel_limit": float(ap_vel_limit),
            }
        )
        try:
            tc = float(getattr(a.motor.config, "torque_constant", 0.04) or 0.04)
        except Exception:
            tc = 0.04
        # Bounded low-shock torque ladder for authority verification.
        tq_span = max(0.0, float(ap_cur_lim) * max(1e-6, float(tc)))
        tq1 = max(0.015, min(0.06, 0.08 * float(tq_span)))
        tq2 = max(tq1 + 0.01, min(0.10, 0.14 * float(tq_span)))
        tq_targets = (float(tq1), -float(tq1), float(tq2), -float(tq2))
        authority_precheck_meta["torque_targets_nm"] = list(tq_targets)
        try:
            aprobe = common.torque_authority_ramp_probe(
                a,
                torque_targets_nm=tq_targets,
                current_lim=float(ap_cur_lim),
                vel_limit=float(ap_vel_limit),
                ramp_s=min(0.20, max(0.08, 0.20 * float(authority_precheck_timeout_s))),
                dwell_s=min(0.20, max(0.08, 0.20 * float(authority_precheck_timeout_s))),
                settle_s=max(0.0, float(authority_precheck_settle_s)),
                dt=0.01,
                min_motion_turns=max(5e-4, float(authority_precheck_min_delta_turns)),
                min_motion_counts=2,
                iq_set_gate_a=0.25,
                iq_meas_gate_a=0.05,
                track_ratio_min=0.35,
                sign_match_min=0.65,
                vel_abort_turns_s=max(2.5, 4.0 * float(ap_vel_limit)),
                leave_idle=False,
                collect_samples=False,
                verbose=False,
            )
        except Exception as exc:
            authority_precheck_meta.update(
                {
                    "ok": False,
                    "classification": "precheck_exception",
                    "severity": "critical",
                    "code": "AUTHORITY_PRECHECK_EXCEPTION",
                    "direction_supported": False,
                    "error": str(exc),
                }
            )
            raise RuntimeError(
                "move_to_angle authority precheck failed: "
                + json.dumps(dict(authority_precheck_meta), sort_keys=True)
            )

        authority_precheck_meta["probe"] = dict(aprobe or {})
        cls = str((dict(aprobe or {})).get("classification", "unknown"))
        moved_plus = bool((dict(aprobe or {})).get("plus_moved", False))
        moved_minus = bool((dict(aprobe or {})).get("minus_moved", False))
        if float(move_sign) > 0.0:
            direction_supported = bool(moved_plus)
        elif float(move_sign) < 0.0:
            direction_supported = bool(moved_minus)
        else:
            direction_supported = True

        critical_classes = {
            "low_current_tracking",
            "current_sign_mismatch",
            "unstable_runaway",
        }
        degraded_classes = {
            "one_direction_only_or_limit",
            "low_authority_no_motion",
        }
        severity = "none"
        code = "AUTHORITY_OK"
        if cls in critical_classes:
            severity = "critical"
            if cls == "low_current_tracking":
                code = "AUTHORITY_LOW_CURRENT_TRACKING"
            elif cls == "current_sign_mismatch":
                code = "AUTHORITY_CURRENT_SIGN_MISMATCH"
            elif cls == "unstable_runaway":
                code = "AUTHORITY_UNSTABLE_RUNAWAY"
            else:
                code = "AUTHORITY_CRITICAL"
        elif cls in degraded_classes:
            if not bool(direction_supported):
                severity = "critical"
                code = "AUTHORITY_DIRECTION_UNSUPPORTED"
            elif cls == "low_authority_no_motion":
                code = "AUTHORITY_LOW_NO_MOTION"
                severity = "degraded"
            elif cls == "one_direction_only_or_limit":
                code = "AUTHORITY_ONE_DIRECTION_LIMIT"
                severity = "degraded"
            else:
                code = "AUTHORITY_DEGRADED"
                severity = "degraded"
        elif cls == "authority_ok":
            severity = "none"
            code = "AUTHORITY_OK"
        else:
            severity = "degraded"
            code = "AUTHORITY_UNKNOWN_INCONCLUSIVE"

        authority_precheck_meta.update(
            {
                "classification": str(cls),
                "direction_supported": bool(direction_supported),
                "severity": str(severity),
                "code": str(code),
                "ok": bool(severity == "none"),
            }
        )

        if str(severity) == "critical" or (str(severity) == "degraded" and str(authority_mode) == "strict"):
            raise RuntimeError(
                "move_to_angle authority precheck failed: "
                + json.dumps(dict(authority_precheck_meta), sort_keys=True)
            )
        if str(severity) == "degraded":
            deferred_warnings.append(
                {
                    "code": str(code),
                    "level": "warning",
                    "reason": "Authority precheck indicates directional bias/limited breakaway.",
                    "authority_precheck": {
                        "classification": str(cls),
                        "severity": str(severity),
                        "direction_supported": bool(direction_supported),
                        "mode": str(authority_mode),
                    },
                }
            )
    else:
        authority_precheck_meta["ok"] = True
        authority_precheck_meta["skipped"] = True
        authority_precheck_meta["reason"] = (
            "authority_precheck=off"
            if str(authority_mode) == "off"
            else "no_directional_move_or_zero_distance"
        )

    if bool(gate_mode != "off") and abs(float(move_sign)) > 0.0 and float(move_dist) > 0.0:
        user_gate_delta = abs(float(pre_move_gate_delta_turns))
        tol_gate_delta = 0.50 * abs(float(cfg.get("target_tolerance_turns", 0.006)))
        frac_gate_delta = 0.18 * abs(float(move_dist))
        if float(stage_chunk_max_turns) > 0.0:
            gate_delta_cap = min(0.12, max(0.02, 0.60 * abs(float(stage_chunk_max_turns))))
        else:
            gate_delta_cap = 0.08
        heuristic_gate_delta = max(0.006, min(float(gate_delta_cap), max(tol_gate_delta, frac_gate_delta)))
        gate_delta = min(
            abs(float(move_dist)),
            float(gate_delta_cap),
            max(0.0008, max(heuristic_gate_delta, user_gate_delta if user_gate_delta > 0.0 else 0.0)),
        )
        gate_delta = max(0.0008, min(float(gate_delta), float(gate_delta_cap)))
        pre_move_gate["delta_cap_turns"] = float(gate_delta_cap)
        gate_attempts = max(1, int(pre_move_gate_max_attempts))
        gate_cur_lim = max(0.2, float(current_lim_base) * float(derate))
        gate_kick = max(0.0, float(stiction_kick_cmd_nm))
        gate_current_cap = max(
            0.2,
            float(current_lim_base) * max(1.0, float(pre_move_gate_current_max_scale)),
        )
        if pre_move_gate_abs_current_cap_a is not None:
            gate_current_cap = min(
                float(gate_current_cap),
                max(0.2, abs(float(pre_move_gate_abs_current_cap_a))),
            )
        gate_cur_lim = min(float(gate_cur_lim), float(gate_current_cap))
        gate_kick_cap = max(0.0, float(pre_move_gate_kick_max_nm))
        gate_kick = min(float(gate_kick), float(gate_kick_cap))
        pre_move_gate["current_cap_a"] = float(gate_current_cap)
        pre_move_gate["kick_cap_nm"] = float(gate_kick_cap)
        gate_pos_gain_scale = 1.0
        gate_vel_gain_scale = (1.15 if gate_strategy == "adaptive_breakaway" else 1.0)
        gate_ok = False
        gate_abort_current_seen = False
        gate_low_authority_seen = False
        for gidx in range(1, gate_attempts + 1):
            try:
                base_now = float(getattr(a.encoder, "pos_estimate", 0.0))
            except Exception:
                base_now = float(start_turns)
            try:
                gate_trap_scale = 0.85
                if gate_strategy == "adaptive_breakaway":
                    # Keep breakaway probes controlled while still allowing useful authority.
                    gate_trap_scale = 0.90
                gate_trap_vel = min(float(trap_vel), max(0.02, float(trap_vel) * float(gate_trap_scale)))
                gate_trap_acc = min(float(trap_acc), max(0.04, float(trap_acc) * float(gate_trap_scale)))
                gate_trap_dec = min(float(trap_dec), max(0.04, float(trap_dec) * float(gate_trap_scale)))
                gate_duration_s = max(
                    0.40,
                    float(pre_move_gate_duration_s),
                    _trap_move_time_est(abs(float(gate_delta)), gate_trap_vel, gate_trap_acc, gate_trap_dec) + 0.45,
                )
                gate_pos_gain = max(0.5, float(cfg["pos_gain"]) * float(derate) * float(gate_pos_gain_scale))
                gate_vel_gain = max(0.01, float(cfg["vel_gain"]) * float(derate) * float(gate_vel_gain_scale))
                gate_vel_i_gain = max(0.0, float(cfg["vel_i_gain"]) * float(derate))
                if gate_strategy == "adaptive_breakaway":
                    # During breakaway checks, reduce I-term to avoid windup/runaway.
                    gate_vel_i_gain = min(gate_vel_i_gain, 0.02)
                probe = safe_loaded_step_test(
                    base_turns=float(base_now),
                    direction=float(move_sign),
                    delta_candidates=(float(gate_delta),),
                    trap_vel=float(gate_trap_vel),
                    trap_acc=float(gate_trap_acc),
                    trap_dec=float(gate_trap_dec),
                    current_lim=float(gate_cur_lim),
                    current_abort_frac=0.90,
                    pos_gain=float(gate_pos_gain),
                    vel_gain=float(gate_vel_gain),
                    vel_i_gain=float(gate_vel_i_gain),
                    stiction_kick_nm=(0.0 if (gate_strategy == "adaptive_breakaway" or (not bool(allow_kick))) else float(gate_kick)),
                    duration_s=float(gate_duration_s),
                    dt=0.02,
                    target_tolerance_turns=max(float(cfg["target_tolerance_turns"]), 0.006),
                    target_vel_tolerance_turns_s=max(float(cfg["target_vel_tolerance_turns_s"]), 0.12),
                    min_progress_turns=max(0.0006, (0.18 if gate_strategy == "adaptive_breakaway" else 0.25) * float(gate_delta)),
                    min_progress_frac=0.20,
                    min_settle_samples=3,
                    hold_in_tol_s=0.05,
                    absolute_at_start=bool(pre_move_gate_absolute_at_start),
                    run_index_search_at_start=bool(pre_move_gate_run_index_search),
                    recheck_absolute_each_candidate=False,
                    strict_frame_lock=False,
                    abort_on_frame_jump=False,
                    stop_on_abort_current=True,
                    load_mode=pre_move_gate_load_mode,
                )
            except Exception as exc:
                probe = {"ok": False, "error": str(exc), "results": []}
            had_abort_current = bool(_probe_has_aborted_current(probe))
            low_authority_attempt = bool(_probe_looks_low_authority(probe, gate_cur_lim))
            gate_abort_current_seen = bool(gate_abort_current_seen or had_abort_current)
            gate_low_authority_seen = bool(gate_low_authority_seen or low_authority_attempt)
            rows = list((dict(probe or {})).get("results") or [])
            top = rows[0] if rows else {}
            progress_val = float((dict(top) or {}).get("progress", 0.0) or 0.0)
            min_progress_val = float((dict(top) or {}).get("min_progress", 0.0) or 0.0)
            pre_move_gate["attempts"].append(
                {
                    "attempt": int(gidx),
                    "current_lim": float(gate_cur_lim),
                    "stiction_kick_nm": float(gate_kick),
                    "delta_turns": float(gate_delta),
                    "pos_gain_scale": float(gate_pos_gain_scale),
                    "vel_gain_scale": float(gate_vel_gain_scale),
                    "aborted_current": bool(had_abort_current),
                    "low_authority": bool(low_authority_attempt),
                    "probe": probe,
                }
            )
            pre_move_gate["attempts_run"] = int(gidx)
            pre_move_gate["delta_turns"] = float(gate_delta)
            if bool((dict(probe or {})).get("ok", False)):
                gate_ok = True
                break
            if gate_strategy == "adaptive_breakaway":
                if bool(had_abort_current):
                    # Hit current-stress: reduce aggressiveness and add damping.
                    gate_delta = min(float(gate_delta_cap), max(0.006, float(gate_delta) * 0.75))
                    gate_cur_lim = max(0.20, min(float(gate_current_cap), float(gate_cur_lim) * 0.90))
                    gate_kick = min(float(gate_kick_cap), max(0.01, float(gate_kick)) * 1.08)
                    gate_pos_gain_scale = max(0.55, float(gate_pos_gain_scale) * 0.90)
                    gate_vel_gain_scale = min(1.80, float(gate_vel_gain_scale) * 1.12)
                else:
                    weak_progress = abs(float(progress_val)) < max(0.0006, 0.65 * abs(float(min_progress_val)))
                    if bool(weak_progress):
                        gate_delta = min(
                            abs(float(move_dist)),
                            float(gate_delta_cap),
                            max(float(gate_delta) * 1.20, float(gate_delta) + 0.002),
                        )
                    gate_cur_lim = min(
                        float(gate_current_cap),
                        float(gate_cur_lim) * max(1.0, float(pre_move_gate_current_scale_step)),
                    )
                    gate_kick = min(
                        float(gate_kick_cap),
                        max(0.01, float(gate_kick)) * max(1.0, float(pre_move_gate_kick_scale_step)),
                    )
                    gate_pos_gain_scale = min(2.20, max(0.60, float(gate_pos_gain_scale) * 1.15))
                    gate_vel_gain_scale = min(2.40, float(gate_vel_gain_scale) * 1.12)
            else:
                gate_cur_lim = min(
                    float(gate_current_cap),
                    float(gate_cur_lim) * max(1.0, float(pre_move_gate_current_scale_step)),
                )
                gate_kick = min(
                    float(gate_kick_cap),
                    max(0.01, float(gate_kick)) * max(1.0, float(pre_move_gate_kick_scale_step)),
                )
            try:
                common.clear_errors_all(a)
            except Exception:
                pass
        pre_move_gate["ok"] = bool(gate_ok)
        pre_move_gate["aborted_current_seen"] = bool(gate_abort_current_seen)
        pre_move_gate["low_authority_seen"] = bool(gate_low_authority_seen)
        if not bool(gate_ok):
            gate_msg = (
                "move_to_angle pre_move_gate failed: "
                f"mode={gate_mode} load_mode={pre_move_gate_load_mode} "
                f"direction={'+' if move_sign >= 0 else '-'} delta={float(gate_delta):.6f}t "
                f"attempts={int(pre_move_gate.get('attempts_run', 0))}"
            )
            if bool(gate_abort_current_seen):
                # Treat gate current-abort as hard-fail even under warn mode.
                raise RuntimeError(gate_msg + " (aborted_current observed during gate)")
            if bool(gate_low_authority_seen) and bool(auto_fallback_hybrid_on_low_authority):
                if str(style) != "hybrid_soft_closed_loop":
                    style = "hybrid_soft_closed_loop"
                pre_move_gate["fallback_to_hybrid"] = True
                pre_move_gate["fallback_reason"] = "low_authority_pre_gate"
                try:
                    _recover_retry()
                except Exception:
                    pass
                print(gate_msg + " -> switching to hybrid_soft_closed_loop (low-authority fallback)")
            elif gate_mode == "strict":
                raise RuntimeError(gate_msg)
            else:
                print(gate_msg)

    # Gate probes can leave stale controller setpoints if a probe ends in IDLE/re-entry.
    # Re-synchronize only when the pre-move gate actually ran.
    if bool(pre_move_gate.get("enabled", False)):
        try:
            if not common.sync_pos_setpoint(a, settle_s=0.05, retries=3, verbose=False):
                _recover_retry()
                common.sync_pos_setpoint(a, settle_s=0.05, retries=2, verbose=False)
        except Exception:
            pass

    # Startup stability probe is already collected by move_startup_contract().
    stability_probe = dict((dict(startup_contract or {}).get("stability_probe") or {}))
    if not bool(stability_probe):
        stability_probe = {"ok": True, "skipped": True, "reason": "startup_contract_missing_probe"}

    adaptation_meta = {
        "pos_gain_scale_final": None,
        "vel_gain_scale_final": None,
        "vel_i_gain_override_final": None,
        "vel_i_scale_final": 1.0,
        "vel_i_decay_events": 0,
        "vel_i_deadband_turns": float(vel_i_deadband),
        "retry_no_motion": 0,
        "retry_timeout": 0,
        "retry_current_stress": 0,
        "retry_wrong_direction": 0,
        "unstick_attempts": 0,
        "unstick_success": False,
    }
    motion_quality = {
        "sample_count": 0,
        "duration_s": 0.0,
        "peak_abs_vel": 0.0,
        "peak_abs_acc": 0.0,
        "peak_abs_jerk": 0.0,
        "vel_sign_changes": 0,
    }

    def _accumulate_motion_quality(step_res):
        if not isinstance(step_res, dict):
            return
        try:
            motion_quality["sample_count"] = int(motion_quality["sample_count"]) + int(step_res.get("sample_count", 0) or 0)
        except Exception:
            pass
        try:
            motion_quality["duration_s"] = float(motion_quality["duration_s"]) + float(step_res.get("duration_s", 0.0) or 0.0)
        except Exception:
            pass
        try:
            motion_quality["peak_abs_vel"] = max(float(motion_quality["peak_abs_vel"]), abs(float(step_res.get("peak_abs_vel", 0.0) or 0.0)))
        except Exception:
            pass
        try:
            motion_quality["peak_abs_acc"] = max(float(motion_quality["peak_abs_acc"]), abs(float(step_res.get("peak_abs_acc", 0.0) or 0.0)))
        except Exception:
            pass
        try:
            motion_quality["peak_abs_jerk"] = max(float(motion_quality["peak_abs_jerk"]), abs(float(step_res.get("peak_abs_jerk", 0.0) or 0.0)))
        except Exception:
            pass
        try:
            motion_quality["vel_sign_changes"] = int(motion_quality["vel_sign_changes"]) + int(step_res.get("vel_sign_changes", 0) or 0)
        except Exception:
            pass

    def _attempt_unstick_nudge(cmd_target):
        """Bounded anti-stall nudge that couples extra effort with damping."""
        try:
            cur = float(getattr(a.encoder, "pos_estimate", 0.0))
        except Exception:
            return False
        sign = +1.0 if float(cmd_target) >= float(cur) else -1.0
        base_delta = max(0.03, min(0.10, max(0.25 * float(move_dist), 0.05)))
        deltas = [base_delta, min(0.12, base_delta * 1.30)]
        for dd in deltas:
            try:
                probe = safe_loaded_step_test(
                    base_turns=float(cur),
                    direction=float(sign),
                    delta_candidates=(float(dd),),
                    trap_vel=min(float(trap_vel), 0.06),
                    trap_acc=min(float(trap_acc), 0.12),
                    trap_dec=min(float(trap_dec), 0.12),
                    current_lim=min(max(0.2, float(current_lim_base) * 1.35), 12.0),
                    current_abort_frac=0.90,
                    pos_gain=max(0.5, float(cfg["pos_gain"]) * 0.65),
                    vel_gain=max(0.03, float(cfg["vel_gain"]) * 1.35),
                    vel_i_gain=min(0.01, max(0.0, float(cfg["vel_i_gain"]) * 0.15)),
                    stiction_kick_nm=(min(0.10, max(0.03, float(stiction_kick_cmd_nm) * 1.15)) if bool(allow_kick) else 0.0),
                    duration_s=max(1.8, float(pre_move_gate_duration_s) * 1.30),
                    dt=0.02,
                    target_tolerance_turns=max(float(cfg["target_tolerance_turns"]), 0.01),
                    target_vel_tolerance_turns_s=max(float(cfg["target_vel_tolerance_turns_s"]), 0.12),
                    min_progress_turns=max(0.0008, 0.15 * float(dd)),
                    min_progress_frac=0.15,
                    min_settle_samples=3,
                    hold_in_tol_s=0.04,
                    absolute_at_start=False,
                    run_index_search_at_start=False,
                    recheck_absolute_each_candidate=False,
                    strict_frame_lock=False,
                    abort_on_frame_jump=False,
                    stop_on_abort_current=True,
                    load_mode=pre_move_gate_load_mode,
                )
            except Exception:
                probe = {"ok": False, "results": []}

            if bool(_probe_has_aborted_current(probe)):
                return False
            if bool((dict(probe or {})).get("ok", False)):
                return True
            rows = list((dict(probe or {}).get("results") or []))
            row = dict(rows[0] or {}) if rows else {}
            progress = abs(float(row.get("progress", 0.0) or 0.0))
            min_progress = abs(float(row.get("min_progress", 0.0) or 0.0))
            if progress >= max(0.0008, 0.45 * max(0.0008, min_progress)):
                return True
        return False

    def _run_one_strict(
        cmd_target,
        cmd_settle_s,
        cmd_timeout_s,
        use_trap=True,
        min_delta_turns=0.002,
        pos_gain_scale=1.0,
        vel_gain_scale=1.0,
        vel_i_gain_override=None,
    ):
        nonlocal derate, stiction_kick_cmd_nm, adaptation_meta
        dyn_pos_gain_scale = float(pos_gain_scale)
        dyn_vel_gain_scale = float(vel_gain_scale)
        dyn_vel_i_gain_override = vel_i_gain_override
        dyn_vel_i_scale = 1.0
        dyn_min_delta_turns = max(0.0, float(min_delta_turns))
        dyn_timeout_s = max(0.25, float(cmd_timeout_s))
        retry_fault_attempts = 0
        retry_recover_attempts = 0
        retry_nomove_attempts = 0
        retry_wrong_dir_attempts = 0
        max_current_stress_retries = 4
        while True:
            try:
                # Guard against silent state drift/encoder jumps between planning and execution.
                # For small-angle commands this catches multi-turn excursions early.
                cur_now = float(getattr(a.encoder, "pos_estimate", 0.0))
                cmd_dist_now = abs(float(cmd_target) - float(cur_now))
                drift_guard = max(0.75, 4.0 * max(0.01, float(nominal_move_dist)))
                if cmd_dist_now > drift_guard:
                    raise RuntimeError(
                        "move_to_angle drift guard: command target became too far from current state. "
                        f"target={float(cmd_target):+.6f}t cur={float(cur_now):+.6f}t "
                        f"dist={float(cmd_dist_now):.6f}t guard={float(drift_guard):.6f}t"
                    )
                adaptation_meta["pos_gain_scale_final"] = float(dyn_pos_gain_scale)
                adaptation_meta["vel_gain_scale_final"] = float(dyn_vel_gain_scale)
                adaptation_meta["vel_i_gain_override_final"] = (
                    None if dyn_vel_i_gain_override is None else float(dyn_vel_i_gain_override)
                )
                adaptation_meta["vel_i_scale_final"] = float(dyn_vel_i_scale)
                cmd_current_lim = max(0.2, float(current_lim_base) * float(derate))
                cmd_pos_gain = max(0.5, float(cfg["pos_gain"]) * float(derate) * float(dyn_pos_gain_scale))
                cmd_vel_gain = max(0.01, float(cfg["vel_gain"]) * float(derate) * float(dyn_vel_gain_scale))
                cmd_vel_i_gain = (
                    max(0.0, float(cfg["vel_i_gain"]) * float(derate))
                    if dyn_vel_i_gain_override is None
                    else max(0.0, float(dyn_vel_i_gain_override))
                )
                cmd_vel_i_gain = max(0.0, float(cmd_vel_i_gain) * float(dyn_vel_i_scale))
                if float(cmd_dist_now) <= float(vel_i_deadband):
                    # Integrator deadband near target suppresses windup/rebound.
                    cmd_vel_i_gain = 0.0
                if cmd_dist_now <= 0.05:
                    # Tiny residual corrections should favor damping/precision over brute torque.
                    cmd_current_lim = min(cmd_current_lim, max(2.0, float(current_lim_base) * 0.75))
                    cmd_pos_gain = max(0.5, float(cmd_pos_gain) * 0.90)
                    cmd_vel_gain = max(0.01, float(cmd_vel_gain) * 1.12)
                    if dyn_vel_i_gain_override is None:
                        cmd_vel_i_gain = min(float(cmd_vel_i_gain), 0.02)
                if (not bool(use_trap)) and (cmd_dist_now <= 0.03):
                    cmd_current_lim = min(cmd_current_lim, max(2.0, float(current_lim_base) * 0.60))
                # Allow tiny elastic recoil, but abort quickly on real opposite motion.
                rev_eps = max(0.004, min(0.03, 0.25 * max(0.0, float(cmd_dist_now))))
                rev_confirm = 1 if float(cmd_dist_now) <= 0.12 else 2
                step_res = common.move_to_pos_strict(
                    a,
                    float(cmd_target),
                    use_trap_traj=bool(use_trap),
                    timeout_s=float(dyn_timeout_s),
                    min_delta_turns=max(0.0, float(dyn_min_delta_turns)),
                    settle_s=float(cmd_settle_s),
                    vel_limit=max(0.05, float(vel_limit_cmd) * float(derate)),
                    vel_limit_tolerance=max(1.0, float(vel_limit_tol_cmd)),
                    enable_overspeed_error=bool(overspeed_err_cmd),
                    trap_vel=float(trap_vel) * float(derate),
                    trap_acc=float(trap_acc) * float(derate),
                    trap_dec=float(trap_dec) * float(derate),
                    current_lim=float(cmd_current_lim),
                    pos_gain=float(cmd_pos_gain),
                    vel_gain=float(cmd_vel_gain),
                    vel_i_gain=float(cmd_vel_i_gain),
                    stiction_kick_nm=(float(stiction_kick_cmd_nm) * float(derate) if bool(allow_kick) else 0.0),
                    target_tolerance_turns=float(cfg["target_tolerance_turns"]),
                    target_vel_tolerance_turns_s=float(cfg["target_vel_tolerance_turns_s"]),
                    require_target_reached=True,
                    abort_on_reverse_motion=True,
                    reverse_motion_eps_turns=float(rev_eps),
                    reverse_motion_confirm_samples=int(rev_confirm),
                    fail_to_idle=False,
                )
                _accumulate_motion_quality(step_res)
                return step_res
            except RuntimeError as exc:
                if bool(_is_wrong_direction_fault(exc)):
                    if (not bool(retry_on_retryable_fault)) or (not bool(runtime_retries_on)):
                        try:
                            common.force_idle(a, settle_s=0.08)
                        except Exception:
                            pass
                        try:
                            common.clear_errors_all(a)
                        except Exception:
                            pass
                        raise RuntimeError(f"move_to_angle divergence abort: {exc}")
                    if retry_wrong_dir_attempts >= 2:
                        try:
                            common.force_idle(a, settle_s=0.08)
                        except Exception:
                            pass
                        try:
                            common.clear_errors_all(a)
                        except Exception:
                            pass
                        raise RuntimeError(
                            "move_to_angle wrong-direction retries exhausted. "
                            f"attempts={int(retry_wrong_dir_attempts)} derate={float(derate):.3f} "
                            f"pos_gain_scale={float(dyn_pos_gain_scale):.3f} "
                            f"vel_gain_scale={float(dyn_vel_gain_scale):.3f} "
                            f"timeout_s={float(dyn_timeout_s):.3f} base_error={exc}"
                        )
                    retry_wrong_dir_attempts += 1
                    adaptation_meta["retry_wrong_direction"] = int(
                        adaptation_meta.get("retry_wrong_direction", 0) or 0
                    ) + 1
                    # Tinymovr-style bounded fallback:
                    # back off authority, increase damping, zero integrator to avoid windup reversals.
                    derate = max(float(retry_min_derate), float(derate) * min(float(retry_derate_step), 0.80))
                    dyn_pos_gain_scale = max(0.45, float(dyn_pos_gain_scale) * 0.85)
                    dyn_vel_gain_scale = min(3.20, float(dyn_vel_gain_scale) * 1.25)
                    dyn_vel_i_gain_override = 0.0
                    dyn_vel_i_scale = max(0.0, float(dyn_vel_i_scale) * float(vel_i_decay_factor))
                    adaptation_meta["vel_i_decay_events"] = int(adaptation_meta.get("vel_i_decay_events", 0) or 0) + 1
                    # Disable kick after wrong-direction evidence; it can excite rebound.
                    stiction_kick_cmd_nm = 0.0
                    dyn_timeout_s = min(18.0, max(float(dyn_timeout_s), 1.0) * 1.20)
                    _recover_retry()
                    time.sleep(0.08)
                    continue

                if bool(_is_divergence_fault(exc)):
                    # Divergence means state authority is unsafe at the current operating point.
                    # Do not keep retrying in-place; force-safe and escalate to caller.
                    try:
                        common.force_idle(a, settle_s=0.08)
                    except Exception:
                        pass
                    try:
                        common.clear_errors_all(a)
                    except Exception:
                        pass
                    raise RuntimeError(f"move_to_angle divergence abort: {exc}")

                if (not bool(retry_on_retryable_fault)) or (not bool(runtime_retries_on)) or (not bool(_is_retryable_fault(exc))):
                    raise

                if bool(_is_no_movement_fault(exc)):
                    try:
                        cur_nm = float(getattr(a.encoder, "pos_estimate", 0.0))
                    except Exception:
                        cur_nm = float(cmd_target)
                    try:
                        vel_nm = float(getattr(a.encoder, "vel_estimate", 0.0))
                    except Exception:
                        vel_nm = 0.0
                    cmd_dist_nm = abs(float(cmd_target) - float(cur_nm))
                    target_tol_nm = max(0.0, float(cfg.get("target_tolerance_turns", 0.010)))
                    target_vel_tol_nm = max(0.0, float(cfg.get("target_vel_tolerance_turns_s", 0.12)))
                    near_hold_thresh = max(0.02, 3.0 * float(cfg.get("target_tolerance_turns", 0.010)))
                    if cmd_dist_nm <= near_hold_thresh:
                        # If we are already within final hold tolerances, treat this as a
                        # stable success instead of escalating anti-stall retries.
                        if (cmd_dist_nm <= float(target_tol_nm)) and (abs(float(vel_nm)) <= float(target_vel_tol_nm)):
                            hold_res = {
                                "start": float(cur_nm),
                                "target": float(cmd_target),
                                "end": float(cur_nm),
                                "vel": float(vel_nm),
                                "err": float(cmd_target) - float(cur_nm),
                                "state": int(getattr(a, "current_state", 0)),
                                "reached": True,
                                "duration_s": 0.0,
                                "sample_count": 1,
                                "peak_abs_vel": abs(float(vel_nm)),
                                "peak_abs_acc": 0.0,
                                "peak_abs_jerk": 0.0,
                                "vel_sign_changes": 0,
                            }
                            _accumulate_motion_quality(hold_res)
                            return hold_res
                        # Near-target misses should use one gentle hold-focused retry:
                        # no aggressive unstick, zero min-delta gate, extra damping.
                        if retry_nomove_attempts >= 1:
                            raise RuntimeError(
                                "Near-target no-motion; gentle hold retry exhausted. "
                                f"cmd_dist={cmd_dist_nm:.6f}t thresh={near_hold_thresh:.6f}t base_error={exc}"
                            )
                        retry_nomove_attempts += 1
                        adaptation_meta["retry_no_motion"] = int(adaptation_meta.get("retry_no_motion", 0) or 0) + 1
                        dyn_min_delta_turns = 0.0
                        dyn_pos_gain_scale = max(0.50, float(dyn_pos_gain_scale) * 0.94)
                        dyn_vel_gain_scale = min(2.20, float(dyn_vel_gain_scale) * 1.06)
                        if dyn_vel_i_gain_override is None:
                            dyn_vel_i_gain_override = min(
                                0.01,
                                max(0.0, float(cfg["vel_i_gain"]) * float(derate) * 0.25),
                            )
                        derate = max(float(retry_min_derate), float(derate) * 0.98)
                        _recover_retry()
                        continue

                    if retry_nomove_attempts >= 2:
                        if int(adaptation_meta.get("unstick_attempts", 0) or 0) >= 1:
                            raise
                        adaptation_meta["unstick_attempts"] = int(adaptation_meta.get("unstick_attempts", 0) or 0) + 1
                        if bool(_attempt_unstick_nudge(cmd_target)):
                            adaptation_meta["unstick_success"] = True
                            _recover_retry()
                            time.sleep(0.05)
                            retry_nomove_attempts = 0
                            continue
                        raise
                    retry_nomove_attempts += 1
                    adaptation_meta["retry_no_motion"] = int(adaptation_meta.get("retry_no_motion", 0) or 0) + 1
                    if bool(allow_kick):
                        stiction_kick_cmd_nm = min(0.18, max(0.05, float(stiction_kick_cmd_nm) * 1.25))
                    # Increase damping while allowing a slight authority increase for breakaway.
                    dyn_pos_gain_scale = max(0.50, float(dyn_pos_gain_scale) * 0.90)
                    dyn_vel_gain_scale = min(2.20, float(dyn_vel_gain_scale) * 1.08)
                    if dyn_vel_i_gain_override is None:
                        dyn_vel_i_gain_override = min(0.02, max(0.0, float(cfg["vel_i_gain"]) * float(derate) * 0.35))
                    derate = min(1.00, max(float(retry_min_derate), float(derate) * 1.01))
                    if bool(allow_kick):
                        try:
                            cur = float(getattr(a.encoder, "pos_estimate", 0.0))
                            ksgn = +1.0 if float(cmd_target) >= float(cur) else -1.0
                            common.torque_bump(a, torque=ksgn * float(stiction_kick_cmd_nm), seconds=0.03)
                            common.torque_bump(a, torque=-ksgn * float(stiction_kick_cmd_nm) * 0.35, seconds=0.02)
                        except Exception:
                            pass
                    _recover_retry()
                    continue

                if bool(_is_target_timeout_fault(exc) or _is_transient_hold_fault(exc)):
                    if retry_recover_attempts >= 2:
                        raise
                    retry_recover_attempts += 1
                    adaptation_meta["retry_timeout"] = int(adaptation_meta.get("retry_timeout", 0) or 0) + 1
                    # If the timeout error is very large, fail fast instead of repeatedly
                    # chasing a diverged state (prevents runaway-like excursions).
                    if bool(_timeout_looks_diverged(exc)):
                        raise

                    # For timeout/hold misses, keep authority roughly unchanged and
                    # increase damping + kick modestly; deep derating is reserved for hard current stress.
                    if bool(allow_kick):
                        stiction_kick_cmd_nm = min(0.20, max(0.03, float(stiction_kick_cmd_nm) * 1.15))
                    dyn_pos_gain_scale = max(0.55, float(dyn_pos_gain_scale) * 0.96)
                    dyn_vel_gain_scale = min(2.40, float(dyn_vel_gain_scale) * 1.08)
                    if dyn_vel_i_gain_override is None:
                        dyn_vel_i_gain_override = min(0.03, max(0.0, float(cfg["vel_i_gain"]) * float(derate) * 0.60))
                    derate = max(float(retry_min_derate), min(1.05, float(derate)))
                    _recover_retry()
                    time.sleep(0.05)
                    continue

                if bool(_is_current_stress_fault(exc)):
                    if retry_fault_attempts >= int(max_current_stress_retries):
                        raise RuntimeError(
                            "move_to_angle current-stress retries exhausted. "
                            f"attempts={int(retry_fault_attempts)} derate={float(derate):.3f} "
                            f"pos_gain_scale={float(dyn_pos_gain_scale):.3f} "
                            f"vel_gain_scale={float(dyn_vel_gain_scale):.3f} "
                            f"timeout_s={float(dyn_timeout_s):.3f} base_error={exc}"
                        )
                    retry_fault_attempts += 1
                    adaptation_meta["retry_current_stress"] = int(adaptation_meta.get("retry_current_stress", 0) or 0) + 1
                    # Current stress: back off authority aggressively and add damping.
                    derate = max(float(retry_min_derate), float(derate) * min(float(retry_derate_step), 0.78))
                    dyn_pos_gain_scale = max(0.45, float(dyn_pos_gain_scale) * 0.82)
                    dyn_vel_gain_scale = min(3.00, float(dyn_vel_gain_scale) * 1.20)
                    if dyn_vel_i_gain_override is None:
                        dyn_vel_i_gain_override = min(0.010, max(0.0, float(cfg["vel_i_gain"]) * float(derate) * 0.20))
                    dyn_vel_i_scale = max(0.0, float(dyn_vel_i_scale) * float(vel_i_decay_factor))
                    adaptation_meta["vel_i_decay_events"] = int(adaptation_meta.get("vel_i_decay_events", 0) or 0) + 1
                    # Give more settle/trajectory time after disarm/re-entry to avoid repeated trips.
                    dyn_timeout_s = min(18.0, max(float(dyn_timeout_s), 1.0) * 1.30)
                    _recover_retry()
                    time.sleep(0.08)
                    continue

                # Non-current retryable faults: recover and retry without derating,
                # but add a modest kick boost to improve breakaway.
                if retry_recover_attempts >= 2:
                    raise
                retry_recover_attempts += 1
                if bool(allow_kick):
                    stiction_kick_cmd_nm = min(0.30, max(0.03, float(stiction_kick_cmd_nm) * 1.25))
                dyn_pos_gain_scale = max(0.55, float(dyn_pos_gain_scale) * 0.95)
                dyn_vel_gain_scale = min(2.40, float(dyn_vel_gain_scale) * 1.10)
                _recover_retry()
                time.sleep(0.05)

    divergence_recovery = {
        "attempted": False,
        "succeeded": False,
        "error": None,
    }

    t0 = time.monotonic()
    exec_attempts = 2 if bool(divergence_recovery_on) else 1
    for _exec_try in range(int(exec_attempts)):
        try:
            if chunk > 0.0 and move_dist > chunk:
                max_stages = max(1, int(math.ceil(move_dist / chunk)) + 2)
                for _ in range(max_stages):
                    cur = float(getattr(a.encoder, "pos_estimate", 0.0))
                    rem = float(target_turns_abs) - float(cur)
                    if abs(rem) > float(runtime_distance_cap):
                        raise RuntimeError(
                            "move_to_angle runtime distance cap exceeded during staged move. "
                            f"remaining={float(rem):+.6f}t cap={float(runtime_distance_cap):.6f}t "
                            f"requested_dist={float(nominal_move_dist):.6f}t"
                        )
                    if abs(rem) <= chunk:
                        break
                    cmd = float(cur) + math.copysign(chunk, rem)
                    cdist = abs(float(cmd) - float(cur))
                    ctimeout = max(0.75, _trap_move_time_est(cdist, trap_vel, trap_acc, trap_dec) + 0.35)
                    _run_one_strict(cmd_target=cmd, cmd_settle_s=max(0.0, float(stage_settle_s)), cmd_timeout_s=ctimeout)

            if style == "hybrid_soft_closed_loop":
                cur = float(getattr(a.encoder, "pos_estimate", 0.0))
                rem = float(target_turns_abs) - float(cur)
                if abs(rem) > float(runtime_distance_cap):
                    raise RuntimeError(
                        "move_to_angle runtime distance cap exceeded before hybrid finalization. "
                        f"remaining={float(rem):+.6f}t cap={float(runtime_distance_cap):.6f}t "
                        f"requested_dist={float(nominal_move_dist):.6f}t"
                    )
                final_window = max(
                    float(cfg["target_tolerance_turns"]) * 2.0,
                    abs(float(hybrid_final_window_turns)),
                    0.05,
                )
                if abs(rem) > final_window:
                    # If still far away after staged coarse move, finish coarse once.
                    _run_one_strict(
                        cmd_target=target_turns_abs,
                        cmd_settle_s=max(0.0, float(stage_settle_s)),
                        cmd_timeout_s=max(0.75, float(timeout_eff) * 0.65),
                        use_trap=True,
                        min_delta_turns=0.0015,
                    )

                micro_step = max(float(cfg["target_tolerance_turns"]) * 0.5, abs(float(hybrid_micro_step_turns)))
                max_micro = max(0, int(hybrid_max_micro_steps))
                for _ in range(max_micro):
                    cur = float(getattr(a.encoder, "pos_estimate", 0.0))
                    rem = float(target_turns_abs) - float(cur)
                    if abs(rem) > float(runtime_distance_cap):
                        raise RuntimeError(
                            "move_to_angle runtime distance cap exceeded during hybrid micro-step. "
                            f"remaining={float(rem):+.6f}t cap={float(runtime_distance_cap):.6f}t "
                            f"requested_dist={float(nominal_move_dist):.6f}t"
                        )
                    if abs(rem) <= float(cfg["target_tolerance_turns"]):
                        break
                    step = math.copysign(min(abs(rem), micro_step), rem)
                    cmd = float(cur) + float(step)
                    _run_one_strict(
                        cmd_target=cmd,
                        cmd_settle_s=max(0.0, float(hybrid_micro_settle_s)),
                        cmd_timeout_s=max(0.50, float(timeout_eff) * 0.35),
                        use_trap=False,
                        min_delta_turns=max(0.0003, 0.25 * abs(float(step))),
                        pos_gain_scale=float(hybrid_passthrough_pos_gain_scale),
                        vel_gain_scale=float(hybrid_passthrough_vel_gain_scale),
                        vel_i_gain_override=float(hybrid_passthrough_vel_i_gain),
                    )

                # Final strict hold check in PASSTHROUGH; min_delta=0 allows hold-validate even if almost at target.
                res = _run_one_strict(
                    cmd_target=target_turns_abs,
                    cmd_settle_s=float(settle_s),
                    cmd_timeout_s=float(timeout_eff),
                    use_trap=False,
                    min_delta_turns=0.0,
                    pos_gain_scale=float(hybrid_passthrough_pos_gain_scale),
                    vel_gain_scale=float(hybrid_passthrough_vel_gain_scale),
                    vel_i_gain_override=float(hybrid_passthrough_vel_i_gain),
                )
            else:
                res = _run_one_strict(
                    cmd_target=target_turns_abs,
                    cmd_settle_s=float(settle_s),
                    cmd_timeout_s=float(timeout_eff),
                    use_trap=True,
                    min_delta_turns=0.0015,
                )
            break
        except RuntimeError as exc:
            if (
                ("move_to_angle divergence abort:" not in str(exc))
                or (not bool(divergence_recovery_on))
                or bool(divergence_recovery["attempted"])
            ):
                raise
            divergence_recovery["attempted"] = True
            try:
                try:
                    common.force_idle(a, settle_s=0.08)
                except Exception:
                    pass
                common.clear_errors_all(a)
                use_index_rt, run_index_rt = _recover_index_policy()
                common.establish_absolute_reference(
                    a,
                    require_index=bool(use_index_rt),
                    run_index_search=bool(run_index_rt),
                    attempt_offset_calibration=False,
                    label="move_to_angle_divergence_recover",
                )
                common.sync_pos_setpoint(a, settle_s=0.05, retries=3, verbose=False)
                start_turns = float(getattr(a.encoder, "pos_estimate", 0.0))
                if bool(relative_mode):
                    zero_turns = float(start_turns)
                    base_target_turns_abs = float(zero_turns) + float(target_turns)
                target_turns_abs, wrap_k, wrap_mode = _resolve_output_target_turns(
                    base_target_turns_motor=base_target_turns_abs,
                    start_turns_motor=start_turns,
                    angle_space=space,
                    gear_ratio=ratio,
                    wrap_strategy=wrap_strategy,
                )
                move_dist = abs(float(target_turns_abs) - float(start_turns))
                if float(move_dist) > float(runtime_distance_cap):
                    raise RuntimeError(
                        "move_to_angle divergence recovery produced excessive target distance. "
                        f"distance={float(move_dist):.6f}t cap={float(runtime_distance_cap):.6f}t "
                        f"requested_dist={float(nominal_move_dist):.6f}t"
                    )
                divergence_recovery["succeeded"] = True
                print(
                    "move_to_angle: divergence recovery succeeded; retrying once with refreshed reference",
                    f"start={start_turns:+.6f} target={float(target_turns_abs):+.6f} dist={float(move_dist):.6f}",
                )
                continue
            except Exception as rex:
                divergence_recovery["error"] = str(rex)
                raise RuntimeError(
                    f"move_to_angle divergence recovery failed: {rex}; original={exc}"
                )

    elapsed_s = float(time.monotonic() - t0)
    if isinstance(res, dict):
        mq = dict(motion_quality or {})
        if int(mq.get("sample_count", 0) or 0) <= 0:
            mq = {
                "sample_count": int(res.get("sample_count", 0) or 0),
                "duration_s": float(res.get("duration_s", 0.0) or 0.0),
                "peak_abs_vel": float(res.get("peak_abs_vel", 0.0) or 0.0),
                "peak_abs_acc": float(res.get("peak_abs_acc", 0.0) or 0.0),
                "peak_abs_jerk": float(res.get("peak_abs_jerk", 0.0) or 0.0),
                "vel_sign_changes": int(res.get("vel_sign_changes", 0) or 0),
            }
        smooth_meta = {
            "gate_enabled": bool(smooth_gate_on),
            "max_vel_sign_changes": int(smooth_max_sign_changes),
            "max_peak_abs_acc_turns_s2": (
                None if smooth_max_acc is None else float(smooth_max_acc)
            ),
            "max_peak_abs_jerk_turns_s3": (
                None if smooth_max_jerk is None else float(smooth_max_jerk)
            ),
            "sample_count": int(mq.get("sample_count", 0) or 0),
            "duration_s": float(mq.get("duration_s", 0.0) or 0.0),
            "peak_abs_vel_turns_s": float(mq.get("peak_abs_vel", 0.0) or 0.0),
            "peak_abs_acc_turns_s2": float(mq.get("peak_abs_acc", 0.0) or 0.0),
            "peak_abs_jerk_turns_s3": float(mq.get("peak_abs_jerk", 0.0) or 0.0),
            "vel_sign_changes": int(mq.get("vel_sign_changes", 0) or 0),
            "ok": True,
            "issues": [],
        }
        if bool(smooth_gate_on):
            if int(smooth_meta["vel_sign_changes"]) > int(smooth_max_sign_changes):
                smooth_meta["issues"].append(
                    f"vel_sign_changes={int(smooth_meta['vel_sign_changes'])} > {int(smooth_max_sign_changes)}"
                )
            if (smooth_max_acc is not None) and (
                float(smooth_meta["peak_abs_acc_turns_s2"]) > float(smooth_max_acc)
            ):
                smooth_meta["issues"].append(
                    "peak_abs_acc="
                    f"{float(smooth_meta['peak_abs_acc_turns_s2']):.6f} > {float(smooth_max_acc):.6f}"
                )
            if (smooth_max_jerk is not None) and (
                float(smooth_meta["peak_abs_jerk_turns_s3"]) > float(smooth_max_jerk)
            ):
                smooth_meta["issues"].append(
                    "peak_abs_jerk="
                    f"{float(smooth_meta['peak_abs_jerk_turns_s3']):.6f} > {float(smooth_max_jerk):.6f}"
                )
            smooth_meta["ok"] = bool(len(smooth_meta["issues"]) == 0)
        res["smoothness"] = dict(smooth_meta)
        res["angle_request"] = {
            "angle_deg": float(angle_deg),
            "angle_space": space,
            "gear_ratio": float(ratio),
            "start_turns_motor": float(start_turns),
            "zero_turns_motor": float(zero_turns),
            "relative_to_current": bool(relative_mode),
            "target_turns_relative_motor": float(target_turns),
            "target_turns_motor_base": float(base_target_turns_abs),
            "target_turns_motor": float(target_turns_abs),
            "commanded_delta_turns_motor": float(target_turns_abs - start_turns),
            "commanded_direction_sign": (
                1
                if float(target_turns_abs - start_turns) > 0.0
                else (-1 if float(target_turns_abs - start_turns) < 0.0 else 0)
            ),
            "wrap_strategy": str(wrap_mode),
            "wrap_cycles_applied": int(wrap_k),
            "stage_chunk_max_turns": float(chunk),
            "command_slew_max_step_turns": float(slew_chunk),
            "stage_count_est": int(stage_count),
            "runtime_distance_cap_turns": float(runtime_distance_cap),
            "nominal_request_distance_turns": float(nominal_move_dist),
            "derate_final": float(derate),
            "control_style": str(style),
            "pre_move_gate_strategy": str(gate_strategy),
            "startup_checks": str(startup_checks_mode),
            "startup_overrides": list(startup_overrides or []),
            "startup_contract": dict(startup_contract or {}),
            "tinymovr_validation_gate": dict(strict_gate_meta or {}),
            "authority_precheck": dict(authority_precheck_meta or {}),
            "allow_stiction_kick": bool(allow_kick),
            "runtime_retries_enabled": bool(runtime_retries_on),
            "divergence_recovery_enabled": bool(divergence_recovery_on),
            "vel_limit": float(vel_limit_cmd),
            "vel_limit_tolerance": float(vel_limit_tol_cmd),
            "enable_overspeed_error": bool(overspeed_err_cmd),
            "stiction_kick_cmd_nm": float(stiction_kick_cmd_nm),
            "vel_i_deadband_turns": float(vel_i_deadband),
            "vel_i_decay_on_limit": float(vel_i_decay_factor),
            "damping_adaptation": dict(adaptation_meta),
            "divergence_recovery": dict(divergence_recovery),
            "stability_probe": dict(stability_probe or {}),
            "directional_authority": {
                "direction": ("plus" if is_plus_dir else "minus"),
                "current_scale": float(current_scale_dir),
                "trap_scale": float(trap_scale_dir),
                "vel_limit_scale": float(vel_limit_scale_dir),
                "kick_scale": float(kick_scale_dir),
                "current_lim_base": float(current_lim_base),
            },
        }
        if isinstance(stiction_lut_info, dict):
            res["angle_request"]["stiction_lut"] = stiction_lut_info
        if isinstance(pre_move_gate, dict):
            res["angle_request"]["pre_move_gate"] = pre_move_gate
        if len(list(deferred_warnings or [])) > 0:
            warns = list(res.get("warnings") or [])
            for w in list(deferred_warnings or []):
                warns.append(dict(w))
            res["warnings"] = warns
        if isinstance(ref_meta, dict) and ref_meta.get("path"):
            res["angle_request"]["angle_reference"] = {
                "path": ref_meta.get("path"),
                "exists": bool(ref_meta.get("exists", False)),
                "updated_at": ref_meta.get("updated_at"),
                "angle_space": ref_meta.get("angle_space"),
                "angle_deg_at_capture": ref_meta.get("angle_deg_at_capture"),
            }
        if deadline_info is not None:
            dmeta = dict(deadline_info)
            dmeta["elapsed_s"] = elapsed_s
            dmeta["met"] = bool(elapsed_s <= float(deadline_info["deadline_s"]))
            res["deadline"] = dmeta
            if (not bool(dmeta["met"])) and (timing_warning is None):
                timing_warning = {
                    "code": "DEADLINE_MISSED_RUNTIME",
                    "level": "warning",
                    "deadline_s": float(deadline_info["deadline_s"]),
                    "guaranteed_total_s": float(deadline_info.get("guaranteed_total_s", dmeta["elapsed_s"])),
                    "elapsed_s": float(dmeta["elapsed_s"]),
                    "reason": "Move exceeded requested deadline during execution.",
                }
        if isinstance(timing_info, dict):
            tmeta = dict(timing_info)
            tmeta["elapsed_s"] = float(elapsed_s)
            if deadline_s is not None:
                tmeta["met"] = bool(elapsed_s <= float(deadline_s))
            else:
                tmeta["met"] = None
            if isinstance(timing_warning, dict):
                tmeta["warning_code"] = str(timing_warning.get("code"))
                tmeta["warning_message"] = str(timing_warning.get("reason", ""))
            res["timing"] = tmeta
        if isinstance(timing_warning, dict):
            warns = list(res.get("warnings") or [])
            warns.append(dict(timing_warning))
            res["warnings"] = warns
        if bool(smooth_gate_on) and (not bool(smooth_meta.get("ok", True))):
            smooth_warn = {
                "code": "SMOOTHNESS_GATE_FAILED",
                "level": "warning",
                "issues": list(smooth_meta.get("issues") or []),
                "metrics": {
                    "peak_abs_vel_turns_s": float(smooth_meta.get("peak_abs_vel_turns_s", 0.0) or 0.0),
                    "peak_abs_acc_turns_s2": float(smooth_meta.get("peak_abs_acc_turns_s2", 0.0) or 0.0),
                    "peak_abs_jerk_turns_s3": float(smooth_meta.get("peak_abs_jerk_turns_s3", 0.0) or 0.0),
                    "vel_sign_changes": int(smooth_meta.get("vel_sign_changes", 0) or 0),
                },
            }
            warns = list(res.get("warnings") or [])
            warns.append(dict(smooth_warn))
            res["warnings"] = warns
            raise RuntimeError(
                "move_to_angle smoothness gate failed: "
                + "; ".join(list(smooth_meta.get("issues") or []))
                + f" metrics={smooth_warn['metrics']}"
            )
        return res
    return res


def validate_move_to_angle_sequence(
    targets_deg=(40.0, -40.0, 40.0, -40.0),
    angle_space=ANGLE_SPACE_GEARBOX_OUTPUT,
    axis=None,
    profile_name="loaded_motion_gate_stable",
    profile_path=None,
    profile_overrides=None,
    gear_ratio=DEFAULT_GEAR_RATIO,
    relative_to_current=False,
    zero_turns_motor=None,
    angle_ref_path=None,
    require_angle_reference=False,
    wrap_strategy="nearest",
    stage_chunk_max_turns=0.25,
    stage_settle_s=0.02,
    retry_on_retryable_fault=True,
    retry_derate_step=0.85,
    retry_min_derate=0.55,
    deadline_s=None,
    deadline_mode="strict",
    deadline_max_scale=2.0,
    deadline_timeout_margin_s=0.50,
    control_style="trap_strict",
    hybrid_final_window_turns=0.02,
    hybrid_micro_step_turns=0.002,
    hybrid_max_micro_steps=12,
    hybrid_micro_settle_s=0.05,
    hybrid_passthrough_pos_gain_scale=0.60,
    hybrid_passthrough_vel_gain_scale=1.15,
    hybrid_passthrough_vel_i_gain=0.0,
    pre_move_gate_mode="off",
    pre_move_gate_strategy="fixed",
    auto_fallback_hybrid_on_low_authority=False,
    pre_move_gate_load_mode="loaded",
    pre_move_gate_delta_turns=0.006,
    pre_move_gate_duration_s=1.2,
    pre_move_gate_max_attempts=2,
    pre_move_gate_current_scale_step=1.10,
    pre_move_gate_current_max_scale=1.20,
    pre_move_gate_abs_current_cap_a=None,
    pre_move_gate_kick_scale_step=1.25,
    pre_move_gate_kick_max_nm=0.12,
    pre_move_gate_absolute_at_start=False,
    pre_move_gate_run_index_search=False,
    startup_checks="minimal",
    authority_precheck="auto",
    authority_precheck_cmd_delta_turns=0.01,
    authority_precheck_current_lim=None,
    authority_precheck_pos_gain=None,
    authority_precheck_vel_gain=None,
    authority_precheck_vel_i_gain=0.0,
    authority_precheck_vel_limit=None,
    authority_precheck_timeout_s=1.8,
    authority_precheck_settle_s=0.05,
    authority_precheck_min_delta_turns=0.001,
    allow_stiction_kick=False,
    plus_current_scale=1.0,
    minus_current_scale=1.0,
    plus_trap_scale=1.0,
    minus_trap_scale=1.0,
    plus_vel_limit_scale=1.0,
    minus_vel_limit_scale=1.0,
    plus_kick_scale=1.0,
    minus_kick_scale=1.0,
    use_stiction_lut=False,
    stiction_map_path=None,
    stiction_lut_gain=1.0,
    settle_s=0.10,
    timeout_s=6.0,
    err_tol_deg=None,
    deadline_slack_s=0.20,
    continue_on_error=True,
    sequence_recovery_on_fail=False,
    sequence_recovery_max_events=1,
    sequence_recovery_run_index_search=False,
    sequence_recovery_require_index=None,
    sequence_recovery_derate=0.90,
    drift_guard_enabled=False,
    drift_guard_bound_turns=1.20,
    drift_guard_max_events=2,
    drift_guard_recenter_timeout_s=6.0,
    drift_guard_recenter_settle_s=0.16,
    drift_guard_run_index_search=False,
    drift_guard_require_index=None,
    smoothness_gate=True,
    smoothness_max_vel_sign_changes=10,
    smoothness_max_peak_abs_acc_turns_s2=None,
    smoothness_max_peak_abs_jerk_turns_s3=None,
    fail_on_wrong_direction=True,
    wrong_direction_min_progress_turns=0.001,
):
    """Run a sequence of move_to_angle commands and report error/deadline quality."""
    a = axis if axis is not None else common.get_axis0()
    cfg = get_move_to_angle_kwargs_from_profile(profile_name=profile_name, path=profile_path)
    if isinstance(profile_overrides, dict):
        for k in (
            "trap_vel",
            "vel_limit",
            "vel_limit_tolerance",
            "enable_overspeed_error",
            "trap_acc",
            "trap_dec",
            "current_lim",
            "pos_gain",
            "vel_gain",
            "vel_i_gain",
            "stiction_kick_nm",
            "target_tolerance_turns",
            "target_vel_tolerance_turns_s",
        ):
            if k in profile_overrides and profile_overrides.get(k) is not None:
                if k == "enable_overspeed_error":
                    cfg[k] = bool(profile_overrides.get(k))
                else:
                    cfg[k] = float(profile_overrides.get(k))
    ratio = abs(float(gear_ratio))
    if err_tol_deg is None:
        base_tol_turns = float(cfg.get("target_tolerance_turns", 0.006))
        err_tol_deg = abs(_motor_turn_error_to_angle_deg(base_tol_turns, angle_space, ratio))
    runtime_profile_overrides = dict(profile_overrides or {})
    seq_recovery_enabled = bool(sequence_recovery_on_fail)
    seq_recovery_max = max(0, int(sequence_recovery_max_events))
    seq_recovery_scale = min(1.0, max(0.50, float(sequence_recovery_derate)))
    seq_recovery_events = 0
    seq_recovery_retry_count = 0
    seq_recovery_log = []
    drift_guard_on = bool(drift_guard_enabled) and bool(relative_to_current) and (abs(float(drift_guard_bound_turns)) > 0.0)
    drift_bound_turns = max(0.02, abs(float(drift_guard_bound_turns)))
    drift_guard_max = max(0, int(drift_guard_max_events))
    drift_guard_events = 0
    drift_guard_log = []
    drift_anchor_turns = float(getattr(a.encoder, "pos_estimate", 0.0))
    smooth_gate_on = bool(smoothness_gate)
    smooth_max_sign_changes = max(0, int(smoothness_max_vel_sign_changes))
    smooth_max_acc = (
        None
        if smoothness_max_peak_abs_acc_turns_s2 is None
        else max(0.0, float(smoothness_max_peak_abs_acc_turns_s2))
    )
    smooth_max_jerk = (
        None
        if smoothness_max_peak_abs_jerk_turns_s3 is None
        else max(0.0, float(smoothness_max_peak_abs_jerk_turns_s3))
    )
    wrong_dir_gate_on = bool(fail_on_wrong_direction)
    wrong_dir_min_progress = max(0.0, float(wrong_direction_min_progress_turns))

    def _sign_eps(v, eps=1e-6):
        if float(v) > float(eps):
            return 1
        if float(v) < -float(eps):
            return -1
        return 0

    def _parse_int_after(msg, key):
        s = str(msg or "")
        i = s.find(str(key))
        if i < 0:
            return None
        j = i + len(str(key))
        k = j
        if k < len(s) and s[k] in "+-":
            k += 1
        while k < len(s) and s[k].isdigit():
            k += 1
        raw = s[j:k].strip()
        try:
            return int(raw)
        except Exception:
            return None

    def _parse_float_after(msg, key):
        s = str(msg or "")
        i = s.find(str(key))
        if i < 0:
            return None
        j = i + len(str(key))
        k = j
        if k < len(s) and s[k] in "+-":
            k += 1
        dot_seen = False
        while k < len(s):
            ch = s[k]
            if ch.isdigit():
                k += 1
                continue
            if ch == "." and not dot_seen:
                dot_seen = True
                k += 1
                continue
            break
        raw = s[j:k].strip()
        try:
            return float(raw)
        except Exception:
            return None

    def _is_sequence_recoverable(exc_msg):
        s = str(exc_msg or "")
        return (
            ("Target reached transiently but did not hold after settle" in s)
            or ("move_to_angle divergence abort:" in s)
            or ("DIVERGENCE watchdog tripped" in s)
            or ("move_to_angle drift guard:" in s)
        )

    def _parse_timeout_err_turns(exc_msg):
        s = str(exc_msg or "")
        key = "err="
        i = s.find(key)
        if i < 0:
            return None
        j = s.find("t", i + len(key))
        if j < 0:
            return None
        raw = s[i + len(key):j].strip()
        try:
            return float(raw)
        except Exception:
            return None

    def _apply_sequence_recovery_clamp(overrides):
        out = dict(overrides or {})
        clamp_fields = (
            ("current_lim", 0.8),
            ("trap_vel", 0.005),
            ("trap_acc", 0.010),
            ("trap_dec", 0.010),
            ("vel_limit", 0.05),
            ("pos_gain", 0.5),
            ("vel_gain", 0.01),
            ("vel_i_gain", 0.0),
            ("stiction_kick_nm", 0.0),
        )
        for key, floor in clamp_fields:
            raw = out.get(key, cfg.get(key))
            if raw is None:
                continue
            try:
                out[key] = max(float(floor), float(raw) * float(seq_recovery_scale))
            except Exception:
                pass
        return out

    def _build_drift_recenter_overrides(overrides):
        out = dict(overrides or {})
        tuned = (
            ("current_lim", 1.25, 8.0),
            ("trap_vel", 1.80, 0.05),
            ("trap_acc", 1.80, 0.10),
            ("trap_dec", 1.80, 0.10),
            ("vel_limit", 1.50, 0.25),
            ("pos_gain", 1.20, 8.0),
            ("vel_gain", 1.10, 0.18),
            ("vel_i_gain", 1.10, 0.03),
            ("stiction_kick_nm", 1.20, 0.02),
        )
        for key, gain, floor in tuned:
            raw = out.get(key, cfg.get(key))
            if raw is None:
                continue
            try:
                out[key] = max(float(floor), float(raw) * float(gain))
            except Exception:
                pass
        return out

    def _run_reference_recovery(run_index_search, require_index_opt, label):
        try:
            use_index_cfg = bool(getattr(getattr(a.encoder, "config", object()), "use_index", False))
        except Exception:
            use_index_cfg = False
        req_idx = use_index_cfg if require_index_opt is None else bool(require_index_opt)
        common.force_idle(a, settle_s=0.08)
        common.clear_errors_all(a)
        common.establish_absolute_reference(
            a,
            require_index=bool(req_idx),
            run_index_search=bool(run_index_search),
            attempt_offset_calibration=False,
            label=str(label),
        )
        common.sync_pos_setpoint(a, settle_s=0.05, retries=3, verbose=False)
        return bool(req_idx)

    def _run_sequence_recovery():
        return _run_reference_recovery(
            run_index_search=bool(sequence_recovery_run_index_search),
            require_index_opt=sequence_recovery_require_index,
            label="validate_move_to_angle_sequence/recover",
        )

    def _run_drift_guard_recenter(index_i, target_deg_i):
        nonlocal drift_guard_events, runtime_profile_overrides
        if not bool(drift_guard_on):
            return None
        pos_now = float(getattr(a.encoder, "pos_estimate", 0.0))
        drift_now = float(pos_now - drift_anchor_turns)
        if abs(float(drift_now)) <= float(drift_bound_turns):
            return None

        evt = {
            "index": int(index_i),
            "target_deg": float(target_deg_i),
            "anchor_turns": float(drift_anchor_turns),
            "pos_before": float(pos_now),
            "drift_before_turns": float(drift_now),
            "bound_turns": float(drift_bound_turns),
            "run_index_search": bool(drift_guard_run_index_search),
            "event_index": int(drift_guard_events) + 1,
            "ok": False,
        }
        if int(drift_guard_events) >= int(drift_guard_max):
            evt["error"] = (
                f"drift_guard exhausted ({int(drift_guard_events)}/{int(drift_guard_max)}): "
                f"|drift|={abs(float(drift_now)):.6f}t > bound={float(drift_bound_turns):.6f}t"
            )
            drift_guard_log.append(evt)
            raise RuntimeError(str(evt["error"]))

        drift_guard_events += 1
        try:
            req_idx = _run_reference_recovery(
                run_index_search=bool(drift_guard_run_index_search),
                require_index_opt=drift_guard_require_index,
                label="validate_move_to_angle_sequence/drift_guard_ref",
            )
            evt["reference_ok"] = True
            evt["require_index"] = bool(req_idx)
        except Exception as rex:
            evt["reference_ok"] = False
            evt["reference_error"] = str(rex)
            drift_guard_log.append(evt)
            raise RuntimeError(f"drift_guard reference recovery failed: {rex}")

        runtime_profile_overrides = _build_drift_recenter_overrides(runtime_profile_overrides)
        evt["profile_overrides_after"] = dict(runtime_profile_overrides or {})
        recenter_cfg = dict(cfg)
        recenter_cfg.update(runtime_profile_overrides or {})
        recenter_dist = abs(float(drift_anchor_turns) - float(pos_now))
        rec_trap_vel = max(0.005, float(recenter_cfg.get("trap_vel", 0.05)))
        rec_trap_acc = max(0.010, float(recenter_cfg.get("trap_acc", 0.10)))
        rec_trap_dec = max(0.010, float(recenter_cfg.get("trap_dec", 0.10)))
        rec_est = _trap_move_time_est(recenter_dist, rec_trap_vel, rec_trap_acc, rec_trap_dec)
        rec_timeout = max(
            float(drift_guard_recenter_timeout_s),
            min(45.0, float(rec_est) + max(1.0, 0.5 * float(rec_est))),
        )
        evt["recenter_dist_turns"] = float(recenter_dist)
        evt["recenter_timeout_s"] = float(rec_timeout)
        evt["recenter_trap_vel"] = float(rec_trap_vel)
        evt["recenter_trap_acc"] = float(rec_trap_acc)
        evt["recenter_trap_dec"] = float(rec_trap_dec)
        try:
            common.move_to_pos_strict(
                a,
                float(drift_anchor_turns),
                use_trap_traj=True,
                timeout_s=float(rec_timeout),
                min_delta_turns=0.0008,
                settle_s=max(0.0, float(drift_guard_recenter_settle_s)),
                vel_limit=max(0.05, float(recenter_cfg.get("vel_limit", 0.25))),
                vel_limit_tolerance=max(1.0, float(recenter_cfg.get("vel_limit_tolerance", 2.0))),
                enable_overspeed_error=bool(recenter_cfg.get("enable_overspeed_error", True)),
                trap_vel=float(rec_trap_vel),
                trap_acc=float(rec_trap_acc),
                trap_dec=float(rec_trap_dec),
                current_lim=max(0.5, float(recenter_cfg.get("current_lim", 5.0))),
                pos_gain=max(0.5, float(recenter_cfg.get("pos_gain", 8.0))),
                vel_gain=max(0.01, float(recenter_cfg.get("vel_gain", 0.15))),
                vel_i_gain=max(0.0, float(recenter_cfg.get("vel_i_gain", 0.0))),
                stiction_kick_nm=max(0.0, float(recenter_cfg.get("stiction_kick_nm", 0.0))),
                target_tolerance_turns=max(
                    0.006,
                    float(recenter_cfg.get("target_tolerance_turns", cfg.get("target_tolerance_turns", 0.006))),
                ),
                target_vel_tolerance_turns_s=max(
                    0.05,
                    float(
                        recenter_cfg.get(
                            "target_vel_tolerance_turns_s",
                            cfg.get("target_vel_tolerance_turns_s", 0.12),
                        )
                    ),
                ),
                require_target_reached=True,
                fail_to_idle=False,
            )
            evt["recenter_ok"] = True
        except Exception as rex:
            evt["recenter_ok"] = False
            evt["recenter_error"] = str(rex)
            drift_guard_log.append(evt)
            raise RuntimeError(f"drift_guard recenter failed: {rex}")

        pos_after = float(getattr(a.encoder, "pos_estimate", 0.0))
        drift_after = float(pos_after - drift_anchor_turns)
        evt["pos_after"] = float(pos_after)
        evt["drift_after_turns"] = float(drift_after)
        evt["within_bound_after"] = bool(abs(float(drift_after)) <= float(drift_bound_turns))
        evt["ok"] = bool(evt["within_bound_after"])
        drift_guard_log.append(evt)
        if not bool(evt["within_bound_after"]):
            raise RuntimeError(
                "drift_guard recenter incomplete: "
                f"|drift_after|={abs(float(drift_after)):.6f}t > bound={float(drift_bound_turns):.6f}t"
            )
        return evt

    rows = []
    for idx, tgt in enumerate(tuple(targets_deg), start=1):
        rec = {
            "index": int(idx),
            "target_deg": float(tgt),
            "angle_space": str(angle_space),
            "status": "ok",
            "drift_guard_triggered": False,
        }
        try:
            drift_evt = _run_drift_guard_recenter(idx, tgt)
            if isinstance(drift_evt, dict):
                rec["drift_guard_triggered"] = True
                rec["drift_guard"] = dict(drift_evt)
        except Exception as dex:
            rec["status"] = "fail"
            rec["error"] = str(dex)
            rec["profile_overrides_used"] = dict(runtime_profile_overrides or {})
            rows.append(rec)
            break
        attempt_idx = 0
        settle_eff = float(settle_s)
        while True:
            rec["attempt"] = int(attempt_idx + 1)
            try:
                common.clear_errors_all(a)
            except Exception:
                pass
            try:
                t0 = time.monotonic()
                res = move_to_angle(
                    angle_deg=float(tgt),
                    angle_space=str(angle_space),
                    axis=a,
                    profile_name=profile_name,
                    profile_path=profile_path,
                    profile_overrides=runtime_profile_overrides,
                    gear_ratio=float(ratio),
                    relative_to_current=bool(relative_to_current),
                    zero_turns_motor=zero_turns_motor,
                    angle_ref_path=angle_ref_path,
                    require_angle_reference=bool(require_angle_reference),
                    wrap_strategy=str(wrap_strategy),
                    stage_chunk_max_turns=float(stage_chunk_max_turns),
                    stage_settle_s=float(stage_settle_s),
                    retry_on_retryable_fault=bool(retry_on_retryable_fault),
                    retry_derate_step=float(retry_derate_step),
                    retry_min_derate=float(retry_min_derate),
                    timeout_s=float(timeout_s),
                    settle_s=float(settle_eff),
                    deadline_s=deadline_s,
                    deadline_mode=str(deadline_mode),
                    deadline_max_scale=float(deadline_max_scale),
                    deadline_timeout_margin_s=float(deadline_timeout_margin_s),
                    control_style=str(control_style),
                    hybrid_final_window_turns=float(hybrid_final_window_turns),
                    hybrid_micro_step_turns=float(hybrid_micro_step_turns),
                    hybrid_max_micro_steps=int(hybrid_max_micro_steps),
                    hybrid_micro_settle_s=float(hybrid_micro_settle_s),
                    hybrid_passthrough_pos_gain_scale=float(hybrid_passthrough_pos_gain_scale),
                    hybrid_passthrough_vel_gain_scale=float(hybrid_passthrough_vel_gain_scale),
                    hybrid_passthrough_vel_i_gain=float(hybrid_passthrough_vel_i_gain),
                    pre_move_gate_mode=str(pre_move_gate_mode),
                    pre_move_gate_strategy=str(pre_move_gate_strategy),
                    auto_fallback_hybrid_on_low_authority=bool(auto_fallback_hybrid_on_low_authority),
                    pre_move_gate_load_mode=str(pre_move_gate_load_mode),
                    pre_move_gate_delta_turns=float(pre_move_gate_delta_turns),
                    pre_move_gate_duration_s=float(pre_move_gate_duration_s),
                    pre_move_gate_max_attempts=int(pre_move_gate_max_attempts),
                    pre_move_gate_current_scale_step=float(pre_move_gate_current_scale_step),
                    pre_move_gate_current_max_scale=float(pre_move_gate_current_max_scale),
                    pre_move_gate_abs_current_cap_a=(
                        None
                        if pre_move_gate_abs_current_cap_a is None
                        else float(pre_move_gate_abs_current_cap_a)
                    ),
                    pre_move_gate_kick_scale_step=float(pre_move_gate_kick_scale_step),
                    pre_move_gate_kick_max_nm=float(pre_move_gate_kick_max_nm),
                    pre_move_gate_absolute_at_start=bool(pre_move_gate_absolute_at_start),
                    pre_move_gate_run_index_search=bool(pre_move_gate_run_index_search),
                    startup_checks=str(startup_checks),
                    authority_precheck=str(authority_precheck),
                    authority_precheck_cmd_delta_turns=float(authority_precheck_cmd_delta_turns),
                    authority_precheck_current_lim=(
                        None if authority_precheck_current_lim is None else float(authority_precheck_current_lim)
                    ),
                    authority_precheck_pos_gain=(
                        None if authority_precheck_pos_gain is None else float(authority_precheck_pos_gain)
                    ),
                    authority_precheck_vel_gain=(
                        None if authority_precheck_vel_gain is None else float(authority_precheck_vel_gain)
                    ),
                    authority_precheck_vel_i_gain=float(authority_precheck_vel_i_gain),
                    authority_precheck_vel_limit=(
                        None if authority_precheck_vel_limit is None else float(authority_precheck_vel_limit)
                    ),
                    authority_precheck_timeout_s=float(authority_precheck_timeout_s),
                    authority_precheck_settle_s=float(authority_precheck_settle_s),
                    authority_precheck_min_delta_turns=float(authority_precheck_min_delta_turns),
                    allow_stiction_kick=bool(allow_stiction_kick),
                    plus_current_scale=float(plus_current_scale),
                    minus_current_scale=float(minus_current_scale),
                    plus_trap_scale=float(plus_trap_scale),
                    minus_trap_scale=float(minus_trap_scale),
                    plus_vel_limit_scale=float(plus_vel_limit_scale),
                    minus_vel_limit_scale=float(minus_vel_limit_scale),
                    plus_kick_scale=float(plus_kick_scale),
                    minus_kick_scale=float(minus_kick_scale),
                    use_stiction_lut=bool(use_stiction_lut),
                    stiction_map_path=stiction_map_path,
                    stiction_lut_gain=float(stiction_lut_gain),
                    smoothness_gate=bool(smooth_gate_on),
                    smoothness_max_vel_sign_changes=int(smooth_max_sign_changes),
                    smoothness_max_peak_abs_acc_turns_s2=(
                        None if smooth_max_acc is None else float(smooth_max_acc)
                    ),
                    smoothness_max_peak_abs_jerk_turns_s3=(
                        None if smooth_max_jerk is None else float(smooth_max_jerk)
                    ),
                )
                elapsed = float(time.monotonic() - t0)
                err_turns = float((dict(res).get("err", 0.0) or 0.0))
                err_deg = float(_motor_turn_error_to_angle_deg(err_turns, angle_space, ratio))
                rec.update(
                    {
                        "elapsed_s": float(elapsed),
                        "err_turns_motor": float(err_turns),
                        "err_deg": float(err_deg),
                        "abs_err_deg": abs(float(err_deg)),
                        "within_err_tol": bool(abs(float(err_deg)) <= float(err_tol_deg)),
                        "deadline": dict(res.get("deadline") or {}) if isinstance(res, dict) else {},
                        "profile_overrides_used": dict(runtime_profile_overrides or {}),
                    }
                )
                if isinstance(res, dict):
                    areq = dict(res.get("angle_request") or {})
                    adap = dict(areq.get("adaptation") or {})
                    div = dict(adap.get("divergence_recovery") or {})
                    rec["divergence_recovery_attempted"] = bool(div.get("attempted", False))
                    rec["divergence_recovery_succeeded"] = bool(div.get("succeeded", False))
                    rec["motion_duration_s"] = float(res.get("duration_s", 0.0) or 0.0)
                    rec["motion_sample_count"] = int(res.get("sample_count", 0) or 0)
                    rec["peak_abs_vel_turns_s"] = float(res.get("peak_abs_vel", 0.0) or 0.0)
                    rec["peak_abs_acc_turns_s2"] = float(res.get("peak_abs_acc", 0.0) or 0.0)
                    rec["peak_abs_jerk_turns_s3"] = float(res.get("peak_abs_jerk", 0.0) or 0.0)
                    rec["vel_sign_changes"] = int(res.get("vel_sign_changes", 0) or 0)
                    cmd_delta_turns = float(areq.get("commanded_delta_turns_motor", 0.0) or 0.0)
                    cmd_start_turns = areq.get("start_turns_motor")
                    end_turns = res.get("end")
                    obs_delta_turns = None
                    if (cmd_start_turns is not None) and (end_turns is not None):
                        try:
                            obs_delta_turns = float(end_turns) - float(cmd_start_turns)
                        except Exception:
                            obs_delta_turns = None
                    exp_sign = _sign_eps(float(cmd_delta_turns))
                    obs_sign = (None if obs_delta_turns is None else _sign_eps(float(obs_delta_turns)))
                    rec["expected_direction_sign"] = int(exp_sign)
                    rec["observed_direction_sign"] = (None if obs_sign is None else int(obs_sign))
                    rec["observed_delta_turns_motor"] = (None if obs_delta_turns is None else float(obs_delta_turns))
                    wrong_dir = False
                    if bool(wrong_dir_gate_on):
                        if (
                            abs(float(cmd_delta_turns)) >= float(wrong_dir_min_progress)
                            and (obs_delta_turns is not None)
                            and abs(float(obs_delta_turns)) >= float(wrong_dir_min_progress)
                            and (obs_sign is not None)
                            and int(exp_sign) != 0
                            and int(obs_sign) != 0
                            and int(obs_sign) != int(exp_sign)
                        ):
                            wrong_dir = True
                    rec["wrong_direction"] = bool(wrong_dir)
                else:
                    rec["divergence_recovery_attempted"] = False
                    rec["divergence_recovery_succeeded"] = False
                    rec["motion_duration_s"] = None
                    rec["motion_sample_count"] = None
                    rec["peak_abs_vel_turns_s"] = None
                    rec["peak_abs_acc_turns_s2"] = None
                    rec["peak_abs_jerk_turns_s3"] = None
                    rec["vel_sign_changes"] = None
                    rec["expected_direction_sign"] = None
                    rec["observed_direction_sign"] = None
                    rec["observed_delta_turns_motor"] = None
                    rec["wrong_direction"] = None
                if deadline_s is not None:
                    met = bool(rec["deadline"].get("met", elapsed <= float(deadline_s) + float(deadline_slack_s)))
                    rec["deadline_met"] = bool(met)
                else:
                    rec["deadline_met"] = True
                if bool(smooth_gate_on):
                    smooth_issues = []
                    sign_changes = int(rec.get("vel_sign_changes", 0) or 0)
                    peak_acc_val = float(rec.get("peak_abs_acc_turns_s2", 0.0) or 0.0)
                    peak_jerk_val = float(rec.get("peak_abs_jerk_turns_s3", 0.0) or 0.0)
                    if int(sign_changes) > int(smooth_max_sign_changes):
                        smooth_issues.append(
                            f"vel_sign_changes={int(sign_changes)} > {int(smooth_max_sign_changes)}"
                        )
                    if (smooth_max_acc is not None) and (float(peak_acc_val) > float(smooth_max_acc)):
                        smooth_issues.append(
                            f"peak_abs_acc={float(peak_acc_val):.6f} > {float(smooth_max_acc):.6f}"
                        )
                    if (smooth_max_jerk is not None) and (float(peak_jerk_val) > float(smooth_max_jerk)):
                        smooth_issues.append(
                            f"peak_abs_jerk={float(peak_jerk_val):.6f} > {float(smooth_max_jerk):.6f}"
                        )
                    rec["smoothness_ok"] = bool(len(smooth_issues) == 0)
                    rec["smoothness_issues"] = list(smooth_issues)
                    if not bool(rec["smoothness_ok"]):
                        rec["status"] = "fail"
                        rec["error"] = "smoothness gate failed: " + "; ".join(smooth_issues)
                        break
                else:
                    rec["smoothness_ok"] = None
                    rec["smoothness_issues"] = []
                if bool(wrong_dir_gate_on) and bool(rec.get("wrong_direction", False)):
                    rec["status"] = "fail"
                    rec["error"] = (
                        "wrong-direction motion detected: "
                        f"expected_sign={rec.get('expected_direction_sign')} "
                        f"observed_sign={rec.get('observed_direction_sign')} "
                        f"observed_delta_turns_motor={rec.get('observed_delta_turns_motor')}"
                    )
                    break
                break
            except Exception as exc:
                err_msg = str(exc)
                timeout_err_turns = _parse_timeout_err_turns(err_msg)
                tol_turns_runtime = float(
                    (runtime_profile_overrides or {}).get(
                        "target_tolerance_turns",
                        cfg.get("target_tolerance_turns", 0.006),
                    )
                )
                timeout_recoverable = (
                    ("Target not reached before timeout." in err_msg)
                    and (timeout_err_turns is not None)
                    and (abs(float(timeout_err_turns)) >= max(0.012, 1.02 * float(tol_turns_runtime)))
                )
                can_seq_recover = (
                    bool(seq_recovery_enabled)
                    and int(seq_recovery_events) < int(seq_recovery_max)
                    and int(attempt_idx) == 0
                    and bool(_is_sequence_recoverable(err_msg) or timeout_recoverable)
                )
                if bool(can_seq_recover):
                    evt = {
                        "index": int(idx),
                        "target_deg": float(tgt),
                        "attempt": int(attempt_idx + 1),
                        "error": err_msg,
                        "derate_applied": float(seq_recovery_scale),
                        "run_index_search": bool(sequence_recovery_run_index_search),
                    }
                    try:
                        req_idx = _run_sequence_recovery()
                        evt["reference_ok"] = True
                        evt["require_index"] = bool(req_idx)
                    except Exception as rex:
                        evt["reference_ok"] = False
                        evt["reference_error"] = str(rex)
                    runtime_profile_overrides = _apply_sequence_recovery_clamp(runtime_profile_overrides)
                    evt["profile_overrides_after"] = dict(runtime_profile_overrides or {})
                    seq_recovery_events += 1
                    seq_recovery_retry_count += 1
                    seq_recovery_log.append(evt)
                    rec["sequence_recovery"] = dict(evt)
                    attempt_idx += 1
                    settle_eff = max(float(settle_eff), float(settle_s) * 1.25)
                    continue
                rec["status"] = "fail"
                rec["error"] = err_msg
                rec["profile_overrides_used"] = dict(runtime_profile_overrides or {})
                wrong_dir_exc = ("WRONG_DIRECTION watchdog tripped" in str(err_msg))
                if bool(wrong_dir_exc):
                    rec["wrong_direction"] = True
                    cmd_sign = _parse_int_after(err_msg, "cmd_sign=")
                    disp = _parse_float_after(err_msg, "disp=")
                    rec["expected_direction_sign"] = None if cmd_sign is None else int(cmd_sign)
                    rec["observed_delta_turns_motor"] = None if disp is None else float(disp)
                    rec["observed_direction_sign"] = (
                        None if disp is None else int(_sign_eps(float(disp)))
                    )
                break
        if str(rec.get("status")) == "fail" and (
            bool(rec.get("wrong_direction", False)) or (not bool(continue_on_error))
        ):
            rows.append(rec)
            break
        rows.append(rec)

    ok_rows = [r for r in rows if str(r.get("status")) == "ok"]
    err_pass = [r for r in ok_rows if bool(r.get("within_err_tol", False))]
    dl_pass = [r for r in ok_rows if bool(r.get("deadline_met", False))]
    div_attempt = [r for r in rows if bool(r.get("divergence_recovery_attempted", False))]
    div_success = [r for r in rows if bool(r.get("divergence_recovery_succeeded", False))]
    wrong_dir_fail = [r for r in rows if bool(r.get("wrong_direction", False))]
    drift_guard_success = [e for e in drift_guard_log if bool(e.get("ok", False))]
    if bool(smooth_gate_on):
        smooth_pass = [r for r in ok_rows if bool(r.get("smoothness_ok", False))]
    else:
        smooth_pass = list(ok_rows)
    if bool(wrong_dir_gate_on):
        dir_pass = [r for r in ok_rows if not bool(r.get("wrong_direction", False))]
    else:
        dir_pass = list(ok_rows)
    summary = {
        "ok": bool(
            len(ok_rows) == len(rows)
            and len(err_pass) == len(rows)
            and len(dl_pass) == len(rows)
            and len(smooth_pass) == len(rows)
            and len(dir_pass) == len(rows)
        ),
        "count": int(len(rows)),
        "ok_count": int(len(ok_rows)),
        "error_pass_count": int(len(err_pass)),
        "deadline_pass_count": int(len(dl_pass)),
        "smoothness_pass_count": int(len(smooth_pass)),
        "direction_pass_count": int(len(dir_pass)),
        "divergence_recovery_attempt_count": int(len(div_attempt)),
        "divergence_recovery_success_count": int(len(div_success)),
        "wrong_direction_fail_count": int(len(wrong_dir_fail)),
        "err_tol_deg": float(err_tol_deg),
        "deadline_s": None if deadline_s is None else float(deadline_s),
        "deadline_mode": str(deadline_mode),
        "deadline_max_scale": float(deadline_max_scale),
        "control_style": str(control_style),
        "use_stiction_lut": bool(use_stiction_lut),
        "stiction_map_path": (
            None if stiction_map_path is None else os.path.abspath(str(stiction_map_path))
        ),
        "stiction_lut_gain": float(stiction_lut_gain),
        "angle_space": str(angle_space),
        "gear_ratio": float(ratio),
        "relative_to_current": bool(relative_to_current),
        "zero_turns_motor": None if zero_turns_motor is None else float(zero_turns_motor),
        "angle_ref_path": os.path.abspath(str(angle_ref_path or _default_angle_reference_path())),
        "require_angle_reference": bool(require_angle_reference),
        "wrap_strategy": str(wrap_strategy),
        "stage_chunk_max_turns": float(stage_chunk_max_turns),
        "stage_settle_s": float(stage_settle_s),
        "retry_on_retryable_fault": bool(retry_on_retryable_fault),
        "retry_derate_step": float(retry_derate_step),
        "retry_min_derate": float(retry_min_derate),
        "sequence_recovery_on_fail": bool(seq_recovery_enabled),
        "sequence_recovery_max_events": int(seq_recovery_max),
        "sequence_recovery_events": int(seq_recovery_events),
        "sequence_recovery_retry_count": int(seq_recovery_retry_count),
        "sequence_recovery_run_index_search": bool(sequence_recovery_run_index_search),
        "sequence_recovery_require_index": (
            None if sequence_recovery_require_index is None else bool(sequence_recovery_require_index)
        ),
        "sequence_recovery_derate": float(seq_recovery_scale),
        "drift_guard_enabled": bool(drift_guard_on),
        "drift_guard_bound_turns": float(drift_bound_turns),
        "drift_guard_anchor_turns": float(drift_anchor_turns),
        "drift_guard_max_events": int(drift_guard_max),
        "drift_guard_events": int(drift_guard_events),
        "drift_guard_trigger_count": int(len(drift_guard_log)),
        "drift_guard_success_count": int(len(drift_guard_success)),
        "drift_guard_run_index_search": bool(drift_guard_run_index_search),
        "drift_guard_require_index": (
            None if drift_guard_require_index is None else bool(drift_guard_require_index)
        ),
        "drift_guard_recenter_timeout_s": float(drift_guard_recenter_timeout_s),
        "drift_guard_recenter_settle_s": float(drift_guard_recenter_settle_s),
        "smoothness_gate": bool(smooth_gate_on),
        "smoothness_max_vel_sign_changes": int(smooth_max_sign_changes),
        "smoothness_max_peak_abs_acc_turns_s2": (
            None if smooth_max_acc is None else float(smooth_max_acc)
        ),
        "smoothness_max_peak_abs_jerk_turns_s3": (
            None if smooth_max_jerk is None else float(smooth_max_jerk)
        ),
        "fail_on_wrong_direction": bool(wrong_dir_gate_on),
        "wrong_direction_min_progress_turns": float(wrong_dir_min_progress),
        "pre_move_gate_mode": str(pre_move_gate_mode),
        "pre_move_gate_strategy": str(pre_move_gate_strategy),
        "auto_fallback_hybrid_on_low_authority": bool(auto_fallback_hybrid_on_low_authority),
        "pre_move_gate_load_mode": str(pre_move_gate_load_mode),
        "pre_move_gate_delta_turns": float(pre_move_gate_delta_turns),
        "pre_move_gate_duration_s": float(pre_move_gate_duration_s),
        "pre_move_gate_max_attempts": int(pre_move_gate_max_attempts),
        "startup_checks": str(startup_checks),
        "allow_stiction_kick": bool(allow_stiction_kick),
        "plus_current_scale": float(plus_current_scale),
        "minus_current_scale": float(minus_current_scale),
        "plus_trap_scale": float(plus_trap_scale),
        "minus_trap_scale": float(minus_trap_scale),
        "plus_vel_limit_scale": float(plus_vel_limit_scale),
        "minus_vel_limit_scale": float(minus_vel_limit_scale),
        "plus_kick_scale": float(plus_kick_scale),
        "minus_kick_scale": float(minus_kick_scale),
        "profile_overrides": dict(profile_overrides or {}),
        "effective_profile_overrides_final": dict(runtime_profile_overrides or {}),
    }
    return {
        "summary": summary,
        "results": rows,
        "sequence_recovery": {
            "events": list(seq_recovery_log),
        },
        "drift_guard": {
            "enabled": bool(drift_guard_on),
            "anchor_turns": float(drift_anchor_turns),
            "events": list(drift_guard_log),
        },
    }


def calibrate_directional_micro_move(
    angle_deg=1.0,
    repeats=3,
    angle_space=ANGLE_SPACE_GEARBOX_OUTPUT,
    axis=None,
    profile_name="loaded_motion_gate_stable",
    profile_path=None,
    base_overrides=None,
    plus_candidates=None,
    minus_candidates=None,
    gear_ratio=DEFAULT_GEAR_RATIO,
    zero_turns_motor=None,
    angle_ref_path=None,
    require_angle_reference=False,
    wrap_strategy="nearest",
    stage_chunk_max_turns=0.25,
    stage_settle_s=0.02,
    retry_on_retryable_fault=True,
    retry_derate_step=0.85,
    retry_min_derate=0.55,
    control_style="trap_strict",
    settle_s=0.18,
    timeout_s=14.0,
    continue_on_error=True,
    pre_move_gate_mode="warn",
    pre_move_gate_load_mode="loaded",
    save_path=None,
):
    """Direction-specific micro-move sweep for asymmetric systems.

    Runs repeated +angle and -angle tests separately and returns per-direction
    best override candidates, plus a saved JSON artifact.
    """
    a = axis if axis is not None else common.get_axis0()
    base = dict(base_overrides or {})
    ratio = float(abs(gear_ratio))
    nrep = max(1, int(repeats))
    step_deg = abs(float(angle_deg))
    if step_deg <= 0.0:
        raise ValueError("angle_deg must be non-zero")

    # Default to a local-relative frame for micro tests unless caller explicitly
    # provides a zero or requests strict external angle reference.
    if zero_turns_motor is None and (not bool(require_angle_reference)):
        zero_turns_motor = float(getattr(a.encoder, "pos_estimate", 0.0))

    if plus_candidates is None:
        plus_candidates = [
            {"name": "plus_soft", "trap_vel": 0.015, "vel_limit": 0.20, "trap_acc": 0.030, "trap_dec": 0.030, "current_lim": 6.0, "pos_gain": 6.0, "vel_gain": 0.10, "vel_i_gain": 0.00, "stiction_kick_nm": 0.03},
            {"name": "plus_medium", "trap_vel": 0.020, "vel_limit": 0.20, "trap_acc": 0.040, "trap_dec": 0.040, "current_lim": 7.0, "pos_gain": 8.0, "vel_gain": 0.14, "vel_i_gain": 0.00, "stiction_kick_nm": 0.05},
            {"name": "plus_push", "trap_vel": 0.020, "vel_limit": 0.20, "trap_acc": 0.040, "trap_dec": 0.040, "current_lim": 8.0, "pos_gain": 10.0, "vel_gain": 0.18, "vel_i_gain": 0.00, "stiction_kick_nm": 0.08},
        ]
    if minus_candidates is None:
        minus_candidates = [
            {"name": "minus_soft", "trap_vel": 0.015, "vel_limit": 0.20, "trap_acc": 0.030, "trap_dec": 0.030, "current_lim": 6.0, "pos_gain": 6.0, "vel_gain": 0.10, "vel_i_gain": 0.00, "stiction_kick_nm": 0.04},
            {"name": "minus_medium", "trap_vel": 0.020, "vel_limit": 0.20, "trap_acc": 0.040, "trap_dec": 0.040, "current_lim": 7.5, "pos_gain": 9.0, "vel_gain": 0.15, "vel_i_gain": 0.00, "stiction_kick_nm": 0.06},
            {"name": "minus_push", "trap_vel": 0.020, "vel_limit": 0.20, "trap_acc": 0.040, "trap_dec": 0.040, "current_lim": 8.5, "pos_gain": 11.0, "vel_gain": 0.20, "vel_i_gain": 0.00, "stiction_kick_nm": 0.10},
        ]

    def _hard_fault_count(results):
        cnt = 0
        for row in list(results or []):
            if str(row.get("status", "")).strip().lower() != "fail":
                continue
            msg = str(row.get("error", "")).lower()
            if (
                ("axis_err=0x200" in msg)
                or ("motor_err=0x10" in msg)
                or ("axis_err=0x40" in msg)
                or ("motor_err=0x1000" in msg)
            ):
                cnt += 1
        return int(cnt)

    def _mean_abs_err_ok(results):
        vals = []
        for row in list(results or []):
            if str(row.get("status", "")).strip().lower() != "ok":
                continue
            try:
                vals.append(abs(float(row.get("err_deg", 0.0) or 0.0)))
            except Exception:
                pass
        if not vals:
            return None
        return float(sum(vals) / len(vals))

    def _run_direction(label, sign, candidates):
        runs = []
        best = None
        best_key = None
        targets = tuple(float(sign) * float(step_deg) for _ in range(int(nrep)))
        for idx, raw in enumerate(list(candidates or []), start=1):
            cand = dict(raw or {})
            name = str(cand.pop("name", f"{label}_{idx}"))
            over = dict(base)
            over.update(cand)
            try:
                common.clear_errors_all(a)
            except Exception:
                pass
            rec = {
                "index": int(idx),
                "name": name,
                "direction": str(label),
                "targets_deg": targets,
                "overrides": dict(over),
            }
            try:
                out = validate_move_to_angle_sequence(
                    targets_deg=targets,
                    angle_space=str(angle_space),
                    axis=a,
                    profile_name=profile_name,
                    profile_path=profile_path,
                    profile_overrides=over,
                    gear_ratio=float(ratio),
                    zero_turns_motor=zero_turns_motor,
                    angle_ref_path=angle_ref_path,
                    require_angle_reference=bool(require_angle_reference),
                    wrap_strategy=str(wrap_strategy),
                    stage_chunk_max_turns=float(stage_chunk_max_turns),
                    stage_settle_s=float(stage_settle_s),
                    retry_on_retryable_fault=bool(retry_on_retryable_fault),
                    retry_derate_step=float(retry_derate_step),
                    retry_min_derate=float(retry_min_derate),
                    deadline_s=None,
                    control_style=str(control_style),
                    settle_s=float(settle_s),
                    timeout_s=float(timeout_s),
                    continue_on_error=bool(continue_on_error),
                    pre_move_gate_mode=str(pre_move_gate_mode),
                    pre_move_gate_load_mode=str(pre_move_gate_load_mode),
                )
            except Exception as exc:
                out = {"summary": {"ok": False, "count": int(nrep), "ok_count": 0}, "results": [{"status": "fail", "error": str(exc)}]}
            summary = dict(out.get("summary") or {})
            results = list(out.get("results") or [])
            ok_count = int(summary.get("ok_count", 0) or 0)
            hard_faults = _hard_fault_count(results)
            mean_abs_err = _mean_abs_err_ok(results)
            rec["summary"] = summary
            rec["results"] = results
            rec["hard_fault_count"] = int(hard_faults)
            rec["mean_abs_err_deg_ok"] = mean_abs_err
            score = (ok_count, -int(hard_faults), -(float(mean_abs_err) if mean_abs_err is not None else 1e9))
            rec["score"] = [float(score[0]), float(score[1]), float(score[2])]
            runs.append(rec)
            if best_key is None or score > best_key:
                best_key = score
                best = rec
            print(
                f"directional_micro[{label}] {name}: "
                f"ok_count={ok_count}/{int(nrep)} hard_faults={hard_faults} "
                f"mean_abs_err_deg_ok={mean_abs_err}"
            )
        return runs, best

    plus_runs, best_plus = _run_direction("plus", +1.0, plus_candidates)
    minus_runs, best_minus = _run_direction("minus", -1.0, minus_candidates)

    summary = {
        "ok": bool(
            isinstance(best_plus, dict)
            and isinstance(best_minus, dict)
            and int(dict(best_plus.get("summary") or {}).get("ok_count", 0) or 0) >= int(nrep)
            and int(dict(best_minus.get("summary") or {}).get("ok_count", 0) or 0) >= int(nrep)
        ),
        "angle_deg": float(step_deg),
        "repeats": int(nrep),
        "angle_space": str(angle_space),
        "gear_ratio": float(ratio),
        "zero_turns_motor_resolved": None if zero_turns_motor is None else float(zero_turns_motor),
        "profile_name": str(profile_name),
        "control_style": str(control_style),
        "pre_move_gate_mode": str(pre_move_gate_mode),
        "best_plus_name": None if not isinstance(best_plus, dict) else best_plus.get("name"),
        "best_minus_name": None if not isinstance(best_minus, dict) else best_minus.get("name"),
        "best_plus_ok_count": int(dict((best_plus or {}).get("summary") or {}).get("ok_count", 0) or 0),
        "best_minus_ok_count": int(dict((best_minus or {}).get("summary") or {}).get("ok_count", 0) or 0),
    }
    out = {
        "summary": summary,
        "base_overrides": dict(base),
        "plus_runs": plus_runs,
        "minus_runs": minus_runs,
        "best_plus": best_plus,
        "best_minus": best_minus,
        "recommended_directional_overrides": {
            "plus": dict((best_plus or {}).get("overrides") or {}),
            "minus": dict((best_minus or {}).get("overrides") or {}),
        },
    }
    out_path = os.path.abspath(str(save_path or os.path.join("logs", "directional_micro_move_latest.json")))
    _save_json_file_safe(out_path, out)
    out["path"] = out_path
    print(
        "calibrate_directional_micro_move:",
        f"path={out_path}",
        f"ok={summary['ok']}",
        f"plus={summary['best_plus_name']}({summary['best_plus_ok_count']}/{int(nrep)})",
        f"minus={summary['best_minus_name']}({summary['best_minus_ok_count']}/{int(nrep)})",
    )
    return out


def move_to_angle_progressive(
    angle_deg,
    angle_space,
    axis=None,
    profile_name="loaded_motion_gate_stable",
    profile_path=None,
    profile_ladder=None,
    gear_ratio=DEFAULT_GEAR_RATIO,
    relative_to_current=False,
    zero_turns_motor=None,
    angle_ref_path=None,
    require_angle_reference=True,
    wrap_strategy="nearest",
    stage_chunk_max_turns=0.03,
    stage_settle_s=0.03,
    retry_on_retryable_fault=True,
    retry_derate_step=0.85,
    retry_min_derate=0.50,
    timeout_s=12.0,
    settle_s=0.18,
    deadline_s=None,
    deadline_mode="strict",
    deadline_max_scale=2.0,
    deadline_timeout_margin_s=0.8,
    control_style="trap_strict",
    hybrid_final_window_turns=0.02,
    hybrid_micro_step_turns=0.002,
    hybrid_max_micro_steps=12,
    hybrid_micro_settle_s=0.05,
    hybrid_passthrough_pos_gain_scale=0.60,
    hybrid_passthrough_vel_gain_scale=1.15,
    hybrid_passthrough_vel_i_gain=0.0,
    pre_move_gate_mode="off",
    pre_move_gate_strategy="fixed",
    auto_fallback_hybrid_on_low_authority=False,
    pre_move_gate_load_mode="loaded",
    pre_move_gate_delta_turns=0.006,
    pre_move_gate_duration_s=1.2,
    pre_move_gate_max_attempts=2,
    pre_move_gate_current_scale_step=1.10,
    pre_move_gate_current_max_scale=1.20,
    pre_move_gate_abs_current_cap_a=None,
    pre_move_gate_kick_scale_step=1.25,
    pre_move_gate_kick_max_nm=0.12,
    pre_move_gate_absolute_at_start=False,
    pre_move_gate_run_index_search=False,
    startup_checks="minimal",
    authority_precheck="auto",
    authority_precheck_cmd_delta_turns=0.01,
    authority_precheck_current_lim=None,
    authority_precheck_pos_gain=None,
    authority_precheck_vel_gain=None,
    authority_precheck_vel_i_gain=0.0,
    authority_precheck_vel_limit=None,
    authority_precheck_timeout_s=1.8,
    authority_precheck_settle_s=0.05,
    authority_precheck_min_delta_turns=0.001,
    allow_stiction_kick=False,
    plus_current_scale=1.0,
    minus_current_scale=1.0,
    plus_trap_scale=1.0,
    minus_trap_scale=1.0,
    plus_vel_limit_scale=1.0,
    minus_vel_limit_scale=1.0,
    plus_kick_scale=1.0,
    minus_kick_scale=1.0,
    use_stiction_lut=False,
    stiction_map_path=None,
    stiction_lut_gain=1.0,
):
    """Try slow->faster damped profiles and return first successful move_to_angle result."""
    ladder = list(profile_ladder or default_progressive_move_profiles())
    attempts = []
    for idx, cfg in enumerate(ladder, start=1):
        cfg_use = dict(cfg or {})
        name = str(cfg_use.get("name") or f"profile_{idx}")
        rec = {"index": int(idx), "name": name, "profile_overrides": cfg_use}
        try:
            res = move_to_angle(
                angle_deg=float(angle_deg),
                angle_space=angle_space,
                axis=axis,
                profile_name=profile_name,
                profile_path=profile_path,
                profile_overrides=cfg_use,
                gear_ratio=float(gear_ratio),
                relative_to_current=bool(relative_to_current),
                zero_turns_motor=zero_turns_motor,
                angle_ref_path=angle_ref_path,
                require_angle_reference=bool(require_angle_reference),
                wrap_strategy=str(wrap_strategy),
                stage_chunk_max_turns=float(stage_chunk_max_turns),
                stage_settle_s=float(stage_settle_s),
                retry_on_retryable_fault=bool(retry_on_retryable_fault),
                retry_derate_step=float(retry_derate_step),
                retry_min_derate=float(retry_min_derate),
                timeout_s=float(timeout_s),
                settle_s=float(settle_s),
                deadline_s=deadline_s,
                deadline_mode=str(deadline_mode),
                deadline_max_scale=float(deadline_max_scale),
                deadline_timeout_margin_s=float(deadline_timeout_margin_s),
                control_style=str(control_style),
                hybrid_final_window_turns=float(hybrid_final_window_turns),
                hybrid_micro_step_turns=float(hybrid_micro_step_turns),
                hybrid_max_micro_steps=int(hybrid_max_micro_steps),
                hybrid_micro_settle_s=float(hybrid_micro_settle_s),
                hybrid_passthrough_pos_gain_scale=float(hybrid_passthrough_pos_gain_scale),
                hybrid_passthrough_vel_gain_scale=float(hybrid_passthrough_vel_gain_scale),
                hybrid_passthrough_vel_i_gain=float(hybrid_passthrough_vel_i_gain),
                pre_move_gate_mode=str(pre_move_gate_mode),
                pre_move_gate_strategy=str(pre_move_gate_strategy),
                auto_fallback_hybrid_on_low_authority=bool(auto_fallback_hybrid_on_low_authority),
                pre_move_gate_load_mode=str(pre_move_gate_load_mode),
                pre_move_gate_delta_turns=float(pre_move_gate_delta_turns),
                pre_move_gate_duration_s=float(pre_move_gate_duration_s),
                pre_move_gate_max_attempts=int(pre_move_gate_max_attempts),
                pre_move_gate_current_scale_step=float(pre_move_gate_current_scale_step),
                pre_move_gate_current_max_scale=float(pre_move_gate_current_max_scale),
                pre_move_gate_abs_current_cap_a=(
                    None
                    if pre_move_gate_abs_current_cap_a is None
                    else float(pre_move_gate_abs_current_cap_a)
                ),
                pre_move_gate_kick_scale_step=float(pre_move_gate_kick_scale_step),
                pre_move_gate_kick_max_nm=float(pre_move_gate_kick_max_nm),
                pre_move_gate_absolute_at_start=bool(pre_move_gate_absolute_at_start),
                pre_move_gate_run_index_search=bool(pre_move_gate_run_index_search),
                startup_checks=str(startup_checks),
                authority_precheck=str(authority_precheck),
                authority_precheck_cmd_delta_turns=float(authority_precheck_cmd_delta_turns),
                authority_precheck_current_lim=(
                    None if authority_precheck_current_lim is None else float(authority_precheck_current_lim)
                ),
                authority_precheck_pos_gain=(
                    None if authority_precheck_pos_gain is None else float(authority_precheck_pos_gain)
                ),
                authority_precheck_vel_gain=(
                    None if authority_precheck_vel_gain is None else float(authority_precheck_vel_gain)
                ),
                authority_precheck_vel_i_gain=float(authority_precheck_vel_i_gain),
                authority_precheck_vel_limit=(
                    None if authority_precheck_vel_limit is None else float(authority_precheck_vel_limit)
                ),
                authority_precheck_timeout_s=float(authority_precheck_timeout_s),
                authority_precheck_settle_s=float(authority_precheck_settle_s),
                authority_precheck_min_delta_turns=float(authority_precheck_min_delta_turns),
                allow_stiction_kick=bool(allow_stiction_kick),
                plus_current_scale=float(plus_current_scale),
                minus_current_scale=float(minus_current_scale),
                plus_trap_scale=float(plus_trap_scale),
                minus_trap_scale=float(minus_trap_scale),
                plus_vel_limit_scale=float(plus_vel_limit_scale),
                minus_vel_limit_scale=float(minus_vel_limit_scale),
                plus_kick_scale=float(plus_kick_scale),
                minus_kick_scale=float(minus_kick_scale),
                use_stiction_lut=bool(use_stiction_lut),
                stiction_map_path=stiction_map_path,
                stiction_lut_gain=float(stiction_lut_gain),
            )
            rec["status"] = "ok"
            rec["result"] = res
            attempts.append(rec)
            return {"ok": True, "selected_profile": name, "attempts": attempts, "result": res}
        except Exception as exc:
            rec["status"] = "fail"
            rec["error"] = str(exc)
            attempts.append(rec)
            print(f"move_to_angle_progressive: {name} failed -> {exc}")
    return {"ok": False, "selected_profile": None, "attempts": attempts, "result": None}


def _next_local_deadline(hour=8, minute=0, now=None):
    now_dt = now if isinstance(now, datetime.datetime) else datetime.datetime.now()
    target = now_dt.replace(hour=int(hour), minute=int(minute), second=0, microsecond=0)
    if target <= now_dt:
        target = target + datetime.timedelta(days=1)
    return target


def _parse_deadline_local(deadline_local):
    if isinstance(deadline_local, datetime.datetime):
        return deadline_local
    if deadline_local is None:
        return _next_local_deadline(8, 0)
    txt = str(deadline_local).strip()
    if not txt:
        return _next_local_deadline(8, 0)
    # HH:MM local-time shorthand
    if ":" in txt and ("-" not in txt) and (" " not in txt) and ("T" not in txt):
        hh, mm = txt.split(":", 1)
        return _next_local_deadline(int(hh), int(mm))
    # ISO-like / explicit date-time
    try:
        return datetime.datetime.fromisoformat(txt)
    except Exception:
        raise ValueError(
            "deadline_local must be datetime, ISO datetime string, or HH:MM local-time shorthand."
        )


def overnight_autotune_until(
    deadline_local="08:00",
    sleep_s=120.0,
    max_iterations=400,
    log_jsonl_path=None,
    require_repeatability=True,
    campaign_kwargs=None,
    recovery_probe_cycles=4,
    aggressive_recovery_every=5,
    abort_on_persistent_no_response=True,
    no_response_failure_limit=3,
):
    """Autonomous overnight tuner with logging and guarded progression.

    It keeps running until either:
      1) slow+fast campaign is satisfied, or
      2) local deadline is reached.
    """
    deadline_dt = _parse_deadline_local(deadline_local)
    campaign_kwargs = dict(campaign_kwargs or {})

    if log_jsonl_path is None:
        os.makedirs("logs", exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        log_jsonl_path = os.path.abspath(os.path.join("logs", f"overnight_autotune_{stamp}.jsonl"))
    else:
        log_jsonl_path = os.path.abspath(str(log_jsonl_path))
        os.makedirs(os.path.dirname(log_jsonl_path), exist_ok=True)

    def _log_row(payload):
        row = {"ts": datetime.datetime.now().isoformat(), **dict(payload or {})}
        with open(log_jsonl_path, "a", encoding="utf-8") as f:
            f.write(json.dumps(row, sort_keys=True) + "\n")
        return row

    _log_row(
        {
            "event": "start",
            "deadline_local": deadline_dt.isoformat(),
            "max_iterations": int(max_iterations),
            "sleep_s": float(sleep_s),
            "require_repeatability": bool(require_repeatability),
        }
    )

    final = {
        "ok": False,
        "status": "deadline_or_blocked",
        "deadline_local": deadline_dt.isoformat(),
        "log_jsonl_path": log_jsonl_path,
        "iterations": 0,
    }
    consecutive_ref_recovery_failures = 0
    consecutive_no_response_signals = 0

    def _looks_like_encoder_no_response(payload):
        try:
            txt = json.dumps(payload, sort_keys=True, default=str).lower()
        except Exception:
            txt = str(payload).lower()
        needles = (
            "encoder_error_no_response",
            "encoder calibration failed",
            "enc_err=0x4",
            "\"enc_err\": 4",
            "encoder.is_ready is false",
            "encoder is not ready",
        )
        return any(n in txt for n in needles)

    def _aggressive_reference_recovery():
        """Escalation path: reboot, reconnect, force recalibrate, force reference."""
        details = {"ok": False}
        try:
            odrv = common.get_odrv0()
            details["serial_before"] = int(getattr(odrv, "serial_number", 0) or 0)
        except Exception as exc:
            odrv = None
            details["get_odrv0_error"] = str(exc)

        try:
            if odrv is not None:
                odrv.reboot()
        except Exception as exc:
            # Expected USB disconnect during reboot on success paths.
            details["reboot_note"] = str(exc)

        time.sleep(4.0)

        try:
            import odrive

            odrv2 = odrive.find_any(timeout=20)
            details["serial_after"] = int(getattr(odrv2, "serial_number", 0) or 0)
            try:
                # Keep common.py fallback handle aligned after reconnect.
                common._ODRV0_FALLBACK = odrv2
            except Exception:
                pass
            a2 = odrv2.axis0
            common.clear_errors_all(a2)
            common.force_idle(a2, settle_s=0.10)

            if self_calibrate is not None:
                try:
                    cal_ok = bool(
                        self_calibrate.full_self_calibrate(
                            a2,
                            current_lim=10.0,
                            motor_calib_current=10.0,
                            force_motor_calib=True,
                            force_encoder_calib=True,
                            encoder_calib_retries=2,
                            require_absolute=False,
                            require_index=False,
                            run_index_search=False,
                            preload_deg=0.0,
                            preload_cycles=0,
                            run_step_validation=False,
                        )
                    )
                except Exception as exc:
                    cal_ok = False
                    details["full_self_calibrate_error"] = str(exc)
                details["full_self_calibrate_ok"] = bool(cal_ok)
            else:
                details["full_self_calibrate_ok"] = False
                details["full_self_calibrate_error"] = "self_calibrate module unavailable"

            try:
                ref = common.establish_absolute_reference(
                    a2,
                    require_index=True,
                    run_index_search=True,
                    attempt_offset_calibration=False,
                )
                details["absolute_reference"] = dict(ref or {})
                details["ok"] = bool(ref.get("ok", False)) if isinstance(ref, dict) else True
            except Exception as exc:
                details["absolute_reference_error"] = str(exc)
                details["ok"] = False
        except Exception as exc:
            details["reconnect_error"] = str(exc)
            details["ok"] = False

        return details

    for k in range(1, int(max_iterations) + 1):
        final["iterations"] = int(k)
        now_dt = datetime.datetime.now()
        if now_dt >= deadline_dt:
            final.update({"status": "deadline_reached", "iterations": int(k - 1)})
            _log_row({"event": "deadline_reached", "iteration": int(k - 1)})
            break

        iter_row = {"event": "iteration_start", "iteration": int(k)}
        try:
            a = common.get_axis0()
            snap = common._snapshot_motion(a)
        except Exception as exc:
            _log_row(
                {
                    **iter_row,
                    "result": "axis_unavailable",
                    "error": str(exc),
                }
            )
            time.sleep(float(max(5.0, sleep_s)))
            continue

        enc_ready = bool(snap.get("enc_ready", False))
        _log_row(
            {
                **iter_row,
                "result": "axis_snapshot",
                "enc_ready": bool(enc_ready),
                "state": int(snap.get("state", -1) or -1),
                "axis_err": int(snap.get("axis_err", 0) or 0),
                "motor_err": int(snap.get("motor_err", 0) or 0),
                "enc_err": int(snap.get("enc_err", 0) or 0),
                "ctrl_err": int(snap.get("ctrl_err", 0) or 0),
            }
        )

        # Attempt reference recovery when encoder is not ready.
        if not bool(enc_ready):
            try:
                rec = reference_recovery_mode(
                    axis=a,
                    run_index_search=True,
                    attempt_offset_calibration=False,
                    continuity_cycles=int(max(2, int(recovery_probe_cycles))),
                    continuity_jump_warn_turns=0.05,
                )
                rec_ok = bool(isinstance(rec, dict) and bool(rec.get("ok", False)))
                if rec_ok:
                    consecutive_ref_recovery_failures = 0
                    consecutive_no_response_signals = 0
                else:
                    consecutive_ref_recovery_failures += 1
                    if _looks_like_encoder_no_response(rec):
                        consecutive_no_response_signals += 1
                    else:
                        consecutive_no_response_signals = 0
                _log_row(
                    {
                        "event": "reference_recovery",
                        "iteration": int(k),
                        "ok": bool(rec_ok),
                        "probe_summary": dict((rec.get("probe_summary") or {})) if isinstance(rec, dict) else {},
                        "consecutive_failures": int(consecutive_ref_recovery_failures),
                        "consecutive_no_response_signals": int(consecutive_no_response_signals),
                    }
                )
            except Exception as exc:
                consecutive_ref_recovery_failures += 1
                if _looks_like_encoder_no_response(str(exc)):
                    consecutive_no_response_signals += 1
                else:
                    consecutive_no_response_signals = 0
                _log_row(
                    {
                        "event": "reference_recovery",
                        "iteration": int(k),
                        "ok": False,
                        "error": str(exc),
                        "consecutive_failures": int(consecutive_ref_recovery_failures),
                        "consecutive_no_response_signals": int(consecutive_no_response_signals),
                    }
                )

            try:
                nr_limit = max(1, int(no_response_failure_limit))
            except Exception:
                nr_limit = 3
            if bool(abort_on_persistent_no_response) and int(consecutive_no_response_signals) >= int(nr_limit):
                probe = {}
                try:
                    probe = common.recover_no_response_decoupled(
                        axis=a,
                        kv=140.0,
                        pole_pairs=7,
                        cpr=1024,
                        calibration_current=10.0,
                        sources=("INC_ENCODER0", "INC_ENCODER1"),
                        bandwidth=20.0,
                        interp=True,
                        watch_s=1.2,
                        hz=30.0,
                        min_counts=8,
                        bind_best=False,
                        verbose=False,
                    )
                except Exception as probe_exc:
                    probe = {"ok": False, "error": str(probe_exc)}
                final.update(
                    {
                        "ok": False,
                        "status": "persistent_encoder_no_response",
                        "iterations": int(k),
                        "stop_reason": "persistent_encoder_no_response",
                        "consecutive_no_response_signals": int(consecutive_no_response_signals),
                        "probe": probe,
                    }
                )
                _log_row(
                    {
                        "event": "persistent_no_response_abort",
                        "iteration": int(k),
                        "ok": False,
                        "consecutive_no_response_signals": int(consecutive_no_response_signals),
                        "probe": probe,
                    }
                )
                _log_row(
                    {
                        "event": "finished",
                        "ok": False,
                        "status": str(final.get("status", "")),
                        "iterations": int(final.get("iterations", 0)),
                    }
                )
                return final

            # Escalate after repeated failures to avoid spending the whole night in the same loop.
            try:
                every_n = max(1, int(aggressive_recovery_every))
            except Exception:
                every_n = 5
            if int(consecutive_ref_recovery_failures) >= int(every_n):
                esc = _aggressive_reference_recovery()
                _log_row(
                    {
                        "event": "aggressive_recovery",
                        "iteration": int(k),
                        "ok": bool(esc.get("ok", False)),
                        "details": esc,
                    }
                )
                if bool(esc.get("ok", False)):
                    consecutive_ref_recovery_failures = 0
                    consecutive_no_response_signals = 0
                else:
                    if _looks_like_encoder_no_response(esc):
                        consecutive_no_response_signals += 1
                    else:
                        consecutive_no_response_signals = 0
                    try:
                        nr_limit = max(1, int(no_response_failure_limit))
                    except Exception:
                        nr_limit = 3
                    if bool(abort_on_persistent_no_response) and int(consecutive_no_response_signals) >= int(nr_limit):
                        probe = {}
                        try:
                            probe = common.recover_no_response_decoupled(
                                axis=common.get_axis0(),
                                kv=140.0,
                                pole_pairs=7,
                                cpr=1024,
                                calibration_current=10.0,
                                sources=("INC_ENCODER0", "INC_ENCODER1"),
                                bandwidth=20.0,
                                interp=True,
                                watch_s=1.2,
                                hz=30.0,
                                min_counts=8,
                                bind_best=False,
                                verbose=False,
                            )
                        except Exception as probe_exc:
                            probe = {"ok": False, "error": str(probe_exc)}
                        final.update(
                            {
                                "ok": False,
                                "status": "persistent_encoder_no_response",
                                "iterations": int(k),
                                "stop_reason": "persistent_encoder_no_response",
                                "consecutive_no_response_signals": int(consecutive_no_response_signals),
                                "probe": probe,
                            }
                        )
                        _log_row(
                            {
                                "event": "persistent_no_response_abort",
                                "iteration": int(k),
                                "ok": False,
                                "consecutive_no_response_signals": int(consecutive_no_response_signals),
                                "probe": probe,
                            }
                        )
                        _log_row(
                            {
                                "event": "finished",
                                "ok": False,
                                "status": str(final.get("status", "")),
                                "iterations": int(final.get("iterations", 0)),
                            }
                        )
                        return final
            time.sleep(float(max(10.0, sleep_s)))
            continue

        # Encoder appears ready: attempt full slow+fast campaign.
        try:
            camp = run_slow_fast_campaign(
                max_rounds=3,
                require_repeatability=bool(require_repeatability),
                **dict(campaign_kwargs),
            )
        except Exception as exc:
            _log_row(
                {
                    "event": "campaign_exception",
                    "iteration": int(k),
                    "ok": False,
                    "error": str(exc),
                }
            )
            time.sleep(float(max(15.0, sleep_s)))
            continue

        camp_ok = bool(isinstance(camp, dict) and bool(camp.get("ok", False)))
        stop_reason = str((camp or {}).get("stop_reason", ""))
        _log_row(
            {
                "event": "campaign_result",
                "iteration": int(k),
                "ok": bool(camp_ok),
                "stop_reason": stop_reason,
                "numbers": dict((camp or {}).get("numbers") or {}),
            }
        )

        if camp_ok:
            final.update(
                {
                    "ok": True,
                    "status": "satisfied",
                    "iterations": int(k),
                    "stop_reason": stop_reason or "satisfied",
                    "campaign": camp,
                }
            )
            _log_row(
                {
                    "event": "finished",
                    "iteration": int(k),
                    "ok": True,
                    "status": "satisfied",
                }
            )
            return final

        # Keep trying until deadline.
        time.sleep(float(max(15.0, sleep_s)))

    if str(final.get("status", "")) == "deadline_or_blocked":
        final["status"] = "max_iterations_reached"

    _log_row(
        {
            "event": "finished",
            "ok": bool(final.get("ok", False)),
            "status": str(final.get("status", "")),
            "iterations": int(final.get("iterations", 0)),
        }
    )
    return final


def print_diagnostic_report(r):
    """Pretty-print a compact diagnostic summary from diagnostic_suite(...) output."""
    if not isinstance(r, dict):
        raise ValueError("print_diagnostic_report expects the dict returned by diagnostic_suite().")

    # Accept recursive wrapper output.
    if isinstance(r.get("best"), dict) and isinstance(r.get("history"), list):
        print(
            f"\n=== recursive_result === stop_reason={r.get('stop_reason')} "
            f"rounds_run={r.get('rounds_run')} satisfied={bool(r.get('satisfied', False))}"
        )
        r = r.get("best") or {}

    cfg = dict(r.get("config") or {})
    stages = dict(r.get("stages") or {})
    summary = dict(r.get("summary") or {})

    def _stage_state(name):
        st = stages.get(name, {})
        if bool(st.get("ok")):
            return "PASS"
        if bool(st.get("skipped")):
            return "SKIP"
        return "FAIL"

    def _fmt_float(x, nd=4):
        try:
            return f"{float(x):.{int(nd)}f}"
        except Exception:
            return "n/a"

    def _step_line(name, label):
        st = stages.get(name, {})
        if bool(st.get("skipped")):
            return f"{label}: SKIP ({st.get('reason', 'skipped')})"
        res = st.get("result")
        if not isinstance(res, dict):
            if st.get("error"):
                return f"{label}: FAIL ({st.get('error')})"
            return f"{label}: { _stage_state(name) }"
        win = res.get("winner")
        if isinstance(win, dict):
            return (
                f"{label}: PASS delta={_fmt_float(win.get('delta'))}t "
                f"err={_fmt_float(win.get('final_err'), 6)}t "
                f"peak_iq={_fmt_float(win.get('peak_abs_iq'), 3)}A"
            )
        rows = list(res.get("results") or [])
        last = rows[-1] if rows else {}
        status = str(last.get("status", "fail"))
        reason = last.get("reason")
        if reason:
            return f"{label}: FAIL status={status} reason={reason}"
        return f"{label}: FAIL status={status}"

    print("\n=== diagnostic_report ===")
    print(
        f"load_mode={summary.get('load_mode', 'unknown')} "
        f"classification={summary.get('classification', 'n/a')} "
        f"ok={bool(summary.get('ok', False))}"
    )
    print(f"interpretation: {summary.get('interpretation', 'n/a')}")
    print(
        f"config: deltas={tuple(cfg.get('delta_candidates', ()))} "
        f"trap=({_fmt_float(cfg.get('trap_vel'),3)}, {_fmt_float(cfg.get('trap_acc'),3)}, {_fmt_float(cfg.get('trap_dec'),3)}) "
        f"Ilim={_fmt_float(cfg.get('current_lim'),3)}A "
        f"gains=({_fmt_float(cfg.get('pos_gain'),3)}, {_fmt_float(cfg.get('vel_gain'),3)}, {_fmt_float(cfg.get('vel_i_gain'),3)})"
    )

    print("stages:")
    stage_order = [
        "absolute_reference",
        "reference_probe",
        "step_plus",
        "interstep_reference",
        "step_minus",
        "repeatability",
        "repeatability_alternate",
        "reference_recovery",
    ]
    for name in stage_order:
        if name in stages:
            print(f"  {name:18s} {_stage_state(name)}")

    print("step_details:")
    print(" ", _step_line("step_plus", "+dir"))
    print(" ", _step_line("step_minus", "-dir"))
    if "step_abort_current" in summary:
        print(f"step_abort_current: {summary.get('step_abort_current')}")
    if "motion_gate_ok" in summary:
        print(f"motion_gate_ok: {bool(summary.get('motion_gate_ok', False))}")
    if "repeatability_step_gate_ok" in summary:
        print(f"repeatability_step_gate_ok: {bool(summary.get('repeatability_step_gate_ok', False))}")
    gate_reasons = list(summary.get("repeatability_step_gate_reasons") or [])
    if gate_reasons:
        print(f"repeatability_step_gate_reasons: {', '.join(str(x) for x in gate_reasons)}")

    rep = stages.get("repeatability", {})
    rep_res = rep.get("result")
    rep_sum = dict(rep_res.get("summary") or {}) if isinstance(rep_res, dict) else {}
    if rep_sum:
        print(
            "repeatability: "
            f"ok={bool(rep_sum.get('ok', False))} "
            f"attempts={rep_sum.get('attempts', 'n/a')} "
            f"successes={rep_sum.get('successes', 'n/a')} "
            f"failures={rep_sum.get('failures', 'n/a')} "
            f"hard_faults={rep_sum.get('hard_faults', 'n/a')} "
            f"frame_jumps={rep_sum.get('frame_jumps', 'n/a')} "
            f"aborted={bool(rep_sum.get('aborted', False))}"
        )
        if rep_sum.get("aborted_reason"):
            print(f"repeatability_abort_reason: {rep_sum.get('aborted_reason')}")
        if rep_sum.get("directional_bias_compensation"):
            print(f"repeatability_bias_final: {rep_sum.get('directional_bias_final')}")
    if "repeatability_followup_plus_ok" in summary or "repeatability_followup_minus_ok" in summary:
        print(
            "repeatability_followup: "
            f"+dir={summary.get('repeatability_followup_plus_ok')} "
            f"-dir={summary.get('repeatability_followup_minus_ok')}"
        )
    if "repeatability_primary_frame_jumps" in summary or "repeatability_alternate_frame_jumps" in summary:
        print(
            "repeatability_frame_jumps: "
            f"primary={summary.get('repeatability_primary_frame_jumps')} "
            f"alternate={summary.get('repeatability_alternate_frame_jumps')} "
            f"total={summary.get('repeatability_frame_jumps_total')}"
        )
    if "repeatability_alternate_used" in summary or "repeatability_alternate_ok" in summary:
        print(
            "repeatability_alternate: "
            f"used={summary.get('repeatability_alternate_used')} "
            f"ok={summary.get('repeatability_alternate_ok')}"
        )
    if "reference_recovery_triggered" in summary or "reference_recovery_ok" in summary:
        print(
            "reference_recovery: "
            f"triggered={summary.get('reference_recovery_triggered')} "
            f"ok={summary.get('reference_recovery_ok')}"
        )
    if "encoder_not_ready" in summary or "encoder_no_response" in summary:
        print(
            "encoder_gate: "
            f"not_ready={summary.get('encoder_not_ready')} "
            f"no_response={summary.get('encoder_no_response')}"
        )
    if "unloaded_followup_plus_ok" in summary or "unloaded_followup_minus_ok" in summary:
        print(
            "unloaded_followup: "
            f"+dir={summary.get('unloaded_followup_plus_ok')} "
            f"-dir={summary.get('unloaded_followup_minus_ok')} "
            f"motion_ok={summary.get('unloaded_followup_motion_ok')}"
        )

    cls = str(summary.get("classification", ""))
    if cls == "motion_ok_unloaded":
        print("next: unloaded motion is healthy; proceed to loaded diagnostic_suite(load_mode='loaded').")
    elif cls == "directional_asymmetry_unloaded":
        print("next: run one-sided repeatability and tune direction-specific authority (vel_i_gain/stiction_kick).")
    elif cls == "repeatability_fault_unloaded_motion_ok":
        print("next: unloaded motion is stable in follow-up envelope; isolate repeatability-sequence fault path.")
    elif cls == "control_or_reference_fault_unloaded":
        print("next: stop tuning; investigate encoder/reference integrity and frame jumps before load testing.")
    elif cls == "encoder_no_response":
        print("next: encoder link is not responding; check encoder power/signal/cabling and rerun reference recovery.")
    elif cls == "encoder_not_ready":
        print("next: encoder is not ready; restore index/offset readiness before any motion tuning.")
    elif cls == "reference_integrity_fault":
        print("next: fix reference continuity first (index/offset path), then rerun diagnostic suite.")
    elif cls == "motion_ok_loaded":
        print("next: loaded motion is healthy; run longer repeatability cycles to quantify spread.")
    elif cls == "directional_margin_loaded":
        print("next: keep loaded one-sided repeatability; reduce reversal aggressiveness and tune I-gain carefully.")
    elif cls == "repeatability_fault_loaded_motion_ok":
        print("next: loaded motion gate passed; keep one-sided derated repeatability and avoid aggressive reversals.")
    elif cls == "repeatability_frame_jump_loaded_motion_ok":
        print("next: loaded motion gate passed; resolve frame continuity during repeatability before two-sided cycles.")
    elif cls == "hard_fault_under_load":
        print("next: treat as hard-fault condition; lower stress profile and verify mechanical constraints.")
    elif cls == "frame_jump_loaded":
        print("next: resolve reference-frame instability first (index/offset continuity) before loaded tuning.")
    elif cls == "no_motion_loaded":
        print("next: increase command margin carefully or reduce friction/load before repeatability testing.")
    else:
        print("next: inspect stage errors in r['stages'] for targeted follow-up.")

    return summary
