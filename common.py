# common.py
from IPython import get_ipython
import math
import time

from odrive.enums import *

# Some firmwares don't provide these enums in the same place.
# We'll try to import them, but the helpers work even if not available.
try:
    from odrive.enums import (
        AXIS_STATE_IDLE,
        AXIS_STATE_FULL_CALIBRATION_SEQUENCE,
        AXIS_STATE_ENCODER_INDEX_SEARCH,
        AXIS_STATE_ENCODER_OFFSET_CALIBRATION,
        AXIS_STATE_CLOSED_LOOP_CONTROL,
        CONTROL_MODE_POSITION_CONTROL,
        CONTROL_MODE_TORQUE_CONTROL,
        INPUT_MODE_PASSTHROUGH,
        INPUT_MODE_TRAP_TRAJ,
    )
except Exception:
    AXIS_STATE_IDLE = 1
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3
    AXIS_STATE_ENCODER_INDEX_SEARCH = 6
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7
    AXIS_STATE_CLOSED_LOOP_CONTROL = 8
    CONTROL_MODE_POSITION_CONTROL = 3
    CONTROL_MODE_TORQUE_CONTROL = 1
    INPUT_MODE_PASSTHROUGH = 1
    INPUT_MODE_TRAP_TRAJ = 5

# --- common.py build/version markers (helps verify reloads) ---
ENCODER_WATCH_CMD_VERSION = "2026-02-25d"
_ODRV0_FALLBACK = None

def _is_anonymous_interface(obj) -> bool:
    """Best-effort detection of a stale/anonymous ODrive handle (common after USB reconnect)."""
    try:
        return obj.__class__.__name__ == "anonymous_interface"
    except Exception:
        return False


def get_live_odrv0(strict: bool = False):
    """Return odrv0 from the live IPython namespace; optionally reject anonymous/stale handles."""
    odrv = get_odrv0()
    if strict and _is_anonymous_interface(odrv):
        raise RuntimeError(
            "odrv0 appears to be an anonymous/stale interface (likely after reconnect). "
            "Wait for odrivetool to reconnect and re-assign odrv0, then re-run."
        )
    return odrv


def get_live_axis0(strict: bool = False):
    """Return odrv0.axis0 from the live namespace (safer than reusing old axis refs)."""
    odrv = get_live_odrv0(strict=strict)
    try:
        return odrv.axis0
    except Exception as ex:
        raise RuntimeError(f"Could not access odrv0.axis0: {ex}")


def sync_pos_setpoint(axis, settle_s: float = 0.05, retries: int = 2, verbose: bool = False) -> bool:
    """Try to make controller.pos_setpoint match controller.input_pos at current pos_estimate.

    This fixes the common 'pos_setpoint=0' / 'controller hunts' issue after mode switches or reconnects.
    """
    tol = 1e-4
    ok = False

    for k in range(int(retries) + 1):
        try:
            p = float(axis.encoder.pos_estimate)
        except Exception:
            p = None

        try:
            axis.controller.input_pos = p
        except Exception:
            return False

        time.sleep(float(settle_s))

        try:
            rb_in = float(axis.controller.input_pos)
        except Exception:
            rb_in = None
        try:
            rb_sp = float(axis.controller.pos_setpoint)
        except Exception:
            rb_sp = None

        if rb_in is not None and rb_sp is not None:
            ok = (abs(rb_sp - rb_in) <= tol)

        if verbose:
            print(f"sync_pos_setpoint[{k}]: rb_in={rb_in} rb_sp={rb_sp} ok={ok}")

        if ok:
            break

        # If not OK: brief idle + closed-loop re-entry to reset internal controller state
        try:
            axis.requested_state = AXIS_STATE_IDLE
        except Exception:
            pass
        time.sleep(0.05)
        try:
            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        except Exception:
            pass
        time.sleep(0.10)

    return bool(ok)

def _whereami(fn):
    """Return best-effort (file, line) for a function for debugging reload issues."""
    try:
        import inspect
        f = inspect.getsourcefile(fn) or "<unknown>"
        line = inspect.getsourcelines(fn)[1]
        return f"{f}:{line}"
    except Exception:
        return "<unknown>"


def get_odrv0():
    """Fetch the current odrv0 from the live odrivetool/IPython namespace.

    When the board resets or USB reconnects, the old object can become an
    anonymous/stale interface. Always re-fetch before issuing commands.
    """
    global _ODRV0_FALLBACK
    ip = get_ipython()
    odrv = ip.user_ns.get("odrv0") if ip else None
    if odrv is None:
        # Terminal/script fallback: try to find a live ODrive directly.
        if _ODRV0_FALLBACK is None:
            try:
                import odrive  # local import to avoid overhead if IPython path is available
                _ODRV0_FALLBACK = odrive.find_any(timeout=3.0)
            except Exception:
                _ODRV0_FALLBACK = None
        odrv = _ODRV0_FALLBACK
    if odrv is None:
        raise RuntimeError(
            "odrv0 not found. Wait for 'Connected ... as odrv0' in odrivetool "
            "or attach from terminal via odrive.find_any()."
        )
    return odrv


def get_axis0():
    # Backwards-compatible name; always re-fetch from live namespace to avoid stale refs.
    return get_live_axis0(strict=False)


def _feed_watchdog(axis):
    """Best-effort watchdog feed for firmwares that require periodic servicing."""
    try:
        if hasattr(axis, "watchdog_feed"):
            axis.watchdog_feed()
    except Exception:
        pass


def st(axis):
    # Extra diagnostics (best-effort; may be None on some firmwares)
    print("axis_state", axis.current_state)
    print("is_armed", getattr(axis.motor, "is_armed", None))
    print("bus_voltage", getattr(getattr(axis, "_parent", None), "vbus_voltage", None))
    print("vel_est", axis.encoder.vel_estimate)
    print("vel_setpoint", getattr(axis.controller, "vel_setpoint", None))
    print("vel_integrator", getattr(axis.controller, "vel_integrator_torque", None))
    print("control_mode", axis.controller.config.control_mode)
    print("input_mode", axis.controller.config.input_mode)
    print("vel_limit", axis.controller.config.vel_limit)
    print("trap vel/acc/dec", axis.trap_traj.config.vel_limit, axis.trap_traj.config.accel_limit, axis.trap_traj.config.decel_limit)
    print("filter_bw", axis.controller.config.input_filter_bandwidth)
    print("input_pos", axis.controller.input_pos)
    print("pos_setpoint", axis.controller.pos_setpoint)
    print("pos_est", axis.encoder.pos_estimate)
    return {
        "state": axis.current_state,
        "axis_err": axis.error,
        "motor_err": axis.motor.error,
        "enc_err": axis.encoder.error,
        "ctrl_err": axis.controller.error,
        "pos": float(axis.encoder.pos_estimate),
        "vel": float(axis.encoder.vel_estimate),
        "Iq_set": float(axis.motor.current_control.Iq_setpoint),
        "Iq_meas": float(axis.motor.current_control.Iq_measured),
    }

def refresh_checks(axis=None, verbose: bool = True, force_live: bool = True):
    """Refresh/sanity-check an axis handle and resync controller setpoints.

    Call this after reconnects, after changing input_mode/control_mode,
    or whenever pos_setpoint mysteriously goes to 0.

    IMPORTANT:
    - `controller.pos_setpoint` only updates meaningfully in CLOSED_LOOP_CONTROL.
    - If the encoder is not ready / has no response, CLOSED_LOOP may be rejected and
      pos_setpoint can remain 0. In that case we return `synced=False` and include a snapshot.
    """
    # Prefer live axis by default (stale handles are common after reconnect).
    if axis is None or force_live or _is_anonymous_interface(axis) or _is_anonymous_interface(getattr(axis, "_parent", None)):
        axis = get_axis0()

    # If basic reads fail, fall back to live axis once.
    try:
        _ = float(axis.encoder.pos_estimate)
        _ = float(axis.controller.input_pos)
    except Exception:
        axis = get_axis0()

    # If encoder readiness is exposed and it's not ready, don't pretend we can sync.
    # (Running offset calibration here can be dangerous with a gearbox, so we only detect.)
    try:
        enc_ready = bool(getattr(axis.encoder, "is_ready", True))
    except Exception:
        enc_ready = True

    # Save previous modes so we can restore them.
    prev_cm = None
    prev_im = None
    try:
        prev_cm = int(axis.controller.config.control_mode)
    except Exception:
        prev_cm = None
    try:
        prev_im = int(axis.controller.config.input_mode)
    except Exception:
        prev_im = None

    # Best-effort enter closed-loop (required for pos_setpoint to track).
    closed_ok = False
    if enc_ready:
        closed_ok = ensure_closed_loop(axis, timeout_s=1.5)

    # Temporarily force deterministic position passthrough for syncing.
    if closed_ok:
        try:
            axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        except Exception:
            pass
        try:
            axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        except Exception:
            pass

    # Attempt to sync (only meaningful in closed loop)
    ok = False
    if closed_ok:
        ok = sync_pos_setpoint(axis, settle_s=0.06, retries=3, verbose=False)

        # If it still didn’t sync, do one “reset loop” and try once more
        if not ok:
            try:
                axis.requested_state = AXIS_STATE_IDLE
            except Exception:
                pass
            time.sleep(0.10)
            try:
                axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            except Exception:
                pass
            time.sleep(0.15)
            ok = sync_pos_setpoint(axis, settle_s=0.06, retries=2, verbose=False)

    # Restore modes
    if prev_cm is not None:
        try:
            axis.controller.config.control_mode = int(prev_cm)
        except Exception:
            pass
    if prev_im is not None:
        try:
            axis.controller.config.input_mode = int(prev_im)
        except Exception:
            pass

    # Readback
    pe = ip = ps = None
    try:
        pe = float(axis.encoder.pos_estimate)
    except Exception:
        pass
    try:
        ip = float(axis.controller.input_pos)
    except Exception:
        pass
    try:
        ps = float(axis.controller.pos_setpoint)
    except Exception:
        pass

    if verbose:
        print("pos_est     :", pe)
        print("input_pos   :", ip)
        print("pos_setpoint:", ps)

        if not enc_ready:
            print("WARN: encoder.is_ready is False (or encoder not ready). Closed-loop may be rejected.")
        if enc_ready and not closed_ok:
            print("WARN: could not enter CLOSED_LOOP_CONTROL; cannot sync pos_setpoint.")
        if closed_ok and not ok:
            print("WARN: pos_setpoint did not sync to input_pos.")

    out = {"pos_est": pe, "input_pos": ip, "pos_setpoint": ps, "synced": bool(ok and closed_ok)}

    # Attach a snapshot when we fail — this is usually the only actionable clue.
    if not out["synced"]:
        try:
            out["snapshot"] = _snapshot_motion(axis)
        except Exception:
            pass

    return out


def enforce_position_start_sync(
    axis=None,
    *,
    force_live: bool = True,
    settle_s: float = 0.05,
    retries: int = 2,
    tol_turns: float = 0.01,
):
    """Force controller position memory to match the live encoder before a move.

    This closes the gap between "encoder says we are here" and
    "controller.input_pos / controller.pos_setpoint still remember somewhere else".
    That stale-state mismatch is exactly what causes first-command catch-up jumps after
    reconnects, manual repositioning, or dirty closed-loop transitions.
    """
    chk = refresh_checks(axis=axis, verbose=False, force_live=bool(force_live))
    pe = chk.get("pos_est")
    ip = chk.get("input_pos")
    ps = chk.get("pos_setpoint")
    tol = max(1e-4, float(tol_turns))

    synced = bool(chk.get("synced"))
    if synced and (pe is not None) and (ip is not None) and (ps is not None):
        synced = (abs(float(ip) - float(pe)) <= tol) and (abs(float(ps) - float(pe)) <= tol)

    out = {
        "synced": bool(synced),
        "pos_est": pe,
        "input_pos": ip,
        "pos_setpoint": ps,
        "delta_input_turns": (None if (pe is None or ip is None) else float(ip) - float(pe)),
        "delta_setpoint_turns": (None if (pe is None or ps is None) else float(ps) - float(pe)),
    }
    if out["synced"]:
        return out

    # One stronger recovery path: explicitly overwrite input_pos with live pos_est and
    # require pos_setpoint to latch to the same place.
    a = get_axis0() if bool(force_live) else axis
    if a is None:
        a = axis
    try:
        pe_now = float(getattr(a.encoder, "pos_estimate", 0.0))
    except Exception:
        pe_now = None
    if pe_now is not None:
        try:
            a.controller.input_pos = float(pe_now)
        except Exception:
            pass
        try:
            sync_pos_setpoint(a, settle_s=float(settle_s), retries=int(retries), verbose=False)
        except Exception:
            pass

    try:
        pe = float(getattr(a.encoder, "pos_estimate"))
    except Exception:
        pe = None
    try:
        ip = float(getattr(a.controller, "input_pos"))
    except Exception:
        ip = None
    try:
        ps = float(getattr(a.controller, "pos_setpoint"))
    except Exception:
        ps = None

    synced = (
        (pe is not None)
        and (ip is not None)
        and (ps is not None)
        and (abs(float(ip) - float(pe)) <= tol)
        and (abs(float(ps) - float(pe)) <= tol)
    )
    out = {
        "synced": bool(synced),
        "pos_est": pe,
        "input_pos": ip,
        "pos_setpoint": ps,
        "delta_input_turns": (None if (pe is None or ip is None) else float(ip) - float(pe)),
        "delta_setpoint_turns": (None if (pe is None or ps is None) else float(ps) - float(pe)),
    }
    if not out["synced"]:
        try:
            out["snapshot"] = _snapshot_motion(a)
        except Exception:
            pass
    return out

def config():
    axis = get_axis0()

    axis.motor.config.current_lim = 5   # start low
    axis.controller.config.vel_limit = 1.0

    axis.trap_traj.config.vel_limit = 0.5
    axis.trap_traj.config.accel_limit = 1.0
    axis.trap_traj.config.decel_limit = 1.0

    axis.controller.config.pos_gain = 24.0
    axis.controller.config.vel_gain = 0.21
    axis.controller.config.vel_integrator_gain = 0.105
    axis.controller.config.input_filter_bandwidth = 20.0
    
def trap_traj():
    axis = get_axis0()
    axis.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
    axis.controller.input_pos = axis.encoder.pos_estimate
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

def clear(axis):
    """Clear errors on firmwares where axis.clear_errors exists."""
    if hasattr(axis, "clear_errors"):
        axis.clear_errors()


def _encoder_id_value(encoder_source="INC_ENCODER0"):
    """Resolve encoder source string/int to EncoderId value."""
    if encoder_source is None:
        return None
    if isinstance(encoder_source, int):
        return int(encoder_source)

    s = str(encoder_source).strip().upper()
    if not s:
        return None
    if s.startswith("ENCODERID."):
        s = s.split(".", 1)[1]

    aliases = {
        "INC0": "INC_ENCODER0",
        "INC1": "INC_ENCODER1",
        "INC2": "INC_ENCODER2",
        "0": "INC_ENCODER0",
        "1": "INC_ENCODER1",
        "2": "INC_ENCODER2",
    }
    s = aliases.get(s, s)
    try:
        return int(getattr(EncoderId, s))
    except Exception:
        return None


def _encoder_id_name(encoder_id):
    """Best-effort reverse map of encoder id to enum name."""
    try:
        eid = int(encoder_id)
    except Exception:
        return str(encoder_id)

    for name in ("INC_ENCODER0", "INC_ENCODER1", "INC_ENCODER2"):
        try:
            if int(getattr(EncoderId, name)) == eid:
                return name
        except Exception:
            pass
    return str(eid)


def _configure_inc_encoder_component(axis, encoder_id, cpr, bandwidth, use_index):
    """Configure selected inc_encoderX component if exposed by this firmware."""
    try:
        parent = getattr(axis, "_parent", None)
    except Exception:
        parent = None
    if parent is None:
        return

    enc_name = _encoder_id_name(encoder_id).lower()
    if not enc_name.startswith("inc_encoder"):
        return

    comp = getattr(parent, enc_name, None)
    if comp is None or (not hasattr(comp, "config")):
        return

    try:
        comp.config.enabled = True
    except Exception:
        pass
    try:
        comp.config.cpr = int(cpr)
    except Exception:
        pass
    try:
        comp.config.use_index = bool(use_index)
    except Exception:
        pass
    try:
        comp.config.bandwidth = float(bandwidth)
    except Exception:
        pass


def set_encoder(axis, cpr=1024, bandwidth=10, interp=False, use_index=False, encoder_source="INC_ENCODER0"):
    """Configure incremental A/B encoder settings.

    Note: Some firmwares expose `ENCODER_MODE_INCREMENTAL` enum; if not, mode=0
    is typically incremental. This helper avoids hard-depending on the enum.
    On ODrive 0.6+, it also binds the axis load/commutation encoder to the selected
    incremental source (default: INC_ENCODER0).
    """
    try:
        mode_inc = ENCODER_MODE_INCREMENTAL
    except Exception:
        mode_inc = 0

    axis.encoder.config.mode = int(mode_inc)
    axis.encoder.config.cpr = int(cpr)
    axis.encoder.config.bandwidth = float(bandwidth)
    axis.encoder.config.enable_phase_interpolation = bool(interp)
    axis.encoder.config.use_index = bool(use_index)

    enc_id = _encoder_id_value(encoder_source)
    if enc_id is None:
        enc_id = _encoder_id_value("INC_ENCODER0")
    if enc_id is None:
        enc_id = 1

    _configure_inc_encoder_component(axis, enc_id, cpr=cpr, bandwidth=bandwidth, use_index=use_index)

    try:
        axis.config.load_encoder = int(enc_id)
    except Exception:
        pass
    try:
        axis.config.commutation_encoder = int(enc_id)
    except Exception:
        pass


def probe_inc_encoder_sources(
    axis=None,
    sources=("INC_ENCODER0", "INC_ENCODER1"),
    cpr=1024,
    bandwidth=20.0,
    interp=True,
    use_index=False,
    watch_s=2.0,
    hz=30.0,
    min_counts=1,
    bind_best=True,
    restore_if_no_winner=True,
    verbose=True,
):
    """Test which incremental encoder source is alive by watching axis.encoder counts.

    This runs in IDLE and expects manual shaft rotation during each source window.
    """
    axis = get_live_axis(axis if axis is not None else "odrv0.axis0", strict=False)

    try:
        prev_load = int(getattr(axis.config, "load_encoder"))
    except Exception:
        prev_load = None
    try:
        prev_comm = int(getattr(axis.config, "commutation_encoder"))
    except Exception:
        prev_comm = None

    try:
        watch_s_f = max(0.2, float(watch_s))
    except Exception:
        watch_s_f = 2.0
    try:
        dt = 1.0 / max(1.0, float(hz))
    except Exception:
        dt = 0.03
    try:
        min_counts_i = max(1, int(min_counts))
    except Exception:
        min_counts_i = 1

    clear_errors_all(axis)
    force_idle(axis, settle_s=0.10)

    if verbose:
        print("probe_inc_encoder_sources: rotate shaft by hand while each source is sampled.")

    results = {}

    for src in list(sources):
        src_id = _encoder_id_value(src)
        src_name = _encoder_id_name(src_id) if src_id is not None else str(src)

        if src_id is None:
            results[str(src_name)] = {"source_id": None, "alive": False, "error": "unknown_encoder_source"}
            if verbose:
                print(f"  {src}: unknown encoder source")
            continue

        set_encoder(
            axis,
            cpr=int(cpr),
            bandwidth=float(bandwidth),
            interp=bool(interp),
            use_index=bool(use_index),
            encoder_source=src_name,
        )
        clear_errors_all(axis)
        force_idle(axis, settle_s=0.05)

        c0 = int(getattr(axis.encoder, "shadow_count", 0))
        p0 = float(getattr(axis.encoder, "pos_estimate", 0.0))
        cmin = cmax = c0

        if verbose:
            print(f"  sampling {src_name} ({watch_s_f:.1f}s)...")

        t0 = time.time()
        while (time.time() - t0) < watch_s_f:
            c = int(getattr(axis.encoder, "shadow_count", c0))
            cmin = min(cmin, c)
            cmax = max(cmax, c)
            time.sleep(dt)

        c1 = int(getattr(axis.encoder, "shadow_count", c0))
        p1 = float(getattr(axis.encoder, "pos_estimate", p0))
        try:
            enc_err = int(getattr(axis.encoder, "error", 0))
        except Exception:
            enc_err = 0

        dc = int(c1 - c0)
        dp = float(p1 - p0)
        span = int(cmax - cmin)
        score = float(abs(span) + abs(dc) + abs(dp) * float(cpr))
        alive = bool((abs(dc) >= min_counts_i) or (abs(span) >= min_counts_i))

        res = {
            "source_id": int(src_id),
            "delta_counts": dc,
            "count_span": span,
            "delta_pos": dp,
            "encoder_error": int(enc_err),
            "alive": bool(alive),
            "score": float(score),
        }
        results[src_name] = res

        if verbose:
            print(
                f"    {src_name}: alive={alive} Δcounts={dc:+d} span={span:+d} "
                f"Δpos={dp:+.6f} enc_err={hex(int(enc_err))}"
            )

    alive_items = [(k, v) for k, v in results.items() if bool(v.get("alive", False))]
    best_source = None
    if alive_items:
        best_source = max(alive_items, key=lambda kv: float(kv[1].get("score", 0.0)))[0]

    valid_items = [v for v in results.values() if isinstance(v, dict) and ("error" not in v)]
    max_count_delta = 0
    max_count_span = 0
    max_pos_delta_counts_equiv = 0.0
    all_enc_err_zero = True
    for v in valid_items:
        try:
            max_count_delta = max(max_count_delta, abs(int(v.get("delta_counts", 0))))
        except Exception:
            pass
        try:
            max_count_span = max(max_count_span, abs(int(v.get("count_span", 0))))
        except Exception:
            pass
        try:
            max_pos_delta_counts_equiv = max(
                max_pos_delta_counts_equiv,
                abs(float(v.get("delta_pos", 0.0))) * float(cpr),
            )
        except Exception:
            pass
        try:
            if int(v.get("encoder_error", 0)) != 0:
                all_enc_err_zero = False
        except Exception:
            pass

    # Heuristic classification:
    # If every tested source is flat and clean, the most likely explanation is that the
    # rotor did not move during the sample windows (or moved too little to measure).
    no_observed_motion = (
        (max_count_delta < min_counts_i)
        and (max_count_span < min_counts_i)
        and (max_pos_delta_counts_equiv < max(0.5, 0.5 * float(min_counts_i)))
    )
    inconclusive_due_to_no_motion = bool(best_source is None and no_observed_motion and all_enc_err_zero)

    if best_source is not None:
        diagnostic_note = (
            f"Detected live incremental source `{best_source}`. "
            "Proceed with preflight and non-index offset calibration first."
        )
    elif inconclusive_due_to_no_motion:
        diagnostic_note = (
            "No source showed measurable counts, but all channels looked electrically clean "
            "(no encoder errors and near-zero deltas). This is inconclusive and usually means "
            "the rotor did not move enough during sampling. Decouple/reduce load, rotate shaft "
            "clearly during each window, then rerun probe_inc_encoder_sources."
        )
    else:
        diagnostic_note = (
            "No live incremental source detected. With confirmed rotor motion, this points to "
            "A/B wiring, pin mapping (ENC0 vs ENC1), encoder power/ground, signal levels, or "
            "encoder output mode configuration."
        )

    restored = False
    if best_source is not None and bool(bind_best):
        best_id = _encoder_id_value(best_source)
        if best_id is not None:
            try:
                axis.config.load_encoder = int(best_id)
            except Exception:
                pass
            try:
                axis.config.commutation_encoder = int(best_id)
            except Exception:
                pass
            clear_errors_all(axis)
            if verbose:
                print(f"probe_inc_encoder_sources: selected {best_source} for load+commutation encoder.")
    elif best_source is None and bool(restore_if_no_winner):
        if prev_load is not None:
            try:
                axis.config.load_encoder = int(prev_load)
            except Exception:
                pass
        if prev_comm is not None:
            try:
                axis.config.commutation_encoder = int(prev_comm)
            except Exception:
                pass
        restored = True
        if verbose:
            print("probe_inc_encoder_sources: no live source detected; restored previous encoder source mapping.")

    if verbose:
        print("probe_inc_encoder_sources: note:", diagnostic_note)

    return {
        "ok": bool(best_source is not None),
        "best_source": best_source,
        "bound_best": bool(best_source is not None and bind_best),
        "restored_previous": bool(restored),
        "inconclusive_due_to_no_motion": bool(inconclusive_due_to_no_motion),
        "diagnostic_note": diagnostic_note,
        "previous": {
            "load_encoder": prev_load,
            "commutation_encoder": prev_comm,
            "load_encoder_name": _encoder_id_name(prev_load) if prev_load is not None else None,
            "commutation_encoder_name": _encoder_id_name(prev_comm) if prev_comm is not None else None,
        },
        "observed_motion": {
            "max_abs_delta_counts": int(max_count_delta),
            "max_abs_count_span": int(max_count_span),
            "max_abs_delta_pos_counts_equiv": float(max_pos_delta_counts_equiv),
            "all_encoder_error_zero": bool(all_enc_err_zero),
        },
        "results": results,
    }


def recover_no_response_decoupled(
    axis=None,
    kv=140.0,
    pole_pairs=7,
    cpr=1024,
    calibration_current=10.0,
    sources=("INC_ENCODER0", "INC_ENCODER1"),
    bandwidth=20.0,
    interp=True,
    watch_s=4.0,
    hz=50.0,
    min_counts=20,
    bind_best=True,
    verbose=True,
):
    """Run the non-index setup + source probe phase of NO_RESPONSE recovery.

    This helper is intended for the decoupled/manual-rotation stage before
    `full_self_calibrate(...)`.
    """
    axis = get_live_axis(axis if axis is not None else "odrv0.axis0", strict=False)

    try:
        motor_type_high_current = MOTOR_TYPE_HIGH_CURRENT
    except Exception:
        try:
            motor_type_high_current = int(getattr(MotorType, "HIGH_CURRENT", 0))
        except Exception:
            motor_type_high_current = 0

    try:
        axis.motor.config.motor_type = int(motor_type_high_current)
    except Exception:
        pass
    try:
        axis.motor.config.pole_pairs = int(pole_pairs)
    except Exception:
        pass
    try:
        axis.motor.config.torque_constant = 8.27 / float(kv)
    except Exception:
        pass
    try:
        axis.motor.config.calibration_current = float(calibration_current)
    except Exception:
        pass

    src_list = list(sources)
    first_source = src_list[0] if src_list else "INC_ENCODER0"
    set_encoder(
        axis,
        cpr=int(cpr),
        bandwidth=float(bandwidth),
        interp=bool(interp),
        use_index=False,
        encoder_source=first_source,
    )

    try:
        axis.config.startup_encoder_index_search = False
    except Exception:
        pass
    try:
        axis.encoder.config.find_idx_on_lockin = False
    except Exception:
        pass

    clear_errors_all(axis)
    force_idle(axis, settle_s=0.10)

    probe = probe_inc_encoder_sources(
        axis=axis,
        sources=tuple(src_list),
        cpr=int(cpr),
        bandwidth=float(bandwidth),
        interp=bool(interp),
        use_index=False,
        watch_s=float(watch_s),
        hz=float(hz),
        min_counts=int(min_counts),
        bind_best=bool(bind_best),
        restore_if_no_winner=True,
        verbose=bool(verbose),
    )

    if probe.get("ok", False):
        next_steps = [
            "preflight_encoder(axis, cpr=..., bandwidth=..., interp=...)",
            "full_self_calibrate(axis, require_absolute=False, require_index=False, run_index_search=False)",
            "enable index settings, then full_self_calibrate(... require_absolute=True ...)",
        ]
    elif probe.get("inconclusive_due_to_no_motion", False):
        next_steps = [
            "decouple/reduce load and rotate shaft more during probe windows",
            "rerun recover_no_response_decoupled(...) or probe_inc_encoder_sources(...)",
        ]
    else:
        next_steps = [
            "verify A/B pin mapping (ENC0 vs ENC1), common GND, and encoder output mode",
            "verify both A and B transitions at ODrive pins",
            "rerun probe after wiring/power fixes",
        ]

    out = {
        "probe": probe,
        "config": {
            "kv": float(kv),
            "pole_pairs": int(pole_pairs),
            "cpr": int(cpr),
            "calibration_current": float(calibration_current),
            "use_index": False,
            "sources_tested": list(src_list),
        },
        "next_steps": next_steps,
    }
    return out


def tame_motion(axis, vel_limit=2.0, traj_vel=1.0, accel=1.0, decel=1.0):
    # Controller
    axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    axis.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
    axis.controller.config.vel_limit = float(vel_limit)

    # Trap traj limits
    axis.trap_traj.config.vel_limit = float(traj_vel)
    axis.trap_traj.config.accel_limit = float(accel)
    axis.trap_traj.config.decel_limit = float(decel)

    # Soft starter gains (reduce "goes nuts" tendency)
    axis.controller.config.pos_gain = 20.0
    axis.controller.config.vel_gain = 0.30
    axis.controller.config.vel_integrator_gain = 0.60
    axis.controller.config.input_filter_bandwidth = 3.0

    # Sync setpoint to current estimate to avoid an immediate jump on next command
    try:
        axis.controller.input_pos = axis.encoder.pos_estimate
    except Exception:
        pass

    print("control_mode", axis.controller.config.control_mode)
    print("input_mode", axis.controller.config.input_mode)
    print("vel_limit", axis.controller.config.vel_limit)
    print("trap vel/acc/dec", axis.trap_traj.config.vel_limit, axis.trap_traj.config.accel_limit, axis.trap_traj.config.decel_limit)
    print("filter_bw", axis.controller.config.input_filter_bandwidth)
    print("input_pos", axis.controller.input_pos)
    print("pos_setpoint", axis.controller.pos_setpoint)
    print("pos_est", axis.encoder.pos_estimate)



def wait_idle(axis, timeout_s=20):
    """Wait until axis returns to IDLE or CLOSED_LOOP; useful after calibration."""
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        if axis.current_state in (AXIS_STATE_IDLE, AXIS_STATE_CLOSED_LOOP_CONTROL):
            return True
        time.sleep(0.05)
    return False


# Wait for axis.current_state to reach a specific value, with timeout and polling.
def wait_state(axis, target_state, timeout_s=2.0, poll_s=0.02, feed_watchdog=True):
    """Wait for axis.current_state to reach target_state."""
    t0 = time.time()
    while time.time() - t0 < float(timeout_s):
        try:
            if int(getattr(axis, "current_state", 0)) == int(target_state):
                return True
        except Exception:
            pass
        if bool(feed_watchdog):
            _feed_watchdog(axis)
        time.sleep(float(poll_s))
    return False


def calibrate(axis):
    """Run full calibration sequence."""
    axis.requested_state = AXIS_STATE_IDLE
    clear_errors_all(axis)
    axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    wait_idle(axis, timeout_s=30)
    return axis.motor.is_calibrated, axis.encoder.is_ready, axis.error


def enter_closed_loop(axis):
    """Safe closed-loop entry: sync setpoint to measured position first."""
    clear_errors_all(axis)
    axis.controller.input_pos = axis.encoder.pos_estimate
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    return axis.current_state, axis.error


def move_deg(axis, deg):
    """Move by degrees using position mode. Degrees at motor shaft."""
    axis.controller.input_pos += (float(deg) / 360.0)


def move_to(axis, turns):
    """Absolute target in turns."""
    axis.controller.input_pos = float(turns)

# --- Strict motion helpers (raise if nothing actually moves) ---

def _safe_f(getter, default=None):
    try:
        return float(getter())
    except Exception:
        return default


def _snapshot_motion(axis):
    """Best-effort snapshot to debug why a move did/didn't happen."""
    # Some 0.6.x firmwares expose these as key diagnostics when axis.error remains zero.
    try:
        disarm_reason = int(getattr(axis, "disarm_reason", 0))
    except Exception:
        disarm_reason = None
    try:
        active_errors = int(getattr(axis, "active_errors", 0))
    except Exception:
        active_errors = None
    try:
        procedure_result = int(getattr(axis, "procedure_result", 0))
    except Exception:
        procedure_result = None
    try:
        enc_ready = bool(getattr(axis.encoder, "is_ready"))
    except Exception:
        enc_ready = None
    try:
        enc_use_index = bool(getattr(axis.encoder.config, "use_index"))
    except Exception:
        enc_use_index = None
    try:
        enc_index_found = getattr(axis.encoder, "index_found", None)
        if enc_index_found is not None:
            enc_index_found = bool(enc_index_found)
    except Exception:
        enc_index_found = None

    return {
        "state": int(getattr(axis, "current_state", 0)),
        "axis_err": int(getattr(axis, "error", 0)),
        "motor_err": int(getattr(axis.motor, "error", 0)),
        "enc_err": int(getattr(axis.encoder, "error", 0)),
        "ctrl_err": int(getattr(axis.controller, "error", 0)),
        "disarm_reason": disarm_reason,
        "active_errors": active_errors,
        "procedure_result": procedure_result,
        "enc_ready": enc_ready,
        "enc_use_index": enc_use_index,
        "enc_index_found": enc_index_found,
        "pos_est": _safe_f(lambda: axis.encoder.pos_estimate, None),
        "input_pos": _safe_f(lambda: axis.controller.input_pos, None),
        "pos_setpoint": _safe_f(lambda: axis.controller.pos_setpoint, None),
        "vel_est": _safe_f(lambda: axis.encoder.vel_estimate, None),
        "Iq_set": _safe_f(lambda: axis.motor.current_control.Iq_setpoint, None),
        "Iq_meas": _safe_f(lambda: axis.motor.current_control.Iq_measured, None),
        "ctrl_mode": int(getattr(axis.controller.config, "control_mode", -1)),
        "input_mode": int(getattr(axis.controller.config, "input_mode", -1)),
        "vel_limit": _safe_f(lambda: axis.controller.config.vel_limit, None),
        "trap_vel": _safe_f(lambda: axis.trap_traj.config.vel_limit, None),
        "trap_acc": _safe_f(lambda: axis.trap_traj.config.accel_limit, None),
        "trap_dec": _safe_f(lambda: axis.trap_traj.config.decel_limit, None),
        "pos_gain": _safe_f(lambda: axis.controller.config.pos_gain, None),
        "vel_gain": _safe_f(lambda: axis.controller.config.vel_gain, None),
        "vel_i_gain": _safe_f(lambda: axis.controller.config.vel_integrator_gain, None),
        "tc": _safe_f(lambda: axis.motor.config.torque_constant, None),
        "current_lim": _safe_f(lambda: axis.motor.config.current_lim, None),
        "shadow_count": int(getattr(axis.encoder, "shadow_count", 0)),
    }


def _decode_error_bits(val: int, prefix: str):
    names = []
    try:
        ival = int(val)
    except Exception:
        ival = 0
    if ival == 0:
        return names
    for k, v in globals().items():
        try:
            if k.startswith(prefix) and isinstance(v, int) and (ival & v):
                names.append(k)
        except Exception:
            pass
    names.sort()
    return names


def get_axis_error_report(axis=None):
    """Return a decoded axis/motor/encoder/controller error report plus snapshot."""
    a = get_axis0() if axis is None else axis
    snap = _snapshot_motion(a)
    ax_err = int(snap.get("axis_err") or 0)
    m_err = int(snap.get("motor_err") or 0)
    e_err = int(snap.get("enc_err") or 0)
    c_err = int(snap.get("ctrl_err") or 0)
    return {
        "snapshot": dict(snap or {}),
        "axis_err_names": _decode_error_bits(ax_err, "AXIS_ERROR_"),
        "motor_err_names": _decode_error_bits(m_err, "MOTOR_ERROR_"),
        "enc_err_names": _decode_error_bits(e_err, "ENCODER_ERROR_"),
        "ctrl_err_names": _decode_error_bits(c_err, "CONTROLLER_ERROR_"),
    }


def diagnose_axis_state(
    axis=None,
    *,
    mounted: bool = True,
    verbose: bool = True,
    kv_est=None,
    line_line_r_ohm=None,
):
    """Explain the current axis state and suggest the next recovery command(s).

    This is aimed at interactive odrivetool use when the raw booleans/bitfields are
    not enough to tell what to do next.
    """
    a = get_axis0() if axis is None else axis
    rep = get_axis_error_report(a)
    snap = dict(rep.get("snapshot") or {})
    ax_names = list(rep.get("axis_err_names") or [])
    m_names = list(rep.get("motor_err_names") or [])
    e_names = list(rep.get("enc_err_names") or [])
    c_names = list(rep.get("ctrl_err_names") or [])

    diagnosis = "Unknown/uncategorized state"
    severity = "info"
    commands = []
    notes = []
    verdicts = []

    state = int(snap.get("state") or 0)
    enc_ready = bool(snap.get("enc_ready")) if snap.get("enc_ready") is not None else False
    idx_found = bool(snap.get("enc_index_found")) if snap.get("enc_index_found") is not None else False
    use_index = bool(snap.get("enc_use_index")) if snap.get("enc_use_index") is not None else False
    motor_is_calibrated = bool(_safe_f(lambda: a.motor.is_calibrated, False))
    phase_r = _safe_f(lambda: float(a.motor.config.phase_resistance), None)
    phase_l = _safe_f(lambda: float(a.motor.config.phase_inductance), None)
    tc_cfg = _safe_f(lambda: float(a.motor.config.torque_constant), None)

    if motor_is_calibrated and (phase_r is not None) and (phase_r > 0.0) and (phase_l is not None) and (phase_l > 0.0):
        verdicts.append("motor model OK")
    else:
        verdicts.append("motor model incomplete")

    if enc_ready and ((not use_index) or idx_found) and (not ax_names) and (not e_names):
        verdicts.append("startup ready")
    else:
        verdicts.append("startup not ready")

    if kv_est is not None:
        try:
            kv_f = float(kv_est)
            kt_ref = (8.27 / kv_f) if kv_f > 0.0 else None
        except Exception:
            kt_ref = None
        if (kt_ref is not None) and (tc_cfg is not None) and (kt_ref > 0.0):
            ratio = float(tc_cfg) / float(kt_ref)
            if (ratio < 0.80) or (ratio > 1.20):
                verdicts.append("torque constant suspect")
            else:
                verdicts.append("torque constant plausible")

    if line_line_r_ohm is not None:
        try:
            ll_r = float(line_line_r_ohm)
            phase_r_ref = (ll_r / 2.0) if ll_r > 0.0 else None
        except Exception:
            phase_r_ref = None
        if (phase_r_ref is not None) and (phase_r is not None) and (phase_r_ref > 0.0):
            rr = float(phase_r) / float(phase_r_ref)
            if (rr < 0.60) or (rr > 1.40):
                verdicts.append("resistance estimate mismatch")

    if not ax_names and not m_names and not e_names and not c_names:
        if state == int(AXIS_STATE_CLOSED_LOOP_CONTROL) and enc_ready:
            diagnosis = "Axis is armed and ready for position moves."
            severity = "ok"
            commands = [
                "# already ready",
                "move_to_angle_continuous(...)",
            ]
        elif state == int(AXIS_STATE_IDLE) and enc_ready:
            diagnosis = "Axis is calibrated and encoder-ready, but currently idle."
            severity = "ok"
            commands = [
                "clear_errors_all(a)",
                "a.requested_state = 8  # CLOSED_LOOP_CONTROL",
            ]
        elif state == int(AXIS_STATE_IDLE) and (not enc_ready):
            diagnosis = "Axis is idle with no latched errors, but encoder is not ready."
            severity = "warn"
            commands = [
                "clear_errors_all(a)",
                "a.requested_state = 3  # FULL_CALIBRATION_SEQUENCE",
            ]
            if use_index and (not idx_found):
                notes.append("Index is required at runtime but has not been latched yet.")
        else:
            diagnosis = "Axis has no latched errors, but is not in a clearly ready motion state."
            severity = "warn"
            commands = [
                "clear_errors_all(a)",
                "a.requested_state = 3  # FULL_CALIBRATION_SEQUENCE",
            ]
    elif "AXIS_ERROR_ENCODER_FAILED" in ax_names:
        severity = "error"
        if "ENCODER_ERROR_CPR_POLEPAIRS_MISMATCH" in e_names:
            diagnosis = "Encoder electrical-angle calibration is invalid. Index may be found, but offset/commutation is not valid."
            commands = [
                "a.requested_state = 1  # IDLE",
                "clear_errors_all(a)",
                "a.requested_state = 3  # FULL_CALIBRATION_SEQUENCE",
            ]
            notes.append("If this repeats while mounted, the motor/encoder commutation path is not repeatable in the current mechanical state.")
            if use_index:
                notes.append("Diagnostic fallback only: try `a.encoder.config.use_index = False` before `a.requested_state = 3` to see if the no-index offset path is more repeatable.")
        elif "ENCODER_ERROR_INDEX_NOT_FOUND_YET" in e_names:
            diagnosis = "Encoder index is required, but the controller has not seen it yet."
            commands = [
                "a.requested_state = 1  # IDLE",
                "clear_errors_all(a)",
                "a.requested_state = 6  # ENCODER_INDEX_SEARCH",
                "a.requested_state = 7  # ENCODER_OFFSET_CALIBRATION",
            ]
        elif "ENCODER_ERROR_NO_RESPONSE" in e_names:
            diagnosis = "Encoder is not producing valid response/edges."
            commands = [
                "a.requested_state = 1  # IDLE",
                "clear_errors_all(a)",
                "# then inspect encoder power, wiring, magnet alignment, and CPR configuration",
            ]
        else:
            diagnosis = "Encoder failed, but the exact sub-cause is not classified above."
            commands = [
                "a.requested_state = 1  # IDLE",
                "clear_errors_all(a)",
                "a.requested_state = 3  # FULL_CALIBRATION_SEQUENCE",
            ]
    elif "MOTOR_ERROR_CURRENT_LIMIT_VIOLATION" in m_names:
        severity = "error"
        diagnosis = "Controller saturated current limit. The axis was either blocked, overloaded, or broke away aggressively."
        commands = [
            "a.requested_state = 1  # IDLE",
            "clear_errors_all(a)",
            "a.requested_state = 3  # FULL_CALIBRATION_SEQUENCE",
        ]
        notes.append("Do not hand-load or manually restrain the output while using the aggressive continuous profile.")
    elif "CONTROLLER_ERROR_OVERSPEED" in c_names:
        severity = "error"
        diagnosis = "Controller hit overspeed. Trajectory or settle behavior is too aggressive for the current plant state."
        commands = [
            "a.requested_state = 1  # IDLE",
            "clear_errors_all(a)",
            "a.requested_state = 3  # FULL_CALIBRATION_SEQUENCE",
        ]
        notes.append("Reduce trajectory aggressiveness or avoid reversal tests until startup is clean again.")
    else:
        severity = "error"
        diagnosis = "Axis has latched errors that need manual inspection before further moves."
        commands = [
            "a.requested_state = 1  # IDLE",
            "clear_errors_all(a)",
        ]

    if mounted:
        notes.append("Mounted-plant rule: avoid segmented intermediate settled waypoints; use one continuous move to the final target.")
    if state == int(AXIS_STATE_CLOSED_LOOP_CONTROL):
        notes.append("Do not manually reposition the output while armed; that invalidates the next aggressive move.")
    if use_index and idx_found and (not enc_ready):
        notes.append("Index is latched but encoder is still not ready. That usually means offset/commutation failed after index.")

    out = {
        "severity": severity,
        "diagnosis": diagnosis,
        "verdict": " | ".join(verdicts),
        "verdicts": list(verdicts),
        "commands": commands,
        "notes": notes,
        "report": rep,
    }

    if verbose:
        if verdicts:
            print("verdict:", " | ".join(verdicts))
        print(f"[{severity}] {diagnosis}")
        print(
            "state:",
            state,
            "enc_ready:",
            enc_ready,
            "index_found:",
            idx_found,
            "use_index:",
            use_index,
        )
        print("axis_err:", ax_names or ["none"])
        print("motor_err:", m_names or ["none"])
        print("enc_err:", e_names or ["none"])
        print("ctrl_err:", c_names or ["none"])
        if commands:
            print("next:")
            for cmd in commands:
                print(" ", cmd)
        if notes:
            print("notes:")
            for n in notes:
                print(" -", n)
    return out


def motor_fact_sheet(axis=None, *, kv_est=None, line_line_r_ohm=None, verbose: bool = True):
    """Print and return a live/configured/inferred fact sheet for the current axis.

    This is intentionally conservative about provenance:
    - measured live: direct board/runtime readback
    - configured: stored on-controller config
    - inferred: derived from user-provided assumptions/measurements
    - unknown: still not independently verified
    """
    a = get_axis0() if axis is None else axis
    odrv = getattr(a, "_parent", None)

    def _read(fn, default=None):
        try:
            return fn()
        except Exception:
            return default

    def _fmt(v, digits=6):
        if v is None:
            return "unknown"
        if isinstance(v, bool):
            return str(v)
        if isinstance(v, int):
            return str(v)
        if isinstance(v, float):
            if math.isnan(v) or math.isinf(v):
                return str(v)
            return f"{v:.{digits}g}"
        return str(v)

    def _build_row(label, kind, value, note=""):
        _emit_sample(
            "return",
            p1,
            v1,
            reached=bool(reached),
            quiet_hold_applied=bool((quiet_hold_meta or {}).get("applied")),
        )

        return {
            "label": str(label),
            "kind": str(kind),
            "value": value,
            "note": str(note),
        }

    measured = [
        _build_row("vbus_voltage_V", "measured live", _read(lambda: float(odrv.vbus_voltage) if odrv is not None else None)),
        _build_row("phase_resistance_ohm", "measured live", _read(lambda: float(a.motor.config.phase_resistance))),
        _build_row("phase_inductance_H", "measured live", _read(lambda: float(a.motor.config.phase_inductance))),
        _build_row("motor_is_calibrated", "measured live", _read(lambda: bool(a.motor.is_calibrated))),
        _build_row("encoder_is_ready", "measured live", _read(lambda: bool(a.encoder.is_ready))),
        _build_row("encoder_index_found", "measured live", _read(lambda: bool(getattr(a.encoder, "index_found", False)))),
        _build_row("pos_estimate_turns", "measured live", _read(lambda: float(a.encoder.pos_estimate))),
        _build_row("shadow_count", "measured live", _read(lambda: int(a.encoder.shadow_count))),
        _build_row("iq_setpoint_A", "measured live", _read(lambda: float(a.motor.current_control.Iq_setpoint))),
        _build_row("iq_measured_A", "measured live", _read(lambda: float(a.motor.current_control.Iq_measured))),
    ]

    configured = [
        _build_row("pole_pairs", "configured", _read(lambda: int(a.motor.config.pole_pairs))),
        _build_row("current_lim_A", "configured", _read(lambda: float(a.motor.config.current_lim))),
        _build_row("torque_constant_Nm_A", "configured", _read(lambda: float(a.motor.config.torque_constant))),
        _build_row("encoder_cpr", "configured", _read(lambda: int(a.encoder.config.cpr))),
        _build_row("encoder_use_index", "configured", _read(lambda: bool(a.encoder.config.use_index))),
        _build_row("motor_direction", "configured", _read(lambda: int(a.motor.config.direction))),
        _build_row("axis_state", "configured", _read(lambda: int(a.current_state))),
        _build_row("axis_error", "configured", _read(lambda: int(a.error))),
        _build_row("motor_error", "configured", _read(lambda: int(a.motor.error))),
        _build_row("encoder_error", "configured", _read(lambda: int(a.encoder.error))),
        _build_row("controller_error", "configured", _read(lambda: int(a.controller.error))),
    ]

    kt_from_kv = (8.27 / float(kv_est)) if (kv_est is not None and float(kv_est) > 0.0) else None
    phase_r_from_ll = (float(line_line_r_ohm) / 2.0) if (line_line_r_ohm is not None) else None

    inferred = [
        _build_row("kv_est_rpm_per_V", "inferred", (None if kv_est is None else float(kv_est)), "from motor spec / user estimate"),
        _build_row("kt_from_kv_Nm_A", "inferred", kt_from_kv, "8.27 / KV"),
        _build_row("line_line_R_ohm", "inferred", (None if line_line_r_ohm is None else float(line_line_r_ohm)), "manual DMM value if valid"),
        _build_row("phase_R_from_ll_ohm", "inferred", phase_r_from_ll, "assumes wye: phase ~= line-line/2"),
    ]

    unknown = [
        _build_row("true_torque_constant", "unknown", None, "needs exact datasheet, KV check, or torque bench test"),
        _build_row("continuous_safe_current", "unknown", None, "needs datasheet and/or thermal test"),
        _build_row("absolute_joint_angle", "unknown", None, "motor-side encoder does not prove output-side absolute angle"),
        _build_row("encoder_low_speed_quality", "unknown", None, "needs quiet settle validation, not just calibration"),
    ]

    r_live = measured[1]["value"]
    tc_live = configured[2]["value"]
    quick_checks = []
    if r_live is not None and phase_r_from_ll is not None:
        ratio = (float(r_live) / float(phase_r_from_ll)) if float(phase_r_from_ll) != 0.0 else None
        quick_checks.append(_build_row("R_live_vs_R_from_ll", "inferred", ratio, "near 1.0 is reassuring; not exact proof"))
    if tc_live is not None and kt_from_kv is not None:
        ratio = (float(tc_live) / float(kt_from_kv)) if float(kt_from_kv) != 0.0 else None
        quick_checks.append(_build_row("Kt_cfg_vs_Kt_from_KV", "inferred", ratio, "far from 1.0 means torque scaling is suspect"))

    out = {
        "measured_live": measured,
        "configured": configured,
        "inferred": inferred,
        "unknown": unknown,
        "quick_checks": quick_checks,
    }

    if verbose:
        print("\n=== Motor Fact Sheet ===\n")
        for section_name, rows in (
            ("measured live", measured),
            ("configured", configured),
            ("inferred", inferred),
        ):
            for row in rows:
                print(
                    f"{row['label']:<28} [{section_name:<12}] {_fmt(row['value']):<18} {row['note']}"
                )
        print("\n--- Still weak / unknown ---")
        for row in unknown:
            print(f"{row['label']:<28} [{row['kind']:<12}] {_fmt(row['value']):<18} {row['note']}")
        if quick_checks:
            print("\n--- Quick checks ---")
            for row in quick_checks:
                print(f"{row['label']:<28} [{row['kind']:<12}] {_fmt(row['value']):<18} {row['note']}")
        print("\n=== End Fact Sheet ===")

    return out


def ensure_closed_loop(axis, timeout_s=2.0, clear_first=True, pre_sync=True, retries=1, require_encoder_ready=True):
    if bool(clear_first):
        clear_errors_all(axis)

    # If the encoder exposes is_ready and it's false, don't spam state changes
    # unless explicitly requested by caller (firmware-compat fallback paths).
    if bool(require_encoder_ready):
        try:
            if hasattr(axis.encoder, "is_ready") and (not bool(axis.encoder.is_ready)):
                return False
        except Exception:
            pass

    tries = max(1, int(retries) + 1)
    for _ in range(tries):
        try:
            axis.controller.vel_integrator_torque = 0.0
        except Exception:
            pass

        try:
            axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        except Exception:
            pass
        try:
            axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        except Exception:
            pass

        if bool(pre_sync):
            try:
                axis.controller.input_pos = float(getattr(axis.encoder, "pos_estimate", 0.0))
            except Exception:
                pass

        _feed_watchdog(axis)
        try:
            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        except Exception:
            pass

        if wait_state(
            axis,
            AXIS_STATE_CLOSED_LOOP_CONTROL,
            timeout_s=float(timeout_s),
            poll_s=0.02,
            feed_watchdog=True,
        ):
            return True

        # One retry path for firmwares that transiently reject closed-loop with no axis.error.
        try:
            axis.requested_state = AXIS_STATE_IDLE
        except Exception:
            pass
        _feed_watchdog(axis)
        time.sleep(0.05)
        if bool(clear_first):
            clear_errors_all(axis, settle_s=0.05)

    return False


def move_startup_contract(
    axis,
    startup_mode="minimal",
    require_index=None,
    run_index_search_on_recover=None,
    require_encoder_ready=True,
    timeout_s=2.5,
    sync_settle_s=0.03,
    stability_observe_s=0.25,
    stability_dt=0.02,
):
    """Deterministic startup gate for motion commands without probe wiggles.

    This helper performs only reference/closed-loop/setpoint synchronization checks.
    It never commands exploratory motion.
    """
    mode = str(startup_mode).strip().lower()
    if mode not in ("minimal", "guarded"):
        raise ValueError("startup_mode must be 'minimal' or 'guarded'")

    out = {
        "ok": False,
        "mode": str(mode),
        "require_index": None,
        "require_encoder_ready": bool(require_encoder_ready),
        "use_index": None,
        "enc_ready_start": None,
        "index_found_start": None,
        "ran_reference_recover": False,
        "ran_index_search_recover": False,
        "closed_loop_ok": False,
        "sync_ok": False,
        "stability_probe": None,
        "error": None,
        "snapshot_start": None,
        "snapshot_end": None,
    }

    try:
        out["snapshot_start"] = _snapshot_motion(axis)
    except Exception:
        out["snapshot_start"] = None

    try:
        use_index = bool(getattr(axis.encoder.config, "use_index", False))
    except Exception:
        use_index = False
    out["use_index"] = bool(use_index)

    if require_index is None:
        require_index = bool(use_index)
    out["require_index"] = bool(require_index)

    try:
        out["enc_ready_start"] = bool(getattr(axis.encoder, "is_ready", False))
    except Exception:
        out["enc_ready_start"] = None
    try:
        idx0 = getattr(axis.encoder, "index_found", None)
        out["index_found_start"] = (None if idx0 is None else bool(idx0))
    except Exception:
        out["index_found_start"] = None

    if bool(require_index) and (not bool(use_index)):
        out["error"] = (
            "move_startup_contract: require_index=True but encoder.config.use_index=False."
        )
        return out

    try:
        clear_errors_all(axis, settle_s=0.05)
    except Exception:
        pass

    if bool(run_index_search_on_recover is None):
        run_index_search_on_recover = bool(mode == "guarded")

    need_ref_recover = False
    if bool(require_index):
        idx_found = out.get("index_found_start")
        if idx_found is None:
            idx_found = False
        enc_ready = out.get("enc_ready_start")
        if enc_ready is None:
            enc_ready = False
        need_ref_recover = bool((not bool(idx_found)) or (bool(require_encoder_ready) and (not bool(enc_ready))))

    if bool(need_ref_recover):
        out["ran_reference_recover"] = True
        out["ran_index_search_recover"] = bool(run_index_search_on_recover)
        try:
            establish_absolute_reference(
                axis,
                require_index=bool(require_index),
                run_index_search=bool(run_index_search_on_recover),
                attempt_offset_calibration=False,
                recover_offset_after_index=True,
                label="move_startup_contract",
            )
        except Exception as exc:
            out["error"] = f"move_startup_contract absolute-reference recover failed: {exc}"
            try:
                out["snapshot_end"] = _snapshot_motion(axis)
            except Exception:
                out["snapshot_end"] = None
            return out

    ok_cl = bool(
        ensure_closed_loop(
            axis,
            timeout_s=float(timeout_s),
            clear_first=False,
            pre_sync=True,
            retries=(2 if mode == "guarded" else 1),
            require_encoder_ready=bool(require_encoder_ready),
        )
    )
    out["closed_loop_ok"] = bool(ok_cl)
    if not bool(ok_cl):
        out["error"] = "move_startup_contract failed to enter CLOSED_LOOP_CONTROL."
        try:
            out["snapshot_end"] = _snapshot_motion(axis)
        except Exception:
            out["snapshot_end"] = None
        return out

    sync_ok = False
    try:
        sync_ok = bool(
            sync_pos_setpoint(
                axis,
                settle_s=max(0.0, float(sync_settle_s)),
                retries=(2 if mode == "guarded" else 1),
                verbose=False,
            )
        )
    except Exception:
        sync_ok = False
    out["sync_ok"] = bool(sync_ok)
    if (not bool(sync_ok)) and bool(mode == "guarded"):
        out["error"] = "move_startup_contract failed to synchronize setpoints."
        try:
            out["snapshot_end"] = _snapshot_motion(axis)
        except Exception:
            out["snapshot_end"] = None
        return out

    if bool(mode == "guarded"):
        vals = []
        t_end = time.time() + max(0.10, float(stability_observe_s))
        while time.time() < t_end:
            p = float(getattr(axis.encoder, "pos_estimate", 0.0))
            v = float(getattr(axis.encoder, "vel_estimate", 0.0))
            vals.append((p, v))
            time.sleep(max(0.005, float(stability_dt)))
        if vals:
            ps = [float(x[0]) for x in vals]
            vs = [abs(float(x[1])) for x in vals]
            span = max(ps) - min(ps)
            peak_vel = max(vs)
        else:
            span = 0.0
            peak_vel = 0.0
        drift_limit = 0.03
        vel_limit = 0.15
        stab_ok = bool((float(span) <= float(drift_limit)) and (float(peak_vel) <= float(vel_limit)))
        out["stability_probe"] = {
            "ok": bool(stab_ok),
            "span": float(span),
            "peak_vel": float(peak_vel),
            "drift_limit": float(drift_limit),
            "vel_limit": float(vel_limit),
            "mode": "guarded",
        }
        if not bool(stab_ok):
            out["error"] = f"move_startup_contract stability failed: {out['stability_probe']}"
            try:
                out["snapshot_end"] = _snapshot_motion(axis)
            except Exception:
                out["snapshot_end"] = None
            return out
    else:
        out["stability_probe"] = {
            "ok": True,
            "skipped": True,
            "reason": "startup_mode=minimal",
            "mode": "minimal",
        }

    out["ok"] = True
    try:
        out["snapshot_end"] = _snapshot_motion(axis)
    except Exception:
        out["snapshot_end"] = None
    return out


def move_to_pos_strict(
    axis,
    target_turns,
    use_trap_traj=True,
    timeout_s=3.0,
    min_delta_turns=0.002,   # ~0.7° at motor shaft
    settle_s=0.20,
    # “quiet but effective” defaults for harmonic drives:
    vel_limit=5.0,
    vel_limit_tolerance=None,
    enable_overspeed_error=None,
    trap_vel=2.0,
    trap_acc=5.0,
    trap_dec=5.0,
    input_filter_bw=0.0,
    # authority knobs:
    current_lim=None,
    pos_gain=20.0,
    vel_gain=0.30,
    vel_i_gain=0.60,
    # optional stiction break:
    stiction_kick_nm=None,
    stiction_kick_s=0.08,
    # Stall watchdog (prevents cooking the power stage if the load doesn't move)
    stall_watchdog=True,
    stall_iq_a=None,          # if None, auto = max(2.0, 0.6*current_lim) when current_lim is known
    stall_pos_eps_turns=3e-4, # ~0.11° at motor shaft
    stall_time_s=0.35,        # seconds of "high Iq + no motion" before abort
    fail_to_idle=True,
    require_target_reached=False,
    target_tolerance_turns=0.004,
    target_vel_tolerance_turns_s=0.10,
    abort_on_reverse_motion=True,
    reverse_motion_eps_turns=0.002,
    reverse_motion_confirm_samples=2,
    refresh_before_move=True,
    force_live_refresh=True,
    pre_command_quiet_s=0.15,
    pre_command_quiet_vel_turns_s=0.20,
    abort_on_inverted_iq=False,
    inverted_iq_min_a=0.35,
    inverted_iq_confirm_samples=3,
    inverted_iq_grace_s=0.03,
    quiet_hold_enable=True,
    quiet_hold_s=0.14,
    quiet_hold_pos_gain_scale=0.60,
    quiet_hold_vel_gain_scale=0.90,
    quiet_hold_vel_i_gain=0.0,
    quiet_hold_vel_limit_scale=0.70,
    quiet_hold_persist=False,
    quiet_hold_reanchor_err_turns=None,
    require_start_sync=True,
    start_sync_tol_turns=None,
    sample_hook=None,
):
    """Command an absolute position and RAISE if pos_est doesn't move.

    If `require_target_reached` is True, this also requires the axis to settle
    within `target_tolerance_turns` and `target_vel_tolerance_turns_s` before returning.
    """
    # Optional pre-refresh. In minimal production paths this can be skipped to avoid
    # extra state transitions before command dispatch.
    if bool(refresh_before_move):
        refresh_checks(axis, verbose=False, force_live=bool(force_live_refresh))
        # NOTE: refresh_checks may switch input_mode temporarily; move_to_pos_strict will re-assert modes below.
        if bool(force_live_refresh):
            axis = get_axis0()

    prev_ilim = None
    prev_vel_limit_tolerance = None
    prev_enable_overspeed_error = None
    if current_lim is not None:
        try:
            prev_ilim = float(axis.motor.config.current_lim)
            axis.motor.config.current_lim = float(current_lim)
        except Exception:
            prev_ilim = None

    if vel_limit_tolerance is not None:
        try:
            prev_vel_limit_tolerance = float(axis.controller.config.vel_limit_tolerance)
            axis.controller.config.vel_limit_tolerance = float(vel_limit_tolerance)
        except Exception:
            prev_vel_limit_tolerance = None

    if enable_overspeed_error is not None:
        try:
            prev_enable_overspeed_error = bool(axis.controller.config.enable_overspeed_error)
            axis.controller.config.enable_overspeed_error = bool(enable_overspeed_error)
        except Exception:
            prev_enable_overspeed_error = None

    try:
        # Configure controller for position moves
        axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        axis.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ if bool(use_trap_traj) else INPUT_MODE_PASSTHROUGH

        # Gains (key vs pos_gain=0.8 case)
        try:
            axis.controller.config.pos_gain = float(pos_gain)
            axis.controller.config.vel_gain = float(vel_gain)
            axis.controller.config.vel_integrator_gain = float(vel_i_gain)
        except Exception:
            pass

        # Limits / filters
        try:
            axis.controller.config.vel_limit = float(vel_limit)
        except Exception:
            pass
        try:
            axis.controller.config.input_filter_bandwidth = float(input_filter_bw)
        except Exception:
            pass

        if bool(use_trap_traj):
            try:
                axis.trap_traj.config.vel_limit = float(trap_vel)
                axis.trap_traj.config.accel_limit = float(trap_acc)
                axis.trap_traj.config.decel_limit = float(trap_dec)
            except Exception:
                pass

        if not ensure_closed_loop(axis, timeout_s=2.0):
            raise RuntimeError(f"Failed to enter CLOSED_LOOP_CONTROL. snapshot={_snapshot_motion(axis)}")

        # ensure_closed_loop() uses PASSTHROUGH as a safe default for entry.
        # Re-assert the requested motion mode for this command afterwards.
        try:
            axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        except Exception:
            pass
        try:
            axis.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ if bool(use_trap_traj) else INPUT_MODE_PASSTHROUGH
        except Exception:
            pass

        # Clear integrator memory before every commanded move.
        # This reduces startup recoil from stale velocity integrator state.
        try:
            axis.controller.vel_integrator_torque = 0.0
        except Exception:
            pass
        try:
            axis.controller.vel_integrator = 0.0
        except Exception:
            pass

        sync_tol = (
            max(0.0025, 2.0 * float(min_delta_turns))
            if start_sync_tol_turns is None
            else max(1e-4, float(start_sync_tol_turns))
        )
        start_contract = enforce_position_start_sync(
            axis,
            force_live=False,
            settle_s=0.05,
            retries=2,
            tol_turns=float(sync_tol),
        )
        if bool(require_start_sync) and (not bool(start_contract.get("synced"))):
            raise RuntimeError(
                "Pre-move controller position state is stale. "
                f"contract={start_contract}"
            )

        p0 = (
            float(start_contract.get("pos_est"))
            if start_contract.get("pos_est") is not None
            else _safe_f(lambda: axis.encoder.pos_estimate, 0.0)
        )

        # Optional pre-command quieting to avoid launching new moves while residual
        # velocity is still present from prior motion/recovery.
        quiet_t_end = time.time() + max(0.0, float(pre_command_quiet_s))
        quiet_v_lim = max(0.0, float(pre_command_quiet_vel_turns_s))
        while (time.time() < quiet_t_end) and (quiet_v_lim > 0.0):
            vq = _safe_f(lambda: axis.encoder.vel_estimate, 0.0)
            if abs(float(vq)) <= float(quiet_v_lim):
                break
            try:
                axis.controller.input_pos = float(p0)
            except Exception:
                pass
            time.sleep(0.01)

        # Optional: tiny torque nudge to break stiction before position move
        if stiction_kick_nm is not None and float(stiction_kick_nm) > 0.0:
            torque_bump(axis, torque=float(stiction_kick_nm), seconds=float(stiction_kick_s))
            torque_bump(axis, torque=-float(stiction_kick_nm), seconds=float(stiction_kick_s))

        # Command the target
        axis.controller.input_pos = float(target_turns)

        # Wait for movement / target reach (or error)
        t0 = time.time()
        moved = False
        reached = False
        target_f = float(target_turns)
        def _emit_sample(stage, pos_val, vel_val, **extra):
            if sample_hook is None:
                return
            try:
                input_pos_now = _safe_f(lambda: axis.controller.input_pos, None)
            except Exception:
                input_pos_now = None
            payload = {
                "stage": str(stage),
                "timestamp_s": time.time(),
                "state": int(getattr(axis, "current_state", 0) or 0),
                "target": float(target_f),
                "pos_est": float(pos_val),
                "vel_est": float(vel_val),
                "input_pos": (None if input_pos_now is None else float(input_pos_now)),
                "pos_setpoint": _safe_f(lambda: axis.controller.pos_setpoint, None),
                "Iq_set": _safe_f(lambda: axis.motor.current_control.Iq_setpoint, None),
                "Iq_meas": _safe_f(lambda: axis.motor.current_control.Iq_measured, None),
                "axis_err": _safe_f(lambda: int(axis.error), 0),
                "motor_err": _safe_f(lambda: int(axis.motor.error), 0),
                "enc_err": _safe_f(lambda: int(axis.encoder.error), 0),
                "ctrl_err": _safe_f(lambda: int(axis.controller.error), 0),
                "enc_ready": _safe_f(lambda: bool(axis.encoder.is_ready), False),
                "enc_index_found": _safe_f(lambda: bool(getattr(axis.encoder, "index_found", False)), False),
            }
            if (payload["input_pos"] is not None) and (payload["pos_est"] is not None):
                payload["tracking_err_turns"] = float(payload["input_pos"]) - float(payload["pos_est"])
            else:
                payload["tracking_err_turns"] = None
            if extra:
                payload.update(extra)
            sample_hook(payload)
        _emit_sample("commanded", p0, 0.0)
        # Motion-quality telemetry (used by higher-level smoothness gating)
        sample_count = 0
        peak_abs_vel = 0.0
        peak_abs_acc = 0.0
        peak_abs_jerk = 0.0
        vel_sign_changes = 0
        vel_sign_eps = 0.003
        prev_sample_t = time.monotonic()
        prev_vel = None
        prev_acc = None
        prev_nonzero_sign = 0

        # Stall watchdog configuration
        wd_enabled = bool(stall_watchdog)
        # Determine a reasonable current threshold if not provided.
        wd_iq = None
        if wd_enabled:
            try:
                # Prefer the explicit current_lim argument if provided, else use configured limit.
                ilim_cfg = float(axis.motor.config.current_lim)
            except Exception:
                ilim_cfg = None
            ilim_ref = float(current_lim) if current_lim is not None else ilim_cfg
            if stall_iq_a is not None:
                wd_iq = float(stall_iq_a)
            elif ilim_ref is not None:
                wd_iq = max(2.0, 0.6 * float(ilim_ref))
            else:
                wd_iq = 5.0

        wd_eps = float(stall_pos_eps_turns)
        wd_t = float(stall_time_s)
        wd_last_pos = p0
        wd_stall_start = None
        # Divergence watchdog: if we move far away from the requested target,
        # abort early instead of timing out while chasing a bad state.
        req_dist = abs(float(target_f) - float(p0))
        cmd_sign = 1 if float(target_f) > float(p0) else (-1 if float(target_f) < float(p0) else 0)
        try:
            motor_direction = -1 if float(getattr(axis.motor.config, "direction", 1.0)) < 0.0 else 1
        except Exception:
            motor_direction = 1
        expected_iq_sign = int(cmd_sign) * int(motor_direction)
        rev_eps = max(0.0, float(reverse_motion_eps_turns))
        rev_need = max(1, int(reverse_motion_confirm_samples))
        rev_hits = 0
        iq_inv_min = max(0.0, float(inverted_iq_min_a))
        iq_inv_need = max(1, int(inverted_iq_confirm_samples))
        iq_inv_grace = max(0.0, float(inverted_iq_grace_s))
        iq_inv_hits = 0
        err0 = float(target_f) - float(p0)
        away_need = 3
        away_hits = 0
        away_eps = max(float(target_tolerance_turns) * 2.0, 0.01, 0.25 * float(req_dist))
        div_err_thresh = max(0.50, 4.0 * float(req_dist), 20.0 * float(target_tolerance_turns))
        div_move_thresh = max(0.20, 2.0 * float(req_dist), 40.0 * float(target_tolerance_turns))
        div_hits = 0

        while time.time() - t0 < float(timeout_s):
            assert_no_errors(axis, label="move_to_pos_strict")
            p = _safe_f(lambda: axis.encoder.pos_estimate, p0)
            v = _safe_f(lambda: axis.encoder.vel_estimate, 0.0)
            sample_count += 1
            peak_abs_vel = max(float(peak_abs_vel), abs(float(v)))
            now_sample_t = time.monotonic()
            dt_sample = max(1e-4, float(now_sample_t) - float(prev_sample_t))
            if prev_vel is not None:
                acc = (float(v) - float(prev_vel)) / float(dt_sample)
                peak_abs_acc = max(float(peak_abs_acc), abs(float(acc)))
                if prev_acc is not None:
                    jerk = (float(acc) - float(prev_acc)) / float(dt_sample)
                    peak_abs_jerk = max(float(peak_abs_jerk), abs(float(jerk)))
                prev_acc = float(acc)
            prev_vel = float(v)
            prev_sample_t = float(now_sample_t)
            v_sign = 1 if float(v) > float(vel_sign_eps) else (-1 if float(v) < -float(vel_sign_eps) else 0)
            if (v_sign != 0) and (prev_nonzero_sign != 0) and (v_sign != prev_nonzero_sign):
                vel_sign_changes += 1
            if v_sign != 0:
                prev_nonzero_sign = int(v_sign)

            _emit_sample(
                "move",
                p,
                v,
                sample_count=int(sample_count),
                peak_abs_vel=float(peak_abs_vel),
                peak_abs_acc=float(peak_abs_acc),
                peak_abs_jerk=float(peak_abs_jerk),
            )

            # Success condition: meaningful motion observed.
            if abs(p - p0) >= float(min_delta_turns):
                moved = True
                if not bool(require_target_reached):
                    break

            if bool(require_target_reached):
                if (abs(target_f - p) <= float(target_tolerance_turns)) and (abs(v) <= float(target_vel_tolerance_turns_s)):
                    moved = True
                    reached = True
                    break

            # Control-sign watchdog: repeatedly commanding opposite-Iq for this
            # position command indicates unstable sign consistency.
            if bool(abort_on_inverted_iq) and int(cmd_sign) != 0 and ((time.time() - t0) >= float(iq_inv_grace)):
                iq_set = _safe_f(lambda: axis.motor.current_control.Iq_setpoint, 0.0)
                iq_sign = 1 if float(iq_set) > float(iq_inv_min) else (-1 if float(iq_set) < -float(iq_inv_min) else 0)
                # Some ODrive setups can expose inverted Iq_set polarity while still
                # moving in the correct commanded direction. Only treat inverted-Iq as
                # unsafe if displacement has not yet confirmed commanded motion.
                disp_now = float(p) - float(p0)
                disp_sign_now = 1 if disp_now > 0.0 else (-1 if disp_now < 0.0 else 0)
                disp_confirm_eps = max(0.5 * float(rev_eps), 0.25 * float(min_delta_turns), 4e-4)
                disp_confirms_cmd = bool(
                    (int(disp_sign_now) != 0)
                    and (int(disp_sign_now) == int(cmd_sign))
                    and (abs(float(disp_now)) >= float(disp_confirm_eps))
                )
                if bool(disp_confirms_cmd):
                    iq_inv_hits = 0
                elif int(iq_sign) != 0 and int(expected_iq_sign) != 0 and int(iq_sign) != int(expected_iq_sign):
                    iq_inv_hits += 1
                else:
                    iq_inv_hits = 0
                if int(iq_inv_hits) >= int(iq_inv_need):
                    if bool(fail_to_idle):
                        try:
                            axis.requested_state = AXIS_STATE_IDLE
                        except Exception:
                            pass
                        time.sleep(0.05)
                    snap = _snapshot_motion(axis)
                    raise RuntimeError(
                        "CONTROL_SIGN_INCONSISTENT watchdog tripped: Iq_set sign opposes commanded position direction. "
                        f"target={float(target_f):+.6f}t start={float(p0):+.6f}t pos={float(p):+.6f}t "
                        f"cmd_sign={int(cmd_sign)} motor_direction={int(motor_direction)} "
                        f"expected_iq_sign={int(expected_iq_sign)} iq_set={float(iq_set):+.6f}A "
                        f"iq_inv_hits={int(iq_inv_hits)} iq_inv_min={float(iq_inv_min):.3f}A snapshot={snap}"
                    )

            # Target-away watchdog: abort if closed-loop drive keeps increasing
            # position error in the same direction as initial error.
            if int(cmd_sign) != 0:
                err_now = float(target_f) - float(p)
                iq_now = _safe_f(lambda: axis.motor.current_control.Iq_setpoint, 0.0)
                pushing = abs(float(iq_now)) >= float(iq_inv_min)
                growing = abs(float(err_now)) > (abs(float(err0)) + float(away_eps))
                same_side = (int((1 if err_now > 0.0 else (-1 if err_now < 0.0 else 0))) == int((1 if err0 > 0.0 else (-1 if err0 < 0.0 else 0))))
                if bool(pushing and growing and same_side):
                    away_hits += 1
                else:
                    away_hits = 0
                if int(away_hits) >= int(away_need):
                    if bool(fail_to_idle):
                        try:
                            axis.requested_state = AXIS_STATE_IDLE
                        except Exception:
                            pass
                        time.sleep(0.05)
                    snap = _snapshot_motion(axis)
                    raise RuntimeError(
                        "TARGET_AWAY_RUNAWAY watchdog tripped: error magnitude increased while driven. "
                        f"target={float(target_f):+.6f}t start={float(p0):+.6f}t pos={float(p):+.6f}t "
                        f"err0={float(err0):+.6f}t err_now={float(err_now):+.6f}t away_hits={int(away_hits)} "
                        f"away_eps={float(away_eps):.6f}t iq_set={float(iq_now):+.6f}A snapshot={snap}"
                    )

            # Wrong-direction watchdog: if confirmed movement opposes command sign, abort early.
            if bool(abort_on_reverse_motion) and int(cmd_sign) != 0 and float(req_dist) > 0.0:
                disp = float(p) - float(p0)
                if abs(float(disp)) >= float(rev_eps):
                    disp_sign = 1 if float(disp) > 0.0 else (-1 if float(disp) < 0.0 else 0)
                    if int(disp_sign) != 0 and int(disp_sign) != int(cmd_sign):
                        rev_hits += 1
                    else:
                        rev_hits = 0
                else:
                    rev_hits = 0
                if int(rev_hits) >= int(rev_need):
                    if bool(fail_to_idle):
                        try:
                            axis.requested_state = AXIS_STATE_IDLE
                        except Exception:
                            pass
                        time.sleep(0.05)
                    snap = _snapshot_motion(axis)
                    raise RuntimeError(
                        "WRONG_DIRECTION watchdog tripped: observed motion opposes commanded target. "
                        f"target={float(target_f):+.6f}t start={float(p0):+.6f}t pos={float(p):+.6f}t "
                        f"disp={float(disp):+.6f}t cmd_sign={int(cmd_sign)} rev_hits={int(rev_hits)} "
                        f"rev_eps={float(rev_eps):.6f}t snapshot={snap}"
                    )

            # Divergence watchdog: avoid long runaways with no hard fault bit.
            # We require repeated outlier samples to avoid tripping on single-sample noise.
            if (abs(float(target_f) - float(p)) >= float(div_err_thresh)) and (abs(float(p) - float(p0)) >= float(div_move_thresh)):
                div_hits += 1
            else:
                div_hits = 0
            if div_hits >= 3:
                if bool(fail_to_idle):
                    try:
                        axis.requested_state = AXIS_STATE_IDLE
                    except Exception:
                        pass
                    time.sleep(0.05)
                snap = _snapshot_motion(axis)
                raise RuntimeError(
                    "DIVERGENCE watchdog tripped: axis moved far away from command target. "
                    f"target={float(target_f):+.6f}t pos={float(p):+.6f}t err={float(target_f) - float(p):+.6f}t "
                    f"vel={float(v):+.6f}t/s req_dist={float(req_dist):.6f}t "
                    f"err_thresh={float(div_err_thresh):.6f}t move_thresh={float(div_move_thresh):.6f}t snapshot={snap}"
                )

            # Stall watchdog: if we're pushing significant current but position isn't changing,
            # abort early to avoid heating the power stage.
            if wd_enabled:
                iq = _safe_f(lambda: axis.motor.current_control.Iq_measured, None)
                now = time.time()
                if iq is not None and abs(iq) >= float(wd_iq):
                    # Consider "no motion" if position hasn't changed beyond a tiny epsilon.
                    if abs(p - wd_last_pos) <= wd_eps:
                        if wd_stall_start is None:
                            wd_stall_start = now
                        elif (now - wd_stall_start) >= wd_t:
                            # Force IDLE immediately (best-effort) and raise.
                            if bool(fail_to_idle):
                                try:
                                    axis.requested_state = AXIS_STATE_IDLE
                                except Exception:
                                    pass
                                time.sleep(0.05)
                            snap = _snapshot_motion(axis)
                            raise RuntimeError(
                                "STALL watchdog tripped: high current with no motion. "
                                f"|Iq|={abs(iq):.3f}A >= {float(wd_iq):.3f}A for {wd_t:.2f}s "
                                f"Δpos_step={p - wd_last_pos:+.6f}t eps={wd_eps:.6f}t snapshot={snap}"
                            )
                    else:
                        # We saw some micro-motion; reset the stall timer.
                        wd_stall_start = None
                        wd_last_pos = p
                else:
                    # Not pushing hard current; don't accumulate stall time.
                    wd_stall_start = None
                    wd_last_pos = p

            time.sleep(0.02)

        time.sleep(float(settle_s))
        p1 = _safe_f(lambda: axis.encoder.pos_estimate, p0)
        v1 = _safe_f(lambda: axis.encoder.vel_estimate, 0.0)
        _emit_sample("post_settle", p1, v1)

        if not moved:
            snap = _snapshot_motion(axis)
            raise RuntimeError(
                "No movement detected after position command. "
                f"Δpos={p1 - p0:+.6f}t (<{min_delta_turns}t) target={float(target_turns):+.6f}t snapshot={snap}"
            )

        quiet_hold_meta = {
            "enabled": bool(quiet_hold_enable),
            "applied": False,
            "persisted": False,
            "duration_s": 0.0,
            "peak_abs_vel": 0.0,
            "peak_abs_err": 0.0,
            "reanchored": False,
            "hold_target": float(target_f),
        }

        if bool(require_target_reached):
            # Accept "late settle" if final post-settle sample is already in tolerance.
            if (not bool(reached)) and (
                (abs(float(target_f) - float(p1)) <= float(target_tolerance_turns))
                and (abs(float(v1)) <= float(target_vel_tolerance_turns_s))
            ):
                reached = True
            if not bool(reached):
                snap = _snapshot_motion(axis)
                abs_err_final = abs(float(target_f) - float(p1))
                abs_move_final = abs(float(p1) - float(p0))
                if (abs_err_final >= float(div_err_thresh)) and (abs_move_final >= float(div_move_thresh)):
                    raise RuntimeError(
                        "DIVERGENCE watchdog tripped at timeout: axis ended far from command target. "
                        f"target={float(target_f):+.6f}t pos={float(p1):+.6f}t err={float(target_f) - float(p1):+.6f}t "
                        f"vel={float(v1):+.6f}t/s req_dist={float(req_dist):.6f}t "
                        f"err_thresh={float(div_err_thresh):.6f}t move_thresh={float(div_move_thresh):.6f}t snapshot={snap}"
                    )
                raise RuntimeError(
                    "Target not reached before timeout. "
                    f"target={target_f:+.6f}t pos={p1:+.6f}t err={target_f - p1:+.6f}t "
                    f"vel={v1:+.6f}t/s tol={float(target_tolerance_turns):.6f}t "
                    f"vel_tol={float(target_vel_tolerance_turns_s):.6f}t/s snapshot={snap}"
                )
            # Re-check after settle delay: some moves briefly satisfy the target
            # condition, then drift back out once the loop relaxes.
            if (abs(target_f - p1) > float(target_tolerance_turns)) or (abs(v1) > float(target_vel_tolerance_turns_s)):
                # Allow a short bounded recovery window before declaring hard failure.
                # This improves robustness on compliant transmissions where residual
                # ringing decays just after the first settle probe.
                hold_recover_s = min(
                    2.0,
                    max(0.80, (2.0 * float(settle_s)), (0.30 * float(timeout_s))),
                )
                hold_deadline = time.time() + float(hold_recover_s)
                hold_samples_need = 3
                hold_samples_ok = 0
                hold_peak_abs_vel = abs(float(v1))
                hold_positions = [float(p1)]
                hold_window_samples = 12
                try:
                    cpr_est = int(getattr(axis.encoder.config, "cpr", 0) or 0)
                except Exception:
                    cpr_est = 0
                stable_pos_span_tol = max(
                    8e-4,
                    0.60 * float(target_tolerance_turns),
                    ((3.0 / float(cpr_est)) if int(cpr_est) > 0 else 0.0),
                )
                recovered = False
                while time.time() < float(hold_deadline):
                    assert_no_errors(axis, label="move_to_pos_strict/hold_recover")
                    try:
                        axis.controller.input_pos = float(target_f)
                    except Exception:
                        pass
                    p_try = _safe_f(lambda: axis.encoder.pos_estimate, p1)
                    v_try = _safe_f(lambda: axis.encoder.vel_estimate, v1)
                    hold_peak_abs_vel = max(float(hold_peak_abs_vel), abs(float(v_try)))
                    hold_positions.append(float(p_try))
                    if len(hold_positions) > int(hold_window_samples):
                        hold_positions = hold_positions[-int(hold_window_samples):]
                    if len(hold_positions) >= 2:
                        hold_span = float(max(hold_positions) - min(hold_positions))
                    else:
                        hold_span = 0.0
                    in_tol_pos = bool(abs(float(target_f) - float(p_try)) <= float(target_tolerance_turns))
                    in_tol_vel = bool(abs(float(v_try)) <= float(target_vel_tolerance_turns_s))
                    stable_pos = bool(float(hold_span) <= float(stable_pos_span_tol))
                    in_tol = bool(in_tol_pos and (in_tol_vel or stable_pos))
                    if bool(in_tol):
                        hold_samples_ok += 1
                    else:
                        hold_samples_ok = 0
                    if int(hold_samples_ok) >= int(hold_samples_need):
                        p1 = float(p_try)
                        v1 = float(v_try)
                        recovered = True
                        break
                    time.sleep(0.02)
                if not bool(recovered):
                    snap = _snapshot_motion(axis)
                    raise RuntimeError(
                        "Target reached transiently but did not hold after settle. "
                        f"target={target_f:+.6f}t pos={p1:+.6f}t err={target_f - p1:+.6f}t "
                        f"vel={v1:+.6f}t/s tol={float(target_tolerance_turns):.6f}t "
                        f"vel_tol={float(target_vel_tolerance_turns_s):.6f}t/s "
                        f"hold_recover_s={float(hold_recover_s):.3f} "
                        f"hold_peak_abs_vel={float(hold_peak_abs_vel):.6f}t/s "
                        f"hold_pos_span_tol={float(stable_pos_span_tol):.6f}t snapshot={snap}"
                    )

            # Final low-noise hold conditioning for compliant geartrains:
            # lower stiffness and clamp velocity authority briefly after reach,
            # which reduces audible hunting/humming at standstill.
            if bool(quiet_hold_enable) and bool(reached):
                q_hold_s = max(0.0, float(quiet_hold_s))
                if q_hold_s > 0.0:
                    q_t0 = time.time()
                    try:
                        q_prev_pos_gain = float(axis.controller.config.pos_gain)
                    except Exception:
                        q_prev_pos_gain = None
                    try:
                        q_prev_vel_gain = float(axis.controller.config.vel_gain)
                    except Exception:
                        q_prev_vel_gain = None
                    try:
                        q_prev_vel_i_gain = float(axis.controller.config.vel_integrator_gain)
                    except Exception:
                        q_prev_vel_i_gain = None
                    try:
                        q_prev_vel_limit = float(axis.controller.config.vel_limit)
                    except Exception:
                        q_prev_vel_limit = None

                    try:
                        q_pos_base = float(q_prev_pos_gain if q_prev_pos_gain is not None else pos_gain)
                    except Exception:
                        q_pos_base = float(pos_gain)
                    try:
                        q_vel_base = float(q_prev_vel_gain if q_prev_vel_gain is not None else vel_gain)
                    except Exception:
                        q_vel_base = float(vel_gain)
                    try:
                        q_vel_lim_base = float(q_prev_vel_limit if q_prev_vel_limit is not None else vel_limit)
                    except Exception:
                        q_vel_lim_base = float(vel_limit)

                    try:
                        q_hold_target = float(target_f)
                        try:
                            q_reanchor_band = (
                                None
                                if quiet_hold_reanchor_err_turns is None
                                else max(0.0, float(quiet_hold_reanchor_err_turns))
                            )
                        except Exception:
                            q_reanchor_band = None
                        if (q_reanchor_band is not None) and (
                            abs(float(target_f) - float(p1)) <= float(q_reanchor_band)
                        ):
                            q_hold_target = float(p1)
                        try:
                            axis.controller.config.pos_gain = max(
                                0.50, float(q_pos_base) * max(0.10, float(quiet_hold_pos_gain_scale))
                            )
                        except Exception:
                            pass
                        try:
                            axis.controller.config.vel_gain = max(
                                0.01, float(q_vel_base) * max(0.10, float(quiet_hold_vel_gain_scale))
                            )
                        except Exception:
                            pass
                        try:
                            axis.controller.config.vel_integrator_gain = max(0.0, float(quiet_hold_vel_i_gain))
                        except Exception:
                            pass
                        try:
                            axis.controller.config.vel_limit = max(
                                0.05, float(q_vel_lim_base) * max(0.10, float(quiet_hold_vel_limit_scale))
                            )
                        except Exception:
                            pass

                        q_deadline = time.time() + float(q_hold_s)
                        q_peak_abs_vel = 0.0
                        q_peak_abs_err = 0.0
                        q_last_p = float(p1)
                        q_last_v = float(v1)
                        while time.time() < float(q_deadline):
                            assert_no_errors(axis, label="move_to_pos_strict/quiet_hold")
                            try:
                                axis.controller.input_pos = float(q_hold_target)
                            except Exception:
                                pass
                            q_last_p = _safe_f(lambda: axis.encoder.pos_estimate, q_last_p)
                            q_last_v = _safe_f(lambda: axis.encoder.vel_estimate, q_last_v)
                            q_peak_abs_vel = max(float(q_peak_abs_vel), abs(float(q_last_v)))
                            q_peak_abs_err = max(float(q_peak_abs_err), abs(float(q_hold_target) - float(q_last_p)))
                            _emit_sample(
                                "quiet_hold",
                                q_last_p,
                                q_last_v,
                                hold_target=float(q_hold_target),
                                hold_peak_abs_err=float(q_peak_abs_err),
                            )
                            time.sleep(0.02)

                        # Update returned terminal sample after quiet-hold.
                        p1 = float(q_last_p)
                        v1 = float(q_last_v)
                        quiet_hold_meta = {
                            "enabled": True,
                            "applied": True,
                            "persisted": bool(quiet_hold_persist),
                            "duration_s": float(time.time() - float(q_t0)),
                            "peak_abs_vel": float(q_peak_abs_vel),
                            "peak_abs_err": float(q_peak_abs_err),
                            "reanchored": bool(q_hold_target != float(target_f)),
                            "hold_target": float(q_hold_target),
                        }
                        _emit_sample(
                            "final",
                            p1,
                            v1,
                            reached=bool(reached),
                            quiet_hold=bool(True),
                        )
                    finally:
                        if not bool(quiet_hold_persist):
                            # Restore command-time gains/limits for subsequent move setup logic.
                            if q_prev_pos_gain is not None:
                                try:
                                    axis.controller.config.pos_gain = float(q_prev_pos_gain)
                                except Exception:
                                    pass
                            if q_prev_vel_gain is not None:
                                try:
                                    axis.controller.config.vel_gain = float(q_prev_vel_gain)
                                except Exception:
                                    pass
                            if q_prev_vel_i_gain is not None:
                                try:
                                    axis.controller.config.vel_integrator_gain = float(q_prev_vel_i_gain)
                                except Exception:
                                    pass
                            if q_prev_vel_limit is not None:
                                try:
                                    axis.controller.config.vel_limit = float(q_prev_vel_limit)
                                except Exception:
                                    pass

        return {
            "start": p0,
            "target": float(target_turns),
            "end": p1,
            "vel": v1,
            "err": target_f - p1,
            "state": int(getattr(axis, "current_state", 0)),
            "reached": bool(reached) if bool(require_target_reached) else None,
            "duration_s": float(time.time() - t0),
            "sample_count": int(sample_count),
            "peak_abs_vel": float(peak_abs_vel),
            "peak_abs_acc": float(peak_abs_acc),
            "peak_abs_jerk": float(peak_abs_jerk),
            "vel_sign_changes": int(vel_sign_changes),
            "quiet_hold": dict(quiet_hold_meta),
        }

    finally:
        # Always stop commanding torque implicitly and try to leave the axis in a safe state.
        # (Important when exceptions occur mid-move, or when the USB/RPC link flakes out.)
        try:
            # Clear any torque command if supported
            if hasattr(axis.controller, "input_torque"):
                axis.controller.input_torque = 0.0
        except Exception:
            pass

        # Restore current limit if we changed it
        if prev_ilim is not None:
            try:
                axis.motor.config.current_lim = float(prev_ilim)
            except Exception:
                pass

        if prev_vel_limit_tolerance is not None:
            try:
                axis.controller.config.vel_limit_tolerance = float(prev_vel_limit_tolerance)
            except Exception:
                pass

        if prev_enable_overspeed_error is not None:
            try:
                axis.controller.config.enable_overspeed_error = bool(prev_enable_overspeed_error)
            except Exception:
                pass

        # If requested, force IDLE to avoid holding current on a stuck load.
        if bool(fail_to_idle):
            try:
                axis.requested_state = AXIS_STATE_IDLE
            except Exception:
                pass
            try:
                time.sleep(0.05)
            except Exception:
                pass


def qtr_turn(axis=None, delta_turns=0.25, use_trap_traj=True, verbose=True):
    """Quarter-turn (or any delta_turns). Raises if it doesn't actually move."""
    if axis is None:
        axis = get_axis0()

    refresh_checks(axis, verbose=False, force_live=True)
    axis = get_axis0()

    start = _safe_f(lambda: axis.encoder.pos_estimate, 0.0)
    target = start + float(delta_turns)

    if verbose:
        print("state:", int(getattr(axis, "current_state", 0)))
        print("start:", start)
        print("target:", target)

    res = move_to_pos_strict(
        axis,
        target,
        use_trap_traj=bool(use_trap_traj),
        # harmonic drive: quiet but enough authority
        current_lim=None,       # set to e.g. 20 if needed
        pos_gain=20.0,
        vel_gain=0.30,
        vel_i_gain=0.60,
        vel_limit=5.0,
        trap_vel=2.0,
        trap_acc=5.0,
        trap_dec=5.0,
        input_filter_bw=0.0,
        stiction_kick_nm=None,  # set to e.g. 0.15 if it sticks
        timeout_s=3.0,
        min_delta_turns=0.002,
        settle_s=0.20,
    )

    if verbose:
        print("end pos_est:", res["end"])
        print("end input_pos:", _safe_f(lambda: axis.controller.input_pos, None))

    return res
    
def torque_bump(axis, torque=0.2, seconds=0.25):
    """Small torque pulse for direction sanity checks, then restore previous modes.

    Uses try/finally so we always restore control/input modes and clear the torque command,
    even if the RPC link drops mid-command.
    """
    import time

    # Save previous controller modes (best-effort)
    try:
        prev_control = axis.controller.config.control_mode
    except Exception:
        prev_control = None
    try:
        prev_input = axis.controller.config.input_mode
    except Exception:
        prev_input = None

    try:
        # Torque mode pulse
        try:
            axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
        except Exception:
            pass
        try:
            axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        except Exception:
            pass

            # Apply torque (Nm)
        try:
            axis.controller.input_torque = float(torque)
        except Exception:
            pass
        time.sleep(float(seconds))

    finally:
        # Always clear torque and restore modes
        try:
            axis.controller.input_torque = 0.0
        except Exception:
            pass
        if prev_control is not None:
            try:
                axis.controller.config.control_mode = prev_control
            except Exception:
                pass
        if prev_input is not None:
            try:
                axis.controller.config.input_mode = prev_input
            except Exception:
                pass

def direction_sanity(axis=None, torque=0.20, seconds=0.15, verbose=True):
    """Check sign consistency between motor torque and encoder pos_est.

    Applies +torque then -torque and reports Δpos for each.
    If +torque makes pos_est move the 'wrong way' or position control runs away,
    you likely have a sign mismatch (motor direction vs encoder direction).
    """
    if axis is None:
        axis = get_axis0()

    # Ensure closed-loop so torque commands have effect
    try:
        if int(getattr(axis, "current_state", 0)) != int(AXIS_STATE_CLOSED_LOOP_CONTROL):
            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            time.sleep(0.20)
    except Exception:
        pass

    try:
        p0 = float(axis.encoder.pos_estimate)
    except Exception:
        p0 = 0.0

    torque_bump(axis, torque=float(torque), seconds=float(seconds))
    try:
        p1 = float(axis.encoder.pos_estimate)
    except Exception:
        p1 = p0

    torque_bump(axis, torque=-float(torque), seconds=float(seconds))
    try:
        p2 = float(axis.encoder.pos_estimate)
    except Exception:
        p2 = p1

    dp_plus = p1 - p0
    dp_minus = p2 - p1

    if verbose:
        print(f"direction_sanity: p0={p0:+.6f} p1={p1:+.6f} p2={p2:+.6f} | +tq Δp={dp_plus:+.6f}t, -tq Δp={dp_minus:+.6f}t")
        try:
            md = int(getattr(axis.motor.config, "direction", 0))
        except Exception:
            md = None
        try:
            ed = int(getattr(axis.encoder.config, "direction", 0))
        except Exception:
            ed = None
        print(f"  motor.config.direction={md} encoder.config.direction={ed} (encoder direction may be unavailable)")

    return {"p0": p0, "p1": p1, "p2": p2, "dp_plus": dp_plus, "dp_minus": dp_minus}


def sign_and_breakaway_probe(
    axis=None,
    cmd_delta_turns=0.05,
    current_lim=12.0,
    pos_gain=20.0,
    vel_gain=0.24,
    vel_i_gain=0.0,
    vel_limit=1.0,
    trap_vel=0.15,
    trap_acc=0.30,
    trap_dec=0.30,
    timeout_s=2.0,
    settle_s=0.05,
    min_delta_turns=0.001,
    leave_idle=True,
    verbose=True,
):
    """Probe loaded behavior with one +position and one -position command.

    Returns a compact classification to quickly separate:
      - high-torque no-motion (likely breakaway/mechanical issue),
      - sign inversion,
      - one-direction-only / directional bias.
    """
    if axis is None:
        axis = get_axis0()

    clear_errors_all(axis)
    if not ensure_closed_loop(axis, timeout_s=3.0, clear_first=False, pre_sync=True, retries=2):
        raise RuntimeError("sign_and_breakaway_probe: failed to enter CLOSED_LOOP_CONTROL")

    probe = {
        "start_pos": float(getattr(axis.encoder, "pos_estimate", 0.0)),
        "start_shadow": int(getattr(axis.encoder, "shadow_count", 0)),
        "cmd_delta_turns": float(cmd_delta_turns),
        "current_lim": float(current_lim),
        "tests": [],
        "classification": None,
    }

    def _run_step(delta):
        clear_errors_all(axis)
        ensure_closed_loop(axis, timeout_s=2.0, clear_first=False, pre_sync=True, retries=1)
        p0 = float(getattr(axis.encoder, "pos_estimate", 0.0))
        q0 = int(getattr(axis.encoder, "shadow_count", 0))
        ok = True
        err = None
        try:
            move_to_pos_strict(
                axis,
                p0 + float(delta),
                use_trap_traj=True,
                timeout_s=float(timeout_s),
                min_delta_turns=float(min_delta_turns),
                settle_s=float(settle_s),
                vel_limit=float(vel_limit),
                vel_limit_tolerance=12.0,
                enable_overspeed_error=False,
                trap_vel=float(trap_vel),
                trap_acc=float(trap_acc),
                trap_dec=float(trap_dec),
                current_lim=float(current_lim),
                pos_gain=float(pos_gain),
                vel_gain=float(vel_gain),
                vel_i_gain=float(vel_i_gain),
                stiction_kick_nm=0.0,
                target_tolerance_turns=0.012,
                target_vel_tolerance_turns_s=0.15,
                require_target_reached=False,
                fail_to_idle=False,
            )
        except Exception as exc:
            ok = False
            err = str(exc)

        p1 = float(getattr(axis.encoder, "pos_estimate", 0.0))
        q1 = int(getattr(axis.encoder, "shadow_count", 0))
        return {
            "delta_cmd": float(delta),
            "ok": bool(ok),
            "dq": int(q1 - q0),
            "dp": float(p1 - p0),
            "p0": float(p0),
            "p1": float(p1),
            "iq_set": float(getattr(axis.motor.current_control, "Iq_setpoint", 0.0)),
            "iq_meas": float(getattr(axis.motor.current_control, "Iq_measured", 0.0)),
            "axis_err": int(getattr(axis, "error", 0)),
            "motor_err": int(getattr(axis.motor, "error", 0)),
            "ctrl_err": int(getattr(axis.controller, "error", 0)),
            "error": err,
        }

    plus = _run_step(abs(float(cmd_delta_turns)))
    minus = _run_step(-abs(float(cmd_delta_turns)))
    probe["tests"] = [plus, minus]

    motion_eps_t = max(float(min_delta_turns), 5e-4)
    moved_plus = abs(float(plus["dp"])) >= motion_eps_t or abs(int(plus["dq"])) >= 2
    moved_minus = abs(float(minus["dp"])) >= motion_eps_t or abs(int(minus["dq"])) >= 2
    iq_peak = max(abs(float(plus["iq_meas"])), abs(float(minus["iq_meas"])))

    if (not moved_plus) and (not moved_minus):
        if iq_peak >= 0.5 * float(current_lim):
            cls = "stuck_high_torque_no_motion"
        else:
            cls = "low_authority_no_motion"
    elif moved_plus and moved_minus:
        if float(plus["dp"]) > 0.0 and float(minus["dp"]) < 0.0:
            cls = "sign_ok"
        elif float(plus["dp"]) < 0.0 and float(minus["dp"]) > 0.0:
            cls = "sign_inverted"
        else:
            cls = "directional_bias_or_limit"
    else:
        cls = "one_direction_only_or_limit"

    probe["classification"] = cls
    probe["iq_peak"] = float(iq_peak)
    probe["end_pos"] = float(getattr(axis.encoder, "pos_estimate", 0.0))
    probe["end_shadow"] = int(getattr(axis.encoder, "shadow_count", 0))
    probe["end_errors"] = {
        "axis_err": int(getattr(axis, "error", 0)),
        "motor_err": int(getattr(axis.motor, "error", 0)),
        "enc_err": int(getattr(axis.encoder, "error", 0)),
        "ctrl_err": int(getattr(axis.controller, "error", 0)),
    }

    if leave_idle:
        try:
            axis.requested_state = AXIS_STATE_IDLE
        except Exception:
            pass

    if verbose:
        print(
            "sign_and_breakaway_probe:",
            f"classification={probe['classification']}",
            f"iq_peak={probe['iq_peak']:.3f}A",
            f"+dp={plus['dp']:+.6f}t",
            f"-dp={minus['dp']:+.6f}t",
            f"+dq={plus['dq']:+d}",
            f"-dq={minus['dq']:+d}",
        )

    return probe


def torque_authority_ramp_probe(
    axis=None,
    torque_targets_nm=(0.02, -0.02, 0.04, -0.04),
    current_lim=10.0,
    vel_limit=0.5,
    ramp_s=0.12,
    dwell_s=0.12,
    settle_s=0.06,
    dt=0.01,
    min_motion_turns=8e-4,
    min_motion_counts=2,
    iq_set_gate_a=0.25,
    iq_meas_gate_a=0.05,
    track_ratio_min=0.35,
    sign_match_min=0.65,
    vel_abort_turns_s=6.0,
    leave_idle=False,
    collect_samples=False,
    verbose=True,
):
    """Low-shock authority probe using bounded torque ramps.

    This checks whether current tracking is physically present (Iq_meas follows Iq_set)
    before higher-level position commands are attempted.
    """
    if axis is None:
        axis = get_axis0()

    probe = {
        "classification": None,
        "ok": False,
        "torque_targets_nm": list(torque_targets_nm or []),
        "current_lim": float(current_lim),
        "vel_limit": float(vel_limit),
        "ramp_s": float(ramp_s),
        "dwell_s": float(dwell_s),
        "settle_s": float(settle_s),
        "dt": float(dt),
        "iq_set_gate_a": float(iq_set_gate_a),
        "iq_meas_gate_a": float(iq_meas_gate_a),
        "track_ratio_min": float(track_ratio_min),
        "sign_match_min": float(sign_match_min),
        "vel_abort_turns_s": float(vel_abort_turns_s),
        "targets": [],
    }

    try:
        clear_errors_all(axis)
    except Exception:
        pass
    if not ensure_closed_loop(axis, timeout_s=3.0, clear_first=False, pre_sync=True, retries=2):
        raise RuntimeError("torque_authority_ramp_probe: failed to enter CLOSED_LOOP_CONTROL")

    try:
        prev_control = int(getattr(axis.controller.config, "control_mode", CONTROL_MODE_POSITION_CONTROL))
    except Exception:
        prev_control = None
    try:
        prev_input = int(getattr(axis.controller.config, "input_mode", INPUT_MODE_PASSTHROUGH))
    except Exception:
        prev_input = None
    try:
        prev_current_lim = float(getattr(axis.motor.config, "current_lim", float(current_lim)))
    except Exception:
        prev_current_lim = None
    try:
        prev_vel_limit = float(getattr(axis.controller.config, "vel_limit", float(vel_limit)))
    except Exception:
        prev_vel_limit = None

    iq_track_ratios = []
    sign_evals = []
    runaway = False
    plus_moved = False
    minus_moved = False
    samples = []

    def _sample(cmd_torque, phase):
        iq_set = float(getattr(axis.motor.current_control, "Iq_setpoint", 0.0))
        iq_meas = float(getattr(axis.motor.current_control, "Iq_measured", 0.0))
        vel = float(getattr(axis.encoder, "vel_estimate", 0.0))
        pos = float(getattr(axis.encoder, "pos_estimate", 0.0))
        q = int(getattr(axis.encoder, "shadow_count", 0))
        if abs(float(iq_set)) >= float(iq_set_gate_a):
            iq_track_ratios.append(abs(float(iq_meas)) / max(1e-6, abs(float(iq_set))))
            if abs(float(iq_meas)) >= float(iq_meas_gate_a):
                sign_evals.append(1.0 if (float(iq_set) * float(iq_meas)) > 0.0 else 0.0)
        if bool(collect_samples):
            samples.append(
                {
                    "phase": str(phase),
                    "cmd_torque": float(cmd_torque),
                    "iq_set": float(iq_set),
                    "iq_meas": float(iq_meas),
                    "vel": float(vel),
                    "pos": float(pos),
                    "shadow": int(q),
                }
            )
        return iq_set, iq_meas, vel, pos, q

    try:
        try:
            axis.motor.config.current_lim = max(0.2, float(current_lim))
        except Exception:
            pass
        try:
            axis.controller.config.vel_limit = max(0.05, float(vel_limit))
        except Exception:
            pass
        try:
            axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
        except Exception:
            pass
        try:
            axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        except Exception:
            pass
        try:
            axis.controller.input_torque = 0.0
        except Exception:
            pass
        time.sleep(0.03)

        targets = list(torque_targets_nm or [])
        for tcmd in targets:
            tgt = float(tcmd)
            rec = {
                "target_torque": float(tgt),
                "runaway": False,
                "aborted": False,
                "dp": 0.0,
                "dq": 0,
                "moved": False,
                "iq_set_peak": 0.0,
                "iq_meas_peak": 0.0,
                "vel_peak": 0.0,
            }
            p0 = float(getattr(axis.encoder, "pos_estimate", 0.0))
            q0 = int(getattr(axis.encoder, "shadow_count", 0))

            n_ramp = max(2, int(max(0.02, float(ramp_s)) / max(0.005, float(dt))))
            for i in range(1, n_ramp + 1):
                cmd = float(tgt) * (float(i) / float(n_ramp))
                try:
                    axis.controller.input_torque = float(cmd)
                except Exception:
                    pass
                iq_set, iq_meas, vel, _, _ = _sample(cmd, "ramp")
                rec["iq_set_peak"] = max(float(rec["iq_set_peak"]), abs(float(iq_set)))
                rec["iq_meas_peak"] = max(float(rec["iq_meas_peak"]), abs(float(iq_meas)))
                rec["vel_peak"] = max(float(rec["vel_peak"]), abs(float(vel)))
                if abs(float(vel)) > float(vel_abort_turns_s):
                    runaway = True
                    rec["runaway"] = True
                    rec["aborted"] = True
                    break
                time.sleep(max(0.005, float(dt)))
            if rec["aborted"]:
                probe["targets"].append(rec)
                break

            n_dwell = max(1, int(max(0.01, float(dwell_s)) / max(0.005, float(dt))))
            for _ in range(n_dwell):
                try:
                    axis.controller.input_torque = float(tgt)
                except Exception:
                    pass
                iq_set, iq_meas, vel, _, _ = _sample(tgt, "dwell")
                rec["iq_set_peak"] = max(float(rec["iq_set_peak"]), abs(float(iq_set)))
                rec["iq_meas_peak"] = max(float(rec["iq_meas_peak"]), abs(float(iq_meas)))
                rec["vel_peak"] = max(float(rec["vel_peak"]), abs(float(vel)))
                if abs(float(vel)) > float(vel_abort_turns_s):
                    runaway = True
                    rec["runaway"] = True
                    rec["aborted"] = True
                    break
                time.sleep(max(0.005, float(dt)))

            try:
                axis.controller.input_torque = 0.0
            except Exception:
                pass
            n_settle = max(1, int(max(0.01, float(settle_s)) / max(0.005, float(dt))))
            for _ in range(n_settle):
                _sample(0.0, "settle")
                time.sleep(max(0.005, float(dt)))

            p1 = float(getattr(axis.encoder, "pos_estimate", 0.0))
            q1 = int(getattr(axis.encoder, "shadow_count", 0))
            dp = float(p1 - p0)
            dq = int(q1 - q0)
            moved = bool((abs(float(dp)) >= float(min_motion_turns)) or (abs(int(dq)) >= int(min_motion_counts)))
            rec["dp"] = float(dp)
            rec["dq"] = int(dq)
            rec["moved"] = bool(moved)
            if float(tgt) > 0.0:
                plus_moved = bool(plus_moved or moved)
            elif float(tgt) < 0.0:
                minus_moved = bool(minus_moved or moved)
            probe["targets"].append(rec)
            if rec["aborted"]:
                break

        if runaway:
            cls = "unstable_runaway"
        else:
            if len(iq_track_ratios) > 0:
                iq_track_ratios = sorted(iq_track_ratios)
                mid = len(iq_track_ratios) // 2
                if len(iq_track_ratios) % 2 == 1:
                    track_ratio_med = float(iq_track_ratios[mid])
                else:
                    track_ratio_med = 0.5 * float(iq_track_ratios[mid - 1] + iq_track_ratios[mid])
            else:
                track_ratio_med = 0.0

            if len(sign_evals) > 0:
                sign_match_frac = float(sum(sign_evals) / float(len(sign_evals)))
            else:
                sign_match_frac = 0.0

            probe["iq_track_ratio_med"] = float(track_ratio_med)
            probe["sign_match_frac"] = float(sign_match_frac)
            probe["plus_moved"] = bool(plus_moved)
            probe["minus_moved"] = bool(minus_moved)

            if float(track_ratio_med) < float(track_ratio_min):
                cls = "low_current_tracking"
            elif len(sign_evals) > 0 and float(sign_match_frac) < float(sign_match_min):
                cls = "current_sign_mismatch"
            elif (not bool(plus_moved)) and (not bool(minus_moved)):
                cls = "low_authority_no_motion"
            elif bool(plus_moved) and bool(minus_moved):
                cls = "authority_ok"
            else:
                cls = "one_direction_only_or_limit"

        probe["classification"] = str(cls)
        probe["ok"] = bool(cls == "authority_ok")
        probe["runaway"] = bool(runaway)
        probe["end_pos"] = float(getattr(axis.encoder, "pos_estimate", 0.0))
        probe["end_shadow"] = int(getattr(axis.encoder, "shadow_count", 0))
        probe["end_errors"] = {
            "axis_err": int(getattr(axis, "error", 0)),
            "motor_err": int(getattr(axis.motor, "error", 0)),
            "enc_err": int(getattr(axis.encoder, "error", 0)),
            "ctrl_err": int(getattr(axis.controller, "error", 0)),
        }
        if bool(collect_samples):
            probe["samples"] = list(samples)

    finally:
        try:
            axis.controller.input_torque = 0.0
        except Exception:
            pass
        if prev_current_lim is not None:
            try:
                axis.motor.config.current_lim = float(prev_current_lim)
            except Exception:
                pass
        if prev_vel_limit is not None:
            try:
                axis.controller.config.vel_limit = float(prev_vel_limit)
            except Exception:
                pass
        if prev_control is not None:
            try:
                axis.controller.config.control_mode = int(prev_control)
            except Exception:
                pass
        if prev_input is not None:
            try:
                axis.controller.config.input_mode = int(prev_input)
            except Exception:
                pass
        if bool(leave_idle):
            try:
                axis.requested_state = AXIS_STATE_IDLE
            except Exception:
                pass

    if verbose:
        print(
            "torque_authority_ramp_probe:",
            f"classification={probe.get('classification')}",
            f"track_med={float(probe.get('iq_track_ratio_med', 0.0)):.3f}",
            f"sign_match={float(probe.get('sign_match_frac', 0.0)):.3f}",
            f"plus_moved={probe.get('plus_moved')}",
            f"minus_moved={probe.get('minus_moved')}",
            f"runaway={probe.get('runaway')}",
        )

    return probe


def directional_breakaway_scan(
    axis=None,
    current_lim=8.0,
    vel_limit=0.6,
    torque_levels_nm=(0.01, 0.02, 0.03, 0.04, 0.05),
    pulse_s=0.18,
    settle_s=0.08,
    dt=0.01,
    motion_eps_turns=8e-4,
    motion_eps_counts=2,
    vel_abort_turns_s=2.5,
    leave_idle=False,
    verbose=False,
):
    """Scan directional torque breakaway and report asymmetry.

    Returns the first +torque and -torque levels that produce directional motion
    above thresholds. If only one direction breaks away, this flags one-sided
    authority (common with preload, friction bias, or sign/commutation faults).
    """
    if axis is None:
        axis = get_axis0()

    levels = []
    for lv in list(torque_levels_nm or []):
        try:
            lvf = abs(float(lv))
        except Exception:
            continue
        if lvf > 0.0:
            levels.append(float(lvf))
    levels = sorted(set(levels))

    res = {
        "ok": False,
        "classification": None,
        "levels_nm": list(levels),
        "threshold_plus_nm": None,
        "threshold_minus_nm": None,
        "asym_ratio": None,
        "runaway": False,
        "results": [],
    }
    if not levels:
        res["classification"] = "invalid_levels"
        return res

    try:
        prev_current_lim = float(getattr(axis.motor.config, "current_lim", current_lim))
    except Exception:
        prev_current_lim = None
    try:
        prev_vel_limit = float(getattr(axis.controller.config, "vel_limit", vel_limit))
    except Exception:
        prev_vel_limit = None
    try:
        prev_control = int(getattr(axis.controller.config, "control_mode", CONTROL_MODE_POSITION_CONTROL))
    except Exception:
        prev_control = None
    try:
        prev_input = int(getattr(axis.controller.config, "input_mode", INPUT_MODE_PASSTHROUGH))
    except Exception:
        prev_input = None

    try:
        clear_errors(axis)
        if prev_current_lim is not None:
            axis.motor.config.current_lim = max(0.2, float(current_lim))
        if prev_vel_limit is not None:
            axis.controller.config.vel_limit = max(0.05, float(vel_limit))
        if not ensure_closed_loop(axis, timeout_s=2.0, clear_first=False, pre_sync=True, retries=1):
            raise RuntimeError("directional_breakaway_scan: failed to enter CLOSED_LOOP_CONTROL")

        axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
        axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        axis.controller.input_torque = 0.0

        thr_plus = None
        thr_minus = None
        runaway = False
        for level in levels:
            for sign in (+1.0, -1.0):
                lbl = "plus" if sign > 0.0 else "minus"
                p0 = float(getattr(axis.encoder, "pos_estimate", 0.0))
                q0 = int(getattr(axis.encoder, "shadow_count", 0))
                iq_set_peak = 0.0
                iq_meas_peak = 0.0
                vel_peak = 0.0
                aborted = False
                reason = None
                t_end = time.time() + max(0.03, float(pulse_s))
                while time.time() < t_end:
                    axis.controller.input_torque = float(sign * level)
                    try:
                        assert_no_errors(axis, label="directional_breakaway_scan")
                    except Exception as exc:
                        aborted = True
                        reason = str(exc)
                        break
                    iq_set = float(getattr(axis.motor.current_control, "Iq_setpoint", 0.0))
                    iq_meas = float(getattr(axis.motor.current_control, "Iq_measured", 0.0))
                    vel = float(getattr(axis.encoder, "vel_estimate", 0.0))
                    iq_set_peak = max(float(iq_set_peak), abs(float(iq_set)))
                    iq_meas_peak = max(float(iq_meas_peak), abs(float(iq_meas)))
                    vel_peak = max(float(vel_peak), abs(float(vel)))
                    if abs(float(vel)) > float(vel_abort_turns_s):
                        aborted = True
                        reason = f"runaway vel={float(vel):.3f}t/s > {float(vel_abort_turns_s):.3f}t/s"
                        runaway = True
                        break
                    time.sleep(max(0.005, float(dt)))

                axis.controller.input_torque = 0.0
                time.sleep(max(0.01, float(settle_s)))
                p1 = float(getattr(axis.encoder, "pos_estimate", 0.0))
                q1 = int(getattr(axis.encoder, "shadow_count", 0))
                dp = float(p1 - p0)
                dq = int(q1 - q0)
                moved = bool(
                    abs(float(dp)) >= float(motion_eps_turns)
                    or abs(int(dq)) >= int(max(1, int(motion_eps_counts)))
                )
                moved_cmd_dir = False
                if moved:
                    if sign > 0.0:
                        moved_cmd_dir = bool((float(dp) > float(motion_eps_turns)) or (int(dq) >= int(max(1, int(motion_eps_counts)))))
                    else:
                        moved_cmd_dir = bool((float(dp) < -float(motion_eps_turns)) or (int(dq) <= -int(max(1, int(motion_eps_counts)))))

                row = {
                    "direction": str(lbl),
                    "level_nm": float(level),
                    "dp": float(dp),
                    "dq": int(dq),
                    "moved": bool(moved),
                    "moved_cmd_dir": bool(moved_cmd_dir),
                    "aborted": bool(aborted),
                    "reason": reason,
                    "iq_set_peak": float(iq_set_peak),
                    "iq_meas_peak": float(iq_meas_peak),
                    "vel_peak": float(vel_peak),
                }
                res["results"].append(row)

                if (not aborted) and bool(moved_cmd_dir):
                    if sign > 0.0 and thr_plus is None:
                        thr_plus = float(level)
                    if sign < 0.0 and thr_minus is None:
                        thr_minus = float(level)
                if bool(aborted) and bool(runaway):
                    break
            if bool(runaway):
                break
            if (thr_plus is not None) and (thr_minus is not None):
                # We already have both directional thresholds; avoid stressing hardware.
                break

        res["threshold_plus_nm"] = thr_plus
        res["threshold_minus_nm"] = thr_minus
        res["runaway"] = bool(runaway)
        if bool(runaway):
            res["classification"] = "unstable_runaway"
        elif (thr_plus is not None) and (thr_minus is not None):
            lo = max(1e-6, min(float(thr_plus), float(thr_minus)))
            hi = max(float(thr_plus), float(thr_minus))
            res["asym_ratio"] = float(hi / lo)
            res["classification"] = "breakaway_measured"
            res["ok"] = True
        elif (thr_plus is None) and (thr_minus is None):
            res["classification"] = "no_breakaway_detected"
        else:
            res["classification"] = "one_direction_only_or_limit"
    finally:
        try:
            axis.controller.input_torque = 0.0
        except Exception:
            pass
        if prev_current_lim is not None:
            try:
                axis.motor.config.current_lim = float(prev_current_lim)
            except Exception:
                pass
        if prev_vel_limit is not None:
            try:
                axis.controller.config.vel_limit = float(prev_vel_limit)
            except Exception:
                pass
        if prev_control is not None:
            try:
                axis.controller.config.control_mode = int(prev_control)
            except Exception:
                pass
        if prev_input is not None:
            try:
                axis.controller.config.input_mode = int(prev_input)
            except Exception:
                pass
        if bool(leave_idle):
            try:
                axis.requested_state = AXIS_STATE_IDLE
            except Exception:
                pass

    if verbose:
        print(
            "directional_breakaway_scan:",
            f"classification={res.get('classification')}",
            f"plus={res.get('threshold_plus_nm')}",
            f"minus={res.get('threshold_minus_nm')}",
            f"ratio={res.get('asym_ratio')}",
            f"runaway={res.get('runaway')}",
        )

    return res


def tinymovr_style_validation_gate(
    axis=None,
    require_index=None,
    run_index_search_on_recover=True,
    current_lim=8.0,
    vel_limit=0.6,
    vel_probe_turns_s=0.08,
    vel_probe_s=0.18,
    vel_probe_settle_s=0.06,
    pos_probe_turns=0.01,
    pos_probe_timeout_s=2.2,
    pos_probe_settle_s=0.08,
    motion_eps_turns=8e-4,
    motion_eps_counts=2,
    breakaway_scan=True,
    breakaway_min_nm=0.01,
    breakaway_max_nm=0.14,
    breakaway_step_nm=0.01,
    breakaway_max_asym_ratio=4.0,
    verbose=True,
):
    """Strict staged gate inspired by TinyMovr bring-up order.

    Order:
      1) reference/closed-loop startup contract
      2) torque/current authority probe
      3) low-speed velocity sign probe
      4) tiny position step/hold probe
    """
    if axis is None:
        axis = get_axis0()

    out = {
        "ok": False,
        "classification": None,
        "error": None,
        "startup_contract": None,
        "authority_probe": None,
        "breakaway_probe": None,
        "velocity_probe": None,
        "position_probe": None,
    }

    if require_index is None:
        try:
            require_index = bool(getattr(axis.encoder.config, "use_index", False))
        except Exception:
            require_index = False

    # Stage 1: deterministic startup contract (no exploratory movement).
    startup = move_startup_contract(
        axis=axis,
        startup_mode="guarded",
        require_index=bool(require_index),
        run_index_search_on_recover=bool(run_index_search_on_recover),
        require_encoder_ready=True,
        timeout_s=3.0,
        sync_settle_s=0.05,
        stability_observe_s=0.25,
        stability_dt=0.02,
    )
    out["startup_contract"] = dict(startup or {})
    if not bool(dict(startup or {}).get("ok", False)):
        out["classification"] = "startup_contract_failed"
        out["error"] = str(dict(startup or {}).get("error", "startup contract failed"))
        return out

    # Stage 2: torque/current authority.
    tc = 0.04
    try:
        tc = float(getattr(axis.motor.config, "torque_constant", 0.04) or 0.04)
    except Exception:
        pass
    tq_span = max(0.0, float(current_lim) * max(1e-6, float(tc)))
    # Keep this stage intentionally low-stress: current-tracking/sign check only.
    tq1 = max(0.012, min(0.040, 0.07 * float(tq_span)))
    authority = torque_authority_ramp_probe(
        axis=axis,
        torque_targets_nm=(float(tq1), -float(tq1)),
        current_lim=max(0.2, float(current_lim)),
        vel_limit=max(0.05, float(vel_limit)),
        ramp_s=0.06,
        dwell_s=0.08,
        settle_s=0.05,
        dt=0.01,
        min_motion_turns=max(5e-4, float(motion_eps_turns)),
        min_motion_counts=max(1, int(motion_eps_counts)),
        iq_set_gate_a=0.25,
        iq_meas_gate_a=0.05,
        track_ratio_min=0.35,
        sign_match_min=0.65,
        vel_abort_turns_s=max(1.2, 2.0 * float(vel_limit)),
        leave_idle=False,
        collect_samples=False,
        verbose=bool(verbose),
    )
    out["authority_probe"] = dict(authority or {})
    a_cls = str(dict(authority or {}).get("classification", "unknown"))
    if a_cls in ("low_current_tracking", "current_sign_mismatch", "unstable_runaway"):
        out["classification"] = "authority_probe_failed"
        out["error"] = (
            "torque authority probe failed: "
            f"classification={a_cls}"
        )
        return out

    # Optional stage: directional breakaway asymmetry check.
    if bool(breakaway_scan):
        levels = []
        lv = float(max(0.001, float(breakaway_min_nm)))
        lv_max = float(max(lv, float(breakaway_max_nm)))
        lv_step = float(max(0.001, float(breakaway_step_nm)))
        while lv <= lv_max + 1e-9:
            levels.append(float(lv))
            lv += lv_step
        bscan = directional_breakaway_scan(
            axis=axis,
            current_lim=max(0.2, float(current_lim)),
            vel_limit=max(0.05, float(vel_limit)),
            torque_levels_nm=tuple(levels),
            pulse_s=0.18,
            settle_s=0.08,
            motion_eps_turns=max(8e-4, float(motion_eps_turns)),
            motion_eps_counts=max(1, int(motion_eps_counts)),
            vel_abort_turns_s=max(2.0, 4.0 * float(vel_limit)),
        )
        out["breakaway_probe"] = dict(bscan or {})
        if bool(dict(bscan or {}).get("runaway", False)):
            out["classification"] = "breakaway_probe_failed"
            out["error"] = f"breakaway scan detected runaway: {bscan}"
            return out
        asym = (dict(bscan or {}).get("asym_ratio", None))
        t_plus = (dict(bscan or {}).get("threshold_plus_nm", None))
        t_minus = (dict(bscan or {}).get("threshold_minus_nm", None))
        if (t_plus is None) or (t_minus is None):
            out["classification"] = "breakaway_probe_failed"
            out["error"] = f"breakaway scan incomplete: {bscan}"
            return out
        if (asym is not None) and (float(asym) > float(max(1.0, float(breakaway_max_asym_ratio)))):
            out["classification"] = "directional_breakaway_asymmetry"
            out["error"] = (
                "breakaway asymmetry too high: "
                f"plus={float(t_plus):.4f}Nm minus={float(t_minus):.4f}Nm ratio={float(asym):.2f}"
            )
            return out

    # Stage 3: low-speed velocity sign sanity.
    try:
        prev_control = int(getattr(axis.controller.config, "control_mode", CONTROL_MODE_POSITION_CONTROL))
    except Exception:
        prev_control = None
    try:
        prev_input = int(getattr(axis.controller.config, "input_mode", INPUT_MODE_PASSTHROUGH))
    except Exception:
        prev_input = None
    vel_ctrl_mode = int(globals().get("CONTROL_MODE_VELOCITY_CONTROL", 2))

    vprobe = {
        "cmd_vel_turns_s": float(vel_probe_turns_s),
        "plus": None,
        "minus": None,
        "levels": [],
        "threshold_plus_turns_s": None,
        "threshold_minus_turns_s": None,
        "sign_ok": False,
    }
    try:
        assert_no_errors(axis, label="tinymovr_gate/velocity/pre")
        if not ensure_closed_loop(axis, timeout_s=2.0, clear_first=False, pre_sync=True, retries=1):
            raise RuntimeError("failed to enter CLOSED_LOOP_CONTROL for velocity probe")
        axis.controller.config.control_mode = int(vel_ctrl_mode)
        axis.controller.config.input_mode = int(INPUT_MODE_PASSTHROUGH)
        try:
            axis.controller.config.vel_limit = max(0.05, float(vel_limit))
        except Exception:
            pass

        def _vel_step(v_cmd):
            p0 = float(getattr(axis.encoder, "pos_estimate", 0.0))
            q0 = int(getattr(axis.encoder, "shadow_count", 0))
            t_end = time.time() + max(0.05, float(vel_probe_s))
            while time.time() < t_end:
                axis.controller.input_vel = float(v_cmd)
                assert_no_errors(axis, label="tinymovr_gate/velocity")
                time.sleep(0.01)
            axis.controller.input_vel = 0.0
            time.sleep(max(0.01, float(vel_probe_settle_s)))
            p1 = float(getattr(axis.encoder, "pos_estimate", 0.0))
            q1 = int(getattr(axis.encoder, "shadow_count", 0))
            return {
                "cmd": float(v_cmd),
                "dp": float(p1 - p0),
                "dq": int(q1 - q0),
            }

        v_base = max(0.02, abs(float(vel_probe_turns_s)))
        v_cap = max(float(v_base), min(0.35, max(0.12, 0.55 * float(vel_limit))))
        v_levels = []
        v_cur = float(v_base)
        for _ in range(4):
            v_levels.append(float(v_cur))
            if float(v_cur) >= float(v_cap) - 1e-9:
                break
            v_cur = min(float(v_cap), max(float(v_cur) + 0.02, 1.6 * float(v_cur)))

        thr_plus = None
        thr_minus = None
        moved_plus_any = False
        moved_minus_any = False
        last_plus = None
        last_minus = None
        for v_abs in v_levels:
            vplus = _vel_step(+abs(float(v_abs)))
            vminus = _vel_step(-abs(float(v_abs)))
            last_plus = dict(vplus)
            last_minus = dict(vminus)
            moved_plus = bool(
                abs(float(vplus["dp"])) >= float(motion_eps_turns)
                or abs(int(vplus["dq"])) >= int(motion_eps_counts)
            )
            moved_minus = bool(
                abs(float(vminus["dp"])) >= float(motion_eps_turns)
                or abs(int(vminus["dq"])) >= int(motion_eps_counts)
            )
            plus_sign_ok = bool(moved_plus and (float(vplus["dp"]) > 0.0 or int(vplus["dq"]) > 0))
            minus_sign_ok = bool(moved_minus and (float(vminus["dp"]) < 0.0 or int(vminus["dq"]) < 0))
            moved_plus_any = bool(moved_plus_any or moved_plus)
            moved_minus_any = bool(moved_minus_any or moved_minus)
            if bool(plus_sign_ok) and thr_plus is None:
                thr_plus = float(v_abs)
            if bool(minus_sign_ok) and thr_minus is None:
                thr_minus = float(v_abs)
            vprobe["levels"].append(
                {
                    "cmd_abs": float(v_abs),
                    "plus": dict(vplus),
                    "minus": dict(vminus),
                    "moved_plus": bool(moved_plus),
                    "moved_minus": bool(moved_minus),
                    "plus_sign_ok": bool(plus_sign_ok),
                    "minus_sign_ok": bool(minus_sign_ok),
                }
            )
            if (thr_plus is not None) and (thr_minus is not None):
                break

        vprobe["plus"] = dict(last_plus or {})
        vprobe["minus"] = dict(last_minus or {})
        vprobe["threshold_plus_turns_s"] = None if thr_plus is None else float(thr_plus)
        vprobe["threshold_minus_turns_s"] = None if thr_minus is None else float(thr_minus)
        vprobe["moved_plus"] = bool(moved_plus_any)
        vprobe["moved_minus"] = bool(moved_minus_any)
        sign_ok = bool((thr_plus is not None) and (thr_minus is not None))
        vprobe["sign_ok"] = bool(sign_ok)
        out["velocity_probe"] = dict(vprobe)
        if not bool(sign_ok):
            out["classification"] = "velocity_probe_failed"
            out["error"] = f"velocity sign probe failed: {vprobe}"
            return out
    except Exception as exc:
        out["velocity_probe"] = dict(vprobe)
        out["classification"] = "velocity_probe_failed"
        out["error"] = f"velocity probe exception: {exc}"
        return out
    finally:
        try:
            axis.controller.input_vel = 0.0
        except Exception:
            pass
        if prev_control is not None:
            try:
                axis.controller.config.control_mode = int(prev_control)
            except Exception:
                pass
        if prev_input is not None:
            try:
                axis.controller.config.input_mode = int(prev_input)
            except Exception:
                pass

    # Stage 4: tiny position step and return.
    try:
        if not ensure_closed_loop(axis, timeout_s=2.0, clear_first=False, pre_sync=True, retries=1):
            out["classification"] = "position_probe_failed"
            out["error"] = "failed to enter CLOSED_LOOP_CONTROL before position probe"
            return out
        p0 = float(getattr(axis.encoder, "pos_estimate", 0.0))
        p1_target = float(p0 + abs(float(pos_probe_turns)))
        pos_plus = move_to_pos_strict(
            axis,
            float(p1_target),
            use_trap_traj=True,
            timeout_s=max(0.5, float(pos_probe_timeout_s)),
            min_delta_turns=max(3e-4, float(motion_eps_turns)),
            settle_s=max(0.0, float(pos_probe_settle_s)),
            vel_limit=max(0.05, float(vel_limit)),
            trap_vel=max(0.03, min(float(vel_limit), 0.12)),
            trap_acc=max(0.05, min(0.30, 2.5 * max(0.03, min(float(vel_limit), 0.12)))),
            trap_dec=max(0.05, min(0.30, 2.5 * max(0.03, min(float(vel_limit), 0.12)))),
            current_lim=max(0.2, float(current_lim)),
            pos_gain=10.0,
            vel_gain=0.18,
            vel_i_gain=0.0,
            require_target_reached=True,
            target_tolerance_turns=0.015,
            target_vel_tolerance_turns_s=0.20,
            abort_on_reverse_motion=True,
            reverse_motion_eps_turns=max(0.004, float(motion_eps_turns)),
            reverse_motion_confirm_samples=1,
            fail_to_idle=False,
        )
        pos_back = move_to_pos_strict(
            axis,
            float(p0),
            use_trap_traj=True,
            timeout_s=max(0.5, float(pos_probe_timeout_s)),
            min_delta_turns=max(3e-4, float(motion_eps_turns)),
            settle_s=max(0.0, float(pos_probe_settle_s)),
            vel_limit=max(0.05, float(vel_limit)),
            trap_vel=max(0.03, min(float(vel_limit), 0.12)),
            trap_acc=max(0.05, min(0.30, 2.5 * max(0.03, min(float(vel_limit), 0.12)))),
            trap_dec=max(0.05, min(0.30, 2.5 * max(0.03, min(float(vel_limit), 0.12)))),
            current_lim=max(0.2, float(current_lim)),
            pos_gain=10.0,
            vel_gain=0.18,
            vel_i_gain=0.0,
            require_target_reached=True,
            target_tolerance_turns=0.015,
            target_vel_tolerance_turns_s=0.20,
            abort_on_reverse_motion=True,
            reverse_motion_eps_turns=max(0.004, float(motion_eps_turns)),
            reverse_motion_confirm_samples=1,
            fail_to_idle=False,
        )
        out["position_probe"] = {
            "start_pos": float(p0),
            "plus": dict(pos_plus or {}),
            "back": dict(pos_back or {}),
        }
    except Exception as exc:
        out["classification"] = "position_probe_failed"
        out["error"] = f"position probe exception: {exc}"
        return out

    out["ok"] = True
    out["classification"] = "gate_ok"
    return out


def jump_vs_slip_diagnostic(
    axis=None,
    idle_watch_s=2.0,
    hold_watch_s=6.0,
    dt=0.02,
    hold_current_lim=9.0,
    hold_pos_gain=10.0,
    hold_vel_gain=0.18,
    hold_vel_i_gain=0.05,
    jump_counts_threshold=64,
    jump_turns_threshold=0.05,
    hold_err_warn_turns=0.20,
    iq_quiet_a=0.8,
):
    """Classify instability as likely encoder-signal jumps vs loaded control/mechanical runaway.

    Sequence (low stress):
    1) IDLE watch with no commanded motion.
    2) CLOSED_LOOP hold at current position (input_pos fixed).
    """
    if axis is None:
        axis = get_axis0()

    clear_errors_all(axis)
    force_idle(axis, settle_s=0.05)

    def _sample(label, duration_s, hold_target=None):
        c_prev = int(getattr(axis.encoder, "shadow_count", 0))
        p_prev = float(getattr(axis.encoder, "pos_estimate", 0.0))
        t0 = time.time()
        samples = 0
        max_abs_dq = 0
        max_abs_dp = 0.0
        max_abs_vel = 0.0
        max_abs_iq = 0.0
        max_abs_track_err = 0.0
        jump_events = []
        while (time.time() - t0) < float(duration_s):
            c = int(getattr(axis.encoder, "shadow_count", c_prev))
            p = float(getattr(axis.encoder, "pos_estimate", p_prev))
            v = float(getattr(axis.encoder, "vel_estimate", 0.0))
            iq = float(getattr(axis.motor.current_control, "Iq_measured", 0.0))
            dq = int(c - c_prev)
            dp = float(p - p_prev)
            max_abs_dq = max(max_abs_dq, abs(int(dq)))
            max_abs_dp = max(max_abs_dp, abs(float(dp)))
            max_abs_vel = max(max_abs_vel, abs(float(v)))
            max_abs_iq = max(max_abs_iq, abs(float(iq)))
            if hold_target is not None:
                te = float(hold_target) - float(p)
                max_abs_track_err = max(max_abs_track_err, abs(te))
            if (abs(int(dq)) >= int(jump_counts_threshold)) or (abs(float(dp)) >= float(jump_turns_threshold)):
                jump_events.append(
                    {
                        "t_s": float(time.time() - t0),
                        "dq": int(dq),
                        "dp_turns": float(dp),
                        "vel_turns_s": float(v),
                        "iq_a": float(iq),
                        "axis_err": int(getattr(axis, "error", 0)),
                        "motor_err": int(getattr(axis.motor, "error", 0)),
                        "enc_err": int(getattr(axis.encoder, "error", 0)),
                        "ctrl_err": int(getattr(axis.controller, "error", 0)),
                    }
                )
            c_prev = c
            p_prev = p
            samples += 1
            time.sleep(max(0.005, float(dt)))
        return {
            "label": str(label),
            "samples": int(samples),
            "max_abs_dq": int(max_abs_dq),
            "max_abs_dp_turns": float(max_abs_dp),
            "max_abs_vel_turns_s": float(max_abs_vel),
            "max_abs_iq_a": float(max_abs_iq),
            "max_abs_track_err_turns": float(max_abs_track_err),
            "jump_events": jump_events,
            "jump_count": int(len(jump_events)),
        }

    idle = _sample("idle", duration_s=float(idle_watch_s), hold_target=None)

    # Configure a gentle hold at current position.
    p_hold = float(getattr(axis.encoder, "pos_estimate", 0.0))
    try:
        axis.motor.config.current_lim = float(hold_current_lim)
    except Exception:
        pass
    try:
        axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        axis.controller.config.pos_gain = float(hold_pos_gain)
        axis.controller.config.vel_gain = float(hold_vel_gain)
        axis.controller.config.vel_integrator_gain = float(hold_vel_i_gain)
    except Exception:
        pass
    if not ensure_closed_loop(axis, timeout_s=2.0, clear_first=False, pre_sync=True, retries=2):
        raise RuntimeError("jump_vs_slip_diagnostic: failed to enter CLOSED_LOOP_CONTROL")
    try:
        axis.controller.input_pos = float(p_hold)
    except Exception:
        pass
    sync_pos_setpoint(axis, settle_s=0.05, retries=2, verbose=False)
    hold = _sample("hold", duration_s=float(hold_watch_s), hold_target=float(p_hold))

    # Classification logic.
    hold_jump_quiet = False
    if int(hold.get("jump_count", 0)) > 0:
        quiet_hits = 0
        for ev in list(hold.get("jump_events", [])):
            if abs(float(ev.get("iq_a", 0.0))) <= float(iq_quiet_a):
                quiet_hits += 1
        hold_jump_quiet = bool(quiet_hits >= max(1, int(hold.get("jump_count", 0)) // 2))

    if int(idle.get("jump_count", 0)) > 0:
        classification = "encoder_jump_in_idle"
        interpretation = "Encoder counts/position jumped while IDLE; signal/wiring/noise is most likely."
    elif hold_jump_quiet:
        classification = "encoder_like_jumps_under_hold"
        interpretation = "Large jumps occurred under hold with low current demand; this looks encoder-like."
    elif (
        float(hold.get("max_abs_track_err_turns", 0.0)) >= float(hold_err_warn_turns)
        and float(hold.get("max_abs_iq_a", 0.0)) >= max(1.0, 0.6 * float(hold_current_lim))
    ):
        classification = "runaway_or_slip_under_load"
        interpretation = "Large hold error with high current indicates real rotor/load runaway or loop instability."
    elif int(hold.get("jump_count", 0)) == 0 and float(hold.get("max_abs_track_err_turns", 0.0)) < 0.05:
        classification = "stable_hold"
        interpretation = "No jump events and hold error remained small."
    else:
        classification = "mixed_or_inconclusive"
        interpretation = "Behavior did not cleanly separate encoder jumps from loaded instability."

    out = {
        "ok": bool(classification == "stable_hold"),
        "classification": str(classification),
        "interpretation": str(interpretation),
        "idle": idle,
        "hold": hold,
        "config": {
            "idle_watch_s": float(idle_watch_s),
            "hold_watch_s": float(hold_watch_s),
            "dt": float(dt),
            "hold_current_lim": float(hold_current_lim),
            "hold_pos_gain": float(hold_pos_gain),
            "hold_vel_gain": float(hold_vel_gain),
            "hold_vel_i_gain": float(hold_vel_i_gain),
            "jump_counts_threshold": int(jump_counts_threshold),
            "jump_turns_threshold": float(jump_turns_threshold),
            "hold_err_warn_turns": float(hold_err_warn_turns),
            "iq_quiet_a": float(iq_quiet_a),
        },
    }

    print(
        "jump_vs_slip_diagnostic:",
        f"classification={out['classification']}",
        f"idle_jumps={idle['jump_count']}",
        f"hold_jumps={hold['jump_count']}",
        f"hold_err_max={hold['max_abs_track_err_turns']:.6f}t",
        f"hold_iq_max={hold['max_abs_iq_a']:.3f}A",
    )

    force_idle(axis, settle_s=0.05)
    return out


def hardware_sign_consistency_validation(
    axis=None,
    cycles=6,
    cmd_delta_turns=0.01,
    current_lim=8.0,
    pos_gain=16.0,
    vel_gain=0.24,
    vel_i_gain=0.0,
    vel_limit=0.6,
    settle_between_s=0.15,
    run_preflight=False,
    preflight_cpr=None,
    preflight_bandwidth=20.0,
    preflight_interp=True,
    run_jump_vs_slip=True,
    jump_hold_s=4.0,
    save_path=None,
    verbose=True,
):
    """Run repeated low-stress direction-sign probes and classify consistency.

    This is intended as a hardware/commutation sanity gate before aggressive tuning.
    It combines:
      1) repeated sign_and_breakaway_probe() runs
      2) optional preflight_encoder() check
      3) optional jump_vs_slip_diagnostic() for encoder-jump vs slip separation
    """
    import os
    import json
    import datetime

    if axis is None:
        axis = get_axis0()

    clear_errors_all(axis)
    force_idle(axis, settle_s=0.08)

    out = {
        "ts": datetime.datetime.now().isoformat(),
        "ok": False,
        "classification": None,
        "interpretation": None,
        "config": {
            "cycles": int(max(1, int(cycles))),
            "cmd_delta_turns": float(cmd_delta_turns),
            "current_lim": float(current_lim),
            "pos_gain": float(pos_gain),
            "vel_gain": float(vel_gain),
            "vel_i_gain": float(vel_i_gain),
            "vel_limit": float(vel_limit),
            "settle_between_s": float(settle_between_s),
            "run_preflight": bool(run_preflight),
            "run_jump_vs_slip": bool(run_jump_vs_slip),
            "jump_hold_s": float(jump_hold_s),
        },
        "start_snapshot": _snapshot_motion(axis),
        "preflight": None,
        "cycles": [],
        "summary": {},
        "jump_vs_slip": None,
        "end_snapshot": None,
        "save_path": None,
    }

    if bool(run_preflight):
        try:
            cpr_cfg = int(preflight_cpr) if preflight_cpr is not None else int(getattr(axis.encoder.config, "cpr", 1024))
            pre_ok = bool(
                preflight_encoder(
                    axis=axis,
                    cpr=int(cpr_cfg),
                    bandwidth=float(preflight_bandwidth),
                    interp=bool(preflight_interp),
                )
            )
            out["preflight"] = {
                "ok": bool(pre_ok),
                "error": None,
                "cpr": int(cpr_cfg),
                "bandwidth": float(preflight_bandwidth),
                "interp": bool(preflight_interp),
            }
        except Exception as exc:
            out["preflight"] = {
                "ok": False,
                "error": str(exc),
                "cpr": (None if preflight_cpr is None else int(preflight_cpr)),
                "bandwidth": float(preflight_bandwidth),
                "interp": bool(preflight_interp),
            }

    cls_counts = {}
    cycle_n = int(max(1, int(cycles)))
    for k in range(1, cycle_n + 1):
        clear_errors_all(axis)
        rec = {
            "index": int(k),
            "ok": False,
            "classification": None,
            "probe": None,
            "error": None,
        }
        try:
            probe = sign_and_breakaway_probe(
                axis=axis,
                cmd_delta_turns=float(cmd_delta_turns),
                current_lim=float(current_lim),
                pos_gain=float(pos_gain),
                vel_gain=float(vel_gain),
                vel_i_gain=float(vel_i_gain),
                vel_limit=float(vel_limit),
                leave_idle=True,
                verbose=False,
            )
            cls = str((dict(probe or {})).get("classification", "unknown"))
            rec["ok"] = True
            rec["classification"] = cls
            rec["probe"] = probe
            cls_counts[cls] = int(cls_counts.get(cls, 0)) + 1
            if verbose:
                try:
                    rows = list((dict(probe or {}).get("tests") or []))
                    plus = dict(rows[0] or {}) if rows else {}
                    minus = dict(rows[1] or {}) if len(rows) > 1 else {}
                    print(
                        f"hardware_sign_cycle[{k}/{cycle_n}]: cls={cls} "
                        f"+dp={float(plus.get('dp', 0.0)):+.6f} -dp={float(minus.get('dp', 0.0)):+.6f} "
                        f"+dq={int(plus.get('dq', 0)):+d} -dq={int(minus.get('dq', 0)):+d}"
                    )
                except Exception:
                    print(f"hardware_sign_cycle[{k}/{cycle_n}]: cls={cls}")
        except Exception as exc:
            rec["error"] = str(exc)
            if verbose:
                print(f"hardware_sign_cycle[{k}/{cycle_n}]: ERROR: {exc}")
        out["cycles"].append(rec)
        time.sleep(max(0.0, float(settle_between_s)))

    sign_ok_n = int(cls_counts.get("sign_ok", 0))
    sign_inverted_n = int(cls_counts.get("sign_inverted", 0))
    one_dir_n = int(cls_counts.get("one_direction_only_or_limit", 0))
    low_auth_n = int(cls_counts.get("low_authority_no_motion", 0))
    stuck_n = int(cls_counts.get("stuck_high_torque_no_motion", 0))
    bias_n = int(cls_counts.get("directional_bias_or_limit", 0))
    mixed_n = int(cls_counts.get("mixed_or_inconclusive", 0))
    err_n = int(sum(1 for r in list(out.get("cycles") or []) if (not bool(r.get("ok", False)))))

    if sign_inverted_n > 0:
        classification = "critical_sign_inversion"
        interpretation = "At least one probe reported sign inversion; this is unsafe for closed-loop use."
    elif sign_ok_n >= cycle_n and err_n == 0:
        classification = "stable_sign"
        interpretation = "Repeated probes were direction-consistent."
    elif (sign_ok_n > 0) and ((one_dir_n + low_auth_n + stuck_n + bias_n + mixed_n + err_n) > 0):
        classification = "intermittent_sign_or_authority"
        interpretation = "Some probes were sign-correct, but behavior is inconsistent across cycles."
    elif (one_dir_n + low_auth_n + stuck_n) >= max(1, cycle_n // 2):
        classification = "authority_limited_or_stiction_dominant"
        interpretation = "Most probes failed to move consistently; likely breakaway/authority limitation."
    else:
        classification = "mixed_or_inconclusive"
        interpretation = "Probe outcomes are mixed; root cause not isolated by sign checks alone."

    out["summary"] = {
        "cycles": int(cycle_n),
        "errors": int(err_n),
        "counts_by_classification": dict(cls_counts),
        "sign_ok": int(sign_ok_n),
        "sign_inverted": int(sign_inverted_n),
        "one_direction_only_or_limit": int(one_dir_n),
        "low_authority_no_motion": int(low_auth_n),
        "stuck_high_torque_no_motion": int(stuck_n),
        "directional_bias_or_limit": int(bias_n),
        "mixed_or_inconclusive": int(mixed_n),
    }
    out["classification"] = str(classification)
    out["interpretation"] = str(interpretation)

    if bool(run_jump_vs_slip):
        try:
            jres = jump_vs_slip_diagnostic(
                axis=axis,
                idle_watch_s=1.5,
                hold_watch_s=float(jump_hold_s),
                dt=0.02,
                hold_current_lim=max(6.0, float(current_lim)),
                hold_pos_gain=max(6.0, 0.75 * float(pos_gain)),
                hold_vel_gain=max(0.06, 1.00 * float(vel_gain)),
                hold_vel_i_gain=max(0.0, min(float(vel_i_gain), 0.05)),
            )
            out["jump_vs_slip"] = jres
            jcls = str((dict(jres or {})).get("classification", ""))
            if jcls in ("encoder_jump_in_idle", "encoder_like_jumps_under_hold"):
                out["classification"] = "encoder_signal_instability"
                out["interpretation"] = (
                    "Sign behavior is affected by encoder-like jumps/noise under idle/hold conditions."
                )
            elif jcls == "runaway_or_slip_under_load" and out["classification"] == "stable_sign":
                out["classification"] = "stable_sign_but_runaway_under_load"
                out["interpretation"] = (
                    "Direction sign is consistent, but hold behavior indicates load-side runaway/slip risk."
                )
        except Exception as exc:
            out["jump_vs_slip"] = {"ok": False, "error": str(exc)}

    out["ok"] = bool(out.get("classification") == "stable_sign")
    out["end_snapshot"] = _snapshot_motion(axis)

    if save_path is None:
        save_path = os.path.abspath(os.path.join("logs", "hardware_sign_consistency_latest.json"))
    if save_path:
        p = os.path.abspath(str(save_path))
        try:
            os.makedirs(os.path.dirname(p), exist_ok=True)
        except Exception:
            pass
        with open(p, "w") as f:
            json.dump(out, f, indent=2)
        out["save_path"] = p

    if verbose:
        print(
            "hardware_sign_consistency_validation:",
            f"classification={out.get('classification')}",
            f"ok={out.get('ok')}",
            f"sign_ok={sign_ok_n}/{cycle_n}",
            f"sign_inverted={sign_inverted_n}",
            f"one_dir={one_dir_n}",
            f"low_auth={low_auth_n}",
            f"errors={err_n}",
        )
        if out.get("save_path"):
            print("hardware_sign_consistency_validation: saved", out.get("save_path"))

    return out


def position_sign_probe(
    axis=None,
    step_turns=0.01,
    hold_s=0.20,
    dt=0.01,
    current_lim=3.0,
    pos_gain=6.0,
    vel_gain=0.14,
    vel_i_gain=0.0,
    vel_limit=0.25,
    verbose=True,
):
    """Low-motion sign diagnostic for POSITION/PASSTHROUGH loop.

    For each step direction (+/-), this probes whether commanded position error sign
    and initial Iq_set sign are consistent. This helps isolate loop-sign issues
    before larger motions.
    """
    if axis is None:
        axis = get_axis0()

    clear_errors_all(axis)
    force_idle(axis, settle_s=0.05)

    try:
        axis.motor.config.current_lim = float(current_lim)
    except Exception:
        pass
    try:
        axis.controller.config.vel_limit = float(vel_limit)
    except Exception:
        pass
    try:
        axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        axis.controller.config.pos_gain = float(pos_gain)
        axis.controller.config.vel_gain = float(vel_gain)
        axis.controller.config.vel_integrator_gain = float(vel_i_gain)
    except Exception:
        pass

    if not ensure_closed_loop(axis, timeout_s=2.0, clear_first=False, pre_sync=True, retries=2):
        raise RuntimeError("position_sign_probe: failed to enter CLOSED_LOOP_CONTROL")

    try:
        sync_pos_setpoint(axis, settle_s=0.05, retries=3, verbose=False)
    except Exception:
        pass

    out = {
        "ok": False,
        "classification": None,
        "config": {
            "step_turns": float(step_turns),
            "hold_s": float(hold_s),
            "dt": float(dt),
            "current_lim": float(current_lim),
            "pos_gain": float(pos_gain),
            "vel_gain": float(vel_gain),
            "vel_i_gain": float(vel_i_gain),
            "vel_limit": float(vel_limit),
        },
        "steps": [],
    }
    try:
        motor_direction = int(getattr(axis.motor.config, "direction", 1))
    except Exception:
        motor_direction = 1
    out["motor_direction"] = int(motor_direction)

    base = float(getattr(axis.encoder, "pos_estimate", 0.0))
    try:
        axis.controller.input_pos = float(base)
    except Exception:
        pass
    time.sleep(0.05)

    for sgn in (+1.0, -1.0):
        p0 = float(getattr(axis.encoder, "pos_estimate", 0.0))
        q0 = int(getattr(axis.encoder, "shadow_count", 0))
        tgt = float(p0 + (float(sgn) * abs(float(step_turns))))
        try:
            axis.controller.input_pos = float(tgt)
        except Exception:
            pass

        t_end = time.time() + max(0.05, float(hold_s))
        iq_set_peak_abs = 0.0
        iq_set_first = None
        iq_set_last = 0.0
        vel_peak_abs = 0.0
        while time.time() < t_end:
            assert_no_errors(axis, label="position_sign_probe")
            iq_set = float(getattr(axis.motor.current_control, "Iq_setpoint", 0.0))
            vel = float(getattr(axis.encoder, "vel_estimate", 0.0))
            if iq_set_first is None:
                iq_set_first = float(iq_set)
            iq_set_last = float(iq_set)
            iq_set_peak_abs = max(float(iq_set_peak_abs), abs(float(iq_set)))
            vel_peak_abs = max(float(vel_peak_abs), abs(float(vel)))
            time.sleep(max(0.005, float(dt)))

        p1 = float(getattr(axis.encoder, "pos_estimate", p0))
        q1 = int(getattr(axis.encoder, "shadow_count", q0))
        dp = float(p1 - p0)
        dq = int(q1 - q0)
        cmd_sign = 1 if float(sgn) > 0.0 else -1
        disp_sign = 1 if float(dp) > 0.0 else (-1 if float(dp) < 0.0 else 0)
        iq_first_sign = 1 if float(iq_set_first or 0.0) > 0.0 else (-1 if float(iq_set_first or 0.0) < 0.0 else 0)
        expected_iq_sign = int(cmd_sign) * int(motor_direction if motor_direction != 0 else 1)
        rec = {
            "cmd_sign": int(cmd_sign),
            "target": float(tgt),
            "start_pos": float(p0),
            "end_pos": float(p1),
            "dp": float(dp),
            "dq": int(dq),
            "disp_sign": int(disp_sign),
            "iq_set_first": (None if iq_set_first is None else float(iq_set_first)),
            "iq_set_last": float(iq_set_last),
            "iq_set_peak_abs": float(iq_set_peak_abs),
            "iq_set_first_sign": int(iq_first_sign),
            "iq_set_expected_sign": int(expected_iq_sign),
            "vel_peak_abs": float(vel_peak_abs),
            "sign_consistent": bool((disp_sign == 0) or (disp_sign == int(cmd_sign))),
            "iq_first_consistent": bool((iq_first_sign == 0) or (iq_first_sign == int(expected_iq_sign))),
        }
        out["steps"].append(rec)

        # Return to base between probes.
        try:
            axis.controller.input_pos = float(base)
        except Exception:
            pass
        time.sleep(0.08)

    bad_disp = [r for r in list(out.get("steps") or []) if not bool(r.get("sign_consistent", False))]
    bad_iq = [r for r in list(out.get("steps") or []) if not bool(r.get("iq_first_consistent", False))]
    low_motion = [r for r in list(out.get("steps") or []) if (abs(float(r.get("dp", 0.0))) < max(8e-4, 0.2 * abs(float(step_turns))))]

    if len(bad_disp) > 0:
        out["classification"] = "position_sign_inconsistent"
        out["ok"] = False
    elif len(bad_iq) > 0:
        # Even if displacement eventually follows the command, opposite-sign
        # initial control effort creates startup wiggle and can trigger runaway
        # under higher load. Treat as not-ready for production motion.
        out["classification"] = "position_sign_iq_mismatch"
        out["ok"] = False
    elif len(low_motion) == len(list(out.get("steps") or [])):
        out["classification"] = "low_authority_no_motion"
        out["ok"] = False
    else:
        out["classification"] = "position_sign_ok"
        out["ok"] = True

    try:
        force_idle(axis, settle_s=0.05)
    except Exception:
        pass

    if bool(verbose):
        print(
            "position_sign_probe:",
            f"classification={out.get('classification')}",
            f"ok={out.get('ok')}",
            f"steps={len(list(out.get('steps') or []))}",
        )
        for i, r in enumerate(list(out.get("steps") or []), start=1):
            print(
                f"  step[{i}] cmd_sign={int(r.get('cmd_sign', 0)):+d} "
                f"dp={float(r.get('dp', 0.0)):+.6f} dq={int(r.get('dq', 0)):+d} "
                f"iq_first={float(r.get('iq_set_first') or 0.0):+.3f} "
                f"disp_ok={bool(r.get('sign_consistent', False))} iq_ok={bool(r.get('iq_first_consistent', False))}"
            )
    return out


def _score_direction_contract_report(report):
    """Compute a comparable score for one direction candidate."""
    rep = dict(report or {})
    summary = dict(rep.get("summary") or {})
    cls = str(rep.get("classification") or "")
    sign_ok = int(summary.get("sign_ok", 0))
    sign_inverted = int(summary.get("sign_inverted", 0))
    one_dir = int(summary.get("one_direction_only_or_limit", 0))
    low_auth = int(summary.get("low_authority_no_motion", 0))
    stuck = int(summary.get("stuck_high_torque_no_motion", 0))
    bias = int(summary.get("directional_bias_or_limit", 0))
    mixed = int(summary.get("mixed_or_inconclusive", 0))
    errors = int(summary.get("errors", 0))
    cycles = int(summary.get("cycles", 0))
    wrong_dir_hits = 0
    for rec in list(rep.get("cycles") or []):
        if "WRONG_DIRECTION watchdog tripped" in str((dict(rec or {})).get("error") or ""):
            wrong_dir_hits += 1

    class_bias = {
        "stable_sign": 260.0,
        "stable_sign_but_runaway_under_load": 140.0,
        "intermittent_sign_or_authority": 70.0,
        "authority_limited_or_stiction_dominant": -100.0,
        "mixed_or_inconclusive": -130.0,
        "encoder_signal_instability": -260.0,
        "critical_sign_inversion": -500.0,
    }.get(cls, -180.0)

    score = float(class_bias)
    score += 95.0 * float(sign_ok)
    score -= 220.0 * float(sign_inverted)
    score -= 70.0 * float(wrong_dir_hits)
    score -= 22.0 * float(errors)
    score -= 7.0 * float(one_dir + low_auth + stuck + bias + mixed)

    # Prefer candidates that produced any useful evidence.
    if cycles > 0 and sign_ok == 0 and sign_inverted == 0:
        score -= 40.0
    return {
        "score": float(score),
        "wrong_direction_hits": int(wrong_dir_hits),
        "sign_ok": int(sign_ok),
        "sign_inverted": int(sign_inverted),
        "cycles": int(cycles),
        "classification": cls,
    }


def auto_direction_contract(
    axis=None,
    candidate_directions=(-1, 1),
    cycles=4,
    cmd_delta_turns=0.01,
    current_lim=8.0,
    pos_gain=16.0,
    vel_gain=0.24,
    vel_i_gain=0.0,
    vel_limit=0.6,
    settle_between_s=0.12,
    run_jump_vs_slip=False,
    jump_hold_s=3.0,
    persist=False,
    save_path=None,
    verbose=True,
):
    """Pick and apply the safest motor direction based on repeated sign checks.

    Runs `hardware_sign_consistency_validation()` for each candidate direction,
    scores each report, applies the best direction, and returns a structured result.
    """
    import os
    import json
    import datetime

    if axis is None:
        axis = get_axis0()

    # Normalize directions to {-1, +1} and preserve order.
    dirs = []
    seen = set()
    for d in list(candidate_directions or []):
        try:
            s = -1 if float(d) < 0.0 else 1
        except Exception:
            continue
        if s not in seen:
            seen.add(s)
            dirs.append(int(s))
    if len(dirs) == 0:
        dirs = [-1, 1]

    try:
        start_direction = int(getattr(axis.motor.config, "direction", 1))
    except Exception:
        start_direction = 1

    out = {
        "ts": datetime.datetime.now().isoformat(),
        "ok": False,
        "start_direction": int(start_direction),
        "selected_direction": int(start_direction),
        "confidence": 0.0,
        "persisted": False,
        "candidates": [],
        "winner": None,
        "summary": None,
        "save_path": None,
    }

    for idx, d in enumerate(dirs, start=1):
        rec = {
            "index": int(idx),
            "direction": int(d),
            "applied": False,
            "score": None,
            "score_breakdown": None,
            "report": None,
            "error": None,
        }
        try:
            clear_errors_all(axis, settle_s=0.06)
            force_idle(axis, settle_s=0.06)
            axis.motor.config.direction = int(d)
            rec["applied"] = True
            time.sleep(0.05)
            clear_errors_all(axis, settle_s=0.05)

            report = hardware_sign_consistency_validation(
                axis=axis,
                cycles=int(max(1, int(cycles))),
                cmd_delta_turns=float(cmd_delta_turns),
                current_lim=float(current_lim),
                pos_gain=float(pos_gain),
                vel_gain=float(vel_gain),
                vel_i_gain=float(vel_i_gain),
                vel_limit=float(vel_limit),
                settle_between_s=float(settle_between_s),
                run_preflight=False,
                run_jump_vs_slip=bool(run_jump_vs_slip),
                jump_hold_s=float(jump_hold_s),
                save_path=None,
                verbose=bool(verbose),
            )
            sb = _score_direction_contract_report(report)
            rec["report"] = report
            rec["score"] = float(sb["score"])
            rec["score_breakdown"] = sb
        except Exception as exc:
            rec["error"] = str(exc)
            rec["score"] = -1e9
        out["candidates"].append(rec)

    ranked = sorted(
        list(out["candidates"]),
        key=lambda r: float((dict(r or {})).get("score", -1e9)),
        reverse=True,
    )
    best = dict(ranked[0] if ranked else {})
    second = dict(ranked[1] if len(ranked) > 1 else {})

    selected = int(best.get("direction", start_direction))
    out["winner"] = best
    out["selected_direction"] = int(selected)

    # Confidence from relative score margin and winner class quality.
    best_score = float(best.get("score", -1e9))
    second_score = float(second.get("score", best_score - 1.0))
    margin = float(best_score - second_score)
    best_cls = str(((dict(best.get("score_breakdown") or {})).get("classification") or ""))
    base = 0.35 if best_cls == "stable_sign" else 0.18
    conf = base + max(0.0, min(0.55, margin / 350.0))
    out["confidence"] = float(max(0.0, min(0.99, conf)))

    # "ok" is strict: require stable sign on winner.
    best_rep = dict(best.get("report") or {})
    best_summary = dict(best_rep.get("summary") or {})
    out["summary"] = {
        "winner_classification": str(best_rep.get("classification")),
        "winner_sign_ok": int(best_summary.get("sign_ok", 0)),
        "winner_sign_inverted": int(best_summary.get("sign_inverted", 0)),
        "winner_cycles": int(best_summary.get("cycles", 0)),
        "runner_up_score_margin": float(margin),
    }
    out["ok"] = bool(str(best_rep.get("classification")) == "stable_sign")

    # Apply the selected direction regardless, then return strict ok/fail status.
    try:
        clear_errors_all(axis, settle_s=0.05)
        force_idle(axis, settle_s=0.05)
        axis.motor.config.direction = int(selected)
        clear_errors_all(axis, settle_s=0.05)
    except Exception as exc:
        out["ok"] = False
        out["summary"]["apply_error"] = str(exc)

    if bool(persist):
        try:
            get_odrv0().save_configuration()
            out["persisted"] = True
        except Exception as exc:
            out["persisted"] = False
            out["summary"]["persist_error"] = str(exc)

    if save_path is None:
        save_path = os.path.abspath(os.path.join("logs", "auto_direction_contract_latest.json"))
    if save_path:
        p = os.path.abspath(str(save_path))
        try:
            os.makedirs(os.path.dirname(p), exist_ok=True)
        except Exception:
            pass
        try:
            with open(p, "w") as f:
                json.dump(out, f, indent=2)
            out["save_path"] = p
        except Exception:
            pass

    if verbose:
        print(
            "auto_direction_contract:",
            f"selected={out['selected_direction']:+d}",
            f"ok={out['ok']}",
            f"confidence={out['confidence']:.2f}",
            f"classification={out['summary'].get('winner_classification')}",
            f"margin={out['summary'].get('runner_up_score_margin', 0.0):.1f}",
        )
        if out.get("save_path"):
            print("auto_direction_contract: saved", out.get("save_path"))

    return out


def confirm_count():
    """Manual rotate motor"""
    axis = get_axis0()
    axis.requested_state = AXIS_STATE_IDLE
    for _ in range(100):
        print("Shadow count: ", int(axis.encoder.shadow_count), " Estimate: ", axis.encoder.pos_estimate)
        time.sleep(0.1)
        
def rotate_1_rev():
    axis = get_axis0()
    axis.requested_state = AXIS_STATE_IDLE
    time.sleep(0.2)

    c0 = int(axis.encoder.shadow_count)
    input("Rotate EXACTLY 1 rev now, then press Enter...")
    c1 = int(axis.encoder.shadow_count)
    print("c0 =", c0, "c1 =", c1, "delta =", c1 - c0)

def force_idle(axis, settle_s=0.25):
    """Force IDLE and wait a short moment for state to settle."""
    try:
        axis.requested_state = AXIS_STATE_IDLE
    except Exception:
        pass
    time.sleep(float(settle_s))


def clear_errors(axis=None):
    """Backwards-compatible alias."""
    clear_errors_all(axis=axis)


def clear_errors_all(axis=None, settle_s=0.15):
    """Clear errors robustly across firmware/interface differences.

    Tries (in order): IDLE, device-level clear (if available), axis-level clear
    (if available), and finally a best-effort parent clear.

    This is designed to survive USB reconnects where the previous odrv0 object
    becomes stale/anonymous.
    """
    if axis is None:
        axis = get_axis0()

    # Go IDLE first (important for some latched faults)
    force_idle(axis, settle_s=settle_s)

    # Re-fetch current device object (important after reconnect)
    try:
        odrv = get_odrv0()
    except Exception:
        odrv = None

    # Device-level clear (not always present)
    if odrv is not None:
        try:
            if hasattr(odrv, "clear_errors"):
                odrv.clear_errors()
        except Exception:
            pass

    # Axis-level clear (not always present)
    try:
        if hasattr(axis, "clear_errors"):
            axis.clear_errors()
    except Exception:
        pass

    # Some builds expose a parent handle on axis
    try:
        parent = getattr(axis, "_parent", None)
        if parent is not None and hasattr(parent, "clear_errors"):
            parent.clear_errors()
    except Exception:
        pass

    time.sleep(float(settle_s))


def assert_no_errors(axis=None, label=""):
    """Raise with a compact report if any error bits are set."""
    if axis is None:
        axis = get_axis0()

    ax_e = int(getattr(axis, "error", 0))
    m_e  = int(getattr(axis.motor, "error", 0))
    e_e  = int(getattr(axis.encoder, "error", 0))
    c_e  = int(getattr(axis.controller, "error", 0))

    if ax_e or m_e or e_e or c_e:
        prefix = (label + ": ") if label else ""
        raise RuntimeError(prefix + f"axis_err={hex(ax_e)} motor_err={hex(m_e)} enc_err={hex(e_e)} ctrl_err={hex(c_e)}")


# --- Helper: ensure encoder.is_ready is True (run offset calibration if needed) ---
def ensure_encoder_ready(axis=None, attempt_calibration=True, timeout_s=30.0):
    """Ensure encoder.is_ready is True.

    For incremental A/B encoders, ODrive typically requires an encoder offset calibration
    to set `encoder.is_ready=True` before it will allow CLOSED_LOOP_CONTROL.

    If `attempt_calibration` is True and the encoder is not ready, this will:
      - clear errors
      - run AXIS_STATE_ENCODER_OFFSET_CALIBRATION
      - wait for IDLE/CLOSED_LOOP

    Returns True if encoder becomes ready, else False.
    """
    if axis is None:
        axis = get_axis0()

    try:
        if bool(getattr(axis.encoder, "is_ready", False)):
            return True
    except Exception:
        # If the attribute isn't available, don't block.
        return True

    if not attempt_calibration:
        return False

    # Encoder not ready: try an encoder offset calibration
    force_idle(axis)
    clear_errors_all(axis)

    try:
        axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    except Exception:
        return False

    # Wait for calibration to complete
    if not wait_idle(axis, timeout_s=float(timeout_s)):
        return False

    # If errors latched, bail
    ax_e = int(getattr(axis, "error", 0))
    enc_e = int(getattr(axis.encoder, "error", 0))
    if ax_e or enc_e:
        return False

    try:
        return bool(getattr(axis.encoder, "is_ready", False))
    except Exception:
        return True


def encoder_watch(axis=None, seconds=5.0, hz=25.0, min_counts=4):
    """Watch encoder counts/estimates while you rotate by hand.

    Useful to diagnose ENCODER_ERROR_NO_RESPONSE (no A/B edges seen).
    """
    if axis is None:
        axis = get_axis0()

    dt = 1.0 / float(hz)
    n = max(1, int(float(seconds) * float(hz)))

    # Prefer IDLE so we don't fight you while you rotate.
    force_idle(axis)
    time.sleep(0.1)

    c0 = int(getattr(axis.encoder, "shadow_count", 0))
    p0 = float(getattr(axis.encoder, "pos_estimate", 0.0))
    print(f"encoder_watch: start shadow_count={c0} pos_est={p0:.6f} enc_err={hex(int(getattr(axis.encoder,'error',0)))}")

    last_c = c0
    cmin = c0
    cmax = c0
    transitions = 0
    for i in range(n):
        c = int(getattr(axis.encoder, "shadow_count", 0))
        p = float(getattr(axis.encoder, "pos_estimate", 0.0))
        v = float(getattr(axis.encoder, "vel_estimate", 0.0))
        e = int(getattr(axis.encoder, "error", 0))
        if c != last_c:
            transitions += 1
        last_c = c
        if c < cmin:
            cmin = c
        if c > cmax:
            cmax = c
        print(f"{i:03d}: shadow_count={c:8d} pos_est={p:+.6f} vel_est={v:+.6f} enc_err={hex(e)}")
        time.sleep(dt)

    c1 = int(getattr(axis.encoder, "shadow_count", 0))
    delta = c1 - c0
    span = cmax - cmin
    try:
        min_counts_i = max(1, int(min_counts))
    except Exception:
        min_counts_i = 4

    moved = (abs(delta) >= min_counts_i) or (span >= min_counts_i)
    jitter_only = (not moved) and (transitions > 0)
    print(
        "encoder_watch: end "
        f"shadow_count={c1} delta={delta} span={span} transitions={transitions} "
        f"min_counts={min_counts_i} moved={moved} jitter_only={jitter_only}"
    )
    return moved


#
# --- New helper: actively provoke encoder counts via closed-loop wiggle ---
def encoder_watch_cmd(axis=None, step_deg=10.0, cycles=4, hold_s=0.20, settle_s=0.15, current_lim=None,
                      mode="pos", torque=0.05, verbose=True, torque_constant=None, ensure_ready=True,
                      pre_set_modes=True, force_idle_first=True,
                      set_input_filter_bw=None, set_vel_limit=None,
                      torque_mode_vel_limit=None,
                      preset=None,
                      # Safety / behavior controls (added)
                      recenter_each_cycle=False,
                      safe_max_delta_turns=None,
                      safe_max_counts=None,
                      safe_max_iq=None,
                      safe_abort=True,
                      safe_check_phase="all",
                      use_trap_traj=None,
                      refresh_axis=True,
                      # Stiction/breakaway helper (for harmonic drives / sticky gearboxes)
                      stiction_kick=False,
                      kick_step_deg=20.0,
                      kick_hold_s=0.20,
                      kick_tries=2,
                      min_move_counts=1,
                      min_move_turns=1e-4):
    """Actively provoke encoder counts without manual rotation.

    Use this when the motor cannot be rotated by hand (e.g. gearbox/harmonic drive).

    - mode="pos": enters CLOSED_LOOP (position passthrough) and applies +/- position steps.
      Logging prints the *commanded* position for each phase.
      In verbose mode, also prints controller internal setpoints (when available) and estimates expected Iq from pos_gain and torque_constant.
    - mode="torque": enters CLOSED_LOOP (torque passthrough) and applies +/- torque bumps.
      Logging prints the *commanded* torque for each phase.

    IMPORTANT DIAGNOSTIC NOTE:
    - In ODrive TORQUE_CONTROL, `controller.input_torque` is interpreted in **Nm** and is converted
      to a current setpoint using `axis.motor.config.torque_constant` (Nm/A). If torque_constant is
      unset/zero/wrong, torque commands can appear to do nothing (Iq_set ~ 0).

    Returns a dict:
      {
        "counts_moved": bool,
        "pos_moved": bool,
        "delta_counts": int,
        "delta_pos": float,
        "start": {"shadow_count": int, "pos_est": float, "enc_err": int},
        "end":   {"shadow_count": int, "pos_est": float, "enc_err": int},
        "diag":  {...},
      }

    Notes:
    - `counts_moved` indicates A/B edges were seen.
    - `pos_moved` indicates the estimator moved (motor likely moved), even if counts did not.
    """
    # IMPORTANT: In odrivetool/IPython it's very common to have a *stale* axis object after
    # a USB reconnect or board reset. Reading properties on a stale object can appear to
    # "hang" (no further prints) because the underlying RPC never returns.
    #
    # Therefore, by default we refresh the axis handle from the live odrv0 namespace.
    # If you truly want to use the passed-in axis object, call with refresh_axis=False.
    if axis is None or bool(refresh_axis):
        axis = get_axis0()
        if verbose:
            print("encoder_watch_cmd: refreshed axis handle from live odrv0.axis0", flush=True)

    # Unconditional banner so you can see *something* even if later logic returns early.
    # This also confirms you are running the updated function (reload issues are common in odrivetool/IPython).
    print(
        f"encoder_watch_cmd[{ENCODER_WATCH_CMD_VERSION}] invoked ({_whereami(encoder_watch_cmd)}) "
        f"mode={mode} preset={preset} use_trap_traj={use_trap_traj}",
        flush=True,
    )

    # Print current controller state (existing behavior)
    if verbose:
        print("encoder_watch_cmd: status snapshot...", flush=True)
    # Status snapshot can still block if the USB link is down; keep it best-effort.
    try:
        st(axis)
    except Exception as ex:
        print(f"encoder_watch_cmd: WARNING: st(axis) failed: {ex}", flush=True)

    # Optional: raise current limit for the duration of the test
    prev_current_lim = None
    if current_lim is not None:
        try:
            prev_current_lim = float(axis.motor.config.current_lim)
            axis.motor.config.current_lim = float(current_lim)
        except Exception:
            prev_current_lim = None

    # Optional: override torque_constant temporarily (useful for torque-mode sanity)
    prev_tc = None
    if torque_constant is not None:
        try:
            prev_tc = float(axis.motor.config.torque_constant)
            axis.motor.config.torque_constant = float(torque_constant)
        except Exception:
            prev_tc = None

    # Save previous controller modes and state
    prev_control = None
    prev_input = None
    prev_state = None

    # Save a few more config fields we might temporarily override
    prev_filter_bw = None
    prev_vel_limit = None
    prev_torque_mode_vel_limit = None

    try:
        prev_filter_bw = float(getattr(axis.controller.config, "input_filter_bandwidth", 0.0))
    except Exception:
        prev_filter_bw = None

    try:
        prev_vel_limit = float(getattr(axis.controller.config, "vel_limit", 0.0))
    except Exception:
        prev_vel_limit = None

    # Different firmwares name this differently; we support either if present.
    try:
        prev_torque_mode_vel_limit = getattr(axis.controller.config, "enable_torque_mode_vel_limit", None)
    except Exception:
        prev_torque_mode_vel_limit = None

    diag = {}

    # --- Presets (optional) ---
    # Presets are just convenient parameter bundles; explicit args still win.
    preset_name = (str(preset).strip().lower() if preset is not None else "")
    if preset_name:
        if preset_name in ("no_filter", "nofilter", "unfiltered", "fast_setpoint"):
            # Make short holds reflect in setpoints immediately
            if set_input_filter_bw is None:
                set_input_filter_bw = 20.0
            if set_vel_limit is None:
                set_vel_limit = 5.0
            # Keep the safe behavior by default
            force_idle_first = True
            pre_set_modes = True
        elif preset_name in ("sticky_gearbox", "stiction", "breakaway"):
            # Bias toward breaking stiction and ensuring authority
            if set_input_filter_bw is None:
                set_input_filter_bw = 10.0
            if set_vel_limit is None:
                set_vel_limit = 5.0
            if str(mode).lower().strip().startswith("pos"):
                if step_deg is None or float(step_deg) < 30.0:
                    step_deg = 30.0
                if hold_s is None or float(hold_s) < 0.35:
                    hold_s = 0.35
            else:
                if torque is None or float(torque) < 0.25:
                    torque = 0.25
                if hold_s is None or float(hold_s) < 0.25:
                    hold_s = 0.25
            force_idle_first = True
            pre_set_modes = True
        elif preset_name in ("safe_probe", "probe", "safe"):
            # Safer diagnostic: small steps, recenters each cycle, and aborts if it runs away.
            if set_input_filter_bw is None:
                set_input_filter_bw = 8.0
            if set_vel_limit is None:
                set_vel_limit = 3.0
            if str(mode).lower().strip().startswith("pos"):
                if step_deg is None or float(step_deg) > 10.0:
                    step_deg = 10.0
                if hold_s is None or float(hold_s) < 0.25:
                    hold_s = 0.25
                if cycles is None or int(cycles) > 3:
                    cycles = 3
            else:
                if torque is None or float(torque) > 0.25:
                    torque = 0.25
                if hold_s is None or float(hold_s) < 0.25:
                    hold_s = 0.25
                if cycles is None or int(cycles) > 4:
                    cycles = 4

            # Safety thresholds (reasonable defaults)
            if safe_max_delta_turns is None:
                safe_max_delta_turns = 0.50  # abort if estimator moves more than this from recentered base
            if safe_max_counts is None:
                safe_max_counts = 8000       # abort if counts jump excessively vs start
            if safe_max_iq is None:
                safe_max_iq = 0.8 * float(current_lim) if current_lim is not None else 8.0

            safe_check_phase = "settle"
            recenter_each_cycle = True
            force_idle_first = True
            pre_set_modes = True
        elif preset_name in ("cnc_sync", "cnc", "sync", "traj_sync"):
            # add these inside cnc_sync preset
            diag.setdefault("preset_overrides", {})
            diag["preset_overrides"]["pos_gain"] = 60.0
            diag["preset_overrides"]["vel_gain"] = 0.50
            diag["preset_overrides"]["vel_integrator_gain"] = 1.00
            # CNC/3D-printer-like small coordinated moves: use TRAP_TRAJ and evaluate safety on settle (final position matters).
            if use_trap_traj is None:
                use_trap_traj = True
            if set_input_filter_bw is None:
                set_input_filter_bw = 0.0
            if set_vel_limit is None:
                set_vel_limit = 2.0
            if step_deg is None or float(step_deg) > 3.0:
                step_deg = 3.0
            if hold_s is None or float(hold_s) < 0.30:
                hold_s = 0.30
            if cycles is None or int(cycles) > 6:
                cycles = 6

            # Apply gain overrides if present
            try:
                if "preset_overrides" in diag and "pos_gain" in diag["preset_overrides"]:
                    axis.controller.config.pos_gain = float(diag["preset_overrides"]["pos_gain"])
                    axis.controller.config.vel_gain = float(diag["preset_overrides"]["vel_gain"])
                    axis.controller.config.vel_integrator_gain = float(diag["preset_overrides"]["vel_integrator_gain"])
            except Exception:
                pass

            if safe_max_delta_turns is None:
                safe_max_delta_turns = 1.0
            if safe_max_counts is None:
                safe_max_counts = 12000
            if safe_max_iq is None:
                safe_max_iq = 0.8 * float(current_lim) if current_lim is not None else 8.0

            safe_check_phase = "settle"
            recenter_each_cycle = True
            force_idle_first = True
            pre_set_modes = True
            # TRAP_TRAJ limits for coordinated motion (keep non-zero)
            diag.setdefault("traj", {})
            diag["traj"].setdefault("vel", 1.0)
            diag["traj"].setdefault("accel", 10.0)
            diag["traj"].setdefault("decel", 10.0)

            # Default: enable a small breakaway kick for sticky gearboxes
            stiction_kick = True
            if kick_step_deg is None or float(kick_step_deg) < 10.0:
                kick_step_deg = 20.0
            if kick_hold_s is None or float(kick_hold_s) < 0.15:
                kick_hold_s = 0.20
            if kick_tries is None or int(kick_tries) < 1:
                kick_tries = 2

    # Track whether we actually overrode these so we can restore correctly
    _overrode_filter_bw = False
    _overrode_vel_limit = False
    _overrode_torque_mode_vel_limit = False

    try:
        # Start clean and in a known state
        if force_idle_first:
            force_idle(axis, settle_s=0.15)
        clear_errors_all(axis)

        # Optionally apply limits that affect whether setpoints/steps actually move
        if set_input_filter_bw is not None:
            try:
                axis.controller.config.input_filter_bandwidth = float(set_input_filter_bw)
                _overrode_filter_bw = True
            except Exception:
                pass

        if set_vel_limit is not None:
            try:
                axis.controller.config.vel_limit = float(set_vel_limit)
                _overrode_vel_limit = True
            except Exception:
                pass

        if torque_mode_vel_limit is not None:
            # Some firmwares gate torque-mode behavior behind this flag
            try:
                if hasattr(axis.controller.config, "enable_torque_mode_vel_limit"):
                    axis.controller.config.enable_torque_mode_vel_limit = torque_mode_vel_limit
                    _overrode_torque_mode_vel_limit = True
            except Exception:
                pass
        # ODrive may refuse CLOSED_LOOP if encoder.is_ready is False.
        if ensure_ready:
            if not ensure_encoder_ready(axis, attempt_calibration=True, timeout_s=30.0):
                s = st(axis)
                try:
                    enc_ready = bool(getattr(axis.encoder, "is_ready", False))
                except Exception:
                    enc_ready = None
                raise RuntimeError(
                    "Encoder is not ready (encoder.is_ready=False); ODrive will not enter CLOSED_LOOP_CONTROL. "
                    "Tried encoder offset calibration but it did not make the encoder ready. "
                    f"state={s['state']} axis_err={hex(int(s['axis_err']))} enc_err={hex(int(s['enc_err']))} encoder.is_ready={enc_ready}. "
                    "This usually indicates ABI wiring/power/pin mapping issues, MT6701 ABI not enabled, or insufficient motion during offset calibration."
                )

        try:
            prev_control = axis.controller.config.control_mode
            prev_input = axis.controller.config.input_mode
        except Exception:
            prev_control = None
            prev_input = None

        try:
            prev_state = int(getattr(axis, "current_state", 0))
        except Exception:
            prev_state = None

        # Enter closed loop safely.
        # Important: pre-set controller modes BEFORE requesting CLOSED_LOOP on some firmwares.
        mode_l = str(mode).lower().strip()
        if pre_set_modes:
            try:
                if mode_l.startswith("tor"):
                    axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
                    axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
                else:
                    axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
                    # Respect trap trajectory request; otherwise use passthrough.
                    if bool(use_trap_traj):
                        axis.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
                    else:
                        axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
            except Exception:
                pass

        # Sync setpoints to current estimate so we don't cause a jump
        try:
            axis.controller.input_pos = axis.encoder.pos_estimate
        except Exception:
            pass
        try:
            axis.controller.input_torque = 0.0
        except Exception:
            pass

        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        # After entering closed-loop, ensure the controller setpoints are synced.
        try:
            sync_pos_setpoint(axis, settle_s=0.05, retries=2, verbose=False)
        except Exception:
            pass

        # Wait for CLOSED_LOOP, otherwise we won't get any authority.
        # Some faults (or prior error latch) can prevent entry even after a clear.
        if not wait_state(axis, AXIS_STATE_CLOSED_LOOP_CONTROL, timeout_s=2.0, poll_s=0.02):
            # Retry once: go IDLE, clear again, re-sync setpoint, and request CLOSED_LOOP again.
            try:
                force_idle(axis, settle_s=0.10)
            except Exception:
                pass
            try:
                clear_errors_all(axis)
            except Exception:
                pass
            try:
                axis.controller.input_pos = axis.encoder.pos_estimate
            except Exception:
                pass
            try:
                axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            except Exception:
                pass

            if not wait_state(axis, AXIS_STATE_CLOSED_LOOP_CONTROL, timeout_s=2.0, poll_s=0.02):
                s = st(axis)
                # Include readiness flags to make diagnosis faster.
                try:
                    motor_cal = bool(getattr(axis.motor, "is_calibrated", False))
                except Exception:
                    motor_cal = None
                try:
                    enc_ready = bool(getattr(axis.encoder, "is_ready", False))
                except Exception:
                    enc_ready = None
                try:
                    enc_mode = int(getattr(axis.encoder.config, "mode", -1))
                except Exception:
                    enc_mode = None

                raise RuntimeError(
                    "Failed to enter CLOSED_LOOP_CONTROL before encoder_watch_cmd. "
                    f"state={s['state']} axis_err={hex(int(s['axis_err']))} motor_err={hex(int(s['motor_err']))} "
                    f"enc_err={hex(int(s['enc_err']))} ctrl_err={hex(int(s['ctrl_err']))} "
                    f"motor.is_calibrated={motor_cal} encoder.is_ready={enc_ready} enc_mode={enc_mode}. "
                    "Hint: If axis_err is 0x1, the axis may be latched in an invalid-state fault; try `clear_errors_all()`, "
                    "ensure motor calibration + encoder config are valid, then re-run. If it persists after a USB reconnect/reset, "
                    "suspect encoder NO_RESPONSE / power / pin-mapping issues that block closed-loop entry."
                )

        time.sleep(0.10)

        # Snapshot start
        c_start = int(getattr(axis.encoder, "shadow_count", 0))
        p_start = float(getattr(axis.encoder, "pos_estimate", 0.0))
        e_start = int(getattr(axis.encoder, "error", 0))

        # Diagnostics: controller state and torque_constant
        try:
            tc = float(getattr(axis.motor.config, "torque_constant", 0.0))
        except Exception:
            tc = None

        try:
            tvel_lim = getattr(axis.controller.config, "enable_torque_mode_vel_limit", None)
        except Exception:
            tvel_lim = None

        diag.update({
            "tc": tc,
            "enable_torque_mode_vel_limit": tvel_lim,
            "current_lim": float(getattr(axis.motor.config, "current_lim", 0.0)),
            "state": int(getattr(axis, "current_state", 0)),
        })

        if verbose:
            print(f"encoder_watch_cmd: start mode={mode} shadow_count={c_start} pos_est={p_start:+.6f} enc_err={hex(e_start)}")
            print(f"  diag: torque_constant={tc} Nm/A, current_lim={diag['current_lim']}, torque_vel_limiter={tvel_lim}")
            if preset_name:
                print(f"  preset: {preset_name}")
            if set_input_filter_bw is not None or set_vel_limit is not None or torque_mode_vel_limit is not None:
                try:
                    cm = int(getattr(axis.controller.config, "control_mode", -1))
                    im = int(getattr(axis.controller.config, "input_mode", -1))
                except Exception:
                    cm, im = -1, -1
                print(
                    f"  overrides: input_filter_bw={set_input_filter_bw} vel_limit={set_vel_limit} "
                    f"torque_mode_vel_limit={torque_mode_vel_limit} use_trap_traj={use_trap_traj} "
                    f"ctrl_mode={cm} input_mode={im}",
                    flush=True,
                )

        counts_moved = False
        pos_moved = False

        aborted = False
        diag.setdefault("pos_cmd_latch_failures", 0)
        diag.setdefault("pos_cmd_read_failures", 0)

        def _maybe_abort(reason, extra=None):
            nonlocal aborted
            aborted = True
            diag["aborted"] = True
            diag["abort_reason"] = str(reason)
            if extra is not None:
                diag["abort_extra"] = extra
            if verbose:
                print(f"  ABORT: {reason}" + (f" | {extra}" if extra is not None else ""))
            try:
                # Bring setpoints back to current estimate to reduce further motion
                axis.controller.input_pos = float(getattr(axis.encoder, "pos_estimate", 0.0))
            except Exception:
                pass
            try:
                axis.controller.input_torque = 0.0
            except Exception:
                pass
            if safe_abort:
                try:
                    force_idle(axis, settle_s=0.10)
                except Exception:
                    pass

        def _should_safety_check(label: str) -> bool:
            # Safety policy: sometimes only the settled/final position matters (e.g. CNC/3D-printer sync moves).
            pol = (str(safe_check_phase).strip().lower() if safe_check_phase is not None else "all")
            if pol in ("all", "always", "every"):
                return True
            if pol in ("settle", "settle_only", "final", "final_only"):
                return ("settle" in str(label))
            return True

        def _safety_check(base_turns, c_start_local, label=""):
            if aborted:
                return
            if not _should_safety_check(label):
                return
            # Counts runaway
            if safe_max_counts is not None:
                try:
                    c_now = int(getattr(axis.encoder, "shadow_count", 0))
                    if abs(c_now - int(c_start_local)) > int(safe_max_counts):
                        _maybe_abort("shadow_count runaway", f"{label} Δc={c_now-int(c_start_local):+d} limit={int(safe_max_counts)}")
                        return
                except Exception:
                    pass
            # Estimator runaway
            if safe_max_delta_turns is not None:
                try:
                    p_now = float(getattr(axis.encoder, "pos_estimate", 0.0))
                    if abs(p_now - float(base_turns)) > float(safe_max_delta_turns):
                        _maybe_abort("pos_est runaway", f"{label} Δp={p_now-float(base_turns):+.6f}t limit={float(safe_max_delta_turns):.6f}t")
                        return
                except Exception:
                    pass
            # Current runaway
            if safe_max_iq is not None:
                try:
                    iqm = float(getattr(axis.motor.current_control, "Iq_measured", 0.0))
                    iqs = float(getattr(axis.motor.current_control, "Iq_setpoint", 0.0))
                    if max(abs(iqm), abs(iqs)) > float(safe_max_iq):
                        _maybe_abort("Iq runaway", f"{label} Iq_set={iqs:+.3f} Iq_meas={iqm:+.3f} limit={float(safe_max_iq):.3f}")
                        return
                except Exception:
                    pass

        if mode_l.startswith("tor"):
            # Torque mode pulse train
            axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
            axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH

            # Helpful warning if torque_constant is invalid
            if tc is None or tc <= 1e-6:
                if verbose:
                    print(
                        "  WARNING: axis.motor.config.torque_constant appears unset/zero. "
                        "Torque commands may translate to ~0 current (Iq_set ~ 0). "
                        "Set torque_constant (Nm/A) ~ 8.23/KV. (Do NOT set it to 1 unless you intentionally want 1 Nm/A, which is usually wrong.)"
                    )

            tq = float(torque)
            for i in range(int(cycles)):
                # + torque phase
                axis.controller.input_torque = tq
                time.sleep(0.01)
                rb_tq_p = getattr(axis.controller, "input_torque", None)
                time.sleep(float(hold_s))

                c_p = int(getattr(axis.encoder, "shadow_count", 0))
                p_p = float(getattr(axis.encoder, "pos_estimate", 0.0))
                ee_p = int(getattr(axis.encoder, "error", 0))

                _safety_check(base_turns=p_start, c_start_local=c_start, label=f"cycle{i+1:02d}-+tq")
                if aborted:
                    break

                # - torque phase
                axis.controller.input_torque = -tq
                time.sleep(0.01)
                rb_tq_n = getattr(axis.controller, "input_torque", None)
                time.sleep(float(hold_s))

                c_n = int(getattr(axis.encoder, "shadow_count", 0))
                p_n = float(getattr(axis.encoder, "pos_estimate", 0.0))
                ee_n = int(getattr(axis.encoder, "error", 0))

                _safety_check(base_turns=p_start, c_start_local=c_start, label=f"cycle{i+1:02d}--tq")
                if aborted:
                    break

                # settle at 0
                axis.controller.input_torque = 0.0
                time.sleep(float(settle_s))

                c = int(getattr(axis.encoder, "shadow_count", 0))
                p = float(getattr(axis.encoder, "pos_estimate", 0.0))

                _safety_check(base_turns=p_start, c_start_local=c_start, label=f"cycle{i+1:02d}-settle")
                if aborted:
                    break

                if c != c_start or c_p != c_start or c_n != c_start:
                    counts_moved = True
                if (abs(p - p_start) > 1e-4) or (abs(p_p - p_start) > 1e-4) or (abs(p_n - p_start) > 1e-4):
                    pos_moved = True

                if verbose:
                    iqm = float(getattr(axis.motor.current_control, "Iq_measured", 0.0))
                    iqs = float(getattr(axis.motor.current_control, "Iq_setpoint", 0.0))
                    print(
                        f"  {i+1:02d}/{int(cycles):02d}: +tq shadow_count={c_p:8d} pos_est={p_p:+.6f} Δc={c_p-c_start:+d} Δp={p_p-p_start:+.6f} cmd_tq={tq:+.3f}Nm rb_tq={rb_tq_p} enc_err={hex(ee_p)}"
                    )
                    print(
                        f"              -tq shadow_count={c_n:8d} pos_est={p_n:+.6f} Δc={c_n-c_start:+d} Δp={p_n-p_start:+.6f} cmd_tq={-tq:+.3f}Nm rb_tq={rb_tq_n} enc_err={hex(ee_n)}"
                    )
                    print(
                        f"            settle shadow_count={c:8d} pos_est={p:+.6f} Δc={c-c_start:+d} Δp={p-p_start:+.6f} cmd_tq={0.0:+.3f}Nm Iq_set={iqs:+.3f} Iq_meas={iqm:+.3f}"
                    )

        else:
            # Position wiggle (PASSTHROUGH or TRAP_TRAJ)
            axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
            axis.controller.config.input_mode = (INPUT_MODE_TRAP_TRAJ if bool(use_trap_traj) else INPUT_MODE_PASSTHROUGH)

            # Verify the mode write actually stuck (stale handles sometimes ignore writes)
            try:
                _cm_now = int(getattr(axis.controller.config, "control_mode", -1))
                _im_now = int(getattr(axis.controller.config, "input_mode", -1))
                diag["ctrl_mode_after"] = _cm_now
                diag["input_mode_after"] = _im_now
                if verbose:
                    print(f"  mode_after: control_mode={_cm_now} input_mode={_im_now}", flush=True)
            except Exception as ex:
                diag["mode_after_read_error"] = str(ex)

            # If using TRAP_TRAJ, make sure the trap limits are set (some firmwares retain old values).
            if bool(use_trap_traj):
                try:
                    tv = float(diag.get("traj", {}).get("vel", axis.trap_traj.config.vel_limit))
                    ta = float(diag.get("traj", {}).get("accel", axis.trap_traj.config.accel_limit))
                    td = float(diag.get("traj", {}).get("decel", axis.trap_traj.config.decel_limit))
                    axis.trap_traj.config.vel_limit = tv
                    axis.trap_traj.config.accel_limit = ta
                    axis.trap_traj.config.decel_limit = td
                except Exception:
                    pass
            # Helpful diagnostics: estimate expected velocity contribution from position loop
            try:
                pos_gain = float(getattr(axis.controller.config, "pos_gain", 0.0))
            except Exception:
                pos_gain = None
            try:
                vel_gain = float(getattr(axis.controller.config, "vel_gain", 0.0))
            except Exception:
                vel_gain = None

            def _ctrl_sp():
                """Best-effort fetch of controller internal setpoints (varies by FW)."""
                out = {}
                for k in ("pos_setpoint", "vel_setpoint", "torque_setpoint", "input_pos"):
                    try:
                        out[k] = float(getattr(axis.controller, k))
                    except Exception:
                        out[k] = None
                return out

            step_turns = float(step_deg) / 360.0
            base = float(getattr(axis.encoder, "pos_estimate", 0.0))

            def _set_input_pos(cmd_turns: float):
                """Set a position command and verify it actually latched (strict).

                Returns:
                    (latched: bool, rb_input_pos: float|None, rb_pos_setpoint: float|None)

                - For TRAP_TRAJ, prefer move_to_pos() when available.
                - If we cannot read back input_pos or pos_setpoint, treat as NOT latched.
                """
                target = float(cmd_turns)

                def rb_input():
                    try:
                        return float(getattr(axis.controller, "input_pos"))
                    except Exception:
                        return None

                def rb_sp():
                    try:
                        return float(getattr(axis.controller, "pos_setpoint"))
                    except Exception:
                        return None

                try:
                    in_mode = int(getattr(axis.controller.config, "input_mode", -1))
                except Exception:
                    in_mode = -1

                used_move_to = False
                if bool(use_trap_traj) and hasattr(axis.controller, "move_to_pos"):
                    try:
                        axis.controller.move_to_pos(target)
                        used_move_to = True
                    except Exception:
                        used_move_to = False

                if not used_move_to:
                    try:
                        axis.controller.input_pos = target
                    except Exception:
                        pass

                time.sleep(0.01)
                rbi = rb_input()
                rsp = rb_sp()

                # If we can't read back, that's *not* a successful latch
                if (rbi is None) or (rsp is None):
                    diag["pos_cmd_read_failures"] += 1
                    return False, rbi, rsp

                # Strict latch criteria: BOTH input_pos and pos_setpoint must be coherent.
                tol_input = 2e-4   # tolerate FW quantization/rounding
                tol_sp = 5e-4

                latched_input = abs(rbi - target) <= tol_input

                # For PASSTHROUGH: pos_setpoint should quickly match target
                if in_mode == int(INPUT_MODE_PASSTHROUGH):
                    latched_sp = abs(rsp - target) <= tol_sp
                else:
                    # For TRAP_TRAJ: pos_setpoint may lag target, but must at least be finite
                    latched_sp = (rsp is not None)

                latched = bool(latched_input and latched_sp)
                return latched, rbi, rsp

            # Track latch quality (some firmwares ignore input_pos writes unless using move_to_pos)
            latched_0, _, _ = _set_input_pos(base)
            _latch_failures = 0 if bool(latched_0) else 1
            time.sleep(0.05)

            for i in range(int(cycles)):
                if aborted:
                    break

                # Optional: recenter each cycle to avoid massive accumulated error if motion occurred
                if recenter_each_cycle:
                    try:
                        base = float(getattr(axis.encoder, "pos_estimate", 0.0))
                        _set_input_pos(base)
                        time.sleep(0.03)
                    except Exception:
                        pass

                # Safety check at start of cycle
                _safety_check(base_turns=base, c_start_local=c_start, label=f"cycle{i+1:02d}-start")
                if aborted:
                    break

                # + step phase
                cmd_p = base + step_turns
                latched_p, rb_cmd_p, rb_sp_p = _set_input_pos(cmd_p)
                if not latched_p:
                    diag["pos_cmd_latch_failures"] += 1
                    if verbose:
                        print(
                            f"    WARN: input_pos did not latch (+pos) (target={cmd_p:+.6f}t rb_input={rb_cmd_p} rb_sp={rb_sp_p})",
                            flush=True,
                        )

                time.sleep(float(hold_s))
                c_p = int(getattr(axis.encoder, "shadow_count", 0))
                p_p = float(getattr(axis.encoder, "pos_estimate", 0.0))
                ee_p = int(getattr(axis.encoder, "error", 0))
                # In TRAP_TRAJ, the controller chases pos_setpoint (which may lag input_pos).
                try:
                    sp_pos = float(getattr(axis.controller, "pos_setpoint"))
                except Exception:
                    sp_pos = float(rb_cmd_p if rb_cmd_p is not None else cmd_p)
                err_sp = sp_pos - p_p

                _safety_check(base_turns=base, c_start_local=c_start, label=f"cycle{i+1:02d}-+pos")
                if aborted:
                    break

                # - step phase
                cmd_n = base - step_turns
                latched_n, rb_cmd_n, rb_sp_n = _set_input_pos(cmd_n)
                if not latched_n:
                    diag["pos_cmd_latch_failures"] += 1
                    if verbose:
                        print(
                            f"    WARN: input_pos did not latch (-pos) (target={cmd_n:+.6f}t rb_input={rb_cmd_n} rb_sp={rb_sp_n})",
                            flush=True,
                        )
                    
                time.sleep(float(hold_s))
                c_n = int(getattr(axis.encoder, "shadow_count", 0))
                p_n = float(getattr(axis.encoder, "pos_estimate", 0.0))
                ee_n = int(getattr(axis.encoder, "error", 0))
                try:
                    sp_pos2 = float(getattr(axis.controller, "pos_setpoint"))
                except Exception:
                    sp_pos2 = float(rb_cmd_n if rb_cmd_n is not None else cmd_n)
                err_sp2 = sp_pos2 - p_n

                _safety_check(base_turns=base, c_start_local=c_start, label=f"cycle{i+1:02d}--pos")
                if aborted:
                    break

                # back to base / settle
                latched_b, rb_cmd_b, rb_sp_b = _set_input_pos(base)
                if not bool(latched_b):
                    _latch_failures += 1
                time.sleep(float(settle_s))

                c = int(getattr(axis.encoder, "shadow_count", 0))
                p = float(getattr(axis.encoder, "pos_estimate", 0.0))

                _safety_check(base_turns=base, c_start_local=c_start, label=f"cycle{i+1:02d}-settle")
                if aborted:
                    break

                if c != c_start or c_p != c_start or c_n != c_start:
                    counts_moved = True
                if (abs(p - p_start) > 1e-4) or (abs(p_p - p_start) > 1e-4) or (abs(p_n - p_start) > 1e-4):
                    pos_moved = True

                if verbose:
                    # Currents
                    iqm = float(getattr(axis.motor.current_control, "Iq_measured", 0.0))
                    iqs = float(getattr(axis.motor.current_control, "Iq_setpoint", 0.0))

                    # Controller internal setpoints (if exposed)
                    sp_p = _ctrl_sp()

                    # Expected velocity contribution from position loop (cascaded position->velocity controller)
                    # In ODrive position control, pos_gain has units [turn/s per turn].
                    # So expected vel_setpoint contribution is roughly: pos_gain * (pos_setpoint - pos_est).
                    try:
                        vel_limit_cfg = float(getattr(axis.controller.config, "vel_limit", 0.0))
                    except Exception:
                        vel_limit_cfg = None

                    def _clamp(v, lim):
                        try:
                            if lim is None or float(lim) <= 0:
                                return v
                            lim = float(lim)
                            if v > lim:
                                return lim
                            if v < -lim:
                                return -lim
                            return v
                        except Exception:
                            return v

                    exp_vel_p = None
                    exp_vel_n = None
                    try:
                        if pos_gain is not None:
                            exp_vel_p = _clamp(float(pos_gain) * float(err_sp), vel_limit_cfg)
                            exp_vel_n = _clamp(float(pos_gain) * float(err_sp2), vel_limit_cfg)
                    except Exception:
                        exp_vel_p = None
                        exp_vel_n = None

                    # Capture a best-effort instantaneous velocity estimate at print time
                    try:
                        vel_est_p = float(getattr(axis.encoder, "vel_estimate", 0.0))
                    except Exception:
                        vel_est_p = None

                    print(
                        f"  {i+1:02d}/{int(cycles):02d}: +pos shadow_count={c_p:8d} pos_est={p_p:+.6f} Δc={c_p-c_start:+d} Δp={p_p-p_start:+.6f} "
                        f"cmd_pos={(rb_cmd_p if rb_cmd_p is not None else cmd_p):+.6f}t latched={bool(latched_p)} rb_in={(rb_cmd_p if rb_cmd_p is not None else 'NA')} rb_sp={(rb_sp_p if rb_sp_p is not None else 'NA')} "
                        f"err_sp={err_sp:+.6f}t pos_gain={pos_gain} "
                        f"vel_lim={vel_limit_cfg} exp_vel≈{(exp_vel_p if exp_vel_p is not None else 'NA')} "
                        f"vel_sp={sp_p.get('vel_setpoint')} vel_est={vel_est_p} "
                        f"Iq_set={iqs:+.3f} Iq_meas={iqm:+.3f} enc_err={hex(ee_p)} sp={sp_p}"
                    )

                    # Refresh currents for the -pos phase print (they may have changed)
                    iqm2 = float(getattr(axis.motor.current_control, "Iq_measured", 0.0))
                    iqs2 = float(getattr(axis.motor.current_control, "Iq_setpoint", 0.0))
                    sp_n = _ctrl_sp()

                    try:
                        vel_est_n = float(getattr(axis.encoder, "vel_estimate", 0.0))
                    except Exception:
                        vel_est_n = None

                    print(
                        f"              -pos shadow_count={c_n:8d} pos_est={p_n:+.6f} Δc={c_n-c_start:+d} Δp={p_n-p_start:+.6f} "
                        f"cmd_pos={(rb_cmd_n if rb_cmd_n is not None else cmd_n):+.6f}t latched={bool(latched_n)} rb_in={(rb_cmd_n if rb_cmd_n is not None else 'NA')} rb_sp={(rb_sp_n if rb_sp_n is not None else 'NA')} "
                        f"err_sp2={err_sp2:+.6f}t pos_gain={pos_gain} "
                        f"vel_lim={vel_limit_cfg} exp_vel≈{(exp_vel_n if exp_vel_n is not None else 'NA')} "
                        f"vel_sp={sp_n.get('vel_setpoint')} vel_est={vel_est_n} "
                        f"Iq_set={iqs2:+.3f} Iq_meas={iqm2:+.3f} enc_err={hex(ee_n)} sp={sp_n}"
                    )

                    # Settle line
                    iqm3 = float(getattr(axis.motor.current_control, "Iq_measured", 0.0))
                    iqs3 = float(getattr(axis.motor.current_control, "Iq_setpoint", 0.0))
                    sp_s = _ctrl_sp()
                    try:
                        vel_est_s = float(getattr(axis.encoder, "vel_estimate", 0.0))
                    except Exception:
                        vel_est_s = None

                    print(
                        f"            settle shadow_count={c:8d} pos_est={p:+.6f} Δc={c-c_start:+d} Δp={p-p_start:+.6f} "
                        f"cmd_pos={(rb_cmd_b if rb_cmd_b is not None else base):+.6f}t latched={bool(latched_b)} rb_in={(rb_cmd_b if rb_cmd_b is not None else 'NA')} rb_sp={(rb_sp_b if rb_sp_b is not None else 'NA')} "
                        f"vel_sp={sp_s.get('vel_setpoint')} vel_est={vel_est_s} "
                        f"Iq_set={iqs3:+.3f} Iq_meas={iqm3:+.3f} sp={sp_s}"
                    )

        # Expose whether position commands actually latched
        try:
            # Keep whichever is larger: incremental failures or initial base latch failure
            diag["pos_cmd_latch_failures"] = int(
                max(diag.get("pos_cmd_latch_failures", 0), _latch_failures)
            )
        except Exception:
            pass

        if aborted and verbose:
            print("encoder_watch_cmd: aborted early; returning partial results")

        # Final snapshot
        c_end = int(getattr(axis.encoder, "shadow_count", 0))
        p_end = float(getattr(axis.encoder, "pos_estimate", 0.0))
        e_end = int(getattr(axis.encoder, "error", 0))

        if c_end != c_start:
            counts_moved = True
        if abs(p_end - p_start) > 1e-4:
            pos_moved = True

        if verbose:
            print(f"encoder_watch_cmd: end   shadow_count={c_end} pos_est={p_end:+.6f} Δc={c_end-c_start:+d} Δp={p_end-p_start:+.6f} enc_err={hex(e_end)}")

        return {
            "counts_moved": bool(counts_moved),
            "pos_moved": bool(pos_moved),
            "delta_counts": int(c_end - c_start),
            "delta_pos": float(p_end - p_start),
            "start": {"shadow_count": int(c_start), "pos_est": float(p_start), "enc_err": int(e_start)},
            "end": {"shadow_count": int(c_end), "pos_est": float(p_end), "enc_err": int(e_end)},
            "aborted": bool(diag.get("aborted", False)),
            "diag": diag,
        }

    finally:
        # --- Always restore temporary overrides ---
        # Note: `finally` executes even if we `return` from inside the try.
        try:
            # Bring setpoints to something benign first
            try:
                axis.controller.input_torque = 0.0
            except Exception:
                pass
            try:
                axis.controller.input_pos = float(getattr(axis.encoder, "pos_estimate", 0.0))
            except Exception:
                pass

            # Restore controller config overrides
            if prev_control is not None:
                try:
                    axis.controller.config.control_mode = prev_control
                except Exception:
                    pass
            if prev_input is not None:
                try:
                    axis.controller.config.input_mode = prev_input
                except Exception:
                    pass

            # Restore filter bandwidth / velocity limit only if we changed them
            if _overrode_filter_bw and (prev_filter_bw is not None):
                try:
                    axis.controller.config.input_filter_bandwidth = float(prev_filter_bw)
                except Exception:
                    pass
            if _overrode_vel_limit and (prev_vel_limit is not None):
                try:
                    axis.controller.config.vel_limit = float(prev_vel_limit)
                except Exception:
                    pass

            # Restore torque-mode velocity limiter flag if we changed it
            if _overrode_torque_mode_vel_limit:
                try:
                    if hasattr(axis.controller.config, "enable_torque_mode_vel_limit"):
                        axis.controller.config.enable_torque_mode_vel_limit = prev_torque_mode_vel_limit
                except Exception:
                    pass

            # Restore motor current limit
            if (current_lim is not None) and (prev_current_lim is not None):
                try:
                    axis.motor.config.current_lim = float(prev_current_lim)
                except Exception:
                    pass

            # Restore torque constant if overridden
            if (torque_constant is not None) and (prev_tc is not None):
                try:
                    axis.motor.config.torque_constant = float(prev_tc)
                except Exception:
                    pass

            # Best-effort: if something faulted, try to idle
            try:
                if int(getattr(axis, "error", 0)):
                    try:
                        force_idle(axis, settle_s=0.10)
                    except Exception:
                        pass
                elif (prev_state is not None) and int(prev_state) == int(AXIS_STATE_IDLE):
                    try:
                        force_idle(axis, settle_s=0.10)
                    except Exception:
                        pass
            except Exception:
                pass

        except Exception:
            # Never let cleanup hide the original error
            pass


def preflight_encoder(axis=None, cpr=None, bandwidth=10.0, interp=False):
    """Pre-flight checks for incremental A/B encoder before calibration.

    If you can rotate by hand, this will watch counts in IDLE.
    If you cannot rotate by hand (gearbox/harmonic drive), it will attempt an active
    closed-loop wiggle to provoke encoder counts (and reports whether the motor actually moved).
    """

    if axis is None:
        axis = get_axis0()

    print("state", int(axis.current_state))
    print("ctrl_mode", int(axis.controller.config.control_mode))
    print("input_mode", int(axis.controller.config.input_mode))
    print("enable_torque_mode", getattr(axis.controller.config, "enable_torque_mode", None))
    print("current_lim", axis.motor.config.current_lim)
    print("load_encoder", getattr(axis.config, "load_encoder", None))
    print("commutation_encoder", getattr(axis.config, "commutation_encoder", None))

    force_idle(axis)
    time.sleep(0.05)

    # Apply encoder config while preserving the current index policy.
    # This avoids surprising flips of encoder.config.use_index during preflight.
    try:
        keep_use_index = bool(getattr(axis.encoder.config, "use_index", False))
    except Exception:
        keep_use_index = False

    if cpr is not None:
        set_encoder(
            axis,
            cpr=int(cpr),
            bandwidth=float(bandwidth),
            interp=bool(interp),
            use_index=keep_use_index,
        )
    else:
        set_encoder(
            axis,
            cpr=int(getattr(axis.encoder.config, "cpr", 1024)),
            bandwidth=float(bandwidth),
            interp=bool(interp),
            use_index=keep_use_index,
        )

    clear_errors_all(axis)
    # If we can't rotate by hand, we'll need encoder.is_ready for CLOSED_LOOP wiggle.
    # Try to prepare it early.
    try:
        if not bool(getattr(axis.encoder, "is_ready", False)):
            ensure_encoder_ready(axis, attempt_calibration=True, timeout_s=30.0)
    except Exception:
        pass

    print("preflight_encoder: checking for A/B activity...")

    # Path A: manual rotation in IDLE (best if possible)
    print("  - If you CAN rotate by hand: rotate now (IDLE) ...")
    moved = encoder_watch(axis, seconds=2.0, hz=20.0)

    if moved:
        print("preflight_encoder: OK (counts moved in IDLE).")
        return True

    # Path B: no manual rotation possible -> active wiggle in closed loop
    print("  - No movement detected in IDLE.")
    print("  - Attempting active wiggle (CLOSED_LOOP) to provoke counts...")

    r_pos = encoder_watch_cmd(axis, step_deg=10.0, cycles=4, hold_s=0.20, settle_s=0.15, current_lim=None, mode="pos", verbose=True)

    # If still no counts, try torque wiggle (can break stiction)
    r_tq = None
    if not r_pos.get("counts_moved", False):
        r_tq = encoder_watch_cmd(axis, cycles=4, hold_s=0.20, settle_s=0.15, current_lim=None,
                                mode="torque", torque=0.05, verbose=True, torque_constant=None)

    counts_ok = r_pos.get("counts_moved", False) or (r_tq and r_tq.get("counts_moved", False))
    pos_ok = r_pos.get("pos_moved", False) or (r_tq and r_tq.get("pos_moved", False))

    # If we barely got any counts, warn: this can be a sign of very low motion/stiction or too small step/torque.
    try:
        dc_pos = int(r_pos.get("delta_counts", 0))
    except Exception:
        dc_pos = 0
    try:
        dc_tq = int(r_tq.get("delta_counts", 0)) if r_tq else 0
    except Exception:
        dc_tq = 0
    if counts_ok and (abs(dc_pos) <= 1) and (abs(dc_tq) <= 1):
        print("WARNING: encoder counts only changed by <= 1 count during wiggle. Encoder is alive, but motion is extremely small. Try step_deg=30..90 or torque=0.1..0.5 (with appropriate current_lim) to confirm robust signal.")

    if not counts_ok:
        if pos_ok:
            raise RuntimeError(
                "Motor position estimate changed during active wiggle but encoder counts did NOT change. "
                "This strongly indicates A/B wiring/level/pin mapping issues or MT6701 ABI output not enabled (NO_RESPONSE)."
            )
        else:
            raise RuntimeError(
                "Active wiggle did not produce motion (pos_est unchanged) and encoder counts did not change. "
                "Increase current_lim/torque, reduce stiction, or verify gearbox is not jammed before diagnosing encoder wiring."
            )

    print("preflight_encoder: OK (counts moved during active wiggle).")
    return True

def silent():
    a = get_axis0()
    clear_errors_all(a)
    force_idle(a)

    c0 = int(a.encoder.shadow_count)
    time.sleep(0.5)
    c1 = int(a.encoder.shadow_count)
    print("shadow_count delta in 0.5s:", c1-c0, "enc_err:", hex(int(a.encoder.error)))


def suggest_wiggle_params(step_deg=10.0, pos_gain=None, min_step_deg=30.0, min_pos_gain=30.0):
    """Quick heuristic guidance when position wiggle shows ~0 Iq_set and no movement.

    If you see:
      - cmd_pos changes, but pos_est and shadow_count stay constant
      - Iq_set stays near 0

    then either the effective setpoint is not moving (filtering / mode mismatch) or the commanded torque from pos_gain*error
    is too small to overcome stiction/backlash.

    Returns a dict of suggested next test parameters.
    """
    out = {
        "try_step_deg": max(float(step_deg), float(min_step_deg)),
        "try_pos_gain": None,
        "try_torque_nm": 0.20,
        "try_hold_s": 0.35,
        "notes": [
            "If using input_filter_bandwidth, temporarily increase it (e.g. 5..20) so setpoint actually moves during short holds.",
            "If pos wiggle still yields Iq_set~0, verify control_mode=POSITION_CONTROL and input_mode=PASSTHROUGH are actually taking effect.",
            "If torque-mode wiggle moves but pos-mode does not, the position loop may be too soft (pos_gain too low) or setpoint filtering is hiding the step.",
        ],
    }
    if pos_gain is not None:
        out["try_pos_gain"] = max(float(pos_gain), float(min_pos_gain))
    else:
        out["try_pos_gain"] = float(min_pos_gain)
    return out

def torque_breakaway():
    a = get_axis0()
    clear_errors_all(a)

    for tq in [0.30, 0.40, 0.50, 0.60, 0.7, 0.8, 1]:
        print("\n=== torque =", tq, "Nm ===")
        encoder_watch_cmd(a, mode="torque", torque=tq, cycles=6, hold_s=0.25, settle_s=0.20, current_lim=20, verbose=True)

def set_values(pos_gain = 25.0, vel_gain = 0.4, vel_integrator_gain = 0.8, input_filter_bandwidth = 10.0, vel_limit = 5.0, tt_vel_limit = 5.0, tt_accel_limit = 10.0, tt_decel_limit = 10.0):
    a = get_axis0()

    a.controller.config.pos_gain = pos_gain
    a.controller.config.vel_gain = vel_gain
    a.controller.config.vel_integrator_gain = vel_integrator_gain
    a.controller.config.input_filter_bandwidth = input_filter_bandwidth

    a.controller.config.vel_limit = vel_limit
    a.trap_traj.config.vel_limit = tt_vel_limit
    a.trap_traj.config.accel_limit = tt_accel_limit
    a.trap_traj.config.decel_limit = tt_decel_limit

def re_sync():
    a = get_axis0()

    # 1) Make sure we're in position + passthrough
    a.controller.config.control_mode = 3          # POSITION_CONTROL
    a.controller.config.input_mode   = 1          # PASSTHROUGH

    # 2) Hard-sync the commanded position to the measured position
    p = a.encoder.pos_estimate
    a.controller.input_pos = p

    # (optional but recommended) also clear integrator windup if you have it enabled
    try:
        a.controller.vel_integrator_torque = 0.0
    except Exception:
        pass


def get_live_odrv(name: str = "odrv0", strict: bool = False):
    """Get an ODrive handle by name from the current IPython namespace."""
    ip = get_ipython()
    odrv = ip.user_ns.get(str(name)) if ip else None
    if odrv is None:
        raise RuntimeError(f"{name} not found. Wait for odrivetool to connect and assign `{name}`.")
    if strict and _is_anonymous_interface(odrv):
        raise RuntimeError(f"{name} appears stale/anonymous after reconnect. Re-run after reconnect is complete.")
    return odrv


def _is_axis_like(obj) -> bool:
    try:
        return hasattr(obj, "controller") and hasattr(obj, "encoder") and hasattr(obj, "motor")
    except Exception:
        return False


def get_live_axis(axis_ref="odrv0.axis0", strict: bool = False):
    """Resolve an axis from object or string ref: 'odrvX.axisY', ('odrvX', Y), or Y."""
    if _is_axis_like(axis_ref):
        axis = axis_ref
        if strict and (_is_anonymous_interface(axis) or _is_anonymous_interface(getattr(axis, "_parent", None))):
            raise RuntimeError("Axis handle appears stale/anonymous after reconnect.")
        return axis

    odrv_name = "odrv0"
    axis_index = 0

    if isinstance(axis_ref, str):
        s = axis_ref.strip()
        if "." in s:
            left, right = s.split(".", 1)
            odrv_name = left.strip() or "odrv0"
            r = right.strip()
            if not r.startswith("axis"):
                raise ValueError(f"Invalid axis ref `{axis_ref}`. Expected e.g. 'odrv0.axis0'.")
            axis_index = int(r[4:])
        elif s.startswith("axis"):
            axis_index = int(s[4:])
        elif s:
            odrv_name = s
            axis_index = 0
    elif isinstance(axis_ref, (tuple, list)) and len(axis_ref) == 2:
        odrv_name = str(axis_ref[0])
        axis_index = int(axis_ref[1])
    elif isinstance(axis_ref, int):
        axis_index = int(axis_ref)
    else:
        raise TypeError(
            "axis_ref must be an axis object, 'odrvX.axisY', ('odrvX', Y), or integer axis index."
        )

    odrv = get_live_odrv(name=odrv_name, strict=strict)
    attr = f"axis{int(axis_index)}"
    try:
        axis = getattr(odrv, attr)
    except Exception as ex:
        raise RuntimeError(f"Could not access {odrv_name}.{attr}: {ex}")

    if strict and (_is_anonymous_interface(axis) or _is_anonymous_interface(getattr(axis, "_parent", None))):
        raise RuntimeError(f"{odrv_name}.{attr} appears stale/anonymous after reconnect.")
    return axis


def _axis_name_from_ref(axis_ref, fallback: str) -> str:
    if isinstance(axis_ref, str):
        s = axis_ref.strip()
        if s:
            return s
    if isinstance(axis_ref, (tuple, list)) and len(axis_ref) == 2:
        try:
            return f"{axis_ref[0]}.axis{int(axis_ref[1])}"
        except Exception:
            return fallback
    return fallback


def _normalize_axis_targets(targets):
    """Normalize targets into [{'axis': axis_obj, 'target': float, 'name': str}, ...]."""
    rows = []

    if isinstance(targets, dict):
        iterable = [{"axis": k, "target": v} for k, v in targets.items()]
    else:
        iterable = list(targets)

    for i, item in enumerate(iterable):
        axis_ref = None
        target = None
        name = None

        if isinstance(item, dict):
            axis_ref = item.get("axis", item.get("axis_ref"))
            target = item.get("target", item.get("target_turns", item.get("turns")))
            name = item.get("name")
        elif isinstance(item, (tuple, list)) and len(item) >= 2:
            axis_ref = item[0]
            target = item[1]
            name = item[2] if len(item) > 2 else None
        else:
            raise ValueError(
                "Each target must be dict {'axis': ..., 'target': ...} or tuple/list (axis_ref, target_turns[, name])."
            )

        if axis_ref is None:
            raise ValueError(f"Target item #{i + 1} is missing axis reference.")
        if target is None:
            raise ValueError(f"Target item #{i + 1} is missing target turns.")

        axis_obj = get_live_axis(axis_ref, strict=False)
        if not _is_axis_like(axis_obj):
            raise RuntimeError(f"Target item #{i + 1} did not resolve to a valid axis object.")

        default_name = f"axis{i}"
        if name is None:
            name = _axis_name_from_ref(axis_ref, fallback=default_name)
        rows.append({"axis": axis_obj, "target": float(target), "name": str(name)})

    if not rows:
        raise ValueError("No targets supplied.")

    # Deduplicate names to keep return/report keys stable.
    used = {}
    for row in rows:
        base = row["name"]
        n = used.get(base, 0)
        if n:
            row["name"] = f"{base}#{n + 1}"
        used[base] = n + 1

    return rows


def establish_absolute_reference(
    axis=None,
    require_index: bool = True,
    run_index_search: bool = False,
    timeout_s: float = 20.0,
    attempt_offset_calibration: bool = True,
    recover_offset_after_index: bool = True,
    label: str = "absolute_ref",
    allow_unready_closed_loop_fallback: bool = False,
):
    """Enforce an absolute-reference contract before motion.

    For incremental encoders, true startup-repeatable absolute positioning usually requires index.
    This helper will raise if the contract is not satisfied.
    """
    axis = get_live_axis(axis if axis is not None else "odrv0.axis0", strict=False)

    # Start clean and make sure encoder readiness exists.
    clear_errors_all(axis)
    defer_ready_until_after_index = False
    if bool(attempt_offset_calibration):
        if not ensure_encoder_ready(axis, attempt_calibration=True, timeout_s=float(timeout_s)):
            # If index-search is part of this contract, allow index-first recovery
            # instead of failing immediately on flaky offset-calibration NO_RESPONSE.
            if bool(require_index) and bool(run_index_search):
                defer_ready_until_after_index = True
                print(
                    f"{label}: encoder not ready after offset calibration; "
                    "deferring readiness to index-search path."
                )
                clear_errors_all(axis, settle_s=0.05)
            else:
                raise RuntimeError(
                    f"{label}: encoder is not ready after offset calibration. "
                    f"snapshot={_snapshot_motion(axis)}"
                )
    else:
        is_ready_attr_present = False
        is_ready_val = True
        try:
            is_ready_attr_present = hasattr(axis.encoder, "is_ready")
        except Exception:
            is_ready_attr_present = False
        if is_ready_attr_present:
            try:
                is_ready_val = bool(axis.encoder.is_ready)
            except Exception:
                is_ready_val = False
            if not bool(is_ready_val):
                # If index-search is explicitly requested, allow readiness to be
                # established in the index-search path (with post-index recovery).
                if bool(require_index) and bool(run_index_search):
                    defer_ready_until_after_index = True
                else:
                    raise RuntimeError(f"{label}: encoder.is_ready is False.")

    if bool(defer_ready_until_after_index):
        print(f"{label}: encoder.is_ready=False at entry; deferring readiness check to post-index recovery.")

    try:
        use_index = bool(getattr(axis.encoder.config, "use_index", False))
    except Exception:
        use_index = False

    if bool(require_index) and (not use_index):
        raise RuntimeError(
            f"{label}: require_index=True but encoder.config.use_index is False. "
            "Enable index (or use a true absolute encoder / homing reference) before closed-loop operation."
        )

    ran_index_search = False
    allow_unready_closed_loop = False

    # If requested, actively run index search now (usually done once per boot).
    if bool(require_index) and bool(run_index_search):
        force_idle(axis, settle_s=0.05)
        clear_errors_all(axis)
        axis.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
        if not wait_idle(axis, timeout_s=float(timeout_s)):
            raise RuntimeError(f"{label}: index search timed out.")
        assert_no_errors(axis, label=f"{label}/index_search")
        ran_index_search = True

        # On newer firmwares, procedure_result can indicate silent failures (axis.error may stay 0).
        try:
            pr = int(getattr(axis, "procedure_result", 0))
        except Exception:
            pr = None
        if pr is not None and pr != 0:
            raise RuntimeError(
                f"{label}: index search returned procedure_result={pr} (non-success). "
                f"snapshot={_snapshot_motion(axis)}"
            )

        # Explicit readiness check after index search (some builds silently leave is_ready False).
        # Recovery path: run one encoder offset calibration after index-search if requested.
        try:
            if hasattr(axis.encoder, "is_ready") and (not bool(axis.encoder.is_ready)):
                if bool(recover_offset_after_index):
                    recovered = False
                    recover_attempts = 2
                    for rec_try in range(1, recover_attempts + 1):
                        recovered = ensure_encoder_ready(
                            axis,
                            attempt_calibration=True,
                            timeout_s=float(timeout_s),
                        )
                        if bool(recovered):
                            break
                        if rec_try < recover_attempts:
                            print(
                                f"{label}: post-index offset recovery attempt {rec_try}/{recover_attempts} failed; "
                                "rerunning index search before retry."
                            )
                            force_idle(axis, settle_s=0.05)
                            clear_errors_all(axis, settle_s=0.05)
                            try:
                                axis.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
                            except Exception:
                                pass
                            if not wait_idle(axis, timeout_s=float(timeout_s)):
                                break
                            try:
                                assert_no_errors(axis, label=f"{label}/post_index_recover_{rec_try}")
                            except Exception:
                                break
                    if not bool(recovered):
                        if not bool(allow_unready_closed_loop_fallback):
                            raise RuntimeError(
                                f"{label}: encoder stayed not-ready after index and post-index recovery. "
                                "Refusing closed-loop fallback while encoder.is_ready=False."
                            )
                        # Firmware-compat fallback (opt-in only):
                        # some builds can report is_ready=False even when index lock is usable.
                        # Try one closed-loop entry without gating on is_ready.
                        clear_errors_all(axis, settle_s=0.05)
                        ok_unready = ensure_closed_loop(
                            axis,
                            timeout_s=2.5,
                            clear_first=False,
                            pre_sync=True,
                            retries=2,
                            require_encoder_ready=False,
                        )
                        if bool(ok_unready):
                            allow_unready_closed_loop = True
                            try:
                                force_idle(axis, settle_s=0.05)
                            except Exception:
                                pass
                            print(
                                f"{label}: encoder.is_ready stayed False after index, "
                                "but CLOSED_LOOP succeeded; proceeding with firmware-compat fallback."
                            )
                        else:
                            # Do not fail early; continue into the deterministic closed-loop
                            # fallback path below for richer diagnostics/retries.
                            allow_unready_closed_loop = True
                            print(
                                f"{label}: encoder stayed not-ready after index and post-index recovery; "
                                "continuing with deferred closed-loop diagnostics."
                            )
                else:
                    raise RuntimeError(
                        f"{label}: index search completed but encoder.is_ready is False. "
                        f"snapshot={_snapshot_motion(axis)}"
                    )
        except RuntimeError:
            raise
        except Exception:
            pass

    # Best-effort explicit index-found check (field availability varies by firmware).
    try:
        idx_found = getattr(axis.encoder, "index_found", None)
    except Exception:
        idx_found = None
    if bool(require_index) and (idx_found is not None) and (not bool(idx_found)):
        raise RuntimeError(f"{label}: index not found (encoder.index_found=False).")

    # Important: after a successful index search, avoid another blanket clear here because some
    # firmware variants can lose transient index readiness after aggressive clear/reset calls.
    clear_for_reentry = not bool(ran_index_search)
    if not ensure_closed_loop(
        axis,
        timeout_s=3.0,
        clear_first=clear_for_reentry,
        pre_sync=True,
        retries=2,
        require_encoder_ready=(not bool(allow_unready_closed_loop)),
    ):
        snap0 = _snapshot_motion(axis)

        # Firmware-specific fallback:
        # Sometimes requested closed-loop is silently rejected (state remains IDLE, no error bits).
        # In that case, do one fully deterministic re-entry sequence before failing hard.
        idle_no_err = (
            int(snap0.get("state", 0)) == int(AXIS_STATE_IDLE)
            and int(snap0.get("axis_err", 0)) == 0
            and int(snap0.get("motor_err", 0)) == 0
            and int(snap0.get("enc_err", 0)) == 0
            and int(snap0.get("ctrl_err", 0)) == 0
        )

        if idle_no_err:
            force_idle(axis, settle_s=0.08)
            clear_errors_all(axis, settle_s=0.08)

            # If index is required, rerun index-search once in fallback path.
            if bool(require_index) and bool(run_index_search):
                try:
                    axis.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
                except Exception:
                    pass
                if not wait_idle(axis, timeout_s=float(timeout_s)):
                    raise RuntimeError(
                        f"{label}: fallback index search timed out. snapshot={_snapshot_motion(axis)}"
                    )
                assert_no_errors(axis, label=f"{label}/fallback_index_search")

            # Re-prepare setpoint and re-enter with extra retries.
            try:
                axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
            except Exception:
                pass
            try:
                axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
            except Exception:
                pass
            try:
                axis.controller.input_pos = float(getattr(axis.encoder, "pos_estimate", 0.0))
            except Exception:
                pass
            try:
                axis.controller.vel_integrator_torque = 0.0
            except Exception:
                pass

            ok2 = ensure_closed_loop(
                axis,
                timeout_s=4.0,
                clear_first=False,
                pre_sync=True,
                retries=4,
                require_encoder_ready=(not bool(allow_unready_closed_loop)),
            )
            if not ok2:
                second_snap = _snapshot_motion(axis)
                index_diag = None

                # Targeted diagnosis: if index is required and closed-loop only fails with
                # use_index=True, then the index reference is likely missing/unseen.
                if bool(require_index):
                    prev_use_index = None
                    try:
                        prev_use_index = bool(getattr(axis.encoder.config, "use_index", True))
                    except Exception:
                        prev_use_index = None

                    try:
                        try:
                            axis.encoder.config.use_index = False
                        except Exception:
                            pass
                        try:
                            axis.controller.input_pos = float(getattr(axis.encoder, "pos_estimate", 0.0))
                        except Exception:
                            pass

                        ok_no_idx = ensure_closed_loop(
                            axis,
                            timeout_s=2.0,
                            clear_first=False,
                            pre_sync=True,
                            retries=1,
                            require_encoder_ready=False,
                        )
                        index_diag = {
                            "closed_loop_with_use_index_false": bool(ok_no_idx),
                            "snapshot_no_index": _snapshot_motion(axis),
                        }
                    except Exception:
                        index_diag = {
                            "closed_loop_with_use_index_false": False,
                            "snapshot_no_index": _snapshot_motion(axis),
                        }
                    finally:
                        try:
                            axis.requested_state = AXIS_STATE_IDLE
                        except Exception:
                            pass
                        try:
                            if prev_use_index is not None:
                                axis.encoder.config.use_index = bool(prev_use_index)
                        except Exception:
                            pass

                idx_msg = ""
                try:
                    if index_diag and bool(index_diag.get("closed_loop_with_use_index_false", False)):
                        idx_msg = (
                            " Diagnostic: CLOSED_LOOP succeeds with use_index=False but fails with use_index=True; "
                            "index reference is likely missing/unseen (Z signal, pin mapping, encoder index output, or index-search conditions)."
                        )
                except Exception:
                    idx_msg = ""

                raise RuntimeError(
                    f"{label}: failed to enter CLOSED_LOOP_CONTROL after fallback re-entry. "
                    f"first_snapshot={snap0} second_snapshot={second_snap}"
                    f"{idx_msg}"
                    f" index_diag={index_diag}"
                )
        else:
            raise RuntimeError(f"{label}: failed to enter CLOSED_LOOP_CONTROL. snapshot={snap0}")

    if not sync_pos_setpoint(axis, settle_s=0.05, retries=3, verbose=False):
        raise RuntimeError(f"{label}: failed to synchronize pos_setpoint/input_pos.")

    return {
        "encoder_ready": bool(getattr(axis.encoder, "is_ready", True)),
        "use_index": bool(use_index),
        "index_found": (bool(idx_found) if idx_found is not None else None),
        "state": int(getattr(axis, "current_state", 0)),
        "ok": True,
    }


def move_axes_absolute_synced(
    targets,
    use_trap_traj: bool = True,
    trap_vel: float = 2.0,
    trap_acc: float = 5.0,
    trap_dec: float = 5.0,
    require_absolute: bool = True,
    require_index: bool = True,
    run_index_search: bool = False,
    wait_until_monotonic=None,
    max_command_skew_s: float = 0.004,
    strict_command_skew: bool = True,
    timeout_s: float = 4.0,
    settle_s: float = 0.05,
    max_error_turns: float = 0.002,
    max_vel_turns_s: float = 0.08,
):
    """Issue absolute position commands to multiple axes with strict safety/verification.

    `targets` examples:
      - [("odrv0.axis0", 1.25), ("odrv1.axis0", -0.50)]
      - [{"axis": "odrv0.axis0", "target": 1.25, "name": "joint0"}, ...]
      - {axis_obj0: 1.25, axis_obj1: -0.50}
    """
    rows = _normalize_axis_targets(targets)

    # 1) Pre-arm every axis with the same contract before any command is issued.
    for row in rows:
        axis = row["axis"]
        name = row["name"]

        clear_errors_all(axis)
        assert_no_errors(axis, label=f"{name}/pre")

        if bool(require_absolute):
            establish_absolute_reference(
                axis=axis,
                require_index=bool(require_index),
                run_index_search=bool(run_index_search),
                timeout_s=max(5.0, float(timeout_s)),
                attempt_offset_calibration=True,
                label=name,
            )
        else:
            if not ensure_closed_loop(axis, timeout_s=2.0):
                raise RuntimeError(f"{name}: failed to enter CLOSED_LOOP_CONTROL. snapshot={_snapshot_motion(axis)}")
            if not sync_pos_setpoint(axis, settle_s=0.05, retries=2, verbose=False):
                raise RuntimeError(f"{name}: failed to sync setpoints before command.")

        axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        axis.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ if bool(use_trap_traj) else INPUT_MODE_PASSTHROUGH
        if bool(use_trap_traj):
            try:
                axis.trap_traj.config.vel_limit = float(trap_vel)
                axis.trap_traj.config.accel_limit = float(trap_acc)
                axis.trap_traj.config.decel_limit = float(trap_dec)
            except Exception:
                pass

        # Prime the commanded position to current estimate to avoid jumps.
        try:
            axis.controller.input_pos = float(getattr(axis.encoder, "pos_estimate", 0.0))
        except Exception:
            pass
        if not sync_pos_setpoint(axis, settle_s=0.03, retries=2, verbose=False):
            raise RuntimeError(f"{name}: setpoint sync failed during prime.")

    # 2) Optional external timing barrier.
    if wait_until_monotonic is not None:
        dt = float(wait_until_monotonic) - float(time.monotonic())
        if dt > 0:
            time.sleep(dt)

    # 3) Issue all commands in a tight burst and measure skew.
    issue_times = {}
    for row in rows:
        axis = row["axis"]
        target = float(row["target"])
        name = row["name"]

        t_cmd = float(time.monotonic())
        if bool(use_trap_traj) and hasattr(axis.controller, "move_to_pos"):
            axis.controller.move_to_pos(target)
        else:
            axis.controller.input_pos = target
        issue_times[name] = t_cmd

    t_vals = list(issue_times.values())
    command_skew_s = (max(t_vals) - min(t_vals)) if len(t_vals) >= 2 else 0.0
    if command_skew_s > float(max_command_skew_s):
        msg = (
            "Command skew too high for strict sync: "
            f"{command_skew_s:.6f}s > {float(max_command_skew_s):.6f}s."
        )
        if bool(strict_command_skew):
            raise RuntimeError(msg)

    # 4) Verify every axis reaches its absolute target and remains healthy.
    deadline = float(time.monotonic()) + float(timeout_s)
    pending = {row["name"] for row in rows}
    latest = {}

    while pending and (float(time.monotonic()) < deadline):
        for row in rows:
            name = row["name"]
            if name not in pending:
                continue
            axis = row["axis"]
            target = float(row["target"])

            assert_no_errors(axis, label=f"{name}/move")
            if int(getattr(axis, "current_state", 0)) != int(AXIS_STATE_CLOSED_LOOP_CONTROL):
                raise RuntimeError(f"{name}: dropped out of CLOSED_LOOP_CONTROL during synced move.")

            pos = float(getattr(axis.encoder, "pos_estimate", 0.0))
            vel = float(getattr(axis.encoder, "vel_estimate", 0.0))
            err = target - pos
            latest[name] = {"target": target, "pos": pos, "vel": vel, "err": err}

            if abs(err) <= float(max_error_turns) and abs(vel) <= float(max_vel_turns_s):
                pending.remove(name)

        if pending:
            time.sleep(0.01)

    if pending:
        snapshots = {}
        for row in rows:
            if row["name"] in pending:
                snap = _snapshot_motion(row["axis"])
                snap["target"] = float(row["target"])
                snapshots[row["name"]] = snap
        raise RuntimeError(
            f"Timed out waiting for synced absolute move completion on {sorted(pending)}. "
            f"latest={latest} snapshots={snapshots}"
        )

    if float(settle_s) > 0.0:
        time.sleep(float(settle_s))

    final = {}
    for row in rows:
        axis = row["axis"]
        name = row["name"]
        target = float(row["target"])
        pos = float(getattr(axis.encoder, "pos_estimate", 0.0))
        vel = float(getattr(axis.encoder, "vel_estimate", 0.0))
        err = target - pos
        final[name] = {"target": target, "pos": pos, "vel": vel, "err": err}

        assert_no_errors(axis, label=f"{name}/final")
        if int(getattr(axis, "current_state", 0)) != int(AXIS_STATE_CLOSED_LOOP_CONTROL):
            raise RuntimeError(f"{name}: not in CLOSED_LOOP_CONTROL at end of synced move.")
        if abs(err) > float(max_error_turns):
            raise RuntimeError(
                f"{name}: final position error too large. err={err:+.6f}t limit={float(max_error_turns):.6f}t "
                f"snapshot={_snapshot_motion(axis)}"
            )

    return {
        "ok": True,
        "command_skew_s": float(command_skew_s),
        "issue_times": issue_times,
        "final": final,
        "count": int(len(rows)),
    }
