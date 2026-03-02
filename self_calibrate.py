# self_calibrate.py
# Run inside odrivetool:
#   %run -i self_calibrate.py
# Then:
#   full_self_calibrate(odrv0.axis0)

import time, math
from IPython import get_ipython
from odrive.enums import *
from common import establish_absolute_reference
from common import auto_direction_contract as run_auto_direction_contract_selector

ip = get_ipython()
odrv0 = ip.user_ns.get("odrv0") if ip else None
if odrv0 is None:
    # Terminal/script fallback when not running inside odrivetool IPython.
    try:
        import odrive
        odrv0 = odrive.find_any(timeout=3.0)
    except Exception:
        odrv0 = None
if odrv0 is None:
    raise RuntimeError(
        "odrv0 not found. Start odrivetool and connect first, "
        "or ensure the ODrive is reachable from terminal via USB."
    )

def errhex(axis):
    def _decode_bits(val: int, prefix: str):
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

    ax_err = int(getattr(axis, "error", 0))
    m_err  = int(getattr(axis.motor, "error", 0))
    e_err  = int(getattr(axis.encoder, "error", 0))
    c_err  = int(getattr(axis.controller, "error", 0))

    return {
        "state": int(getattr(axis, "current_state", -1)),
        "axis_err": hex(ax_err),
        "axis_err_names": _decode_bits(ax_err, "AXIS_ERROR_"),
        "motor_err": hex(m_err),
        "motor_err_names": _decode_bits(m_err, "MOTOR_ERROR_"),
        "enc_err": hex(e_err),
        "enc_err_names": _decode_bits(e_err, "ENCODER_ERROR_"),
        "ctrl_err": hex(c_err),
        "ctrl_err_names": _decode_bits(c_err, "CONTROLLER_ERROR_"),
    }

def _feed_watchdog(axis):
    try:
        if hasattr(axis, "watchdog_feed"):
            axis.watchdog_feed()
    except Exception:
        pass


def _sleep_with_watchdog(axis, seconds: float, step: float = 0.02):
    """Sleep safely in small increments; avoids negative sleep due to timing jitter.
    Also feeds the watchdog if supported.
    """
    try:
        total = float(seconds)
    except Exception:
        total = 0.0

    if total <= 0.0:
        _feed_watchdog(axis)
        return

    try:
        step_s = max(0.001, float(step))
    except Exception:
        step_s = 0.02

    t0 = time.time()
    while True:
        _feed_watchdog(axis)
        remaining = total - (time.time() - t0)
        if remaining <= 0.0:
            break
        time.sleep(min(step_s, remaining))
        
        
def clear_errors(axis):
    try:
        axis.clear_errors()
    except Exception:
        pass
    # fallbacks (some firmwares allow this, some don’t)
    for obj in (axis, axis.motor, axis.encoder, axis.controller):
        try:
            obj.error = 0
        except Exception:
            pass

def ensure_idle_and_cleared(axis, tries=3, settle_s=0.25):
    """Force IDLE and clear errors. If a fault immediately reasserts, fail early.

    Prevents the pattern where one failed calibration leaves the axis faulted,
    making subsequent attempts fail instantly until a board reset.
    """
    try:
        tries_i = max(1, int(tries))
    except Exception:
        tries_i = 1

    try:
        settle = float(settle_s)
    except Exception:
        settle = 0.25

    for _ in range(tries_i):
        # Force IDLE
        try:
            axis.requested_state = AXIS_STATE_IDLE
        except Exception:
            pass

        # Give it a moment to actually enter IDLE
        _sleep_with_watchdog(axis, max(0.10, settle), step=0.02)

        # Clear errors (best effort)
        clear_errors(axis)
        _sleep_with_watchdog(axis, max(0.05, settle * 0.5), step=0.02)

        # Check if clean
        try:
            ax_e = int(getattr(axis, "error", 0))
            m_e  = int(getattr(axis.motor, "error", 0))
            e_e  = int(getattr(axis.encoder, "error", 0))
            c_e  = int(getattr(axis.controller, "error", 0))
        except Exception:
            ax_e = m_e = e_e = c_e = 0

        if (ax_e == 0) and (m_e == 0) and (e_e == 0) and (c_e == 0):
            return

    print("Persistent fault after clear:", errhex(axis))
    raise RuntimeError("persistent fault (won't clear) - remove root cause or reset/power-cycle")
    
def sync_to_est(axis, repeats=2, dt=0.10):
    # PASSTHROUGH: setpoint tracks input_pos directly, so syncing matters.
    for _ in range(int(repeats)):
        axis.controller.input_pos = float(axis.encoder.pos_estimate)
        _sleep_with_watchdog(axis, dt, step=0.02)

def watch(axis, seconds=0.8, dt=0.02, vel_abort=1.0, iq_abort=6.0):
    t0 = time.time()
    while time.time() - t0 < seconds:
        st = int(axis.current_state)
        ax_e = int(axis.error)
        m_e  = int(axis.motor.error)
        c_e  = int(axis.controller.error)

        pos_set = float(axis.controller.pos_setpoint)
        in_pos  = float(axis.controller.input_pos)
        pos_est = float(axis.encoder.pos_estimate)
        vel_est = float(axis.encoder.vel_estimate)
        iq_set  = float(axis.motor.current_control.Iq_setpoint)
        iq_meas = float(axis.motor.current_control.Iq_measured)

        print("in:", in_pos, "set:", pos_set, "est:", pos_est,
              "vel:", vel_est, "Iq_set:", iq_set, "Iq_meas:", iq_meas,
              "state:", st, "axis_err:", ax_e, "motor_err:", m_e, "ctrl_err:", c_e)

        if st != int(AXIS_STATE_CLOSED_LOOP_CONTROL) or ax_e or m_e or c_e:
            print("ABORT: left closed loop or fault")
            print("errors:", errhex(axis))
            raise RuntimeError("left closed loop / fault")

        if abs(vel_est) > vel_abort or abs(iq_meas) > iq_abort:
            print("ABORT: runaway")
            print("errors:", errhex(axis))
            axis.requested_state = AXIS_STATE_IDLE
            raise RuntimeError("runaway")

        _sleep_with_watchdog(axis, dt, step=0.02)

def step_deg(axis, deg):
    # Always compute from live estimate (avoids stale input_pos weirdness)
    est = float(axis.encoder.pos_estimate)
    axis.controller.input_pos = est + (float(deg) / 360.0)


# Helper: Preload/break stiction for gearboxes/harmonic drives
def preload_break_stiction(axis, deg=3.0, hold_s=0.30, settle_s=0.20, cycles=1):
    """Apply a small position dither to preload gearboxes/harmonic drives.
    This helps break static friction and makes subsequent torque-sign tests repeatable.

    Runs in POSITION + PASSTHROUGH.
    """
    try:
        cycles_i = max(1, int(cycles))
    except Exception:
        cycles_i = 1

    # Ensure we're in position passthrough and synced
    axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    axis.controller.config.input_mode   = INPUT_MODE_PASSTHROUGH
    sync_to_est(axis, repeats=2, dt=0.08)
    _sleep_with_watchdog(axis, 0.05, step=0.02)

    for _ in range(cycles_i):
        p0 = float(axis.encoder.pos_estimate)
        axis.controller.input_pos = p0 + (float(deg) / 360.0)
        _sleep_with_watchdog(axis, float(hold_s), step=0.02)
        # Return to a fresh estimate (avoids stale setpoint)
        p1 = float(axis.encoder.pos_estimate)
        axis.controller.input_pos = p1
        _sleep_with_watchdog(axis, float(settle_s), step=0.02)


def _enter_closed_loop_position(axis, retries=3, settle_s=0.10):
    """Best-effort closed-loop entry in POSITION+PASSTHROUGH with synced setpoint."""
    tries = max(1, int(retries))
    for _ in range(tries):
        ensure_idle_and_cleared(axis, tries=2, settle_s=max(0.08, float(settle_s)))
        try:
            axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
            axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        except Exception:
            pass
        sync_to_est(axis, repeats=2, dt=0.08)
        try:
            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        except Exception:
            pass
        _sleep_with_watchdog(axis, 0.30, step=0.02)
        try:
            st = int(getattr(axis, "current_state", 0))
            ax_e = int(getattr(axis, "error", 0))
            m_e = int(getattr(axis.motor, "error", 0))
            e_e = int(getattr(axis.encoder, "error", 0))
            c_e = int(getattr(axis.controller, "error", 0))
        except Exception:
            st, ax_e, m_e, e_e, c_e = 0, 1, 1, 1, 1
        if st == int(AXIS_STATE_CLOSED_LOOP_CONTROL) and ax_e == 0 and m_e == 0 and e_e == 0 and c_e == 0:
            sync_to_est(axis, repeats=2, dt=0.05)
            return True
    return False

def torque_sign_sanity(axis, torque_nm=0.05, pulse_s=0.03, settle_s=0.18, repeats=8,
                      vel_soft=0.8, vel_hard=2.0, min_avg_counts=6, min_avg_vel=0.05):
    cpr = int(axis.encoder.config.cpr)

    # Switch to torque control
    prev_cm = int(axis.controller.config.control_mode)
    prev_im = int(axis.controller.config.input_mode)
    axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
    axis.controller.config.input_mode   = INPUT_MODE_PASSTHROUGH
    axis.controller.input_torque = 0.0
    _sleep_with_watchdog(axis, 0.05, step=0.01)

    pos_d = []; neg_d = []
    pos_v = []; neg_v = []
    soft_ab_pos = soft_ab_neg = 0
    hard_ab_pos = hard_ab_neg = 0

    def pulse(tau):
        q0 = int(axis.encoder.shadow_count)
        t0 = time.time()
        v_sum = 0.0
        n = 0
        v_peak = 0.0
        soft = hard = False

        axis.controller.input_torque = float(tau)
        while True:
            elapsed = time.time() - t0
            remaining = float(pulse_s) - elapsed
            if remaining <= 0.0:
                break

            v = float(axis.encoder.vel_estimate)
            v_sum += v
            n += 1
            v_peak = max(v_peak, abs(v))
            if abs(v) > vel_hard:
                hard = True
                break
            if abs(v) > vel_soft:
                soft = True

            # small tick; also feeds watchdog
            _sleep_with_watchdog(axis, min(0.01, remaining), step=0.01)

        axis.controller.input_torque = 0.0
        _sleep_with_watchdog(axis, settle_s, step=0.02)
        q1 = int(axis.encoder.shadow_count)
        avg_v = (v_sum / n) if n else 0.0
        return (q1 - q0), avg_v, v_peak, soft, hard

    for i in range(int(repeats)):
        d, av, vp, soft, hard = pulse(+torque_nm)
        pos_d.append(d); pos_v.append(av)
        soft_ab_pos += 1 if soft else 0
        hard_ab_pos += 1 if hard else 0
        print(f"  + pulse {i+1}/{repeats}: Δcounts={d:6d} avg_vel={av:+.3f} peak_vel={vp:.3f} soft={soft} hard={hard}")

    for i in range(int(repeats)):
        d, av, vp, soft, hard = pulse(-torque_nm)
        neg_d.append(d); neg_v.append(av)
        soft_ab_neg += 1 if soft else 0
        hard_ab_neg += 1 if hard else 0
        print(f"  - pulse {i+1}/{repeats}: Δcounts={d:6d} avg_vel={av:+.3f} peak_vel={vp:.3f} soft={soft} hard={hard}")

    avg_pos = sum(pos_d)/len(pos_d)
    avg_neg = sum(neg_d)/len(neg_d)
    avg_pos_v = sum(pos_v)/len(pos_v)
    avg_neg_v = sum(neg_v)/len(neg_v)

    score_counts = avg_pos - avg_neg
    score_vel = avg_pos_v - avg_neg_v

    print("\nSummary:")
    print("  pos_deltas:", pos_d)
    print("  neg_deltas:", neg_d)
    print("  avg_pos=%.2f counts, avg_neg=%.2f counts, score_counts=%.2f" % (avg_pos, avg_neg, score_counts))
    print("  avg_pos_v=%+.3f t/s, avg_neg_v=%+.3f t/s, score_vel=%+.3f" % (avg_pos_v, avg_neg_v, score_vel))
    print(f"  soft_aborts: +{soft_ab_pos}/{repeats}, -{soft_ab_neg}/{repeats}")
    print(f"  hard_aborts: +{hard_ab_pos}/{repeats}, -{hard_ab_neg}/{repeats}")

    # Restore previous modes
    axis.controller.input_torque = 0.0
    axis.controller.config.control_mode = prev_cm
    axis.controller.config.input_mode   = prev_im

    # Decide pass/fail
    # Require directional evidence on BOTH sides; one-sided movement is inconclusive.
    pos_motion_ok = (float(avg_pos) >= float(min_avg_counts)) or (float(avg_pos_v) >= float(min_avg_vel))
    neg_motion_ok = ((-float(avg_neg)) >= float(min_avg_counts)) or ((-float(avg_neg_v)) >= float(min_avg_vel))
    motion_ok = bool(pos_motion_ok and neg_motion_ok)

    info = {
        "avg_pos": avg_pos,
        "avg_neg": avg_neg,
        "avg_pos_v": avg_pos_v,
        "avg_neg_v": avg_neg_v,
        "score_counts": score_counts,
        "score_vel": score_vel,
        "soft_aborts_pos": int(soft_ab_pos),
        "soft_aborts_neg": int(soft_ab_neg),
        "hard_aborts_pos": int(hard_ab_pos),
        "hard_aborts_neg": int(hard_ab_neg),
        "pos_motion_ok": bool(pos_motion_ok),
        "neg_motion_ok": bool(neg_motion_ok),
    }

    # Any hard-abort pulse means this run is not trustworthy for sign validation.
    if int(hard_ab_pos) > 0 or int(hard_ab_neg) > 0:
        info["reason"] = "inconclusive"
        info["inconclusive_cause"] = "hard_abort"
        return False, info

    if not motion_ok:
        info["reason"] = "inconclusive"
        info["inconclusive_cause"] = "insufficient_bidirectional_motion"
        return False, info

    sign_ok = (score_counts > 0.0) and (score_vel > 0.0)
    if not bool(sign_ok):
        info["reason"] = "sign_mismatch"
    return bool(sign_ok), info

def full_self_calibrate(axis,
                        current_lim=10.0,
                        motor_calib_current=None,
                        motor_calib_retries=2,
                        force_motor_calib=False,
                        force_encoder_calib=False,
                        encoder_calib_retries=1,
                        torque_nm=0.05,
                        torque_pulse_s=0.03,
                        torque_settle_s=0.18,
                        preload_deg=3.0,
                        preload_hold_s=0.30,
                        preload_settle_s=0.20,
                        preload_cycles=1,
                        require_absolute=True,
                        require_index=True,
                        run_index_search=True,
                        run_step_validation=True,
                        step_validation_deg=0.5,
                        run_auto_direction_contract=False,
                        auto_direction_cycles=4,
                        auto_direction_run_jump_vs_slip=False,
                        auto_direction_persist=False,
                        auto_direction_save_path=None):
    print("\n=== full_self_calibrate ===")
    
    try:
        enc_ready_entry = bool(getattr(axis.encoder, "is_ready", False))
    except Exception:
        enc_ready_entry = False
    try:
        idx_found_entry = getattr(axis.encoder, "index_found", None)
        if idx_found_entry is not None:
            idx_found_entry = bool(idx_found_entry)
    except Exception:
        idx_found_entry = None

    print(f"params: current_lim={current_lim}, motor_calib_current={motor_calib_current}, motor_calib_retries={motor_calib_retries}, force_motor_calib={force_motor_calib}, "
          f"force_encoder_calib={force_encoder_calib}, encoder_calib_retries={encoder_calib_retries}, "
          f"torque_nm={torque_nm}, pulse={torque_pulse_s}s, settle={torque_settle_s}s, preload={preload_deg}° x{preload_cycles} (hold {preload_hold_s}s, settle {preload_settle_s}s), "
          f"require_absolute={require_absolute}, require_index={require_index}, run_index_search={run_index_search}, "
          f"run_step_validation={run_step_validation}, step_validation_deg={step_validation_deg}, "
          f"run_auto_direction_contract={run_auto_direction_contract}, auto_direction_cycles={auto_direction_cycles}")
    print(f"entry_state: encoder.is_ready={enc_ready_entry} encoder.index_found={idx_found_entry}")

    # Conservative current limits
    try:
        axis.motor.config.current_lim = float(current_lim)
        axis.motor.config.current_lim_margin = max(1.0, float(current_lim) * 0.33)
    except Exception:
        pass

    ensure_idle_and_cleared(axis, tries=3, settle_s=0.25)

    # Motor calibration
    print("\n--- Motor calibration ---")

    # Skip motor calibration when we already have a usable motor model.
    # ODrive motor calibration (R/L) can fail intermittently (wiring, EMI, gearboxes attached,
    # long phase leads, etc.). If we already have valid phase R/L or the motor is marked
    # calibrated, we prefer to continue instead of blocking the rest of the bring-up tests.
    try:
        pre_cal = bool(axis.motor.config.pre_calibrated)
    except Exception:
        pre_cal = False

    try:
        is_cal = bool(getattr(axis.motor, "is_calibrated", False))
    except Exception:
        is_cal = False

    try:
        pr = float(getattr(axis.motor.config, "phase_resistance", 0.0))
    except Exception:
        pr = 0.0
    try:
        pl = float(getattr(axis.motor.config, "phase_inductance", 0.0))
    except Exception:
        pl = 0.0

    have_rl = (pr > 0.0) and (pl > 0.0)

    # If NOT forcing calibration, skip when:
    #  - motor is already calibrated OR
    #  - config says pre_calibrated OR
    #  - phase R/L are already present
    if (not bool(force_motor_calib)) and (is_cal or pre_cal or have_rl):
        print(
            "Motor already calibrated; skipping MOTOR_CALIBRATION "
            f"(is_calibrated={is_cal}, pre_calibrated={pre_cal}, "
            f"phase_resistance={pr:.6g}, phase_inductance={pl:.6g})."
        )
    else:
        # Some setups fail motor calibration if calibration_current is too low.
        # If motor_calib_current is not provided, choose a conservative value from current_lim.
        try:
            cur_lim_f = float(current_lim)
        except Exception:
            cur_lim_f = 10.0

        if motor_calib_current is None:
            mc0 = max(2.0, min(cur_lim_f * 0.7, cur_lim_f - 0.5))
        else:
            mc0 = float(motor_calib_current)
            mc0 = max(0.5, min(mc0, max(0.5, cur_lim_f - 0.5)))

        # Build a sweep. If a specific motor_calib_current was requested, try that exact
        # value first, then step upward. Otherwise, use fractions of current_lim.
        sweep = [mc0]
        if motor_calib_current is not None:
            sweep += [
                max(0.5, min(mc0 + 2.0, max(0.5, cur_lim_f - 0.5))),
                max(0.5, min(mc0 + 4.0, max(0.5, cur_lim_f - 0.5))),
                max(0.5, min(cur_lim_f * 0.95, max(0.5, cur_lim_f - 0.5))),
            ]
        else:
            sweep += [
                max(0.5, min(cur_lim_f * 0.85, cur_lim_f - 0.5)),
                max(0.5, min(cur_lim_f * 0.95, cur_lim_f - 0.5)),
            ]

        # De-dup while preserving order
        _seen = set()
        _sweep = []
        for x in sweep:
            k = round(float(x), 3)
            if k not in _seen:
                _seen.add(k)
                _sweep.append(float(x))
        sweep = _sweep

        attempts = 0
        max_attempts = max(1, int(motor_calib_retries) + 1)

        last_err = None
        for mc in sweep:
            if attempts >= max_attempts:
                break
            attempts += 1

            try:
                axis.motor.config.calibration_current = float(mc)
            except Exception:
                pass

            # Ensure we are not starting the calibration while faulted/latched
            ensure_idle_and_cleared(axis, tries=2, settle_s=0.15)

            axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
            _sleep_with_watchdog(axis, 0.2, step=0.02)
            while int(axis.current_state) != int(AXIS_STATE_IDLE):
                _sleep_with_watchdog(axis, 0.05, step=0.02)

            # Clear any latched bits so a subsequent attempt doesn't "fail instantly"
            clear_errors(axis)
            _sleep_with_watchdog(axis, 0.10, step=0.02)

            if int(axis.error) == 0 and int(axis.motor.error) == 0:
                last_err = None
                break

            last_err = errhex(axis)
            print(f"Motor calibration attempt {attempts}/{max_attempts} failed (calibration_current={mc:.2f}A):", last_err)

        if last_err is not None:
            # Refresh after attempts (it may have become valid even if the last attempt failed)
            try:
                is_cal = bool(getattr(axis.motor, "is_calibrated", False))
            except Exception:
                is_cal = False
            try:
                pr = float(getattr(axis.motor.config, "phase_resistance", 0.0))
            except Exception:
                pr = 0.0
            try:
                pl = float(getattr(axis.motor.config, "phase_inductance", 0.0))
            except Exception:
                pl = 0.0
            have_rl = (pr > 0.0) and (pl > 0.0)

            if (not bool(force_motor_calib)) and (is_cal or have_rl):
                print(
                    "Motor calibration failed, but existing calibration data is present; continuing. "
                    f"(is_calibrated={is_cal}, phase_resistance={pr:.6g}, phase_inductance={pl:.6g})"
                )
                clear_errors(axis)
            else:
                print("Motor calibration failed:", last_err)
                raise RuntimeError("motor calibration failed")

    if int(getattr(axis, "error", 0)) or int(getattr(axis.motor, "error", 0)) or int(getattr(axis.encoder, "error", 0)) or int(getattr(axis.controller, "error", 0)):
        print("Fault present before encoder calibration:", errhex(axis))
        raise RuntimeError("fault present before encoder calibration")

    # Absolute-first path:
    # When index is required and we will run index search, avoid forcing a standalone
    # offset calibration first. On some setups this is the flaky step (NO_RESPONSE),
    # while index-search + post-index recovery is stable.
    absolute_first_bootstrap = bool(require_absolute) and bool(require_index) and bool(run_index_search)

    if bool(absolute_first_bootstrap):
        print("\n--- Absolute reference contract (index-first bootstrap) ---")
        # Robust bootstrap for incremental+index setups:
        # 1) run encoder offset calibration with use_index=False (more reliable on some builds)
        # 2) mark encoder pre_calibrated
        # 3) restore use_index=True and run index search / absolute reference contract
        try:
            prev_use_index = bool(getattr(axis.encoder.config, "use_index", True))
        except Exception:
            prev_use_index = True

        try:
            axis.encoder.config.use_index = False
        except Exception:
            pass

        try:
            enc_ready_before = bool(getattr(axis.encoder, "is_ready", False))
        except Exception:
            enc_ready_before = False

        if bool(force_encoder_calib) or (not bool(enc_ready_before)):
            print("index bootstrap: running ENCODER_OFFSET_CALIBRATION with use_index=False")
            try:
                tries = max(1, int(encoder_calib_retries) + 1)
            except Exception:
                tries = 2

            enc_ok = False
            last_enc_err = None
            for attempt in range(1, tries + 1):
                ensure_idle_and_cleared(axis, tries=2, settle_s=0.15)
                axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
                _sleep_with_watchdog(axis, 0.2, step=0.02)

                t0 = time.time()
                while int(axis.current_state) != int(AXIS_STATE_IDLE):
                    if (time.time() - t0) > 25.0:
                        break
                    _sleep_with_watchdog(axis, 0.05, step=0.02)

                ax_e = int(getattr(axis, "error", 0))
                en_e = int(getattr(axis.encoder, "error", 0))
                try:
                    enc_ready_now = bool(getattr(axis.encoder, "is_ready", False))
                except Exception:
                    enc_ready_now = False

                if (ax_e == 0) and (en_e == 0) and bool(enc_ready_now):
                    enc_ok = True
                    break

                last_enc_err = errhex(axis)
                print(f"Encoder calibration attempt {attempt}/{tries} failed:", last_enc_err)
                clear_errors(axis)
                _sleep_with_watchdog(axis, 0.10, step=0.02)

            if not enc_ok:
                print("Encoder calibration failed:", last_enc_err if last_enc_err is not None else errhex(axis))
                raise RuntimeError("encoder calibration failed")
        else:
            print("index bootstrap: encoder already ready; skipping ENCODER_OFFSET_CALIBRATION.")

        try:
            axis.encoder.config.pre_calibrated = True
        except Exception:
            pass

        try:
            axis.encoder.config.use_index = True
        except Exception:
            try:
                axis.encoder.config.use_index = bool(prev_use_index)
            except Exception:
                pass

        clear_errors(axis)
        establish_absolute_reference(
            axis=axis,
            require_index=True,
            run_index_search=True,
            timeout_s=30.0,
            attempt_offset_calibration=False,
            recover_offset_after_index=True,
            label="full_self_calibrate",
        )
    else:
        # Encoder offset calibration
        print("\n--- Encoder offset calibration ---")
        try:
            enc_ready_before = bool(getattr(axis.encoder, "is_ready", False))
        except Exception:
            enc_ready_before = False

        # If encoder was already ready (at entry or now), do not force a new offset calibration unless requested.
        if (not bool(force_encoder_calib)) and (bool(enc_ready_before) or bool(enc_ready_entry)):
            print("Encoder already ready; skipping ENCODER_OFFSET_CALIBRATION.")
        else:
            try:
                tries = max(1, int(encoder_calib_retries) + 1)
            except Exception:
                tries = 2

            enc_ok = False
            last_enc_err = None
            for attempt in range(1, tries + 1):
                ensure_idle_and_cleared(axis, tries=2, settle_s=0.15)
                axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
                _sleep_with_watchdog(axis, 0.2, step=0.02)

                t0 = time.time()
                while int(axis.current_state) != int(AXIS_STATE_IDLE):
                    if (time.time() - t0) > 25.0:
                        break
                    _sleep_with_watchdog(axis, 0.05, step=0.02)

                ax_e = int(getattr(axis, "error", 0))
                en_e = int(getattr(axis.encoder, "error", 0))
                try:
                    enc_ready_now = bool(getattr(axis.encoder, "is_ready", False))
                except Exception:
                    enc_ready_now = True

                if (ax_e == 0) and (en_e == 0) and bool(enc_ready_now):
                    enc_ok = True
                    break

                last_enc_err = errhex(axis)
                print(f"Encoder calibration attempt {attempt}/{tries} failed:", last_enc_err)
                clear_errors(axis)
                _sleep_with_watchdog(axis, 0.10, step=0.02)

            if not enc_ok:
                # Recovery path: if firmware already reports is_ready=True now, continue unless forced.
                try:
                    enc_ready_now = bool(getattr(axis.encoder, "is_ready", False))
                except Exception:
                    enc_ready_now = False

                if (not bool(force_encoder_calib)) and bool(enc_ready_now):
                    print("Encoder calibration attempts failed, but encoder.is_ready=True; continuing.")
                    clear_errors(axis)
                else:
                    print("Encoder calibration failed:", last_enc_err if last_enc_err is not None else errhex(axis))
                    raise RuntimeError("encoder calibration failed")

        if int(getattr(axis, "error", 0)) or int(getattr(axis.motor, "error", 0)) or int(getattr(axis.encoder, "error", 0)) or int(getattr(axis.controller, "error", 0)):
            print("Fault present before closed-loop entry:", errhex(axis))
            raise RuntimeError("fault present before closed-loop entry")

        # Closed loop in POSITION + PASSTHROUGH (bringup mode)
        print("\n--- Enter closed loop (POSITION + PASSTHROUGH) ---")
        axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        axis.controller.config.input_mode   = INPUT_MODE_PASSTHROUGH
        sync_to_est(axis)
        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        _sleep_with_watchdog(axis, 0.35, step=0.02)
        sync_to_est(axis)

        if bool(require_absolute):
            print("\n--- Absolute reference contract ---")
            establish_absolute_reference(
                axis=axis,
                require_index=bool(require_index),
                run_index_search=bool(run_index_search),
                timeout_s=30.0,
                attempt_offset_calibration=False,
                label="full_self_calibrate",
            )

    if bool(run_auto_direction_contract):
        print("\n--- Auto direction contract ---")
        # Keep this low-stress: enough authority to classify direction, not enough to excite runaways.
        try:
            adc_current_lim = max(6.0, min(float(current_lim), 12.0))
        except Exception:
            adc_current_lim = 8.0

        adc = run_auto_direction_contract_selector(
            axis=axis,
            candidate_directions=(-1, 1),
            cycles=max(2, int(auto_direction_cycles)),
            cmd_delta_turns=0.01,
            current_lim=float(adc_current_lim),
            pos_gain=16.0,
            vel_gain=0.24,
            vel_i_gain=0.0,
            vel_limit=0.60,
            settle_between_s=0.12,
            run_jump_vs_slip=bool(auto_direction_run_jump_vs_slip),
            jump_hold_s=3.0,
            persist=bool(auto_direction_persist),
            save_path=auto_direction_save_path,
            verbose=True,
        )
        print(
            "auto_direction_contract result:",
            {
                "ok": bool(adc.get("ok")),
                "selected_direction": adc.get("selected_direction"),
                "confidence": adc.get("confidence"),
                "classification": (dict(adc.get("summary") or {}).get("winner_classification")),
            },
        )
        if not bool(adc.get("ok")):
            raise RuntimeError(
                "auto_direction_contract failed to establish stable sign consistency; "
                "fix hardware/sign path before aggressive calibration."
            )
        if not _enter_closed_loop_position(axis, retries=3, settle_s=0.10):
            print("Failed to enter closed loop after auto_direction_contract:", errhex(axis))
            raise RuntimeError("closed loop entry failed (post auto_direction_contract)")

    # Preload/dither to break stiction (especially with gearboxes/harmonic drives)
    if float(preload_deg) != 0.0 and float(preload_cycles) > 0:
        print("\n--- Preload (break stiction) ---")
        print(f"preload: {float(preload_deg):.2f}° x{int(preload_cycles)} (hold {float(preload_hold_s):.2f}s, settle {float(preload_settle_s):.2f}s)")

        # Ensure loop is armed before preload.
        if not _enter_closed_loop_position(axis, retries=3, settle_s=0.10):
            print("Failed to enter closed loop before preload:", errhex(axis))
            raise RuntimeError("closed loop entry failed (preload setup)")

        preload_candidates = [float(preload_deg), float(preload_deg) * 0.5, float(preload_deg) * 0.25]
        preload_ok = False
        for i, d in enumerate(preload_candidates, start=1):
            try:
                preload_break_stiction(
                    axis,
                    deg=float(d),
                    hold_s=float(preload_hold_s),
                    settle_s=float(preload_settle_s),
                    cycles=int(preload_cycles),
                )
            except Exception:
                pass

            ax_e = int(getattr(axis, "error", 0))
            m_e = int(getattr(axis.motor, "error", 0))
            e_e = int(getattr(axis.encoder, "error", 0))
            c_e = int(getattr(axis.controller, "error", 0))
            st = int(getattr(axis, "current_state", 0))
            if st == int(AXIS_STATE_CLOSED_LOOP_CONTROL) and ax_e == 0 and m_e == 0 and e_e == 0 and c_e == 0:
                preload_ok = True
                break

            diag = errhex(axis)
            print(f"preload attempt {i}/{len(preload_candidates)} failed at {float(d):.2f}°:", diag)
            clear_errors(axis)
            _sleep_with_watchdog(axis, 0.10, step=0.02)
            if not _enter_closed_loop_position(axis, retries=3, settle_s=0.10):
                break

        if not preload_ok:
            print("preload: skipped after fault(s); continuing without preload.")
        sync_to_est(axis, repeats=2, dt=0.08)

    if int(axis.current_state) != int(AXIS_STATE_CLOSED_LOOP_CONTROL) or int(axis.error):
        print("Closed-loop unhealthy after preload; attempting recovery re-entry.")
        if not _enter_closed_loop_position(axis, retries=3, settle_s=0.10):
            print("Failed to enter closed loop:", errhex(axis))
            raise RuntimeError("closed loop entry failed")
        print("Closed-loop recovered.")

    # Torque sign sanity (decisive when it moves; adaptive when it doesn't)
    print("\n--- Torque sign sanity ---")
    # Quick hint: extremely low pos_gain/current limits often make motion checks inconclusive,
    # especially with gearboxes/harmonic drives.
    try:
        _pg0 = float(axis.controller.config.pos_gain)
        _cl0 = float(axis.motor.config.current_lim)
        if _pg0 < 2.0 or _cl0 < 8.0:
            print(f"Note: pos_gain={_pg0:.3g}, current_lim={_cl0:.3g} -> may be too low for motion checks (gearboxes/stiction).")
    except Exception:
        pass

    # Save/restore current limit during adaptive sweeps
    try:
        _cur_lim_prev = float(axis.motor.config.current_lim)
    except Exception:
        _cur_lim_prev = None

    ok, info = torque_sign_sanity(axis,
                                 torque_nm=float(torque_nm),
                                 pulse_s=float(torque_pulse_s),
                                 settle_s=float(torque_settle_s),
                                 repeats=8)

    # If pulses produce no measurable motion (common with harmonic drives), sweep more authority.
    if (not ok) and isinstance(info, dict) and info.get("reason") == "inconclusive":
        print("Torque sign sanity inconclusive (very low motion). Running adaptive sweep...")

        sweep = [
            # (current_lim, torque_nm, pulse_s, settle_s)
            (10.0, max(0.08, float(torque_nm) * 1.5), max(0.05, float(torque_pulse_s) * 1.5), max(0.18, float(torque_settle_s))),
            (15.0, max(0.12, float(torque_nm) * 2.0), max(0.07, float(torque_pulse_s) * 2.0), max(0.20, float(torque_settle_s) * 1.1)),
            (20.0, max(0.16, float(torque_nm) * 2.5), max(0.09, float(torque_pulse_s) * 2.5), max(0.22, float(torque_settle_s) * 1.2)),
            (25.0, max(0.20, float(torque_nm) * 3.0), max(0.12, float(torque_pulse_s) * 3.0), max(0.25, float(torque_settle_s) * 1.3)),
        ]

        best_ok, best_info = ok, info
        best_score = abs(float(info.get("avg_pos", 0.0))) + abs(float(info.get("avg_neg", 0.0)))

        for cur_lim, tnm, ps, ss in sweep:
            try:
                axis.motor.config.current_lim = float(cur_lim)
                axis.motor.config.current_lim_margin = max(1.0, float(cur_lim) * 0.33)
            except Exception:
                pass

            ok2, info2 = torque_sign_sanity(axis,
                                            torque_nm=float(tnm),
                                            pulse_s=float(ps),
                                            settle_s=float(ss),
                                            repeats=8)

            score2 = 0.0
            if isinstance(info2, dict):
                score2 = abs(float(info2.get("avg_pos", 0.0))) + abs(float(info2.get("avg_neg", 0.0)))

            if score2 > best_score:
                best_ok, best_info, best_score = ok2, info2, score2

            # If we got a decisive result (ok==True OR mismatch), stop sweeping.
            if ok2 or (isinstance(info2, dict) and info2.get("reason") != "inconclusive"):
                best_ok, best_info = ok2, info2
                break

        ok, info = best_ok, best_info

        # Restore current limit after sweep
        if _cur_lim_prev is not None:
            try:
                axis.motor.config.current_lim = float(_cur_lim_prev)
                axis.motor.config.current_lim_margin = max(1.0, float(_cur_lim_prev) * 0.33)
            except Exception:
                pass

    # If still inconclusive, do a POSITION step direction check (works with stiction/gearboxes)
    if (not ok) and isinstance(info, dict) and info.get("reason") == "inconclusive":
        print("Torque sign still inconclusive. Falling back to POSITION step direction check...")

        # Temporarily increase authority for stiction/gearboxes, then restore.
        try:
            _cur_lim_prev2 = float(axis.motor.config.current_lim)
        except Exception:
            _cur_lim_prev2 = None
        try:
            _pos_gain_prev = float(axis.controller.config.pos_gain)
        except Exception:
            _pos_gain_prev = None
        try:
            _vel_limit_prev = float(axis.controller.config.vel_limit)
        except Exception:
            _vel_limit_prev = None
        try:
            _vel_gain_prev = float(axis.controller.config.vel_gain)
        except Exception:
            _vel_gain_prev = None
        try:
            _vel_i_gain_prev = float(axis.controller.config.vel_integrator_gain)
        except Exception:
            _vel_i_gain_prev = None
        try:
            _vel_tol_prev = float(axis.controller.config.vel_limit_tolerance)
        except Exception:
            _vel_tol_prev = None
        try:
            _overs_prev = bool(axis.controller.config.enable_overspeed_error)
        except Exception:
            _overs_prev = None

        def _restore_dir_check_cfg():
            if _cur_lim_prev2 is not None:
                try:
                    axis.motor.config.current_lim = float(_cur_lim_prev2)
                    axis.motor.config.current_lim_margin = max(1.0, float(_cur_lim_prev2) * 0.33)
                except Exception:
                    pass
            if _pos_gain_prev is not None:
                try:
                    axis.controller.config.pos_gain = float(_pos_gain_prev)
                except Exception:
                    pass
            if _vel_limit_prev is not None:
                try:
                    axis.controller.config.vel_limit = float(_vel_limit_prev)
                except Exception:
                    pass
            if _vel_gain_prev is not None:
                try:
                    axis.controller.config.vel_gain = float(_vel_gain_prev)
                except Exception:
                    pass
            if _vel_i_gain_prev is not None:
                try:
                    axis.controller.config.vel_integrator_gain = float(_vel_i_gain_prev)
                except Exception:
                    pass
            if _vel_tol_prev is not None:
                try:
                    axis.controller.config.vel_limit_tolerance = float(_vel_tol_prev)
                except Exception:
                    pass
            if _overs_prev is not None:
                try:
                    axis.controller.config.enable_overspeed_error = bool(_overs_prev)
                except Exception:
                    pass

        # Conservative bump: enough to move a harmonic drive, but not crazy.
        try:
            axis.motor.config.current_lim = max(12.0, float(axis.motor.config.current_lim))
            axis.motor.config.current_lim_margin = max(1.0, float(axis.motor.config.current_lim) * 0.33)
        except Exception:
            pass
        try:
            _pg = float(axis.controller.config.pos_gain)
            # Keep this moderate to avoid overshoot/disarm while still breaking stiction.
            axis.controller.config.pos_gain = min(20.0, max(4.0, _pg * 4.0))
        except Exception:
            pass
        try:
            _vg = float(axis.controller.config.vel_gain)
            axis.controller.config.vel_gain = min(0.35, max(0.08, _vg * 1.25))
        except Exception:
            pass
        try:
            # Avoid integrator wind-up during this one-shot sanity move.
            axis.controller.config.vel_integrator_gain = 0.0
        except Exception:
            pass
        try:
            # Low external vel_limit settings (e.g. 0.25) can falsely trip overspeed here.
            axis.controller.config.vel_limit = max(2.0, float(axis.controller.config.vel_limit))
        except Exception:
            pass
        try:
            axis.controller.config.vel_limit_tolerance = max(4.0, float(axis.controller.config.vel_limit_tolerance))
        except Exception:
            pass
        try:
            axis.controller.config.enable_overspeed_error = False
        except Exception:
            pass

        try:
            # Ensure we're in position passthrough and synced
            axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
            axis.controller.config.input_mode   = INPUT_MODE_PASSTHROUGH
            sync_to_est(axis, repeats=3, dt=0.08)
            _sleep_with_watchdog(axis, 0.10, step=0.02)

            cpr = int(axis.encoder.config.cpr)
            q0 = int(axis.encoder.shadow_count)
            p0 = float(axis.encoder.pos_estimate)

            # Command a larger step to overcome stiction (gearboxes/harmonic drives)
            cmd_deg = 30.0
            axis.controller.input_pos = p0 + (cmd_deg / 360.0)
            watch(axis, seconds=1.00, vel_abort=5.0, iq_abort=20.0)

            q1 = int(axis.encoder.shadow_count)
            p1 = float(axis.encoder.pos_estimate)
            dq = q1 - q0
            dp_deg = (p1 - p0) * 360.0

            expected = int(round(cpr * (cmd_deg / 360.0)))
            print(f"Direction sanity: cmd +{cmd_deg:.1f}° -> Δshadow_count {dq} (expected ≈ +{expected}), Δpos {dp_deg:.2f}°")

            # Wrong sign is dangerous (positive feedback)
            if dq < -max(3, expected // 4):
                print("FAIL: Encoder decreased on a +position command -> direction mismatch (positive feedback risk).")
                print("Fix: swap encoder A/B OR swap any two motor phases, then rerun.")
                axis.requested_state = AXIS_STATE_IDLE
                raise RuntimeError("torque sign sanity failed (direction mismatch)")

            if abs(dq) < 2 and abs(dp_deg) < 0.25:
                print("Still no measurable motion. Increase current_lim/torque or reduce stiction, then rerun.")
                axis.requested_state = AXIS_STATE_IDLE
                raise RuntimeError("torque sign sanity failed (still inconclusive)")

            print("PASS: Direction sign looks safe (no positive feedback detected).")
            ok = True
            info = {
                "reason": "direction_sanity_only",
                "delta_shadow_count": int(dq),
                "expected_shadow_count": int(expected),
                "delta_pos_deg": float(dp_deg),
                "note": "Torque-pulse sweep was inconclusive; sign validated via position-step fallback.",
            }
        finally:
            _restore_dir_check_cfg()

    if not ok:
        print("Torque sign sanity FAILED:", info)
        print("If sign mismatch: swap any two motor phases OR swap encoder A/B, then rerun.")
        try:
            axis.requested_state = AXIS_STATE_IDLE
        except Exception:
            pass
        _sleep_with_watchdog(axis, 0.20, step=0.02)
        clear_errors(axis)
        _sleep_with_watchdog(axis, 0.10, step=0.02)
        raise RuntimeError("torque sign sanity failed")

    if isinstance(info, dict) and info.get("reason") == "inconclusive":
        # Defensive: this should not happen when ok==True, but keep messaging explicit.
        print("WARN: torque sign sanity remained inconclusive:", info)
    elif isinstance(info, dict) and info.get("reason") == "direction_sanity_only":
        print("PASS: torque direction sanity OK (position-step fallback):", info)
    else:
        print("PASS: torque sign sanity OK:", info)

    if not bool(run_step_validation):
        print("\n--- Step validation skipped by request ---")
    else:
        # Resync + small step validation (still PASSTHROUGH, no TRAP_TRAJ)
        # Use temporarily gentle loop gains to avoid false current-limit trips on unloaded motors.
        print(f"\n--- +{float(step_validation_deg):.3g}° step validation (PASSTHROUGH) ---")
        try:
            _pg_prev = float(axis.controller.config.pos_gain)
        except Exception:
            _pg_prev = None
        try:
            _vg_prev = float(axis.controller.config.vel_gain)
        except Exception:
            _vg_prev = None
        try:
            _vig_prev = float(axis.controller.config.vel_integrator_gain)
        except Exception:
            _vig_prev = None

        try:
            if _pg_prev is not None:
                axis.controller.config.pos_gain = min(float(_pg_prev), 8.0)
        except Exception:
            pass
        try:
            if _vg_prev is not None:
                axis.controller.config.vel_gain = min(float(_vg_prev), 0.12)
        except Exception:
            pass
        try:
            axis.controller.config.vel_integrator_gain = 0.0
        except Exception:
            pass
        try:
            axis.controller.vel_integrator_torque = 0.0
        except Exception:
            pass

        sync_to_est(axis)
        _sleep_with_watchdog(axis, 0.05, step=0.02)
        step_deg(axis, float(step_validation_deg))
        try:
            _iq_abort = max(6.0, float(axis.motor.config.current_lim) * 1.25)
        except Exception:
            _iq_abort = 6.0
        try:
            watch(axis, seconds=0.8, vel_abort=1.0, iq_abort=_iq_abort)
            print(f"PASS: stayed in closed loop after +{float(step_validation_deg):.3g}°")
        finally:
            # Restore user gains after validation.
            if _pg_prev is not None:
                try:
                    axis.controller.config.pos_gain = float(_pg_prev)
                except Exception:
                    pass
            if _vg_prev is not None:
                try:
                    axis.controller.config.vel_gain = float(_vg_prev)
                except Exception:
                    pass
            if _vig_prev is not None:
                try:
                    axis.controller.config.vel_integrator_gain = float(_vig_prev)
                except Exception:
                    pass

    # Save configuration if available
    try:
        odrv0.save_configuration()
        print("Saved configuration")
    except Exception:
        print("Note: save_configuration() not supported by this firmware/interface")

    # Final state hygiene:
    # Some firmware paths leave a benign post-procedure disarm latched
    # (AXIS_ERROR_MOTOR_DISARMED + MOTOR_ERROR_CONTROL_DEADLINE_MISSED)
    # even though calibration succeeded. Clear that pair automatically so
    # the next diagnostic stage starts from a clean slate.
    try:
        axis.requested_state = AXIS_STATE_IDLE
    except Exception:
        pass
    _sleep_with_watchdog(axis, 0.10, step=0.02)

    try:
        ax_e = int(getattr(axis, "error", 0))
        m_e = int(getattr(axis.motor, "error", 0))
        e_e = int(getattr(axis.encoder, "error", 0))
        c_e = int(getattr(axis.controller, "error", 0))
    except Exception:
        ax_e, m_e, e_e, c_e = 0, 0, 0, 0

    benign_post = (
        ax_e == int(AXIS_ERROR_MOTOR_DISARMED)
        and m_e == int(MOTOR_ERROR_CONTROL_DEADLINE_MISSED)
        and e_e == 0
        and c_e == 0
    )
    if bool(benign_post):
        clear_errors(axis)
        _sleep_with_watchdog(axis, 0.05, step=0.02)
        print("Cleared benign post-calibration disarm (MOTOR_DISARMED + CONTROL_DEADLINE_MISSED).")
    elif any((ax_e, m_e, e_e, c_e)):
        print("Warning: non-benign error remains after calibration:", errhex(axis))

    print("\n=== DONE ===")
    return True
