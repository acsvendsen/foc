# tests1.py
# Run inside odrivetool:
#   %run -i tests1.py

import time
from IPython import get_ipython
from odrive.enums import *

from common import st, tame_motion

import math

# --- Error decoding helpers (works even when firmware lacks axis.dump_errors()) ---

def _decode_bitmask(value: int, enum_type):
    try:
        v = int(value)
    except Exception:
        v = 0
    if v == 0:
        return ["NO_ERROR"]

    names = []
    for name in dir(enum_type):
        if name.startswith("_"):
            continue
        try:
            bit = int(getattr(enum_type, name))
        except Exception:
            continue
        if bit != 0 and (v & bit) == bit:
            names.append(name)

    if not names:
        names = [f"UNKNOWN_BITS_{hex(v)}"]
    return names


def print_errors(axis):
    """Print axis/motor/encoder/controller errors with best-effort decode."""
    try:
        ax = int(getattr(axis, "error", 0))
    except Exception:
        ax = 0
    try:
        mo = int(getattr(axis.motor, "error", 0))
    except Exception:
        mo = 0
    try:
        en = int(getattr(axis.encoder, "error", 0))
    except Exception:
        en = 0
    try:
        co = int(getattr(axis.controller, "error", 0))
    except Exception:
        co = 0

    print("\nErrors (raw):")
    print("  axis.error       =", hex(ax), ax)
    print("  motor.error      =", hex(mo), mo)
    print("  encoder.error    =", hex(en), en)
    print("  controller.error =", hex(co), co)

    # Decode using enums from odrive.enums (best-effort)
    try:
        print("Errors (decoded):")
        print("  AxisError:", ", ".join(_decode_bitmask(ax, AxisError)))
        print("  MotorError:", ", ".join(_decode_bitmask(mo, MotorError)))
        print("  EncoderError:", ", ".join(_decode_bitmask(en, EncoderError)))
        print("  ControllerError:", ", ".join(_decode_bitmask(co, ControllerError)))
    except Exception:
        pass


def clear_all_errors(axis):
    """Clear errors across axis submodules in a firmware-tolerant way."""
    try:
        axis.clear_errors()
    except Exception:
        pass
    for obj in (axis, getattr(axis, "motor", None), getattr(axis, "encoder", None), getattr(axis, "controller", None)):
        if obj is None:
            continue
        try:
            obj.error = 0
        except Exception:
            pass

# Get odrv0 from odrivetool interactive namespace
ip = get_ipython()
odrv0 = ip.user_ns.get("odrv0") if ip else None

if odrv0 is None:
    raise RuntimeError(
        "odrv0 not found. Start odrivetool and wait for 'Connected ... as odrv0', then run: %run -i tests1.py"
    )

axis = odrv0.axis0

print("use_index:", getattr(axis.encoder.config, "use_index", None))
print("startup_encoder_index_search:", getattr(axis.config, "startup_encoder_index_search", None))
print("find_idx_on_lockin:", getattr(axis.encoder.config, "find_idx_on_lockin", None))
print("shadow_count:", int(axis.encoder.shadow_count))
print("pos_estimate:", float(axis.encoder.pos_estimate))

# ---- Control mode for bringup ----
# TRAP_TRAJ is great once the loop is stable, but during bringup it can retain/lag internal setpoints
# and combined with earlier torque pulses it can cause a large first-step jump (overspeed/controller_failed).
# So we start in POSITION+PASSTHROUGH and only enable TRAP_TRAJ after small-step stability is confirmed.
USE_TRAP_TRAJ = False

axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
axis.controller.config.input_mode   = INPUT_MODE_PASSTHROUGH if not USE_TRAP_TRAJ else INPUT_MODE_TRAP_TRAJ

# --- Encoder / estimation ---
# With ~1024 CPR (≈0.35°/count), enable interpolation to reduce quantization jitter.
try:
    axis.encoder.config.enable_phase_interpolation = True
except Exception:
    pass

# If Z/index is wired/noisy, it can reset counts mid-move and look like a huge jump.
# Disable index usage for this test (you can re-enable later once Z is verified clean).
try:
    axis.encoder.config.use_index = False
except Exception:
    pass
try:
    axis.config.startup_encoder_index_search = False
except Exception:
    pass
try:
    axis.config.startup_encoder_offset_calibration = False
except Exception:
    pass

# --- Bringup tuning profiles ---
# We keep three profiles:
#  - profile 0: ultra-safe (very low authority)
#  - profile 1: moderate (usually enough to overcome cogging)
#  - profile 2: strong (still safe-ish, but can move briskly)
#
# IMPORTANT: If motor/encoder direction is wrong, closed-loop becomes positive feedback and will run away.
# So we verify direction first, then (if motion is too weak) we step up authority.

# --- Torque constant helper ---
# Many firmwares interpret `input_torque` in Nm using motor.config.torque_constant.
# If torque_constant is unset/incorrect, torque pulses can be far too small and look like "no motion".
DEFAULT_KV = 140.0

def ensure_torque_constant(axis, kv: float = DEFAULT_KV):
    """Ensure motor.config.torque_constant is set to a sane value (Nm/A).

    For a BLDC motor: Kt ~= 60 / (2*pi*Kv) Nm/A.
    If the firmware rejects the write, we just continue.
    """
    try:
        existing = float(getattr(axis.motor.config, "torque_constant", 0.0))
    except Exception:
        existing = 0.0

    # If it already looks sane, keep it.
    if existing and 0.005 < abs(existing) < 1.0:
        return existing

    kt = 60.0 / (2.0 * math.pi * float(kv))
    try:
        axis.motor.config.torque_constant = float(kt)
    except Exception:
        pass

    try:
        return float(getattr(axis.motor.config, "torque_constant", kt))
    except Exception:
        return kt


def with_temp_limits(axis, *, current_lim=None, vel_limit=None):
    """Context-like helper: returns a restore() function after applying temporary limits."""
    prev = {}
    if current_lim is not None:
        try:
            prev["current_lim"] = float(axis.motor.config.current_lim)
            axis.motor.config.current_lim = float(current_lim)
        except Exception:
            pass
    if vel_limit is not None:
        try:
            prev["vel_limit"] = float(axis.controller.config.vel_limit)
            axis.controller.config.vel_limit = float(vel_limit)
        except Exception:
            pass

    def restore():
        if "current_lim" in prev:
            try:
                axis.motor.config.current_lim = float(prev["current_lim"])
            except Exception:
                pass
        if "vel_limit" in prev:
            try:
                axis.controller.config.vel_limit = float(prev["vel_limit"])
            except Exception:
                pass

    return restore

def apply_profile(level: int = 0):
    level = int(level)
    # Ensure torque constant is set for meaningful torque mode tests
    ensure_torque_constant(axis, kv=DEFAULT_KV)

    if level <= 0:
        # Ultra-safe bringup: low authority but *functional* position control.
        # NOTE: In ODrive position mode, pos_gain generates a velocity setpoint,
        # and vel_gain / vel_integrator_gain generate torque/current from velocity error.
        # If vel_gain == 0 and integrator == 0, the axis will NOT produce torque.
        axis.controller.config.pos_gain = 6.0
        axis.controller.config.vel_gain = 0.05
        axis.controller.config.vel_integrator_gain = 0.10
        try:
            axis.controller.config.vel_integrator_limit = 3.0
        except Exception:
            pass

        # Light filtering to reduce quantization jitter without adding too much lag.
        axis.controller.config.input_filter_bandwidth = 10.0

        # Tight safety limits.
        axis.controller.config.vel_limit = 0.35
        axis.trap_traj.config.vel_limit = 0.3
        axis.trap_traj.config.accel_limit = 1.0
        axis.trap_traj.config.decel_limit = 1.0

        # Conservative current.
        axis.motor.config.current_lim = 10.0
        axis.motor.config.current_lim_margin = 2.0

    elif level == 1:
        # Moderate: enough authority to overcome light cogging/stiction.
        axis.controller.config.pos_gain = 18.0
        axis.controller.config.vel_gain = 0.12
        axis.controller.config.vel_integrator_gain = 0.25
        try:
            axis.controller.config.vel_integrator_limit = 4.0
        except Exception:
            pass

        axis.controller.config.input_filter_bandwidth = 8.0

        axis.controller.config.vel_limit = 1.0
        axis.trap_traj.config.vel_limit = 0.6
        axis.trap_traj.config.accel_limit = 2.0
        axis.trap_traj.config.decel_limit = 2.0

        axis.motor.config.current_lim = 8.0
        axis.motor.config.current_lim_margin = 3.0

    else:
        # Strong-ish: still conservative, but should move reliably.
        axis.controller.config.pos_gain = 25.0
        axis.controller.config.vel_gain = 0.18
        axis.controller.config.vel_integrator_gain = 0.40
        try:
            axis.controller.config.vel_integrator_limit = 6.0
        except Exception:
            pass

        axis.controller.config.input_filter_bandwidth = 10.0

        axis.controller.config.vel_limit = 2.0
        axis.trap_traj.config.vel_limit = 1.2
        axis.trap_traj.config.accel_limit = 4.0
        axis.trap_traj.config.decel_limit = 4.0

        axis.motor.config.current_lim = 12.0
        axis.motor.config.current_lim_margin = 4.0
def _feed_watchdog(axis):
    try:
        if hasattr(axis, "watchdog_feed"):
            axis.watchdog_feed()
    except Exception:
        pass


def _sleep_with_watchdog(axis, seconds: float, step: float = 0.02):
    """Sleep in small increments while feeding the watchdog.

    Uses a remaining-time calculation to avoid negative sleeps caused by timing jitter.
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
        elapsed = time.time() - t0
        remaining = total - elapsed
        if remaining <= 0.0:
            break
        time.sleep(min(step_s, remaining))


def watch(seconds=1.0, dt=0.02, vel_abort=1.0, iq_abort=None, stall_ms=600, stall_counts=2, jump_abort_frac=0.25):
    """Print a short status stream and abort on runaway OR on stall (Iq rising but encoder not moving)."""
    cpr = int(axis.encoder.config.cpr)
    last_shadow = int(axis.encoder.shadow_count)
    last_counts = int(round(float(axis.encoder.pos_estimate) * cpr))
    last_move_t = time.time()

    t0 = time.time()
    while time.time() - t0 < seconds:
        pos_set = float(axis.controller.pos_setpoint)
        in_pos  = float(axis.controller.input_pos)
        pos_est = float(axis.encoder.pos_estimate)
        iq_set  = float(axis.motor.current_control.Iq_setpoint)
        iq_meas = float(axis.motor.current_control.Iq_measured)
        state   = int(axis.current_state)
        ax_e    = int(axis.error)
        m_e     = int(axis.motor.error)
        c_e     = int(axis.controller.error)

        print(
            "in:", in_pos,
            "set:", pos_set,
            "est:", pos_est,
            "Iq_set:", iq_set,
            "Iq_meas:", iq_meas,
            "state:", state,
            "axis_err:", ax_e,
            "motor_err:", m_e,
            "ctrl_err:", c_e,
        )

        # Hard abort on runaway current (scale default threshold from current_lim)
        if iq_abort is None:
            try:
                # Default: 85% of current_lim, but capped for safety during bringup.
                iq_abort_eff = 0.85 * float(axis.motor.config.current_lim)
            except Exception:
                iq_abort_eff = 6.0
            # Cap so temporary current_lim boosts don't disable runaway protection.
            iq_abort_eff = min(iq_abort_eff, 10.0)
        else:
            iq_abort_eff = float(iq_abort)

        if abs(iq_meas) > iq_abort_eff:
            print("SAFETY ABORT: runaway iq -> IDLE")
            axis.requested_state = AXIS_STATE_IDLE
            raise RuntimeError("Safety abort: runaway")

        # Abort if we leave closed loop or fault (but print errors first)
        if state != int(AXIS_STATE_CLOSED_LOOP_CONTROL):
            print("Dropped out of closed loop; going IDLE")
            try:
                print_errors(axis)
            except Exception:
                pass
            axis.requested_state = AXIS_STATE_IDLE
            raise RuntimeError("Safety abort: left closed loop")
        if ax_e != 0 or m_e != 0 or c_e != 0:
            print("FAULT detected; going IDLE")
            axis.requested_state = AXIS_STATE_IDLE
            try:
                print("  axis.error:", ax_e, "motor.error:", m_e, "ctrl.error:", c_e)
            except Exception:
                pass
            raise RuntimeError("Safety abort: fault detected")

        # Stall detection: if encoder isn't moving but Iq ramps for too long, stop.
        now_counts = int(round(pos_est * cpr))
        if abs(now_counts - last_counts) >= stall_counts:
            last_counts = now_counts
            last_move_t = time.time()
        else:
            if (time.time() - last_move_t) * 1000.0 > stall_ms and abs(iq_meas) > 1.2:
                print("SAFETY ABORT: stall (no encoder motion while Iq high) -> IDLE")
                axis.requested_state = AXIS_STATE_IDLE
                raise RuntimeError("Safety abort: stall")

        # Detect suspicious count resets/jumps (often caused by index/Z or wiring noise)
        sh = int(axis.encoder.shadow_count)
        jump_thresh = max(4, int(cpr * float(jump_abort_frac)))
        if abs(sh - last_shadow) > jump_thresh:
            print(f"SAFETY ABORT: shadow_count jump {last_shadow} -> {sh} (Δ {sh-last_shadow}) exceeds {jump_thresh}. Likely Z/index/noise or wiring glitch -> IDLE")
            axis.requested_state = AXIS_STATE_IDLE
            raise RuntimeError("Safety abort: encoder jump")
        last_shadow = sh

        # Feed watchdog if enabled (prevents unwanted timeout during long watches)
        _feed_watchdog(axis)
        time.sleep(dt)


def sync_to_est(axis, repeats: int = 2, dt: float = 0.08):
    """Align commanded position to the current estimate to avoid jumps.

    Some firmware builds update setpoints asynchronously; additionally, in some edge cases
    `input_pos` may not latch immediately. This routine writes repeatedly and verifies.
    """
    for _ in range(int(repeats)):
        pe = float(axis.encoder.pos_estimate)
        try:
            axis.controller.input_pos = pe
        except Exception:
            pass
        time.sleep(float(dt))

    # Verify and re-apply if needed
    try:
        for _ in range(25):
            pe = float(axis.encoder.pos_estimate)
            ip = float(axis.controller.input_pos)
            if abs(ip - pe) > 0.01:  # turns (~3.6°)
                axis.controller.input_pos = pe
                time.sleep(0.02)
                continue
            try:
                ps = float(axis.controller.pos_setpoint)
                if abs(ps - pe) > 0.01:
                    axis.controller.input_pos = pe
                    time.sleep(0.02)
                    continue
            except Exception:
                pass
            break
    except Exception:
        pass

def reenter_closed_loop_position(axis, settle_s: float = 0.25, sync_repeats: int = 4):
    """Re-enter CLOSED_LOOP cleanly in POSITION+PASSTHROUGH and force setpoint alignment.

    Important after torque-control pulses: some firmwares can leave a stale `pos_setpoint`
    that lags behind `input_pos`, causing an immediate controller_failed/overspeed on the
    next position move.
    """
    # Go IDLE first
    axis.requested_state = AXIS_STATE_IDLE
    time.sleep(0.15)

    clear_all_errors(axis)

    # Force safe position passthrough
    axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH

    # Enter closed loop and repeatedly sync to the live estimate
    axis.controller.input_pos = float(axis.encoder.pos_estimate)
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(float(settle_s))

    # Re-sync a few times to make sure pos_setpoint catches up
    sync_to_est(axis, repeats=int(sync_repeats), dt=0.06)

    # Wait briefly for pos_setpoint to follow (if exposed)
    try:
        for _ in range(20):
            ps = float(axis.controller.pos_setpoint)
            pe = float(axis.encoder.pos_estimate)
            if abs(ps - pe) < 0.003:  # turns (~1.1°)
                break
            axis.controller.input_pos = pe
            time.sleep(0.03)
    except Exception:
        pass

    # Final explicit alignment (helps on firmwares where pos_setpoint lags one cycle)
    try:
        pe = float(axis.encoder.pos_estimate)
        axis.controller.input_pos = pe
        time.sleep(0.05)
    except Exception:
        pass


# --- Helper: stiction kick for small step moves ---
def stiction_kick(axis, direction: float, torque_nm: float = 0.12, pulse_ms: int = 120, settle_ms: int = 220, *, kick_current_lim: float = 15.0, kick_vel_limit: float = 1.2):
    """Short torque pulse to break static friction/cogging, then resync position setpoints.

    direction: +1 or -1 (sign only)
    """
    prev_control_mode = int(axis.controller.config.control_mode)
    prev_input_mode   = int(axis.controller.config.input_mode)
    
    # Temporarily raise current/vel limits for the kick, then restore.
    restore_limits = with_temp_limits(axis, current_lim=kick_current_lim, vel_limit=kick_vel_limit)

    axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
    axis.controller.config.input_mode   = INPUT_MODE_PASSTHROUGH

    try:
        axis.controller.input_torque = float(direction) * float(torque_nm)
        t0 = time.time()
        while (time.time() - t0) * 1000.0 < float(pulse_ms):
            time.sleep(0.005)
    finally:
        try:
            axis.controller.input_torque = 0.0
            restore_limits()
        except Exception:
            pass

    time.sleep(float(settle_ms) / 1000.0)

    try:
        axis.controller.config.control_mode = prev_control_mode
        axis.controller.config.input_mode   = prev_input_mode
    except Exception:
        pass

    # Important: after torque mode, ensure no stale internal setpoints
    reenter_closed_loop_position(axis, settle_s=0.20, sync_repeats=3)

def step_deg(axis, deg):
    """Command a relative position step in degrees using a small slew/ramp.

    This prevents the controller from seeing a large instantaneous error (which can trigger
    overspeed/controller_failed on sensitive setups).
    """
    # Align commanded position first
    sync_to_est(axis, repeats=3, dt=0.03)

    start = float(axis.encoder.pos_estimate)
    target = start + (float(deg) / 360.0)

    steps = 25
    dt = 0.008  # ~200ms total
    for i in range(1, steps + 1):
        axis.controller.input_pos = start + (target - start) * (i / steps)
        _sleep_with_watchdog(axis, dt, step=dt)


def torque_direction_sanity(axis, torque_nm=0.05, pulse_ms=30, repeats=6, settle_ms=180, *, kv=DEFAULT_KV, min_counts=1):
    """Counts-only direction sanity using torque pulses.

    Avoids reading `axis.encoder.vel_estimate` which can block/hang on some firmware/USB stacks.

    Returns: (score, avg_pos_counts, avg_neg_counts)
      score > 0 means +torque -> +counts on average; score < 0 means sign mismatch.

    Raises RuntimeError if motion is too small (inconclusive).
    """
    ensure_torque_constant(axis, kv=float(kv))

    prev_control_mode = int(axis.controller.config.control_mode)
    prev_input_mode   = int(axis.controller.config.input_mode)

    axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
    axis.controller.config.input_mode   = INPUT_MODE_PASSTHROUGH

    pulse_s  = float(pulse_ms) / 1000.0
    settle_s = float(settle_ms) / 1000.0

    def _pulse_counts(tau_nm: float):
        q0 = int(axis.encoder.shadow_count)
        axis.controller.input_torque = float(tau_nm)
        _sleep_with_watchdog(axis, pulse_s, step=0.01)
        axis.controller.input_torque = 0.0
        _sleep_with_watchdog(axis, settle_s, step=0.02)
        q1 = int(axis.encoder.shadow_count)
        return q1 - q0

    deltas_pos, deltas_neg = [], []
    for _ in range(int(repeats)):
        deltas_pos.append(_pulse_counts(+float(torque_nm)))
        deltas_neg.append(_pulse_counts(-float(torque_nm)))

    avg_pos = sum(deltas_pos) / max(1, len(deltas_pos))
    avg_neg = sum(deltas_neg) / max(1, len(deltas_neg))

    if abs(avg_pos) < float(min_counts) and abs(avg_neg) < float(min_counts):
        raise RuntimeError(
            f"Inconclusive torque direction: avg_pos={avg_pos:.2f} counts, avg_neg={avg_neg:.2f} counts. "
            "Increase torque_nm or pulse_ms, or increase current_lim."
        )

    score = avg_pos - avg_neg

    print("\nTorque-direction sanity:")
    print("  deltas_pos:", deltas_pos)
    print("  deltas_neg:", deltas_neg)
    print(f"  avg_pos={avg_pos:.2f} counts, avg_neg={avg_neg:.2f} counts, score={score:.2f} (want > 0)")

    try:
        axis.controller.input_torque = 0.0
    except Exception:
        pass

    try:
        axis.controller.config.control_mode = prev_control_mode
        axis.controller.config.input_mode   = prev_input_mode
    except Exception:
        pass

    return score, avg_pos, avg_neg

# --- Direction sanity check helper ---
def direction_sanity(axis, step_deg_cmd=5.0):
    """Verify that a +position command yields a +encoder change.

    Returns: (dq_counts, dp_deg, expected_counts)
    - Hard FAIL if direction is wrong.
    - SOFT WARN if motion is too weak (likely current/gains too low / cogging).
    """
    cpr = int(axis.encoder.config.cpr)

    # Sync and take baseline
    sync_to_est(axis)
    time.sleep(0.05)
    q0 = int(axis.encoder.shadow_count)
    p0 = float(axis.encoder.pos_estimate)

    # Command a small positive step and observe briefly
    step_deg(axis, step_deg_cmd)
    watch(seconds=0.25)

    q1 = int(axis.encoder.shadow_count)
    p1 = float(axis.encoder.pos_estimate)

    dq = q1 - q0
    dp_deg = (p1 - p0) * 360.0
    expected = int(round((step_deg_cmd / 360.0) * cpr))

    print(f"Direction sanity: cmd +{step_deg_cmd:.1f}° -> Δshadow_count {dq} (expected ≈ +{expected}), Δpos {dp_deg:.2f}°")

    # Hard fail only on wrong sign (positive feedback risk)
    if dq < -1:
        print("FAIL: Encoder decreased on a +position command -> direction mismatch (positive feedback risk).")
        print("Fix (MT6701 ABI/incremental): swap encoder A and B (this flips direction), then rerun axis0.py calibration.")
        print("Alternative: swap any two motor phases, but you MUST rerun axis0.py calibration afterwards.")
        axis.requested_state = AXIS_STATE_IDLE
        raise RuntimeError("Direction sanity failed — do not proceed")

    return dq, dp_deg, expected
    
# --- Direction sanity check (must pass before larger steps) ---
print("\nDirection sanity check (TORQUE pulses first)")

# Ensure we're in closed loop
if int(axis.current_state) != int(AXIS_STATE_CLOSED_LOOP_CONTROL):
    print("Axis not in closed loop.")
    try:
        print_errors(axis)
    except Exception:
        pass
    raise RuntimeError("Axis not in closed loop; run axis0.py then rerun tests1.py")

# Start from ultra-safe profile settings
apply_profile(0)

# Adaptive sweep: if pulses show 0 motion (your current failure), automatically increase pulse width/torque
# and temporarily raise current_lim a bit (still bounded by vel_soft/vel_hard).
ensure_torque_constant(axis, kv=DEFAULT_KV)

sweep = [
    # (current_lim, torque_nm_list, pulse_ms_list, settle_ms)
    (10.0,  [0.06, 0.08],                 [40, 80],           220),
    (15.0,  [0.10, 0.12],                 [80, 140],          260),
    (20.0,  [0.14, 0.16],                 [120, 200],         320),
]

last_err = None
score = avg_pos = avg_neg = None

for cur_lim, torques, pulses, settle in sweep:
    restore = with_temp_limits(axis, current_lim=cur_lim, vel_limit=1.0)
    try:
        for tnm in torques:
            for pms in pulses:
                try:
                    score, avg_pos, avg_neg = torque_direction_sanity(
                        axis,
                        torque_nm=tnm,
                        pulse_ms=pms,
                        repeats=6,
                        settle_ms=settle,
                        kv=DEFAULT_KV,
                        min_counts=1,
                    )
                    last_err = None
                    raise StopIteration
                except RuntimeError as e:
                    last_err = e
                    continue
    except StopIteration:
        restore()
        break
    finally:
        restore()

#
# Decide direction from torque pulses if we got a usable score.
# score > 0  => +torque produced +counts relative to -torque (good)
# score < 0  => sign mismatch (dangerous: positive feedback risk)
# score == 0 => inconclusive

if last_err is not None:
    # Torque-mode direction sanity can be inconclusive on some firmwares if torque_constant
    # is unavailable/ignored or if static friction/cogging wins on short pulses.
    # Fall back to a POSITION-step sign check, but ONLY accept it if we see real motion.
    print(str(last_err))
    print("Torque-direction sanity was inconclusive. Falling back to POSITION step direction check...")

    # Re-enter position closed loop cleanly (torque mode may have left stale setpoints)
    reenter_closed_loop_position(axis, settle_s=0.25, sync_repeats=4)

    # Use a slightly stronger (still safe) profile for the fallback
    apply_profile(2)
    axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    axis.controller.config.input_mode   = INPUT_MODE_PASSTHROUGH
    restore_fb = with_temp_limits(axis, current_lim=15.0, vel_limit=1.2)
    sync_to_est(axis, repeats=4, dt=0.06)

    try:
        dq, dp_deg, expected = direction_sanity(axis, step_deg_cmd=10.0)

        # Hard fail on wrong sign
        if dq < -1:
            axis.requested_state = AXIS_STATE_IDLE
            raise RuntimeError("Direction sanity failed (wrong sign) — do not proceed")

        # If we didn't actually move, do NOT declare success.
        if abs(dq) <= 0 and abs(dp_deg) < 0.25:
            axis.requested_state = AXIS_STATE_IDLE
            raise RuntimeError(
                "Direction sanity inconclusive: fallback POSITION step produced near-zero motion. "
                "Increase current_lim / gains or remove mechanical stiction, then rerun."
            )

        # Accept: we moved and sign wasn't wrong.
        score = 1.0
    finally:
        try:
            restore_fb()
        except Exception:
            pass

# If torque sanity succeeded (last_err is None), use that result decisively.
if last_err is None:
    if score is None:
        axis.requested_state = AXIS_STATE_IDLE
        raise RuntimeError("Direction sanity internal error: score not set")
    if float(score) < 0:
        print("FAIL: Torque pulses indicate direction mismatch (positive feedback risk).")
        print("Fix (incremental AB encoder like MT6701 ABI): swap encoder A and B, then rerun axis0.py calibration.")
        print("Alternative: swap any two motor phases, but you MUST rerun axis0.py calibration afterwards.")
        axis.requested_state = AXIS_STATE_IDLE
        raise RuntimeError("Direction sanity failed — do not proceed")
    if float(score) == 0:
        axis.requested_state = AXIS_STATE_IDLE
        raise RuntimeError("Direction sanity inconclusive (score==0). Increase pulse or current_lim and retry.")

print("PASS: Direction sign looks safe (no positive feedback detected).")

# After torque mode, re-enter closed loop position cleanly to avoid stale pos_setpoint issues
reenter_closed_loop_position(axis, settle_s=0.25, sync_repeats=4)
print("After reenter:", st(axis))

# If you later enable trap traj, only do it after small-step stability is confirmed
if USE_TRAP_TRAJ:
    axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    axis.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
    sync_to_est(axis, repeats=3, dt=0.08)



# If direction sign is OK but motion is far below expectation, don't auto-increase authority here.
# Instead, stop and let the user raise current/gains intentionally after confirming stability.
if __name__ == "__main__":
    # --- Test 1: small incremental steps (stability-first) ---
    # Big steps have been causing dropouts/runaway on this setup.
    # We start with 1° then 2° then 5° only if the previous step was stable.

    def run_step(label: str, step_deg_cmd: float, watch_s: float = 1.2):
        print(f"\n{label}: +{step_deg_cmd:.1f} degrees")

        # Always re-enter closed loop cleanly and sync setpoints before each step.
        # Prevents stale internal setpoints from previous test causing a huge jump.
        reenter_closed_loop_position(axis, settle_s=0.20, sync_repeats=4)

        # Baseline
        sync_to_est(axis, repeats=3, dt=0.04)
        time.sleep(0.05)
        p0 = float(axis.encoder.pos_estimate)
        q0 = int(axis.encoder.shadow_count)

        # Command step
        step_deg(axis, step_deg_cmd)

        # For tiny steps, if encoder doesn't budge early, apply a short kick and retry once.
        if float(step_deg_cmd) <= 2.0:
            time.sleep(0.10)
            q_early = int(axis.encoder.shadow_count)
            if abs(q_early - q0) <= 0:
                kick_dir = 1.0 if float(step_deg_cmd) >= 0.0 else -1.0
                print("No early encoder movement on small step -> applying stiction kick")
                stiction_kick(axis, kick_dir, torque_nm=0.14, pulse_ms=160, settle_ms=260)
                sync_to_est(axis, repeats=2, dt=0.05)
                time.sleep(0.03)
                step_deg(axis, step_deg_cmd)

        try:
            watch(watch_s)
        except RuntimeError as e:
            print(str(e))
            # Force IDLE, then cleanly re-enter position closed loop so the user can rerun
            axis.requested_state = AXIS_STATE_IDLE
            time.sleep(0.15)
            reenter_closed_loop_position(axis, settle_s=0.25, sync_repeats=4)
            return False

        # Settle and measure
        time.sleep(0.20)
        p1 = float(axis.encoder.pos_estimate)
        q1 = int(axis.encoder.shadow_count)

        cpr = int(axis.encoder.config.cpr)
        expected_counts = int(round((step_deg_cmd / 360.0) * cpr))
        moved_counts = q1 - q0
        moved_deg = (p1 - p0) * 360.0

        # Sanity: if motion is wildly larger than commanded, treat it as a setpoint/encoder glitch.
        max_expected = max(20, int(6 * abs(expected_counts) + 40))
        if abs(moved_counts) > max_expected:
            print(f"SAFETY ABORT: moved_counts {moved_counts} exceeds sanity threshold {max_expected} for a {step_deg_cmd}° command")
            print("This usually indicates a stale setpoint / input_pos overwrite, or encoder count reset/glitch.")
            try:
                print_errors(axis)
            except Exception:
                pass
            axis.requested_state = AXIS_STATE_IDLE
            return False

        cmd_turns = float(axis.controller.input_pos)
        set_deg = cmd_turns * 360.0
        err_deg = (cmd_turns - p1) * 360.0

        print(f"encoder_counts: {q0} -> {q1}  (Δ {moved_counts}, expected ≈ {expected_counts})")
        print(f"moved_deg: {moved_deg:.2f}  final_err_deg: {err_deg:.2f}  set_deg: {set_deg:.2f}")
        print("status:", st(axis))

        # Require at least some real motion (counts or degrees). With 1024 CPR, 1 count ≈ 0.35°.
        return (abs(moved_counts) >= 1) or (abs(moved_deg) >= 0.25)

    apply_profile(0)  # extra conservative for unloaded motor

    # Force PASSTHROUGH for initial bringup steps.
    # If you later set USE_TRAP_TRAJ=True, only switch after these steps are stable.
    axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    axis.controller.config.input_mode   = INPUT_MODE_PASSTHROUGH
    sync_to_est(axis, repeats=4, dt=0.06)

    ok = run_step("Test 1a", 10.0, watch_s=3.0)
    if ok:
        ok = run_step("Test 1b", 2.0, watch_s=1.2)
    if ok:
        ok = run_step("Test 1c", 5.0, watch_s=1.5)

    print("\nDone.")

    # Leave the motor free-spinning after the test
    sync_to_est(axis)
    time.sleep(0.05)
    axis.requested_state = AXIS_STATE_IDLE
