from odrive.enums import *
import common

def _motor_type_high_current():
    try:
        return MOTOR_TYPE_HIGH_CURRENT
    except Exception:
        pass
    try:
        return int(MotorType.HIGH_CURRENT)
    except Exception:
        return 0


def _encoder_mode_incremental():
    try:
        return ENCODER_MODE_INCREMENTAL
    except Exception:
        return 0


def _encoder_id_value(encoder_source="INC_ENCODER0"):
    if encoder_source is None:
        return None
    if isinstance(encoder_source, int):
        return int(encoder_source)

    s = str(encoder_source).strip().upper()
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


def _encoder_attr_name(encoder_id):
    try:
        eid = int(encoder_id)
    except Exception:
        return None
    for name in ("INC_ENCODER0", "INC_ENCODER1", "INC_ENCODER2"):
        try:
            if int(getattr(EncoderId, name)) == eid:
                return name.lower()
        except Exception:
            pass
    return None


def g_calibrate(axis, kv=140.0, pole_pairs=7, cpr=1024, calibration_current=10.0, use_index=True, encoder_source="INC_ENCODER0"):
    """Baseline config for a 140kV BLDC with incremental encoder."""
    kv_f = float(kv)
    enc_id = _encoder_id_value(encoder_source)
    if enc_id is None:
        enc_id = _encoder_id_value("INC_ENCODER0")

    # Motor setup
    axis.motor.config.motor_type = int(_motor_type_high_current())
    axis.motor.config.pole_pairs = int(pole_pairs)
    axis.motor.config.torque_constant = 8.27 / kv_f  # Nm/A ~= 8.27 / kV
    axis.motor.config.calibration_current = float(calibration_current)

    # Encoder setup
    axis.encoder.config.mode = int(_encoder_mode_incremental())
    axis.encoder.config.cpr = int(cpr)
    axis.encoder.config.use_index = bool(use_index)

    # ODrive 0.6+ explicit incremental encoder path
    try:
        parent = getattr(axis, "_parent", None)
    except Exception:
        parent = None
    try:
        enc_attr = _encoder_attr_name(enc_id)
        inc_obj = getattr(parent, enc_attr, None) if (parent is not None and enc_attr is not None) else None
        if inc_obj is not None and hasattr(inc_obj, "config"):
            try:
                inc_obj.config.enabled = True
            except Exception:
                pass
            try:
                inc_obj.config.cpr = int(cpr)
            except Exception:
                pass
            try:
                inc_obj.config.use_index = bool(use_index)
            except Exception:
                pass
            try:
                inc_obj.config.bandwidth = float(20.0)
            except Exception:
                pass
    except Exception:
        pass
    try:
        axis.config.load_encoder = int(enc_id)
    except Exception:
        pass
    try:
        axis.config.commutation_encoder = int(enc_id)
    except Exception:
        pass

    # Startup index search is needed if you want repeatable absolute reference after power-up.
    try:
        axis.config.startup_encoder_index_search = bool(use_index)
    except Exception:
        pass
    try:
        axis.encoder.config.find_idx_on_lockin = bool(use_index)
    except Exception:
        pass

    return {
        "kv": kv_f,
        "pole_pairs": int(pole_pairs),
        "cpr": int(cpr),
        "torque_constant": float(axis.motor.config.torque_constant),
        "use_index": bool(use_index),
        "encoder_source": str(encoder_source),
    }

def sweep_test_single():
    a = common.get_axis0()
    common.establish_absolute_reference(a, require_index=True, run_index_search=True, attempt_offset_calibration=False)

    targets = [0.0, 0.25, 0.50, 0.75, 1.00, 0.75, 0.50, 0.25, 0.0]
    for t in targets:
        r = common.move_to_pos_strict(a, t, timeout_s=5.0, use_trap_traj=True, fail_to_idle=False)
        err = r["target"] - r["end"]
        print(f"target={t:+.3f} end={r['end']:+.6f} err={err:+.6f}")
