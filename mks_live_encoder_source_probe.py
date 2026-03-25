#!/usr/bin/env python3
"""Live incremental encoder-source probe for MKS/ODrive-style boards.

Run this while manually rotating the motor shaft. The script samples each
incremental source window in IDLE and reports which source, if any, produces
counts.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import common
from odrive.enums import AXIS_STATE_IDLE, AXIS_STATE_LOCKIN_SPIN, AXIS_STATE_MOTOR_CALIBRATION

from mks_axis_characterize import _axis_snapshot, clear_local_errors, neutralize_controller_idle_state, resolve_odrv_axis


def _probe_inc_encoder_sources_lockin(
    axis,
    odrv,
    *,
    sources,
    cpr,
    bandwidth,
    interp,
    use_index,
    pole_pairs,
    current_lim,
    current_control_bandwidth,
    motor_calibration_current,
    lockin_current,
    lockin_vel,
    lockin_accel,
    lockin_ramp_distance,
    timeout_s,
    bind_best,
    restore_if_no_winner,
    verbose,
):
    selection_supported = common._supports_encoder_source_selection(axis)
    try:
        prev_load = int(getattr(axis.config, "load_encoder"))
    except Exception:
        prev_load = None
    try:
        prev_comm = int(getattr(axis.config, "commutation_encoder"))
    except Exception:
        prev_comm = None

    results = {}

    if verbose:
        print("lockin encoder probe: driving open-loop lockin spin per source.")

    if not selection_supported:
        clear_local_errors(axis, odrv=odrv, settle_s=0.05)
        common.force_idle(axis, settle_s=0.05)
        axis.motor.config.pole_pairs = int(pole_pairs)
        axis.motor.config.current_lim = float(current_lim)
        axis.motor.config.current_control_bandwidth = float(current_control_bandwidth)
        axis.motor.config.calibration_current = float(motor_calibration_current)
        common.set_encoder(
            axis,
            cpr=int(cpr),
            bandwidth=float(bandwidth),
            interp=bool(interp),
            use_index=bool(use_index),
            encoder_source="INC_ENCODER0",
        )
        neutralize_controller_idle_state(axis)
        axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
        motor_ok = common.wait_state(axis, AXIS_STATE_IDLE, timeout_s=float(timeout_s), poll_s=0.05, feed_watchdog=False)
        lockin = axis.config.calibration_lockin
        lockin.current = float(lockin_current)
        lockin.vel = float(lockin_vel)
        lockin.accel = float(lockin_accel)
        lockin.ramp_distance = float(lockin_ramp_distance)

        c0 = int(getattr(axis.encoder, "shadow_count", 0))
        p0 = float(getattr(axis.encoder, "pos_estimate", 0.0))
        axis.requested_state = AXIS_STATE_LOCKIN_SPIN
        spin_idle_ok = common.wait_state(axis, AXIS_STATE_IDLE, timeout_s=float(timeout_s), poll_s=0.05, feed_watchdog=False)
        c1 = int(getattr(axis.encoder, "shadow_count", c0))
        p1 = float(getattr(axis.encoder, "pos_estimate", p0))
        dc = int(c1 - c0)
        dp = float(p1 - p0)
        try:
            enc_err = int(getattr(axis.encoder, "error", 0))
        except Exception:
            enc_err = 0
        try:
            axis_err = int(getattr(axis, "error", 0))
        except Exception:
            axis_err = 0
        try:
            motor_err = int(getattr(axis.motor, "error", 0))
        except Exception:
            motor_err = 0

        alive = bool(abs(dc) >= 1 or abs(dp) >= (1.0 / max(1.0, float(cpr))))
        return {
            "ok": bool(alive),
            "best_source": None,
            "bound_best": False,
            "restored_previous": False,
            "selection_supported": False,
            "diagnostic_note": (
                "This firmware exposes only the single legacy `axis.encoder` path. "
                "Per-source probing of INC_ENCODER0/1/2 is not supported here; this lockin test "
                "only shows whether the legacy encoder path produces counts during open-loop spin."
            ),
            "results": {
                "LEGACY_AXIS_ENCODER": {
                    "alive": bool(alive),
                    "delta_counts": dc,
                    "delta_pos": dp,
                    "motor_ok": bool(motor_ok),
                    "spin_idle_ok": bool(spin_idle_ok),
                    "axis_err": axis_err,
                    "motor_err": motor_err,
                    "encoder_error": enc_err,
                    "enc_ready": bool(getattr(axis.encoder, "is_ready", False)),
                    "score": float(abs(dc) + abs(dp) * float(cpr)),
                }
            },
        }

    for src in list(sources):
        src_name = str(src).strip()
        try:
            clear_local_errors(axis, odrv=odrv, settle_s=0.05)
            common.force_idle(axis, settle_s=0.05)
            axis.motor.config.pole_pairs = int(pole_pairs)
            axis.motor.config.current_lim = float(current_lim)
            axis.motor.config.current_control_bandwidth = float(current_control_bandwidth)
            axis.motor.config.calibration_current = float(motor_calibration_current)
            common.set_encoder(
                axis,
                cpr=int(cpr),
                bandwidth=float(bandwidth),
                interp=bool(interp),
                use_index=bool(use_index),
                encoder_source=src_name,
            )
            neutralize_controller_idle_state(axis)

            axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
            motor_ok = common.wait_state(axis, AXIS_STATE_IDLE, timeout_s=float(timeout_s), poll_s=0.05, feed_watchdog=False)

            lockin = axis.config.calibration_lockin
            lockin.current = float(lockin_current)
            lockin.vel = float(lockin_vel)
            lockin.accel = float(lockin_accel)
            lockin.ramp_distance = float(lockin_ramp_distance)

            c0 = int(getattr(axis.encoder, "shadow_count", 0))
            p0 = float(getattr(axis.encoder, "pos_estimate", 0.0))
            axis.requested_state = AXIS_STATE_LOCKIN_SPIN
            spin_idle_ok = common.wait_state(axis, AXIS_STATE_IDLE, timeout_s=float(timeout_s), poll_s=0.05, feed_watchdog=False)
            c1 = int(getattr(axis.encoder, "shadow_count", c0))
            p1 = float(getattr(axis.encoder, "pos_estimate", p0))
            dc = int(c1 - c0)
            dp = float(p1 - p0)
            try:
                enc_err = int(getattr(axis.encoder, "error", 0))
            except Exception:
                enc_err = 0
            try:
                axis_err = int(getattr(axis, "error", 0))
            except Exception:
                axis_err = 0
            try:
                motor_err = int(getattr(axis.motor, "error", 0))
            except Exception:
                motor_err = 0

            alive = bool(abs(dc) >= 1 or abs(dp) >= (1.0 / max(1.0, float(cpr))))
            res = {
                "alive": bool(alive),
                "delta_counts": dc,
                "delta_pos": dp,
                "motor_ok": bool(motor_ok),
                "spin_idle_ok": bool(spin_idle_ok),
                "axis_err": axis_err,
                "motor_err": motor_err,
                "encoder_error": enc_err,
                "enc_ready": bool(getattr(axis.encoder, "is_ready", False)),
                "score": float(abs(dc) + abs(dp) * float(cpr)),
            }
            results[src_name] = res

            if verbose:
                print(
                    f"  {src_name}: alive={alive} Δcounts={dc:+d} Δpos={dp:+.6f} "
                    f"motor_ok={motor_ok} spin_idle_ok={spin_idle_ok} "
                    f"axis_err={hex(axis_err)} motor_err={hex(motor_err)} enc_err={hex(enc_err)}"
                )
        except Exception as e:
            results[src_name] = {"alive": False, "error": f"{type(e).__name__}: {e}"}
            if verbose:
                print(f"  {src_name}: error={type(e).__name__}: {e}")
        finally:
            try:
                clear_local_errors(axis, odrv=odrv, settle_s=0.05)
                common.force_idle(axis, settle_s=0.05)
                neutralize_controller_idle_state(axis)
            except Exception:
                pass

    alive_items = [(k, v) for k, v in results.items() if isinstance(v, dict) and bool(v.get("alive", False))]
    best_source = None
    if alive_items:
        best_source = max(alive_items, key=lambda kv: float(kv[1].get("score", 0.0)))[0]

    restored = False
    if best_source is not None and bool(bind_best):
        try:
            common.set_encoder(axis, cpr=int(cpr), bandwidth=float(bandwidth), interp=bool(interp), use_index=bool(use_index), encoder_source=best_source)
        except Exception:
            pass
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

    if best_source is not None:
        diagnostic_note = f"Detected live incremental source `{best_source}` during motor-driven lockin spin."
    else:
        diagnostic_note = (
            "No incremental source produced counts during motor-driven lockin spin. "
            "That points to encoder signal/mode/wiring or magnet coupling, not profile tuning."
        )

    return {
        "ok": bool(best_source is not None),
        "best_source": best_source,
        "bound_best": bool(best_source is not None and bind_best),
        "restored_previous": bool(restored),
        "diagnostic_note": diagnostic_note,
        "results": results,
    }


def run_live_encoder_source_probe(
    serial_number: str | None = None,
    axis_index: int = 0,
    *,
    odrv=None,
    axis=None,
    timeout_s: float = 10.0,
    sources: list[str] | tuple[str, ...] = ("INC_ENCODER0", "INC_ENCODER1", "INC_ENCODER2"),
    cpr: int = 1024,
    bandwidth: float = 200.0,
    interp: bool = True,
    use_index: bool = False,
    watch_s: float = 3.0,
    hz: float = 40.0,
    min_counts: int = 4,
    drive_mode: str = "manual",
    pole_pairs: int = 11,
    current_lim: float = 8.0,
    current_control_bandwidth: float = 300.0,
    motor_calibration_current: float = 2.5,
    lockin_current: float = 10.0,
    lockin_vel: float = 10.0,
    lockin_accel: float = 20.0,
    lockin_ramp_distance: float = 6.283185307179586,
    bind_best: bool = False,
    restore_if_no_winner: bool = True,
    verbose: bool = True,
    out_path: str | None = None,
):
    odrv, axis = resolve_odrv_axis(
        odrv=odrv,
        axis=axis,
        serial_number=serial_number,
        axis_index=axis_index,
        timeout_s=timeout_s,
    )

    report = {
        "board_serial": str(getattr(odrv, "serial_number", "")),
        "axis_index": int(axis_index),
        "snapshot_before": _axis_snapshot(axis, odrv),
        "drive_mode": str(drive_mode),
        "probe": (
            common.probe_inc_encoder_sources(
                axis=axis,
                sources=tuple(sources),
                cpr=int(cpr),
                bandwidth=float(bandwidth),
                interp=bool(interp),
                use_index=bool(use_index),
                watch_s=float(watch_s),
                hz=float(hz),
                min_counts=int(min_counts),
                bind_best=bool(bind_best),
                restore_if_no_winner=bool(restore_if_no_winner),
                verbose=bool(verbose),
            )
            if str(drive_mode).lower() == "manual"
            else _probe_inc_encoder_sources_lockin(
                axis,
                odrv,
                sources=tuple(sources),
                cpr=int(cpr),
                bandwidth=float(bandwidth),
                interp=bool(interp),
                use_index=bool(use_index),
                pole_pairs=int(pole_pairs),
                current_lim=float(current_lim),
                current_control_bandwidth=float(current_control_bandwidth),
                motor_calibration_current=float(motor_calibration_current),
                lockin_current=float(lockin_current),
                lockin_vel=float(lockin_vel),
                lockin_accel=float(lockin_accel),
                lockin_ramp_distance=float(lockin_ramp_distance),
                timeout_s=float(max(10.0, watch_s)),
                bind_best=bool(bind_best),
                restore_if_no_winner=bool(restore_if_no_winner),
                verbose=bool(verbose),
            )
        ),
        "snapshot_after": _axis_snapshot(axis, odrv),
    }

    if out_path:
        p = Path(out_path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")

    return report


def _build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--serial-number", required=True, help="USB hex serial or decimal ODrive serial")
    p.add_argument("--axis-index", type=int, default=0)
    p.add_argument("--watch-s", type=float, default=3.0)
    p.add_argument("--hz", type=float, default=40.0)
    p.add_argument("--min-counts", type=int, default=4)
    p.add_argument("--drive-mode", choices=("manual", "lockin"), default="manual")
    p.add_argument("--cpr", type=int, default=1024)
    p.add_argument("--bandwidth", type=float, default=200.0)
    p.add_argument("--pole-pairs", type=int, default=11)
    p.add_argument("--current-lim", type=float, default=8.0)
    p.add_argument("--current-control-bandwidth", type=float, default=300.0)
    p.add_argument("--motor-calibration-current", type=float, default=2.5)
    p.add_argument("--lockin-current", type=float, default=10.0)
    p.add_argument("--lockin-vel", type=float, default=10.0)
    p.add_argument("--lockin-accel", type=float, default=20.0)
    p.add_argument("--lockin-ramp-distance", type=float, default=6.283185307179586)
    p.add_argument("--bind-best", action="store_true")
    p.add_argument("--use-index", action="store_true")
    p.add_argument("--sources", default="INC_ENCODER0,INC_ENCODER1,INC_ENCODER2")
    p.add_argument("--out", default=None)
    return p


def main() -> int:
    args = _build_arg_parser().parse_args()
    sources = [s.strip() for s in str(args.sources).split(",") if s.strip()]
    report = run_live_encoder_source_probe(
        serial_number=args.serial_number,
        axis_index=args.axis_index,
        sources=sources,
        cpr=args.cpr,
        bandwidth=args.bandwidth,
        interp=True,
        use_index=bool(args.use_index),
        watch_s=args.watch_s,
        hz=args.hz,
        min_counts=args.min_counts,
        drive_mode=args.drive_mode,
        pole_pairs=args.pole_pairs,
        current_lim=args.current_lim,
        current_control_bandwidth=args.current_control_bandwidth,
        motor_calibration_current=args.motor_calibration_current,
        lockin_current=args.lockin_current,
        lockin_vel=args.lockin_vel,
        lockin_accel=args.lockin_accel,
        lockin_ramp_distance=args.lockin_ramp_distance,
        bind_best=bool(args.bind_best),
        restore_if_no_winner=True,
        verbose=True,
        out_path=args.out,
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
