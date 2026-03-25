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
from mks_axis_characterize import _axis_snapshot, resolve_odrv_axis


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
        "probe": common.probe_inc_encoder_sources(
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
    p.add_argument("--cpr", type=int, default=1024)
    p.add_argument("--bandwidth", type=float, default=200.0)
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
        bind_best=bool(args.bind_best),
        restore_if_no_winner=True,
        verbose=True,
        out_path=args.out,
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
