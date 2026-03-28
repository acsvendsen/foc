#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json

from output_sensor_bridge import OutputSensorBridgeConfig, OutputSensorBridge


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Inspect or zero the external output sensor bridge")
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--rate-hz", type=int, default=400)
    parser.add_argument("--timeout-s", type=float, default=3.0)
    parser.add_argument("--mark-home", action="store_true")
    parser.add_argument("--set-zero-counts", type=int)
    return parser


def main() -> int:
    args = _parser().parse_args()
    bridge = OutputSensorBridge(
        OutputSensorBridgeConfig(
            port=str(args.port),
            baudrate=int(args.baud),
            sample_rate_hz=int(args.rate_hz),
        )
    )
    bridge.start()
    try:
        if args.mark_home:
            bridge.mark_home()
            bridge.wait_for_status(timeout_s=float(args.timeout_s), require_homed=True)
            bridge.wait_for_sample(timeout_s=float(args.timeout_s))
        elif args.set_zero_counts is not None:
            bridge.set_zero_offset_counts(int(args.set_zero_counts))
            bridge.wait_for_status(timeout_s=float(args.timeout_s), require_homed=True)
            bridge.wait_for_sample(timeout_s=float(args.timeout_s))
        else:
            bridge.wait_for_sample(timeout_s=float(args.timeout_s))
        snapshot = bridge.latest_snapshot()
        print(json.dumps(snapshot, indent=2, sort_keys=False))
        return 0 if snapshot.get("output_turns") is not None else 1
    finally:
        bridge.stop()


if __name__ == "__main__":
    raise SystemExit(main())
