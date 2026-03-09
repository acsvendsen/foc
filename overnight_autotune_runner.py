#!/usr/bin/env python3
import argparse
import json
import os
import datetime

import test_motor_setup as t


def main():
    p = argparse.ArgumentParser(description="Run overnight autotune until local deadline.")
    p.add_argument("--deadline", default="08:00", help="Local deadline (HH:MM or ISO datetime). Default: 08:00")
    p.add_argument("--sleep-s", type=float, default=120.0, help="Sleep between iterations (seconds).")
    p.add_argument("--max-iterations", type=int, default=400, help="Maximum loop iterations.")
    p.add_argument("--log-jsonl", default="", help="Path to JSONL log file.")
    p.add_argument("--require-repeatability", action="store_true", help="Require repeatability gate for success.")
    p.add_argument(
        "--aggressive-recovery-every",
        type=int,
        default=5,
        help="Escalate to reboot+forced recalibration after this many consecutive reference-recovery failures.",
    )
    p.add_argument(
        "--no-response-failure-limit",
        type=int,
        default=3,
        help="Abort early after this many consecutive encoder no-response signals.",
    )
    p.add_argument(
        "--no-abort-on-persistent-no-response",
        action="store_true",
        help="Disable early abort when encoder no-response is persistently detected.",
    )
    args = p.parse_args()

    log_path = args.log_jsonl.strip() or None
    res = t.overnight_autotune_until(
        deadline_local=args.deadline,
        sleep_s=float(args.sleep_s),
        max_iterations=int(args.max_iterations),
        log_jsonl_path=log_path,
        require_repeatability=bool(args.require_repeatability),
        aggressive_recovery_every=int(args.aggressive_recovery_every),
        abort_on_persistent_no_response=not bool(args.no_abort_on_persistent_no_response),
        no_response_failure_limit=int(args.no_response_failure_limit),
    )

    out_dir = os.path.abspath("logs")
    os.makedirs(out_dir, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    summary_path = os.path.join(out_dir, f"overnight_autotune_summary_{stamp}.json")
    with open(summary_path, "w", encoding="utf-8") as f:
        json.dump(res, f, indent=2, sort_keys=True)

    print("overnight_autotune_summary_path:", summary_path)
    print(json.dumps(res, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
