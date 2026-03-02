#!/usr/bin/env bash
set -euo pipefail

ROOT="/Users/as/Development/Robot"
cd "$ROOT"

mkdir -p logs
TS="$(date +%Y%m%d_%H%M%S)"
OUT="logs/overnight_autotune_${TS}.out"

python3 -u "$ROOT/overnight_autotune_runner.py" \
  --deadline 8:00 \
  --sleep-s 90 \
  --max-iterations 500 \
  --aggressive-recovery-every 5 \
  --no-response-failure-limit 3 \
  --require-repeatability \
  | tee "$OUT"

echo "log_out: $OUT"
