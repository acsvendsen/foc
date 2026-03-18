#!/usr/bin/env zsh
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
VENV_DIR="$ROOT_DIR/.venv"

if [[ ! -x "$VENV_DIR/bin/odrivetool" ]]; then
  echo "odrivetool is not installed in $VENV_DIR" >&2
  echo "Rebuild the local environment first." >&2
  exit 1
fi

source "$VENV_DIR/bin/activate"
exec odrivetool liveplotter "$@"
