# New Mac Setup

This is the current working setup for this repo on a new Mac.

## Goal

Get a clean Python environment for:

- `odrivetool`
- `odrive` Python package
- `matplotlib` for `liveplotter`

Avoid installing into Xcode's bundled Python.

## Current Recommended Path

Use:

1. `uv` for a standalone Python runtime
2. a project-local `.venv`
3. wrapper scripts in the repo root

## One-Time Setup

From the repo root:

```bash
cd /Users/acs/Development/Robot

curl -LsSf https://astral.sh/uv/install.sh | sh
source "$HOME/.local/bin/env"

uv python install 3.11
uv venv --python 3.11 .venv

.venv/bin/python -m ensurepip --upgrade
.venv/bin/python -m pip install --upgrade pip setuptools wheel
.venv/bin/python -m pip install --upgrade odrive matplotlib
```

## Daily Use

### Start `odrivetool`

```bash
cd /Users/acs/Development/Robot
./run_odrivetool.sh
```

### Start `liveplotter`

```bash
cd /Users/acs/Development/Robot
./run_liveplotter.sh
```

## Manual Venv Use

If you want the shell activated:

```bash
cd /Users/acs/Development/Robot
source .venv/bin/activate
odrivetool
```

## Verify The Environment

```bash
cd /Users/acs/Development/Robot
.venv/bin/python --version
.venv/bin/python -c "import odrive, matplotlib; print(odrive.__file__); print(matplotlib.__version__)"
./run_odrivetool.sh --help
./run_liveplotter.sh --help
```

## Optional Homebrew Path

Homebrew is not required for this repo now, but if you want it installed the official command is:

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

Then on Apple Silicon:

```bash
echo 'eval "$(/opt/homebrew/bin/brew shellenv)"' >> ~/.zprofile
eval "$(/opt/homebrew/bin/brew shellenv)"
brew --version
```

If you want a Homebrew-managed Python too:

```bash
brew install python@3.11
```

Then rebuild the repo venv on top of that interpreter:

```bash
cd /Users/acs/Development/Robot
python3.11 -m venv .venv
source .venv/bin/activate
python -m ensurepip --upgrade
python -m pip install --upgrade pip setuptools wheel
python -m pip install --upgrade odrive matplotlib
```

## Notes

- `.venv/` is intentionally gitignored.
- `.swiftpm/` is intentionally gitignored.
- If `odrivetool` cannot connect, make sure no other process is already using the board.
- For this project, keep hardware commands serialized. Do not run multiple ODrive control processes against the same board at once.
