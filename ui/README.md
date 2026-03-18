# Operator UI

This UI is intentionally split in two layers:

1. `ui/backend/odrive_operator_backend.py`
   - authoritative motor command bridge
   - reuses the proven Python helpers in `/Users/as/Development/Robot/common.py` and `/Users/as/Development/Robot/test_motor_setup.py`
2. `ui/OperatorConsole`
   - SwiftUI front-end for operator use
   - does not implement motor control logic itself

## Why this split

Rewriting the ODrive motion logic in Swift right now would create a second control stack and make debugging worse. The Python layer already contains the validated startup, diagnosis, and continuous-move behavior.

## Backend commands

The backend returns JSON for:

- `status`
- `diagnose`
- `fact-sheet`
- `idle`
- `clear-errors`
- `startup`
- `move-continuous`
- `profiles`

## SwiftUI app

Open `ui/OperatorConsole/Package.swift` in Xcode.

The app:
- shows readiness/status verdicts at a glance
- enables/disables buttons based on backend capability flags
- exposes only the safe continuous move path
- supports capturing the current position as a zero reference for absolute moves
- auto-detects the repo root from the source location when launched from Xcode

If auto-detection still fails, set:

```bash
ROBOT_REPO_ROOT=/Users/acs/Development/Robot
```

before launching, or paste the repo path into the UI field.

## Local constraints

This machine currently has Swift CLI tools available, but `xcodebuild` is still pointing at Command Line Tools rather than the full Xcode app. If Xcode build/run fails, fix the developer directory first:

```bash
sudo xcode-select -s /Applications/Xcode.app/Contents/Developer
```

Only do that if full Xcode is already installed.
