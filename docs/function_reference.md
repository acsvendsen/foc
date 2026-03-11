# Function Reference

This is the short operator reference for the current ODrive/FOC workflow in this repo.

The two main code files are:

- [/Users/as/Development/Robot/common.py](/Users/as/Development/Robot/common.py)
- [/Users/as/Development/Robot/test_motor_setup.py](/Users/as/Development/Robot/test_motor_setup.py)

## Load Helpers

Run these first in `odrivetool` / IPython:

```python
%run /Users/as/Development/Robot/common.py
%run /Users/as/Development/Robot/test_motor_setup.py
```

Set the active axis:

```python
a = odrv0.axis0
```

## Startup / Recovery

### `clear_errors_all(axis=None, settle_s=0.15)`
File: [/Users/as/Development/Robot/common.py#L4385](/Users/as/Development/Robot/common.py#L4385)

Use this before a fresh calibration or after a failed move.

```python
clear_errors_all(a)
```

### `a.requested_state = 3`
Meaning: `AXIS_STATE_FULL_CALIBRATION_SEQUENCE`

Use this as the default clean startup gate on the working board/path.

```python
clear_errors_all(a)
a.requested_state = 3
```

Then confirm:

```python
print(bool(a.motor.is_calibrated), bool(a.encoder.is_ready), bool(a.encoder.index_found))
```

Expected:

```python
True True True
```

## State Diagnosis

### `diagnose_axis_state(axis=None, mounted=True, verbose=True, kv_est=None, line_line_r_ohm=None)`
File: [/Users/as/Development/Robot/common.py#L1100](/Users/as/Development/Robot/common.py#L1100)

Use this when you see confusing combinations like:
- no latched errors but encoder not ready
- `AXIS_ERROR_ENCODER_FAILED`
- `ENCODER_ERROR_CPR_POLEPAIRS_MISMATCH`
- current-limit or overspeed faults

Example:

```python
diagnose_axis_state(a, kv_est=140.0, line_line_r_ohm=0.30)
```

It prints:
- a one-line verdict
- decoded error names
- likely diagnosis
- next recovery commands

## Motor Fact Sheet

### `motor_fact_sheet(axis=None, kv_est=None, line_line_r_ohm=None, verbose=True)`
File: [/Users/as/Development/Robot/common.py#L1257](/Users/as/Development/Robot/common.py#L1257)

Use this to print the current board/motor/encoder fact sheet with provenance labels:
- `measured live`
- `configured`
- `inferred`
- `unknown`

Example:

```python
motor_fact_sheet(a, kv_est=140.0, line_line_r_ohm=0.30)
```

## Low-Level Direct Move

### `move_to_pos_strict(...)`
File: [/Users/as/Development/Robot/common.py#L1641](/Users/as/Development/Robot/common.py#L1641)

This is the low-level direct position move helper. Use this when you want precise control over:
- trap trajectory
- current limit
- gains
- quiet final hold

Important:
- it now enforces a pre-move controller/encoder sync contract
- if controller position memory is stale, it raises instead of silently nudging

Typical use:

```python
r = move_to_pos_strict(
    a,
    target_turns,
    use_trap_traj=True,
    timeout_s=8.0,
    trap_vel=4.0,
    trap_acc=1.0,
    trap_dec=1.0,
    vel_limit=4.4,
    current_lim=15.0,
    pos_gain=12.0,
    vel_gain=0.22,
    vel_i_gain=0.0,
    require_target_reached=True,
    target_tolerance_turns=0.03,
    target_vel_tolerance_turns_s=0.20,
    quiet_hold_enable=True,
    quiet_hold_s=0.06,
    quiet_hold_pos_gain_scale=0.45,
    quiet_hold_vel_gain_scale=0.70,
    quiet_hold_vel_i_gain=0.0,
    quiet_hold_vel_limit_scale=0.50,
    quiet_hold_persist=True,
    quiet_hold_reanchor_err_turns=0.035,
    fail_to_idle=False,
)
```

## Continuous Gearbox-Output Move

### `move_to_angle_continuous(...)`
File: [/Users/as/Development/Robot/test_motor_setup.py#L4014](/Users/as/Development/Robot/test_motor_setup.py#L4014)

Use this for the current best mounted operating mode:
- one continuous move to the final target
- no intermediate settled waypoints
- one soft reanchored final hold

This is the preferred quiet path right now.

Example, relative move:

```python
r = move_to_angle_continuous(
    30.0,
    angle_space="gearbox_output",
    axis=a,
    relative_to_current=True,
    profile_name="gearbox_output_continuous_aggressive_20260309",
)
```

Example, absolute move from a known zero anchor:

```python
zero = float(a.encoder.pos_estimate)
r = move_to_angle_continuous(
    30.0,
    angle_space="gearbox_output",
    axis=a,
    zero_turns_motor=zero,
    relative_to_current=False,
    profile_name="gearbox_output_continuous_aggressive_20260309",
)
```

Important:
- if you manually touch/reposition the output, re-run calibration before aggressive moves
- do not use intermediate settled chunks for long moves

## Higher-Level Angle Move

### `move_to_angle(...)`
File: [/Users/as/Development/Robot/test_motor_setup.py#L5638](/Users/as/Development/Robot/test_motor_setup.py#L5638)

Use this when you want the full higher-level motion wrapper. It supports:
- absolute or relative angles
- multiple control styles
- retry logic
- staged motion
- pre-move gates

For the current mounted quiet operating mode, prefer `move_to_angle_continuous(...)` unless you specifically need the extra wrapper behavior.

## Position Sync Helpers

### `sync_pos_setpoint(...)`
File: [/Users/as/Development/Robot/common.py#L65](/Users/as/Development/Robot/common.py#L65)

Best-effort helper to align controller target fields to live encoder position.

### `enforce_position_start_sync(...)`
File: [/Users/as/Development/Robot/common.py#L324](/Users/as/Development/Robot/common.py#L324)

Hard pre-move contract. This is what prevents "controller remembered some other position" starts.

You usually do not call it directly; `move_to_pos_strict(...)` does it for you.

## Current Working Motion Rule

For the mounted system, the validated rule is:

1. `clear_errors_all(a)`
2. `a.requested_state = 3`
3. one continuous move to final target
4. no intermediate settled waypoints
5. soft final hold only

Current saved profiles:

- `gearbox_output_continuous_quiet_20260309`
- `gearbox_output_continuous_aggressive_20260309`

## When To Stop

Stop and diagnose instead of tuning further if:

1. `diagnose_axis_state(...)` says startup is not ready
2. `motor_fact_sheet(...)` shows encoder not ready / index not found
3. a move trips:
   - `STALL watchdog`
   - `WRONG_DIRECTION watchdog`
   - `MOTOR_ERROR_CURRENT_LIMIT_VIOLATION`
   - `CONTROLLER_ERROR_OVERSPEED`

Then return to:

```python
a.requested_state = 1
clear_errors_all(a)
a.requested_state = 3
```
