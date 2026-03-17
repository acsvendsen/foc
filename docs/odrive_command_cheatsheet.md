# ODrive Command Cheat Sheet

This is the short, practical cheat sheet for the commands and state numbers we actually use on this project.

Main code files:

- [/Users/as/Development/Robot/common.py](/Users/as/Development/Robot/common.py)
- [/Users/as/Development/Robot/test_motor_setup.py](/Users/as/Development/Robot/test_motor_setup.py)

Load them first in `odrivetool` / IPython:

```python
%run /Users/as/Development/Robot/common.py
%run /Users/as/Development/Robot/test_motor_setup.py

a = odrv0.axis0
```

## State Numbers We Actually Use

### `1` = `AXIS_STATE_IDLE`
Use to disarm / stop motion.

```python
a.requested_state = 1
```

### `3` = `AXIS_STATE_FULL_CALIBRATION_SEQUENCE`
Default clean startup gate for this project.

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

### `4` = `AXIS_STATE_MOTOR_CALIBRATION`
Motor electrical calibration only.

```python
a.requested_state = 4
```

Use when you want to re-measure:
- phase resistance
- phase inductance

### `6` = `AXIS_STATE_ENCODER_INDEX_SEARCH`
Use only when you explicitly need index search.

```python
a.requested_state = 6
```

### `7` = `AXIS_STATE_ENCODER_OFFSET_CALIBRATION`
Use to calibrate encoder electrical offset / commutation relationship.

```python
a.requested_state = 7
```

Usually not needed directly if `state=3` is working.

### `8` = `AXIS_STATE_CLOSED_LOOP_CONTROL`
Arms the axis for encoder-based closed-loop motion.

```python
a.requested_state = 8
```

## State Numbers To Avoid

### `5`
Do **not** use this for the robot joint.

It is sensorless mode on this ODrive-style stack, and it is the wrong path for:
- low-speed motion
- harmonic drive motion
- precise robotic arm control

Typical result:
- stops short
- `ControllerError.INVALID_ESTIMATE`
- `AxisError.CONTROLLER_FAILED`

## Common Recovery Pattern

If the state is weird, encoder not ready, or you manually touched the output:

```python
a.requested_state = 1
clear_errors_all(a)
a.requested_state = 3
```

## Diagnose Current State

Use:

```python
diagnose_axis_state(a, kv_est=140.0, line_line_r_ohm=0.30)
```

This prints:
- verdict line
- likely diagnosis
- next commands

Examples of verdicts:
- `motor model OK`
- `startup not ready`
- `torque constant suspect`

## Print Live Motor Fact Sheet

Use:

```python
motor_fact_sheet(a, kv_est=140.0, line_line_r_ohm=0.30)
```

This prints:
- `measured live`
- `configured`
- `inferred`
- `unknown`

Use it when you want to know:
- whether startup is actually ready
- the current `phase_resistance`
- the current `phase_inductance`
- whether `torque_constant` looks suspicious

## Best Current Motion Command

For the mounted gearbox system, the best current operating mode is:

- one continuous move to final target
- no intermediate settled waypoints
- one soft final hold only

Use:

```python
r = move_to_angle_continuous(
    30.0,
    angle_space="gearbox_output",
    axis=a,
    relative_to_current=True,
    profile_name="gearbox_output_continuous_aggressive_20260309",
)
print(r)
```

## Absolute vs Relative Move

### Relative move
Move from where you are now:

```python
r = move_to_angle_continuous(
    30.0,
    angle_space="gearbox_output",
    axis=a,
    relative_to_current=True,
    profile_name="gearbox_output_continuous_aggressive_20260309",
)
```

### Absolute move
First define a real zero:

```python
zero = float(a.encoder.pos_estimate)
```

Then move to an absolute angle from that zero:

```python
r = move_to_angle_continuous(
    30.0,
    angle_space="gearbox_output",
    axis=a,
    zero_turns_motor=zero,
    relative_to_current=False,
    profile_name="gearbox_output_continuous_aggressive_20260309",
)
```

## When A Move Fails But `dump_errors()` Is Clean

That usually means our own software gate rejected the move, not the drive.

Typical example:
- `Target not reached before timeout`
- no axis/motor/encoder/controller errors

Meaning:
- the move almost finished
- but it did not satisfy the final tolerance contract

That is a software/runtime contract failure, not necessarily a hardware fault.

## If You Touch The Output By Hand

Rule:
- do **not** manually reposition while armed (`state = 8`)

If you touched it anyway:

```python
a.requested_state = 1
clear_errors_all(a)
a.requested_state = 3
```

Then continue.

## Quick Live Status Print

Use:

```python
print(
    bool(a.motor.is_calibrated),
    bool(a.encoder.is_ready),
    bool(getattr(a.encoder, "index_found", False)),
    int(a.current_state),
    int(a.error), int(a.motor.error), int(a.encoder.error), int(a.controller.error),
)
```

Interpretation:
- `True True True 8 0 0 0 0`
  - fully armed and ready
- `True False True 1 256 0 2 0`
  - index latched, but encoder/commutation not valid
- `True False False 1 0 0 0 0`
  - startup not ready yet

## Project Rules That Matter

1. Do not use segmented settled waypoints for long moves.
2. Use one continuous move to final target.
3. Treat `state=3` as the startup gate.
4. If startup is not ready, stop tuning motion and fix startup first.
5. Do not use sensorless mode (`state=5`) for this arm.
