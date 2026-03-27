# RP2040 Output Encoder Bridge

This directory is the phase-1 firmware scaffold for the external output-sensor bridge.

## Current Scope

This firmware is meant to do one job first:
- read the output encoder
- stream absolute angle samples over USB CDC

It is **not** yet the outer motion controller.

## What Is Already Defined

- fixed binary bridge protocol shared with Python
- RP2040 SPI pin assignment for `MT6835`
- USB CDC streaming path
- minimal `HELLO`, `STATUS`, and `SENSOR_SAMPLE` frame generation

## What Still Needs Bench Validation

These parts are still scaffolding until tested on hardware:
- the exact `MT6835` burst-angle transaction
- raw count width/mask handling for the chosen module
- zeroing semantics and home command handling
- host command parsing on USB CDC
- magnetic-status and diagnostic register reads

Do not treat the current `mt6835.c` driver as production-ready until a bench trace confirms the returned angle word format.

## Intended Bring-Up Order

1. Build and flash the RP2040 firmware
2. Verify USB CDC enumeration
3. Verify `HELLO` frame reception on the host
4. Verify hand rotation changes `output_turns`
5. Only then wire the bridge into the backend/FOCUI workflow

## Expected Host Environment

The Python bridge client uses:
- `ROBOT_OUTPUT_SENSOR_PORT`
- `ROBOT_OUTPUT_SENSOR_BAUD`
- `ROBOT_OUTPUT_SENSOR_RATE_HZ`
- `ROBOT_OUTPUT_SENSOR_GEAR_RATIO`

## Build Prerequisites

- `PICO_SDK_PATH` must be set
- Pico SDK with `pico_stdlib` and `hardware_spi`

Example:
```bash
export PICO_SDK_PATH=/path/to/pico-sdk
cmake -S /Users/acs/Development/Robot/rp2040/output_encoder_bridge -B /tmp/output_encoder_bridge-build
cmake --build /tmp/output_encoder_bridge-build
```
