# External Output Sensor Architecture

## Why This Exists

The current MKS ODrive clone and legacy firmware only expose a single native encoder path. On this hardware, there is no usable split between:

- motor encoder for commutation
- output encoder for joint/load feedback

That makes native dual-encoder harmonic-drive control a dead end on the current controller.

The correct phase-1 architecture is:

- `MKS ODrive clone`: inner actuator only
- `MT6835` on gearbox output: true joint-angle sensing
- `RP2040`: output-sensor bridge
- `Python backend / FOCUI`: read-only output-angle integration first

Only after that foundation is stable should an outer loop be moved onto the RP2040 or another controller.

## Chosen v1 Hardware

### Output Encoder
- `MT6835`

Why this part:
- already in stock
- absolute magnetic angle sensor
- standard 4-wire SPI
- materially better sensing foundation than `AS5600`
- avoids the mode ambiguity that wasted time on `MT6701`

### Companion MCU
- `RP2040`

Why:
- cheap and easy to replace
- enough SPI and USB bandwidth
- can serve as a pure sensor bridge first
- can later host an outer loop if needed

## Explicit Non-Goals For v1

Do not do these in the first version:
- do not wire the output encoder into the MKS as if it were a second native control encoder
- do not move the joint outer loop onto the RP2040 yet
- do not retune the old motor-side-only position/trap profiles as if they will become precision joint control

## Control Split

### Phase 1
- MKS remains responsible for:
  - motor startup calibration
  - current loop
  - velocity loop
  - simple motion execution
- RP2040 remains responsible for:
  - reading `MT6835`
  - zero offset storage in RAM
  - streaming timestamped output-angle samples
- Python backend remains responsible for:
  - combining motor angle and output angle
  - showing compliance lag
  - preparing later outer-loop experiments

### Phase 2
Only if phase 1 is stable:
- RP2040 may move to outer-loop responsibility
- MKS may become a pure velocity/torque plant

## Wiring

### MT6835 -> RP2040
- `VDD` -> `3V3`
- `GND` -> `GND`
- `SCK` -> `GP18`
- `MISO` -> `GP16`
- `MOSI` -> `GP19`
- `CSN` -> `GP17`
- optional diagnostic pin -> `GP20`

These RP2040 pins keep the sensor on a single SPI bus and leave UART/CAN options open.

### RP2040 -> Host
- `USB CDC`

This is the v1 transport. It avoids adding MKS-side communication variables before the sensor bridge itself is proven.

### Optional Later RP2040 -> MKS UART
- `RP2040 GP4 TX` -> `MKS RX`
- `RP2040 GP5 RX` <- `MKS TX`
- `GND` shared

Do not depend on this UART link in v1.

## Power Strategy

For first bring-up:
- power `RP2040` over USB
- power `MT6835` from RP2040 `3V3`
- share `GND` with the MKS only if you want combined measurements or later UART

Reason:
- this avoids starting with encoder power coming from the noisy motor board rail

## MT6835 SPI Notes

Use `SPI mode 3`:
- `CPOL = 1`
- `CPHA = 1`

Relevant device facts from the datasheet:
- 4-wire SPI interface
- angle data available through burst angle read
- MISO drives on falling edge and is captured on rising edge

## Protocol

Use a compact binary frame for both:
- USB CDC in phase 1
- UART in phase 2

Frame layout:
- `u8 sof0 = 0xA5`
- `u8 sof1 = 0x5A`
- `u8 version = 0x01`
- `u8 msg_type`
- `u16 seq`
- `u16 payload_len`
- `u8 payload[payload_len]`
- `u16 crc16_ccitt_false`

Little-endian for all multi-byte fields.

### Message Types
- `0x01 HELLO`
- `0x02 SENSOR_SAMPLE`
- `0x03 STATUS`
- `0x04 FAULT`
- `0x10 STREAM_ENABLE`
- `0x11 STREAM_DISABLE`
- `0x12 SET_ZERO`
- `0x13 REQUEST_STATUS`
- `0x14 MARK_HOME`

### Payload Definitions

`HELLO`
- `u8 encoder_kind`
- `u8 flags`
- `u16 sample_rate_hz`
- `u32 firmware_build`

`SENSOR_SAMPLE`
- `u32 timestamp_us`
- `i32 output_turns_uturn`
- `i32 output_vel_uturn_s`
- `u16 raw_angle_counts`
- `u16 mag_status_bits`
- `u16 diag_bits`
- `u16 reserved`

`STATUS`
- `u8 streaming_enabled`
- `u8 homed`
- `u16 last_fault_code`
- `i32 zero_offset_counts`
- `u16 sample_rate_hz`
- `u16 reserved`

`FAULT`
- `u16 fault_code`
- `u16 detail`
- `u32 timestamp_us`

## Repo Structure

### New Files
- `/Users/acs/Development/Robot/rp2040/output_encoder_bridge/CMakeLists.txt`
- `/Users/acs/Development/Robot/rp2040/output_encoder_bridge/src/main.c`
- `/Users/acs/Development/Robot/rp2040/output_encoder_bridge/src/mt6835.c`
- `/Users/acs/Development/Robot/rp2040/output_encoder_bridge/src/mt6835.h`
- `/Users/acs/Development/Robot/rp2040/output_encoder_bridge/src/bridge_protocol.c`
- `/Users/acs/Development/Robot/rp2040/output_encoder_bridge/src/bridge_protocol.h`
- `/Users/acs/Development/Robot/output_sensor_protocol.py`
- `/Users/acs/Development/Robot/output_sensor_bridge.py`
- `/Users/acs/Development/Robot/output_sensor_zero_calibrate.py`

### Existing Files To Extend
- `/Users/acs/Development/Robot/ui/backend/odrive_operator_backend.py`
- `/Users/acs/Development/Robot/ui/FOCUI/FOCUI/Models.swift`
- `/Users/acs/Development/Robot/ui/FOCUI/FOCUI/ContentView.swift`

## Environment Variables For v1 Backend Integration

- `ROBOT_OUTPUT_SENSOR_PORT`
- `ROBOT_OUTPUT_SENSOR_BAUD` default `921600`
- `ROBOT_OUTPUT_SENSOR_RATE_HZ` default `400`
- `ROBOT_OUTPUT_SENSOR_GEAR_RATIO` default `25`

If `ROBOT_OUTPUT_SENSOR_PORT` is unset, the backend should behave exactly as before.

## Validation Order

1. RP2040 enumerates over USB CDC
2. backend can parse `HELLO` and `STATUS`
3. hand-rotating the output changes `output_turns`
4. `FOCUI` shows output angle and lag
5. only then try output-angle-aware control experiments

## Sources

- [MT6835 datasheet](https://www.magntek.com.cn/upload/pdf/202407/MT6835_Rev.1.3.pdf)
- [RP2040 documentation](https://www.raspberrypi.com/documentation/microcontrollers/rp2040.html)
