#ifndef OUTPUT_ENCODER_BRIDGE_PROTOCOL_H
#define OUTPUT_ENCODER_BRIDGE_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BRIDGE_SOF0 0xA5
#define BRIDGE_SOF1 0x5A
#define BRIDGE_PROTOCOL_VERSION 0x01

#define BRIDGE_MSG_HELLO          0x01
#define BRIDGE_MSG_SENSOR_SAMPLE  0x02
#define BRIDGE_MSG_STATUS         0x03
#define BRIDGE_MSG_FAULT          0x04
#define BRIDGE_MSG_LOOP_STATUS    0x05   /* Outer-loop telemetry (device→host) */

#define BRIDGE_MSG_STREAM_ENABLE  0x10
#define BRIDGE_MSG_STREAM_DISABLE 0x11
#define BRIDGE_MSG_SET_ZERO       0x12
#define BRIDGE_MSG_REQUEST_STATUS 0x13
#define BRIDGE_MSG_MARK_HOME      0x14

/* Outer-loop control commands (host→device) */
#define BRIDGE_MSG_SET_SETPOINT    0x20  /* Set position target (output µturns) */
#define BRIDGE_MSG_SET_LOOP_GAINS  0x21  /* Set Kp, Kd, vel_limit (milligain)  */
#define BRIDGE_MSG_SET_LOOP_ENABLE 0x22  /* Enable/disable outer loop          */

#define BRIDGE_ENCODER_MT6835     0x01

#define BRIDGE_MAX_PAYLOAD_LEN 32u
#define BRIDGE_MAX_FRAME_LEN (8u + BRIDGE_MAX_PAYLOAD_LEN + 2u)

typedef struct {
    uint32_t timestamp_us;
    int32_t output_turns_uturn;
    int32_t output_vel_uturn_s;
    uint32_t raw_angle_counts;
    uint16_t mag_status_bits;
    uint16_t diag_bits;
    uint16_t reserved;
} bridge_sensor_sample_t;

typedef struct {
    uint8_t streaming_enabled;
    uint8_t homed;
    uint16_t last_fault_code;
    int32_t zero_offset_counts;
    uint16_t sample_rate_hz;
    uint16_t reserved;
} bridge_status_t;

/* Outer-loop telemetry payload (24 bytes, fits in BRIDGE_MAX_PAYLOAD_LEN). */
typedef struct {
    int32_t  setpoint_uturn;     /* Target output position  [µturns]        */
    int32_t  position_uturn;     /* Measured output position [µturns]       */
    int32_t  error_uturn;        /* setpoint - position      [µturns]       */
    int32_t  vel_cmd_umturn_s;   /* Motor velocity command   [µ-motor-t/s]  */
    uint32_t tick_count;
    uint8_t  enabled;
    uint8_t  _pad[3];
} bridge_loop_status_t;

uint16_t bridge_crc16_ccitt_false(const uint8_t *data, size_t len);
size_t bridge_encode_frame(uint8_t msg_type, uint16_t seq, const uint8_t *payload, uint16_t payload_len, uint8_t *out_buf, size_t out_cap);
size_t bridge_encode_hello(uint16_t seq, uint16_t sample_rate_hz, uint32_t firmware_build, uint8_t *out_buf, size_t out_cap);
size_t bridge_encode_sensor_sample(uint16_t seq, const bridge_sensor_sample_t *sample, uint8_t *out_buf, size_t out_cap);
size_t bridge_encode_status(uint16_t seq, const bridge_status_t *status, uint8_t *out_buf, size_t out_cap);
size_t bridge_encode_fault(uint16_t seq, uint16_t fault_code, uint16_t detail, uint32_t timestamp_us, uint8_t *out_buf, size_t out_cap);
size_t bridge_encode_loop_status(uint16_t seq, const bridge_loop_status_t *ls, uint8_t *out_buf, size_t out_cap);

#endif
