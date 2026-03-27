#include "bridge_protocol.h"

#include <string.h>

static void put_u16_le(uint8_t *dst, uint16_t value) {
    dst[0] = (uint8_t)(value & 0xFFu);
    dst[1] = (uint8_t)((value >> 8) & 0xFFu);
}

static void put_u32_le(uint8_t *dst, uint32_t value) {
    dst[0] = (uint8_t)(value & 0xFFu);
    dst[1] = (uint8_t)((value >> 8) & 0xFFu);
    dst[2] = (uint8_t)((value >> 16) & 0xFFu);
    dst[3] = (uint8_t)((value >> 24) & 0xFFu);
}

uint16_t bridge_crc16_ccitt_false(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int bit = 0; bit < 8; ++bit) {
            if ((crc & 0x8000u) != 0u) {
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

size_t bridge_encode_frame(uint8_t msg_type, uint16_t seq, const uint8_t *payload, uint16_t payload_len, uint8_t *out_buf, size_t out_cap) {
    const size_t total_len = 8u + (size_t)payload_len + 2u;
    uint16_t crc = 0u;
    if (out_buf == NULL || total_len > out_cap || payload_len > BRIDGE_MAX_PAYLOAD_LEN) {
        return 0u;
    }
    out_buf[0] = BRIDGE_SOF0;
    out_buf[1] = BRIDGE_SOF1;
    out_buf[2] = BRIDGE_PROTOCOL_VERSION;
    out_buf[3] = msg_type;
    put_u16_le(&out_buf[4], seq);
    put_u16_le(&out_buf[6], payload_len);
    if (payload_len > 0u && payload != NULL) {
        memcpy(&out_buf[8], payload, payload_len);
    }
    crc = bridge_crc16_ccitt_false(&out_buf[2], 6u + (size_t)payload_len);
    put_u16_le(&out_buf[8u + payload_len], crc);
    return total_len;
}

size_t bridge_encode_hello(uint16_t seq, uint16_t sample_rate_hz, uint32_t firmware_build, uint8_t *out_buf, size_t out_cap) {
    uint8_t payload[8] = {0};
    payload[0] = BRIDGE_ENCODER_MT6835;
    payload[1] = 0u;
    put_u16_le(&payload[2], sample_rate_hz);
    put_u32_le(&payload[4], firmware_build);
    return bridge_encode_frame(BRIDGE_MSG_HELLO, seq, payload, (uint16_t)sizeof(payload), out_buf, out_cap);
}

size_t bridge_encode_sensor_sample(uint16_t seq, const bridge_sensor_sample_t *sample, uint8_t *out_buf, size_t out_cap) {
    uint8_t payload[20] = {0};
    if (sample == NULL) {
        return 0u;
    }
    put_u32_le(&payload[0], sample->timestamp_us);
    put_u32_le(&payload[4], (uint32_t)sample->output_turns_uturn);
    put_u32_le(&payload[8], (uint32_t)sample->output_vel_uturn_s);
    put_u16_le(&payload[12], sample->raw_angle_counts);
    put_u16_le(&payload[14], sample->mag_status_bits);
    put_u16_le(&payload[16], sample->diag_bits);
    put_u16_le(&payload[18], sample->reserved);
    return bridge_encode_frame(BRIDGE_MSG_SENSOR_SAMPLE, seq, payload, (uint16_t)sizeof(payload), out_buf, out_cap);
}

size_t bridge_encode_status(uint16_t seq, const bridge_status_t *status, uint8_t *out_buf, size_t out_cap) {
    uint8_t payload[12] = {0};
    if (status == NULL) {
        return 0u;
    }
    payload[0] = status->streaming_enabled;
    payload[1] = status->homed;
    put_u16_le(&payload[2], status->last_fault_code);
    put_u32_le(&payload[4], (uint32_t)status->zero_offset_counts);
    put_u16_le(&payload[8], status->sample_rate_hz);
    put_u16_le(&payload[10], status->reserved);
    return bridge_encode_frame(BRIDGE_MSG_STATUS, seq, payload, 12u, out_buf, out_cap);
}

size_t bridge_encode_fault(uint16_t seq, uint16_t fault_code, uint16_t detail, uint32_t timestamp_us, uint8_t *out_buf, size_t out_cap) {
    uint8_t payload[8] = {0};
    put_u16_le(&payload[0], fault_code);
    put_u16_le(&payload[2], detail);
    put_u32_le(&payload[4], timestamp_us);
    return bridge_encode_frame(BRIDGE_MSG_FAULT, seq, payload, (uint16_t)sizeof(payload), out_buf, out_cap);
}
