#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"

#include "bridge_protocol.h"
#include "mt6835.h"

#define BRIDGE_SAMPLE_RATE_HZ 400u
#define BRIDGE_FIRMWARE_BUILD 0x20260328u
#define BRIDGE_SPI_BAUD_HZ 1000000u
#define MT6835_COUNTS_PER_TURN 2097152
#define MT6835_HALF_TURN_COUNTS (MT6835_COUNTS_PER_TURN / 2)

#define PIN_SPI_MISO 16u
#define PIN_SPI_CS   17u
#define PIN_SPI_SCK  18u
#define PIN_SPI_MOSI 19u
#define PIN_SPI_DIAG 20u

static uint16_t g_seq = 1u;
static int32_t g_zero_offset_counts = 0;
static bool g_streaming_enabled = true;
static bool g_homed = false;
static uint16_t g_last_fault_code = 0u;
static uint32_t g_last_raw_counts = 0u;
static uint32_t g_last_timestamp_us = 0u;

static uint8_t g_rx_buffer[BRIDGE_MAX_FRAME_LEN * 2u] = {0};
static size_t g_rx_len = 0u;

static uint16_t next_seq(void) {
    uint16_t out = g_seq;
    g_seq = (uint16_t)(g_seq + 1u);
    return out;
}

static void write_frame(const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        putchar_raw((int)data[i]);
    }
    stdio_flush();
}

static int32_t counts_delta(uint32_t now_counts, uint32_t then_counts) {
    int32_t delta = (int32_t)now_counts - (int32_t)then_counts;
    if (delta > MT6835_HALF_TURN_COUNTS) {
        delta -= MT6835_COUNTS_PER_TURN;
    } else if (delta < -MT6835_HALF_TURN_COUNTS) {
        delta += MT6835_COUNTS_PER_TURN;
    }
    return delta;
}

static void emit_hello(void) {
    uint8_t frame[BRIDGE_MAX_FRAME_LEN] = {0};
    size_t len = bridge_encode_hello(next_seq(), BRIDGE_SAMPLE_RATE_HZ, BRIDGE_FIRMWARE_BUILD, frame, sizeof(frame));
    if (len > 0u) {
        write_frame(frame, len);
    }
}

static void emit_status(void) {
    uint8_t frame[BRIDGE_MAX_FRAME_LEN] = {0};
    bridge_status_t status = {
        .streaming_enabled = g_streaming_enabled ? 1u : 0u,
        .homed = g_homed ? 1u : 0u,
        .last_fault_code = g_last_fault_code,
        .zero_offset_counts = g_zero_offset_counts,
        .sample_rate_hz = BRIDGE_SAMPLE_RATE_HZ,
        .reserved = 0u,
    };
    size_t len = bridge_encode_status(next_seq(), &status, frame, sizeof(frame));
    if (len > 0u) {
        write_frame(frame, len);
    }
}

static void emit_sample(const mt6835_sample_t *sample) {
    uint8_t frame[BRIDGE_MAX_FRAME_LEN] = {0};
    uint32_t now_us = (uint32_t)to_us_since_boot(get_absolute_time());
    int32_t relative_counts = counts_delta(sample->raw_angle_counts, (uint32_t)g_zero_offset_counts);
    int32_t output_turns_uturn = (int32_t)llround(((double)relative_counts / (double)MT6835_COUNTS_PER_TURN) * 1000000.0);
    int32_t vel_uturn_s = 0;
    bridge_sensor_sample_t payload;

    if (g_last_timestamp_us != 0u) {
        int32_t delta_counts = counts_delta(sample->raw_angle_counts, g_last_raw_counts);
        uint32_t delta_us = now_us - g_last_timestamp_us;
        if (delta_us > 0u) {
            double delta_turns = (double)delta_counts / (double)MT6835_COUNTS_PER_TURN;
            double vel_turns_s = delta_turns / ((double)delta_us / 1000000.0);
            vel_uturn_s = (int32_t)llround(vel_turns_s * 1000000.0);
        }
    }

    payload.timestamp_us = now_us;
    payload.output_turns_uturn = output_turns_uturn;
    payload.output_vel_uturn_s = vel_uturn_s;
    payload.raw_angle_counts = sample->raw_angle_counts;
    payload.mag_status_bits = sample->mag_status_bits;
    payload.diag_bits = sample->diag_bits;
    payload.reserved = 0u;

    size_t len = bridge_encode_sensor_sample(next_seq(), &payload, frame, sizeof(frame));
    if (len > 0u) {
        write_frame(frame, len);
    }

    g_last_raw_counts = sample->raw_angle_counts;
    g_last_timestamp_us = now_us;
}

static uint16_t get_u16_le(const uint8_t *src) {
    return (uint16_t)src[0] | ((uint16_t)src[1] << 8);
}

static int32_t get_i32_le(const uint8_t *src) {
    uint32_t raw = (uint32_t)src[0]
                 | ((uint32_t)src[1] << 8)
                 | ((uint32_t)src[2] << 16)
                 | ((uint32_t)src[3] << 24);
    return (int32_t)raw;
}

static bool process_frame(uint8_t msg_type, const uint8_t *payload, uint16_t payload_len) {
    switch (msg_type) {
        case BRIDGE_MSG_STREAM_ENABLE:
            if (payload_len < 2u) {
                return false;
            }
            g_streaming_enabled = true;
            emit_status();
            return true;
        case BRIDGE_MSG_STREAM_DISABLE:
            g_streaming_enabled = false;
            emit_status();
            return true;
        case BRIDGE_MSG_REQUEST_STATUS:
            emit_status();
            return true;
        case BRIDGE_MSG_SET_ZERO:
            if (payload_len != 4u) {
                return false;
            }
            g_zero_offset_counts = get_i32_le(payload);
            g_homed = true;
            emit_status();
            return true;
        case BRIDGE_MSG_MARK_HOME:
            g_zero_offset_counts = (int32_t)g_last_raw_counts;
            g_homed = true;
            emit_status();
            return true;
        default:
            return false;
    }
}

static void emit_fault(uint16_t fault_code, uint16_t detail) {
    uint8_t frame[BRIDGE_MAX_FRAME_LEN] = {0};
    size_t len = bridge_encode_fault(next_seq(), fault_code, detail, (uint32_t)to_us_since_boot(get_absolute_time()), frame, sizeof(frame));
    if (len > 0u) {
        write_frame(frame, len);
    }
}

static void process_host_commands(void) {
    int ch;
    while ((ch = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        if (ch < 0) {
            break;
        }
        if (g_rx_len < sizeof(g_rx_buffer)) {
            g_rx_buffer[g_rx_len++] = (uint8_t)ch;
        } else {
            g_rx_len = 0u;
        }
    }

    while (g_rx_len >= 10u) {
        size_t sof_index = SIZE_MAX;
        for (size_t i = 0; i + 1u < g_rx_len; ++i) {
            if (g_rx_buffer[i] == BRIDGE_SOF0 && g_rx_buffer[i + 1u] == BRIDGE_SOF1) {
                sof_index = i;
                break;
            }
        }
        if (sof_index == SIZE_MAX) {
            g_rx_len = 0u;
            return;
        }
        if (sof_index > 0u) {
            memmove(g_rx_buffer, g_rx_buffer + sof_index, g_rx_len - sof_index);
            g_rx_len -= sof_index;
        }
        if (g_rx_len < 10u) {
            return;
        }
        uint16_t payload_len = get_u16_le(&g_rx_buffer[6]);
        size_t frame_len = 8u + (size_t)payload_len + 2u;
        if (payload_len > BRIDGE_MAX_PAYLOAD_LEN) {
            memmove(g_rx_buffer, g_rx_buffer + 2u, g_rx_len - 2u);
            g_rx_len -= 2u;
            continue;
        }
        if (g_rx_len < frame_len) {
            return;
        }
        uint16_t expected_crc = get_u16_le(&g_rx_buffer[8u + payload_len]);
        uint16_t actual_crc = bridge_crc16_ccitt_false(&g_rx_buffer[2], 6u + (size_t)payload_len);
        if (expected_crc != actual_crc) {
            emit_fault(1u, 1u);
            memmove(g_rx_buffer, g_rx_buffer + 2u, g_rx_len - 2u);
            g_rx_len -= 2u;
            continue;
        }
        uint8_t msg_type = g_rx_buffer[3];
        const uint8_t *payload = &g_rx_buffer[8];
        if (!process_frame(msg_type, payload, payload_len)) {
            emit_fault(2u, msg_type);
        }
        if (g_rx_len > frame_len) {
            memmove(g_rx_buffer, g_rx_buffer + frame_len, g_rx_len - frame_len);
        }
        g_rx_len -= frame_len;
    }
}

int main(void) {
    mt6835_config_t sensor_cfg = {
        .spi = spi0,
        .cs_pin = PIN_SPI_CS,
        .sck_pin = PIN_SPI_SCK,
        .mosi_pin = PIN_SPI_MOSI,
        .miso_pin = PIN_SPI_MISO,
        .diag_pin = PIN_SPI_DIAG,
        .spi_baud_hz = BRIDGE_SPI_BAUD_HZ,
    };
    mt6835_sample_t sample;
    absolute_time_t next_tick;
    absolute_time_t next_meta_tick;

    stdio_init_all();
    sleep_ms(1200);

    mt6835_init(&sensor_cfg);

    emit_hello();
    emit_status();

    /* Use microseconds to avoid integer truncation:
     * 1000 / 400 = 2 ms (wrong), 1000000 / 400 = 2500 us (correct). */
#define BRIDGE_SAMPLE_PERIOD_US (1000000u / BRIDGE_SAMPLE_RATE_HZ)

    next_tick      = make_timeout_time_us(BRIDGE_SAMPLE_PERIOD_US);
    next_meta_tick = make_timeout_time_ms(1000u);
    while (true) {
        process_host_commands();
        if (absolute_time_diff_us(get_absolute_time(), next_meta_tick) <= 0) {
            /* Status only — HELLO is static identification, sent once above. */
            emit_status();
            next_meta_tick = delayed_by_ms(next_meta_tick, 1000u);
        }
        if (g_streaming_enabled) {
            if (mt6835_read_sample(&sample)) {
                emit_sample(&sample);
            }
        }
        sleep_until(next_tick);
        next_tick = delayed_by_us(next_tick, BRIDGE_SAMPLE_PERIOD_US);
    }
}
