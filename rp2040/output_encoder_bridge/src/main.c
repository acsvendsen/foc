#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"

#include "bridge_protocol.h"
#include "mt6835.h"

#define BRIDGE_SAMPLE_RATE_HZ 400u
#define BRIDGE_FIRMWARE_BUILD 0x20260327u
#define BRIDGE_SPI_BAUD_HZ 1000000u

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
static uint16_t g_last_raw_counts = 0u;
static int32_t g_last_output_turns_uturn = 0;
static uint32_t g_last_timestamp_us = 0u;

static uint16_t next_seq(void) {
    uint16_t out = g_seq;
    g_seq = (uint16_t)(g_seq + 1u);
    return out;
}

static void write_frame(const uint8_t *data, size_t len) {
    fwrite(data, 1u, len, stdout);
    fflush(stdout);
}

static int32_t counts_delta(uint16_t now_counts, uint16_t then_counts) {
    int32_t delta = (int32_t)now_counts - (int32_t)then_counts;
    if (delta > 32768) {
        delta -= 65536;
    } else if (delta < -32768) {
        delta += 65536;
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
    int32_t unwrapped_counts = (int32_t)sample->raw_angle_counts - g_zero_offset_counts;
    int32_t output_turns_uturn = (int32_t)llround(((double)unwrapped_counts / 65536.0) * 1000000.0);
    int32_t vel_uturn_s = 0;
    bridge_sensor_sample_t payload;

    if (g_last_timestamp_us != 0u) {
        int32_t delta_counts = counts_delta(sample->raw_angle_counts, g_last_raw_counts);
        uint32_t delta_us = now_us - g_last_timestamp_us;
        if (delta_us > 0u) {
            double delta_turns = (double)delta_counts / 65536.0;
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
    g_last_output_turns_uturn = output_turns_uturn;
    g_last_timestamp_us = now_us;
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

    stdio_init_all();
    sleep_ms(1200);

    mt6835_init(&sensor_cfg);

    emit_hello();
    emit_status();

    next_tick = make_timeout_time_ms(1000u / BRIDGE_SAMPLE_RATE_HZ);
    while (true) {
        if (g_streaming_enabled) {
            if (mt6835_read_sample(&sample)) {
                emit_sample(&sample);
            }
        }
        sleep_until(next_tick);
        next_tick = delayed_by_ms(next_tick, 1000u / BRIDGE_SAMPLE_RATE_HZ);
    }
}
