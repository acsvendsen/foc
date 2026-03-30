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
#include "odrive_uart.h"
#include "outer_loop.h"

/* ---------------------------------------------------------------------------
 * Timing & rates
 *
 * The sensor loop runs at SENSOR_RATE_HZ — this is also the outer-loop
 * controller rate.  USB CDC streaming to the host is decimated to
 * STREAM_RATE_HZ for bandwidth.  Outer-loop telemetry is sent at
 * LOOP_TELEM_RATE_HZ.  UART commands to ODrive are sent every iteration
 * when the loop is enabled (the ODrive's input mode accepts overwriting).
 * -------------------------------------------------------------------------*/
#define SENSOR_RATE_HZ         2000u   /* MT6835 read + outer loop  */
#define STREAM_RATE_HZ          200u   /* Sensor samples to host    */
#define LOOP_TELEM_RATE_HZ       50u   /* Outer-loop status to host */
#define META_RATE_HZ               1u  /* Status heartbeat          */

#define SENSOR_PERIOD_US   (1000000u / SENSOR_RATE_HZ)
#define STREAM_DECIMATION  (SENSOR_RATE_HZ / STREAM_RATE_HZ)
#define LOOP_TELEM_DECIM   (SENSOR_RATE_HZ / LOOP_TELEM_RATE_HZ)

#define BRIDGE_FIRMWARE_BUILD 0x20260330u
#define BRIDGE_SPI_BAUD_HZ   1000000u
#define MT6835_COUNTS_PER_TURN 2097152
#define MT6835_HALF_TURN_COUNTS (MT6835_COUNTS_PER_TURN / 2)

/* --- Pin assignments --- */
#define PIN_SPI_MISO 16u
#define PIN_SPI_CS   17u
#define PIN_SPI_SCK  18u
#define PIN_SPI_MOSI 19u
#define PIN_SPI_DIAG 20u

#define PIN_UART_TX   0u   /* RP2040 GP0 → ODrive GPIO2 (RX) */
#define PIN_UART_RX   1u   /* RP2040 GP1 ← ODrive GPIO1 (TX) */
#define ODRIVE_BAUD   115200u

/* ---------------------------------------------------------------------------
 * Globals
 * -------------------------------------------------------------------------*/
static uint16_t g_seq = 1u;
static int32_t  g_zero_offset_counts = 0;
static bool     g_streaming_enabled = true;
static bool     g_homed = false;
static uint16_t g_last_fault_code = 0u;
static uint32_t g_last_raw_counts = 0u;
static uint32_t g_last_timestamp_us = 0u;

/* Outer loop */
static outer_loop_state_t  g_loop_state;
static outer_loop_gains_t  g_loop_gains;

/*
 * Velocity direction sign applied to the ODrive command.
 *
 * +1: ODrive positive velocity → encoder position increasing (default).
 * -1: ODrive positive velocity → encoder position decreasing.
 *
 * Set this to -1 when output_sign = -1 (encoder is mounted "backwards"
 * relative to the ODrive motor direction).  Sent from the host as the
 * optional 4th int32 in the SET_LOOP_GAINS payload (sign_milli = ±1000).
 */
static int g_vel_sign = +1;

/*
 * Multi-turn position accumulator.
 *
 * The MT6835 is a single-turn absolute encoder (0..2^21-1 counts per rev).
 * counts_delta() handles wrap-around correctly for incremental deltas, but
 * the outer loop needs a position that grows monotonically across multiple
 * revolutions — otherwise the PD controller sees the error sign flip every
 * half-turn and the motor spins endlessly.
 *
 * g_acc_counts accumulates the running total of incremental deltas from the
 * homed zero point.  It is reset to the single-turn position whenever a new
 * zero offset is set (SET_ZERO / MARK_HOME).
 */
static int32_t  g_acc_counts = 0;          /* accumulated output counts      */
static uint32_t g_acc_prev_raw = 0u;       /* raw counts at last sample      */
static bool     g_acc_valid = false;       /* first sample not yet received  */

/* Host RX buffer */
static uint8_t g_rx_buffer[BRIDGE_MAX_FRAME_LEN * 2u] = {0};
static size_t  g_rx_len = 0u;

/* ---------------------------------------------------------------------------
 * Helpers
 * -------------------------------------------------------------------------*/
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

/* ---------------------------------------------------------------------------
 * Frame emitters
 * -------------------------------------------------------------------------*/
static void emit_hello(void) {
    uint8_t frame[BRIDGE_MAX_FRAME_LEN] = {0};
    size_t len = bridge_encode_hello(next_seq(), SENSOR_RATE_HZ, BRIDGE_FIRMWARE_BUILD, frame, sizeof(frame));
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
        .sample_rate_hz = SENSOR_RATE_HZ,
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

    payload.timestamp_us       = now_us;
    payload.output_turns_uturn = output_turns_uturn;
    payload.output_vel_uturn_s = vel_uturn_s;
    payload.raw_angle_counts   = sample->raw_angle_counts;
    payload.mag_status_bits    = sample->mag_status_bits;
    payload.diag_bits          = sample->diag_bits;
    payload.reserved           = 0u;

    size_t len = bridge_encode_sensor_sample(next_seq(), &payload, frame, sizeof(frame));
    if (len > 0u) {
        write_frame(frame, len);
    }

    g_last_raw_counts    = sample->raw_angle_counts;
    g_last_timestamp_us  = now_us;
}

static void emit_loop_status(void) {
    uint8_t frame[BRIDGE_MAX_FRAME_LEN] = {0};
    bridge_loop_status_t ls = {
        .setpoint_uturn   = (int32_t)llround((double)g_loop_state.setpoint   * 1000000.0),
        .position_uturn   = (int32_t)llround((double)g_loop_state.position   * 1000000.0),
        .error_uturn      = (int32_t)llround((double)g_loop_state.error      * 1000000.0),
        .vel_cmd_umturn_s = (int32_t)llround((double)g_loop_state.vel_command * 1000000.0),
        .tick_count       = g_loop_state.tick_count,
        .enabled          = g_loop_state.enabled ? 1u : 0u,
    };
    size_t len = bridge_encode_loop_status(next_seq(), &ls, frame, sizeof(frame));
    if (len > 0u) {
        write_frame(frame, len);
    }
}

static void emit_fault(uint16_t fault_code, uint16_t detail) {
    uint8_t frame[BRIDGE_MAX_FRAME_LEN] = {0};
    size_t len = bridge_encode_fault(next_seq(), fault_code, detail, (uint32_t)to_us_since_boot(get_absolute_time()), frame, sizeof(frame));
    if (len > 0u) {
        write_frame(frame, len);
    }
}

/* ---------------------------------------------------------------------------
 * Host command parsing helpers
 * -------------------------------------------------------------------------*/
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
            /* Reset accumulator — position is now zero at this offset */
            g_acc_counts = 0;
            g_acc_valid  = false;
            emit_status();
            return true;
        case BRIDGE_MSG_MARK_HOME:
            g_zero_offset_counts = (int32_t)g_last_raw_counts;
            g_homed = true;
            /* Reset accumulator — current shaft position becomes zero */
            g_acc_counts = 0;
            g_acc_valid  = false;
            emit_status();
            return true;

        /* --- Outer-loop commands ---------------------------------------- */
        case BRIDGE_MSG_SET_SETPOINT:
            if (payload_len < 4u) {
                return false;
            }
            /* Payload: int32 setpoint in µturns (output side). */
            outer_loop_set_setpoint(
                &g_loop_state,
                (float)get_i32_le(payload) / 1000000.0f
            );
            return true;

        case BRIDGE_MSG_SET_LOOP_GAINS:
            if (payload_len < 12u) {
                return false;
            }
            /* Payload: int32 kp_milli, int32 kd_milli, int32 vel_limit_milli */
            outer_loop_set_gains(
                &g_loop_gains,
                (float)get_i32_le(&payload[0]) / 1000.0f,
                (float)get_i32_le(&payload[4]) / 1000.0f,
                (float)get_i32_le(&payload[8]) / 1000.0f
            );
            return true;

        case BRIDGE_MSG_SET_LOOP_ENABLE: {
            if (payload_len < 1u) {
                return false;
            }
            bool enable = (payload[0] != 0u);
            outer_loop_set_enabled(&g_loop_state, enable);
            /* Drive the ODrive axis state to match:
             *   enable  → AXIS_STATE_CLOSED_LOOP_CONTROL (8)
             *   disable → AXIS_STATE_IDLE                (1)
             * This means the FOCUI toggle is a single-click arm/disarm. */
            odrive_uart_set_axis_state(enable ? 8u : 1u);
            emit_loop_status();   /* Immediate feedback to host */
            return true;
        }

        default:
            return false;
    }
}

/* ---------------------------------------------------------------------------
 * Host command ingestion (same SOF/CRC framing as before)
 * -------------------------------------------------------------------------*/
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
        const uint8_t *cmd_payload = &g_rx_buffer[8];
        if (!process_frame(msg_type, cmd_payload, payload_len)) {
            emit_fault(2u, msg_type);
        }
        if (g_rx_len > frame_len) {
            memmove(g_rx_buffer, g_rx_buffer + frame_len, g_rx_len - frame_len);
        }
        g_rx_len -= frame_len;
    }
}

/* ===========================================================================
 * Main
 * =========================================================================*/
int main(void) {
    mt6835_config_t sensor_cfg = {
        .spi       = spi0,
        .cs_pin    = PIN_SPI_CS,
        .sck_pin   = PIN_SPI_SCK,
        .mosi_pin  = PIN_SPI_MOSI,
        .miso_pin  = PIN_SPI_MISO,
        .diag_pin  = PIN_SPI_DIAG,
        .spi_baud_hz = BRIDGE_SPI_BAUD_HZ,
    };
    odrive_uart_config_t uart_cfg = {
        .uart   = uart0,
        .tx_pin = PIN_UART_TX,
        .rx_pin = PIN_UART_RX,
        .baud   = ODRIVE_BAUD,
        .axis   = 0u,
    };
    mt6835_sample_t sample;
    absolute_time_t next_tick;
    absolute_time_t next_meta_tick;
    uint32_t tick_counter = 0u;

    stdio_init_all();
    sleep_ms(1200);

    mt6835_init(&sensor_cfg);
    odrive_uart_init(&uart_cfg);
    outer_loop_init(&g_loop_state, &g_loop_gains);

    emit_hello();
    emit_status();

    next_tick      = make_timeout_time_us(SENSOR_PERIOD_US);
    next_meta_tick = make_timeout_time_ms(1000u);

    while (true) {
        /* ---- Process any pending host commands ---- */
        process_host_commands();

        /* ---- 1 Hz heartbeat ---- */
        if (absolute_time_diff_us(get_absolute_time(), next_meta_tick) <= 0) {
            emit_status();
            next_meta_tick = delayed_by_ms(next_meta_tick, 1000u);
        }

        /* ---- Read sensor ---- */
        bool sample_ok = mt6835_read_sample(&sample);

        if (sample_ok) {
            uint32_t now_us = (uint32_t)to_us_since_boot(get_absolute_time());

            /*
             * Accumulate multi-turn position.
             *
             * On the first valid sample after power-on or a home event,
             * seed the accumulator with the single-turn position relative
             * to the current zero offset.  On subsequent samples, add the
             * incremental delta — which wraps correctly within ±half-turn
             * as long as the shaft doesn't move >0.5 turns between samples
             * (impossible at 2 kHz; the shaft would need >1000 t/s).
             */
            float output_vel_turns_s = 0.0f;

            if (!g_acc_valid) {
                g_acc_counts = counts_delta(sample.raw_angle_counts,
                                            (uint32_t)g_zero_offset_counts);
                g_acc_prev_raw = sample.raw_angle_counts;
                g_acc_valid    = true;
            } else {
                int32_t delta = counts_delta(sample.raw_angle_counts, g_acc_prev_raw);
                g_acc_counts  += delta;
                g_acc_prev_raw = sample.raw_angle_counts;

                if (g_last_timestamp_us != 0u) {
                    uint32_t delta_us = now_us - g_last_timestamp_us;
                    if (delta_us > 0u) {
                        float delta_turns = (float)delta / (float)MT6835_COUNTS_PER_TURN;
                        output_vel_turns_s = delta_turns / ((float)delta_us / 1000000.0f);
                    }
                }
            }

            float output_pos_turns = (float)g_acc_counts / (float)MT6835_COUNTS_PER_TURN;

            /* ---- Run outer loop controller ---- */
            float vel_cmd = outer_loop_update(
                &g_loop_state,
                &g_loop_gains,
                output_pos_turns,
                output_vel_turns_s
            );

            /* Send velocity command to ODrive if loop is enabled. */
            if (g_loop_state.enabled && odrive_uart_is_ready()) {
                odrive_uart_set_velocity(vel_cmd, 0.0f);
            }

            /* ---- Decimated USB CDC streaming ---- */
            if (g_streaming_enabled && (tick_counter % STREAM_DECIMATION) == 0u) {
                emit_sample(&sample);
            }

            /* ---- Decimated outer-loop telemetry ---- */
            if (g_loop_state.enabled && (tick_counter % LOOP_TELEM_DECIM) == 0u) {
                emit_loop_status();
            }

            /* Update last-sample state (used by both velocity calc and emit_sample) */
            g_last_raw_counts   = sample.raw_angle_counts;
            g_last_timestamp_us = (uint32_t)to_us_since_boot(get_absolute_time());
        }

        tick_counter++;

        /* ---- Wait for next tick ---- */
        sleep_until(next_tick);
        next_tick = delayed_by_us(next_tick, SENSOR_PERIOD_US);
    }
}
