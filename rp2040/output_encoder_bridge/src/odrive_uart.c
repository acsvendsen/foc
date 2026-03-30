#include "odrive_uart.h"

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"

static odrive_uart_config_t g_cfg;
static bool g_ready = false;

void odrive_uart_init(const odrive_uart_config_t *cfg) {
    if (cfg == NULL || cfg->uart == NULL) {
        return;
    }
    memcpy(&g_cfg, cfg, sizeof(g_cfg));

    uart_init(g_cfg.uart, g_cfg.baud);
    gpio_set_function(g_cfg.tx_pin, GPIO_FUNC_UART);
    gpio_set_function(g_cfg.rx_pin, GPIO_FUNC_UART);

    /* 8N1, no flow control */
    uart_set_hw_flow(g_cfg.uart, false, false);
    uart_set_format(g_cfg.uart, 8, 1, UART_PARITY_NONE);

    /* Enable TX FIFO so writes don't block on single bytes */
    uart_set_fifo_enabled(g_cfg.uart, true);

    g_ready = true;
}

/*
 * Write a null-terminated string over UART.
 * Uses blocking write — each command is short (~15 bytes) and the FIFO
 * absorbs it instantly at 115200 baud.
 */
static void uart_write_str(const char *s) {
    if (!g_ready) {
        return;
    }
    uart_puts(g_cfg.uart, s);
}

void odrive_uart_set_velocity(float vel_turns_s, float torque_ff) {
    char buf[40];
    snprintf(buf, sizeof(buf), "v %u %.5f %.3f\n",
             (unsigned)g_cfg.axis, (double)vel_turns_s, (double)torque_ff);
    uart_write_str(buf);
}

void odrive_uart_set_position(float pos_turns, float vel_ff, float torque_ff) {
    char buf[48];
    snprintf(buf, sizeof(buf), "p %u %.5f %.5f %.3f\n",
             (unsigned)g_cfg.axis, (double)pos_turns, (double)vel_ff, (double)torque_ff);
    uart_write_str(buf);
}

void odrive_uart_set_torque(float torque_a) {
    char buf[32];
    snprintf(buf, sizeof(buf), "c %u %.4f\n",
             (unsigned)g_cfg.axis, (double)torque_a);
    uart_write_str(buf);
}

bool odrive_uart_is_ready(void) {
    return g_ready;
}
