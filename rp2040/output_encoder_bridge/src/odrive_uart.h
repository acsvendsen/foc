#ifndef OUTPUT_ENCODER_BRIDGE_ODRIVE_UART_H
#define OUTPUT_ENCODER_BRIDGE_ODRIVE_UART_H

/*
 * ODrive v3.6 ASCII protocol over UART.
 *
 * The ODrive exposes a simple text protocol on its UART pins (GPIO1=TX,
 * GPIO2=RX on the ODrive board).  We connect from the RP2040 via UART0:
 *   RP2040 GP0 (TX)  -->  ODrive GPIO2 (RX)
 *   RP2040 GP1 (RX)  <--  ODrive GPIO1 (TX)
 *
 * The baud rate must match the ODrive configuration (default 115200).
 *
 * Command format examples:
 *   "v 0 1.234 0\n"   -- velocity command, axis 0, 1.234 turns/s, 0 torque FF
 *   "p 0 3.5 0 0\n"   -- position command, axis 0, 3.5 turns, 0 vel FF, 0 torque FF
 *   "c 0 0.5\n"        -- torque (current) command, axis 0, 0.5 A
 */

#include <stdbool.h>
#include <stdint.h>

#include "hardware/uart.h"

typedef struct {
    uart_inst_t *uart;
    uint         tx_pin;
    uint         rx_pin;
    uint32_t     baud;
    uint8_t      axis;       /* ODrive axis index (0 or 1) */
} odrive_uart_config_t;

void     odrive_uart_init(const odrive_uart_config_t *cfg);

/* Send a velocity command: v <axis> <vel_turns_s> <torque_ff>\n */
void     odrive_uart_set_velocity(float vel_turns_s, float torque_ff);

/* Send a position command: p <axis> <pos_turns> <vel_ff> <torque_ff>\n */
void     odrive_uart_set_position(float pos_turns, float vel_ff, float torque_ff);

/* Send a torque/current command: c <axis> <torque_a>\n */
void     odrive_uart_set_torque(float torque_a);

/* Returns true if UART has been initialised successfully. */
bool     odrive_uart_is_ready(void);

#endif
