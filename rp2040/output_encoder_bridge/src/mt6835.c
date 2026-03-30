#include "mt6835.h"

#include <string.h>

#include "hardware/gpio.h"

#define MT6835_REG_ANGLE_MSB      0x003u
#define MT6835_REG_ANGLE_CRC      0x006u

#define MT6835_CMD_READ           0x03u
#define MT6835_ANGLE_MASK         0x001FFFFFu

/*
 * MT6835 CRC-8 (polynomial 0x07, initial value 0x00).
 * Covers the three angle/status bytes (reg 0x003..0x005).
 * The expected CRC is in reg 0x006.
 */
static uint8_t mt6835_crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00u;
    for (size_t i = 0u; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            if ((crc & 0x80u) != 0u) {
                crc = (uint8_t)((crc << 1) ^ 0x07u);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static mt6835_config_t g_cfg;

void mt6835_init(const mt6835_config_t *config) {
    if (config == NULL) {
        return;
    }
    memcpy(&g_cfg, config, sizeof(g_cfg));
    spi_init(g_cfg.spi, g_cfg.spi_baud_hz);
    spi_set_format(g_cfg.spi, 8u, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

    gpio_set_function(g_cfg.sck_pin, GPIO_FUNC_SPI);
    gpio_set_function(g_cfg.mosi_pin, GPIO_FUNC_SPI);
    gpio_set_function(g_cfg.miso_pin, GPIO_FUNC_SPI);

    gpio_init(g_cfg.cs_pin);
    gpio_set_dir(g_cfg.cs_pin, GPIO_OUT);
    gpio_put(g_cfg.cs_pin, 1);

    if (g_cfg.diag_pin != 0u) {
        gpio_init(g_cfg.diag_pin);
        gpio_set_dir(g_cfg.diag_pin, GPIO_IN);
        gpio_pull_up(g_cfg.diag_pin);
    }
}

bool mt6835_read_sample(mt6835_sample_t *out_sample) {
    /*
     * Sequential read from register 0x003.
     *
     * One 6-byte SPI transaction: [cmd_byte, addr_byte, d, d, d, d].
     * The MT6835 auto-increments while CS is held low, so:
     *   rx[2] = reg 0x003  (angle[20:13])
     *   rx[3] = reg 0x004  (angle[12:5])
     *   rx[4] = reg 0x005  (angle[4:0] | status[2:0])
     *   rx[5] = reg 0x006  (CRC-8)
     */
    uint8_t tx[6] = {0u};
    uint8_t rx[6] = {0u};
    uint8_t reg003, reg004, reg005, reg006_crc;
    uint32_t angle_counts;
    uint16_t diag_bits = 0u;

    if (out_sample == NULL || g_cfg.spi == NULL) {
        return false;
    }

    tx[0] = (uint8_t)((MT6835_CMD_READ << 4) | ((MT6835_REG_ANGLE_MSB >> 8) & 0x0Fu));
    tx[1] = (uint8_t)(MT6835_REG_ANGLE_MSB & 0xFFu);

    gpio_put(g_cfg.cs_pin, 0);
    spi_write_read_blocking(g_cfg.spi, tx, rx, 6u);
    gpio_put(g_cfg.cs_pin, 1);

    reg003     = rx[2];
    reg004     = rx[3];
    reg005     = rx[4];
    reg006_crc = rx[5];

    /* Verify sensor CRC before accepting data */
    uint8_t crc_data[3] = {reg003, reg004, reg005};
    if (mt6835_crc8(crc_data, 3u) != reg006_crc) {
        return false;
    }

    angle_counts = (((uint32_t)reg003) << 13)
                 | (((uint32_t)reg004) << 5)
                 | (((uint32_t)reg005) >> 3);
    angle_counts &= MT6835_ANGLE_MASK;

    if (g_cfg.diag_pin != 0u && gpio_get(g_cfg.diag_pin) == 0u) {
        diag_bits |= 0x0001u;
    }

    out_sample->raw_angle_counts = angle_counts;
    out_sample->mag_status_bits  = (uint16_t)(reg005 & 0x07u);
    out_sample->diag_bits        = diag_bits;

    return true;
}
