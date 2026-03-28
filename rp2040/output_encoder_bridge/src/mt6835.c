#include "mt6835.h"

#include <string.h>

#include "hardware/gpio.h"

#define MT6835_REG_ANGLE_MSB      0x003u
#define MT6835_REG_ANGLE_MID      0x004u
#define MT6835_REG_ANGLE_LSB_STAT 0x005u
#define MT6835_REG_ANGLE_CRC      0x006u

#define MT6835_CMD_READ_ONE_BYTE  0x03u
#define MT6835_ANGLE_MASK         0x001FFFFFu

static void mt6835_build_read_command(uint16_t address, uint8_t *dst) {
    dst[0] = (uint8_t)((MT6835_CMD_READ_ONE_BYTE << 4) | ((address >> 8) & 0x0Fu));
    dst[1] = (uint8_t)(address & 0xFFu);
    dst[2] = 0u;
}

static uint8_t mt6835_crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0u;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (uint8_t)((crc & 0x80u) != 0u ? ((crc << 1) ^ 0x07u) : (crc << 1));
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
    uint8_t tx[12] = {0u};
    uint8_t rx[12] = {0u};
    uint8_t reg003;
    uint8_t reg004;
    uint8_t reg005;
    uint8_t reg006;
    uint8_t crc_data[3];
    uint8_t crc_actual;
    uint16_t diag_bits = 0u;
    uint32_t angle_counts = 0u;

    if (out_sample == NULL || g_cfg.spi == NULL) {
        return false;
    }

    mt6835_build_read_command(MT6835_REG_ANGLE_MSB, &tx[0]);
    mt6835_build_read_command(MT6835_REG_ANGLE_MID, &tx[3]);
    mt6835_build_read_command(MT6835_REG_ANGLE_LSB_STAT, &tx[6]);
    mt6835_build_read_command(MT6835_REG_ANGLE_CRC, &tx[9]);

    gpio_put(g_cfg.cs_pin, 0);
    spi_write_read_blocking(g_cfg.spi, tx, rx, 12u);
    gpio_put(g_cfg.cs_pin, 1);

    reg003 = rx[2];
    reg004 = rx[5];
    reg005 = rx[8];
    reg006 = rx[11];

    angle_counts = (((uint32_t)reg003) << 13)
                 | (((uint32_t)reg004) << 5)
                 | (((uint32_t)reg005) >> 3);
    angle_counts &= MT6835_ANGLE_MASK;

    crc_data[0] = reg003;
    crc_data[1] = reg004;
    crc_data[2] = reg005;
    crc_actual = mt6835_crc8(crc_data, sizeof(crc_data));

    if (g_cfg.diag_pin != 0u && gpio_get(g_cfg.diag_pin) == 0u) {
        diag_bits |= 0x0001u;
    }
    if (crc_actual != reg006) {
        diag_bits |= 0x0002u;
    }

    out_sample->raw_angle_counts = angle_counts;
    out_sample->mag_status_bits = (uint16_t)(reg005 & 0x07u);
    out_sample->diag_bits = diag_bits;

    return true;
}
