#include "mt6835.h"

#include <string.h>

#include "hardware/gpio.h"

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
    uint8_t tx[3] = {0xA0u, 0x00u, 0x00u};
    uint8_t rx[3] = {0u, 0u, 0u};
    uint32_t raw = 0u;

    if (out_sample == NULL || g_cfg.spi == NULL) {
        return false;
    }

    gpio_put(g_cfg.cs_pin, 0);
    spi_write_read_blocking(g_cfg.spi, tx, rx, 3u);
    gpio_put(g_cfg.cs_pin, 1);

    raw = ((uint32_t)rx[0] << 16) | ((uint32_t)rx[1] << 8) | (uint32_t)rx[2];

    out_sample->raw_angle_counts = (uint16_t)(raw & 0xFFFFu);
    out_sample->mag_status_bits = 0u;
    out_sample->diag_bits = (g_cfg.diag_pin != 0u && gpio_get(g_cfg.diag_pin) == 0u) ? 0x0001u : 0u;

    return true;
}
