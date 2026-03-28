#ifndef OUTPUT_ENCODER_BRIDGE_MT6835_H
#define OUTPUT_ENCODER_BRIDGE_MT6835_H

#include <stdbool.h>
#include <stdint.h>

#include "hardware/spi.h"

typedef struct {
    spi_inst_t *spi;
    uint cs_pin;
    uint sck_pin;
    uint mosi_pin;
    uint miso_pin;
    uint diag_pin;
    uint32_t spi_baud_hz;
} mt6835_config_t;

typedef struct {
    uint32_t raw_angle_counts;
    uint16_t mag_status_bits;
    uint16_t diag_bits;
} mt6835_sample_t;

void mt6835_init(const mt6835_config_t *config);
bool mt6835_read_sample(mt6835_sample_t *out_sample);

#endif
