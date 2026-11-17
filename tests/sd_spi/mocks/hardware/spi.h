#pragma once

#include <stddef.h>
#include <stdint.h>

typedef struct {
    unsigned index;
} spi_inst_t;

extern spi_inst_t mock_spi_instances[2];
#define spi0 (&mock_spi_instances[0])
#define spi1 (&mock_spi_instances[1])
#define SPI_CPOL_0 0
#define SPI_CPHA_0 0
#define SPI_MSB_FIRST 1

int spi_write_read_blocking(spi_inst_t *spi, const uint8_t *src, uint8_t *dst, size_t len);
int spi_read_blocking(spi_inst_t *spi, uint8_t repeated_tx_data, uint8_t *dst, size_t len);
int spi_write_blocking(spi_inst_t *spi, const uint8_t *src, size_t len);

unsigned spi_init(spi_inst_t *spi, unsigned baudrate);

unsigned spi_set_baudrate(spi_inst_t *spi, unsigned baudrate);
void spi_set_format(spi_inst_t *spi, unsigned bits, unsigned cpol, unsigned cpha, unsigned order);
