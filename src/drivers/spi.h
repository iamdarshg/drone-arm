#ifndef DRIVERS_SPI_H
#define DRIVERS_SPI_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void spi_init(uint32_t bitrate_hz);
bool spi_transfer(const uint8_t *tx, uint8_t *rx, size_t len);

/* Pure-computation helper exposed for host tests. */
void spi_calc_divisors(uint32_t clk_hz, uint32_t bitrate_hz,
                       uint32_t *cpsdvsr, uint32_t *scr);

#endif
