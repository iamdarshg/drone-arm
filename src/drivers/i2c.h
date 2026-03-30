#ifndef DRIVERS_I2C_H
#define DRIVERS_I2C_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void i2c_init(uint32_t bitrate_hz);
bool i2c_write(uint8_t addr, const uint8_t *data, size_t len);
bool i2c_read(uint8_t addr, uint8_t *data, size_t len);
bool i2c_write_dma(uint32_t tx_channel, uint8_t addr, const uint8_t *data, size_t len);
bool i2c_read_dma(uint32_t tx_channel, uint32_t rx_channel, uint8_t addr, uint8_t *data, size_t len);

/* Pure-computation helper exposed for host tests. */
void i2c_calc_scl_counts(uint32_t peri_clk, uint32_t bitrate,
                         uint32_t *hcnt, uint32_t *lcnt);
size_t i2c_format_write_cmds(const uint8_t *data, size_t len, uint16_t *cmd_buf, size_t cap);
size_t i2c_format_read_cmds(size_t len, uint16_t *cmd_buf, size_t cap);

#endif
