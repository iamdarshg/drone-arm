/*
 * I2C Header - Optimized for RP2350
 * Matching SPI.h format exactly
 */
#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware_defs/i2c_config.h"

// I2C register bit definitions
#define I2C_CON_MASTER_MODE_BIT     (1U << 0)
#define I2C_CON_SPEED_SHIFT         1
#define I2C_CON_SPEED_MASK          (0x3U << I2C_CON_SPEED_SHIFT)
#define I2C_CON_SPEED_STANDARD      (0x1U << I2C_CON_SPEED_SHIFT)
#define I2C_CON_SPEED_FAST          (0x2U << I2C_CON_SPEED_SHIFT)
#define I2C_CON_SPEED_HIGH          (0x3U << I2C_CON_SPEED_SHIFT)
#define I2C_CON_10BITADDR_SLAVE_BIT (1U << 3)
#define I2C_CON_10BITADDR_MASTER_BIT (1U << 4)
#define I2C_CON_RESTART_EN_BIT      (1U << 5)
#define I2C_CON_SLAVE_DISABLE_BIT   (1U << 6)

#define I2C_ENABLE_BIT              (1U << 0)
#define I2C_ABORT_BIT               (1U << 1)

#define I2C_STATUS_ACTIVITY_BIT     (1U << 0)
#define I2C_STATUS_TFNF_BIT         (1U << 1)
#define I2C_STATUS_TFE_BIT          (1U << 2)
#define I2C_STATUS_RFNE_BIT         (1U << 3)
#define I2C_STATUS_RFF_BIT          (1U << 4)
#define I2C_STATUS_MST_ACTIVITY_BIT (1U << 5)
#define I2C_STATUS_SLV_ACTIVITY_BIT (1U << 6)

#define I2C_DATA_CMD_DAT_MASK       0xFFU
#define I2C_DATA_CMD_CMD_BIT        (1U << 8)
#define I2C_DATA_CMD_STOP_BIT       (1U << 9)
#define I2C_DATA_CMD_RESTART_BIT    (1U << 10)
#define I2C_DATA_CMD_READ           (1U << 8)

// Hardware pointer (defined in i2c.c)
extern void* i2c_get_hw(uint8_t id);

// Control functions
void i2c_init(uint8_t i2c_id, uint32_t baudrate, bool master);
void i2c_set_baud_mode_master(uint8_t i2c_id, uint32_t baudrate, bool master);
void i2c_deinit(uint8_t i2c_id);
void disable_i2c(uint8_t i2c_id);
void enable_i2c(uint8_t i2c_id);

// Transfer functions
void i2c_transfer_blocking(uint8_t i2c_id, uint8_t addr, uint8_t *tx_buf, uint8_t *rx_buf, uint32_t tx_len, uint32_t rx_len);
void i2c_write_stream(uint8_t i2c_id, uint8_t addr, const uint8_t *tx_buf, uint32_t len);
void i2c_write_reg(uint8_t i2c_id, uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len);
void i2c_read_reg(uint8_t i2c_id, uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len);

// Interrupt handler for FIFO full - waits via scheduler
void i2c_fifo_full_handler(uint8_t i2c_id);

#endif // I2C_H
