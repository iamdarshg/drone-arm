/*
 * I2C implementation matching SPI.c format exactly
 * Optimized for RP2350 with async/scheduler support
 */
#include "pico-sdk\src\rp2350\hardware_structs\include\hardware\structs\i2c.h"
#include <stdio.h>
#include "hardware_defs/pins.h"
#include "common/errors.h"
#include "common/utils.h"
#include "gpio.h"
#include "common/scheduler.h"

// ============================================================================
// GPIO Initialization
// ============================================================================

static void i2c_gpio_init(uint8_t i2c_id, uint8_t scl, uint8_t sda) {
    uint32_t io_base = IO_BANK0_BASE;
    uint32_t pads_base = PADS_BANK0_BASE;

    global_pin_func_map[scl] = GPIO_FUNC_I2C;
    global_pin_func_map[sda] = GPIO_FUNC_I2C;

    global_pin_direction[scl] = true; // SCL is output
    global_pin_direction[sda] = true; // SDA is open-drain, but configured as output

    // Configure SCL
    gpio_init(scl);
    
    // Configure SDA
    gpio_init(sda);
    
    // Enable pull-ups on both lines (required for I2C)
    gpio_set_pull(scl, GPIO_PULL_UP);
    gpio_set_pull(sda, GPIO_PULL_UP);
}

inline *i2c_hw_t i2c_get_hw(uint8_t id) {
    return I2C0_BASE & (id <<16) ;
}

// ============================================================================
// I2C Initialization
// ============================================================================

void i2c_init(uint8_t i2c_id, uint32_t baudrate, uint8_t bool master) {
    if (i2c_id >= I2C_NUM_INTERFACES) {
        log_error("Invalid I2C ID", 2, "i2c_init");
        return;
    }
    
    // Disable I2C before configuration
    i2c_disable(i2c_id);
    
    // Configure GPIOs based on I2C ID
    if (i2c_id == I2C_ID_0) {
        i2c_gpio_init(i2c_id, I2C0_SCL_PIN, I2C0_SDA_PIN);
    } else {
        i2c_gpio_init(i2c_id, I2C1_SCL_PIN, I2C1_SDA_PIN);
    }
    
    i2c_set_baud_mode_master(i2c_id, baudrate, master);
    
    // Enable I2C
    i2c_enable(i2c_id);
}


void i2c_set_baud_mode_master(uint8_t i2c_id, uint32_t baudrate, bool master) {
    if (i2c_id >= I2C_NUM_INTERFACES) {
        return false;
    }
    
    // Calculate timing parameters for target baudrate
    uint32_t clk_sys = I2C_CLK_SYS_FREQ;
    
    // For standard mode (100kHz): HCNT + LCNT = clk_sys / baudrate
    // For fast mode (400kHz): HCNT + LCNT = clk_sys / baudrate
    uint32_t total_cycles = clk_sys / baudrate;
    
    // Standard ratio: HCNT = 40%, LCNT = 60%
    uint32_t hcnt = (total_cycles * 40) / 100;
    uint32_t lcnt = (total_cycles * 60) / 100;
    
    // Clamp to valid ranges
    if (hcnt > 65535) hcnt = 65535;
    if (lcnt > 65535) lcnt = 65535;
    if (hcnt < 8) hcnt = 8;
    if (lcnt < 8) lcnt = 8;
    
    // Apply settings
    bool was_enabled = i2c_is_enabled(i2c_id);
    if (was_enabled) {
        i2c_disable(i2c_id);
    }
    
    // Set timing registers
    i2c_get_hw(i2c_id)->fs_scl_hcnt = hcnt;
    i2c_get_hw(i2c_id)->fs_scl_lcnt = lcnt;
    
    // Configure control register for master mode
    uint32_t con = 0;
    if (master) {
        con |= I2C_CON_MASTER_MODE_BIT;
        con |= I2C_CON_SPEED_FAST;  // Fast mode (400kHz)
        con |= I2C_CON_RESTART_EN_BIT;
        con |= I2C_CON_SLAVE_DISABLE_BIT;
    }
    i2c_get_hw(i2c_id)->con = con;
    
    // Set SDA hold time
    i2c_get_hw(i2c_id)->sda_hold = I2C_SDA_TX_HOLD;
    
    // Set FIFO thresholds
    i2c_get_hw(i2c_id)->tx_tl = I2C_TX_TL;
    i2c_get_hw(i2c_id)->rx_tl = I2C_RX_TL;
    
    if (was_enabled) {
        i2c_enable(i2c_id);
    }
}

void i2c_deinit(uint8_t i2c_id) {
    if (i2c_id >= I2C_NUM_INTERFACES) {
        return;
    }
    
    // Disable I2C
    i2c_disable(i2c_id);
    
    // TODO: Reset GPIOs to NULL function
}

void disable_i2c(uint8_t i2c_id){
    i2c_get_hw(id)->enable = !(i2c_get_hw(id)->enable || 1);
}

void enable_i2c(uint8_t i2c_id){
    i2c_get_hw(id)->enable &= 1;
}

// ============================================================================
// Transfer Functions (non-inline wrappers)
// ============================================================================

inline void i2c_transfer_blocking(uint8_t i2c_id, uint8_t addr, uint8_t *tx_buf, uint8_t *rx_buf, uint32_t tx_len, uint32_t rx_len) {
    enable_i2c(i2c_id);
    
    // Set target address
    i2c_get_hw(i2c_id)->tar = addr & 0x3FF;
    
    // Write phase
    for(uint32_t idx = 0; idx < tx_len; idx++){
        sched_wait_until(((i2c_get_hw(i2c_id)->status>>1)<<31)==0);
        
        uint32_t cmd = tx_buf[idx] & I2C_DATA_CMD_DAT_MASK;
        // No stop if we have read phase
        if (rx_len > 0 || idx < tx_len - 1) {
            cmd &= ~I2C_DATA_CMD_STOP_BIT;
        } else {
            cmd |= I2C_DATA_CMD_STOP_BIT;
        }
        i2c_get_hw(i2c_id)->data_cmd = cmd;
    }
    
    // Read phase
    for(uint32_t idx = 0; idx < rx_len; idx++){
        sched_wait_until(((i2c_get_hw(i2c_id)->status>>1)<<31)==0);
        
        uint32_t cmd = I2C_DATA_CMD_READ;
        if (idx == rx_len - 1) {
            cmd |= I2C_DATA_CMD_STOP_BIT;
        }
        i2c_get_hw(i2c_id)->data_cmd = cmd;
        
        // Wait for data
        sched_wait_until(((i2c_get_hw(i2c_id)->status>>3)<<31)==0);
        rx_buf[idx] = (uint8_t)(i2c_get_hw(i2c_id)->data_cmd & I2C_DATA_CMD_DAT_MASK);
    }
    
    disable_i2c(i2c_id);
}


inline void i2c_write_stream(uint8_t i2c_id, uint8_t addr, const uint8_t *tx_buf, uint32_t len) {
    enable_i2c(i2c_id);
    
    // Set target address
    i2c_get_hw(i2c_id)->tar = addr & 0x3FF;
    
    for(uint32_t idx = 0; idx < len; idx++){
        sched_wait_until(((i2c_get_hw(i2c_id)->status>>1)<<31)==0);
        
        uint32_t cmd = tx_buf[idx] & I2C_DATA_CMD_DAT_MASK;
        if (idx == len - 1) {
            cmd |= I2C_DATA_CMD_STOP_BIT;
        }
        i2c_get_hw(i2c_id)->data_cmd = cmd;
    }
}

inline void i2c_write_reg(uint8_t i2c_id, uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len) {
    uint8_t out[len + 1];
    out[0] = reg;
    for(uint32_t idx = 0; idx < len; idx++){
        out[idx + 1] = data[idx];
    }
    i2c_write_stream(i2c_id, addr, out, len + 1);
}

inline void i2c_read_reg(uint8_t i2c_id, uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len) {
    i2c_transfer_blocking(i2c_id, addr, &reg, data, 1, len);
}
