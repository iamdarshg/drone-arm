/**
 * I2C Driver for Main Controller (RP2350B)
 * P10 Compliant - Fixed memory management and control flow
 */

#include "i2c.h"
#include "../../../include/hardware/structs/i2c.h"
#include "../../../include/hardware/regs/addressmap.h"
#include "../../common/assert.h"
#include "../../common/errors.h"
#include "../hardware_defs/pins.h"
#include "gpio.h"
#include "scheduler.h"

// ============================================================================
// CONFIGURATION
// ============================================================================

// Maximum transfer size for pre-allocated buffers
#define I2C_MAX_TRANSFER_SIZE 32

// Timeout for I2C operations (in iterations)
#define I2C_TIMEOUT_ITERATIONS 100000

// ============================================================================
// STATIC STATE (Pre-allocated at init time - no dynamic memory)
// ============================================================================

static uint8_t i2c_tx_buffer[I2C_MAX_TRANSFER_SIZE];
static uint8_t i2c_rx_buffer[I2C_MAX_TRANSFER_SIZE];
static bool i2c_initialized = false;

// ============================================================================
// HARDWARE ACCESS
// ============================================================================

i2c_hw_t* i2c_get_hw(uint8_t id) {
    PRECONDITION(id < I2C_NUM_INTERFACES);
    return (i2c_hw_t*)(I2C0_BASE + (id * 0x1000));
}

// ============================================================================
// GPIO INITIALIZATION
// ============================================================================

static void i2c_gpio_init(uint8_t i2c_id, uint8_t scl, uint8_t sda) {
    PRECONDITION(i2c_id < I2C_NUM_INTERFACES);
    ASSERT_RANGE(scl, 0, 47);
    ASSERT_RANGE(sda, 0, 47);
    
    global_pin_func_map[scl] = GPIO_FUNC_I2C;
    global_pin_func_map[sda] = GPIO_FUNC_I2C;
    global_pin_direction[scl] = true;
    global_pin_direction[sda] = true;
    
    gpio_init(scl);
    gpio_init(sda);
    gpio_set_pull(scl, GPIO_PULL_UP);
    gpio_set_pull(sda, GPIO_PULL_UP);
}

// ============================================================================
// INITIALIZATION
// ============================================================================

void i2c_init(uint8_t i2c_id, uint32_t baudrate, bool master) {
    PRECONDITION(i2c_id < I2C_NUM_INTERFACES);
    ASSERT(baudrate > 0);
    
    i2c_initialized = true;
    
    disable_i2c(i2c_id);
    
    if (i2c_id == I2C_ID_0) {
        i2c_gpio_init(i2c_id, I2C0_SCL_PIN, I2C0_SDA_PIN);
    } else {
        i2c_gpio_init(i2c_id, I2C1_SCL_PIN, I2C1_SDA_PIN);
    }
    
    bool result = i2c_set_baud_mode_master(i2c_id, baudrate, master);
    ASSERT_MSG(result, "I2C baud rate configuration failed");
    
    enable_i2c(i2c_id);
}

// Helper function to calculate I2C timing parameters
static bool calculate_i2c_params(uint32_t baudrate, uint32_t* hcnt, uint32_t* lcnt) {
    PRECONDITION(baudrate > 0);
    ASSERT_NOT_NULL(hcnt);
    ASSERT_NOT_NULL(lcnt);
    
    uint32_t clk_sys = I2C_CLK_SYS_FREQ;
    uint32_t total_cycles = clk_sys / baudrate;
    
    // Standard ratio: HCNT = 40%, LCNT = 60%
    *hcnt = (total_cycles * 40) / 100;
    *lcnt = (total_cycles * 60) / 100;
    
    // Clamp to valid ranges
    if (*hcnt > 65535) *hcnt = 65535;
    if (*lcnt > 65535) *lcnt = 65535;
    if (*hcnt < 8) *hcnt = 8;
    if (*lcnt < 8) *lcnt = 8;
    
    return true;
}

bool i2c_set_baud_mode_master(uint8_t i2c_id, uint32_t baudrate, bool master) {
    PRECONDITION(i2c_id < I2C_NUM_INTERFACES);
    ASSERT(baudrate > 0);
    
    uint32_t hcnt;
    uint32_t lcnt;
    
    if (!calculate_i2c_params(baudrate, &hcnt, &lcnt)) {
        return false;
    }
    
    bool was_enabled = (i2c_get_hw(i2c_id)->enable & I2C_ENABLE_BIT) != 0;
    if (was_enabled) {
        disable_i2c(i2c_id);
    }
    
    // Set timing registers
    i2c_get_hw(i2c_id)->fs_scl_hcnt = hcnt;
    i2c_get_hw(i2c_id)->fs_scl_lcnt = lcnt;
    
    // Configure control register
    uint32_t con = 0;
    if (master) {
        con |= I2C_CON_MASTER_MODE_BIT;
        con |= I2C_CON_SPEED_FAST;
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
        enable_i2c(i2c_id);
    }
    
    return true;
}

void i2c_deinit(uint8_t i2c_id) {
    PRECONDITION(i2c_id < I2C_NUM_INTERFACES);
    disable_i2c(i2c_id);
}

void disable_i2c(uint8_t i2c_id) {
    PRECONDITION(i2c_id < I2C_NUM_INTERFACES);
    i2c_get_hw(i2c_id)->enable &= ~I2C_ENABLE_BIT;
}

void enable_i2c(uint8_t i2c_id) {
    PRECONDITION(i2c_id < I2C_NUM_INTERFACES);
    i2c_get_hw(i2c_id)->enable |= I2C_ENABLE_BIT;
}

// ============================================================================
// TRANSFER FUNCTIONS
// ============================================================================

void i2c_transfer_blocking(uint8_t i2c_id, uint8_t addr, uint8_t *tx_buf, uint8_t *rx_buf, uint32_t tx_len, uint32_t rx_len) {
    PRECONDITION(i2c_id < I2C_NUM_INTERFACES);
    ASSERT_NOT_NULL(tx_buf);
    ASSERT(rx_len == 0 || rx_buf != NULL);
    ASSERT(tx_len <= I2C_MAX_TRANSFER_SIZE);
    ASSERT(rx_len <= I2C_MAX_TRANSFER_SIZE);
    ASSERT(i2c_initialized);
    
    enable_i2c(i2c_id);
    
    // Set target address
    i2c_get_hw(i2c_id)->tar = addr & 0x3FF;
    
    // Write phase
    for (uint32_t idx = 0; idx < tx_len; idx++) {
        ASSERT_TERMINATION(idx, I2C_MAX_TRANSFER_SIZE);
        
        uint32_t timeout = I2C_TIMEOUT_ITERATIONS;
        while ((i2c_get_hw(i2c_id)->status & I2C_STATUS_TFNF_BIT) == 0) {
            ASSERT_TERMINATION(timeout--, I2C_TIMEOUT_ITERATIONS + 1);
            if (timeout == 0) {
                log_error("I2C TX timeout", 2, "i2c_transfer_blocking");
                disable_i2c(i2c_id);
                return;
            }
        }
        
        uint32_t cmd = tx_buf[idx] & I2C_DATA_CMD_DAT_MASK;
        if (rx_len > 0 || idx < tx_len - 1) {
            cmd &= ~I2C_DATA_CMD_STOP_BIT;
        } else {
            cmd |= I2C_DATA_CMD_STOP_BIT;
        }
        i2c_get_hw(i2c_id)->data_cmd = cmd;
    }
    
    // Read phase
    for (uint32_t idx = 0; idx < rx_len; idx++) {
        ASSERT_TERMINATION(idx, I2C_MAX_TRANSFER_SIZE);
        
        uint32_t timeout = I2C_TIMEOUT_ITERATIONS;
        while ((i2c_get_hw(i2c_id)->status & I2C_STATUS_TFNF_BIT) == 0) {
            ASSERT_TERMINATION(timeout--, I2C_TIMEOUT_ITERATIONS + 1);
            if (timeout == 0) {
                log_error("I2C TX timeout", 2, "i2c_transfer_blocking");
                disable_i2c(i2c_id);
                return;
            }
        }
        
        uint32_t cmd = I2C_DATA_CMD_READ;
        if (idx == rx_len - 1) {
            cmd |= I2C_DATA_CMD_STOP_BIT;
        }
        i2c_get_hw(i2c_id)->data_cmd = cmd;
        
        // Wait for data
        timeout = I2C_TIMEOUT_ITERATIONS;
        while ((i2c_get_hw(i2c_id)->status & I2C_STATUS_RFNE_BIT) == 0) {
            ASSERT_TERMINATION(timeout--, I2C_TIMEOUT_ITERATIONS + 1);
            if (timeout == 0) {
                log_error("I2C RX timeout", 2, "i2c_transfer_blocking");
                disable_i2c(i2c_id);
                return;
            }
        }
        rx_buf[idx] = (uint8_t)(i2c_get_hw(i2c_id)->data_cmd & I2C_DATA_CMD_DAT_MASK);
    }
    
    disable_i2c(i2c_id);
}

void i2c_write_stream(uint8_t i2c_id, uint8_t addr, const uint8_t *tx_buf, uint32_t len) {
    PRECONDITION(i2c_id < I2C_NUM_INTERFACES);
    ASSERT_NOT_NULL(tx_buf);
    ASSERT(len > 0);
    ASSERT(len <= I2C_MAX_TRANSFER_SIZE);
    ASSERT(i2c_initialized);
    
    enable_i2c(i2c_id);
    
    i2c_get_hw(i2c_id)->tar = addr & 0x3FF;
    
    for (uint32_t idx = 0; idx < len; idx++) {
        ASSERT_TERMINATION(idx, I2C_MAX_TRANSFER_SIZE);
        
        uint32_t timeout = I2C_TIMEOUT_ITERATIONS;
        while ((i2c_get_hw(i2c_id)->status & I2C_STATUS_TFNF_BIT) == 0) {
            ASSERT_TERMINATION(timeout--, I2C_TIMEOUT_ITERATIONS + 1);
            if (timeout == 0) {
                log_error("I2C TX timeout", 2, "i2c_write_stream");
                disable_i2c(i2c_id);
                return;
            }
        }
        
        uint32_t cmd = tx_buf[idx] & I2C_DATA_CMD_DAT_MASK;
        if (idx == len - 1) {
            cmd |= I2C_DATA_CMD_STOP_BIT;
        }
        i2c_get_hw(i2c_id)->data_cmd = cmd;
    }
    
    // Wait for transfer complete
    uint32_t timeout = I2C_TIMEOUT_ITERATIONS;
    while ((i2c_get_hw(i2c_id)->status & I2C_STATUS_MST_ACTIVITY_BIT) != 0) {
        ASSERT_TERMINATION(timeout--, I2C_TIMEOUT_ITERATIONS + 1);
        if (timeout == 0) {
            break;
        }
    }
    
    disable_i2c(i2c_id);
}

void i2c_write_reg(uint8_t i2c_id, uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len) {
    PRECONDITION(i2c_id < I2C_NUM_INTERFACES);
    ASSERT_NOT_NULL(data);
    ASSERT(len > 0);
    ASSERT(len + 1 <= I2C_MAX_TRANSFER_SIZE);
    ASSERT(i2c_initialized);
    
    // Use pre-allocated buffer (Rule 3: no dynamic memory)
    i2c_tx_buffer[0] = reg;
    
    for (uint32_t idx = 0; idx < len; idx++) {
        ASSERT_TERMINATION(idx, I2C_MAX_TRANSFER_SIZE);
        i2c_tx_buffer[idx + 1] = data[idx];
    }
    
    i2c_write_stream(i2c_id, addr, i2c_tx_buffer, len + 1);
}

void i2c_read_reg(uint8_t i2c_id, uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len) {
    PRECONDITION(i2c_id < I2C_NUM_INTERFACES);
    ASSERT_NOT_NULL(data);
    ASSERT(len > 0);
    ASSERT(len <= I2C_MAX_TRANSFER_SIZE);
    ASSERT(i2c_initialized);
    
    i2c_transfer_blocking(i2c_id, addr, &reg, data, 1, len);
}

// ============================================================================
// INTERRUPT HANDLER
// ============================================================================

void i2c_fifo_full_handler(uint8_t i2c_id) {
    PRECONDITION(i2c_id < I2C_NUM_INTERFACES);
    log_error("I2C FIFO full", 1, "i2c_fifo_full_handler");
    disable_i2c(i2c_id);
    enable_i2c(i2c_id);
}
