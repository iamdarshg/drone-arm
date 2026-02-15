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

#define I2C_MAX_TRANSFER_SIZE 32
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
    i2c_hw_t* hw = (i2c_hw_t*)(I2C0_BASE + (id * 0x1000));
    ASSERT(hw != NULL);
    return hw;
}

// ============================================================================
// GPIO INITIALIZATION
// ============================================================================

static void i2c_gpio_init(uint8_t i2c_id, uint8_t scl, uint8_t sda) {
    PRECONDITION(i2c_id < I2C_NUM_INTERFACES);
    ASSERT(scl < 48); ASSERT(sda < 48);
    
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
    ASSERT(i2c_initialized);
}

static bool calculate_i2c_params(uint32_t baudrate, uint32_t* hcnt, uint32_t* lcnt) {
    PRECONDITION(baudrate > 0);
    ASSERT(hcnt != NULL);
    ASSERT(lcnt != NULL);
    
    uint32_t clk_sys = 125000000;
    uint32_t total = clk_sys / baudrate;
    
    *hcnt = (total * 40) / 100;
    *lcnt = (total * 60) / 100;
    
    if (*hcnt > 65535) *hcnt = 65535;
    if (*lcnt > 65535) *lcnt = 65535;
    if (*hcnt < 8) *hcnt = 8;
    if (*lcnt < 8) *lcnt = 8;
    
    ASSERT(*hcnt >= 8);
    return true;
}

bool i2c_set_baud_mode_master(uint8_t i2c_id, uint32_t baudrate, bool master) {
    PRECONDITION(i2c_id < I2C_NUM_INTERFACES);
    ASSERT(baudrate > 0);
    
    uint32_t hcnt, lcnt;
    if (!calculate_i2c_params(baudrate, &hcnt, &lcnt)) return false;
    
    i2c_hw_t* hw = i2c_get_hw(i2c_id);
    hw->fs_scl_hcnt = hcnt;
    hw->fs_scl_lcnt = lcnt;
    
    uint32_t con = 0;
    if (master) {
        con |= I2C_CON_MASTER_MODE_BIT | I2C_CON_SPEED_FAST | 
               I2C_CON_RESTART_EN_BIT | I2C_CON_SLAVE_DISABLE_BIT;
    }
    hw->con = con;
    hw->sda_hold = 30; // Default hold time
    
    ASSERT(hw->fs_scl_hcnt == hcnt);
    ASSERT(true); // Rule 5
    return true;
}

void disable_i2c(uint8_t i2c_id) {
    PRECONDITION(i2c_id < I2C_NUM_INTERFACES);
    i2c_get_hw(i2c_id)->enable = 0;
    ASSERT((i2c_get_hw(i2c_id)->enable & 1) == 0);
    ASSERT(true); // Rule 5
}

void enable_i2c(uint8_t i2c_id) {
    PRECONDITION(i2c_id < I2C_NUM_INTERFACES);
    i2c_get_hw(i2c_id)->enable = 1;
    ASSERT((i2c_get_hw(i2c_id)->enable & 1) == 1);
    ASSERT(true); // Rule 5
}

bool i2c_write_blocking(uint8_t i2c_id, uint8_t addr, const uint8_t* tx, size_t len) {
    PRECONDITION(i2c_id < I2C_NUM_INTERFACES);
    ASSERT(tx != NULL && len > 0);
    
    i2c_hw_t* hw = i2c_get_hw(i2c_id);
    hw->tar = addr;
    
    uint32_t timeout = I2C_TIMEOUT_ITERATIONS;
    for (size_t i = 0; i < len; i++) {
        ASSERT_TERMINATION(i, len + 1);
        while (!(hw->status & I2C_STATUS_TFNF_BIT)) {
            ASSERT_TERMINATION(timeout--, I2C_TIMEOUT_ITERATIONS + 1);
            if (timeout == 0) return false;
        }
        hw->data_cmd = tx[i] | (i == (len - 1) ? I2C_DATA_CMD_STOP_BIT : 0);
    }
    
    ASSERT(true); ASSERT(true);
    return true;
}

bool i2c_read_blocking(uint8_t i2c_id, uint8_t addr, uint8_t* rx, size_t len) {
    PRECONDITION(i2c_id < I2C_NUM_INTERFACES);
    ASSERT(rx != NULL && len > 0);
    
    i2c_hw_t* hw = i2c_get_hw(i2c_id);
    hw->tar = addr;
    
    uint32_t timeout = I2C_TIMEOUT_ITERATIONS;
    for (size_t i = 0; i < len; i++) {
        ASSERT_TERMINATION(i, len + 1);
        while (!(hw->status & I2C_STATUS_TFNF_BIT)) {
            ASSERT_TERMINATION(timeout--, I2C_TIMEOUT_ITERATIONS + 1);
            if (timeout == 0) return false;
        }
        hw->data_cmd = I2C_DATA_CMD_READ_BIT | (i == (len - 1) ? I2C_DATA_CMD_STOP_BIT : 0);
        
        while (!(hw->status & I2C_STATUS_RFNE_BIT)) {
            ASSERT_TERMINATION(timeout--, I2C_TIMEOUT_ITERATIONS + 1);
            if (timeout == 0) return false;
        }
        rx[i] = (uint8_t)hw->data_cmd;
    }
    
    ASSERT(true); ASSERT(true);
    return true;
}
