#include "kernel/assert.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "drivers/i2c.h"
#include "hal/platform.h"

enum {
    I2C_CON = 0x00u,
    I2C_TAR = 0x04u,
    I2C_DATA_CMD = 0x10u,
    I2C_SS_SCL_HCNT = 0x14u,
    I2C_SS_SCL_LCNT = 0x18u,
    I2C_ENABLE = 0x6Cu,
    I2C_STATUS = 0x70u,
    I2C_TXFLR = 0x74u,
    I2C_RXFLR = 0x78u,
    I2C_ENABLE_STATUS = 0x9Cu,
    I2C_CON_MASTER = 1u << 0,
    I2C_CON_SPEED_STD = 1u << 1,
    I2C_CON_RESTART_EN = 1u << 5,
    I2C_CON_SLAVE_DISABLE = 1u << 6,
    I2C_DATA_CMD_READ = 1u << 8,
    I2C_DATA_CMD_STOP = 1u << 9,
    I2C_STATUS_TFNF = 1u << 1,
    I2C_STATUS_RFNE = 1u << 3,
    I2C_TIMEOUT = 200000u,
};

static uint32_t i2c_base(void) {
    return I2C0_BASE;
}

static bool wait_status(uint32_t mask, bool set) {
    uint32_t i;
    uint32_t base;
    base = i2c_base();
    for (i = 0u; i < I2C_TIMEOUT; ++i) {
        uint32_t value;
        value = REG_RO(base + I2C_STATUS);
        if (set) {
            if ((value & mask) != 0u) {
                return true;
            }
        } else if ((value & mask) == 0u) {
            return true;
        }
    }
    ASSERT(i < I2C_TIMEOUT);
    return false;
}

void i2c_init(uint32_t bitrate_hz) {
    uint32_t base;
    uint32_t i;
    ASSERT(bitrate_hz > 0u);
    base = i2c_base();
    REG_RW(base + I2C_ENABLE) = 0u;
    for (i = 0u; i < I2C_TIMEOUT; ++i) {
        if (REG_RO(base + I2C_ENABLE_STATUS) == 0u) {
            break;
        }
    }
    ASSERT(i < I2C_TIMEOUT);
    REG_RW(base + I2C_CON) = I2C_CON_MASTER | I2C_CON_SPEED_STD | I2C_CON_RESTART_EN | I2C_CON_SLAVE_DISABLE;
    REG_RW(base + I2C_SS_SCL_HCNT) = 600u;
    REG_RW(base + I2C_SS_SCL_LCNT) = 700u;
    REG_RW(base + I2C_ENABLE) = 1u;
}

bool i2c_write(uint8_t addr, const uint8_t *data, size_t len) {
    size_t i;
    uint32_t base;
    ASSERT(data != 0 || len == 0u);
    base = i2c_base();
    REG_RW(base + I2C_TAR) = addr;
    for (i = 0u; i < len; ++i) {
        uint32_t cmd;
        if (!wait_status(I2C_STATUS_TFNF, true)) {
            return false;
        }
        cmd = data[i];
        if (i + 1u == len) {
            cmd |= I2C_DATA_CMD_STOP;
        }
        REG_RW(base + I2C_DATA_CMD) = cmd;
    }
    return true;
}

bool i2c_read(uint8_t addr, uint8_t *data, size_t len) {
    size_t i;
    uint32_t base;
    ASSERT(data != 0 || len == 0u);
    base = i2c_base();
    REG_RW(base + I2C_TAR) = addr;
    for (i = 0u; i < len; ++i) {
        uint32_t cmd;
        if (!wait_status(I2C_STATUS_TFNF, true)) {
            return false;
        }
        cmd = I2C_DATA_CMD_READ;
        if (i + 1u == len) {
            cmd |= I2C_DATA_CMD_STOP;
        }
        REG_RW(base + I2C_DATA_CMD) = cmd;
        if (!wait_status(I2C_STATUS_RFNE, true)) {
            return false;
        }
        data[i] = (uint8_t)REG_RO(base + I2C_DATA_CMD);
    }
    return true;
}
