#include "kernel/assert.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "drivers/clock.h"
#include "drivers/dma.h"
#include "drivers/i2c.h"
#include "hal/platform.h"

/*
 * DW_apb_i2c register offsets (RP2350 TRM §4.3).
 */
enum {
    I2C_CON          = 0x00u,
    I2C_TAR          = 0x04u,
    I2C_DATA_CMD     = 0x10u,
    I2C_SS_SCL_HCNT  = 0x14u,  /* Standard mode SCL high period */
    I2C_SS_SCL_LCNT  = 0x18u,  /* Standard mode SCL low  period */
    I2C_FS_SCL_HCNT  = 0x1Cu,  /* Fast    mode SCL high period  */
    I2C_FS_SCL_LCNT  = 0x20u,  /* Fast    mode SCL low  period  */
    I2C_INTR_MASK    = 0x30u,
    I2C_CLR_INTR     = 0x40u,
    I2C_SDA_HOLD     = 0x7Cu,  /* [23:16]=RX hold, [15:0]=TX hold (cycles) */
    I2C_ENABLE       = 0x6Cu,
    I2C_STATUS       = 0x70u,
    I2C_TXFLR        = 0x74u,
    I2C_RXFLR        = 0x78u,
    I2C_ENABLE_STATUS = 0x9Cu,
    /* IC_CON bits */
    I2C_CON_MASTER        = (1u << 0),
    I2C_CON_SPEED_STD     = (1u << 1),   /* bits[2:1] = 01 → standard */
    I2C_CON_SPEED_FAST    = (1u << 2),   /* bits[2:1] = 10 → fast     */
    I2C_CON_RESTART_EN    = (1u << 5),
    I2C_CON_SLAVE_DISABLE = (1u << 6),
    /* IC_DATA_CMD bits */
    I2C_DATA_CMD_READ     = (1u << 8),
    I2C_DATA_CMD_STOP     = (1u << 9),
    /* IC_STATUS bits */
    I2C_STATUS_TFNF       = (1u << 1),  /* TX FIFO not full  */
    I2C_STATUS_RFNE       = (1u << 3),  /* RX FIFO not empty */
    I2C_TIMEOUT = 200000u,
    /* Standard-mode I2C bitrate threshold */
    I2C_FAST_THRESHOLD_HZ = 200000u,
    /* SDA TX hold: 300 ns minimum (standard), 100 ns (fast) */
    I2C_SDA_HOLD_STD_NS  = 300u,
    I2C_SDA_HOLD_FAST_NS = 100u,
};

static uint32_t i2c_base(void) { return I2C0_BASE; }

/*
 * Compute IC_SCL_xCNT values from peri_clk and target bitrate.
 *
 * RP2040/RP2350 SDK approach (simple 3/5 split):
 *   period = peri_clk / bitrate
 *   lcnt   = period × 3/5     (60 % low)
 *   hcnt   = period − lcnt    (40 % high)
 * Both values are clamped to a minimum of 8 (DW_apb_i2c hard limit).
 */
void i2c_calc_scl_counts(uint32_t peri_clk, uint32_t bitrate,
                         uint32_t *hcnt, uint32_t *lcnt) {
    uint32_t period;
    uint32_t lc;
    uint32_t hc;
    ASSERT(peri_clk > 0u);
    ASSERT(bitrate  > 0u);
    ASSERT(hcnt != NULL);
    ASSERT(lcnt != NULL);
    period = (peri_clk + bitrate / 2u) / bitrate; /* rounded */
    lc = (period * 3u) / 5u;
    hc = period - lc;
    *lcnt = (lc  > 8u) ? lc  : 8u;
    *hcnt = (hc  > 8u) ? hc  : 8u;
}

static bool wait_status(uint32_t mask, bool set) {
    uint32_t i;
    uint32_t base = i2c_base();
    for (i = 0u; i < I2C_TIMEOUT; ++i) {
        uint32_t value = REG_RO(base + I2C_STATUS);
        if (set  && ((value & mask) != 0u)) { return true; }
        if (!set && ((value & mask) == 0u)) { return true; }
    }
    ASSERT(i < I2C_TIMEOUT);
    return false;
}

void i2c_init(uint32_t bitrate_hz) {
    uint32_t base;
    uint32_t i;
    uint32_t peri_clk;
    uint32_t hcnt;
    uint32_t lcnt;
    uint32_t hold_cycles;
    uint32_t speed_bits;
    ASSERT(bitrate_hz > 0u);

    base     = i2c_base();
    peri_clk = clock_get_hz(CLOCK_ID_PERI);
    if (peri_clk == 0u) { peri_clk = 150000000u; }

    /* Disable controller and wait for it to become inactive. */
    REG_RW(base + I2C_ENABLE) = 0u;
    for (i = 0u; i < I2C_TIMEOUT; ++i) {
        if ((REG_RO(base + I2C_ENABLE_STATUS) & 1u) == 0u) { break; }
    }
    ASSERT(i < I2C_TIMEOUT);

    /* Clear any pending interrupts. */
    (void)REG_RO(base + I2C_CLR_INTR);

    /* Select standard or fast mode depending on bitrate. */
    speed_bits = (bitrate_hz > I2C_FAST_THRESHOLD_HZ) ?
                 I2C_CON_SPEED_FAST : I2C_CON_SPEED_STD;

    REG_RW(base + I2C_CON) = I2C_CON_MASTER | speed_bits |
                              I2C_CON_RESTART_EN | I2C_CON_SLAVE_DISABLE;

    /* SCL timing – computed from peri clock and target bitrate. */
    i2c_calc_scl_counts(peri_clk, bitrate_hz, &hcnt, &lcnt);
    if (bitrate_hz > I2C_FAST_THRESHOLD_HZ) {
        REG_RW(base + I2C_FS_SCL_HCNT) = hcnt;
        REG_RW(base + I2C_FS_SCL_LCNT) = lcnt;
    } else {
        REG_RW(base + I2C_SS_SCL_HCNT) = hcnt;
        REG_RW(base + I2C_SS_SCL_LCNT) = lcnt;
    }

    /* SDA TX hold time – minimum 300 ns (std) or 100 ns (fast). */
    {
        uint32_t hold_ns = (bitrate_hz > I2C_FAST_THRESHOLD_HZ) ?
                           I2C_SDA_HOLD_FAST_NS : I2C_SDA_HOLD_STD_NS;
        hold_cycles = ((peri_clk / 1000000u) * hold_ns) / 1000u;
        if (hold_cycles < 1u) { hold_cycles = 1u; }
        REG_RW(base + I2C_SDA_HOLD) = hold_cycles;
    }

    REG_RW(base + I2C_ENABLE) = 1u;
}

bool i2c_write(uint8_t addr, const uint8_t *data, size_t len) {
    size_t   i;
    uint32_t base = i2c_base();
    ASSERT(data != NULL || len == 0u);
    REG_RW(base + I2C_TAR) = addr;
    for (i = 0u; i < len; ++i) {
        uint32_t cmd;
        if (!wait_status(I2C_STATUS_TFNF, true)) { return false; }
        cmd = data[i];
        if (i + 1u == len) { cmd |= I2C_DATA_CMD_STOP; }
        REG_RW(base + I2C_DATA_CMD) = cmd;
    }
    return true;
}

bool i2c_read(uint8_t addr, uint8_t *data, size_t len) {
    size_t   i;
    uint32_t base = i2c_base();
    ASSERT(data != NULL || len == 0u);
    REG_RW(base + I2C_TAR) = addr;
    for (i = 0u; i < len; ++i) {
        uint32_t cmd;
        if (!wait_status(I2C_STATUS_TFNF, true)) { return false; }
        cmd = I2C_DATA_CMD_READ;
        if (i + 1u == len) { cmd |= I2C_DATA_CMD_STOP; }
        REG_RW(base + I2C_DATA_CMD) = cmd;
        if (!wait_status(I2C_STATUS_RFNE, true)) { return false; }
        data[i] = (uint8_t)REG_RO(base + I2C_DATA_CMD);
    }
    return true;
}

size_t i2c_format_write_cmds(const uint8_t *data, size_t len, uint16_t *cmd_buf, size_t cap) {
    size_t i;
    ASSERT((data != NULL) || (len == 0u));
    ASSERT((cmd_buf != NULL) || (cap == 0u));
    if (cap < len) {
        return 0u;
    }
    for (i = 0u; i < len; ++i) {
        uint16_t cmd = data[i];
        if (i + 1u == len) {
            cmd |= (uint16_t)I2C_DATA_CMD_STOP;
        }
        cmd_buf[i] = cmd;
    }
    return len;
}

size_t i2c_format_read_cmds(size_t len, uint16_t *cmd_buf, size_t cap) {
    size_t i;
    ASSERT((cmd_buf != NULL) || (cap == 0u));
    if (cap < len) {
        return 0u;
    }
    for (i = 0u; i < len; ++i) {
        uint16_t cmd = (uint16_t)I2C_DATA_CMD_READ;
        if (i + 1u == len) {
            cmd |= (uint16_t)I2C_DATA_CMD_STOP;
        }
        cmd_buf[i] = cmd;
    }
    return len;
}

bool i2c_write_dma(uint32_t tx_channel, uint8_t addr, const uint8_t *data, size_t len) {
    uint32_t base = i2c_base();
    static uint16_t g_cmds[256];
    size_t cmd_count;
    ASSERT((data != NULL) || (len == 0u));
    if (len == 0u) {
        return true;
    }
    if (len > 256u) {
        return false;
    }
    REG_RW(base + I2C_TAR) = addr;
    cmd_count = i2c_format_write_cmds(data, len, g_cmds, 256u);
    if (cmd_count != len) {
        return false;
    }
    dma_channel_start(tx_channel,
                      (uintptr_t)g_cmds, (uintptr_t)(base + I2C_DATA_CMD),
                      cmd_count, DMA_SIZE_HWORD, true, false, DREQ_I2C0_TX);
    return dma_channel_wait(tx_channel);
}

bool i2c_read_dma(uint32_t tx_channel, uint32_t rx_channel, uint8_t addr, uint8_t *data, size_t len) {
    uint32_t base = i2c_base();
    static uint16_t g_cmds[256];
    size_t cmd_count;
    ASSERT((data != NULL) || (len == 0u));
    if (len == 0u) {
        return true;
    }
    if (len > 256u) {
        return false;
    }
    REG_RW(base + I2C_TAR) = addr;
    cmd_count = i2c_format_read_cmds(len, g_cmds, 256u);
    if (cmd_count != len) {
        return false;
    }
    dma_channel_start(rx_channel,
                      (uintptr_t)(base + I2C_DATA_CMD), (uintptr_t)data,
                      len, DMA_SIZE_BYTE, false, true, DREQ_I2C0_RX);
    dma_channel_start(tx_channel,
                      (uintptr_t)g_cmds, (uintptr_t)(base + I2C_DATA_CMD),
                      cmd_count, DMA_SIZE_HWORD, true, false, DREQ_I2C0_TX);
    if (!dma_channel_wait(tx_channel)) {
        return false;
    }
    return dma_channel_wait(rx_channel);
}
