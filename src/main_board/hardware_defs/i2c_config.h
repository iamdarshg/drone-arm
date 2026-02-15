#ifndef I2C_CONFIG_H
#define I2C_CONFIG_H

#include <stdint.h>

// I2C Interface IDs (runtime selectable)
#define I2C_ID_0        0
#define I2C_ID_1        1
#define I2C_NUM_INTERFACES  2

// I2C Clock frequency (CLK_SYS, typically 48MHz or 150MHz)
#define I2C_CLK_SYS_FREQ    150000000   // 150 MHz - adjust to actual clock

// I2C Speed Modes
#define I2C_SPEED_STANDARD  100000      // 100 kHz
#define I2C_SPEED_FAST      400000      // 400 kHz
#define I2C_SPEED_FAST_PLUS 1000000     // 1 MHz

// Default I2C speed
#define I2C_DEFAULT_SPEED   I2C_SPEED_FAST

// I2C Timing calculations (for Fast Mode 400kHz at 150MHz)
// SCL period = (HCNT + LCNT) / CLK_SYS
// Target: 2.5us period for 400kHz
// HCNT + LCNT = 2.5us * 150MHz = 375 cycles
// Standard ratio: HCNT = 40%, LCNT = 60%
// HCNT = 150, LCNT = 225
#define I2C_FS_SCL_HCNT     150
#define I2C_FS_SCL_LCNT     225

// Spike suppression (filter glitches < 50ns)
// At 150MHz: 50ns * 150MHz = 7.5, so SPKLEN = 8
#define I2C_FS_SPKLEN       8

// SDA hold time (minimum 300ns for fast mode)
// At 150MHz: 300ns * 150MHz = 45 cycles
#define I2C_SDA_TX_HOLD     45
#define I2C_SDA_RX_HOLD     0

// TX/RX FIFO thresholds
#define I2C_RX_TL           0           // Trigger when RX FIFO has >=1 byte
#define I2C_TX_TL           8           // Trigger when TX FIFO has <=8 bytes (half empty)

// FIFO depths
#define I2C_FIFO_DEPTH      16

// Status register bits
#define I2C_STATUS_SLV_ACTIVITY     (1U << 6)
#define I2C_STATUS_MST_ACTIVITY     (1U << 5)
#define I2C_STATUS_RFF              (1U << 4)
#define I2C_STATUS_RFNE             (1U << 3)
#define I2C_STATUS_TFE              (1U << 2)
#define I2C_STATUS_TFNF             (1U << 1)
#define I2C_STATUS_ACTIVITY         (1U << 0)

// Interrupt status bits
#define I2C_INTR_RESTART_DET        (1U << 12)
#define I2C_INTR_GEN_CALL           (1U << 11)
#define I2C_INTR_START_DET          (1U << 10)
#define I2C_INTR_STOP_DET           (1U << 9)
#define I2C_INTR_ACTIVITY           (1U << 8)
#define I2C_INTR_RX_DONE            (1U << 7)
#define I2C_INTR_TX_ABRT            (1U << 6)
#define I2C_INTR_RD_REQ             (1U << 5)
#define I2C_INTR_TX_EMPTY           (1U << 4)
#define I2C_INTR_TX_OVER            (1U << 3)
#define I2C_INTR_RX_FULL            (1U << 2)
#define I2C_INTR_RX_OVER            (1U << 1)
#define I2C_INTR_RX_UNDER           (1U << 0)

// Data/Command register bits
#define I2C_DATA_CMD_FIRST_DATA_BYTE    (1U << 11)
#define I2C_DATA_CMD_RESTART            (1U << 10)
#define I2C_DATA_CMD_STOP               (1U << 9)
#define I2C_DATA_CMD_READ               (1U << 8)   // 1 = read, 0 = write

// Timeout for bus operations (microseconds)
#define I2C_TIMEOUT_US      10000       // 10ms timeout

#endif // I2C_CONFIG_H
