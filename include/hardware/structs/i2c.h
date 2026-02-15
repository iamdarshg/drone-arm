/*
 * I2C hardware structs
 */
#ifndef _HARDWARE_STRUCTS_I2C_H
#define _HARDWARE_STRUCTS_I2C_H

#include "../address_mapped.h"
#include "../regs/addressmap.h"

// I2C register offsets
#define I2C_IC_CON_OFFSET           0x00
#define I2C_IC_TAR_OFFSET           0x04
#define I2C_IC_SAR_OFFSET           0x08
#define I2C_IC_DATA_CMD_OFFSET      0x10
#define I2C_IC_SS_SCL_HCNT_OFFSET   0x14
#define I2C_IC_SS_SCL_LCNT_OFFSET   0x18
#define I2C_IC_FS_SCL_HCNT_OFFSET   0x1c
#define I2C_IC_FS_SCL_LCNT_OFFSET   0x20
#define I2C_IC_INTR_STAT_OFFSET     0x2c
#define I2C_IC_INTR_MASK_OFFSET     0x30
#define I2C_IC_RAW_INTR_STAT_OFFSET 0x34
#define I2C_IC_RX_TL_OFFSET         0x38
#define I2C_IC_TX_TL_OFFSET         0x3c
#define I2C_IC_CLR_INTR_OFFSET      0x40
#define I2C_IC_CLR_RX_UNDER_OFFSET  0x44
#define I2C_IC_CLR_RX_OVER_OFFSET   0x48
#define I2C_IC_CLR_TX_OVER_OFFSET   0x4c
#define I2C_IC_CLR_RD_REQ_OFFSET    0x50
#define I2C_IC_CLR_TX_ABRT_OFFSET   0x54
#define I2C_IC_CLR_RX_DONE_OFFSET   0x58
#define I2C_IC_CLR_ACTIVITY_OFFSET  0x5c
#define I2C_IC_CLR_STOP_DET_OFFSET  0x60
#define I2C_IC_CLR_START_DET_OFFSET 0x64
#define I2C_IC_ENABLE_OFFSET        0x6c
#define I2C_IC_STATUS_OFFSET        0x70
#define I2C_IC_TXFLR_OFFSET         0x74
#define I2C_IC_RXFLR_OFFSET         0x78
#define I2C_IC_SDA_HOLD_OFFSET      0x7c
#define I2C_IC_TX_ABRT_SOURCE_OFFSET 0x80
#define I2C_IC_ENABLE_STATUS_OFFSET 0x9c

// I2C hardware register block
typedef struct {
    io_rw_32 con;
    io_rw_32 tar;
    io_rw_32 sar;
    uint32_t _pad0;
    io_rw_32 data_cmd;
    io_rw_32 ss_scl_hcnt;
    io_rw_32 ss_scl_lcnt;
    io_rw_32 fs_scl_hcnt;
    io_rw_32 fs_scl_lcnt;
    uint32_t _pad1[2];
    io_ro_32 intr_stat;
    io_rw_32 intr_mask;
    io_ro_32 raw_intr_stat;
    io_rw_32 rx_tl;
    io_rw_32 tx_tl;
    io_rw_32 clr_intr;
    io_rw_32 clr_rx_under;
    io_rw_32 clr_rx_over;
    io_rw_32 clr_tx_over;
    io_rw_32 clr_rd_req;
    io_rw_32 clr_tx_abrt;
    io_rw_32 clr_rx_done;
    io_rw_32 clr_activity;
    io_rw_32 clr_stop_det;
    io_rw_32 clr_start_det;
    uint32_t _pad2;
    io_rw_32 enable;
    io_ro_32 status;
    io_ro_32 txflr;
    io_ro_32 rxflr;
    io_rw_32 sda_hold;
    io_ro_32 tx_abrt_source;
    uint32_t _pad3[6];
    io_ro_32 enable_status;
} i2c_hw_t;

#define i2c0_hw ((i2c_hw_t *const)I2C0_BASE)
#define i2c1_hw ((i2c_hw_t *const)I2C1_BASE)

// CON bits
#define I2C_CON_MASTER_MODE_BIT     (1u << 0)
#define I2C_CON_SPEED_STD           (0x1 << 1)
#define I2C_CON_SPEED_FAST          (0x2 << 1)
#define I2C_CON_10BITADDR_MASTER_BIT (1u << 4)
#define I2C_CON_RESTART_EN_BIT      (1u << 5)
#define I2C_CON_SLAVE_DISABLE_BIT   (1u << 6)
#define I2C_CON_STOP_DET_IFADDRESSED_BIT (1u << 7)
#define I2C_CON_TX_EMPTY_CTRL_BIT   (1u << 8)

// DATA_CMD bits
#define I2C_DATA_CMD_DAT_MASK       0xff
#define I2C_DATA_CMD_CMD_BIT        (1u << 8)  // 0=write, 1=read
#define I2C_DATA_CMD_STOP_BIT       (1u << 9)
#define I2C_DATA_CMD_RESTART_BIT    (1u << 10)

// STATUS bits
#define I2C_STATUS_ACTIVITY_BIT     (1u << 0)
#define I2C_STATUS_TFNF_BIT         (1u << 1)  // TX FIFO not full
#define I2C_STATUS_TFE_BIT          (1u << 2)  // TX FIFO empty
#define I2C_STATUS_RFNE_BIT         (1u << 3)  // RX FIFO not empty
#define I2C_STATUS_RFF_BIT          (1u << 4)  // RX FIFO full
#define I2C_STATUS_MST_ACTIVITY_BIT (1u << 5)
#define I2C_STATUS_SLAVE_ACTIVITY_BIT (1u << 6)

// ENABLE bits
#define I2C_ENABLE_ENABLE_BIT       (1u << 0)
#define I2C_ENABLE_ABORT_BIT        (1u << 1)

#endif // _HARDWARE_STRUCTS_I2C_H
