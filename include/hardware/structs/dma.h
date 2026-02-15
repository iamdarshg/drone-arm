/*
 * DMA hardware structs
 */
#ifndef _HARDWARE_STRUCTS_DMA_H
#define _HARDWARE_STRUCTS_DMA_H

#include "../address_mapped.h"
#include "../regs/addressmap.h"

// DMA channel registers
typedef struct {
    io_rw_32 read_addr;
    io_rw_32 write_addr;
    io_rw_32 transfer_count;
    io_rw_32 ctrl_trig;
    io_rw_32 al1_ctrl;
    io_rw_32 al1_read_addr;
    io_rw_32 al1_write_addr;
    io_rw_32 al1_transfer_count_trig;
    io_rw_32 al2_ctrl;
    io_rw_32 al2_transfer_count;
    io_rw_32 al2_read_addr;
    io_rw_32 al2_write_addr_trig;
    io_rw_32 al3_ctrl;
    io_rw_32 al3_write_addr;
    io_rw_32 al3_transfer_count;
    io_rw_32 al3_read_addr_trig;
} dma_channel_t;

// DMA hardware register block
typedef struct {
    dma_channel_t ch[12];  // 12 channels
    uint32_t _pad0[16];
    io_rw_32 intr;
    io_rw_32 inte0;
    io_rw_32 intf0;
    io_ro_32 ints0;
    io_rw_32 inte1;
    io_rw_32 intf1;
    io_ro_32 ints1;
    uint32_t _pad1;
    io_rw_32 timer[4];
    io_rw_32 multi_chan_trigger;
    io_rw_32 sniff_ctrl;
    io_rw_32 sniff_data;
    uint32_t _pad2;
    io_ro_32 fifo_levels;
    io_rw_32 chan_abort;
    uint32_t _pad3;
    io_ro_32 n_channels;
} dma_hw_t;

#define dma_hw ((dma_hw_t *const)DMA_BASE)

// CTRL_TRIG bits
#define DMA_CTRL_TRIG_EN_BIT            (1u << 0)
#define DMA_CTRL_TRIG_HIGH_PRIORITY_BIT (1u << 1)
#define DMA_CTRL_TRIG_DATA_SIZE_LSB     2
#define DMA_CTRL_TRIG_DATA_SIZE_BITS    0x0000000c
#define DMA_CTRL_TRIG_INCR_READ_BIT     (1u << 4)
#define DMA_CTRL_TRIG_INCR_WRITE_BIT    (1u << 5)
#define DMA_CTRL_TRIG_RING_SIZE_LSB     6
#define DMA_CTRL_TRIG_RING_SIZE_BITS    0x000007c0
#define DMA_CTRL_TRIG_RING_SEL_BIT      (1u << 10)
#define DMA_CTRL_TRIG_CHAIN_TO_LSB      11
#define DMA_CTRL_TRIG_CHAIN_TO_BITS     0x00007800
#define DMA_CTRL_TRIG_TREQ_SEL_LSB      15
#define DMA_CTRL_TRIG_TREQ_SEL_BITS     0x001f8000
#define DMA_CTRL_TRIG_IRQ_QUIET_BIT     (1u << 21)
#define DMA_CTRL_TRIG_BSWAP_BIT         (1u << 22)
#define DMA_CTRL_TRIG_SNIFF_EN_BIT      (1u << 23)
#define DMA_CTRL_TRIG_BUSY_BIT          (1u << 24)

// Data sizes
#define DMA_SIZE_BYTE       0
#define DMA_SIZE_HALFWORD   1
#define DMA_SIZE_WORD       2

#endif // _HARDWARE_STRUCTS_DMA_H
