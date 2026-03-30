#include "kernel/assert.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "drivers/dma.h"
#include "hal/platform.h"

/*
 * DMA channel register offsets (RP2350 TRM §2.12).
 * Each channel occupies a 0x40-byte block.
 */
enum {
    DMA_CHANNEL_STRIDE   = 0x40u,
    DMA_READ_ADDR        = 0x00u,
    DMA_WRITE_ADDR       = 0x04u,
    DMA_TRANS_COUNT      = 0x08u,
    DMA_CTRL_TRIG        = 0x0Cu,
    /* CTRL_TRIG field positions */
    DMA_CTRL_EN          = (1u << 0),
    DMA_CTRL_INCR_READ   = (1u << 4),
    DMA_CTRL_INCR_WRITE  = (1u << 5),
    /* DATA_SIZE at bits [3:2]: 0=byte, 1=half-word, 2=word */
    DMA_DATA_SIZE_8      = (0u << 2),
    DMA_DATA_SIZE_32     = (2u << 2),
    /* TREQ_SEL at bits [20:15] (6 bits, selects DREQ source) */
    DMA_CTRL_TREQ_SHIFT  = 15u,
    DMA_CTRL_TREQ_MASK   = (0x3Fu << 15u),
    /* BUSY flag */
    DMA_CTRL_BUSY        = (1u << 24),
    DMA_MAX_CHANNELS     = 16u,
    DMA_TIMEOUT          = 200000u,
};

/* Build the TREQ_SEL field value from a DREQ_ constant. */
#define DREQ_FIELD(d)  (((uint32_t)(d) & 0x3Fu) << DMA_CTRL_TREQ_SHIFT)

static uint32_t dma_ch_base(uint32_t channel) {
    ASSERT(channel < DMA_MAX_CHANNELS);
    return DMA_BASE + (channel * DMA_CHANNEL_STRIDE);
}

static bool dma_wait_done(uint32_t base) {
    uint32_t i;
    for (i = 0u; i < DMA_TIMEOUT; ++i) {
        if ((REG_RO(base + DMA_CTRL_TRIG) & DMA_CTRL_BUSY) == 0u) {
            return true;
        }
    }
    ASSERT(i < DMA_TIMEOUT);
    return false;
}

/*
 * dma_memcpy – byte-granularity, blocking, memory-to-memory.
 * Uses DREQ_PERMANENT so the DMA runs at full bus speed.
 */
bool dma_memcpy(uint32_t channel, void *dst, const void *src, size_t bytes) {
    uint32_t base;
    ASSERT(dst   != NULL);
    ASSERT(src   != NULL);
    ASSERT(bytes  > 0u);
    base = dma_ch_base(channel);
    REG_RW(base + DMA_READ_ADDR)   = (uint32_t)(uintptr_t)src;
    REG_RW(base + DMA_WRITE_ADDR)  = (uint32_t)(uintptr_t)dst;
    REG_RW(base + DMA_TRANS_COUNT) = (uint32_t)bytes;
    REG_RW(base + DMA_CTRL_TRIG)   = DMA_CTRL_EN | DMA_DATA_SIZE_8 |
                                     DMA_CTRL_INCR_READ | DMA_CTRL_INCR_WRITE |
                                     DREQ_FIELD(DREQ_PERMANENT);
    return dma_wait_done(base);
}

/*
 * dma_memcpy32 – word-granularity, blocking, memory-to-memory (4× throughput).
 * Both src and dst must be 4-byte aligned; 'words' = count in 32-bit words.
 */
bool dma_memcpy32(uint32_t channel, void *dst, const void *src, size_t words) {
    uint32_t base;
    ASSERT(dst   != NULL);
    ASSERT(src   != NULL);
    ASSERT(words  > 0u);
    ASSERT(((uintptr_t)dst & 3u) == 0u);
    ASSERT(((uintptr_t)src & 3u) == 0u);
    base = dma_ch_base(channel);
    REG_RW(base + DMA_READ_ADDR)   = (uint32_t)(uintptr_t)src;
    REG_RW(base + DMA_WRITE_ADDR)  = (uint32_t)(uintptr_t)dst;
    REG_RW(base + DMA_TRANS_COUNT) = (uint32_t)words;
    REG_RW(base + DMA_CTRL_TRIG)   = DMA_CTRL_EN | DMA_DATA_SIZE_32 |
                                     DMA_CTRL_INCR_READ | DMA_CTRL_INCR_WRITE |
                                     DREQ_FIELD(DREQ_PERMANENT);
    return dma_wait_done(base);
}

/*
 * dma_channel_start – non-blocking generic channel setup.
 *
 * Configures all channel registers and writes CTRL_TRIG to trigger the
 * transfer.  The caller must poll with dma_channel_wait() when needed.
 */
void dma_channel_start(uint32_t channel,
                       uintptr_t src_addr, uintptr_t dst_addr,
                       size_t count,
                       uint32_t data_size,
                       bool incr_read, bool incr_write,
                       uint32_t dreq) {
    uint32_t base;
    uint32_t ctrl;
    ASSERT(channel   < DMA_MAX_CHANNELS);
    ASSERT(count     > 0u);
    ASSERT(data_size <= DMA_SIZE_WORD);
    base = dma_ch_base(channel);
    ctrl = DMA_CTRL_EN |
           ((data_size & 3u) << 2u) |
           (incr_read  ? DMA_CTRL_INCR_READ  : 0u) |
           (incr_write ? DMA_CTRL_INCR_WRITE : 0u) |
           DREQ_FIELD(dreq);
    REG_RW(base + DMA_READ_ADDR)   = (uint32_t)src_addr;
    REG_RW(base + DMA_WRITE_ADDR)  = (uint32_t)dst_addr;
    REG_RW(base + DMA_TRANS_COUNT) = (uint32_t)count;
    REG_RW(base + DMA_CTRL_TRIG)   = ctrl;
}

/* dma_channel_wait – block until channel is idle. */
bool dma_channel_wait(uint32_t channel) {
    return dma_wait_done(dma_ch_base(channel));
}
