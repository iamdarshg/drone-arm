#include "kernel/assert.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "drivers/dma.h"
#include "hal/platform.h"

/*
 * DMA channel register offsets (RP2350 TRM §2.12).
 * Each channel has a 0x40-byte register block.
 */
enum {
    DMA_CHANNEL_STRIDE   = 0x40u,
    DMA_READ_ADDR        = 0x00u,
    DMA_WRITE_ADDR       = 0x04u,
    DMA_TRANS_COUNT      = 0x08u,
    DMA_CTRL_TRIG        = 0x0Cu,
    /* CTRL bits */
    DMA_CTRL_EN          = (1u << 0),
    DMA_CTRL_INCR_READ   = (1u << 4),
    DMA_CTRL_INCR_WRITE  = (1u << 5),
    DMA_CTRL_BUSY        = (1u << 24),
    /* DATA_SIZE field at bits [3:2]: 0=byte, 1=half-word, 2=word */
    DMA_DATA_SIZE_8      = (0u << 2),
    DMA_DATA_SIZE_16     = (1u << 2),
    DMA_DATA_SIZE_32     = (2u << 2),
    DMA_MAX_CHANNELS     = 16u,
    DMA_TIMEOUT          = 200000u,
};

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
 * dma_memcpy – byte-granularity DMA copy (any size, any alignment).
 * TRANS_COUNT is in bytes when DATA_SIZE=byte.
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
                                     DMA_CTRL_INCR_READ | DMA_CTRL_INCR_WRITE;
    return dma_wait_done(base);
}

/*
 * dma_memcpy32 – word-granularity DMA copy (4× throughput).
 *
 * Both src and dst must be 4-byte aligned; 'words' is the count of
 * 32-bit words (total bytes = words × 4).  Ideal for bulk firmware
 * copies and IMU DMA bursts.
 */
bool dma_memcpy32(uint32_t channel, void *dst, const void *src, size_t words) {
    uint32_t base;
    ASSERT(dst   != NULL);
    ASSERT(src   != NULL);
    ASSERT(words  > 0u);
    ASSERT(((uintptr_t)dst & 3u) == 0u); /* must be word-aligned */
    ASSERT(((uintptr_t)src & 3u) == 0u);
    base = dma_ch_base(channel);
    REG_RW(base + DMA_READ_ADDR)   = (uint32_t)(uintptr_t)src;
    REG_RW(base + DMA_WRITE_ADDR)  = (uint32_t)(uintptr_t)dst;
    REG_RW(base + DMA_TRANS_COUNT) = (uint32_t)words;
    REG_RW(base + DMA_CTRL_TRIG)   = DMA_CTRL_EN | DMA_DATA_SIZE_32 |
                                     DMA_CTRL_INCR_READ | DMA_CTRL_INCR_WRITE;
    return dma_wait_done(base);
}
