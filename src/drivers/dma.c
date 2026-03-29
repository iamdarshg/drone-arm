#include "kernel/assert.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "drivers/dma.h"
#include "hal/platform.h"

enum {
    DMA_CHANNEL_STRIDE = 0x40u,
    DMA_READ_ADDR = 0x00u,
    DMA_WRITE_ADDR = 0x04u,
    DMA_TRANS_COUNT = 0x08u,
    DMA_CTRL_TRIG = 0x0Cu,
    DMA_CTRL_EN = 1u << 0,
    DMA_CTRL_DATA_SIZE_8 = 0u << 2,
    DMA_CTRL_INCR_READ = 1u << 4,
    DMA_CTRL_INCR_WRITE = 1u << 5,
    DMA_CTRL_BUSY = 1u << 24,
    DMA_MAX_CHANNELS = 16u,
    DMA_TIMEOUT = 200000u,
};

static uint32_t dma_ch_base(uint32_t channel) {
    ASSERT(channel < DMA_MAX_CHANNELS);
    return DMA_BASE + (channel * DMA_CHANNEL_STRIDE);
}

bool dma_memcpy(uint32_t channel, void *dst, const void *src, size_t bytes) {
    uint32_t base;
    uint32_t i;
    ASSERT(dst != 0);
    ASSERT(src != 0);
    ASSERT(bytes > 0u);
    base = dma_ch_base(channel);
    REG_RW(base + DMA_READ_ADDR) = (uint32_t)(uintptr_t)src;
    REG_RW(base + DMA_WRITE_ADDR) = (uint32_t)(uintptr_t)dst;
    REG_RW(base + DMA_TRANS_COUNT) = (uint32_t)bytes;
    REG_RW(base + DMA_CTRL_TRIG) = DMA_CTRL_EN | DMA_CTRL_DATA_SIZE_8 | DMA_CTRL_INCR_READ | DMA_CTRL_INCR_WRITE;
    for (i = 0u; i < DMA_TIMEOUT; ++i) {
        if ((REG_RO(base + DMA_CTRL_TRIG) & DMA_CTRL_BUSY) == 0u) {
            return true;
        }
    }
    ASSERT(i < DMA_TIMEOUT);
    return false;
}
