#ifndef DRIVERS_DMA_H
#define DRIVERS_DMA_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

bool dma_memcpy(uint32_t channel, void *dst, const void *src, size_t bytes);

#endif
