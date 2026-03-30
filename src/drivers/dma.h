#ifndef DRIVERS_DMA_H
#define DRIVERS_DMA_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Byte-granularity copy – any alignment, any size. */
bool dma_memcpy(uint32_t channel, void *dst, const void *src, size_t bytes);

/*
 * Word-granularity copy (4× throughput).
 * src and dst must be 4-byte aligned; 'words' = transfer count in 32-bit words.
 */
bool dma_memcpy32(uint32_t channel, void *dst, const void *src, size_t words);

#endif
