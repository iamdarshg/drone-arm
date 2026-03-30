#ifndef DRIVERS_DMA_H
#define DRIVERS_DMA_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * RP2350 DMA request (DREQ) source numbers (TRM §2.12, Table 120).
 *
 * RP2350 adds PIO2 (DREQ 16-23) vs RP2040's two-PIO layout, which
 * shifts all peripheral DREQs up by 8 relative to RP2040.
 */
#define DREQ_PIO2_TX0   16u
#define DREQ_PIO2_RX0   20u
#define DREQ_SPI0_TX    24u
#define DREQ_SPI0_RX    25u
#define DREQ_I2C0_TX    44u
#define DREQ_I2C0_RX    45u
#define DREQ_PERMANENT  63u  /* no throttling – memory-to-memory */

/* Transfer size selectors for dma_channel_start(). */
#define DMA_SIZE_BYTE   0u
#define DMA_SIZE_HWORD  1u
#define DMA_SIZE_WORD   2u

/* --- blocking memory-to-memory helpers ----------------------------------- */

/* Byte-granularity copy – any alignment, any size. */
bool dma_memcpy(uint32_t channel, void *dst, const void *src, size_t bytes);

/*
 * Word-granularity copy (4× throughput).
 * src and dst must be 4-byte aligned; 'words' = count in 32-bit words.
 */
bool dma_memcpy32(uint32_t channel, void *dst, const void *src, size_t words);

/* --- peripheral DMA primitives ------------------------------------------- */

/*
 * Configure and trigger a DMA channel without blocking.
 *
 *   src_addr / dst_addr – source/destination (use the peripheral FIFO
 *                         register address for peripheral transfers).
 *   count               – number of transfers (units = data_size).
 *   data_size           – DMA_SIZE_BYTE, DMA_SIZE_HWORD, or DMA_SIZE_WORD.
 *   incr_read           – increment src address after each transfer.
 *   incr_write          – increment dst address after each transfer.
 *   dreq                – DREQ_* constant; use DREQ_PERMANENT for mem→mem.
 */
void dma_channel_start(uint32_t channel,
                       uintptr_t src_addr, uintptr_t dst_addr,
                       size_t count,
                       uint32_t data_size,
                       bool incr_read, bool incr_write,
                       uint32_t dreq);

/*
 * Block until the given channel is no longer busy.
 * Returns false if the channel does not complete within the timeout.
 */
bool dma_channel_wait(uint32_t channel);

#endif
