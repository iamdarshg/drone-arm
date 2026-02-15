#ifndef DMA_H
#define DMA_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * @brief Initialize the DMA controller.
 */
void init_dma(void);

/**
 * @brief Configure and start a DMA transfer.
 */
void dma_start_transfer(uint8_t channel, const void *read_addr, void *write_addr, uint32_t count, uint32_t ctrl);

/**
 * @brief Check if a DMA channel is busy.
 */
bool dma_is_busy(uint8_t channel);

/**
 * @brief Abort a DMA transfer on a specific channel.
 */
void dma_abort(uint8_t channel);

#endif // DMA_H
