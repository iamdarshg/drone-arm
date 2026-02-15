/*
 * SPI Header - Optimized for RP2350
 * Hardcoded for Motorola SPI mode (mode 0)
 */
#ifndef SPI_H
#define SPI_H

 #include <stdint.h>
 #include <stdbool.h>
 #include <stddef.h>
 #include "../../../include/hardware/structs/spi.h"
 #include "../hardware_defs/spi_config.h"

// SPI register bit definitions
#ifndef SPI_CR0_DSS_SHIFT
#define SPI_CR0_DSS_SHIFT       0
#endif
#ifndef SPI_CR0_DSS_MASK
#define SPI_CR0_DSS_MASK        (0xFU << SPI_CR0_DSS_SHIFT)
#endif
#ifndef SPI_CR0_FRF_SHIFT
#define SPI_CR0_FRF_SHIFT       4
#endif
#ifndef SPI_CR0_FRF_MASK
#define SPI_CR0_FRF_MASK        (0x3U << SPI_CR0_FRF_SHIFT)
#endif
#ifndef SPI_CR0_SPO_BIT
#define SPI_CR0_SPO_BIT         (1U << 6)
#endif
#ifndef SPI_CR0_SPH_BIT
#define SPI_CR0_SPH_BIT         (1U << 7)
#endif
#ifndef SPI_CR0_SCR_SHIFT
#define SPI_CR0_SCR_SHIFT       8
#endif
#ifndef SPI_CR0_SCR_MASK
#define SPI_CR0_SCR_MASK        (0xFFU << SPI_CR0_SCR_SHIFT)
#endif

#ifndef SPI_CR1_LBM_BIT
#define SPI_CR1_LBM_BIT         (1U << 0)
#endif
#ifndef SPI_CR1_SSE_BIT
#define SPI_CR1_SSE_BIT         (1U << 1)
#endif
#ifndef SPI_CR1_MS_BIT
#define SPI_CR1_MS_BIT          (1U << 2)
#endif
#ifndef SPI_CR1_SOD_BIT
#define SPI_CR1_SOD_BIT         (1U << 3)
#endif

#ifndef SPI_SR_TFE_BIT
#define SPI_SR_TFE_BIT          (1U << 0)
#endif
#ifndef SPI_SR_TNF_BIT
#define SPI_SR_TNF_BIT          (1U << 1)
#endif
#ifndef SPI_SR_RNE_BIT
#define SPI_SR_RNE_BIT          (1U << 2)
#endif
#ifndef SPI_SR_RFF_BIT
#define SPI_SR_RFF_BIT          (1U << 3)
#endif
#ifndef SPI_SR_BSY_BIT
#define SPI_SR_BSY_BIT          (1U << 4)
#endif

#define SPI_CPSR_MASK           0xFE

// Hardware pointer (defined in spi.c)
#include "../../../include/hardware/structs/spi.h"
extern spi_hw_t* spi_get_hw(uint8_t id);

// Control functions
/** @brief Standardized SPI initialization wrapper. */
void init_spi(void);

void spi_init(uint8_t spi_id, uint32_t baudrate, bool master);
bool spi_set_baud_format_mode(uint8_t spi_id, uint32_t baudrate, bool master);
void spi_deinit(uint8_t spi_id);
void disable_spi(uint8_t spi_id);
void enable_spi(uint8_t spi_id);

// Transfer functions
// Transfer functions
bool spi_transfer_blocking(uint8_t spi_id, const uint8_t *tx_buf, uint8_t *rx_buf, size_t len);
bool spi_write_stream(uint8_t spi_id, const uint8_t *tx_buf, size_t len);
bool spi_write_address(uint8_t spi_id, uint8_t address, const uint8_t *data, size_t len);
bool spi_read_address(uint8_t spi_id, uint8_t address, uint8_t *data, size_t len);

// Interrupt handler for FIFO full - waits via scheduler
void spi_fifo_full_handler(uint8_t spi_id);

#endif // SPI_H
