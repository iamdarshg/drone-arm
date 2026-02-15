/*
 * SPI interrupt handler with scheduler wait
 * Handles RX/TX FIFO full conditions
 */
#include "spi.h"
#include "pico-sdk\src\rp2350\hardware_structs\include\hardware\structs\spi.h"
#include "common/scheduler.h"

// SPI FIFO status bits
#define SPI_FIFO_RX_FULL    (1U << 3)   // RFF
#define SPI_FIFO_TX_FULL    (1U << 4)   // TNF inverted

// Interrupt handler for SPI FIFO full
// Called when RX FIFO is full or TX FIFO is full
// Waits using scheduler until condition clears
void spi_fifo_full_handler(uint8_t spi_id) {
    if (spi_id >= SPI_NUM_INTERFACES) {
        return;
    }
    
    spi_hw_t *hw = spi_get_hw(spi_id);
    
    // Check which FIFO is full and wait accordingly
    uint32_t sr = hw->sr;
    
    if (sr & SPI_FIFO_RX_FULL) {
        // RX FIFO full - wait until not full
        sched_wait_until((bool (*)(void))((sr & SPI_FIFO_RX_FULL) == 0));
    } else if (!(sr & SPI_SR_TNF_BIT)) {
        // TX FIFO full - wait until not full
        sched_wait_until((bool (*)(void))((sr & SPI_SR_TNF_BIT) != 0));
    }
}

// Alternative: Non-blocking check with yield
void spi_fifo_wait_yield(uint8_t spi_id) {
    if (spi_id >= SPI_NUM_INTERFACES) {
        return;
    }
    
    spi_hw_t *hw = spi_get_hw(spi_id);
    uint32_t sr = hw->sr;
    
    // If either FIFO is full, yield to scheduler
    if ((sr & SPI_FIFO_RX_FULL) || !(sr & SPI_SR_TNF_BIT)) {
        sched_yield();
    }
}
