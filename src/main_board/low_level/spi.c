/**
 * SPI Driver for Main Controller (RP2350B)
 * P10 Compliant - Fixed memory management and control flow
 */

#include "spi.h"
#include "../../../include/hardware/structs/spi.h"
#include "../../../include/hardware/structs/io_bank0.h"
#include "../../../include/hardware/regs/addressmap.h"
#include "../../common/assert.h"
#include "../../common/errors.h"
#include "../hardware_defs/pins.h"
#include "gpio.h"
#include "scheduler.h"

// ============================================================================
// CONFIGURATION
// ============================================================================

// Maximum transfer size for pre-allocated buffers
#define SPI_MAX_TRANSFER_SIZE 32

// ============================================================================
// STATIC STATE (Pre-allocated at init time - no dynamic memory)
// ============================================================================

// Pre-allocated buffers for address-based operations (Rule 3 compliance)
static uint16_t spi_tx_buffer[SPI_MAX_TRANSFER_SIZE];
static uint16_t spi_rx_buffer[SPI_MAX_TRANSFER_SIZE];
static bool spi_initialized = false;

// ============================================================================
// HARDWARE ACCESS
// ============================================================================

spi_hw_t* spi_get_hw(uint8_t id) {
    PRECONDITION(id < SPI_NUM_INTERFACES);
    return (spi_hw_t*)(SPI0_BASE + (id * 0x1000));
}

// ============================================================================
// GPIO INITIALIZATION
// ============================================================================

static void spi_gpio_init(uint8_t spi_id, uint8_t sck, uint8_t mosi, uint8_t miso, uint8_t cs) {
    PRECONDITION(spi_id < SPI_NUM_INTERFACES);
    ASSERT_RANGE(sck, 0, 47);
    ASSERT_RANGE(mosi, 0, 47);
    ASSERT_RANGE(miso, 0, 47);
    ASSERT_RANGE(cs, 0, 47);
    
    // Configure pin function map
    global_pin_func_map[sck] = GPIO_FUNC_SPI;
    global_pin_func_map[mosi] = GPIO_FUNC_SPI;
    global_pin_func_map[miso] = GPIO_FUNC_SPI;
    global_pin_func_map[cs] = GPIO_FUNC_SIO;

    global_pin_direction[sck] = true;
    global_pin_direction[mosi] = true;
    global_pin_direction[miso] = false;
    global_pin_direction[cs] = true;

    // Configure pins
    gpio_init(sck);
    gpio_init(mosi);
    gpio_init(miso);
    gpio_init(cs);
    gpio_set(cs, true);
}

// ============================================================================
// INITIALIZATION
// ============================================================================

void spi_init(uint8_t spi_id, uint32_t baudrate, bool master) {
    PRECONDITION(spi_id < SPI_NUM_INTERFACES);
    ASSERT(baudrate > 0);
    
    // Mark as initialized
    spi_initialized = true;
    
    // Disable SPI before configuration
    disable_spi(spi_id);
    
    // Configure GPIOs based on SPI ID
    if (spi_id == SPI_ID_0) {
        spi_gpio_init(spi_id, SPI0_SCK_PIN, SPI0_MOSI_PIN, SPI0_MISO_PIN, SPI0_CS0_PIN);
    } else {
        spi_gpio_init(spi_id, SPI1_SCK_PIN, SPI1_MOSI_PIN, SPI1_MISO_PIN, SPI1_CS0_PIN);
    }
    
    bool result = spi_set_baud_format_mode(spi_id, baudrate, master);
    ASSERT_MSG(result, "SPI baud rate configuration failed");
    
    // Enable SPI
    enable_spi(spi_id);
}

// Helper function to calculate best baud rate parameters
static bool calculate_spi_params(uint32_t baudrate, uint8_t* best_cpsdvsr, uint8_t* best_scr) {
    PRECONDITION(baudrate > 0);
    ASSERT_NOT_NULL(best_cpsdvsr);
    ASSERT_NOT_NULL(best_scr);
    
    uint32_t clk_peri = SPI_CLK_PERI_FREQ;
    uint32_t best_error = 0xFFFFFFFFU;
    bool found = false;
    
    // Fixed upper bound loop (Rule 2 compliance)
    for (uint8_t cpsdvsr = SPI_CPSDVSR_MIN; cpsdvsr <= 254; cpsdvsr += 2) {
        ASSERT_TERMINATION(cpsdvsr, 255);
        
        uint32_t prescale = baudrate * cpsdvsr;
        uint8_t scr = (uint8_t)((clk_peri + prescale - 1) / prescale - 1);
        
        if (scr > 255) {
            scr = 255;
        }
        
        uint32_t actual_baudrate = clk_peri / (cpsdvsr * (1 + scr));
        uint32_t error = (actual_baudrate > baudrate) ? 
                        (actual_baudrate - baudrate) : (baudrate - actual_baudrate);
        
        if (error < best_error) {
            best_error = error;
            *best_cpsdvsr = cpsdvsr;
            *best_scr = scr;
            found = true;
            
            if (error == 0) {
                break;  // Exact match found
            }
        }
    }
    
    return found;
}

bool spi_set_baud_format_mode(uint8_t spi_id, uint32_t baudrate, bool master) {
    PRECONDITION(spi_id < SPI_NUM_INTERFACES);
    ASSERT(baudrate > 0);
    
    uint8_t best_cpsdvsr;
    uint8_t best_scr;
    
    if (!calculate_spi_params(baudrate, &best_cpsdvsr, &best_scr)) {
        return false;
    }
    
    bool was_enabled = (spi_get_hw(spi_id)->cr1 & SPI_CR1_SSE_BIT) != 0;
    if (was_enabled) {
        disable_spi(spi_id);
    }
    
    // Set clock prescaler
    spi_get_hw(spi_id)->cpsr = best_cpsdvsr & SPI_CPSR_MASK;
    
    // Set control register 0
    uint32_t cr0 = spi_get_hw(spi_id)->cr0;
    cr0 &= ~SPI_CR0_SCR_MASK;
    cr0 |= ((uint32_t)best_scr << SPI_CR0_SCR_SHIFT) & SPI_CR0_SCR_MASK;
    cr0 &= 0x0FU;  // Set DSS, FRF, SPH, SPO to Motorola values at 16-bit word length
    spi_get_hw(spi_id)->cr0 = cr0;
    
    // Set master/slave mode
    if (master) {
        spi_get_hw(spi_id)->cr1 &= ~SPI_CR1_MS_BIT;
    } else {
        spi_get_hw(spi_id)->cr1 |= SPI_CR1_MS_BIT;
    }
    
    if (was_enabled) {
        enable_spi(spi_id);
    }
    
    return true;
}

void spi_deinit(uint8_t spi_id) {
    PRECONDITION(spi_id < SPI_NUM_INTERFACES);
    
    disable_spi(spi_id);
    // Note: GPIO reset not implemented as per original
}

void disable_spi(uint8_t spi_id) {
    PRECONDITION(spi_id < SPI_NUM_INTERFACES);
    spi_get_hw(spi_id)->cr1 &= ~SPI_CR1_SSE_BIT;
}

void enable_spi(uint8_t spi_id) {
    PRECONDITION(spi_id < SPI_NUM_INTERFACES);
    spi_get_hw(spi_id)->cr1 |= SPI_CR1_SSE_BIT;
}

// ============================================================================
// TRANSFER FUNCTIONS
// ============================================================================

void spi_transfer_blocking(uint8_t spi_id, uint16_t *tx_buf, uint16_t *rx_buf, uint32_t len) {
    PRECONDITION(spi_id < SPI_NUM_INTERFACES);
    ASSERT_NOT_NULL(tx_buf);
    ASSERT_NOT_NULL(rx_buf);
    ASSERT(len > 0);
    ASSERT(len <= SPI_MAX_TRANSFER_SIZE);
    ASSERT(spi_initialized);
    
    enable_spi(spi_id);
    
    for (uint32_t idx = 0; idx < len; idx++) {
        ASSERT_TERMINATION(idx, SPI_MAX_TRANSFER_SIZE);
        
        // Wait until TX FIFO not full (Rule 2: fixed bound spin wait)
        uint32_t timeout = 100000;
        while ((spi_get_hw(spi_id)->sr & SPI_SR_TNF_BIT) == 0) {
            ASSERT_TERMINATION(timeout--, 100001);
            if (timeout == 0) {
                log_error("SPI TX timeout", 2, "spi_transfer_blocking");
                disable_spi(spi_id);
                return;
            }
        }
        
        spi_get_hw(spi_id)->dr = tx_buf[idx];
        
        // Wait until RX FIFO not empty
        timeout = 100000;
        while ((spi_get_hw(spi_id)->sr & SPI_SR_RNE_BIT) == 0) {
            ASSERT_TERMINATION(timeout--, 100001);
            if (timeout == 0) {
                log_error("SPI RX timeout", 2, "spi_transfer_blocking");
                disable_spi(spi_id);
                return;
            }
        }
        
        rx_buf[idx] = (uint16_t)spi_get_hw(spi_id)->dr;
    }
    
    disable_spi(spi_id);
}

void spi_write_stream(uint8_t spi_id, const uint16_t *tx_buf, uint32_t len) {
    PRECONDITION(spi_id < SPI_NUM_INTERFACES);
    ASSERT_NOT_NULL(tx_buf);
    ASSERT(len > 0);
    ASSERT(len <= SPI_MAX_TRANSFER_SIZE);
    ASSERT(spi_initialized);
    
    enable_spi(spi_id);
    
    for (uint32_t idx = 0; idx < len; idx++) {
        ASSERT_TERMINATION(idx, SPI_MAX_TRANSFER_SIZE);
        
        // Wait until TX FIFO not full
        uint32_t timeout = 100000;
        while ((spi_get_hw(spi_id)->sr & SPI_SR_TNF_BIT) == 0) {
            ASSERT_TERMINATION(timeout--, 100001);
            if (timeout == 0) {
                log_error("SPI TX timeout", 2, "spi_write_stream");
                disable_spi(spi_id);
                return;
            }
        }
        
        spi_get_hw(spi_id)->dr = tx_buf[idx];
    }
    
    // Wait for transfer complete
    uint32_t timeout = 100000;
    while ((spi_get_hw(spi_id)->sr & SPI_SR_BSY_BIT) != 0) {
        ASSERT_TERMINATION(timeout--, 100001);
        if (timeout == 0) {
            break;
        }
    }
    
    disable_spi(spi_id);
}

void spi_write_address(uint8_t spi_id, uint8_t address, uint8_t *data, uint8_t len) {
    PRECONDITION(spi_id < SPI_NUM_INTERFACES);
    ASSERT_NOT_NULL(data);
    ASSERT(len > 0);
    ASSERT(len <= SPI_MAX_TRANSFER_SIZE - 1);  // Leave room for address
    ASSERT(spi_initialized);
    
    // Use pre-allocated buffer (Rule 3: no dynamic memory)
    spi_tx_buffer[0] = (uint16_t)((address << 8) | data[0]);
    
    for (uint8_t idx = 1; idx < len; idx++) {
        ASSERT_TERMINATION(idx, SPI_MAX_TRANSFER_SIZE);
        spi_tx_buffer[idx] = (uint16_t)((address + idx) << 8) | data[idx];
    }
    
    spi_write_stream(spi_id, spi_tx_buffer, len);
}

void spi_read_address(uint8_t spi_id, uint8_t address, uint8_t *data, uint8_t len) {
    PRECONDITION(spi_id < SPI_NUM_INTERFACES);
    ASSERT_NOT_NULL(data);
    ASSERT(len > 0);
    ASSERT(len <= SPI_MAX_TRANSFER_SIZE - 1);
    ASSERT(spi_initialized);
    
    // Use pre-allocated buffers
    for (uint8_t idx = 0; idx < len; idx++) {
        ASSERT_TERMINATION(idx, SPI_MAX_TRANSFER_SIZE);
        spi_tx_buffer[idx] = (uint16_t)((address + idx) << 8);
    }
    
    spi_transfer_blocking(spi_id, spi_tx_buffer, spi_rx_buffer, len);
    
    // Extract data from receive buffer
    for (uint8_t idx = 0; idx < len; idx++) {
        ASSERT_TERMINATION(idx, SPI_MAX_TRANSFER_SIZE);
        data[idx] = (uint8_t)(spi_rx_buffer[idx] & 0xFF);
    }
}

// ============================================================================
// INTERRUPT HANDLER
// ============================================================================

void spi_fifo_full_handler(uint8_t spi_id) {
    PRECONDITION(spi_id < SPI_NUM_INTERFACES);
    log_error("SPI FIFO full", 1, "spi_fifo_full_handler");
    // Clear FIFO or reset SPI as needed
    disable_spi(spi_id);
    enable_spi(spi_id);
}
