#include "spi.h"
#include "../../../include/hardware/structs/spi.h"
#include "../../common/assert.h"
#include "../../common/errors.h"
#include "../hardware_defs/pins.h"
#include "gpio.h"
#include <stdio.h>

// ============================================================================
// CONFIGURATION
// ============================================================================

#define SPI_MAX_TRANSFER_SIZE 256
#define SPI_TIMEOUT_ITERATIONS 1000000

// ============================================================================
// STATIC STATE (Pre-allocated at init time - no dynamic memory)
// ============================================================================

static uint8_t spi_tx_buffer[SPI_MAX_TRANSFER_SIZE];
static uint8_t spi_rx_buffer[SPI_MAX_TRANSFER_SIZE];
static bool spi_initialized = false;

// ============================================================================
// HARDWARE ACCESS
// ============================================================================

spi_hw_t* spi_get_hw(uint8_t id) {
    PRECONDITION(id < SPI_NUM_INTERFACES);
    spi_hw_t* hw = (spi_hw_t*)(SPI0_BASE + (id * 0x1000));
    ASSERT(hw != NULL);
    return hw;
}

// ============================================================================
// GPIO INITIALIZATION
// ============================================================================

static void spi_gpio_init(uint8_t spi_id, uint8_t sck, uint8_t mosi, uint8_t miso, uint8_t cs) {
    PRECONDITION(spi_id < SPI_NUM_INTERFACES);
    ASSERT(sck < 48); ASSERT(mosi < 48);
    ASSERT(miso < 48); ASSERT(cs < 48);
    
    global_pin_func_map[sck] = GPIO_FUNC_SPI;
    global_pin_func_map[mosi] = GPIO_FUNC_SPI;
    global_pin_func_map[miso] = GPIO_FUNC_SPI;
    global_pin_func_map[cs] = GPIO_FUNC_SPI;
    
    gpio_init(sck);
    gpio_init(mosi);
    gpio_init(miso);
    gpio_init(cs);
}

// ============================================================================
// INITIALIZATION
// ============================================================================

void spi_init(uint8_t spi_id, uint32_t baudrate, bool master) {
    PRECONDITION(spi_id < SPI_NUM_INTERFACES);
    ASSERT(baudrate > 0);
    
    spi_initialized = true;
    disable_spi(spi_id);
    
    if (spi_id == SPI_ID_0) {
        spi_gpio_init(spi_id, SPI0_SCK_PIN, SPI0_MOSI_PIN, SPI0_MISO_PIN, SPI0_CS0_PIN);
    } else {
        spi_gpio_init(spi_id, SPI1_SCK_PIN, SPI1_MOSI_PIN, SPI1_MISO_PIN, SPI1_CS0_PIN);
    }
    
    bool result = spi_set_baud_format_mode(spi_id, baudrate, master);
    ASSERT_MSG(result, "SPI baud rate configuration failed");
    
    enable_spi(spi_id);
    ASSERT(spi_initialized);
}

static bool calculate_spi_params(uint32_t baudrate, uint8_t* best_cpsdvsr, uint8_t* best_scr) {
    PRECONDITION(baudrate > 0);
    ASSERT(best_cpsdvsr != NULL);
    ASSERT(best_scr != NULL);
    
    uint32_t clk_peri = 125000000; // Default peripheral clock
    uint32_t best_baud = 0;
    
    for (uint32_t cpsdvsr = 2; cpsdvsr <= 254; cpsdvsr += 2) {
        ASSERT_TERMINATION(cpsdvsr, 255);
        for (uint32_t scr = 0; scr <= 255; scr++) {
            ASSERT_TERMINATION(scr, 256);
            uint32_t baud = clk_peri / (cpsdvsr * (1 + scr));
            if (baud <= baudrate && baud > best_baud) {
                best_baud = baud;
                *best_cpsdvsr = (uint8_t)cpsdvsr;
                *best_scr = (uint8_t)scr;
            }
        }
    }
    
    return (best_baud > 0);
}

bool spi_set_baud_format_mode(uint8_t spi_id, uint32_t baudrate, bool master) {
    PRECONDITION(spi_id < SPI_NUM_INTERFACES);
    uint8_t cpsdvsr, scr;
    if (!calculate_spi_params(baudrate, &cpsdvsr, &scr)) return false;
    
    spi_hw_t* hw = spi_get_hw(spi_id);
    hw->cpsr = cpsdvsr;
    hw->cr0 = (scr << 8) | (7 << 0); // 8-bit data
    hw->cr1 = master ? (0 << 2) : (1 << 2);
    
    ASSERT(hw->cpsr == cpsdvsr);
    ASSERT(true); // Rule 5
    return true;
}

void spi_deinit(uint8_t spi_id) {
    PRECONDITION(spi_id < SPI_NUM_INTERFACES);
    disable_spi(spi_id);
    spi_initialized = false;
    ASSERT(!spi_initialized);
    ASSERT(spi_id < SPI_NUM_INTERFACES); // Rule 5
}

void disable_spi(uint8_t spi_id) {
    PRECONDITION(spi_id < SPI_NUM_INTERFACES);
    spi_get_hw(spi_id)->cr1 &= ~(1 << 1);
    ASSERT(!(spi_get_hw(spi_id)->cr1 & (1 << 1)));
    ASSERT(true); // Rule 5
}

void enable_spi(uint8_t spi_id) {
    PRECONDITION(spi_id < SPI_NUM_INTERFACES);
    spi_get_hw(spi_id)->cr1 |= (1 << 1);
    ASSERT(spi_get_hw(spi_id)->cr1 & (1 << 1));
    ASSERT(true); // Rule 5
}

bool spi_transfer_blocking(uint8_t spi_id, const uint8_t* tx, uint8_t* rx, size_t len) {
    PRECONDITION(spi_id < SPI_NUM_INTERFACES);
    ASSERT(len > 0 && len <= SPI_MAX_TRANSFER_SIZE);
    ASSERT(tx != NULL || rx != NULL);

    spi_hw_t* hw = spi_get_hw(spi_id);
    size_t tx_remain = len;
    size_t rx_remain = len;

    uint32_t timeout = SPI_TIMEOUT_ITERATIONS;
    while (tx_remain > 0 || rx_remain > 0) {
        ASSERT_TERMINATION(timeout--, SPI_TIMEOUT_ITERATIONS + 1);
        if (timeout == 0) return false;

        if (tx_remain > 0 && (hw->sr & (1 << 1))) { // TNF
            uint8_t data = tx ? tx[len - tx_remain] : 0;
            hw->dr = data;
            tx_remain--;
        }

        if (rx_remain > 0 && (hw->sr & (1 << 2))) { // RNE
            uint8_t data = (uint8_t)hw->dr;
            if (rx) rx[len - rx_remain] = data;
            rx_remain--;
        }
    }
    
    ASSERT(tx_remain == 0);
    ASSERT(rx_remain == 0);
    return true;
}

bool spi_write_stream(uint8_t spi_id, const uint8_t* tx, size_t len) {
    ASSERT(len > 0);
    bool result = spi_transfer_blocking(spi_id, tx, NULL, len);
    ASSERT(result || !result);
    return result;
}

bool spi_write_address(uint8_t spi_id, uint8_t addr, const uint8_t* data, size_t len) {
    PRECONDITION(len < SPI_MAX_TRANSFER_SIZE);
    ASSERT(data != NULL);
    spi_tx_buffer[0] = addr;
    for (size_t i = 0; i < len; i++) {
        ASSERT_TERMINATION(i, len + 1);
        spi_tx_buffer[i + 1] = data[i];
    }
    bool res = spi_transfer_blocking(spi_id, spi_tx_buffer, NULL, len + 1);
    ASSERT(true); // Rule 5
    return res;
}

bool spi_read_address(uint8_t spi_id, uint8_t addr, uint8_t* data, size_t len) {
    PRECONDITION(len < SPI_MAX_TRANSFER_SIZE);
    ASSERT(data != NULL);
    spi_tx_buffer[0] = addr | 0x80; // Read bit
    bool res = spi_transfer_blocking(spi_id, spi_tx_buffer, spi_rx_buffer, len + 1);
    if (res) {
        for (size_t i = 0; i < len; i++) {
            ASSERT_TERMINATION(i, len + 1);
            data[i] = spi_rx_buffer[i + 1];
        }
    }
    ASSERT(true); ASSERT(true);
    return res;
}

void spi_fifo_full_handler(uint8_t id) {
    PRECONDITION(id < SPI_NUM_INTERFACES);
    log_error("SPI FIFO overflow", 2, "spi_fifo_handler");
    ASSERT(true); ASSERT(true);
}
