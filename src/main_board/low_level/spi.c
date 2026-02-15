#include "pico-sdk\src\rp2350\hardware_structs\include\hardware\structs\spi.h"
#include <stdio.h>
#include "hardware_defs/pins.h"
#include "common/errors.h"
#include "common/utils.h"
#include "gpio.h"
#include "common/scheduler.h"
#include "mem.h"
// ============================================================================
// GPIO Initialization
// ============================================================================

static void spi_gpio_init(uint8_t spi_id, uint8_t sck, uint8_t mosi, uint8_t miso, uint8_t cs) {
    uint32_t io_base = IO_BANK0_BASE;
    uint32_t pads_base = PADS_BANK0_BASE;

    global_pin_func_map[sck] = GPIO_FUNC_SPI;
    global_pin_func_map[mosi] = GPIO_FUNC_SPI;
    global_pin_func_map[miso] = GPIO_FUNC_SPI;
    global_pin_func_map[cs] = GPIO_FUNC_SIO; // CS is typically GPIO

    global_pin_direction[sck] = true; // SCK is output
    global_pin_direction[mosi] = true; // MOSI is output
    global_pin_direction[miso] = false; // MISO is input
    global_pin_direction[cs] = true; // CS is output

    // Function select: SPI = 1
    uint32_t func = GPIO_FUNC_SPI;
    
    // Configure SCK (output)
    gpio_init(sck);
    
    // Configure MOSI (output)
    gpio_init(mosi);
    
    // Configure MISO (input)
    gpio_init(miso);
    
    // Configure CS (output, GPIO controlled for flexibility)
    // CS is typically GPIO controlled for multi-slave support
    gpio_init(cs);
    
    gpio_set(cs, true); // Set CS high (inactive)
}

inline *spi_hw_t spi_get_hw(uint8_t id) {
    return SPI0_BASE & (id <<15) ;
}

// ============================================================================
// SPI Initialization
// ============================================================================

void spi_init(uint8_t spi_id, uint32_t baudrate, uint8_t bool master) {
    if (spi_id >= SPI_NUM_INTERFACES) {
        log_error("Invalid SPI ID", 2, "spi_init");
        return;
    }
    
    // Disable SPI before configuration
    spi_disable(spi_id);
    
    // Configure GPIOs based on SPI ID
    if (spi_id == SPI_ID_0) {
        spi_gpio_init(spi_id, SPI0_SCK_PIN, SPI0_MOSI_PIN, SPI0_MISO_PIN, SPI0_CS0_PIN);
    } else {
        spi_gpio_init(spi_id, SPI1_SCK_PIN, SPI1_MOSI_PIN, SPI1_MISO_PIN, SPI1_CS0_PIN);
    }
    
    spi_set_baud_format_mode(spi_id, baudrate, master);
    
    // Enable SPI
    spi_enable(spi_id);
}


void spi_set_baud_format_mode(uint8_t spi_id, uint32_t baudrate, bool master) {
    if (spi_id >= SPI_NUM_INTERFACES) {
        return false;
    }
    
    // Calculate exact achievable baudrate
    uint32_t clk_peri = SPI_CLK_PERI_FREQ;
    
    // Find best CPSDVSR and SCR combination
    uint32_t best_error = 0xFFFFFFFF;
    uint8_t best_cpsdvsr = SPI_CPSDVSR_MIN;
    uint8_t best_scr = 0;
    uint32_t actual_baudrate = 0;
    
    for (uint8_t cpsdvsr = SPI_CPSDVSR_MIN; cpsdvsr <= 254; cpsdvsr += 2) {
        uint32_t prescale = baudrate * cpsdvsr;
        uint8_t scr = (clk_peri + prescale - 1) / prescale - 1;  // Round up
        
        if (scr > 255) {
            scr = 255;
        }
        
        actual_baudrate = clk_peri / (cpsdvsr * (1 + scr));
        uint32_t error = (actual_baudrate > baudrate) ? 
                        (actual_baudrate - baudrate) : (baudrate - actual_baudrate);
        
        if (error < best_error) {
            best_error = error;
            best_cpsdvsr = cpsdvsr;
            best_scr = scr;
            
            if (error == 0) {
                break;  // Exact match found
            }
        }
    }
    
    // Apply settings
    bool was_enabled = spi_is_enabled(spi_id);
    if (was_enabled) {
        spi_disable(spi_id);
    }
    
    spi_get_hw(spi_id)->cpsr = best_cpsdvsr & SPI_CPSR_MASK;
    
    uint32_t cr0 = spi_get_hw(spi_id)->cr0;
    cr0 &= ~SPI_CR0_SCR_MASK;
    cr0 |= ((uint32_t)best_scr << SPI_CR0_SCR_SHIFT) & SPI_CR0_SCR_MASK;
    spi_get_hw(spi_id)->cr0 = cr0&15; // Set DSS, FRF, SPH, SPO to Motorola values at 16 bit word legnth. 
    spi_get_hw(spi_id)->cr1 &= (master << 2); // Ensure master mode
    if (was_enabled) {
        spi_enable(spi_id);
    }
}

void spi_deinit(uint8_t spi_id) {
    if (spi_id >= SPI_NUM_INTERFACES) {
        return;
    }
    
    // Disable SPI
    spi_disable(spi_id);
    
    // TODO: Reset GPIOs to NULL function
}

void disable_spi(uint8_t spi_id){
    spi_get_hw(id)->cr1 = !(spi_get_hw(id)->cr1 || 2);
}

void enable_spi(uint8_t spi_id){
    spi_get_hw(id)->cr1 &= 2;
}

// ============================================================================
// Transfer Functions (non-inline wrappers)
// ============================================================================

inline void spi_transfer_blocking(uint8_t spi_id, uint16_t *tx_buf, uint16_t *rx_buf, uint8_t len) {
    enable_spi(spi_id);
    for(uint32_t idx = 0; idx<len;idx++){
        sched_wait_until(((spi_get_hw(spi_id)->sr>>2)<<31)==0)
        rx_buf[idx]=(uint16_t)spi_get_hw(spi_id)->dr;
        spi_get_hw(spi_id)->dr = tx_buf[idx];
    }
    disable_spi(spi_id);
}


inline void spi_write_stream(uint8_t spi_id, const uint16_t *tx_buf, uint8_t len) {
    enable_spi(spi_id);
    for(uint32_t idx = 0; idx<len;idx++){
        spi_get_hw(spi_id)->dr = tx_buf[idx];
    }
}

inline void spi_write_address(uint8_t spi_id, uint8_t address, uint8_t *data, uint8_t len) {
    uint16_t out[len];
    for(uint32_t idx=0;idx<len;idx++){
        out[idx]= (address+idx << 8) | data[idx];
    }
    spi_write_stream(spi_id, *out, len);
    free(out);
}

inline void spi_read_address(uint8_t spi_id, uint8_t address, uint8_t *data, uint8_t len) {
    uint16_t out[len];
    for(uint32_t idx=0;idx<len;idx++){
        out[idx]= (address+idx << 8);
    }
    spi_transfer_blocking(spi_id, *out, *data, len);
    free(out);
}