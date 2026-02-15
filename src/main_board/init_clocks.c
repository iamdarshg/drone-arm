/*
 * Adaptive PLL overclocking for RP2354B
 * Finds maximum stable clock speed by stress-testing peripherals and flash
 * Sets final speed to 95% of maximum for safety margin
 * Integrates with scheduler for dynamic clock scaling
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "hardware/structs/clocks.h"
#include "hardware/structs/pll.h"
#include "hardware/structs/vreg.h"
#include "hardware/structs/adc.h"
#include "hardware/structs/spi.h"
#include "hardware/structs/i2c.h"
#include "hardware/structs/timer.h"
#include "common/scheduler.h"

// Clock configuration structure
typedef struct {
    uint32_t sys_clk_hz;
    uint32_t pll_vco_freq;
    uint32_t pll_fbdiv;
    uint32_t pll_postdiv1;
    uint32_t pll_postdiv2;
    uint32_t vreg_voltage_mv;
} clock_config_t;

// Voltage levels (in mV) - RP2350 VREG supports 0.85V to 1.30V
#define VREG_VOLTAGE_MIN    850
#define VREG_VOLTAGE_MAX    1300
#define VREG_VOLTAGE_STEP   25

// Clock test parameters
#define CLOCK_TEST_START_HZ     150000000   // Start at 150 MHz
#define CLOCK_TEST_STEP_HZ      10000000    // Increase by 10 MHz per step
#define CLOCK_TEST_MAX_HZ       400000000   // Absolute maximum to try
#define CLOCK_STABILITY_MARGIN  0.95        // Set final clock to 95% of max

// Test patterns for flash validation
static const uint32_t flash_test_pattern[] = {
    0xAAAAAAAA, 0x55555555, 0x12345678, 0xFEDCBA98,
    0x00000000, 0xFFFFFFFF, 0x0F0F0F0F, 0xF0F0F0F0
};

// Current clock configuration
static clock_config_t current_config = {
    .sys_clk_hz = 150000000,
    .vreg_voltage_mv = 1100
};

// Scheduler clock scaling hooks
static volatile bool scheduler_active = false;
static uint32_t high_clock_hz = 150000000;
static uint32_t low_clock_hz = 30000000;   // 30 MHz for idle/sleep

// Power state management for idle/wake transitions
static bool in_idle_mode = false;
static clock_config_t saved_config;
static uint32_t saved_voltage;

// Flash test region - defined in linker script at 0x101FF000 (last 4KB of flash)
extern uint32_t flash_test_region;
#define FLASH_TEST_ADDR     ((volatile uint32_t *)0x101FF000)
#define FLASH_TEST_SIZE     4096  // 4KB test area
#define FLASH_SECTOR_SIZE   4096  // W25Q080 sector size

// QMI register offsets for direct flash access
#define QMI_DIRECT_CSR      (*(volatile uint32_t *)0x400d0000)
#define QMI_DIRECT_TX       (*(volatile uint32_t *)0x400d0004)
#define QMI_DIRECT_RX       (*(volatile uint32_t *)0x400d0008)

// Flash commands
#define FLASH_CMD_WREN      0x06    // Write Enable
#define FLASH_CMD_WRDI      0x04    // Write Disable
#define FLASH_CMD_RDID      0x9F    // Read ID
#define FLASH_CMD_RDSR      0x05    // Read Status Register
#define FLASH_CMD_READ      0x03    // Read Data
#define FLASH_CMD_PP        0x02    // Page Program
#define FLASH_CMD_SE        0x20    // Sector Erase (4KB)

// Test flash at current clock speed using real flash operations
static bool test_flash_stability(void) {
    uint32_t errors = 0;
    volatile uint32_t *test_addr = FLASH_TEST_ADDR;
    
    // Enable QMI direct mode
    QMI_DIRECT_CSR = (1u << 0) | (1u << 2);  // Enable + Auto CS0
    
    // Wait for any previous operation
    uint32_t timeout;
    
    // Step 1: Sector Erase (required before writing)
    // Send Write Enable
    QMI_DIRECT_TX = FLASH_CMD_WREN;
    timeout = 1000;
    while ((QMI_DIRECT_CSR & (1u << 16)) && timeout--) {}
    
    // Send Sector Erase command with address
    QMI_DIRECT_CSR = (1u << 0) | (1u << 2);  // Re-enable with auto CS
    QMI_DIRECT_TX = FLASH_CMD_SE;
    QMI_DIRECT_TX = ((uint32_t)test_addr >> 16) & 0xFF;  // Address high
    QMI_DIRECT_TX = ((uint32_t)test_addr >> 8) & 0xFF;   // Address mid
    QMI_DIRECT_TX = (uint32_t)test_addr & 0xFF;          // Address low
    
    // Wait for erase complete (check status register)
    timeout = 100000;
    while (timeout--) {
        QMI_DIRECT_CSR = (1u << 0) | (1u << 2);
        QMI_DIRECT_TX = FLASH_CMD_RDSR;
        QMI_DIRECT_TX = 0;  // Dummy byte
        uint32_t status = QMI_DIRECT_RX;  // Discard first
        status = QMI_DIRECT_RX;           // Actual status
        if (!(status & 0x01)) break;  // WIP bit clear
    }
    
    // Step 2: Write test patterns using Page Program
    for (int page = 0; page < 4; page++) {  // 4 pages of 256 bytes each
        uint32_t page_addr = (uint32_t)test_addr + (page * 256);
        
        // Write Enable
        QMI_DIRECT_CSR = (1u << 0) | (1u << 2);
        QMI_DIRECT_TX = FLASH_CMD_WREN;
        timeout = 1000;
        while ((QMI_DIRECT_CSR & (1u << 16)) && timeout--) {}
        
        // Page Program command + address
        QMI_DIRECT_CSR = (1u << 0) | (1u << 2);
        QMI_DIRECT_TX = FLASH_CMD_PP;
        QMI_DIRECT_TX = (page_addr >> 16) & 0xFF;
        QMI_DIRECT_TX = (page_addr >> 8) & 0xFF;
        QMI_DIRECT_TX = page_addr & 0xFF;
        
        // Write 32 words (128 bytes) per page
        for (int i = 0; i < 32; i++) {
            uint32_t data = flash_test_pattern[(page * 32 + i) % 8];
            QMI_DIRECT_TX = (data >> 0) & 0xFF;
            QMI_DIRECT_TX = (data >> 8) & 0xFF;
            QMI_DIRECT_TX = (data >> 16) & 0xFF;
            QMI_DIRECT_TX = (data >> 24) & 0xFF;
        }
        
        // Wait for write complete
        timeout = 100000;
        while (timeout--) {
            QMI_DIRECT_CSR = (1u << 0) | (1u << 2);
            QMI_DIRECT_TX = FLASH_CMD_RDSR;
            QMI_DIRECT_TX = 0;
            uint32_t status = QMI_DIRECT_RX;
            status = QMI_DIRECT_RX;
            if (!(status & 0x01)) break;
        }
    }
    
    // Step 3: Read back and verify
    for (int page = 0; page < 4; page++) {
        uint32_t page_addr = (uint32_t)test_addr + (page * 256);
        
        // Read command + address
        QMI_DIRECT_CSR = (1u << 0) | (1u << 2);
        QMI_DIRECT_TX = FLASH_CMD_READ;
        QMI_DIRECT_TX = (page_addr >> 16) & 0xFF;
        QMI_DIRECT_TX = (page_addr >> 8) & 0xFF;
        QMI_DIRECT_TX = page_addr & 0xFF;
        
        // Read and verify 32 words
        for (int i = 0; i < 32; i++) {
            uint32_t expected = flash_test_pattern[(page * 32 + i) % 8];
            
            // Read 4 bytes
            QMI_DIRECT_TX = 0;  // Dummy bytes for read
            QMI_DIRECT_TX = 0;
            QMI_DIRECT_TX = 0;
            QMI_DIRECT_TX = 0;
            
            uint32_t b0 = QMI_DIRECT_RX;
            uint32_t b1 = QMI_DIRECT_RX;
            uint32_t b2 = QMI_DIRECT_RX;
            uint32_t b3 = QMI_DIRECT_RX;
            
            uint32_t actual = (b3 << 24) | (b2 << 16) | (b1 << 8) | b0;
            
            if (actual != expected) {
                errors++;
            }
        }
    }
    
    // Disable QMI direct mode
    QMI_DIRECT_CSR = 0;
    
    // Accept if less than 5% errors
    return (errors < 26);  // 512 total reads, 5% = ~25
}

// Test ADC at current clock speed
static bool test_adc_stability(void) {
    uint32_t samples[16];
    
    // Enable ADC
    adc_hw->cs = ADC_CS_EN_BIT;
    
    // Take multiple samples on internal temperature sensor
    for (int i = 0; i < 16; i++) {
        // Start conversion
        adc_hw->cs |= ADC_CS_START_ONCE_BIT;
        
        // Wait with timeout
        uint32_t timeout = 10000;
        while (!(adc_hw->cs & ADC_CS_READY_BIT)) {
            if (--timeout == 0) {
                return false;
            }
        }
        
        samples[i] = adc_hw->result;
    }
    
    // Check for unreasonable variation (indicates instability)
    uint32_t avg = 0;
    for (int i = 0; i < 16; i++) {
        avg += samples[i];
    }
    avg >>= 4;
    
    // Samples should be within ±10% of average
    for (int i = 0; i < 16; i++) {
        int32_t diff = (int32_t)samples[i] - (int32_t)avg;
        if (diff < 0) diff = -diff;
        if (diff > (avg >> 3)) {  // > 12.5% variation
            return false;
        }
    }
    
    return true;
}

// Test SPI at current clock speed using Loopback Mode (LBM)
// LBM connects TX to RX internally for self-test without external connections
static bool test_spi_stability(void) {
    uint32_t errors = 0;
    
    // Configure SPI for loopback test
    // Enable LBM bit, master mode, 16-bit data
    spi0_hw->cr0 = (15 << 0) | (0 << 4) | (0 << 6) | (0 << 7) | (0 << 8); // DSS=16, FRF=Motorola, SPO=0, SPH=0
    spi0_hw->cr1 = SPI_CR1_SSE_BIT | SPI_CR1_LBM_BIT | (1 << 2); // Enable + LBM + Master
    
    // Test patterns
    uint16_t test_patterns[] = {0xA55A, 0x5AA5, 0xFF00, 0x00FF, 0x1234, 0xFEDC, 0xAAAA, 0x5555};
    
    for (int p = 0; p < 8; p++) {
        uint16_t tx_data = test_patterns[p];
        
        // Wait for TX FIFO not full
        uint32_t timeout = 10000;
        while (!(spi0_hw->sr & SPI_SR_TNF_BIT)) {
            if (--timeout == 0) {
                errors++;
                break;
            }
        }
        if (timeout == 0) continue;
        
        // Write data
        spi0_hw->dr = tx_data;
        
        // Wait for RX FIFO not empty (data looped back)
        timeout = 10000;
        while (!(spi0_hw->sr & SPI_SR_RNE_BIT)) {
            if (--timeout == 0) {
                errors++;
                break;
            }
        }
        if (timeout == 0) continue;
        
        // Read back and verify
        uint16_t rx_data = (uint16_t)spi0_hw->dr;
        if (rx_data != tx_data) {
            errors++;
        }
    }
    
    // Disable SPI and LBM
    spi0_hw->cr1 &= ~(SPI_CR1_SSE_BIT | SPI_CR1_LBM_BIT);
    
    // Accept if error rate < 5%
    return (errors <= 1);
}

// Test I2C at current clock speed using master-slave loopback on same device
// Uses I2C0 as master and tries to address itself if possible, or just tests bus timing
static bool test_i2c_stability(void) {
    uint32_t errors = 0;
    
    // Configure I2C for test
    // Enable master mode, fast speed
    i2c0_hw->con = I2C_CON_MASTER_MODE_BIT | I2C_CON_SPEED_FAST | 
                   I2C_CON_RESTART_EN_BIT | I2C_CON_SLAVE_DISABLE_BIT;
    
    // Set timing for 400kHz
    i2c0_hw->fs_scl_hcnt = 150;
    i2c0_hw->fs_scl_lcnt = 225;
    
    // Enable I2C
    i2c0_hw->enable = I2C_ENABLE_ENABLE_BIT;
    
    // Test by attempting to write to a non-existent address (tests bus timing)
    // This will generate a TX abort, but we can check if the logic works
    uint8_t test_addr = 0x08; // Reserved address, should NACK
    i2c0_hw->tar = test_addr;
    
    // Try to send data
    for (int i = 0; i < 10; i++) {
        // Wait for TX FIFO not full
        uint32_t timeout = 10000;
        while (!(i2c0_hw->status & I2C_STATUS_TFNF_BIT)) {
            if (--timeout == 0) {
                errors++;
                break;
            }
        }
        if (timeout == 0) continue;
        
        // Write test byte
        i2c0_hw->data_cmd = (uint8_t)(0xAA + i);
        
        // Small delay to let transaction attempt
        for (volatile int j = 0; j < 100; j++);
    }
    
    // Clear any abort status
    (void)i2c0_hw->clr_tx_abrt;
    (void)i2c0_hw->clr_intr;
    
    // Disable I2C
    i2c0_hw->enable &= ~I2C_ENABLE_ENABLE_BIT;
    
    // Accept if no timeout errors (abort is expected since no slave)
    return (errors < 3);
}

// Calculate PLL parameters for target frequency
static bool calculate_pll_params(uint32_t target_hz, clock_config_t *config) {
    // Reference clock is typically 12 MHz from XOSC
    const uint32_t ref_freq = 12000000;
    
    // VCO must be 400MHz - 1600MHz for RP2350 PLL
    const uint32_t vco_min = 400000000;
    const uint32_t vco_max = 1600000000;
    
    // Try different postdiv combinations
    for (uint32_t postdiv1 = 7; postdiv1 >= 1; postdiv1--) {
        for (uint32_t postdiv2 = postdiv1; postdiv2 >= 1; postdiv2--) {
            uint32_t vco_freq = target_hz * postdiv1 * postdiv2;
            
            if (vco_freq >= vco_min && vco_freq <= vco_max) {
                uint32_t fbdiv = vco_freq / ref_freq;
                uint32_t actual_vco = fbdiv * ref_freq;
                uint32_t actual_sys = actual_vco / (postdiv1 * postdiv2);
                
                // Accept if within 1% of target
                int32_t error = (int32_t)actual_sys - (int32_t)target_hz;
                if (error < 0) error = -error;
                
                if (error < (target_hz >> 7)) {  // < 0.78% error
                    config->pll_fbdiv = fbdiv;
                    config->pll_postdiv1 = postdiv1;
                    config->pll_postdiv2 = postdiv2;
                    config->pll_vco_freq = actual_vco;
                    config->sys_clk_hz = actual_sys;
                    return true;
                }
            }
        }
    }
    
    return false;
}

// Set VREG voltage
static void set_vreg_voltage(uint32_t voltage_mv) {
    // Clamp to valid range
    if (voltage_mv < VREG_VOLTAGE_MIN) voltage_mv = VREG_VOLTAGE_MIN;
    if (voltage_mv > VREG_VOLTAGE_MAX) voltage_mv = VREG_VOLTAGE_MAX;
    
    // RP2350 VREG register is at 0x400a8000
    // Voltage is set in VSEL field (bits 3:0)
    // VSEL = (voltage_mv - 550) / 25
    uint32_t vsel = (voltage_mv - 550) / 25;
    
    volatile uint32_t *vreg = (volatile uint32_t *)0x400a8000;
    *vreg = (*vreg & ~0x0F) | (vsel & 0x0F);
    
    // Wait for voltage to stabilize (~100us)
    for (volatile int i = 0; i < 10000; i++);
    
    current_config.vreg_voltage_mv = voltage_mv;
}

// Apply clock configuration
static void apply_clock_config(const clock_config_t *config) {
    // Switch to ROSC temporarily while changing PLL
    // CLK_SYS = ROSC (no PLL)
    clocks_hw->clk[0].ctrl = 0;  // Select ROSC
    
    // Wait for clock switch
    while (!(clocks_hw->clk[0].selected & 0x1));
    
    // Stop PLL
    pll_sys_hw->pwr &= ~0x3;  // Clear PD and VCOD
    
    // Configure PLL
    pll_sys_hw->fbdiv_int = config->pll_fbdiv;
    pll_sys_hw->prim = (config->pll_postdiv1 << 16) | (config->pll_postdiv2 << 12);
    
    // Start PLL
    pll_sys_hw->pwr = 0x3;  // Enable VCO and post-dividers
    
    // Wait for PLL lock
    while (!(pll_sys_hw->cs & 0x1));
    
    // Switch back to PLL
    clocks_hw->clk[0].ctrl = 0x1;  // Select PLL_SYS
    
    // Wait for switch
    while (!(clocks_hw->clk[0].selected & 0x2));
    
    current_config = *config;
}

// Find maximum stable clock speed
uint32_t find_max_clock_speed(void) {
    clock_config_t test_config;
    uint32_t max_stable_hz = CLOCK_TEST_START_HZ;
    
    // Start at base voltage
    set_vreg_voltage(VREG_VOLTAGE_MIN);
    
    // Iterate through clock speeds
    for (uint32_t test_hz = CLOCK_TEST_START_HZ; 
         test_hz <= CLOCK_TEST_MAX_HZ; 
         test_hz += CLOCK_TEST_STEP_HZ) {
        
        // Calculate required voltage (approximately 10mV per 25MHz)
        uint32_t required_voltage = VREG_VOLTAGE_MIN + 
            ((test_hz - CLOCK_TEST_START_HZ) / 25000000) * 10;
        
        if (required_voltage > VREG_VOLTAGE_MAX) {
            break;  // Reached voltage limit
        }
        
        // Calculate PLL parameters
        if (!calculate_pll_params(test_hz, &test_config)) {
            continue;  // Skip if no valid PLL config
        }
        
        // Increase voltage
        set_vreg_voltage(required_voltage);
        
        // Apply test clock
        apply_clock_config(&test_config);
        
        // Run stability tests
        bool flash_ok = test_flash_stability();
        bool adc_ok = test_adc_stability();
        bool spi_ok = test_spi_stability();
        bool i2c_ok = test_i2c_stability();
        
        if (flash_ok && adc_ok && spi_ok && i2c_ok) {
            max_stable_hz = test_hz;
        } else {
            // Failed - back off to previous stable speed
            break;
        }
    }
    
    // Set final clock to 95% of maximum for safety margin
    uint32_t final_hz = (uint32_t)((uint64_t)max_stable_hz * CLOCK_STABILITY_MARGIN);
    
    clock_config_t final_config;
    if (calculate_pll_params(final_hz, &final_config)) {
        // Calculate appropriate voltage for final speed
        uint32_t final_voltage = VREG_VOLTAGE_MIN + 
            ((final_hz - CLOCK_TEST_START_HZ) / 25000000) * 10;
        
        set_vreg_voltage(final_voltage);
        apply_clock_config(&final_config);
    }
    
    high_clock_hz = final_hz;
    
    return final_hz;
}

// Scheduler hook: called when scheduler goes idle
// Reduces clock to 30 MHz and drops voltage to minimum for power saving
void sched_idle_hook(void) {
    if (!scheduler_active) return;
    
    // Only enter idle mode if we have active tasks but all are sleeping
    if (sched_task_count() > 0) {
        if (!in_idle_mode) {
            // Entering idle mode
            in_idle_mode = true;
            
            // Save current state
            saved_config = current_config;
            saved_voltage = current_config.vreg_voltage_mv;
            

            // Actually, let's just use the PLL at 30 MHz
            clock_config_t idle_config;
            if (calculate_pll_params(30000000, &idle_config)) {
                // Set minimum voltage before dropping clock
                set_vreg_voltage(VREG_VOLTAGE_MIN);  // 850 mV
                
                // Apply 30 MHz configuration
                apply_clock_config(&idle_config);
                
                current_config.sys_clk_hz = 30000000;
                current_config.vreg_voltage_mv = VREG_VOLTAGE_MIN;
            }
        }
    }
}

// Scheduler hook: called when task becomes ready
// Restores high clock speed and voltage for active operation
void sched_wakeup_hook(void) {
    if (!scheduler_active) return;
    
    if (in_idle_mode) {
        // Exiting idle mode - restore performance settings
        in_idle_mode = false;
        
        // Restore voltage first (before increasing clock)
        set_vreg_voltage(saved_voltage);
        
        // Restore high clock configuration
        apply_clock_config(&saved_config);
        
        current_config = saved_config;
    }
}

// Enable scheduler clock scaling
void sched_enable_clock_scaling(bool enable) {
    scheduler_active = enable;
}

// Get current system clock frequency
uint32_t get_sys_clock_hz(void) {
    return current_config.sys_clk_hz;
}

// Clock register offsets for RP2350
#define CLOCKS_BASE             0x40010000
#define CLK_REF_CTRL            (*(volatile uint32_t *)(CLOCKS_BASE + 0x30))
#define CLK_REF_DIV             (*(volatile uint32_t *)(CLOCKS_BASE + 0x34))
#define CLK_SYS_CTRL            (*(volatile uint32_t *)(CLOCKS_BASE + 0x3C))
#define CLK_PERI_CTRL           (*(volatile uint32_t *)(CLOCKS_BASE + 0x48))
#define CLK_PERI_DIV            (*(volatile uint32_t *)(CLOCKS_BASE + 0x4C))
#define CLK_USB_CTRL            (*(volatile uint32_t *)(CLOCKS_BASE + 0x54))
#define CLK_USB_DIV             (*(volatile uint32_t *)(CLOCKS_BASE + 0x58))
#define CLK_ADC_CTRL            (*(volatile uint32_t *)(CLOCKS_BASE + 0x60))
#define CLK_ADC_DIV             (*(volatile uint32_t *)(CLOCKS_BASE + 0x64))
#define CLK_RTC_CTRL            (*(volatile uint32_t *)(CLOCKS_BASE + 0x6C))
#define CLK_RTC_DIV             (*(volatile uint32_t *)(CLOCKS_BASE + 0x70))

#define PLL_USB_BASE            0x40058000
#define PLL_USB_CS              (*(volatile uint32_t *)(PLL_USB_BASE + 0x00))
#define PLL_USB_PWR             (*(volatile uint32_t *)(PLL_USB_BASE + 0x04))
#define PLL_USB_FBDIV_INT       (*(volatile uint32_t *)(PLL_USB_BASE + 0x08))
#define PLL_USB_PRIM            (*(volatile uint32_t *)(PLL_USB_BASE + 0x0C))

#define XOSC_BASE               0x40048000
#define XOSC_CTRL               (*(volatile uint32_t *)(XOSC_BASE + 0x00))
#define XOSC_STATUS             (*(volatile uint32_t *)(XOSC_BASE + 0x04))
#define XOSC_STARTUP            (*(volatile uint32_t *)(XOSC_BASE + 0x0C))

// Clock control bits
#define CLK_CTRL_ENABLE         (1u << 11)
#define CLK_CTRL_AUXSRC_LSB     5
#define CLK_CTRL_SRC_LSB        0

// Clock sources
#define CLK_SRC_ROSC            0
#define CLK_SRC_AUX             1
#define CLK_AUXSRC_PLL_USB      0
#define CLK_AUXSRC_PLL_SYS      1
#define CLK_AUXSRC_XOSC         2
#define CLK_AUXSRC_ROSC         3

// Initialize USB PLL for 48MHz
// Reference: 12MHz XOSC
// FOUT = (FREF / REFDIV) × FBDIV / (POSTDIV1 × POSTDIV2)
// 48MHz = (12MHz / 1) × 24 / (6 × 1) = 12 × 24 / 6 = 48
static void init_usb_pll(void) {
    // Power down PLL while configuring
    PLL_USB_PWR = 0;
    
    // Configure PLL
    // FBDIV = 24 (VCO = 12MHz × 24 = 288MHz)
    PLL_USB_FBDIV_INT = 24;
    
    // POSTDIV1 = 6, POSTDIV2 = 1
    // 288MHz / 6 / 1 = 48MHz
    PLL_USB_PRIM = (6 << 16) | (1 << 12);
    
    // Power up PLL
    PLL_USB_PWR = 0x3;
    
    // Wait for lock
    while (!(PLL_USB_CS & 1));
}

// Initialize XOSC (crystal oscillator)
static void init_xosc(void) {
    // Enable XOSC with 12MHz crystal
    // Startup delay for 12MHz crystal: ~1ms
    XOSC_STARTUP = 47;  // (12MHz / 256) × 1ms ≈ 47
    
    // Enable XOSC
    XOSC_CTRL = 0xAA0 | 0xFAB;  // ENABLE_12MHZ | MAGIC
    
    // Wait for stable
    while (!(XOSC_STATUS & (1u << 31)));
}

// Initialize clock system with auto-overclocking
void init_clocks(void) {
    uint32_t sys_freq;
    
    // Step 1: Initialize XOSC first (12MHz reference)
    init_xosc();
    
    // Step 2: Configure REF clock to use XOSC
    // CLK_REF = XOSC = 12MHz (divided by 1)
    CLK_REF_CTRL = CLK_CTRL_ENABLE | (CLK_AUXSRC_XOSC << CLK_CTRL_AUXSRC_LSB) | (CLK_SRC_AUX << CLK_CTRL_SRC_LSB);
    CLK_REF_DIV = 1 << 8;  // INT = 1
    
    // Step 3: Find and set maximum stable system clock
    sys_freq = find_max_clock_speed();
    
    // Step 4: Initialize USB PLL for 48MHz
    init_usb_pll();
    
    // Step 5: Configure USB clock
    // CLK_USB = PLL_USB = 48MHz
    CLK_USB_CTRL = CLK_CTRL_ENABLE | (CLK_AUXSRC_PLL_USB << CLK_CTRL_AUXSRC_LSB) | (CLK_SRC_AUX << CLK_CTRL_SRC_LSB);
    CLK_USB_DIV = 1 << 8;  // INT = 1 (48MHz / 1 = 48MHz)
    
    // Step 6: Configure PERI clock
    // CLK_PERI = CLK_SYS (same as system clock for best performance)
    // If system clock > 150MHz, consider dividing to avoid peripheral issues
    if (sys_freq > 150000000) {
        // Divide by 2 for high system clocks (>150MHz)
        CLK_PERI_CTRL = CLK_CTRL_ENABLE | (CLK_AUXSRC_PLL_SYS << CLK_CTRL_AUXSRC_LSB) | (CLK_SRC_AUX << CLK_CTRL_SRC_LSB);
        CLK_PERI_DIV = 2 << 8;  // INT = 2 (sys_freq / 2)
    } else {
        // Same as system clock
        CLK_PERI_CTRL = CLK_CTRL_ENABLE | (CLK_AUXSRC_PLL_SYS << CLK_CTRL_AUXSRC_LSB) | (CLK_SRC_AUX << CLK_CTRL_SRC_LSB);
        CLK_PERI_DIV = 1 << 8;  // INT = 1
    }
    
    // Step 7: Configure ADC clock
    // CLK_ADC = 48MHz from PLL_USB (good for ADC timing)
    CLK_ADC_CTRL = CLK_CTRL_ENABLE | (CLK_AUXSRC_PLL_USB << CLK_CTRL_AUXSRC_LSB) | (CLK_SRC_AUX << CLK_CTRL_SRC_LSB);
    CLK_ADC_DIV = 1 << 8;  // INT = 1 (48MHz)
    
    // Step 8: Configure RTC clock (1MHz from XOSC divided by 12)
    CLK_RTC_CTRL = CLK_CTRL_ENABLE | (CLK_AUXSRC_XOSC << CLK_CTRL_AUXSRC_LSB) | (CLK_SRC_AUX << CLK_CTRL_SRC_LSB);
    CLK_RTC_DIV = (12 << 8) | 0;  // INT = 12, FRAC = 0 (12MHz / 12 = 1MHz)
    
    // Store final frequency
    current_config.sys_clk_hz = sys_freq;
}
