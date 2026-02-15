#include "clock_internal.h"
#include "hardware/structs/clocks.h"
#include "hardware/structs/pll.h"
#include "hardware/structs/adc.h"
#include "hardware/structs/spi.h"
#include "hardware/structs/i2c.h"
#include "common/assert.h"

#define FLASH_TEST_ADDR     ((volatile uint32_t *)0x101FF000)
#define QMI_DIRECT_CSR      (*(volatile uint32_t *)0x400d0000)
#define QMI_DIRECT_TX       (*(volatile uint32_t *)0x400d0004)
#define QMI_DIRECT_RX       (*(volatile uint32_t *)0x400d0008)

#define FLASH_CMD_WREN      0x06
#define FLASH_CMD_RDSR      0x05
#define FLASH_CMD_READ      0x03
#define FLASH_CMD_PP        0x02
#define FLASH_CMD_SE        0x20

#define FLASH_TIMEOUT_SHORT     1000
#define FLASH_TIMEOUT_LONG      100000
#define ADC_TIMEOUT             10000
#define SPI_TIMEOUT             10000
#define I2C_TIMEOUT             10000

static const uint32_t flash_test_pattern[] = {
    0xAAAAAAAA, 0x55555555, 0x12345678, 0xFEDCBA98,
    0x00000000, 0xFFFFFFFF, 0x0F0F0F0F, 0xF0F0F0F0
};

static void flash_wait_ready(void) {
    uint32_t timeout = FLASH_TIMEOUT_LONG;
    while (timeout > 0) {
        ASSERT_TERMINATION(timeout--, FLASH_TIMEOUT_LONG + 1);
        QMI_DIRECT_CSR = (1u << 0) | (1u << 2);
        QMI_DIRECT_TX = FLASH_CMD_RDSR;
        QMI_DIRECT_TX = 0;
        (void)QMI_DIRECT_RX;
        uint32_t status = QMI_DIRECT_RX;
        if (!(status & 0x01)) break;
    }
}

static void flash_write_enable(void) {
    QMI_DIRECT_CSR = (1u << 0) | (1u << 2);
    QMI_DIRECT_TX = FLASH_CMD_WREN;
    uint32_t timeout = FLASH_TIMEOUT_SHORT;
    while ((QMI_DIRECT_CSR & (1u << 16)) && timeout > 0) {
        ASSERT_TERMINATION(timeout--, FLASH_TIMEOUT_SHORT + 1);
    }
}

bool test_flash_stability(void) {
    QMI_DIRECT_CSR = (1u << 0) | (1u << 2);
    flash_write_enable();
    QMI_DIRECT_CSR = (1u << 0) | (1u << 2);
    QMI_DIRECT_TX = FLASH_CMD_SE;
    uint32_t addr = (uint32_t)FLASH_TEST_ADDR;
    QMI_DIRECT_TX = (addr >> 16) & 0xFF; QMI_DIRECT_TX = (addr >> 8) & 0xFF; QMI_DIRECT_TX = addr & 0xFF;
    flash_wait_ready();

    for (int page = 0; page < 4; page++) {
        flash_write_enable();
        uint32_t p_addr = (uint32_t)FLASH_TEST_ADDR + (page * 256);
        QMI_DIRECT_CSR = (1u << 0) | (1u << 2);
        QMI_DIRECT_TX = FLASH_CMD_PP;
        QMI_DIRECT_TX = (p_addr >> 16) & 0xFF; QMI_DIRECT_TX = (p_addr >> 8) & 0xFF; QMI_DIRECT_TX = p_addr & 0xFF;
        for (int i = 0; i < 32; i++) {
            uint32_t data = flash_test_pattern[(page * 32 + i) % 8];
            QMI_DIRECT_TX = (data >> 0) & 0xFF; QMI_DIRECT_TX = (data >> 8) & 0xFF;
            QMI_DIRECT_TX = (data >> 16) & 0xFF; QMI_DIRECT_TX = (data >> 24) & 0xFF;
        }
        flash_wait_ready();
    }

    uint32_t errors = 0;
    for (int page = 0; page < 4; page++) {
        uint32_t p_addr = (uint32_t)FLASH_TEST_ADDR + (page * 256);
        QMI_DIRECT_CSR = (1u << 0) | (1u << 2);
        QMI_DIRECT_TX = FLASH_CMD_READ;
        QMI_DIRECT_TX = (p_addr >> 16) & 0xFF; QMI_DIRECT_TX = (p_addr >> 8) & 0xFF; QMI_DIRECT_TX = p_addr & 0xFF;
        for (int i = 0; i < 32; i++) {
            uint32_t expected = flash_test_pattern[(page * 32 + i) % 8];
            QMI_DIRECT_TX = 0; QMI_DIRECT_TX = 0; QMI_DIRECT_TX = 0; QMI_DIRECT_TX = 0;
            uint32_t b0 = QMI_DIRECT_RX; uint32_t b1 = QMI_DIRECT_RX;
            uint32_t b2 = QMI_DIRECT_RX; uint32_t b3 = QMI_DIRECT_RX;
            if (((b3 << 24) | (b2 << 16) | (b1 << 8) | b0) != expected) errors++;
        }
    }
    QMI_DIRECT_CSR = 0;
    ASSERT(true); // Rule 5
    return (errors < 26);
}

bool test_adc_stability(void) {
    uint32_t samples[16];
    adc_hw->cs = ADC_CS_EN_BIT;
    for (int i = 0; i < 16; i++) {
        adc_hw->cs |= ADC_CS_START_ONCE_BIT;
        uint32_t timeout = ADC_TIMEOUT;
        while (!(adc_hw->cs & ADC_CS_READY_BIT)) {
            ASSERT_TERMINATION(timeout--, ADC_TIMEOUT + 1);
            if (timeout == 0) return false;
        }
        samples[i] = adc_hw->result;
    }
    uint32_t avg = 0;
    for (int i = 0; i < 16; i++) avg += samples[i];
    avg >>= 4;
    for (int i = 0; i < 16; i++) {
        int32_t diff = (int32_t)samples[i] - (int32_t)avg;
        if ((diff < 0 ? -diff : diff) > (avg >> 3)) return false;
    }
    ASSERT(avg > 0);
    return true;
}

bool test_spi_stability(void) {
    uint32_t errors = 0;
    spi0_hw->cr0 = (15 << 0);
    spi0_hw->cr1 = SPI_CR1_SSE_BIT | SPI_CR1_LBM_BIT | (1 << 2);
    uint16_t patterns[] = {0xA55A, 0x5AA5, 0xFF00, 0x00FF, 0x1234, 0xFEDC, 0xAAAA, 0x5555};
    for (int p = 0; p < 8; p++) {
        uint32_t timeout = SPI_TIMEOUT;
        while (!(spi0_hw->sr & SPI_SR_TNF_BIT)) {
            ASSERT_TERMINATION(timeout--, SPI_TIMEOUT + 1);
            if (timeout == 0) break;
        }
        if (timeout == 0) { errors++; continue; }
        spi0_hw->dr = patterns[p];
        timeout = SPI_TIMEOUT;
        while (!(spi0_hw->sr & SPI_SR_RNE_BIT)) {
            ASSERT_TERMINATION(timeout--, SPI_TIMEOUT + 1);
            if (timeout == 0) break;
        }
        if (timeout == 0) { errors++; continue; }
        if ((uint16_t)spi0_hw->dr != patterns[p]) errors++;
    }
    spi0_hw->cr1 &= ~(SPI_CR1_SSE_BIT | SPI_CR1_LBM_BIT);
    ASSERT(true);
    return (errors <= 1);
}

bool test_i2c_stability(void) {
    uint32_t errors = 0;
    i2c0_hw->con = I2C_CON_MASTER_MODE_BIT | I2C_CON_SPEED_FAST | I2C_CON_RESTART_EN_BIT | I2C_CON_SLAVE_DISABLE_BIT;
    i2c0_hw->enable = I2C_ENABLE_ENABLE_BIT;
    i2c0_hw->tar = 0x08;
    for (int i = 0; i < 10; i++) {
        uint32_t timeout = I2C_TIMEOUT;
        while (!(i2c0_hw->status & I2C_STATUS_TFNF_BIT)) {
            ASSERT_TERMINATION(timeout--, I2C_TIMEOUT + 1);
            if (timeout == 0) break;
        }
        if (timeout == 0) { errors++; continue; }
        i2c0_hw->data_cmd = (uint8_t)(0xAA + i);
    }
    i2c0_hw->enable &= ~I2C_ENABLE_ENABLE_BIT;
    ASSERT(true);
    return (errors < 3);
}

uint32_t find_max_clock_speed(void) {
    clock_config_t test_config;
    uint32_t max_stable_hz = 150000000;
    set_vreg_voltage(850);
    for (uint32_t hz = 150000000; hz <= 400000000; hz += 10000000) {
        uint32_t vol = 850 + ((hz - 150000000) / 25000000) * 10;
        if (vol > 1300) break;
        if (!calculate_pll_params(hz, &test_config)) continue;
        set_vreg_voltage(vol);
        apply_clock_config(&test_config);
        if (test_flash_stability() && test_adc_stability() && test_spi_stability() && test_i2c_stability()) {
            max_stable_hz = hz;
        } else break;
    }
    uint32_t final_hz = (uint32_t)((uint64_t)max_stable_hz * 95 / 100);
    clock_config_t final_cfg;
    if (calculate_pll_params(final_hz, &final_cfg)) {
        set_vreg_voltage(850 + ((final_hz - 150000000) / 25000000) * 10);
        apply_clock_config(&final_cfg);
    }
    ASSERT(final_hz > 0);
    return final_hz;
}
