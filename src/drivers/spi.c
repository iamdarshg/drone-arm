#include "kernel/assert.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "drivers/spi.h"
#include "hal/platform.h"

enum {
    SPI_SSPCR0 = 0x00u,
    SPI_SSPCR1 = 0x04u,
    SPI_SSPDR = 0x08u,
    SPI_SSPSR = 0x0Cu,
    SPI_SSPCPSR = 0x10u,
    SPI_SR_TFE = 1u << 0,
    SPI_SR_TNF = 1u << 1,
    SPI_SR_RNE = 1u << 2,
    SPI_SR_BSY = 1u << 4,
    SPI_CR1_SSE = 1u << 1,
    SPI_TIMEOUT = 200000u,
};

static uint32_t spi_base(void) {
    return SPI0_BASE;
}

static bool wait_sr(uint32_t mask, bool set) {
    uint32_t i;
    uint32_t base;
    base = spi_base();
    for (i = 0u; i < SPI_TIMEOUT; ++i) {
        uint32_t sr;
        sr = REG_RO(base + SPI_SSPSR);
        if (set) {
            if ((sr & mask) != 0u) {
                return true;
            }
        } else if ((sr & mask) == 0u) {
            return true;
        }
    }
    ASSERT(i < SPI_TIMEOUT);
    return false;
}

void spi_init(uint32_t bitrate_hz) {
    (void)bitrate_hz;
    REG_RW(spi_base() + SPI_SSPCR1) = 0u;
    REG_RW(spi_base() + SPI_SSPCPSR) = 2u;
    REG_RW(spi_base() + SPI_SSPCR0) = 0x0007u;
    REG_RW(spi_base() + SPI_SSPCR1) = SPI_CR1_SSE;
}

bool spi_transfer(const uint8_t *tx, uint8_t *rx, size_t len) {
    size_t i;
    uint32_t base;
    base = spi_base();
    for (i = 0u; i < len; ++i) {
        uint32_t out;
        if (!wait_sr(SPI_SR_TNF, true)) {
            return false;
        }
        out = tx != 0 ? tx[i] : 0xFFu;
        REG_RW(base + SPI_SSPDR) = out;
        if (!wait_sr(SPI_SR_RNE, true)) {
            return false;
        }
        out = REG_RO(base + SPI_SSPDR) & 0xFFu;
        if (rx != 0) {
            rx[i] = (uint8_t)out;
        }
    }
    if (!wait_sr(SPI_SR_BSY, false)) {
        return false;
    }
    return wait_sr(SPI_SR_TFE, true);
}
