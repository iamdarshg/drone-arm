#include "kernel/assert.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "drivers/clock.h"
#include "drivers/dma.h"
#include "drivers/spi.h"
#include "hal/platform.h"

/*
 * ARM PrimeCell SSP (PL022) register offsets (RP2350 TRM §4.4).
 */
enum {
    SPI_SSPCR0  = 0x00u,  /* [15:8]=SCR  [7]=SPH [6]=SPO [5:4]=FRF [3:0]=DSS */
    SPI_SSPCR1  = 0x04u,  /* [1]=SSE (enable)                                  */
    SPI_SSPDR   = 0x08u,
    SPI_SSPSR   = 0x0Cu,  /* Status: [4]=BSY [3]=RFF [2]=RNE [1]=TNF [0]=TFE  */
    SPI_SSPCPSR = 0x10u,  /* Clock prescale divisor (CPSDVSR, even 2-254)      */
    /* SR bits */
    SPI_SR_TFE  = (1u << 0),
    SPI_SR_TNF  = (1u << 1),
    SPI_SR_RNE  = (1u << 2),
    SPI_SR_BSY  = (1u << 4),
    /* CR1 bits */
    SPI_CR1_SSE = (1u << 1),
    SPI_TIMEOUT = 200000u,
    /* SSPCR0 field values for 8-bit Motorola SPI (CPOL=0, CPHA=0, FRF=0) */
    SPI_CR0_DSS8_MOTO = 0x0007u,  /* DSS=8-bit, FRF=Motorola, SPO=0, SPH=0 */
};

static uint32_t spi_base(void) { return SPI0_BASE; }

/*
 * Compute the SSP clock prescale divisor (CPSDVSR) and serial clock rate
 * (SCR) such that:
 *   bitrate = SSPCLK / (CPSDVSR × (1 + SCR))
 *
 * Strategy: fix CPSDVSR=2 (minimum) and derive SCR.  This works for all
 * practical SPI speeds (up to 75 MHz) with a 150 MHz peripheral clock.
 * If SCR would exceed 255 increase CPSDVSR by 2 until it fits.
 *
 * Both out-parameters receive the final values.
 */
void spi_calc_divisors(uint32_t clk_hz, uint32_t bitrate_hz,
                       uint32_t *cpsdvsr, uint32_t *scr) {
    uint32_t dvsr;
    ASSERT(clk_hz > 0u);
    ASSERT(bitrate_hz > 0u);
    ASSERT(cpsdvsr != NULL);
    ASSERT(scr != NULL);
    dvsr = 2u;
    while (dvsr <= 254u) {
        uint32_t s = (clk_hz / (dvsr * bitrate_hz));
        if (s >= 1u && s <= 256u) {
            *cpsdvsr = dvsr;
            *scr     = s - 1u;
            return;
        }
        dvsr += 2u;
    }
    /* Fallback: slowest valid setting */
    *cpsdvsr = 254u;
    *scr     = 255u;
}

void spi_init(uint32_t bitrate_hz) {
    uint32_t per_clk;
    uint32_t cpsdvsr;
    uint32_t scr;
    ASSERT(bitrate_hz > 0u);

    per_clk = clock_get_hz(CLOCK_ID_PERI);
    if (per_clk == 0u) {
        per_clk = 150000000u;
    }
    spi_calc_divisors(per_clk, bitrate_hz, &cpsdvsr, &scr);

    /* Disable SSP before programming clock and format fields. */
    REG_RW(spi_base() + SPI_SSPCR1) = 0u;
    /* Set clock prescaler */
    REG_RW(spi_base() + SPI_SSPCPSR) = cpsdvsr;
    /* Set SCR, Motorola frame, 8-bit data */
    REG_RW(spi_base() + SPI_SSPCR0) = (scr << 8) | SPI_CR0_DSS8_MOTO;
    /* Enable SSP */
    REG_RW(spi_base() + SPI_SSPCR1) = SPI_CR1_SSE;
}

static bool wait_sr(uint32_t mask, bool set) {
    uint32_t i;
    uint32_t base = spi_base();
    for (i = 0u; i < SPI_TIMEOUT; ++i) {
        uint32_t sr = REG_RO(base + SPI_SSPSR);
        if (set  && ((sr & mask) != 0u)) { return true; }
        if (!set && ((sr & mask) == 0u)) { return true; }
    }
    ASSERT(i < SPI_TIMEOUT);
    return false;
}

bool spi_transfer(const uint8_t *tx, uint8_t *rx, size_t len) {
    size_t   i;
    uint32_t base = spi_base();
    for (i = 0u; i < len; ++i) {
        uint32_t out;
        if (!wait_sr(SPI_SR_TNF, true)) { return false; }
        out = (tx != NULL) ? (uint32_t)tx[i] : 0xFFu;
        REG_RW(base + SPI_SSPDR) = out;
        if (!wait_sr(SPI_SR_RNE, true)) { return false; }
        out = REG_RO(base + SPI_SSPDR) & 0xFFu;
        if (rx != NULL) { rx[i] = (uint8_t)out; }
    }
    if (!wait_sr(SPI_SR_BSY, false)) { return false; }
    return wait_sr(SPI_SR_TFE, true);
}

bool spi_transfer_dma(uint32_t tx_channel, uint32_t rx_channel,
                      const uint8_t *tx, uint8_t *rx, size_t len) {
    uint32_t base = spi_base();
    static uint8_t g_tx_dummy = 0xFFu;
    static uint8_t g_rx_sink;
    uintptr_t tx_src;
    uintptr_t rx_dst;
    ASSERT(len > 0u);
    tx_src = (tx != NULL) ? (uintptr_t)tx : (uintptr_t)&g_tx_dummy;
    rx_dst = (rx != NULL) ? (uintptr_t)rx : (uintptr_t)&g_rx_sink;

    dma_channel_start(rx_channel,
                      (uintptr_t)(base + SPI_SSPDR), rx_dst, len,
                      DMA_SIZE_BYTE, false, (rx != NULL), DREQ_SPI0_RX);
    dma_channel_start(tx_channel,
                      tx_src, (uintptr_t)(base + SPI_SSPDR), len,
                      DMA_SIZE_BYTE, (tx != NULL), false, DREQ_SPI0_TX);

    if (!dma_channel_wait(tx_channel)) {
        return false;
    }
    if (!dma_channel_wait(rx_channel)) {
        return false;
    }
    if (!wait_sr(SPI_SR_BSY, false)) {
        return false;
    }
    return wait_sr(SPI_SR_TFE, true);
}
