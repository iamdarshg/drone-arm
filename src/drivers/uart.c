#include "kernel/assert.h"
#include <stdbool.h>
#include <stdint.h>

#include "drivers/clock.h"
#include "drivers/uart.h"
#include "hal/platform.h"

/*
 * ARM PL011 UART register offsets (RP2350 TRM §4.2).
 */
enum {
    UART_DR    = 0x00u,
    UART_FR    = 0x18u,  /* Flag Register                    */
    UART_IBRD  = 0x24u,  /* Integer Baud Rate Divisor        */
    UART_FBRD  = 0x28u,  /* Fractional Baud Rate Divisor     */
    UART_LCR_H = 0x2Cu,  /* Line Control Register            */
    UART_CR    = 0x30u,  /* Control Register                 */
    /* FR bits */
    UART_FR_TXFF = (1u << 5),  /* TX FIFO full                */
    UART_FR_RXFE = (1u << 4),  /* RX FIFO empty               */
    /* CR bits */
    UART_CR_UARTEN = (1u << 0),
    UART_CR_TXE    = (1u << 8),
    UART_CR_RXE    = (1u << 9),
    /* LCR_H bits */
    UART_LCR_H_FEN    = (1u << 4),  /* FIFO enable               */
    UART_LCR_H_WLEN_8 = (3u << 5),  /* 8-bit word length         */
    UART_TIMEOUT = 200000u,
};

/*
 * Compute the integer baud-rate divisor.
 *
 * ARM PL011 formula (TRM §2.6):
 *   BRD      = UARTCLK / (16 × baud)
 *   IBRD     = floor(BRD)
 *   FBRD     = round(frac(BRD) × 64)
 *
 * SDK shorthand (avoids 64-bit maths):
 *   div  = (8 × UARTCLK) / baud         [integer; gives 128×BRD]
 *   IBRD = div >> 7                      [= floor(BRD)]
 *   FBRD = ((div & 0x7F) + 1) >> 1      [rounds frac × 64]
 */
uint32_t uart_calc_ibrd(uint32_t clk_hz, uint32_t baud) {
    ASSERT(clk_hz > 0u);
    ASSERT(baud > 0u);
    return (8u * clk_hz / baud) >> 7;
}

uint32_t uart_calc_fbrd(uint32_t clk_hz, uint32_t baud) {
    uint32_t div;
    ASSERT(clk_hz > 0u);
    ASSERT(baud > 0u);
    div = 8u * clk_hz / baud;
    return ((div & 0x7Fu) + 1u) >> 1u;
}

void uart_init(uint32_t baudrate) {
    uint32_t per_clk;
    ASSERT(baudrate > 0u);

    per_clk = clock_get_hz(CLOCK_ID_PERI);
    if (per_clk == 0u) {
        per_clk = 150000000u;
    }

    /* Disable UART before changing LCR_H (ARM PL011 TRM §3.3.7). */
    REG_RW(UART0_BASE + UART_CR) = 0u;

    REG_RW(UART0_BASE + UART_IBRD)  = uart_calc_ibrd(per_clk, baudrate);
    REG_RW(UART0_BASE + UART_FBRD)  = uart_calc_fbrd(per_clk, baudrate);
    REG_RW(UART0_BASE + UART_LCR_H) = UART_LCR_H_FEN | UART_LCR_H_WLEN_8;
    REG_RW(UART0_BASE + UART_CR)    = UART_CR_UARTEN | UART_CR_TXE | UART_CR_RXE;
}

bool uart_putc(char c) {
    uint32_t i;
    for (i = 0u; i < UART_TIMEOUT; ++i) {
        if ((REG_RO(UART0_BASE + UART_FR) & UART_FR_TXFF) == 0u) {
            REG_RW(UART0_BASE + UART_DR) = (uint32_t)(uint8_t)c;
            return true;
        }
    }
    ASSERT(i < UART_TIMEOUT);
    return false;
}

bool uart_getc(char *c) {
    uint32_t i;
    ASSERT(c != NULL);
    for (i = 0u; i < UART_TIMEOUT; ++i) {
        if ((REG_RO(UART0_BASE + UART_FR) & UART_FR_RXFE) == 0u) {
            *c = (char)(REG_RO(UART0_BASE + UART_DR) & 0xFFu);
            return true;
        }
    }
    ASSERT(i < UART_TIMEOUT);
    return false;
}

bool uart_puts(const char *s) {
    ASSERT(s != NULL);
    while (*s != '\0') {
        if (!uart_putc(*s)) {
            return false;
        }
        s++;
    }
    return true;
}
