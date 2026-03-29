#include "kernel/assert.h"
#include <stdbool.h>
#include <stdint.h>

#include "drivers/clock.h"
#include "drivers/uart.h"
#include "hal/platform.h"

enum {
    UART_DR = 0x00u,
    UART_FR = 0x18u,
    UART_IBRD = 0x24u,
    UART_FBRD = 0x28u,
    UART_LCR_H = 0x2Cu,
    UART_CR = 0x30u,
    UART_FR_TXFF = 1u << 5,
    UART_FR_RXFE = 1u << 4,
    UART_CR_UARTEN = 1u << 0,
    UART_CR_TXE = 1u << 8,
    UART_CR_RXE = 1u << 9,
    UART_LCR_H_FEN = 1u << 4,
    UART_LCR_H_WLEN_8 = 3u << 5,
    UART_TIMEOUT = 200000u,
};

void uart_init(uint32_t baudrate) {
    uint32_t per_clk;
    uint32_t div;
    ASSERT(baudrate > 0u);
    per_clk = clock_get_hz(CLOCK_ID_PERI);
    if (per_clk == 0u) {
        per_clk = 150000000u;
    }
    div = (8u * per_clk) / baudrate;
    REG_RW(UART0_BASE + UART_CR) = 0u;
    REG_RW(UART0_BASE + UART_IBRD) = div / 64u;
    REG_RW(UART0_BASE + UART_FBRD) = div & 63u;
    REG_RW(UART0_BASE + UART_LCR_H) = UART_LCR_H_FEN | UART_LCR_H_WLEN_8;
    REG_RW(UART0_BASE + UART_CR) = UART_CR_UARTEN | UART_CR_TXE | UART_CR_RXE;
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
    ASSERT(c != 0);
    for (i = 0u; i < UART_TIMEOUT; ++i) {
        if ((REG_RO(UART0_BASE + UART_FR) & UART_FR_RXFE) == 0u) {
            *c = (char)(REG_RO(UART0_BASE + UART_DR) & 0xFFu);
            return true;
        }
    }
    ASSERT(i < UART_TIMEOUT);
    return false;
}
