#ifndef DRIVERS_UART_H
#define DRIVERS_UART_H

#include <stdbool.h>
#include <stdint.h>

void     uart_init(uint32_t baudrate);
bool     uart_putc(char c);
bool     uart_getc(char *c);
bool     uart_puts(const char *s);

/* Pure-computation helpers exposed for host tests. */
uint32_t uart_calc_ibrd(uint32_t clk_hz, uint32_t baud);
uint32_t uart_calc_fbrd(uint32_t clk_hz, uint32_t baud);

#endif
