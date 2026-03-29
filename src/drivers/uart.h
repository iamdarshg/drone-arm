#ifndef DRIVERS_UART_H
#define DRIVERS_UART_H

#include <stdbool.h>
#include <stdint.h>

void uart_init(uint32_t baudrate);
bool uart_putc(char c);
bool uart_getc(char *c);

#endif
