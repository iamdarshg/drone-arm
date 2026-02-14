#include <stdint.h>
#include <stdbool.h>

void uart_init(uint8_t uart_id, uint32_t baud_rate);
void uart_write(uint8_t uart_id, uint8_t byte);
uint8_t uart_read(uint8_t uart_id);
bool uart_available(uint8_t uart_id);
void uart_write_string(uint8_t uart_id, const char *str);
