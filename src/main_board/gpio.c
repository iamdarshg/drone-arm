#include <stdint.h>
#include <stdbool.h>

void gpio_init_pins(void);
void gpio_set(uint8_t pin, bool value);
bool gpio_get(uint8_t pin);
void gpio_toggle(uint8_t pin);
void gpio_set_direction(uint8_t pin, bool output);
void gpio_set_function(uint8_t pin, uint8_t function);
