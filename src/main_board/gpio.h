#include <stdint.h>
#include <stdbool.h>

void gpio_init(void);
void gpio_set(uint8_t pin, bool value);
bool gpio_get(uint8_t pin);
void gpio_toggle(uint8_t pin);
