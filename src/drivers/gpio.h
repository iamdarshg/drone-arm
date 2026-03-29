#ifndef DRIVERS_GPIO_H
#define DRIVERS_GPIO_H

#include <stdbool.h>
#include <stdint.h>

void gpio_init(void);
void gpio_set_function(uint32_t pin, uint32_t function);
void gpio_set_pulls(uint32_t pin, bool pull_up, bool pull_down);
void gpio_set_dir(uint32_t pin, bool is_output);
void gpio_put(uint32_t pin, bool value);
bool gpio_get(uint32_t pin);

#endif
