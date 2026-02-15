/*
 * Optimized GPIO driver for RP2350
 * Hardcoded for 48-pin RP2350B with minimal overhead
 * Uses global pin configuration arrays from pins.h
 */
#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>
#include <stdbool.h>
#include "../hardware_defs/pins.h"
#include "../../../include/hardware/structs/sio.h"

// Pin count for RP2350B
#define GPIO_NUM_PINS       48
#define GPIO_PIN_MASK       0x3f    // 6 bits for pin number

// Core API - implemented in gpio.c
void gpio_init_all(void);
bool gpio_init_pin(uint8_t pin);
static inline void gpio_set_dir(uint8_t pin, bool output){
    global_pin_direction[pin]=output;
    gpio_init_pin(pin);
}

static inline void gpio_set_func(uint8_t pin, uint8_t function) {
    global_pin_func_map[pin]=function;
    gpio_init_pin(pin);
}
static inline void  gpio_set_pull(uint8_t pin, uint8_t pull){
    global_pin_pullup[pin]=pull;
    gpio_init_pin(pin);
}
bool gpio_get(uint8_t pin);
void gpio_toggle(uint8_t pin);
void gpio_set(uint8_t pin, bool value);

#endif // GPIO_H
