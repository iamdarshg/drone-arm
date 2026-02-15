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

// Pin count for RP2350B
#define GPIO_NUM_PINS       48
#define GPIO_PIN_MASK       0x3f    // 6 bits for pin number

// Core API - implemented in gpio.c
void gpio_init_all(void);
void gpio_set_func(uint8_t pin, uint8_t func);
void gpio_set_dir(uint8_t pin, bool output);
void gpio_set_pull(uint8_t pin, uint8_t pull);
void gpio_set_drive(uint8_t pin, uint8_t drive);
void gpio_set_schmitt(uint8_t pin, bool enable);

// Fast inline operations (always inlined for speed)
static inline void gpio_set(uint8_t pin, bool value) {
    extern volatile uint32_t *const gpio_sio_out_set;
    extern volatile uint32_t *const gpio_sio_out_clr;
    extern volatile uint32_t *const gpio_sio_hi_out_set;
    extern volatile uint32_t *const gpio_sio_hi_out_clr;
    
    if (pin < 32) {
        if (value) {
            *gpio_sio_out_set = (1u << pin);
        } else {
            *gpio_sio_out_clr = (1u << pin);
        }
    } else {
        if (value) {
            *gpio_sio_hi_out_set = (1u << (pin - 32));
        } else {
            *gpio_sio_hi_out_clr = (1u << (pin - 32));
        }
    }
}

static inline bool gpio_get(uint8_t pin) {
    extern volatile uint32_t *const gpio_sio_in;
    extern volatile uint32_t *const gpio_sio_hi_in;
    
    if (pin < 32) {
        return (*gpio_sio_in >> pin) & 1u;
    } else {
        return (*gpio_sio_hi_in >> (pin - 32)) & 1u;
    }
}

static inline void gpio_toggle(uint8_t pin) {
    extern volatile uint32_t *const gpio_sio_out_xor;
    extern volatile uint32_t *const gpio_sio_hi_out_xor;
    
    if (pin < 32) {
        *gpio_sio_out_xor = (1u << pin);
    } else {
        *gpio_sio_hi_out_xor = (1u << (pin - 32));
    }
}

// Batch operations for efficiency
void gpio_set_mask(uint32_t mask_low, uint32_t mask_high, bool value);
void gpio_toggle_mask(uint32_t mask_low, uint32_t mask_high);
uint32_t gpio_get_mask(uint32_t mask_low, uint32_t mask_high);

// GPIO Coprocessor (GPC) high-speed operations
// Uses ARM MCR/MRC instructions for single-cycle atomic GPIO operations
void gpio_coprocessor_init(void);
void gpio_set_fast(uint8_t pin, bool value);
void gpio_toggle_fast(uint8_t pin);

#endif // GPIO_H
