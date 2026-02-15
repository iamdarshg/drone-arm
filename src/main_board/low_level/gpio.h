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
void gpio_init_pins(void);
bool gpio_init_pin(uint8_t pin);
void gpio_set_direction(uint8_t pin);
void gpio_set_func(uint8_t pin, uint8_t func);
void gpio_set_dir(uint8_t pin, bool output);
void gpio_set_pull(uint8_t pin, uint8_t pull);
void gpio_set_drive(uint8_t pin, uint8_t drive);
void gpio_set_schmitt(uint8_t pin, bool enable);

// Fast inline operations (always inlined for speed)
// These use direct register access through sio_hw pointer
static inline void gpio_set(uint8_t pin, bool value) {
    if (pin >= GPIO_NUM_PINS) return;
    
    if (pin < 32) {
        if (value) {
            sio_hw->gpio_out_set = (1u << pin);
        } else {
            sio_hw->gpio_out_clr = (1u << pin);
        }
    } else {
        if (value) {
            sio_hw->gpio_hi_out_set = (1u << (pin - 32));
        } else {
            sio_hw->gpio_hi_out_clr = (1u << (pin - 32));
        }
    }
}

static inline bool gpio_get(uint8_t pin) {
    if (pin >= GPIO_NUM_PINS) return false;
    
    if (pin < 32) {
        return (sio_hw->gpio_in & (1u << pin)) != 0;
    } else {
        return (sio_hw->gpio_hi_in & (1u << (pin - 32))) != 0;
    }
}

static inline void gpio_toggle(uint8_t pin) {
    if (pin >= GPIO_NUM_PINS) return;
    
    if (pin < 32) {
        sio_hw->gpio_out_xor = (1u << pin);
    } else {
        sio_hw->gpio_hi_out_xor = (1u << (pin - 32));
    }
}

// Batch operations for efficiency
void gpio_set_mask(uint32_t mask_low, uint32_t mask_high, bool value);
void gpio_toggle_mask(uint32_t mask_low, uint32_t mask_high);
uint32_t gpio_get_mask(uint32_t mask_low, uint32_t mask_high);

// GPIO fast operations using direct register access
void gpio_set_fast(uint8_t pin, bool value);
void gpio_toggle_fast(uint8_t pin);

#endif // GPIO_H
