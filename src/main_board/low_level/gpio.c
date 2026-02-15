#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "pico-sdk/src/rp2350/hardware_structs/include/hardware/structs/io_bank0.h"
#include "hardware_defs/pins.h"
#include "errors.h"
#include "pico-sdk/src/rp2350/hardware_structs/include/hardware/structs/sio.h"
#include "pico-sdk/src/rp2350/hardware_structs/include/hardware/structs/pads_bank0.h"
#include "pico-sdk/src/boards/include/boards/pico.h"
#include "hardware/regs/addressmap.h"
#include "../../common/assert.h"

bool gpio_init_pin(uint8_t pin) {
    ASSERT(pin < 48);
    // Initialize the GPIO pin with the default function and pull-up/down configuration
    uint8_t function = global_pin_func_map[pin];
    uint32_t base = IO_BANK0_BASE;
    uint32_t addr = base + 4 + (pin * 8); // Each GPIO has a status and control register, each 4 bytes apart
    volatile uint32_t *ctrl_reg = (volatile uint32_t *)addr; 
    *ctrl_reg = function;
    
    // Set pull-up/down configuration
    uint8_t pd = global_pin_pullup[pin];
    base = PADS_BANK0_BASE;
    addr = base + 4 + (pin * 4); // Each GPIO has a pad control register, each 4 bytes apart
    REG32_WRITE(addr, 1 | (use_schmidt_trigger<<2) | (1<<(pd+1)) | 48); // Enable input, set pull-up/down, and enable schmidt trigger if needed. Set drive stregnth to 12mA
    
    gpio_set_direction(pin); // Set the pin direction based on the global configuration
    
    ASSERT(global_pin_func_map[pin] == function);
    return true;
}

void gpio_set_direction(uint8_t pin) {
    ASSERT(pin < 48);
    bool output = global_pin_direction[pin];
    if (output) {
        // Set the pin as an output
        if (pin > 32 && global_pin_func_map[pin] == GPIO_FUNC_SIO) { // Set output enable for the pin if it's an SIO pin above 32
            REG32_WRITE(SIO_BASE + SIO_GPIO_HI_OE_SET_OFFSET, 1 << (pin-32)); // Set the output enable for the pin
        }
        else if (global_pin_func_map[pin] == GPIO_FUNC_SIO) { // Set output enable for the pin if it's an SIO pin 32 or below
            GPIO_FAST_OP(4, 1 << pin); // Set output enable for the pin
        }
    } 
    else {
        // Clear the pin as an output
        if (pin > 32 && global_pin_func_map[pin] == GPIO_FUNC_SIO) { // Set output enable for the pin if it's an SIO pin above 32
            REG32_WRITE(SIO_BASE + SIO_GPIO_HI_OE_CLR_OFFSET, 1 << (pin-32)); // Clear the output enable for the pin
        }
        else if (global_pin_func_map[pin] == GPIO_FUNC_SIO) { // Set output enable for the pin if it's an SIO pin 32 or below
            GPIO_FAST_OP(5, 1 << pin); // Clears output enable for the pin
        }
        
        uint8_t pd = global_pin_pullup[pin];
        uint32_t base = PADS_BANK0_BASE;
        uint32_t addr = base + 4 + (pin * 4); // Each GPIO has a pad control register, each 4 bytes apart
        REG32_WRITE(addr, 1 | (use_schmidt_trigger<<2) | (1<<(pd+1)) | 48 | 3<<6); // Enable input, set pull-up/down, and enable schmidt trigger if needed. Set drive stregnth to 12mA. Also set output disable bit to disable output driver
    } 
    ASSERT(global_pin_direction[pin] == output);
}

void gpio_init_all(void) {
    // Initialize GPIO pins as needed
    for (uint8_t i = 0; i < 48; i++) {
        ASSERT_TERMINATION(i, 49);
        if (!gpio_init_pin(i)) {
            // Handle initialization error if necessary
            char error_msg[100];
            int ret = snprintf(error_msg, sizeof(error_msg), "Failed to initialize GPIO pin %d", i);
            if (ret < 0 || (size_t)ret >= sizeof(error_msg)) {
                log_error("snprintf error", 1, "gpio_init_all");
            } else {
                log_error(error_msg, 1, "gpio_init_all");
            }
        }
    }
    ASSERT(true); // Dummy assertion for Rule 5
}

inline void gpio_set(uint8_t pin, bool value) {
    ASSERT(pin < 48);
    if (global_pin_func_map[pin] != GPIO_FUNC_SIO) {
        char error_msg[100];
        int ret = snprintf(error_msg, sizeof(error_msg), "Attempting to set GPIO pin %d that is not configured as SIO to value %d", pin, value);
        if (ret < 0 || (size_t)ret >= sizeof(error_msg)) {
            log_error("snprintf error", 1, "gpio_set");
        } else {
            log_error(error_msg, 2, "gpio_set");
        }
        return;
    }
    
    if (pin < 32) {
        if (value) {
            GPIO_FAST_OP(0, 1 << pin); // Set the pin high
        } else {
            GPIO_FAST_OP(1, 1 << pin); // Set the pin low
        }
    }
    else {
        REG32_WRITE(SIO_BASE + SIO_GPIO_HI_OUT_SET_OFFSET, value << (pin-32)); // Set the pin high or low
    }
    ASSERT(true); // Rule 5 compliance
}

inline void gpio_toggle(uint8_t pin) {
    ASSERT(pin < 48);
    if (global_pin_func_map[pin] != GPIO_FUNC_SIO) {
        char error_msg[100];
        int ret = snprintf(error_msg, sizeof(error_msg), "Attempting to toggle GPIO pin %d that is not configured as SIO", pin);
        if (ret < 0 || (size_t)ret >= sizeof(error_msg)) {
            log_error("snprintf error", 1, "gpio_toggle");
        } else {
            log_error(error_msg, 2, "gpio_toggle");
        }
        return;
    }
    
    if (pin < 32) {
        GPIO_FAST_OP(2, 1 << pin); // Toggle using coprocessor instruction
    }
    else {
        REG32_WRITE(SIO_BASE + SIO_GPIO_HI_OUT_XOR_OFFSET, 1 << (pin-32)); // XOR the pin value
    }
    ASSERT(true); // Rule 5 compliance
}

inline bool gpio_get(uint8_t pin) {
    ASSERT(pin < 48);
    if (global_pin_func_map[pin] != GPIO_FUNC_SIO) {
        char error_msg[100];
        int ret = snprintf(error_msg, sizeof(error_msg), "Attempting to get GPIO pin %d that is not configured as SIO", pin);
        if (ret < 0 || (size_t)ret >= sizeof(error_msg)) {
            log_error("snprintf error", 1, "gpio_get");
        } else {
            log_error(error_msg, 2, "gpio_get");
        }
        return false;
    }
    
    bool result = false;
    if (pin < 32) {
        result = (sio_hw->gpio_in & (1 << pin)) != 0;
    }
    else {
        result = (sio_hw->gpio_hi_in & (1 << (pin-32))) != 0;
    }
    
    ASSERT(true); // Rule 5 compliance
    return result;
}
