#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "pico-sdk/src/rp2350/hardware_structs/include/hardware/structs/io_bank0.h"
#include "hardware_defs/pins.h"
#include "errors.h"
#include "pico-sdk/src/rp2350/hardware_structs/include/hardware/structs/sio.h"
#include "pico-sdk/src/rp2350/hardware_structs/include/hardware/structs/pads_bank0.h"
#include "pico-sdk/src/boards/include/boards/pico.h"


void gpio_init_pins(void);
void gpio_set(uint8_t pin, bool value);
bool gpio_get(uint8_t pin);
void gpio_toggle(uint8_t pin);
void gpio_set_direction(uint8_t pin);

bool gpio_init_pin(uint8_t pin) {
    // Initialize the GPIO pin with the default function and pull-up/down configuration
    gpio_function_t function = global_pin_func_map[pin];
    uint32_t base = IO_BANK0_BASE;
    uint32_t addr = base + 4 + (pin * 8); // Each GPIO has a status and control register, each 4 bytes apart
    volatile uint32_t *ctrl_reg = (volatile uint32_t *)addr; 
    *ctrl_reg = function;
    // Set pull-up/down configuration, PADS registers are required for functioning, we assume all pin fucntions stay constant thorughout flight, or that this function is called again if/when they are changed.s
    uint8_t pd = global_pin_pullup[pin];
    base = PADS_BANK0_BASE;
    addr = base + 4 + (pin * 4); // Each GPIO has a pad control register, each 4 bytes apart
    REG32_WRITE(addr, 1 | (use_schmidt_trigger<<2) | (1<<(pd+1)) | 48); // Enable input, set pull-up/down, and enable schmidt trigger if needed. Set drive stregnth to 12mA
    // TODO: Add error handling if necessary, for example if an invalid pin number is provided or if the hardware registers cannot be accessed
    return true;
}

void gpio_set_direction(uint8_t pin) {
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
        base = PADS_BANK0_BASE;
        addr = base + 4 + (pin * 4); // Each GPIO has a pad control register, each 4 bytes apart
        REG32_WRITE(addr, 1 | (use_schmidt_trigger<<2) | (1<<(pd+1)) | 48 | 3<<6); // Enable input, set pull-up/down, and enable schmidt trigger if needed. Set drive stregnth to 12mA. Also set output disable bit to disable output driver
    } 
}

void gpio_init_pins(void) {
    // Initialize GPIO pins as needed
    for (uint8_t i = 0; i < 48; i++) {
        if (!gpio_init_pin(i)) {
            // Handle initialization error if necessary
            char error_msg[100];
            snprintf(error_msg, sizeof(error_msg), "Failed to initialize GPIO pin %d", i);
            log_error(error_msg, 1, "gpio_init_pins");
        }
    }
}


void gpio_set(uint8_t pin, bool value) {
    if (global_pin_func_map[pin] != GPIO_FUNC_SIO) {
        char error_msg[100];
        snprintf(error_msg, sizeof(error_msg), "Attempting to set GPIO pin %d that is not configured as SIO to value %d", pin, value);
        log_error(error_msg, 2, "gpio_set");
        return;
    }
    if (pin<32){
        if (value) {
            GPIO_FAST_OP(0, 1 << pin); // Set the pin high using coprocessor instruction for fast operation
        } else {
            GPIO_FAST_OP(1, 1 << pin); // Set the pin low using coprocessor instruction for fast operations
        }
    }
    else{
        REG32_WRITE(SIO_BASE + SIO_GPIO_HI_OUT_SET_OFFSET, value << (pin-32)); // Set the pin high or low
    }
}

void gpio_toggle(uint8_t pin) {
    if (global_pin_func_map[pin] != GPIO_FUNC_SIO) {
        char error_msg[100];
        snprintf(error_msg, sizeof(error_msg), "Attempting to toggle GPIO pin %d that is not configured as SIO", pin);
        log_error(error_msg, 2, "gpio_toggle");
        return;
    }
    if (pin<32){
        GPIO_FAST_OP(2, 1 << pin); // Toggle using coprocessor instruction for fast operation
    }
    else{
        REG32_WRITE(SIO_BASE + SIO_GPIO_HI_OUT_XOR_OFFSET, 1 << (pin-32)); // XOR the pin value
    }
}

void gpio_get(uint8_t pin){
    if (global_pin_func_map[pin] != GPIO_FUNC_SIO) {
        char error_msg[100];
        snprintf(error_msg, sizeof(error_msg), "Attempting to get GPIO pin %d that is not configured as SIO", pin);
        log_error(error_msg, 2, "gpio_get");
        return;
    }
    if (pin<32){
        return (sio_hw->gpio_in & (1 << pin)) != 0; // Read the pin value using coprocessor instruction for fast operation
    }
    else{
        return (sio_hw->gpio_hi_in & (1 << (pin-32))) != 0; // Read the pin value
    }
}
