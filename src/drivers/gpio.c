#include "kernel/assert.h"
#include <stdbool.h>
#include <stdint.h>

#include "drivers/gpio.h"
#include "hal/platform.h"

enum {
    GPIO_MAX = 48u,
    GPIO_CTRL_BASE = 0x04u,
    GPIO_CTRL_STRIDE = 0x08u,
    PAD_CTRL_BASE = 0x04u,
    PAD_CTRL_STRIDE = 0x04u,
    PAD_IE = 1u << 6,
    PAD_PUE = 1u << 3,
    PAD_PDE = 1u << 2,
    SIO_GPIO_IN = 0x04u,
    SIO_GPIO_OUT_SET = 0x14u,
    SIO_GPIO_OUT_CLR = 0x18u,
    SIO_GPIO_OE_SET = 0x24u,
    SIO_GPIO_OE_CLR = 0x28u,
};

static uint32_t bit_for_pin(uint32_t pin) {
    ASSERT(pin < GPIO_MAX);
    return 1u << (pin & 31u);
}

void gpio_init(void) {
}

void gpio_set_function(uint32_t pin, uint32_t function) {
    uint32_t off;
    ASSERT(pin < GPIO_MAX);
    off = GPIO_CTRL_BASE + (pin * GPIO_CTRL_STRIDE);
    REG_RW(IO_BANK0_BASE + off) = function & 0x1Fu;
}

void gpio_set_pulls(uint32_t pin, bool pull_up, bool pull_down) {
    uint32_t off;
    uint32_t value;
    ASSERT(pin < GPIO_MAX);
    off = PAD_CTRL_BASE + (pin * PAD_CTRL_STRIDE);
    value = PAD_IE;
    if (pull_up) {
        value |= PAD_PUE;
    }
    if (pull_down) {
        value |= PAD_PDE;
    }
    REG_RW(PADS_BANK0_BASE + off) = value;
}

void gpio_set_dir(uint32_t pin, bool is_output) {
    uint32_t bit;
    ASSERT(pin < GPIO_MAX);
    bit = bit_for_pin(pin);
    if (is_output) {
        REG_RW(SIO_BASE + SIO_GPIO_OE_SET) = bit;
    } else {
        REG_RW(SIO_BASE + SIO_GPIO_OE_CLR) = bit;
    }
}

void gpio_put(uint32_t pin, bool value) {
    uint32_t bit;
    ASSERT(pin < GPIO_MAX);
    bit = bit_for_pin(pin);
    if (value) {
        REG_RW(SIO_BASE + SIO_GPIO_OUT_SET) = bit;
    } else {
        REG_RW(SIO_BASE + SIO_GPIO_OUT_CLR) = bit;
    }
}

bool gpio_get(uint32_t pin) {
    ASSERT(pin < GPIO_MAX);
    return (REG_RO(SIO_BASE + SIO_GPIO_IN) & bit_for_pin(pin)) != 0u;
}
