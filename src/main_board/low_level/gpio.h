#include <stdint.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "pico-sdk/src/rp2350/hardware_structs/include/hardware/structs/io_bank0.h"
#include "hardware_defs/pins.h"
#include "errors.h"
#include "pico-sdk/src/rp2350/hardware_structs/include/hardware/structs/sio.h"
#include "pico-sdk/src/rp2350/hardware_structs/include/hardware/structs/pads_bank0.h"
#include "pico-sdk/src/boards/include/boards/pico.h"

void gpio_init(void);
void gpio_set(uint8_t pin, bool value);
bool gpio_get(uint8_t pin);
void gpio_toggle(uint8_t pin);

#define GPIO_FAST_OP(op, mask) __arm_mcr(0, 0, (mask), 0, 0, (op))