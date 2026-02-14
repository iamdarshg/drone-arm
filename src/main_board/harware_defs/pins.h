#include "pico-sdk/src/rp2350/hardware_structs/include/hardware/structs/io_bank0.h"
#include <stdint.h>
#include <stdbool.h>
#ifndef PINS_H
#define PINS_H

#define GPIO_FUNC_HSTX  0, ///< Select HSTX as GPIO pin function
#define GPIO_FUNC_SPI  1, ///< Select SPI as GPIO pin function
#define GPIO_FUNC_UART  2, ///< Select UART as GPIO pin function
#define GPIO_FUNC_I2C 3, ///< Select I2C as GPIO pin function
#define GPIO_FUNC_PWM  4, ///< Select PWM as GPIO pin function
#define GPIO_FUNC_SIO  5, ///< Select SIO as GPIO pin function
#define GPIO_FUNC_PIO0  6, ///< Select PIO0 as GPIO pin function
#define GPIO_FUNC_PIO1  7, ///< Select PIO1 as GPIO pin function
#define GPIO_FUNC_PIO2  8, ///< Select PIO2 as GPIO pin function
#define GPIO_FUNC_GPCK 9, ///< Select GPCK as GPIO pin function
#define GPIO_FUNC_XIP_CS1  9, ///< Select XIP CS1 as GPIO pin function
#define GPIO_FUNC_CORESIGHT_TRACE  9, ///< Select CORESIGHT TRACE as GPIO pin function
#define GPIO_FUNC_USB  10, ///< Select USB as GPIO pin function
#define GPIO_FUNC_UART_AUX  11, ///< Select UART_AUX as GPIO pin function
#define GPIO_FUNC_NULL  0x1f, ///< Select NULL as GPIO pin function

uint8_t global_pin_func_map[48] = {
    GPIO_FUNC_NULL, // GPIO0
    GPIO_FUNC_NULL, // GPIO1
    GPIO_FUNC_NULL, // GPIO2
    GPIO_FUNC_NULL, // GPIO3
    GPIO_FUNC_NULL, // GPIO4
    GPIO_FUNC_NULL, // GPIO5
    GPIO_FUNC_NULL, // GPIO6
    GPIO_FUNC_NULL, // GPIO7
    GPIO_FUNC_NULL, // GPIO8
    GPIO_FUNC_NULL, // GPIO9
    GPIO_FUNC_NULL, // GPIO10
    GPIO_FUNC_NULL, // GPIO11
    GPIO_FUNC_NULL, // GPIO12
    GPIO_FUNC_NULL, // GPIO13
    GPIO_FUNC_NULL, // GPIO14
    GPIO_FUNC_NULL, // GPIO15
    GPIO_FUNC_NULL, // GPIO16
    GPIO_FUNC_NULL, // GPIO17
    GPIO_FUNC_NULL, // GPIO18
    GPIO_FUNC_NULL, // GPIO19
    GPIO_FUNC_NULL, // GPIO20
    GPIO_FUNC_NULL, // GPIO21
    GPIO_FUNC_NULL, // GPIO22
    GPIO_FUNC_NULL, // GPIO23
    GPIO_FUNC_NULL, // GPIO24
    GPIO_FUNC_NULL, // GPIO25
    GPIO_FUNC_NULL, // GPIO26
    GPIO_FUNC_NULL, // GPIO27
    GPIO_FUNC_NULL, // GPIO28
    GPIO_FUNC_NULL, //GPIO29 
    GPIO_FUNC_NULL, //GPIO30 
    GPIO_FUNC_NULL,  //GPIO31
    GPIO_FUNC_NULL, //GPIO32
    GPIO_FUNC_NULL, //GPIO33
    GPIO_FUNC_NULL, //GPIO34
    GPIO_FUNC_NULL, //GPIO35
    GPIO_FUNC_NULL, //GPIO36
    GPIO_FUNC_NULL, //GPIO37
    GPIO_FUNC_NULL, //GPIO38
    GPIO_FUNC_NULL, //GPIO39
    GPIO_FUNC_NULL, //GPIO40
    GPIO_FUNC_NULL, //GPIO41
    GPIO_FUNC_NULL, //GPIO42
    GPIO_FUNC_NULL, //GPIO43
    GPIO_FUNC_NULL, //GPIO44
    GPIO_FUNC_NULL, //GPIO45
    GPIO_FUNC_NULL, //GPIO46
    GPIO_FUNC_NULL //GPIO47
};


#define GPIO_PULL_NONE 0
#define GPIO_PULL_UP 1
#define GPIO_PULL_DOWN 2
#define use_schmidt_trigger true
uint8_t global_pin_pullup[48] = {
    GPIO_PULL_NONE, // GPIO0
    GPIO_PULL_NONE, // GPIO1
    GPIO_PULL_NONE, // GPIO2
    GPIO_PULL_NONE, // GPIO3
    GPIO_PULL_NONE, // GPIO4
    GPIO_PULL_NONE, // GPIO5
    GPIO_PULL_NONE, // GPIO6
    GPIO_PULL_NONE, // GPIO7
    GPIO_PULL_NONE, // GPIO8
    GPIO_PULL_NONE, // GPIO9
    GPIO_PULL_NONE, // GPIO10
    GPIO_PULL_NONE, // GPIO11
    GPIO_PULL_NONE, // GPIO12
    GPIO_PULL_NONE, // GPIO13
    GPIO_PULL_NONE, // GPIO14
    GPIO_PULL_NONE, // GPIO15
    GPIO_PULL_NONE, // GPIO16
    GPIO_PULL_NONE, //GPIO17 
    GPIO_PULL_NONE, //GPIO18
    GPIO_PULL_NONE, //GPIO19
    GPIO_PULL_NONE, //GPIO20
    GPIO_PULL_NONE, //GPIO21
    GPIO_PULL_NONE, //GPIO22
    GPIO_PULL_NONE, //GPIO23
    GPIO_PULL_NONE, //GPIO24
    GPIO_PULL_NONE, //GPIO25
    GPIO_PULL_NONE, //GPIO26
    GPIO_PULL_NONE, //GPIO27
    GPIO_PULL_NONE, //GPIO28
    GPIO_PULL_NONE, //GPIO29
    GPIO_PULL_NONE, //GPIO30
    GPIO_PULL_NONE, //GPIO31
    GPIO_PULL_NONE, //GPIO32
    GPIO_PULL_NONE, //GPIO33
    GPIO_PULL_NONE, //GPIO34
    GPIO_PULL_NONE, //GPIO35
    GPIO_PULL_NONE, //GPIO36
    GPIO_PULL_NONE, //GPIO37
    GPIO_PULL_NONE, //GPIO38
    GPIO_PULL_NONE, //GPIO39
    GPIO_PULL_NONE, //GPIO40
    GPIO_PULL_NONE, //GPIO41
    GPIO_PULL_NONE, //GPIO42
    GPIO_PULL_NONE, //GPIO43
    GPIO_PULL_NONE, //GPIO44
    GPIO_PULL_NONE, //GPIO45
    GPIO_PULL_NONE, //GPIO46
    GPIO_PULL_NONE //GPIO47
};

uint8_t global_pin_direction[48] = { // 0 for input, 1 for output
    0, // GPIO0
    0, // GPIO1
    0, // GPIO2
    0, // GPIO3
    0, // GPIO4
    0, // GPIO5
    0, // GPIO6
    0, // GPIO7
    0, // GPIO8
    0, // GPIO9
    0, // GPIO10
    0, // GPIO11
    0, // GPIO12
    0, // GPIO13
    0, // GPIO14
    0, // GPIO15
    0, // GPIO16
    0, //GPIO17 
    0, //GPIO18
    0, //GPIO19
    0, //GPIO20
    0, //GPIO21
    0, //GPIO22
    0, //GPIO23
    0, //GPIO24
    0, //GPIO25
    0, //GPIO26
    0, //GPIO27
    0, //GPIO28
    0, //GPIO29
    0, //GPIO30
    0, //GPIO31
    0, //GPIO32
    0, //GPIO33
    0, //GPIO34
    0, //GPIO35
    0, //GPIO36
    0, //GPIO37
    0, //GPIO38
    0, //GPIO39
    0, //GPIO40
    0, //GPIO41
    0, //GPIO42
    0, //GPIO43
    0, //GPIO44
    0, //GPIO45
    0, //GPIO46
    0//GPIO47 
}

#endif // PINS_H