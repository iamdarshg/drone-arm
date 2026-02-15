#include "../../../include/hardware/structs/io_bank0.h"
#include <stdint.h>
#include <stdbool.h>
#ifndef PINS_H
#define PINS_H

#define GPIO_FUNC_HSTX  0 ///< Select HSTX as GPIO pin function
#define GPIO_FUNC_SPI  1 ///< Select SPI as GPIO pin function
#define GPIO_FUNC_UART  2 ///< Select UART as GPIO pin function
#define GPIO_FUNC_I2C 3 ///< Select I2C as GPIO pin function
#define GPIO_FUNC_PWM  4 ///< Select PWM as GPIO pin function
#define GPIO_FUNC_SIO  5 ///< Select SIO as GPIO pin function
#define GPIO_FUNC_PIO0  6 ///< Select PIO0 as GPIO pin function
#define GPIO_FUNC_PIO1  7 ///< Select PIO1 as GPIO pin function
#define GPIO_FUNC_PIO2  8 ///< Select PIO2 as GPIO pin function
#define GPIO_FUNC_GPCK 9 ///< Select GPCK as GPIO pin function
#define GPIO_FUNC_XIP_CS1  9 ///< Select XIP CS1 as GPIO pin function
#define GPIO_FUNC_CORESIGHT_TRACE  9 ///< Select CORESIGHT TRACE as GPIO pin function
#define GPIO_FUNC_USB  10 ///< Select USB as GPIO pin function
#define GPIO_FUNC_UART_AUX  11 ///< Select UART_AUX as GPIO pin function
#define GPIO_FUNC_NULL  0x1f ///< Select NULL as GPIO pin function

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
uint8_t global_pin_pullup[48]

uint8_t global_pin_direction[48]

// SPI0 default pins
#define SPI0_SCK_PIN    18
#define SPI0_MOSI_PIN   19
#define SPI0_MISO_PIN   16
#define SPI0_CS0_PIN    17
#define SPI0_CS1_PIN    20
#define SPI0_CS2_PIN    21

// SPI1 default pins
#define SPI1_SCK_PIN    10
#define SPI1_MOSI_PIN   11
#define SPI1_MISO_PIN   12
#define SPI1_CS0_PIN    13
#define SPI1_CS1_PIN    14
#define SPI1_CS2_PIN    15  

// I2C0 default pins
#define I2C0_SCL_PIN    5
#define I2C0_SDA_PIN    4

// I2C1 default pins
#define I2C1_SCL_PIN    7
#define I2C1_SDA_PIN    6

#endif // PINS_H