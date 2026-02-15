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

extern uint8_t global_pin_func_map[48];


#define GPIO_PULL_NONE 0
#define GPIO_PULL_UP 1
#define GPIO_PULL_DOWN 2
#define use_schmidt_trigger true

extern uint8_t global_pin_pullup[48];
#define GPIO_DIR_IN 0
#define GPIO_DIR_OUT 1

extern uint8_t global_pin_direction[48];

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