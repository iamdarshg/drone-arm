/*
 * IO Bank0 hardware structs for GPIO control
 */
#ifndef _HARDWARE_STRUCTS_IO_BANK0_H
#define _HARDWARE_STRUCTS_IO_BANK0_H

#include "../address_mapped.h"
#include "../regs/addressmap.h"

// GPIO function select values (FUNCSEL field)
// Use a typedef to avoid conflicts with user code
typedef enum {
    GPIO_FUNC_XIP   = 0,
    GPIO_FUNC_SPI   = 1,
    GPIO_FUNC_UART  = 2,
    GPIO_FUNC_I2C   = 3,
    GPIO_FUNC_PWM   = 4,
    GPIO_FUNC_SIO   = 5,  // Single-cycle IO (software controlled)
    GPIO_FUNC_PIO0  = 6,
    GPIO_FUNC_PIO1  = 7,
    GPIO_FUNC_PIO2  = 8,
    GPIO_FUNC_GPCK  = 9,
    GPIO_FUNC_USB   = 10,
    GPIO_FUNC_NULL  = 0x1f,
} gpio_function_t;

// GPIO status/control registers (one per GPIO)
typedef struct {
    io_ro_32 status;
    io_rw_32 ctrl;
} io_bank0_status_ctrl_t;

// Interrupt registers
typedef struct {
    io_rw_32 inte;
    io_rw_32 intf;
    io_ro_32 ints;
} io_bank0_irq_t;

// IO Bank0 hardware register block
typedef struct {
    // Status and control for GPIO0-47 (6 banks of 8, but we have 48 GPIOs on RP2350B)
    io_bank0_status_ctrl_t gpio[48];
    
    uint32_t _pad0[8];  // Padding to align to 0x100
    
    // Interrupt registers per GPIO
    io_bank0_irq_t irq[48];
} io_bank0_hw_t;

#define io_bank0_hw ((io_bank0_hw_t *const)IO_BANK0_BASE)

// CTRL register field offsets
#define IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB     0
#define IO_BANK0_GPIO0_CTRL_FUNCSEL_BITS    0x0000001f
#define IO_BANK0_GPIO0_CTRL_OUTOVER_LSB     8
#define IO_BANK0_GPIO0_CTRL_OUTOVER_BITS    0x00000300
#define IO_BANK0_GPIO0_CTRL_OEOVER_LSB      12
#define IO_BANK0_GPIO0_CTRL_OEOVER_BITS     0x00003000
#define IO_BANK0_GPIO0_CTRL_INOVER_LSB      16
#define IO_BANK0_GPIO0_CTRL_INOVER_BITS     0x00030000
#define IO_BANK0_GPIO0_CTRL_IRQOVER_LSB     28
#define IO_BANK0_GPIO0_CTRL_IRQOVER_BITS    0x30000000

#endif // _HARDWARE_STRUCTS_IO_BANK0_H
