#ifndef PLATFORM_H
#define PLATFORM_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef volatile uint32_t io_rw_32;
typedef const volatile uint32_t io_ro_32;
typedef volatile uint32_t io_wo_32;

#define ROM_BASE         0x00000000UL
#define XIP_BASE         0x10000000UL
#define XIP_SRAM_BASE    0x13FFC000UL
#define SRAM_BASE        0x20000000UL
#define SRAM_SCRATCH_X_BASE 0x20080000UL
#define SRAM_SCRATCH_Y_BASE 0x20081000UL

#define SYSINFO_BASE     0x40000000UL
#define SYSCFG_BASE      0x40008000UL
#define CLOCKS_BASE      0x40010000UL
#define PSM_BASE         0x40018000UL
#define RESETS_BASE      0x40020000UL
#define IO_BANK0_BASE    0x40028000UL
#define IO_BANK1_BASE    0x40030000UL
#define PADS_BANK0_BASE  0x40038000UL
#define PADS_BANK1_BASE  0x40040000UL
#define XOSC_BASE        0x40048000UL
#define PLL_SYS_BASE     0x40050000UL
#define PLL_USB_BASE     0x40058000UL
#define UART0_BASE       0x40070000UL
#define UART1_BASE       0x40078000UL
#define SPI0_BASE        0x40080000UL
#define SPI1_BASE        0x40088000UL
#define I2C0_BASE        0x40090000UL
#define I2C1_BASE        0x40098000UL
#define ADC_BASE         0x400A0000UL
#define PWM_BASE         0x400A8000UL
#define TIMER0_BASE      0x400B0000UL
#define TIMER1_BASE      0x400B8000UL
#define XIP_CTRL_BASE    0x400C0000UL
#define QMI_BASE         0x400D0000UL
#define WATCHDOG_BASE    0x400D8000UL
#define DMA_BASE         0x50000000UL
#define USBCTRL_BASE     0x50110000UL
#define PIO0_BASE        0x50200000UL
#define PIO1_BASE        0x50300000UL
#define PIO2_BASE        0x50400000UL
#define SIO_BASE         0xD0000000UL

#define REG_RW(addr) (*(io_rw_32 *)(uintptr_t)(addr))
#define REG_RO(addr) (*(io_ro_32 *)(uintptr_t)(addr))

#define REG_XOR(addr, val) (REG_RW((addr) + 0x1000UL) = (val))
#define REG_SET(addr, val) (REG_RW((addr) + 0x2000UL) = (val))
#define REG_CLR(addr, val) (REG_RW((addr) + 0x3000UL) = (val))

static inline void __dsb(void) { __asm__ volatile("dsb" ::: "memory"); }
static inline void __dmb(void) { __asm__ volatile("dmb" ::: "memory"); }
static inline void __isb(void) { __asm__ volatile("isb" ::: "memory"); }

#define ATTR_RAMFUNC __attribute__((section(".ramfunc")))

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define MIN(a,b) (((a) < (b)) ? (a) : (b))
#define MAX(a,b) (((a) > (b)) ? (a) : (b))
#define UNUSED(x) ((void)(x))


#endif
