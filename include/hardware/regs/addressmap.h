/*
 * Memory address map for RP2350
 * Minimal version based on hardware/regs/addressmap.h
 */
#ifndef _ADDRESSMAP_H
#define _ADDRESSMAP_H

#include "../platform_defs.h"

// Atomic alias offsets
#define REG_ALIAS_RW_BITS  (_u(0x0) << _u(12))
#define REG_ALIAS_XOR_BITS (_u(0x1) << _u(12))
#define REG_ALIAS_SET_BITS (_u(0x2) << _u(12))
#define REG_ALIAS_CLR_BITS (_u(0x3) << _u(12))

// Base addresses
#define ROM_BASE                    _u(0x00000000)
#define XIP_BASE                    _u(0x10000000)
#define XIP_SRAM_BASE               _u(0x13ffc000)
#define XIP_NOCACHE_NOALLOC_BASE    _u(0x14000000)
#define SRAM_BASE                   _u(0x20000000)
#define SRAM_SCRATCH_X_BASE         _u(0x20080000)
#define SRAM_SCRATCH_Y_BASE         _u(0x20081000)
#define SRAM_END                    _u(0x20082000)

// Peripheral base addresses
#define SYSINFO_BASE                _u(0x40000000)
#define SYSCFG_BASE                 _u(0x40008000)
#define CLOCKS_BASE                 _u(0x40010000)
#define PSM_BASE                    _u(0x40018000)
#define RESETS_BASE                 _u(0x40020000)
#define IO_BANK0_BASE               _u(0x40028000)
#define IO_QSPI_BASE                _u(0x40030000)
#define PADS_BANK0_BASE             _u(0x40038000)
#define PADS_QSPI_BASE              _u(0x40040000)
#define XOSC_BASE                   _u(0x40048000)
#define PLL_SYS_BASE                _u(0x40050000)
#define PLL_USB_BASE                _u(0x40058000)
#define UART0_BASE                  _u(0x40070000)
#define UART1_BASE                  _u(0x40078000)
#define SPI0_BASE                   _u(0x40080000)
#define SPI1_BASE                   _u(0x40088000)
#define I2C0_BASE                   _u(0x40090000)
#define I2C1_BASE                   _u(0x40098000)
#define ADC_BASE                    _u(0x400a0000)
#define PWM_BASE                    _u(0x400a8000)
#define TIMER0_BASE                 _u(0x400b0000)
#define TIMER1_BASE                 _u(0x400b8000)
#define HSTX_CTRL_BASE              _u(0x400c0000)
#define XIP_CTRL_BASE               _u(0x400c8000)
#define XIP_QMI_BASE                _u(0x400d0000)
#define WATCHDOG_BASE               _u(0x400d8000)
#define BOOTRAM_BASE                _u(0x400e0000)
#define ROSC_BASE                   _u(0x400e8000)
#define DMA_BASE                    _u(0x50000000)
#define USBCTRL_BASE                _u(0x50100000)
#define USBCTRL_DPRAM_BASE          _u(0x50100000)
#define USBCTRL_REGS_BASE           _u(0x50110000)
#define PIO0_BASE                   _u(0x50200000)
#define PIO1_BASE                   _u(0x50300000)
#define PIO2_BASE                   _u(0x50400000)

// SIO (Single-cycle IO) base
#define SIO_BASE                    _u(0xd0000000)

#endif // _ADDRESSMAP_H
