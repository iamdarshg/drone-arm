/*
 * SIO (Single-cycle IO) hardware structs
 * Minimal version for fast GPIO operations
 */
#ifndef _HARDWARE_STRUCTS_SIO_H
#define _HARDWARE_STRUCTS_SIO_H

#include "../address_mapped.h"

// SIO register offsets
#define SIO_GPIO_IN_OFFSET          _u(0x00000004)
#define SIO_GPIO_HI_IN_OFFSET       _u(0x00000008)
#define SIO_GPIO_OUT_OFFSET         _u(0x00000010)
#define SIO_GPIO_OUT_SET_OFFSET     _u(0x00000014)
#define SIO_GPIO_OUT_CLR_OFFSET     _u(0x00000018)
#define SIO_GPIO_OUT_XOR_OFFSET     _u(0x0000001c)
#define SIO_GPIO_OE_OFFSET          _u(0x00000020)
#define SIO_GPIO_OE_SET_OFFSET      _u(0x00000024)
#define SIO_GPIO_OE_CLR_OFFSET      _u(0x00000028)
#define SIO_GPIO_OE_XOR_OFFSET      _u(0x0000002c)
#define SIO_GPIO_HI_OUT_OFFSET      _u(0x00000030)
#define SIO_GPIO_HI_OUT_SET_OFFSET  _u(0x00000034)
#define SIO_GPIO_HI_OUT_CLR_OFFSET  _u(0x00000038)
#define SIO_GPIO_HI_OUT_XOR_OFFSET  _u(0x0000003c)
#define SIO_GPIO_HI_OE_OFFSET       _u(0x00000040)
#define SIO_GPIO_HI_OE_SET_OFFSET   _u(0x00000044)
#define SIO_GPIO_HI_OE_CLR_OFFSET   _u(0x00000048)
#define SIO_GPIO_HI_OE_XOR_OFFSET   _u(0x0000004c)

// SIO hardware register block
typedef struct {
    io_rw_32 cpuid;
    io_ro_32 gpio_in;
    io_ro_32 gpio_hi_in;
    uint32_t _pad0;
    io_rw_32 gpio_out;
    io_rw_32 gpio_out_set;
    io_rw_32 gpio_out_clr;
    io_rw_32 gpio_out_xor;
    io_rw_32 gpio_oe;
    io_rw_32 gpio_oe_set;
    io_rw_32 gpio_oe_clr;
    io_rw_32 gpio_oe_xor;
    io_rw_32 gpio_hi_out;
    io_rw_32 gpio_hi_out_set;
    io_rw_32 gpio_hi_out_clr;
    io_rw_32 gpio_hi_out_xor;
    io_rw_32 gpio_hi_oe;
    io_rw_32 gpio_hi_oe_set;
    io_rw_32 gpio_hi_oe_clr;
    io_rw_32 gpio_hi_oe_xor;
} sio_hw_t;

#define sio_hw ((sio_hw_t *const)SIO_BASE)

#endif // _HARDWARE_STRUCTS_SIO_H
