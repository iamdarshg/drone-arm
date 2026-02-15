/*
 * Minimal address mapping macros
 * Based on hardware/address_mapped.h
 */
#ifndef _ADDRESS_MAPPED_H
#define _ADDRESS_MAPPED_H

#include "platform_defs.h"

// Volatile register types
typedef volatile uint32_t io_rw_32;
typedef volatile uint16_t io_rw_16;
typedef volatile uint8_t  io_rw_8;

typedef const volatile uint32_t io_ro_32;
typedef const volatile uint16_t io_ro_16;
typedef const volatile uint8_t  io_ro_8;

// Atomic alias offsets
#define REG_ALIAS_RW_BITS  (_u(0x0) << _u(12))
#define REG_ALIAS_XOR_BITS (_u(0x1) << _u(12))
#define REG_ALIAS_SET_BITS (_u(0x2) << _u(12))
#define REG_ALIAS_CLR_BITS (_u(0x3) << _u(12))

// Register access helper
#define REG32_WRITE(addr, val) (*(volatile uint32_t *)(addr) = (val))
#define REG32_READ(addr) (*(volatile uint32_t *)(addr))

#endif // _ADDRESS_MAPPED_H
