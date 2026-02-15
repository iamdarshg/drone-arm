/*
 * PADS Bank0 hardware structs for GPIO pad configuration
 */
#ifndef _HARDWARE_STRUCTS_PADS_BANK0_H
#define _HARDWARE_STRUCTS_PADS_BANK0_H

#include "../address_mapped.h"
#include "../regs/addressmap.h"

// Voltage select (0=3.3V, 1=1.8V)
typedef enum {
    PADS_VOLTAGE_3V3 = 0,
    PADS_VOLTAGE_1V8 = 1,
} pads_voltage_t;

// Pad control register for each GPIO
typedef struct {
    io_rw_32 voltage_select;
    io_rw_32 gpio[48];
} pads_bank0_hw_t;

#define pads_bank0_hw ((pads_bank0_hw_t *const)PADS_BANK0_BASE)

// Pad register bit definitions
#define PADS_BANK0_GPIO0_SLEWFAST_BIT       (1u << 0)
#define PADS_BANK0_GPIO0_SCHMITT_BIT        (1u << 1)
#define PADS_BANK0_GPIO0_PDE_BIT            (1u << 2)  // Pull-down enable
#define PADS_BANK0_GPIO0_PUE_BIT            (1u << 3)  // Pull-up enable
#define PADS_BANK0_GPIO0_DRIVE_LSB          4
#define PADS_BANK0_GPIO0_DRIVE_BITS         0x00000030
#define PADS_BANK0_GPIO0_IE_BIT             (1u << 6)  // Input enable
#define PADS_BANK0_GPIO0_OD_BIT             (1u << 7)  // Output disable

// Drive strength values (2mA, 4mA, 8mA, 12mA)
#define PADS_DRIVE_2MA  0
#define PADS_DRIVE_4MA  1
#define PADS_DRIVE_8MA  2
#define PADS_DRIVE_12MA 3

#endif // _HARDWARE_STRUCTS_PADS_BANK0_H
