/*
 * ADC hardware structs
 */
#ifndef _HARDWARE_STRUCTS_ADC_H
#define _HARDWARE_STRUCTS_ADC_H

#include "../address_mapped.h"
#include "../regs/addressmap.h"

// ADC register offsets
#define ADC_CS_OFFSET           0x00
#define ADC_RESULT_OFFSET       0x04
#define ADC_FCS_OFFSET          0x08
#define ADC_FIFO_OFFSET         0x0c
#define ADC_DIV_OFFSET          0x10
#define ADC_INTR_OFFSET         0x14
#define ADC_INTE_OFFSET         0x18
#define ADC_INTF_OFFSET         0x1c
#define ADC_INTS_OFFSET         0x20

// ADC hardware register block
typedef struct {
    io_rw_32 cs;
    io_ro_32 result;
    io_rw_32 fcs;
    io_ro_32 fifo;
    io_rw_32 div;
    io_ro_32 intr;
    io_rw_32 inte;
    io_rw_32 intf;
    io_ro_32 ints;
} adc_hw_t;

#define adc_hw ((adc_hw_t *const)ADC_BASE)

// CS bits
#define ADC_CS_EN_BIT           (1u << 0)
#define ADC_CS_TS_EN_BIT        (1u << 1)  // Temperature sensor enable
#define ADC_CS_START_MANY_BIT   (1u << 3)
#define ADC_CS_START_ONCE_BIT   (1u << 2)
#define ADC_CS_READY_BIT        (1u << 8)
#define ADC_CS_ERR_BIT          (1u << 9)
#define ADC_CS_AINSEL_LSB       12
#define ADC_CS_AINSEL_BITS      0x00007000
#define ADC_CS_RROBIN_LSB       16
#define ADC_CS_RROBIN_BITS      0x00ff0000

// FCS bits
#define ADC_FCS_EN_BIT          (1u << 0)
#define ADC_FCS_SHIFT_LSB       1
#define ADC_FCS_SHIFT_BITS      0x0000001e
#define ADC_FCS_THRESH_LSB      16
#define ADC_FCS_THRESH_BITS     0x000f0000
#define ADC_FCS_LEVEL_LSB       24
#define ADC_FCS_LEVEL_BITS      0x0f000000
#define ADC_FCS_OVER_BIT        (1u << 29)
#define ADC_FCS_UNDER_BIT       (1u << 30)

#endif // _HARDWARE_STRUCTS_ADC_H
