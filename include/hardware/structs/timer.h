/*
 * Timer hardware structs
 */
#ifndef _HARDWARE_STRUCTS_TIMER_H
#define _HARDWARE_STRUCTS_TIMER_H

#include "../address_mapped.h"
#include "../regs/addressmap.h"

// Timer register offsets
#define TIMER_TIMEHW_OFFSET     0x00
#define TIMER_TIMELW_OFFSET     0x04
#define TIMER_TIMEHR_OFFSET     0x08
#define TIMER_TIMELR_OFFSET     0x0c
#define TIMER_ALARM0_OFFSET     0x10
#define TIMER_ALARM1_OFFSET     0x14
#define TIMER_ALARM2_OFFSET     0x18
#define TIMER_ALARM3_OFFSET     0x1c
#define TIMER_ARMED_OFFSET      0x20
#define TIMER_TIMERAWH_OFFSET   0x24
#define TIMER_TIMERAWL_OFFSET   0x28
#define TIMER_DBGPAUSE_OFFSET   0x2c
#define TIMER_PAUSE_OFFSET      0x30
#define TIMER_LOCKED_OFFSET     0x34
#define TIMER_SOURCE_OFFSET     0x38
#define TIMER_INTR_OFFSET       0x3c
#define TIMER_INTE_OFFSET       0x40
#define TIMER_INTF_OFFSET       0x44
#define TIMER_INTS_OFFSET       0x48

// Timer hardware register block
typedef struct {
    io_rw_32 timehw;
    io_rw_32 timelw;
    io_ro_32 timehr;
    io_ro_32 timelr;
    io_rw_32 alarm[4];
    io_rw_32 armed;
    io_ro_32 timerawh;
    io_ro_32 timerawl;
    io_rw_32 dbgpause;
    io_rw_32 pause;
    io_rw_32 locked;
    io_rw_32 source;
    io_ro_32 intr;
    io_rw_32 inte;
    io_rw_32 intf;
    io_ro_32 ints;
} timer_hw_t;

#define timer0_hw ((timer_hw_t *const)TIMER0_BASE)
#define timer1_hw ((timer_hw_t *const)TIMER1_BASE)

// Use timer0 as the default timer
#define timer_hw timer0_hw

// INTE/INTF/INTS bits (one per alarm)
#define TIMER_ALARM0_BIT        (1u << 0)
#define TIMER_ALARM1_BIT        (1u << 1)
#define TIMER_ALARM2_BIT        (1u << 2)
#define TIMER_ALARM3_BIT        (1u << 3)

#endif // _HARDWARE_STRUCTS_TIMER_H
