/*
 * PWM hardware structs for RP2350B
 * Optimized for maximum speed and minimal overhead
 */
#ifndef _HARDWARE_STRUCTS_PWM_H
#define _HARDWARE_STRUCTS_PWM_H

#include "../address_mapped.h"
#include "../regs/addressmap.h"

// PWM channel registers (12 slices for RP2350B, 2 channels each)
typedef struct {
    io_rw_32 csr;   // Control and status
    io_rw_32 div;   // Clock divider (8.4 fixed point)
    io_rw_32 ctr;   // Counter value
    io_rw_32 cc;    // Compare values (A and B)
    io_rw_32 top;   // Counter wrap value
} pwm_slice_t;

// PWM hardware register block for RP2350B
typedef struct {
    pwm_slice_t slice[12];  // 12 slices on RP2350B (8 on RP2040)
    io_rw_32 en;            // Global enable register
    io_rw_32 intr;          // Raw interrupts
    io_rw_32 inte;          // Interrupt enable
    io_rw_32 intf;          // Interrupt force
    io_ro_32 ints;          // Interrupt status
    io_rw_32 inte1;         // Interrupt enable 1 (RP2350)
    io_rw_32 intf1;         // Interrupt force 1 (RP2350)
    io_ro_32 ints1;         // Interrupt status 1 (RP2350)
} pwm_hw_t;

#define pwm_hw ((pwm_hw_t *const)PWM_BASE)

// CSR register bits
#define PWM_CSR_EN_BIT              (1u << 0)
#define PWM_CSR_PH_CORRECT_BIT      (1u << 1)
#define PWM_CSR_A_INV_BIT           (1u << 2)
#define PWM_CSR_B_INV_BIT           (1u << 3)
#define PWM_CSR_DIVMODE_LSB         4
#define PWM_CSR_DIVMODE_BITS        (3u << 4)
#define PWM_CSR_DIVMODE_FREE_RUN    (0u << 4)
#define PWM_CSR_DIVMODE_HIGH        (1u << 4)
#define PWM_CSR_DIVMODE_RISING      (2u << 4)
#define PWM_CSR_DIVMODE_FALLING     (3u << 4)
#define PWM_CSR_PH_RET_BIT          (1u << 6)
#define PWM_CSR_PH_ADV_BIT          (1u << 7)

// DIV register (8.4 fixed point format)
#define PWM_DIV_INT_LSB             4
#define PWM_DIV_INT_MASK            (0xFFu << 4)
#define PWM_DIV_FRAC_MASK           0x0Fu

// CC register channel positions
#define PWM_CC_A_LSB                0
#define PWM_CC_B_LSB                16
#define PWM_CC_A_MASK               0x0000FFFFu
#define PWM_CC_B_MASK               0xFFFF0000u

// Channel enable bits
#define PWM_EN_CH0_BIT              (1u << 0)
#define PWM_EN_CH1_BIT              (1u << 1)
#define PWM_EN_CH2_BIT              (1u << 2)
#define PWM_EN_CH3_BIT              (1u << 3)
#define PWM_EN_CH4_BIT              (1u << 4)
#define PWM_EN_CH5_BIT              (1u << 5)
#define PWM_EN_CH6_BIT              (1u << 6)
#define PWM_EN_CH7_BIT              (1u << 7)
#define PWM_EN_CH8_BIT              (1u << 8)
#define PWM_EN_CH9_BIT              (1u << 9)
#define PWM_EN_CH10_BIT             (1u << 10)
#define PWM_EN_CH11_BIT             (1u << 11)

// Maximum TOP value (16-bit counter)
#define PWM_TOP_MAX                 0xFFFFu

// Fastest configuration: TOP=1, DIV=1 gives 75MHz @ 150MHz sysclk
#define PWM_FASTEST_TOP             1u
#define PWM_FASTEST_DIV             ((1u << PWM_DIV_INT_LSB) | 0u)

#endif // _HARDWARE_STRUCTS_PWM_H
