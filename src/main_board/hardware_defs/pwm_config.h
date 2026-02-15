/**
 * PWM Configuration for Main Controller Board (RP2350B)
 * 
 * This header provides easy configuration for:
 * - PWM frequency settings
 * - Active slice selection
 * - GPIO pin mapping
 * 
 * Usage:
 *   1. Uncomment PWM_SLICE_x_ENABLED to enable specific slices
 *   2. Set PWM_FREQ_HZ for desired frequency (default: 20 kHz)
 *   3. Configure GPIO pins for each enabled slice
 */

#ifndef PWM_CONFIG_H
#define PWM_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

//============================================================================
// USER CONFIGURATION - EDIT THESE VALUES
//============================================================================

// System clock frequency (RP2350B can run at 150 MHz)
#ifndef SYSCLK_FREQ_HZ
#define SYSCLK_FREQ_HZ 150000000u
#endif

// PWM base frequency (default: 20 kHz for motor control)
// Formula: freq = SYSCLK / ((TOP + 1) * DIV)
#ifndef PWM_FREQ_HZ
#define PWM_FREQ_HZ 20000u
#endif

// Enable phase-correct mode? (better for motor control, slightly lower freq)
// 0 = fast PWM (asymmetric), 1 = phase-correct (symmetric)
#ifndef PWM_PHASE_CORRECT
#define PWM_PHASE_CORRECT 0
#endif

//============================================================================
// SLICE CONFIGURATION
// Set to 1 to enable slice, 0 to disable
//============================================================================

#define PWM_SLICE_0_ENABLED  0
#define PWM_SLICE_0_GPIO_A   0
#define PWM_SLICE_0_GPIO_B   1

#define PWM_SLICE_1_ENABLED  0
#define PWM_SLICE_1_GPIO_A   2
#define PWM_SLICE_1_GPIO_B   3

#define PWM_SLICE_2_ENABLED  0
#define PWM_SLICE_2_GPIO_A   4
#define PWM_SLICE_2_GPIO_B   5

#define PWM_SLICE_3_ENABLED  0
#define PWM_SLICE_3_GPIO_A   6
#define PWM_SLICE_3_GPIO_B   7

#define PWM_SLICE_4_ENABLED  0
#define PWM_SLICE_4_GPIO_A   8
#define PWM_SLICE_4_GPIO_B   9

#define PWM_SLICE_5_ENABLED  0
#define PWM_SLICE_5_GPIO_A   10
#define PWM_SLICE_5_GPIO_B   11

#define PWM_SLICE_6_ENABLED  0
#define PWM_SLICE_6_GPIO_A   12
#define PWM_SLICE_6_GPIO_B   13

#define PWM_SLICE_7_ENABLED  0
#define PWM_SLICE_7_GPIO_A   14
#define PWM_SLICE_7_GPIO_B   15

// RP2350B has 4 additional slices (8-11)
#define PWM_SLICE_8_ENABLED  0
#define PWM_SLICE_8_GPIO_A   16
#define PWM_SLICE_8_GPIO_B   17

#define PWM_SLICE_9_ENABLED  0
#define PWM_SLICE_9_GPIO_A   18
#define PWM_SLICE_9_GPIO_B   19

#define PWM_SLICE_10_ENABLED 0
#define PWM_SLICE_10_GPIO_A  20
#define PWM_SLICE_10_GPIO_B  21

#define PWM_SLICE_11_ENABLED 0
#define PWM_SLICE_11_GPIO_A  22
#define PWM_SLICE_11_GPIO_B  23

//============================================================================
// AUTOMATIC CALCULATIONS - DO NOT EDIT BELOW
//============================================================================

// Number of slices available (12 for RP2350B, 8 for RP2040)
#define PWM_NUM_SLICES 12
#define PWM_NUM_CHANNELS (PWM_NUM_SLICES * 2)

// Build active slice mask
#define PWM_ACTIVE_SLICE_MASK ( \
    (PWM_SLICE_0_ENABLED  << 0)  | \
    (PWM_SLICE_1_ENABLED  << 1)  | \
    (PWM_SLICE_2_ENABLED  << 2)  | \
    (PWM_SLICE_3_ENABLED  << 3)  | \
    (PWM_SLICE_4_ENABLED  << 4)  | \
    (PWM_SLICE_5_ENABLED  << 5)  | \
    (PWM_SLICE_6_ENABLED  << 6)  | \
    (PWM_SLICE_7_ENABLED  << 7)  | \
    (PWM_SLICE_8_ENABLED  << 8)  | \
    (PWM_SLICE_9_ENABLED  << 9)  | \
    (PWM_SLICE_10_ENABLED << 10) | \
    (PWM_SLICE_11_ENABLED << 11) )

// Calculate divider and TOP values for requested frequency
#if PWM_PHASE_CORRECT
    // Phase-correct mode: counts up then down, so effective period is 2*TOP
    #define PWM_CYCLES_PER_PERIOD ((SYSCLK_FREQ_HZ + (PWM_FREQ_HZ / 2)) / PWM_FREQ_HZ / 2)
#else
    // Fast PWM mode: counts up only
    #define PWM_CYCLES_PER_PERIOD ((SYSCLK_FREQ_HZ + (PWM_FREQ_HZ / 2)) / PWM_FREQ_HZ)
#endif

// Determine if we need a divider or can run at DIV=1
#if PWM_CYCLES_PER_PERIOD <= 65536
    // Can achieve with DIV=1
    #define PWM_DIV_INT 1
    #define PWM_DIV_FRAC 0
    #define PWM_TOP_VALUE (PWM_CYCLES_PER_PERIOD - 1)
#else
    // Need divider
    #define PWM_DIV_INT ((PWM_CYCLES_PER_PERIOD + 32767) / 65536)
    #if PWM_DIV_INT > 255
        #error "Requested PWM frequency too low for current system clock"
    #endif
    #define PWM_DIV_FRAC 0
    #define PWM_TOP_VALUE ((PWM_CYCLES_PER_PERIOD / PWM_DIV_INT) - 1)
#endif

// Validate TOP value
#if PWM_TOP_VALUE > 65535
    #error "Calculated TOP value exceeds 16-bit maximum"
#endif

// Actual achieved frequency
#define PWM_ACTUAL_FREQ_HZ (SYSCLK_FREQ_HZ / ((PWM_TOP_VALUE + 1) * PWM_DIV_INT * (PWM_PHASE_CORRECT ? 2 : 1)))

//============================================================================
// GPIO PIN CONFIGURATION ARRAYS
//============================================================================

// Array of GPIO pins for channel A of each slice
static const uint8_t pwm_gpio_a[PWM_NUM_SLICES] = {
    PWM_SLICE_0_GPIO_A,
    PWM_SLICE_1_GPIO_A,
    PWM_SLICE_2_GPIO_A,
    PWM_SLICE_3_GPIO_A,
    PWM_SLICE_4_GPIO_A,
    PWM_SLICE_5_GPIO_A,
    PWM_SLICE_6_GPIO_A,
    PWM_SLICE_7_GPIO_A,
    PWM_SLICE_8_GPIO_A,
    PWM_SLICE_9_GPIO_A,
    PWM_SLICE_10_GPIO_A,
    PWM_SLICE_11_GPIO_A,
};

// Array of GPIO pins for channel B of each slice
static const uint8_t pwm_gpio_b[PWM_NUM_SLICES] = {
    PWM_SLICE_0_GPIO_B,
    PWM_SLICE_1_GPIO_B,
    PWM_SLICE_2_GPIO_B,
    PWM_SLICE_3_GPIO_B,
    PWM_SLICE_4_GPIO_B,
    PWM_SLICE_5_GPIO_B,
    PWM_SLICE_6_GPIO_B,
    PWM_SLICE_7_GPIO_B,
    PWM_SLICE_8_GPIO_B,
    PWM_SLICE_9_GPIO_B,
    PWM_SLICE_10_GPIO_B,
    PWM_SLICE_11_GPIO_B,
};

// Array of enabled flags for each slice
static const bool pwm_slice_enabled[PWM_NUM_SLICES] = {
    PWM_SLICE_0_ENABLED != 0,
    PWM_SLICE_1_ENABLED != 0,
    PWM_SLICE_2_ENABLED != 0,
    PWM_SLICE_3_ENABLED != 0,
    PWM_SLICE_4_ENABLED != 0,
    PWM_SLICE_5_ENABLED != 0,
    PWM_SLICE_6_ENABLED != 0,
    PWM_SLICE_7_ENABLED != 0,
    PWM_SLICE_8_ENABLED != 0,
    PWM_SLICE_9_ENABLED != 0,
    PWM_SLICE_10_ENABLED != 0,
    PWM_SLICE_11_ENABLED != 0,
};

//============================================================================
// HELPER MACROS
//============================================================================

// Channel numbers for direct reference
#define PWM_CHAN_0A  0
#define PWM_CHAN_0B  1
#define PWM_CHAN_1A  2
#define PWM_CHAN_1B  3
#define PWM_CHAN_2A  4
#define PWM_CHAN_2B  5
#define PWM_CHAN_3A  6
#define PWM_CHAN_3B  7
#define PWM_CHAN_4A  8
#define PWM_CHAN_4B  9
#define PWM_CHAN_5A  10
#define PWM_CHAN_5B  11
#define PWM_CHAN_6A  12
#define PWM_CHAN_6B  13
#define PWM_CHAN_7A  14
#define PWM_CHAN_7B  15
#define PWM_CHAN_8A  16
#define PWM_CHAN_8B  17
#define PWM_CHAN_9A  18
#define PWM_CHAN_9B  19
#define PWM_CHAN_10A 20
#define PWM_CHAN_10B 21
#define PWM_CHAN_11A 22
#define PWM_CHAN_11B 23

// Convert channel to slice number
#define PWM_CHANNEL_SLICE(ch) ((ch) >> 1)

// Convert channel to A/B selection (true = A, false = B)
#define PWM_CHANNEL_IS_A(ch) (((ch) & 1) == 0)

// Check if a slice is enabled in configuration
#define PWM_SLICE_IS_ENABLED(slice) ((PWM_ACTIVE_SLICE_MASK & (1u << (slice))) != 0)

#endif // PWM_CONFIG_H
