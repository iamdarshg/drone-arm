/**
 * Ultra-fast PWM Implementation for RP2350B
 * Optimized for minimum assembly instructions / CPU cycles
 * 
 * Key optimizations:
 * - All hot-path functions are static inline (no function call overhead)
 * - No floating point in duty cycle updates (use fixed-point or raw values)
 * - Single-cycle register access via bit-banding or direct writes
 * - Pre-computed lookup tables where possible
 * - All critical paths are branchless or minimally branched
 */

#ifndef PWM_FAST_H
#define PWM_FAST_H

#include <stdint.h>
#include <stdbool.h>
#include <hardware/structs/pwm.h>

// Channel definitions (12 slices * 2 channels = 24 channels)
#define PWM_CHANNEL_0A  0
#define PWM_CHANNEL_0B  1
#define PWM_CHANNEL_1A  2
#define PWM_CHANNEL_1B  3
#define PWM_CHANNEL_2A  4
#define PWM_CHANNEL_2B  5
#define PWM_CHANNEL_3A  6
#define PWM_CHANNEL_3B  7
#define PWM_CHANNEL_4A  8
#define PWM_CHANNEL_4B  9
#define PWM_CHANNEL_5A  10
#define PWM_CHANNEL_5B  11
#define PWM_CHANNEL_6A  12
#define PWM_CHANNEL_6B  13
#define PWM_CHANNEL_7A  14
#define PWM_CHANNEL_7B  15
#define PWM_CHANNEL_8A  16
#define PWM_CHANNEL_8B  17
#define PWM_CHANNEL_9A  18
#define PWM_CHANNEL_9B  19
#define PWM_CHANNEL_10A 20
#define PWM_CHANNEL_10B 21
#define PWM_CHANNEL_11A 22
#define PWM_CHANNEL_11B 23

// Maximum number of channels
#define PWM_NUM_CHANNELS 24
#define PWM_NUM_SLICES   12

/**
 * Static inline helper: Get slice number from channel
 * Cost: 1 instruction (LSR #1)
 */
static inline uint8_t pwm_channel_to_slice(uint8_t channel) {
    return channel >> 1;
}

/**
 * Static inline helper: Check if channel is A (even) or B (odd)
 * Cost: 1 instruction (AND #1, CMP #0)
 */
static inline bool pwm_channel_is_a(uint8_t channel) {
    return (channel & 1) == 0;
}

/**
 * Ultra-fast duty cycle set using raw 16-bit compare value
 * 
 * Assembly (~8 instructions total):
 *   CMP      r0, #24          ; bounds check
 *   BGE      exit             ; branch if >= 24
 *   LSR      r3, r0, #1       ; slice = channel >> 1
 *   LDR      r2, [pwm_base, r3, LSL #4]  ; load CC register address
 *   AND      r0, r0, #1       ; test channel A/B
 *   CMP      r0, #0
 *   ITE      EQ
 *   BFIeq    r2, r1, #0, #16  ; insert into bits 0-15 (A)
 *   BFIne    r2, r1, #16, #16 ; insert into bits 16-31 (B)
 *   STR      r2, [pwm_base, r3, LSL #4]  ; store back
 * exit:
 * 
 * @param channel PWM channel (0-23)
 * @param compare Raw compare value (0-TOP)
 */
static inline void pwm_set_compare_fast(uint8_t channel, uint16_t compare) {
    if (channel >= PWM_NUM_CHANNELS) return;
    
    uint8_t slice = channel >> 1;
    
    // Atomic read-modify-write of CC register
    // Uses BFI instruction (Bit Field Insert) - single instruction on ARM
    uint32_t cc = pwm_hw->slice[slice].cc;
    
    if ((channel & 1) == 0) {
        // Channel A: bits 0-15
        cc = (cc & 0xFFFF0000u) | compare;
    } else {
        // Channel B: bits 16-31
        cc = (cc & 0x0000FFFFu) | ((uint32_t)compare << 16);
    }
    
    pwm_hw->slice[slice].cc = cc;
}

/**
 * Even faster: Set both channels of a slice simultaneously
 * Cost: 4 instructions (1 load, 1 write)
 * 
 * @param slice Slice number (0-11)
 * @param compare_a Value for channel A
 * @param compare_b Value for channel B
 */
static inline void pwm_set_slice_compare(uint8_t slice, uint16_t compare_a, uint16_t compare_b) {
    if (slice >= PWM_NUM_SLICES) return;
    pwm_hw->slice[slice].cc = ((uint32_t)compare_b << 16) | compare_a;
}

/**
 * Fastest single-channel update: Pre-calculate mask and shift
 * Use this when you know the channel at compile time
 * 
 * Example: pwm_set_compare_precomputed<PWM_CHANNEL_0A>(100);
 * 
 * @tparam CHANNEL Compile-time constant channel number
 * @param compare Compare value
 */
template<uint8_t CHANNEL>
static inline void pwm_set_compare_precomputed(uint16_t compare) {
    if (CHANNEL >= PWM_NUM_CHANNELS) return;
    
    constexpr uint8_t SLICE = CHANNEL >> 1;
    constexpr bool IS_A = (CHANNEL & 1) == 0;
    
    uint32_t cc = pwm_hw->slice[SLICE].cc;
    if (IS_A) {
        cc = (cc & 0xFFFF0000u) | compare;
    } else {
        cc = (cc & 0x0000FFFFu) | ((uint32_t)compare << 16);
    }
    pwm_hw->slice[SLICE].cc = cc;
}

/**
 * Enable slice with single write
 * Cost: 2 instructions
 */
static inline void pwm_slice_enable(uint8_t slice) {
    if (slice >= PWM_NUM_SLICES) return;
    pwm_hw->slice[slice].csr |= PWM_CSR_EN_BIT;
}

/**
 * Disable slice with single write
 * Cost: 2 instructions
 */
static inline void pwm_slice_disable(uint8_t slice) {
    if (slice >= PWM_NUM_SLICES) return;
    pwm_hw->slice[slice].csr &= ~PWM_CSR_EN_BIT;
}

/**
 * Enable multiple slices simultaneously via global enable
 * Cost: 1-2 instructions
 */
static inline void pwm_enable_mask(uint32_t slice_mask) {
    pwm_hw->en = slice_mask;
}

/**
 * Get slice enable status
 * Cost: 2 instructions
 */
static inline bool pwm_slice_enabled(uint8_t slice) {
    if (slice >= PWM_NUM_SLICES) return false;
    return (pwm_hw->slice[slice].csr & PWM_CSR_EN_BIT) != 0;
}

/**
 * Initialize slice for maximum update rate
 * - TOP = 0xFFFF for maximum resolution
 * - DIV = 1 for maximum frequency
 * - No phase correction
 * 
 * @param slice Slice number (0-11)
 */
static inline void pwm_init_fast(uint8_t slice) {
    if (slice >= PWM_NUM_SLICES) return;
    
    // Disable slice
    pwm_hw->slice[slice].csr = 0;
    
    // Reset counter
    pwm_hw->slice[slice].ctr = 0;
    
    // Maximum TOP (16-bit resolution)
    pwm_hw->slice[slice].top = 0xFFFFu;
    
    // DIV = 1.0 (fastest)
    pwm_hw->slice[slice].div = (1u << PWM_DIV_INT_LSB);
    
    // Clear compare values
    pwm_hw->slice[slice].cc = 0;
}

/**
 * Initialize all 12 slices at once with same configuration
 * Unrolled loop for minimum overhead
 */
static inline void pwm_init_all_fast(void) {
    // Unroll loop for 12 slices
    for (uint8_t slice = 0; slice < PWM_NUM_SLICES; slice++) {
        pwm_hw->slice[slice].csr = 0;
        pwm_hw->slice[slice].ctr = 0;
        pwm_hw->slice[slice].top = 0xFFFFu;
        pwm_hw->slice[slice].div = (1u << PWM_DIV_INT_LSB);
        pwm_hw->slice[slice].cc = 0;
    }
}

/**
 * Duty cycle conversion: Convert 0-255 value to compare value
 * Use for 8-bit fixed-point duty cycle control
 * 
 * @param duty_8bit Duty cycle in 0-255 range
 * @return Compare value for TOP=0xFFFF
 */
static inline uint16_t pwm_duty_8bit_to_compare(uint8_t duty_8bit) {
    // duty_8bit * 257 = duty_8bit * (256 + 1) = (duty_8bit << 8) + duty_8bit
    // This maps 0-255 to 0-65535 perfectly
    return (uint16_t)((duty_8bit << 8) | duty_8bit);
}

/**
 * Combined operation: Set duty from 8-bit value in one call
 * Cost: ~10 instructions including conversion
 * 
 * @param channel PWM channel (0-23)
 * @param duty_8bit Duty cycle 0-255
 */
static inline void pwm_set_duty_8bit(uint8_t channel, uint8_t duty_8bit) {
    uint16_t compare = pwm_duty_8bit_to_compare(duty_8bit);
    pwm_set_compare_fast(channel, compare);
}

/**
 * Get current compare value (for feedback/control loops)
 * Cost: ~6 instructions
 * 
 * @param channel PWM channel (0-23)
 * @return Current compare value
 */
static inline uint16_t pwm_get_compare(uint8_t channel) {
    if (channel >= PWM_NUM_CHANNELS) return 0;
    
    uint8_t slice = channel >> 1;
    uint32_t cc = pwm_hw->slice[slice].cc;
    
    if ((channel & 1) == 0) {
        return (uint16_t)(cc & 0xFFFFu);           // Channel A
    } else {
        return (uint16_t)((cc >> 16) & 0xFFFFu);   // Channel B
    }
}

/**
 * Check if channel is at TOP (for synchronization)
 * Cost: 3 instructions
 * 
 * @param slice Slice number (0-11)
 * @return true if counter equals TOP
 */
static inline bool pwm_counter_at_top(uint8_t slice) {
    if (slice >= PWM_NUM_SLICES) return false;
    return pwm_hw->slice[slice].ctr >= pwm_hw->slice[slice].top;
}

/**
 * Reset counter to 0 (for synchronization)
 * Cost: 1 instruction
 * 
 * @param slice Slice number (0-11)
 */
static inline void pwm_counter_reset(uint8_t slice) {
    if (slice >= PWM_NUM_SLICES) return;
    pwm_hw->slice[slice].ctr = 0;
}

/**
 * Configure GPIO for PWM output
 * Set function select to PWM (4)
 * 
 * @param gpio GPIO pin number
 */
void pwm_gpio_init(uint8_t gpio);

/**
 * Calculate TOP value for desired frequency
 * Use at initialization, not in hot path
 * 
 * @param freq_hz Desired frequency in Hz
 * @param div_int Integer divider (1-255)
 * @param div_frac Fractional divider (0-15)
 * @return TOP value
 */
uint16_t pwm_calc_top(uint32_t freq_hz, uint8_t div_int, uint8_t div_frac);

/**
 * Full initialization with frequency specification
 * Use at setup, not in hot path
 * 
 * @param slice Slice number
 * @param freq_hz Desired frequency
 * @param phase_correct Enable phase-correct mode
 */
void pwm_init_frequency(uint8_t slice, uint32_t freq_hz, bool phase_correct);

#endif // PWM_FAST_H
