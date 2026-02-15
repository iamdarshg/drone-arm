/**
 * Fast PWM Implementation for RP2350B
 * 
 * Optimizations for maximum speed:
 * - Direct register access (no SDK overhead)
 * - Inline functions for critical paths
 * - Supports all 12 PWM slices (24 channels) on RP2350B
 * - Maximum frequency: 75 MHz @ 150 MHz sysclk (TOP=1, DIV=1)
 */

#include "pwm.h"
#include <hardware/structs/pwm.h>
#include <hardware/regs/addressmap.h>
#include <hardware/structs/io_bank0.h>
#include <stdbool.h>

// GPIO to PWM slice/channel mapping for RP2350
// Even GPIOs -> Channel A, Odd GPIOs -> Channel B
// Slice = GPIO / 2 (for GPIOs 0-29)
static inline uint8_t gpio_to_slice(uint8_t gpio) {
    if (gpio < 32) {
        return (gpio >> 1) & 0x0F;  // 12 slices max
    }
    return 0xFF;  // Invalid
}

static inline bool gpio_is_channel_a(uint8_t gpio) {
    return (gpio & 1) == 0;
}

// Clock frequency (RP2350 can run up to 150 MHz)
#ifndef SYSCLK_FREQ_HZ
#define SYSCLK_FREQ_HZ 150000000u
#endif

void pwm_init(uint8_t channel, uint32_t freq_hz) {
    if (channel >= 24) return;  // 12 slices * 2 channels = 24
    
    uint8_t slice_num = channel >> 1;
    bool is_channel_a = (channel & 1) == 0;
    
    // Calculate divider for requested frequency
    // freq = sysclk / (TOP + 1) / DIV
    // For maximum frequency, use smallest TOP (1) and DIV (1)
    // which gives: freq = sysclk / 2
    
    uint32_t top = PWM_TOP_MAX;
    uint32_t div_int = 1;
    uint32_t div_frac = 0;
    
    if (freq_hz > 0 && freq_hz < (SYSCLK_FREQ_HZ / 2)) {
        // Calculate TOP and DIV for requested frequency
        // Try to maximize TOP for better resolution
        uint32_t cycles = SYSCLK_FREQ_HZ / freq_hz;
        
        if (cycles <= (PWM_TOP_MAX + 1)) {
            // Can achieve with DIV=1
            top = cycles - 1;
        } else {
            // Need divider
            uint32_t div = (cycles + (PWM_TOP_MAX / 2)) / (PWM_TOP_MAX + 1);
            if (div > 255) div = 255;
            div_int = div;
            top = (SYSCLK_FREQ_HZ / freq_hz / div) - 1;
        }
    } else if (freq_hz == 0) {
        // Fastest possible: 75 MHz @ 150 MHz sysclk
        top = 1;
        div_int = 1;
    }
    
    // Configure slice
    pwm_hw->slice[slice_num].csr = 0;  // Disable and clear settings
    pwm_hw->slice[slice_num].ctr = 0;  // Reset counter
    pwm_hw->slice[slice_num].top = top;
    pwm_hw->slice[slice_num].div = (div_int << PWM_DIV_INT_LSB) | div_frac;
    
    // Initialize compare value to 0 (0% duty cycle)
    uint32_t cc = pwm_hw->slice[slice_num].cc;
    if (is_channel_a) {
        cc = (cc & PWM_CC_B_MASK) | 0;
    } else {
        cc = (cc & PWM_CC_A_MASK) | 0;
    }
    pwm_hw->slice[slice_num].cc = cc;
}

void pwm_set_duty(uint8_t channel, float duty) {
    if (channel >= 24) return;
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;
    
    uint8_t slice_num = channel >> 1;
    bool is_channel_a = (channel & 1) == 0;
    
    uint32_t top = pwm_hw->slice[slice_num].top;
    uint32_t compare_value = (uint32_t)(duty * (top + 1));
    if (compare_value > top) compare_value = top;
    
    // Update compare register atomically
    uint32_t cc = pwm_hw->slice[slice_num].cc;
    if (is_channel_a) {
        cc = (cc & PWM_CC_B_MASK) | (compare_value << PWM_CC_A_LSB);
    } else {
        cc = (cc & PWM_CC_A_MASK) | (compare_value << PWM_CC_B_LSB);
    }
    pwm_hw->slice[slice_num].cc = cc;
}

float pwm_get_duty(uint8_t channel) {
    if (channel >= 24) return 0.0f;
    
    uint8_t slice_num = channel >> 1;
    bool is_channel_a = (channel & 1) == 0;
    
    uint32_t top = pwm_hw->slice[slice_num].top;
    uint32_t cc = pwm_hw->slice[slice_num].cc;
    uint32_t compare_value;
    
    if (is_channel_a) {
        compare_value = (cc >> PWM_CC_A_LSB) & 0xFFFF;
    } else {
        compare_value = (cc >> PWM_CC_B_LSB) & 0xFFFF;
    }
    
    return (float)compare_value / (float)(top + 1);
}

void pwm_enable(uint8_t channel) {
    if (channel >= 24) return;
    
    uint8_t slice_num = channel >> 1;
    pwm_hw->en |= (1u << slice_num);
    pwm_hw->slice[slice_num].csr |= PWM_CSR_EN_BIT;
}

void pwm_disable(uint8_t channel) {
    if (channel >= 24) return;
    
    uint8_t slice_num = channel >> 1;
    
    // Only disable if other channel is not in use
    // Check if either channel A or B is still active
    // For simplicity, disable the slice
    pwm_hw->slice[slice_num].csr &= ~PWM_CSR_EN_BIT;
    pwm_hw->en &= ~(1u << slice_num);
}

// Extended API for maximum performance

/**
 * Initialize PWM for maximum speed (75 MHz @ 150 MHz sysclk)
 * Use this when you need the fastest possible PWM frequency
 */
void pwm_init_fastest(uint8_t slice_num) {
    if (slice_num >= 12) return;
    
    pwm_hw->slice[slice_num].csr = 0;
    pwm_hw->slice[slice_num].ctr = 0;
    pwm_hw->slice[slice_num].top = PWM_FASTEST_TOP;  // TOP=1
    pwm_hw->slice[slice_num].div = PWM_FASTEST_DIV;  // DIV=1
    pwm_hw->slice[slice_num].cc = 0;
}

/**
 * Set duty cycle using raw compare value for maximum speed
 * No floating point, no calculations - direct register write
 */
void pwm_set_duty_raw(uint8_t slice_num, bool channel_a, uint16_t compare_value) {
    if (slice_num >= 12) return;
    
    uint32_t cc = pwm_hw->slice[slice_num].cc;
    if (channel_a) {
        cc = (cc & PWM_CC_B_MASK) | ((uint32_t)compare_value << PWM_CC_A_LSB);
    } else {
        cc = (cc & PWM_CC_A_MASK) | ((uint32_t)compare_value << PWM_CC_B_LSB);
    }
    pwm_hw->slice[slice_num].cc = cc;
}

/**
 * Enable multiple slices simultaneously using global enable register
 * This is faster than enabling slices individually
 */
void pwm_enable_slices(uint32_t slice_mask) {
    pwm_hw->en = slice_mask;
    for (int i = 0; i < 12; i++) {
        if (slice_mask & (1u << i)) {
            pwm_hw->slice[i].csr |= PWM_CSR_EN_BIT;
        }
    }
}

/**
 * Disable all PWM slices at once
 */
void pwm_disable_all(void) {
    pwm_hw->en = 0;
    for (int i = 0; i < 12; i++) {
        pwm_hw->slice[i].csr &= ~PWM_CSR_EN_BIT;
    }
}

/**
 * Configure GPIO pin for PWM function (FUNCSEL = 4)
 */
void pwm_gpio_init(uint8_t gpio) {
    if (gpio >= 48) return;  // RP2350B has 48 GPIOs
    
    // Set GPIO function to PWM (funcsel = 4)
    // Clear previous function and set to PWM
    io_bank0_hw->gpio[gpio].ctrl = (io_bank0_hw->gpio[gpio].ctrl & ~0x1F) | 4;
}

/**
 * Get current frequency of a PWM slice
 */
uint32_t pwm_get_frequency(uint8_t slice_num) {
    if (slice_num >= 12) return 0;
    
    uint32_t top = pwm_hw->slice[slice_num].top;
    uint32_t div_reg = pwm_hw->slice[slice_num].div;
    uint32_t div_int = (div_reg & PWM_DIV_INT_MASK) >> PWM_DIV_INT_LSB;
    uint32_t div_frac = div_reg & PWM_DIV_FRAC_MASK;
    
    if (div_int == 0) div_int = 1;  // DIV=0 means DIV=1
    
    // Calculate actual divider value (integer + fraction/16)
    float divider = (float)div_int + ((float)div_frac / 16.0f);
    
    return (uint32_t)(SYSCLK_FREQ_HZ / ((top + 1) * divider));
}
