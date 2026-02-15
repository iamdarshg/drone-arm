/**
 * PWM Driver for Main Controller (RP2350B)
 * 
 * Usage:
 *   1. Configure pwm_config.h with desired slices and frequency
 *   2. Call pwm_init_all() to initialize all configured slices
 *   3. Use pwm_set_duty() to set duty cycles
 *   4. Use pwm_enable() / pwm_disable() to control output
 */

#ifndef PWM_H
#define PWM_H

#include <stdint.h>
#include <stdbool.h>
#include "../hardware_defs/pwm_config.h"
#include "../../../include/hardware/structs/pwm.h"

/**
 * Initialize all enabled PWM slices according to pwm_config.h
 * Configures GPIO, divider, TOP value, and initial state
 */
/** @brief Standardized PWM initialization wrapper. */
void init_pwm(void);

void pwm_init_all(void);

/**
 * Initialize a specific slice (even if not enabled in config)
 * 
 * @param slice Slice number (0-11)
 * @param freq_hz Desired frequency in Hz (0 for config default)
 */
void pwm_slice_init(uint8_t slice, uint32_t freq_hz);

/**
 * Set duty cycle as percentage (0.0 - 1.0)
 * 
 * @param channel PWM channel (0-23)
 * @param duty Duty cycle 0.0 to 1.0
 */
void pwm_set_duty(uint8_t channel, float duty);

/**
 * Set duty cycle using raw compare value
 * Faster than pwm_set_duty, no floating point
 * 
 * @param channel PWM channel (0-23)
 * @param compare Raw compare value (0 to TOP)
 */
static inline void pwm_set_compare(uint8_t channel, uint16_t compare) {
    if (channel >= PWM_NUM_CHANNELS) return;
    
    uint8_t slice = channel >> 1;
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
 * Get current duty cycle as percentage
 * 
 * @param channel PWM channel (0-23)
 * @return Duty cycle 0.0 to 1.0
 */
float pwm_get_duty(uint8_t channel);

/**
 * Enable PWM output for a channel
 * 
 * @param channel PWM channel (0-23)
 */
void pwm_enable(uint8_t channel);

/**
 * Disable PWM output for a channel
 * 
 * @param channel PWM channel (0-23)
 */
void pwm_disable(uint8_t channel);

/**
 * Enable all configured slices at once
 */
static inline void pwm_enable_all(void) {
    pwm_hw->en = PWM_ACTIVE_SLICE_MASK;
    for (int i = 0; i < PWM_NUM_SLICES; i++) {
        if (PWM_SLICE_IS_ENABLED(i)) {
            pwm_hw->slice[i].csr |= PWM_CSR_EN_BIT;
        }
    }
}

/**
 * Disable all PWM slices
 */
static inline void pwm_disable_all(void) {
    pwm_hw->en = 0;
    for (int i = 0; i < PWM_NUM_SLICES; i++) {
        pwm_hw->slice[i].csr &= ~PWM_CSR_EN_BIT;
    }
}

/**
 * Set both channels of a slice simultaneously
 * More efficient than setting individually
 * 
 * @param slice Slice number (0-11)
 * @param duty_a Duty for channel A (0.0 - 1.0)
 * @param duty_b Duty for channel B (0.0 - 1.0)
 */
void pwm_set_slice_duty(uint8_t slice, float duty_a, float duty_b);

/**
 * Get the configured TOP value
 * 
 * @return TOP register value
 */
static inline uint16_t pwm_get_top(void) {
    return PWM_TOP_VALUE;
}

/**
 * Get the configured frequency
 * 
 * @return Frequency in Hz
 */
static inline uint32_t pwm_get_frequency(void) {
    return PWM_ACTUAL_FREQ_HZ;
}

/**
 * Check if a channel is enabled
 * 
 * @param channel PWM channel (0-23)
 * @return true if enabled
 */
bool pwm_is_enabled(uint8_t channel);

#endif // PWM_H
