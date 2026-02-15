/**
 * PWM Driver Implementation for Main Controller (RP2350B)
 */

#include "pwm.h"
#include <stdlib.h>
#include "../../../include/hardware/structs/io_bank0.h"

// Configure GPIO for PWM function
static void pwm_gpio_init(uint8_t gpio) {
    if (gpio >= 48) return;  // RP2350B has 48 GPIOs
    
    // Set GPIO function to PWM (funcsel = 4)
    // RP2350: GPIO_CTRL_FUNCSEL = 4 for PWM
    io_bank0_hw->gpio[gpio].ctrl = (io_bank0_hw->gpio[gpio].ctrl & ~0x1F) | 4;
}

void init_pwm(void) {
    pwm_init_all();
}

void pwm_init_all(void) {
    for (uint8_t slice = 0; slice < PWM_NUM_SLICES; slice++) {
        if (!pwm_slice_enabled[slice]) continue;
        
        // Initialize GPIO pins
        uint8_t gpio_a = pwm_gpio_a[slice];
        uint8_t gpio_b = pwm_gpio_b[slice];
        
        if (gpio_a != 0xFF) pwm_gpio_init(gpio_a);
        if (gpio_b != 0xFF) pwm_gpio_init(gpio_b);
        
        // Configure slice
        pwm_slice_init(slice, 0);  // 0 means use config default
    }
}

void pwm_slice_init(uint8_t slice, uint32_t freq_hz) {
    if (slice >= PWM_NUM_SLICES) return;
    
    // Disable slice first
    pwm_hw->slice[slice].csr = 0;
    pwm_hw->slice[slice].ctr = 0;
    
    // Use config values if freq_hz is 0
    uint16_t top = PWM_TOP_VALUE;
    uint32_t div = (PWM_DIV_INT << PWM_DIV_INT_LSB) | PWM_DIV_FRAC;
    
    if (freq_hz != 0 && freq_hz != PWM_ACTUAL_FREQ_HZ) {
        // Calculate new TOP and DIV for requested frequency
        uint32_t cycles = SYSCLK_FREQ_HZ / freq_hz;
        
        if (PWM_PHASE_CORRECT) {
            cycles /= 2;
        }
        
        if (cycles <= 65536) {
            top = cycles - 1;
            div = (1u << PWM_DIV_INT_LSB);
        } else {
            uint32_t div_int = (cycles + 32767) / 65536;
            if (div_int > 255) div_int = 255;
            top = (cycles / div_int) - 1;
            div = (div_int << PWM_DIV_INT_LSB);
        }
    }
    
    // Set registers
    pwm_hw->slice[slice].top = top;
    pwm_hw->slice[slice].div = div;
    pwm_hw->slice[slice].cc = 0;  // Start with 0% duty
}

void pwm_set_duty(uint8_t channel, float duty) {
    if (channel >= PWM_NUM_CHANNELS) return;
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;
    
    uint8_t slice = channel >> 1;
    uint16_t top = pwm_hw->slice[slice].top;
    uint16_t compare = (uint16_t)(duty * (top + 1));
    
    if (compare > top) compare = top;
    
    pwm_set_compare(channel, compare);
}

float pwm_get_duty(uint8_t channel) {
    if (channel >= PWM_NUM_CHANNELS) return 0.0f;
    
    uint8_t slice = channel >> 1;
    uint16_t top = pwm_hw->slice[slice].top;
    uint32_t cc = pwm_hw->slice[slice].cc;
    uint16_t compare;
    
    if ((channel & 1) == 0) {
        compare = (uint16_t)(cc & 0xFFFFu);
    } else {
        compare = (uint16_t)((cc >> 16) & 0xFFFFu);
    }
    
    return (float)compare / (float)(top + 1);
}

void pwm_enable(uint8_t channel) {
    if (channel >= PWM_NUM_CHANNELS) return;
    
    uint8_t slice = channel >> 1;
    pwm_hw->en |= (1u << slice);
    pwm_hw->slice[slice].csr |= PWM_CSR_EN_BIT;
}

void pwm_disable(uint8_t channel) {
    if (channel >= PWM_NUM_CHANNELS) return;
    
    uint8_t slice = channel >> 1;
    
    // Check if other channel of this slice is enabled
    uint8_t other_channel = channel ^ 1;
    bool other_enabled = pwm_is_enabled(other_channel);
    
    if (!other_enabled) {
        // Safe to disable entire slice
        pwm_hw->slice[slice].csr &= ~PWM_CSR_EN_BIT;
        pwm_hw->en &= ~(1u << slice);
    }
    // If other channel is enabled, we can't disable without affecting it
    // In that case, just set duty to 0
    else {
        pwm_set_compare(channel, 0);
    }
}

void pwm_set_slice_duty(uint8_t slice, float duty_a, float duty_b) {
    if (slice >= PWM_NUM_SLICES) return;
    
    if (duty_a < 0.0f) duty_a = 0.0f;
    if (duty_a > 1.0f) duty_a = 1.0f;
    if (duty_b < 0.0f) duty_b = 0.0f;
    if (duty_b > 1.0f) duty_b = 1.0f;
    
    uint16_t top = pwm_hw->slice[slice].top;
    uint16_t compare_a = (uint16_t)(duty_a * (top + 1));
    uint16_t compare_b = (uint16_t)(duty_b * (top + 1));
    
    if (compare_a > top) compare_a = top;
    if (compare_b > top) compare_b = top;
    
    pwm_hw->slice[slice].cc = ((uint32_t)compare_b << 16) | compare_a;
}

bool pwm_is_enabled(uint8_t channel) {
    if (channel >= PWM_NUM_CHANNELS) return false;
    
    uint8_t slice = channel >> 1;
    return (pwm_hw->slice[slice].csr & PWM_CSR_EN_BIT) != 0;
}
