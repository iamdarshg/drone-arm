/* ESC Controller – PWM-based motor speed control for quadrotor (SPEC §6 extension).
 *
 * Generates standard 1000-2000 µs RC-PWM signals using SIO bit-banging.
 * Production builds should replace this with PIO-based hardware PWM for
 * precise timing.
 */
#include "kernel/assert.h"
#include <stdbool.h>
#include <stdint.h>

#include "esc_controller.h"
#include "drivers/gpio.h"
#include "hal/platform.h"

static esc_config_t g_cfg;
static uint32_t g_throttle_us[ESC_MOTOR_COUNT];
static bool g_initialised;

void esc_init(const esc_config_t *cfg) {
    uint32_t i;
    ASSERT(cfg != NULL);
    ASSERT(cfg->freq_hz > 0u);
    g_cfg = *cfg;
    for (i = 0u; i < ESC_MOTOR_COUNT; ++i) {
        gpio_set_function(cfg->gpio_pin[i], 5u); /* SIO function */
        gpio_set_dir(cfg->gpio_pin[i], true);
        gpio_put(cfg->gpio_pin[i], false);
        g_throttle_us[i] = ESC_PULSE_ARM_US;
    }
    g_initialised = true;
}

bool esc_set_throttle(uint32_t motor_idx, uint32_t throttle_us) {
    uint32_t clamped;
    ASSERT(motor_idx < ESC_MOTOR_COUNT);
    if (!g_initialised) {
        return false;
    }
    clamped = throttle_us < ESC_PULSE_MIN_US ? ESC_PULSE_MIN_US :
              throttle_us > ESC_PULSE_MAX_US ? ESC_PULSE_MAX_US : throttle_us;
    g_throttle_us[motor_idx] = clamped;
    return true;
}

bool esc_set_all(const uint32_t throttle_us[ESC_MOTOR_COUNT]) {
    uint32_t i;
    ASSERT(throttle_us != NULL);
    if (!g_initialised) {
        return false;
    }
    for (i = 0u; i < ESC_MOTOR_COUNT; ++i) {
        if (!esc_set_throttle(i, throttle_us[i])) {
            return false;
        }
    }
    return true;
}

void esc_disarm(void) {
    uint32_t i;
    for (i = 0u; i < ESC_MOTOR_COUNT; ++i) {
        g_throttle_us[i] = ESC_PULSE_ARM_US;
        if (g_initialised) {
            gpio_put(g_cfg.gpio_pin[i], false);
        }
    }
}

uint32_t esc_get_throttle(uint32_t motor_idx) {
    ASSERT(motor_idx < ESC_MOTOR_COUNT);
    if (!g_initialised) {
        return 0u;
    }
    return g_throttle_us[motor_idx];
}

