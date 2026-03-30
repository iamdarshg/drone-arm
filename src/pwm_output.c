/* pwm_output.c – generic RC-PWM output driver (SPEC §6.7).
 *
 * Generates standard RC-PWM pulses on arbitrary GPIO pins at a
 * user-defined frequency.  Works for both ESCs and servos.
 *
 * Pulse timing uses SIO bit-banging via a cycle-count busy-wait.
 * Replace the busy-wait body with a PIO or hardware-timer backend
 * for multi-channel simultaneous output.
 *
 * Frame period:  period_us = 1 000 000 / freq_hz
 * Pulse range:   [pulse_min_us, pulse_max_us], neutral = pulse_neutral_us
 */
#include "kernel/assert.h"
#include <stdbool.h>
#include <stdint.h>

#include "pwm_output.h"
#include "drivers/clock.h"
#include "drivers/gpio.h"
#include "hal/platform.h"

/* ---------- module state --------------------------------------------------- */

static pwm_output_config_t g_cfg;
static uint32_t g_pulse_us[PWM_OUTPUT_MAX_CHANNELS];
static uint32_t g_period_us;
static bool g_initialised;

/* ---------- internal helpers ----------------------------------------------- */

/* Busy-wait for 'us' microseconds using the SysTick or a cycle counter.
 * On RP2350 the TIMER0_BASE TIME_LO register provides a 1 µs free-running
 * counter sourced from the system clock.                                     */
enum {
    TIMER_TIMELR = 0x28u, /* Read-latched low 32 bits of the 64-bit timer     */
};

#ifdef TEST_HOST
/* No hardware available on the host – delay_us is a no-op. */
static void delay_us(uint32_t us) { (void)us; }
#else
static void delay_us(uint32_t us) {
    uint32_t start;
    uint32_t i;
    start = REG_RO(TIMER0_BASE + TIMER_TIMELR);
    for (i = 0u; i < us * 200u; ++i) { /* fallback cycle spin */
        if ((REG_RO(TIMER0_BASE + TIMER_TIMELR) - start) >= us) {
            return;
        }
    }
}
#endif

static uint32_t clamp_pulse(uint32_t pulse_us) {
    if (pulse_us < g_cfg.pulse_min_us) { return g_cfg.pulse_min_us; }
    if (pulse_us > g_cfg.pulse_max_us) { return g_cfg.pulse_max_us; }
    return pulse_us;
}

/* ---------- public API ----------------------------------------------------- */

void pwm_output_init(const pwm_output_config_t *cfg) {
    uint32_t i;
    ASSERT(cfg != NULL);
    ASSERT(cfg->num_channels >= 1u);
    ASSERT(cfg->num_channels <= PWM_OUTPUT_MAX_CHANNELS);
    ASSERT(cfg->freq_hz > 0u);
    ASSERT(cfg->pulse_min_us > 0u);
    ASSERT(cfg->pulse_max_us > cfg->pulse_min_us);
    ASSERT(cfg->pulse_neutral_us >= cfg->pulse_min_us);
    ASSERT(cfg->pulse_neutral_us <= cfg->pulse_max_us);

    g_cfg = *cfg;
    g_period_us = 1000000u / cfg->freq_hz;

    for (i = 0u; i < cfg->num_channels; ++i) {
        gpio_set_function(cfg->gpio_pin[i], 5u); /* SIO function */
        gpio_set_dir(cfg->gpio_pin[i], true);
        gpio_put(cfg->gpio_pin[i], false);
        g_pulse_us[i] = cfg->pulse_neutral_us;
    }
    g_initialised = true;
}

bool pwm_output_set_pulse(uint32_t channel_idx, uint32_t pulse_us) {
    ASSERT(channel_idx < PWM_OUTPUT_MAX_CHANNELS);
    if (!g_initialised || channel_idx >= g_cfg.num_channels) {
        return false;
    }
    g_pulse_us[channel_idx] = clamp_pulse(pulse_us);
    return true;
}

bool pwm_output_set_all(const uint32_t *pulse_us, uint32_t count) {
    uint32_t i;
    ASSERT(pulse_us != NULL);
    if (!g_initialised || count > g_cfg.num_channels) {
        return false;
    }
    for (i = 0u; i < count; ++i) {
        g_pulse_us[i] = clamp_pulse(pulse_us[i]);
    }
    return true;
}

void pwm_output_reset(void) {
    uint32_t i;
    for (i = 0u; i < PWM_OUTPUT_MAX_CHANNELS; ++i) {
        g_pulse_us[i] = g_initialised ? g_cfg.pulse_neutral_us : 0u;
        if (g_initialised && i < g_cfg.num_channels) {
            gpio_put(g_cfg.gpio_pin[i], false);
        }
    }
}

uint32_t pwm_output_get_pulse(uint32_t channel_idx) {
    ASSERT(channel_idx < PWM_OUTPUT_MAX_CHANNELS);
    if (!g_initialised || channel_idx >= g_cfg.num_channels) {
        return 0u;
    }
    return g_pulse_us[channel_idx];
}

uint32_t pwm_output_get_period_us(void) {
    if (!g_initialised) {
        return 0u;
    }
    return g_period_us;
}

void pwm_output_update(void) {
    uint32_t i;
    if (!g_initialised) {
        return;
    }
    for (i = 0u; i < g_cfg.num_channels; ++i) {
        gpio_put(g_cfg.gpio_pin[i], true);
        delay_us(g_pulse_us[i]);
        gpio_put(g_cfg.gpio_pin[i], false);
    }
}

#ifdef TEST_HOST
/* Reset all internal state – for host-unit-test use only. */
void pwm_output_test_reset_state(void) {
    uint32_t i;
    g_initialised = false;
    g_period_us = 0u;
    for (i = 0u; i < PWM_OUTPUT_MAX_CHANNELS; ++i) {
        g_pulse_us[i] = 0u;
    }
}
#endif
