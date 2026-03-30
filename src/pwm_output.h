#ifndef PWM_OUTPUT_H
#define PWM_OUTPUT_H

/*
 * pwm_output – generic RC-PWM output driver.
 *
 * Drives any number of GPIO pins with standard RC-PWM pulses (typically
 * 500–2500 µs) at a caller-specified frequency.  Suitable for both ESCs
 * (1000–2000 µs, 50–400 Hz) and servos (500–2500 µs, 50 Hz) without any
 * code changes – the operating parameters are supplied at init time.
 *
 * The driver stores the desired pulse widths and computes the frame period
 * (period_us = 1 000 000 / freq_hz).  Actual waveform generation is
 * performed by pwm_output_update(), which must be called from a periodic
 * timer task or ISR at the configured frequency.
 */

#include <stdbool.h>
#include <stdint.h>

/* Hard upper limit on the number of channels in a single instance. */
#define PWM_OUTPUT_MAX_CHANNELS 8u

typedef struct {
    uint32_t gpio_pin[PWM_OUTPUT_MAX_CHANNELS]; /* GPIO pins, one per channel */
    uint32_t num_channels;   /* 1 .. PWM_OUTPUT_MAX_CHANNELS                  */
    uint32_t freq_hz;        /* Frame rate: 50 Hz (servo/slow-ESC), 400 Hz … */
    uint32_t pulse_min_us;   /* Minimum valid pulse width in µs (e.g. 1000)   */
    uint32_t pulse_max_us;   /* Maximum valid pulse width in µs (e.g. 2000)   */
    uint32_t pulse_neutral_us; /* Safe/neutral pulse at init/reset (µs)       */
} pwm_output_config_t;

/* Convenience initialisers for common use-cases. */
#define PWM_OUTPUT_CFG_ESC_DEFAULTS \
    .freq_hz = 400u, .pulse_min_us = 1000u, .pulse_max_us = 2000u, .pulse_neutral_us = 1000u

#define PWM_OUTPUT_CFG_SERVO_DEFAULTS \
    .freq_hz = 50u, .pulse_min_us = 500u, .pulse_max_us = 2500u, .pulse_neutral_us = 1500u

/* ---------- lifecycle ------------------------------------------------------ */

/*
 * Initialise all output GPIOs and set every channel to pulse_neutral_us.
 * Must be called exactly once before any other function.
 */
void pwm_output_init(const pwm_output_config_t *cfg);

/* ---------- pulse control -------------------------------------------------- */

/*
 * Set the pulse width for a single channel.
 * pulse_us is clamped to [pulse_min_us, pulse_max_us].
 * Returns false if the driver is uninitialised or channel_idx is out of range.
 */
bool pwm_output_set_pulse(uint32_t channel_idx, uint32_t pulse_us);

/*
 * Set all channels in one call.
 * pulse_us must point to an array of at least num_channels values.
 */
bool pwm_output_set_all(const uint32_t *pulse_us, uint32_t count);

/*
 * Reset every channel to the neutral pulse (safe position).
 * Drives the GPIO lines low after the trailing edge of the next frame.
 */
void pwm_output_reset(void);

/* ---------- queries -------------------------------------------------------- */

/*
 * Return the last-requested pulse width for a channel (µs).
 * Returns 0 if uninitialised or channel_idx is out of range.
 */
uint32_t pwm_output_get_pulse(uint32_t channel_idx);

/*
 * Return the frame period in µs derived from freq_hz  (= 1 000 000 / freq_hz).
 * Returns 0 if uninitialised.
 */
uint32_t pwm_output_get_period_us(void);

/* ---------- output --------------------------------------------------------- */

/*
 * Generate one complete PWM frame on all channels using SIO bit-banging.
 *
 * Call this from a periodic timer ISR or cooperative task that fires at
 * freq_hz.  Each channel pin is driven HIGH for pulse_us then LOW;
 * channels are output sequentially in index order.
 *
 * For precise simultaneous multi-channel PWM use PIO instead.
 */
void pwm_output_update(void);

#ifdef TEST_HOST
/* Reset all internal state.  For host-unit-test use only. */
void pwm_output_test_reset_state(void);
#endif

#endif
