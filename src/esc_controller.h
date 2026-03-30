#ifndef ESC_CONTROLLER_H
#define ESC_CONTROLLER_H

/*
 * ESC Controller – PWM-based motor speed control for quadrotor.
 *
 * Each motor is driven by a standard 1000-2000 µs RC-PWM signal at 50 Hz
 * (or 400 Hz for fast-ESC-capable units).
 *
 * Motor layout (top-down view, X-frame):
 *   M1(CW)  M2(CCW)
 *      \    /
 *      /    \
 *   M3(CCW) M4(CW)
 */

#include <stdbool.h>
#include <stdint.h>

#define ESC_MOTOR_COUNT 4u

/* Pulse width range in microseconds (standard RC-PWM). */
#define ESC_PULSE_MIN_US  1000u
#define ESC_PULSE_MAX_US  2000u
#define ESC_PULSE_ARM_US  1000u

typedef struct {
    uint32_t gpio_pin[ESC_MOTOR_COUNT]; /* GPIO pins for each motor */
    uint32_t freq_hz;                   /* PWM update frequency (e.g. 50 or 400) */
} esc_config_t;

/* Initialise ESC PWM outputs.  Must be called once at startup. */
void esc_init(const esc_config_t *cfg);

/*
 * Set throttle for a single motor.
 * throttle_us: pulse width in µs, clamped to [ESC_PULSE_MIN_US, ESC_PULSE_MAX_US].
 */
bool esc_set_throttle(uint32_t motor_idx, uint32_t throttle_us);

/*
 * Set all four motor throttles in one call.
 * throttle_us[ESC_MOTOR_COUNT]: one entry per motor.
 */
bool esc_set_all(const uint32_t throttle_us[ESC_MOTOR_COUNT]);

/* Disarm all motors (set pulse to ESC_PULSE_ARM_US). */
void esc_disarm(void);

/* Return the last-written throttle for a motor (µs), or 0 if uninitialised. */
uint32_t esc_get_throttle(uint32_t motor_idx);

#endif
