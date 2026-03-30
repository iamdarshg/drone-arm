/*
 * Host-side unit tests for the generic PWM output driver (SPEC §6.7).
 *
 * pwm_output.c is compiled as a separate translation unit (see meson.build)
 * with -DTEST_HOST=1 so hardware register access is disabled.  GPIO calls
 * are satisfied by tests/gpio_stub.c.
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "pwm_output.h"

/* ---- minimal test framework ---------------------------------------------- */
static int g_passed = 0;
static int g_failed = 0;

#define TEST_ASSERT(expr) do { \
    if (!(expr)) { \
        printf("  FAIL: %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        g_failed++; \
    } else { \
        g_passed++; \
    } \
} while (0)

#define RUN_TEST(name, func) do { \
    printf("Testing %s ...\n", (name)); \
    (func)(); \
} while (0)

/* ---- helpers -------------------------------------------------------------- */

static pwm_output_config_t make_esc_cfg(uint32_t num_ch, uint32_t freq_hz) {
    pwm_output_config_t cfg;
    uint32_t i;
    memset(&cfg, 0, sizeof(cfg));
    cfg.num_channels     = num_ch;
    cfg.freq_hz          = freq_hz;
    cfg.pulse_min_us     = 1000u;
    cfg.pulse_max_us     = 2000u;
    cfg.pulse_neutral_us = 1000u;
    for (i = 0u; i < num_ch; ++i) {
        cfg.gpio_pin[i] = i + 10u;
    }
    return cfg;
}

static pwm_output_config_t make_servo_cfg(uint32_t num_ch) {
    pwm_output_config_t cfg;
    uint32_t i;
    memset(&cfg, 0, sizeof(cfg));
    cfg.num_channels     = num_ch;
    cfg.freq_hz          = 50u;
    cfg.pulse_min_us     = 500u;
    cfg.pulse_max_us     = 2500u;
    cfg.pulse_neutral_us = 1500u;
    for (i = 0u; i < num_ch; ++i) {
        cfg.gpio_pin[i] = i + 20u;
    }
    return cfg;
}

/* ---- test cases ----------------------------------------------------------- */

static void test_esc_init_and_period(void) {
    pwm_output_config_t cfg = make_esc_cfg(4u, 400u);
    pwm_output_init(&cfg);

    /* 1 000 000 / 400 = 2500 µs per frame */
    TEST_ASSERT(pwm_output_get_period_us() == 2500u);

    /* all channels start at neutral */
    TEST_ASSERT(pwm_output_get_pulse(0u) == 1000u);
    TEST_ASSERT(pwm_output_get_pulse(3u) == 1000u);
}

static void test_servo_init_and_period(void) {
    pwm_output_config_t cfg = make_servo_cfg(2u);
    pwm_output_init(&cfg);

    /* 1 000 000 / 50 = 20 000 µs per frame */
    TEST_ASSERT(pwm_output_get_period_us() == 20000u);

    /* all channels start at neutral (1500 µs center) */
    TEST_ASSERT(pwm_output_get_pulse(0u) == 1500u);
    TEST_ASSERT(pwm_output_get_pulse(1u) == 1500u);
}

static void test_user_defined_frequency(void) {
    /* Verify period_us is derived directly from whatever freq_hz is given */
    pwm_output_config_t cfg;

    cfg = make_esc_cfg(1u, 200u);
    pwm_output_init(&cfg);
    TEST_ASSERT(pwm_output_get_period_us() == 5000u);  /* 1e6/200 = 5000 */

    cfg = make_esc_cfg(1u, 100u);
    pwm_output_init(&cfg);
    TEST_ASSERT(pwm_output_get_period_us() == 10000u); /* 1e6/100 = 10000 */

    cfg = make_servo_cfg(1u);
    cfg.freq_hz = 333u;
    pwm_output_init(&cfg);
    TEST_ASSERT(pwm_output_get_period_us() == 3003u);  /* 1e6/333 ≈ 3003 */
}

static void test_set_pulse_clamps(void) {
    pwm_output_config_t cfg = make_esc_cfg(2u, 400u);
    pwm_output_init(&cfg);

    /* Normal value within range */
    TEST_ASSERT(pwm_output_set_pulse(0u, 1500u));
    TEST_ASSERT(pwm_output_get_pulse(0u) == 1500u);

    /* Clamp below minimum */
    TEST_ASSERT(pwm_output_set_pulse(0u, 500u));
    TEST_ASSERT(pwm_output_get_pulse(0u) == 1000u);

    /* Clamp above maximum */
    TEST_ASSERT(pwm_output_set_pulse(0u, 9999u));
    TEST_ASSERT(pwm_output_get_pulse(0u) == 2000u);

    /* Out-of-range channel index */
    TEST_ASSERT(!pwm_output_set_pulse(7u, 1500u)); /* only 2 channels */
}

static void test_set_all(void) {
    pwm_output_config_t cfg = make_esc_cfg(4u, 400u);
    uint32_t pulses[4] = {1100u, 1200u, 1300u, 1400u};
    pwm_output_init(&cfg);

    TEST_ASSERT(pwm_output_set_all(pulses, 4u));
    TEST_ASSERT(pwm_output_get_pulse(0u) == 1100u);
    TEST_ASSERT(pwm_output_get_pulse(1u) == 1200u);
    TEST_ASSERT(pwm_output_get_pulse(2u) == 1300u);
    TEST_ASSERT(pwm_output_get_pulse(3u) == 1400u);

    /* Supplying more channels than configured must fail */
    TEST_ASSERT(!pwm_output_set_all(pulses, 5u));
}

static void test_reset(void) {
    pwm_output_config_t cfg = make_esc_cfg(4u, 400u);
    uint32_t pulses[4] = {2000u, 2000u, 2000u, 2000u};
    pwm_output_init(&cfg);
    pwm_output_set_all(pulses, 4u);
    TEST_ASSERT(pwm_output_get_pulse(0u) == 2000u);

    pwm_output_reset();
    TEST_ASSERT(pwm_output_get_pulse(0u) == 1000u); /* back to neutral */
    TEST_ASSERT(pwm_output_get_pulse(3u) == 1000u);
}

static void test_servo_full_range(void) {
    pwm_output_config_t cfg = make_servo_cfg(3u);
    pwm_output_init(&cfg);

    /* Min edge */
    TEST_ASSERT(pwm_output_set_pulse(0u, 500u));
    TEST_ASSERT(pwm_output_get_pulse(0u) == 500u);

    /* Max edge */
    TEST_ASSERT(pwm_output_set_pulse(1u, 2500u));
    TEST_ASSERT(pwm_output_get_pulse(1u) == 2500u);

    /* Center */
    TEST_ASSERT(pwm_output_set_pulse(2u, 1500u));
    TEST_ASSERT(pwm_output_get_pulse(2u) == 1500u);
}

static void test_uninitialised_returns_safe_values(void) {
    pwm_output_test_reset_state(); /* TEST_HOST helper: clears g_initialised */

    TEST_ASSERT(pwm_output_get_period_us() == 0u);
    TEST_ASSERT(pwm_output_get_pulse(0u) == 0u);
    TEST_ASSERT(!pwm_output_set_pulse(0u, 1500u));
    {
        uint32_t p = 1500u;
        TEST_ASSERT(!pwm_output_set_all(&p, 1u));
    }
}

/* ---- main ---------------------------------------------------------------- */

int main(void) {
    printf("\n");
    printf("========================================\n");
    printf("PWM Output Unit Tests (SPEC §6.7)\n");
    printf("========================================\n\n");

    RUN_TEST("esc_init_and_period",           test_esc_init_and_period);
    RUN_TEST("servo_init_and_period",         test_servo_init_and_period);
    RUN_TEST("user_defined_frequency",        test_user_defined_frequency);
    RUN_TEST("set_pulse_clamps",              test_set_pulse_clamps);
    RUN_TEST("set_all",                       test_set_all);
    RUN_TEST("reset",                         test_reset);
    RUN_TEST("servo_full_range",              test_servo_full_range);
    RUN_TEST("uninitialised_safe_values",     test_uninitialised_returns_safe_values);

    printf("\n========================================\n");
    printf("Results: %d passed, %d failed\n", g_passed, g_failed);
    printf("========================================\n");

    return g_failed > 0 ? 1 : 0;
}

