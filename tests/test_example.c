/*
 * tests/test_example.c
 *
 * Example host-side unit test demonstrating the test harness conventions used
 * across this repository (SPEC §7.2, §6.7).
 *
 * Tests a small set of pure-computation helpers to verify that the test
 * scaffold, CI pipeline, and Meson test integration are all functioning.
 *
 * Build:
 *   meson setup builddir && ninja -C builddir && meson test -C builddir example_test
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ---- minimal test framework (shared convention across tests/) ------------ */

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

/* ---- module under test: simple integer arithmetic stubs ----------------- */

/*
 * clamp_u32 – returns v clamped to [lo, hi].
 *
 * Used internally by pwm_output and other modules to guard pulse-width
 * values against out-of-range inputs.
 */
static uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/*
 * us_to_ticks – converts a microsecond duration to scheduler ticks at a
 * fixed tick rate of 1 kHz (1 tick = 1 ms = 1000 µs).
 */
static uint32_t us_to_ticks(uint32_t us) {
    return us / 1000u;
}

/* ---- test cases --------------------------------------------------------- */

static void test_clamp_within_range(void) {
    TEST_ASSERT(clamp_u32(1500u, 1000u, 2000u) == 1500u);
}

static void test_clamp_below_min(void) {
    TEST_ASSERT(clamp_u32(500u, 1000u, 2000u) == 1000u);
}

static void test_clamp_above_max(void) {
    TEST_ASSERT(clamp_u32(2500u, 1000u, 2000u) == 2000u);
}

static void test_clamp_at_min_boundary(void) {
    TEST_ASSERT(clamp_u32(1000u, 1000u, 2000u) == 1000u);
}

static void test_clamp_at_max_boundary(void) {
    TEST_ASSERT(clamp_u32(2000u, 1000u, 2000u) == 2000u);
}

static void test_us_to_ticks_basic(void) {
    /* 20 000 µs (50 Hz frame) → 20 ticks */
    TEST_ASSERT(us_to_ticks(20000u) == 20u);
}

static void test_us_to_ticks_sub_ms(void) {
    /* sub-millisecond durations round down to 0 ticks */
    TEST_ASSERT(us_to_ticks(999u) == 0u);
}

static void test_us_to_ticks_exact_ms(void) {
    TEST_ASSERT(us_to_ticks(1000u) == 1u);
}

/* ---- main --------------------------------------------------------------- */

int main(void) {
    RUN_TEST("clamp_within_range",    test_clamp_within_range);
    RUN_TEST("clamp_below_min",       test_clamp_below_min);
    RUN_TEST("clamp_above_max",       test_clamp_above_max);
    RUN_TEST("clamp_at_min_boundary", test_clamp_at_min_boundary);
    RUN_TEST("clamp_at_max_boundary", test_clamp_at_max_boundary);
    RUN_TEST("us_to_ticks_basic",     test_us_to_ticks_basic);
    RUN_TEST("us_to_ticks_sub_ms",    test_us_to_ticks_sub_ms);
    RUN_TEST("us_to_ticks_exact_ms",  test_us_to_ticks_exact_ms);

    printf("\nResults: %d passed, %d failed\n", g_passed, g_failed);
    return (g_failed != 0) ? 1 : 0;
}
