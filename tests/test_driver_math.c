/*
 * tests/test_driver_math.c
 *
 * Host-side unit tests for the pure-computation parts of the HAL/driver
 * startup sequences.  No hardware registers are touched; the helpers under
 * test are exposed by the driver headers and linked from the driver .c files.
 *
 * Coverage:
 *   – UART IBRD/FBRD divisor formula   (uart_calc_ibrd / uart_calc_fbrd)
 *   – SPI CPSDVSR + SCR divisor pair   (spi_calc_divisors)
 *   – I2C SCL high/low count pair      (i2c_calc_scl_counts)
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/*
 * Stub out clock_get_hz so driver .c files can be linked without clock.c.
 * Declaration comes from drivers/clock.h.
 */
#include "drivers/clock.h"
uint32_t clock_get_hz(clock_id_t id) { (void)id; return 150000000u; }

#include "drivers/uart.h"
#include "drivers/spi.h"
#include "drivers/i2c.h"

/* ---------- minimal test framework --------------------------------------- */

static int g_passed = 0;
static int g_failed = 0;

#define ASSERT_EQ(a, b) do { \
    if ((a) != (b)) { \
        printf("  FAIL %s:%d  got %u expected %u\n", \
               __FILE__, __LINE__, (unsigned)(a), (unsigned)(b)); \
        g_failed++; \
    } else { \
        g_passed++; \
    } \
} while (0)

#define ASSERT_RANGE(val, lo, hi) do { \
    if ((val) < (lo) || (val) > (hi)) { \
        printf("  FAIL %s:%d  value %u not in [%u,%u]\n", \
               __FILE__, __LINE__, (unsigned)(val), (unsigned)(lo), (unsigned)(hi)); \
        g_failed++; \
    } else { \
        g_passed++; \
    } \
} while (0)

/* ===== UART divisor tests ================================================= */

static void test_uart_115200(void) {
    /*
     * At 150 MHz peri clock and 115 200 baud:
     *   BRD  = 150 000 000 / (16 × 115 200) = 81.38…
     *   IBRD = 81
     *   FBRD ≈ 24  (SDK rounding)
     */
    uint32_t ibrd = uart_calc_ibrd(150000000u, 115200u);
    uint32_t fbrd = uart_calc_fbrd(150000000u, 115200u);
    ASSERT_EQ(ibrd, 81u);
    /* Allow ±1 for rounding differences between implementations */
    ASSERT_RANGE(fbrd, 23u, 26u);
}

static void test_uart_9600(void) {
    /*
     * At 150 MHz / 9 600 baud:
     *   BRD  = 150 000 000 / (16 × 9 600) = 976.5625
     *   IBRD = 976
     *   FBRD = round(0.5625 × 64) = 36
     */
    uint32_t ibrd = uart_calc_ibrd(150000000u, 9600u);
    uint32_t fbrd = uart_calc_fbrd(150000000u, 9600u);
    ASSERT_EQ(ibrd, 976u);
    ASSERT_RANGE(fbrd, 35u, 37u);
}

static void test_uart_exact_divisor(void) {
    /*
     * At 16 MHz / 9 600 baud:
     *   BRD  = 16 000 000 / (16 × 9 600) = 104.1667
     *   IBRD = 104
     *   FBRD = round(0.1667 × 64) ≈ 11
     */
    uint32_t ibrd = uart_calc_ibrd(16000000u, 9600u);
    uint32_t fbrd = uart_calc_fbrd(16000000u, 9600u);
    ASSERT_EQ(ibrd, 104u);
    ASSERT_RANGE(fbrd, 10u, 12u);
}

/* ===== SPI divisor tests ================================================== */

static void test_spi_1mhz(void) {
    /*
     * 150 MHz → 1 MHz:
     *   CPSDVSR = 2  (minimum even)
     *   SCR     = 150/(2×1) − 1 = 74
     *   actual  = 150/(2×75) = 1 MHz exactly
     */
    uint32_t cpsdvsr = 0u;
    uint32_t scr     = 0u;
    spi_calc_divisors(150000000u, 1000000u, &cpsdvsr, &scr);
    ASSERT_EQ(cpsdvsr, 2u);
    ASSERT_EQ(scr,    74u);
}

static void test_spi_10mhz(void) {
    /*
     * 150 MHz → 10 MHz:
     *   CPSDVSR = 2, SCR = 150/(2×10) − 1 = 6  → actual ≈ 10.7 MHz (closest)
     */
    uint32_t cpsdvsr = 0u;
    uint32_t scr     = 0u;
    spi_calc_divisors(150000000u, 10000000u, &cpsdvsr, &scr);
    ASSERT_EQ(cpsdvsr, 2u);
    ASSERT_EQ(scr,     6u);
}

static void test_spi_max(void) {
    /*
     * 150 MHz → 75 MHz (maximum with CPSDVSR=2, SCR=0):
     */
    uint32_t cpsdvsr = 0u;
    uint32_t scr     = 0u;
    spi_calc_divisors(150000000u, 75000000u, &cpsdvsr, &scr);
    ASSERT_EQ(cpsdvsr, 2u);
    ASSERT_EQ(scr,     0u);
}

static void test_spi_low_speed(void) {
    /*
     * 150 MHz → 300 kHz: SCR = 150/(2×0.3) − 1 = 249
     */
    uint32_t cpsdvsr = 0u;
    uint32_t scr     = 0u;
    spi_calc_divisors(150000000u, 300000u, &cpsdvsr, &scr);
    ASSERT_EQ(cpsdvsr, 2u);
    ASSERT_EQ(scr,   249u);
}

/* ===== I2C SCL count tests ================================================ */

static void test_i2c_100khz(void) {
    /*
     * 150 MHz / 100 kHz → period = 1500 cycles
     *   lcnt = 1500×3/5 = 900   (60 %)
     *   hcnt = 1500−900 = 600   (40 %)
     */
    uint32_t hcnt = 0u;
    uint32_t lcnt = 0u;
    i2c_calc_scl_counts(150000000u, 100000u, &hcnt, &lcnt);
    ASSERT_EQ(hcnt, 600u);
    ASSERT_EQ(lcnt, 900u);
}

static void test_i2c_400khz(void) {
    /*
     * 150 MHz / 400 kHz → period = 375 cycles
     *   lcnt = 375×3/5 = 225
     *   hcnt = 375−225 = 150
     */
    uint32_t hcnt = 0u;
    uint32_t lcnt = 0u;
    i2c_calc_scl_counts(150000000u, 400000u, &hcnt, &lcnt);
    ASSERT_EQ(hcnt, 150u);
    ASSERT_EQ(lcnt, 225u);
}

static void test_i2c_floor_clamp(void) {
    /*
     * Sanity: both counts must always be ≥ 8 (DW I2C hard lower limit)
     * even with unrealistically fast bitrates.
     */
    uint32_t hcnt = 0u;
    uint32_t lcnt = 0u;
    i2c_calc_scl_counts(16000000u, 3200000u, &hcnt, &lcnt);
    ASSERT_RANGE(hcnt, 8u, 8192u);
    ASSERT_RANGE(lcnt, 8u, 8192u);
}

/* ===== main =============================================================== */

int main(void) {
    printf("\n");
    printf("========================================\n");
    printf("Driver Math Unit Tests\n");
    printf("(UART divisors, SPI divisors, I2C SCL)\n");
    printf("========================================\n\n");

    printf("-- UART baudrate divisors --\n");
    test_uart_115200();
    test_uart_9600();
    test_uart_exact_divisor();

    printf("-- SPI clock divisors --\n");
    test_spi_1mhz();
    test_spi_10mhz();
    test_spi_max();
    test_spi_low_speed();

    printf("-- I2C SCL counts --\n");
    test_i2c_100khz();
    test_i2c_400khz();
    test_i2c_floor_clamp();

    printf("\n========================================\n");
    printf("Results: %d passed, %d failed\n", g_passed, g_failed);
    printf("========================================\n");
    return (g_failed > 0) ? 1 : 0;
}

