#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "sensors/state_vector.h"

static int g_passed = 0;
static int g_failed = 0;

#define TEST_ASSERT(expr) do { \
    if (!(expr)) { \
        printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #expr); \
        g_failed++; \
    } else { \
        g_passed++; \
    } \
} while (0)

int main(void) {
    state_vector_ready_t ready;
    imu_state_vector_t imu;
    gps_state_vector_t gps;
    const volatile state_vector_shared_t *shared;
    state_vector_init();
    TEST_ASSERT(state_vector_register_imu(0u, 0x6Au));
    TEST_ASSERT(state_vector_register_imu(1u, 0x67u));
    TEST_ASSERT(state_vector_register_gps(0u, 0x42u));
    TEST_ASSERT(state_vector_request_all_async());
    TEST_ASSERT(state_vector_poll_ready(&ready));
    TEST_ASSERT((ready.imu_ready_mask & 0x3u) == 0x3u);
    TEST_ASSERT((ready.gps_ready_mask & 0x1u) == 0x1u);
    TEST_ASSERT(state_vector_register_shared_region_with_scheduler());
    shared = state_vector_shared();
    TEST_ASSERT(shared != NULL);
    TEST_ASSERT((shared->ready.imu_ready_mask & 0x3u) == 0x3u);
    TEST_ASSERT((shared->ready.gps_ready_mask & 0x1u) == 0x1u);
    TEST_ASSERT(state_vector_read_imu(0u, &imu));
    TEST_ASSERT(state_vector_read_gps(0u, &gps));
    TEST_ASSERT((shared->ready.imu_ready_mask & 0x1u) == 0u);
    TEST_ASSERT((shared->ready.gps_ready_mask & 0x1u) == 0u);
    (void)imu;
    (void)gps;
    printf("state_vector tests: %d passed, %d failed\n", g_passed, g_failed);
    return g_failed == 0 ? 0 : 1;
}
