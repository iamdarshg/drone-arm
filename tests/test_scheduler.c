/*
 * Host-side unit tests for the cooperative scheduler (SPEC §7.2).
 *
 * Build without hardware: scheduler.c is compiled with -DNDEBUG so all
 * ASSERT macros are no-ops and no ARM-specific code is required.
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "kernel/scheduler.h"

/* ---------- minimal test framework ---------------------------------------- */

static int g_tests_passed = 0;
static int g_tests_failed = 0;

#define TEST_ASSERT(expr) do { \
    if (!(expr)) { \
        printf("  FAIL: %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        g_tests_failed++; \
    } else { \
        g_tests_passed++; \
    } \
} while (0)

#define RUN_TEST(name, func) do { \
    printf("Testing %s ...\n", (name)); \
    (func)(); \
} while (0)

/* ---------- helpers -------------------------------------------------------- */

static volatile uint32_t g_run_count = 0u;

static void counting_task(uint8_t task_id) {
    (void)task_id;
    g_run_count++;
}

static volatile uint32_t g_shared_flag = 0u;

static void flagged_task(uint8_t task_id) {
    (void)task_id;
    g_shared_flag = 1u;
}

/* ---------- test cases ----------------------------------------------------- */

static void test_init(void) {
    scheduler_init();
    const sched_stats_t *s = scheduler_stats();
    TEST_ASSERT(s != NULL);
    TEST_ASSERT(s->creates == 0u);
    TEST_ASSERT(s->kills == 0u);
    TEST_ASSERT(s->runs == 0u);
}

static void test_create_and_query(void) {
    bool alive;

    scheduler_init();

    uint8_t t = scheduler_create(0u, counting_task);
    TEST_ASSERT(t != SCHED_INVALID_TASK);

    TEST_ASSERT(scheduler_query(t, &alive));
    TEST_ASSERT(alive);

    TEST_ASSERT(scheduler_stats()->creates == 1u);
}

static void test_kill(void) {
    bool alive;

    scheduler_init();

    uint8_t t = scheduler_create(0u, counting_task);
    TEST_ASSERT(t != SCHED_INVALID_TASK);

    TEST_ASSERT(scheduler_kill(t));
    TEST_ASSERT(scheduler_query(t, &alive));
    TEST_ASSERT(!alive);

    TEST_ASSERT(scheduler_stats()->kills == 1u);

    /* Killing a dead task returns false */
    TEST_ASSERT(!scheduler_kill(t));
}

static void test_run_once(void) {
    scheduler_init();
    g_run_count = 0u;

    uint8_t t = scheduler_create(0u, counting_task);
    TEST_ASSERT(t != SCHED_INVALID_TASK);

    scheduler_run_once(0u);
    TEST_ASSERT(g_run_count == 1u);

    scheduler_run_once(0u);
    TEST_ASSERT(g_run_count == 2u);

    TEST_ASSERT(scheduler_stats()->runs == 2u);
}

static void test_run_once_core_affinity(void) {
    scheduler_init();
    g_run_count = 0u;

    /* Task pinned to core 1 must not run when core 0 ticks. */
    uint8_t t = scheduler_create(1u, counting_task);
    TEST_ASSERT(t != SCHED_INVALID_TASK);

    scheduler_run_once(0u);
    TEST_ASSERT(g_run_count == 0u);

    scheduler_run_once(1u);
    TEST_ASSERT(g_run_count == 1u);
}

static void test_sleep(void) {
    scheduler_init();
    g_run_count = 0u;

    uint8_t t = scheduler_create(0u, counting_task);
    TEST_ASSERT(t != SCHED_INVALID_TASK);

    TEST_ASSERT(scheduler_sleep(t, 3u));

    /* Task should be blocked for the next 3 ticks. */
    scheduler_run_once(0u);
    TEST_ASSERT(g_run_count == 0u);
    scheduler_run_once(0u);
    TEST_ASSERT(g_run_count == 0u);
    scheduler_run_once(0u);
    TEST_ASSERT(g_run_count == 0u);

    /* On the 4th tick the sleep counter reaches 0 and the task runs. */
    scheduler_run_once(0u);
    TEST_ASSERT(g_run_count == 1u);

    TEST_ASSERT(scheduler_stats()->sleeps >= 1u);
}

static void test_wait(void) {
    scheduler_init();
    g_shared_flag = 0u;

    uint8_t t = scheduler_create(0u, flagged_task);
    TEST_ASSERT(t != SCHED_INVALID_TASK);

    /* Wait until g_shared_flag == 1. */
    TEST_ASSERT(scheduler_wait(t, &g_shared_flag, 1u));

    /* Task must not run while the condition is false (flag stays 0). */
    scheduler_run_once(0u);
    TEST_ASSERT(g_shared_flag == 0u);

    /* Satisfy the condition externally; now the task should run. */
    g_shared_flag = 1u;
    scheduler_run_once(0u);
    /* The flagged_task sets it to 1, but it was already 1 – no change. */
    TEST_ASSERT(g_shared_flag == 1u);
    TEST_ASSERT(scheduler_stats()->runs >= 1u);
}

static void test_yield(void) {
    scheduler_init();

    uint8_t t = scheduler_create(0u, counting_task);
    TEST_ASSERT(t != SCHED_INVALID_TASK);

    scheduler_yield(t);
    TEST_ASSERT(scheduler_stats()->yields == 1u);

    scheduler_yield(t);
    TEST_ASSERT(scheduler_stats()->yields == 2u);
}

static void test_max_tasks(void) {
    uint8_t i;
    uint8_t ids[16];

    scheduler_init();

    for (i = 0u; i < 16u; ++i) {
        ids[i] = scheduler_create(0u, counting_task);
        TEST_ASSERT(ids[i] != SCHED_INVALID_TASK);
    }

    /* Table is full – next create must fail. */
    uint8_t extra = scheduler_create(0u, counting_task);
    TEST_ASSERT(extra == SCHED_INVALID_TASK);

    /* Kill one slot and retry. */
    TEST_ASSERT(scheduler_kill(ids[0]));
    uint8_t reused = scheduler_create(0u, counting_task);
    TEST_ASSERT(reused != SCHED_INVALID_TASK);
}

/* ---------- main ----------------------------------------------------------- */

int main(void) {
    printf("\n");
    printf("========================================\n");
    printf("Scheduler Unit Tests (SPEC §7.2)\n");
    printf("========================================\n\n");

    RUN_TEST("scheduler_init",            test_init);
    RUN_TEST("scheduler_create_query",    test_create_and_query);
    RUN_TEST("scheduler_kill",            test_kill);
    RUN_TEST("scheduler_run_once",        test_run_once);
    RUN_TEST("scheduler_core_affinity",   test_run_once_core_affinity);
    RUN_TEST("scheduler_sleep",           test_sleep);
    RUN_TEST("scheduler_wait",            test_wait);
    RUN_TEST("scheduler_yield",           test_yield);
    RUN_TEST("scheduler_max_tasks",       test_max_tasks);

    printf("\n========================================\n");
    printf("Results: %d passed, %d failed\n", g_tests_passed, g_tests_failed);
    printf("========================================\n");

    return g_tests_failed > 0 ? 1 : 0;
}
