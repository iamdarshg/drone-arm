/*
 * Unit tests for scheduler and async operations
 * Uses lightweight test framework
 */
#include <stdio.h>
#include <string.h>
#include "scheduler.h"
#include "hardware/structs/timer.h"

// Simple test framework
typedef struct {
    const char *name;
    int (*test_func)(void);
} test_case_t;

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST_ASSERT(expr) do { \
    if (!(expr)) { \
        printf("  FAIL: %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        return 1; \
    } \
} while(0)

#define RUN_TEST(name, func) do { \
    printf("Testing %s...\n", name); \
    if (func() == 0) { \
        printf("  PASS\n"); \
        tests_passed++; \
    } else { \
        tests_failed++; \
    } \
} while(0)

// ============================================
// Scheduler Tests
// ============================================

static int test_sched_init(void) {
    sched_init();
    TEST_ASSERT(sched_task_count() == 0);
    return 0;
}

static int test_sched_create(void) {
    sched_init();
    
    void dummy_task(uint8_t id) { (void)id; }
    
    uint8_t task1 = sched_create(dummy_task);
    TEST_ASSERT(task1 != SCHED_INVALID_TASK);
    TEST_ASSERT(sched_task_count() == 1);
    
    uint8_t task2 = sched_create(dummy_task);
    TEST_ASSERT(task2 != SCHED_INVALID_TASK);
    TEST_ASSERT(sched_task_count() == 2);
    
    sched_kill(task1);
    TEST_ASSERT(sched_task_count() == 1);
    
    sched_kill(task2);
    TEST_ASSERT(sched_task_count() == 0);
    
    return 0;
}

static volatile int async_counter = 0;

static void async_test_task(uint8_t task_id) {
    (void)task_id;
    TASK_LOCAL_BEGIN;
    
    async_counter++;
    AWAIT(async_counter > 1);
    async_counter++;
    
    TASK_LOCAL_END;
}

static int test_sched_async(void) {
    sched_init();
    async_counter = 0;
    
    uint8_t task = sched_create(async_test_task);
    TEST_ASSERT(task != SCHED_INVALID_TASK);
    
    // Run scheduler for a few iterations
    for (int i = 0; i < 10 && sched_task_count() > 0; i++) {
        // Manually call task function for testing
        async_test_task(task);
    }
    
    TEST_ASSERT(async_counter >= 2);
    
    return 0;
}

// ============================================
// GPIO Tests
// ============================================

static int test_gpio_init(void) {
    extern void gpio_init_all(void);
    
    gpio_init_all();
    
    // Test that pins are configured correctly
    // In real test, would verify register values
    return 0;
}

// ============================================
// Timer Tests
// ============================================

static int test_timer_read(void) {
    uint64_t t1 = sched_now_us();
    
    // Busy wait a bit
    for (volatile int i = 0; i < 1000; i++);
    
    uint64_t t2 = sched_now_us();
    
    TEST_ASSERT(t2 > t1);
    
    return 0;
}

// ============================================
// Main Test Runner
// ============================================

int main(void) {
    printf("\n");
    printf("========================================\n");
    printf("Drone-Arm Unit Tests\n");
    printf("========================================\n");
    printf("\n");
    
    // Scheduler tests
    printf("--- Scheduler Tests ---\n");
    RUN_TEST("sched_init", test_sched_init);
    RUN_TEST("sched_create", test_sched_create);
    RUN_TEST("sched_async", test_sched_async);
    printf("\n");
    
    // Hardware abstraction tests
    printf("--- Hardware Abstraction Tests ---\n");
    RUN_TEST("gpio_init", test_gpio_init);
    RUN_TEST("timer_read", test_timer_read);
    printf("\n");
    
    // Summary
    printf("========================================\n");
    printf("Test Results: %d passed, %d failed\n", tests_passed, tests_failed);
    printf("========================================\n");
    
    return tests_failed > 0 ? 1 : 0;
}
