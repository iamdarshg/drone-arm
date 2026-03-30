#include "kernel/assert.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "kernel/scheduler.h"

enum {
    MAX_TASKS = 16u,
    MAX_CORES = 2u,
    INVALID_TASK = 0xFFu,
};

typedef struct {
    bool used;
    uint8_t core;
    task_fn_t fn;
    uint32_t sleep_ticks;
    volatile uint32_t *wait_addr;
    uint32_t wait_expected;
} task_t;

static task_t g_tasks[MAX_TASKS];
static uint8_t g_rr[MAX_CORES];
static sched_stats_t g_stats;

void scheduler_init(void) {
    uint32_t i;
    for (i = 0u; i < MAX_TASKS; ++i) {
        g_tasks[i].used = false;
        g_tasks[i].fn = NULL;
        g_tasks[i].sleep_ticks = 0u;
        g_tasks[i].wait_addr = NULL;
        g_tasks[i].wait_expected = 0u;
        g_tasks[i].core = 0u;
    }
    g_rr[0] = 0u;
    g_rr[1] = 0u;
    g_stats = (sched_stats_t){0};
}

uint8_t scheduler_create(uint8_t core_id, task_fn_t fn) {
    uint8_t i;
    ASSERT(core_id < MAX_CORES);
    ASSERT(fn != NULL);
    for (i = 0u; i < MAX_TASKS; ++i) {
        if (!g_tasks[i].used) {
            g_tasks[i].used = true;
            g_tasks[i].core = core_id;
            g_tasks[i].fn = fn;
            g_tasks[i].sleep_ticks = 0u;
            g_tasks[i].wait_addr = NULL;
            g_tasks[i].wait_expected = 0u;
            g_stats.creates++;
            return i;
        }
    }
    return INVALID_TASK;
}

bool scheduler_kill(uint8_t task_id) {
    ASSERT(task_id < MAX_TASKS);
    if (!g_tasks[task_id].used) {
        return false;
    }
    g_tasks[task_id].used = false;
    g_stats.kills++;
    return true;
}

bool scheduler_query(uint8_t task_id, bool *alive) {
    ASSERT(task_id < MAX_TASKS);
    ASSERT(alive != NULL);
    *alive = g_tasks[task_id].used;
    return true;
}

bool scheduler_sleep(uint8_t task_id, uint32_t ticks) {
    ASSERT(task_id < MAX_TASKS);
    if (!g_tasks[task_id].used) {
        return false;
    }
    g_tasks[task_id].sleep_ticks = ticks;
    g_stats.sleeps++;
    return true;
}

bool scheduler_wait(uint8_t task_id, volatile uint32_t *addr, uint32_t expected) {
    ASSERT(task_id < MAX_TASKS);
    ASSERT(addr != NULL);
    if (!g_tasks[task_id].used) {
        return false;
    }
    g_tasks[task_id].wait_addr = addr;
    g_tasks[task_id].wait_expected = expected;
    g_stats.waits++;
    return true;
}

void scheduler_yield(uint8_t task_id) {
    ASSERT(task_id < MAX_TASKS);
    (void)task_id;
    g_stats.yields++;
}

static bool task_ready(uint8_t id) {
    if (g_tasks[id].sleep_ticks > 0u) {
        g_tasks[id].sleep_ticks--;
        return false;
    }
    if (g_tasks[id].wait_addr != NULL) {
        if (*(g_tasks[id].wait_addr) != g_tasks[id].wait_expected) {
            return false;
        }
        g_tasks[id].wait_addr = NULL;
    }
    return true;
}

void scheduler_run_once(uint8_t core_id) {
    uint8_t n;
    uint8_t start;
    ASSERT(core_id < MAX_CORES);
    start = g_rr[core_id];
    for (n = 0u; n < MAX_TASKS; ++n) {
        uint8_t id;
        id = (uint8_t)((start + n) % MAX_TASKS);
        if (g_tasks[id].used && g_tasks[id].core == core_id && task_ready(id)) {
            g_tasks[id].fn(id);
            g_rr[core_id] = (uint8_t)((id + 1u) % MAX_TASKS);
            g_stats.runs++;
            break;
        }
    }
}

const sched_stats_t *scheduler_stats(void) {
    return &g_stats;
}
