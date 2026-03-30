#include "kernel/assert.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "kernel/scheduler.h"
#include "hal/platform.h"

enum {
    MAX_TASKS = 16u,
    MAX_CORES = 2u,
    INVALID_TASK = 0xFFu,
    MAX_SHARED_REGIONS = 8u,
};

typedef struct {
    bool used;
    uint8_t core;
    task_fn_t fn;
    uint32_t sleep_ticks;
    volatile uint32_t *wait_addr;
    uint32_t wait_expected;
} task_t;

typedef struct {
    bool used;
    uintptr_t base;
    size_t size;
} memory_region_t;

static task_t g_tasks[MAX_TASKS];
static uint8_t g_rr[MAX_CORES];
static sched_stats_t g_stats;
static uint8_t g_current_task[MAX_CORES];
static memory_region_t g_task_regions[MAX_TASKS];
static memory_region_t g_shared_regions[MAX_SHARED_REGIONS];
static uint32_t g_clock_throughput_hints[MAX_TASKS];

static void scheduler_mpu_apply_task(uint8_t task_id);

#if defined(__ARM_ARCH) || defined(__thumb__) || defined(__arm__)
enum {
    MPU_BASE_ADDR = 0xE000ED90u,
    MPU_TYPE      = 0x00u,
    MPU_CTRL      = 0x04u,
    MPU_RNR       = 0x08u,
    MPU_RBAR      = 0x0Cu,
    MPU_RLAR      = 0x10u,
    MPU_MAIR0     = 0x30u,
    MPU_REGION_ALIGN_MASK = 0x1Fu,
    MPU_RLAR_ATTRINDX_SHIFT = 1u,
    MPU_RLAR_ENABLE = 1u,
    MPU_MAX_REGIONS = 8u,
};
static const uint32_t MPU_MAIR_NORMAL_NONCACHEABLE = 0x44u;
#endif

static bool region_contains(const memory_region_t *r, uintptr_t addr, size_t size) {
    uintptr_t end;
    uintptr_t rend;
    ASSERT(r != NULL);
    if (!r->used || size == 0u) {
        return false;
    }
    if (size > (size_t)(UINTPTR_MAX - addr)) {
        return false;
    }
    if (r->size > (size_t)(UINTPTR_MAX - r->base)) {
        return false;
    }
    end = addr + size;
    rend = r->base + r->size;
    return addr >= r->base && end <= rend;
}

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
    g_current_task[0] = SCHED_INVALID_TASK;
    g_current_task[1] = SCHED_INVALID_TASK;
    for (i = 0u; i < MAX_TASKS; ++i) {
        g_task_regions[i].used = false;
        g_task_regions[i].base = 0u;
        g_task_regions[i].size = 0u;
    }
    for (i = 0u; i < MAX_SHARED_REGIONS; ++i) {
        g_shared_regions[i].used = false;
        g_shared_regions[i].base = 0u;
        g_shared_regions[i].size = 0u;
    }
    for (i = 0u; i < MAX_TASKS; ++i) {
        g_clock_throughput_hints[i] = 0u;
    }
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
    g_task_regions[task_id].used = false;
    if (g_tasks[task_id].core < MAX_CORES &&
        g_current_task[g_tasks[task_id].core] == task_id) {
        g_current_task[g_tasks[task_id].core] = SCHED_INVALID_TASK;
    }
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
            g_current_task[core_id] = id;
            scheduler_mpu_apply_task(id);
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

bool scheduler_set_task_region(uint8_t task_id, uintptr_t base, size_t size) {
    ASSERT(task_id < MAX_TASKS);
    if (!g_tasks[task_id].used || size == 0u) {
        return false;
    }
    if (size > (size_t)(UINTPTR_MAX - base)) {
        return false;
    }
    g_task_regions[task_id].used = true;
    g_task_regions[task_id].base = base;
    g_task_regions[task_id].size = size;
    return true;
}

bool scheduler_add_shared_region(uintptr_t base, size_t size) {
    uint8_t i;
    if (size == 0u || (size > (size_t)(UINTPTR_MAX - base))) {
        return false;
    }
    for (i = 0u; i < MAX_SHARED_REGIONS; ++i) {
        if (!g_shared_regions[i].used) {
            g_shared_regions[i].used = true;
            g_shared_regions[i].base = base;
            g_shared_regions[i].size = size;
            return true;
        }
    }
    return false;
}

void scheduler_clear_shared_regions(void) {
    uint8_t i;
    for (i = 0u; i < MAX_SHARED_REGIONS; ++i) {
        g_shared_regions[i].used = false;
        g_shared_regions[i].base = 0u;
        g_shared_regions[i].size = 0u;
    }
}

bool scheduler_memory_access_allowed(uint8_t task_id, uintptr_t addr, size_t size) {
    uint8_t i;
    ASSERT(task_id < MAX_TASKS);
    if (!g_tasks[task_id].used || size == 0u) {
        return false;
    }
    if (region_contains(&g_task_regions[task_id], addr, size)) {
        return true;
    }
    for (i = 0u; i < MAX_SHARED_REGIONS; ++i) {
        if (region_contains(&g_shared_regions[i], addr, size)) {
            return true;
        }
    }
    return false;
}

bool scheduler_memory_access_allowed_current(uint8_t core_id, uintptr_t addr, size_t size) {
    uint8_t tid;
    ASSERT(core_id < MAX_CORES);
    tid = g_current_task[core_id];
    if (tid == SCHED_INVALID_TASK) {
        return false;
    }
    return scheduler_memory_access_allowed(tid, addr, size);
}

void scheduler_clock_policy_hint(uint8_t task_id, uint32_t throughput_hint) {
    ASSERT(task_id < MAX_TASKS);
    g_clock_throughput_hints[task_id] = throughput_hint;
}

void scheduler_clock_manager_tick(void) {
    /* Stub only: future implementation should aggregate throughput hints
     * and tune SYS/PERI clocks + voltage through clock driver policies. */
}

bool scheduler_clock_management_supported(void) {
    return false;
}

static bool scheduler_mpu_hw_present(void) {
#if defined(__ARM_ARCH) || defined(__thumb__) || defined(__arm__)
    uint32_t type = REG_RO(MPU_BASE_ADDR + MPU_TYPE);
    return ((type >> 8u) & 0xFFu) != 0u;
#else
    return false;
#endif
}

static void scheduler_mpu_apply_task(uint8_t task_id) {
#if defined(__ARM_ARCH) || defined(__thumb__) || defined(__arm__)
    uint8_t i;
    uint32_t region = 0u;
    if (!scheduler_mpu_hw_present() || task_id >= MAX_TASKS) {
        return;
    }
    /* Minimal MPU programming stub for RP2350/Cortex-M33:
     * region0 = current task private window, following regions = shared windows.
     * Attribute/memory-type policy is intentionally minimal for now. */
    REG_RW(MPU_BASE_ADDR + MPU_CTRL) = 0u;
    /* AttrIdx0 in MAIR0 => Normal memory, Inner/Outer non-cacheable. */
    REG_RW(MPU_BASE_ADDR + MPU_MAIR0) = MPU_MAIR_NORMAL_NONCACHEABLE;
    if (g_task_regions[task_id].used) {
        uintptr_t base = g_task_regions[task_id].base & ~(uintptr_t)MPU_REGION_ALIGN_MASK;
        uintptr_t limit = (g_task_regions[task_id].base + g_task_regions[task_id].size - 1u) &
                          ~(uintptr_t)MPU_REGION_ALIGN_MASK;
        REG_RW(MPU_BASE_ADDR + MPU_RNR) = region++;
        REG_RW(MPU_BASE_ADDR + MPU_RBAR) = (uint32_t)base;
        REG_RW(MPU_BASE_ADDR + MPU_RLAR) =
            ((uint32_t)limit & ~MPU_REGION_ALIGN_MASK) |
            (0u << MPU_RLAR_ATTRINDX_SHIFT) |
            MPU_RLAR_ENABLE;
    }
    for (i = 0u; i < MAX_SHARED_REGIONS && region < MPU_MAX_REGIONS; ++i) {
        if (g_shared_regions[i].used) {
            uintptr_t base = g_shared_regions[i].base & ~(uintptr_t)MPU_REGION_ALIGN_MASK;
            uintptr_t limit = (g_shared_regions[i].base + g_shared_regions[i].size - 1u) &
                              ~(uintptr_t)MPU_REGION_ALIGN_MASK;
            REG_RW(MPU_BASE_ADDR + MPU_RNR) = region++;
            REG_RW(MPU_BASE_ADDR + MPU_RBAR) = (uint32_t)base;
            REG_RW(MPU_BASE_ADDR + MPU_RLAR) =
                ((uint32_t)limit & ~MPU_REGION_ALIGN_MASK) |
                (0u << MPU_RLAR_ATTRINDX_SHIFT) |
                MPU_RLAR_ENABLE;
        }
    }
    REG_RW(MPU_BASE_ADDR + MPU_CTRL) = 1u; /* enable MPU */
    __dsb();
    __isb();
#else
    (void)scheduler_mpu_hw_present();
    (void)task_id;
#endif
}
