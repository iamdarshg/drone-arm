#ifndef KERNEL_SCHEDULER_H
#define KERNEL_SCHEDULER_H

#include <stdbool.h>
#include <stdint.h>

typedef void (*task_fn_t)(uint8_t task_id);

typedef struct {
    uint32_t creates;
    uint32_t kills;
    uint32_t yields;
    uint32_t sleeps;
    uint32_t waits;
    uint32_t runs;
} sched_stats_t;

void scheduler_init(void);
uint8_t scheduler_create(uint8_t core_id, task_fn_t fn);
bool scheduler_kill(uint8_t task_id);
bool scheduler_query(uint8_t task_id, bool *alive);
bool scheduler_sleep(uint8_t task_id, uint32_t ticks);
bool scheduler_wait(uint8_t task_id, volatile uint32_t *addr, uint32_t expected);
void scheduler_yield(uint8_t task_id);
void scheduler_run_once(uint8_t core_id);
const sched_stats_t *scheduler_stats(void);

#define SCHED_INVALID_TASK ((uint8_t)0xFFu)

#endif
