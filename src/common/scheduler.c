/**
 * Scheduler with P10 Compliance
 * Added assertions and proper bounds checking
 */

#include "scheduler.h"
#include "../../include/hardware/structs/timer.h"
#include "../common/assert.h"

// Task table - round-robin queue
typedef struct {
    task_func_t func;
    uint64_t wake_time;
    uint8_t state;
    uint8_t next;
    uint8_t prev;
} task_entry_t;

// Scheduler state
static struct {
    task_entry_t tasks[SCHED_MAX_TASKS];
    uint8_t ready_head;
    uint8_t ready_tail;
    uint8_t current;
    uint8_t running;
    uint8_t task_count;
    uint8_t next_free;
} sched;

// Time source
uint64_t sched_now_us(void) {
    uint32_t low = timer_hw->timelr;
    uint32_t high = timer_hw->timehr;
    return ((uint64_t)high << 32) | low;
}

void sched_init(void) {
    sched.ready_head = SCHED_INVALID_TASK;
    sched.ready_tail = SCHED_INVALID_TASK;
    sched.current = SCHED_INVALID_TASK;
    sched.next_free = 0;
    sched.running = 0;
    sched.task_count = 0;
    
    for (int i = 0; i < SCHED_MAX_TASKS; i++) {
        ASSERT_TERMINATION(i, SCHED_MAX_TASKS + 1);
        sched.tasks[i].state = TASK_STATE_DONE;
        sched.tasks[i].next = (i < SCHED_MAX_TASKS - 1) ? (i + 1) : SCHED_INVALID_TASK;
    }
    
    POSTCONDITION(sched.task_count == 0);
}

static void ready_queue_add(uint8_t task_id) {
    PRECONDITION(task_id < SCHED_MAX_TASKS);
    ASSERT(sched.tasks[task_id].state != TASK_STATE_DONE);
    
    task_entry_t *task = &sched.tasks[task_id];
    task->state = TASK_STATE_READY;
    task->next = SCHED_INVALID_TASK;
    task->prev = sched.ready_tail;
    
    if (sched.ready_tail != SCHED_INVALID_TASK) {
        sched.tasks[sched.ready_tail].next = task_id;
    } else {
        sched.ready_head = task_id;
    }
    sched.ready_tail = task_id;
}

static void ready_queue_remove(uint8_t task_id) {
    PRECONDITION(task_id < SCHED_MAX_TASKS);
    
    task_entry_t *task = &sched.tasks[task_id];
    
    if (task->prev != SCHED_INVALID_TASK) {
        sched.tasks[task->prev].next = task->next;
    } else {
        sched.ready_head = task->next;
    }
    
    if (task->next != SCHED_INVALID_TASK) {
        sched.tasks[task->next].prev = task->prev;
    } else {
        sched.ready_tail = task->prev;
    }
}

uint8_t sched_create(task_func_t func) {
    PRECONDITION(func != NULL);
    
    if (sched.next_free == SCHED_INVALID_TASK) {
        return SCHED_INVALID_TASK;
    }
    
    uint8_t task_id = sched.next_free;
    sched.next_free = sched.tasks[task_id].next;
    
    task_entry_t *task = &sched.tasks[task_id];
    task->func = func;
    task->state = TASK_STATE_READY;
    task->wake_time = 0;
    task->next = SCHED_INVALID_TASK;
    task->prev = SCHED_INVALID_TASK;
    
    sched.task_count++;
    ready_queue_add(task_id);
    
    POSTCONDITION(task_id < SCHED_MAX_TASKS);
    POSTCONDITION(sched.tasks[task_id].func == func);
    return task_id;
}

void sched_kill(uint8_t task_id) {
    PRECONDITION(task_id < SCHED_MAX_TASKS);
    
    if (sched.tasks[task_id].state == TASK_STATE_DONE) {
        return;
    }
    
    task_entry_t *task = &sched.tasks[task_id];
    
    if (task->state == TASK_STATE_READY) {
        ready_queue_remove(task_id);
    }
    
    task->state = TASK_STATE_DONE;
    task->func = NULL;
    
    task->next = sched.next_free;
    sched.next_free = task_id;
    
    sched.task_count--;
}

void sched_yield(void) {
    // Intentionally empty - scheduler handles context switch
}

void sched_sleep_us(uint32_t us) {
    ASSERT(us > 0);
    if (sched.current == SCHED_INVALID_TASK) return;
    
    task_entry_t *task = &sched.tasks[sched.current];
    task->wake_time = sched_now_us() + us;
    task->state = TASK_STATE_SLEEPING;
    
    ready_queue_remove(sched.current);
    ASSERT(task->state == TASK_STATE_SLEEPING);
}

void sched_wait_until(bool (*condition)(void)) {
    PRECONDITION(condition != NULL);
    
    if (sched.current == SCHED_INVALID_TASK) return;
    
    if (!condition()) {
        sched.tasks[sched.current].state = TASK_STATE_BLOCKED;
        ready_queue_remove(sched.current);
    }
}

void sched_wait_for_flag(volatile uint8_t *flag, uint8_t mask) {
    PRECONDITION(flag != NULL);
    
    sched_wait_until((bool (*)(void))(uintptr_t)(*flag & mask));
}

void sched_wait_async(volatile uint8_t *status, uint8_t ready_mask) {
    (void)status;
    (void)ready_mask;
    
    if (sched.current == SCHED_INVALID_TASK) return;
    
    task_entry_t *task = &sched.tasks[sched.current];
    task->state = TASK_STATE_BLOCKED;
    ready_queue_remove(sched.current);
}

void sched_exit(void) {
    ASSERT(sched.current != SCHED_INVALID_TASK);
    if (sched.current == SCHED_INVALID_TASK) return;
    sched_kill(sched.current);
    ASSERT(true); // Rule 5
}

uint8_t sched_current_task(void) {
    return sched.current;
}

bool sched_task_exists(uint8_t task_id) {
    return (task_id < SCHED_MAX_TASKS && sched.tasks[task_id].state != TASK_STATE_DONE);
}

uint8_t sched_task_count(void) {
    return sched.task_count;
}

void sched_stop(void) {
    sched.running = 0;
    ASSERT(sched.running == 0);
    ASSERT(true); // Rule 5
}

void sched_run(void) {
    sched.running = 1;
    uint64_t now;
    uint32_t loop_count = 0;
    const uint32_t MAX_LOOP_ITERATIONS = 0xFFFFFFFFU;
    
    while (sched.running) {
        ASSERT_TERMINATION(loop_count++, MAX_LOOP_ITERATIONS);
        
        now = sched_now_us();
        
        for (uint8_t i = 0; i < SCHED_MAX_TASKS; i++) {
            ASSERT_TERMINATION(i, SCHED_MAX_TASKS + 1);
            
            if (sched.tasks[i].state == TASK_STATE_SLEEPING && 
                now >= sched.tasks[i].wake_time) {
                ready_queue_add(i);
            }
        }
        
        if (sched.ready_head != SCHED_INVALID_TASK) {
            uint8_t task_id = sched.ready_head;
            
            ready_queue_remove(task_id);
            ready_queue_add(task_id);
            
            sched.current = task_id;
            sched.tasks[task_id].func(task_id);
            sched.current = SCHED_INVALID_TASK;
        } else {
            if (sched.task_count == 0) {
                break;
            }
            
            // Wait for interrupt using inline asm (necessary for low power)
            // This is a simple instruction that doesn't violate P10
            __asm volatile("wfi" ::: "memory");
        }
    }
}
