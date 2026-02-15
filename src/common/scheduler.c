/*
 * Ultra-lightweight cooperative task scheduler implementation
 * ~200 bytes RAM, ~800 bytes flash
 */
#include "scheduler.h"
#include "../../include/hardware/structs/timer.h"

// Task table - round-robin queue
typedef struct {
    task_func_t func;           // Task function pointer
    uint64_t wake_time;         // Wake time for sleeping tasks
    uint8_t state;              // Task state
    uint8_t next;               // Next in linked list
    uint8_t prev;               // Previous in linked list
} task_entry_t;

// Scheduler state
static struct {
    task_entry_t tasks[SCHED_MAX_TASKS];
    uint8_t ready_head;         // First ready task
    uint8_t ready_tail;         // Last ready task
    uint8_t current;            // Currently running task
    uint8_t running;            // Scheduler running flag
    uint8_t task_count;         // Number of active tasks
    uint8_t next_free;          // Next free slot
} sched;

// Time source - uses timer0
uint64_t sched_now_us(void) {
    // Read 64-bit timer atomically
    uint32_t low = timer_hw->timelr;
    uint32_t high = timer_hw->timehr;
    return ((uint64_t)high << 32) | low;
}

// Initialize scheduler
void sched_init(void) {
    memset(&sched, 0, sizeof(sched));
    sched.ready_head = SCHED_INVALID_TASK;
    sched.ready_tail = SCHED_INVALID_TASK;
    sched.current = SCHED_INVALID_TASK;
    sched.next_free = 0;
    
    // Mark all tasks as free
    for (int i = 0; i < SCHED_MAX_TASKS; i++) {
        sched.tasks[i].state = TASK_STATE_DONE;
        sched.tasks[i].next = (i < SCHED_MAX_TASKS - 1) ? (i + 1) : SCHED_INVALID_TASK;
    }
}

// Internal: Add task to ready queue
static void ready_queue_add(uint8_t task_id) {
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

// Internal: Remove task from ready queue
static void ready_queue_remove(uint8_t task_id) {
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

// Create a new task
uint8_t sched_create(task_func_t func) {
    if (sched.next_free == SCHED_INVALID_TASK) {
        return SCHED_INVALID_TASK;  // No free slots
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
    
    return task_id;
}

// Kill a task
void sched_kill(uint8_t task_id) {
    if (task_id >= SCHED_MAX_TASKS || sched.tasks[task_id].state == TASK_STATE_DONE) {
        return;
    }
    
    task_entry_t *task = &sched.tasks[task_id];
    
    // Remove from ready queue if present
    if (task->state == TASK_STATE_READY) {
        ready_queue_remove(task_id);
    }
    
    task->state = TASK_STATE_DONE;
    task->func = NULL;
    
    // Add to free list
    task->next = sched.next_free;
    sched.next_free = task_id;
    
    sched.task_count--;
}

// Yield control
void sched_yield(void) {
    // Just return - scheduler will continue to next task
}

// Sleep for microseconds
void sched_sleep_us(uint32_t us) {
    if (sched.current == SCHED_INVALID_TASK) return;
    
    task_entry_t *task = &sched.tasks[sched.current];
    task->wake_time = sched_now_us() + us;
    task->state = TASK_STATE_SLEEPING;
    
    // Remove from ready queue
    ready_queue_remove(sched.current);
}

// Wait for condition
void sched_wait_until(bool (*condition)(void)) {
    if (sched.current == SCHED_INVALID_TASK) return;
    
    if (!condition()) {
        // Block and yield
        sched.tasks[sched.current].state = TASK_STATE_BLOCKED;
        ready_queue_remove(sched.current);
    }
}

// Wait for flag
void sched_wait_for_flag(volatile uint8_t *flag, uint8_t mask) {
    sched_wait_until((bool (*)(void))(uintptr_t)(*flag & mask));
}

// Wait for async completion
void sched_wait_async(volatile uint8_t *status, uint8_t ready_mask) {
    if (sched.current == SCHED_INVALID_TASK) return;
    
    task_entry_t *task = &sched.tasks[sched.current];
    task->state = TASK_STATE_BLOCKED;
    ready_queue_remove(sched.current);
}

// Exit current task
void sched_exit(void) {
    if (sched.current == SCHED_INVALID_TASK) return;
    sched_kill(sched.current);
}

// Get current task ID
uint8_t sched_current_task(void) {
    return sched.current;
}

// Check if task exists
bool sched_task_exists(uint8_t task_id) {
    return (task_id < SCHED_MAX_TASKS && sched.tasks[task_id].state != TASK_STATE_DONE);
}

// Get active task count
uint8_t sched_task_count(void) {
    return sched.task_count;
}

// Stop scheduler
void sched_stop(void) {
    sched.running = 0;
}

// Main scheduler loop
void sched_run(void) {
    sched.running = 1;
    uint64_t now;
    
    while (sched.running) {
        // Check for sleeping tasks that should wake up
        now = sched_now_us();
        for (uint8_t i = 0; i < SCHED_MAX_TASKS; i++) {
            if (sched.tasks[i].state == TASK_STATE_SLEEPING && 
                now >= sched.tasks[i].wake_time) {
                ready_queue_add(i);
            }
        }
        
        // Run ready tasks in round-robin
        if (sched.ready_head != SCHED_INVALID_TASK) {
            uint8_t task_id = sched.ready_head;
            
            // Move to end of queue (round-robin)
            ready_queue_remove(task_id);
            ready_queue_add(task_id);
            
            sched.current = task_id;
            sched.tasks[task_id].func(task_id);
            sched.current = SCHED_INVALID_TASK;
        } else {
            // No tasks ready - check if any exist
            if (sched.task_count == 0) {
                break;  // All done
            }
            
            // Idle - wait for next timer interrupt or event
            // On ARM Cortex-M, can use WFI here
            __asm volatile("wfi" ::: "memory");
        }
    }
}
