#ifndef SYSTEM_MULTICORE_H
#define SYSTEM_MULTICORE_H

#include <stdbool.h>
#include <stdint.h>

typedef void (*core_entry_t)(void);

bool multicore_fifo_push_blocking(uint32_t value);
bool multicore_fifo_pop_blocking(uint32_t *value);
bool multicore_spinlock_acquire(uint32_t lock_num);
void multicore_spinlock_release(uint32_t lock_num);

/*
 * Launch Core1 using the RP2350 ROM's 6-step FIFO boot handshake.
 *
 * entry     – function Core1 will enter after launch.
 * stack_top – pointer to the top of Core1's stack (must be 8-byte aligned).
 */
bool multicore_launch_core1(core_entry_t entry, uint32_t *stack_top);

#endif
