#ifndef SYSTEM_MULTICORE_H
#define SYSTEM_MULTICORE_H

#include <stdbool.h>
#include <stdint.h>

typedef void (*core_entry_t)(void);

bool multicore_fifo_push_blocking(uint32_t value);
bool multicore_fifo_pop_blocking(uint32_t *value);
bool multicore_spinlock_acquire(uint32_t lock_num);
void multicore_spinlock_release(uint32_t lock_num);
bool multicore_launch_core1(void);

#endif
