 ```
#ifndef MULTICORE_RP2350B_H
#define MULTICORE_RP2350B_H

#include <stdint.h>
#include <stdbool.h>

// Core identifiers for RP2350B (dual M33 cores)
#define CORE0_ID                0
#define CORE1_ID                1
#define NUM_CORES               2

// Inter-core FIFO addresses (RP2350B specific)
#define SIO_FIFO_ST_VLD         0x00000001
#define SIO_FIFO_RDY            0x00000002
#define SIO_FIFO_ROE            0x00000010
#define SIO_FIFO_WOF            0x00000020

// Message types for inter-core communication
#define MSG_TYPE_TASK           0x01
#define MSG_TYPE_DATA           0x02
#define MSG_TYPE_SYNC           0x03

// Platform-specific multicore functions for RP2350B

// Initialize the multicore system (call on core 0 before launching core 1)
void multicore_rp2350B_init(void);

// Launch core 1 with a given entry point
void multicore_rp2350B_launch_core1(void (*entry)(void));

// Get the ID of the current core (0 or 1)
uint32_t multicore_rp2350B_get_core_id(void);

// Check if the current core is core 0
static inline bool multicore_rp2350B_is_core0(void) {
    return multicore_rp2350B_get_core_id() == CORE0_ID;
}

// Check if the current core is core 1
static inline bool multicore_rp2350B_is_core1(void) {
    return multicore_rp2350B_get_core_id() == CORE1_ID;
}

// Send a 32-bit value to the other core via hardware FIFO (non-blocking)
bool multicore_rp2350B_fifo_push_blocking(uint32_t data);

// Receive a 32-bit value from the other core via hardware FIFO (blocking)
uint32_t multicore_rp2350B_fifo_pop_blocking(void);

// Check if FIFO has data available
bool multicore_rp2350B_fifo_available(void);

// Clear the FIFO (reset both cores)
void multicore_rp2350B_fifo_clear(void);

// Send a message to a specific core (with type and data)
bool multicore_rp2350B_send_message(uint8_t target_core, uint8_t msg_type, uint32_t data);

// Receive a message (returns message type, stores data in pointer)
uint8_t multicore_rp2350B_receive_message(uint32_t *data);

// Software reset of the other core
void multicore_rp2350B_reset_core1(void);

// Set the vector table offset for core 1 (if using different vector table)
void multicore_rp2350B_set_core1_vector_table(uint32_t offset);

// Spinlock functions for synchronization between cores
typedef uint32_t spinlock_t;

// Initialize a spinlock
void multicore_rp2350B_spinlock_init(spinlock_t *lock);

// Acquire a spinlock (blocking)
void multicore_rp2350B_spinlock_acquire(spinlock_t *lock);

// Release a spinlock
void multicore_rp2350B_spinlock_release(spinlock_t *lock);

// Try to acquire a spinlock (non-blocking)
bool multicore_rp2350B_spinlock_try_acquire(spinlock_t *lock);

// Memory barrier to ensure ordering between cores
static inline void multicore_rp2350B_memory_barrier(void) {
    __asm__ volatile("dmb" ::: "memory");
}

// Data synchronization barrier
static inline void multicore_rp2350B_data_sync_barrier(void) {
    __asm__ volatile("dsb" ::: "memory");
}

// Instruction synchronization barrier
static inline void multicore_rp2350B_instruction_sync_barrier(void) {
    __asm__ volatile("isb" ::: "memory");
}

#endif // MULTICORE_RP2350B_H
