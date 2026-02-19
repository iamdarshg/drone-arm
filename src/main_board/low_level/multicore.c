#include "multicore.h"
#include "../../../../include/hardware/multicore_rp2350B.h"
#include "../../../src/common/assert.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// ============================================================================
// RP2350B Hardware Register Definitions
// ============================================================================

// SIO (Single-cycle I/O) Registers for inter-core communication
typedef struct {
    volatile uint32_t cpuid;        // 0x000 - CPU ID register
    volatile uint32_t gpio_in;      // 0x004 - GPIO input
    volatile uint32_t gpio_hi_in;   // 0x008 - GPIO high input
    uint32_t _pad0[4];              // 0x00C-0x018
    volatile uint32_t gpio_out;     // 0x01C - GPIO output
    volatile uint32_t gpio_set;     // 0x020 - GPIO output set
    volatile uint32_t gpio_clr;     // 0x024 - GPIO output clear
    volatile uint32_t gpio_xor;     // 0x028 - GPIO output XOR
    volatile uint32_t gpio_oe;      // 0x02C - GPIO output enable
    volatile uint32_t gpio_oe_set;  // 0x030 - GPIO output enable set
    volatile uint32_t gpio_oe_clr;  // 0x034 - GPIO output enable clear
    volatile uint32_t gpio_hi_out;  // 0x038 - GPIO high output
    volatile uint32_t gpio_hi_set;  // 0x03C - GPIO high set
    volatile uint32_t gpio_hi_clr;  // 0x040 - GPIO high clear
    volatile uint32_t gpio_hi_xor;  // 0x044 - GPIO high XOR
    volatile uint32_t gpio_hi_oe;   // 0x048 - GPIO high output enable
    volatile uint32_t gpio_hi_oe_set;// 0x04C - GPIO high output enable set
    volatile uint32_t gpio_hi_oe_clr;// 0x050 - GPIO high output enable clear
    uint32_t _pad1[15];             // 0x054-0x08C
    volatile uint32_t fifo_st;      // 0x090 - FIFO status
    volatile uint32_t fifo_wr;      // 0x094 - FIFO write
    volatile uint32_t fifo_rd;      // 0x098 - FIFO read
    uint32_t _pad2[1];              // 0x09C
    volatile uint32_t spinlock[32]; // 0x100-0x17C - Spinlock array
} sio_hw_t;

// Base address for SIO (Single-cycle I/O)
#define SIO_BASE 0xD0000000

// SIO hardware instance
#define sio_hw ((sio_hw_t *)SIO_BASE)

// FIFO status bits
#define SIO_FIFO_ST_VLD         (1u << 0)   // FIFO is valid (has data)
#define SIO_FIFO_ST_RDY         (1u << 1)   // FIFO is ready (can accept data)
#define SIO_FIFO_ST_ROE         (1u << 4)   // Read overrun error
#define SIO_FIFO_ST_WOF         (1u << 5)   // Write overflow error

// ============================================================================
// Private State Variables
// ============================================================================

static volatile uint32_t core1_status = 0;           // Track core 1 status
static volatile multicore_task_func_t core1_task = NULL; // Active task on core 1
static volatile void* core1_task_arg = NULL;         // Task argument
static multicore_spinlock_t state_lock = 0;           // Global state lock

// ============================================================================
// Hardware Initialization
// ============================================================================

void multicore_rp2350B_init(void) {
    PRECONDITION(state_lock == 0);  // Lock should be initialized to 0
    
    // Clear FIFO errors (Rule 6: Use simple initialization)
    sio_hw->fifo_st = (SIO_FIFO_ST_ROE | SIO_FIFO_ST_WOF);
    
    // Drain any existing data in FIFO
    while ((sio_hw->fifo_st & SIO_FIFO_ST_VLD) != 0) {
        volatile uint32_t dummy = sio_hw->fifo_rd;
        (void)dummy;
    }
    
    // Initialize spinlock
    multicore_spinlock_init(&state_lock);
    
    ASSERT((sio_hw->fifo_st & SIO_FIFO_ST_VLD) == 0);  // FIFO should be empty
    POSTCONDITION(state_lock == 0);  // Lock should remain unlocked
}

uint32_t multicore_rp2350B_get_core_id(void) {
    uint32_t core_id = sio_hw->cpuid;
    ASSERT_RANGE(core_id, CORE0_ID, CORE1_ID);  // Must be 0 or 1
    return core_id;
}

// ============================================================================
// FIFO Operations
// ============================================================================

static bool fifo_push(uint32_t data) {
    // Check if FIFO is ready to accept data
    if ((sio_hw->fifo_st & SIO_FIFO_ST_RDY) != 0) {
        sio_hw->fifo_wr = data;
        return true;
    }
    return false;
}

static bool fifo_pop(uint32_t *out) {
    PRECONDITION_NOT_NULL(out);
    
    // Check if FIFO has valid data
    if ((sio_hw->fifo_st & SIO_FIFO_ST_VLD) != 0) {
        *out = sio_hw->fifo_rd;
        return true;
    }
    return false;
}

// ============================================================================
// Message Passing
// ============================================================================

bool multicore_send_message_blocking(uint8_t target_core, uint32_t message) {
    PRECONDITION(target_core == CORE0_ID || target_core == CORE1_ID);
    PRECONDITION(target_core != multicore_get_current_core_id());  // Can't send to self
    
    // Wait for FIFO to be ready, with timeout protection
    uint32_t timeout_counter = 0;
    while (!fifo_push(message)) {
        timeout_counter++;
        ASSERT_TERMINATION(timeout_counter, 1000000);  // Prevent infinite loop
    }
    
    POSTCONDITION((sio_hw->fifo_st & SIO_FIFO_ST_WOF) == 0);  // No overflow
    return true;
}

uint32_t multicore_receive_message_blocking(uint8_t* sender_core) {
    PRECONDITION_NOT_NULL(sender_core);
    
    // Wait for message with timeout protection
    uint32_t timeout_counter = 0;
    uint32_t message;
    while (!fifo_pop(&message)) {
        timeout_counter++;
        ASSERT_TERMINATION(timeout_counter, 1000000);  // Rule 2: Must terminate
    }
    
    // Determine sender based on current core (dual-core system)
    *sender_core = (multicore_get_current_core_id() == CORE0_ID) ? CORE1_ID : CORE0_ID;
    
    POSTCONDITION((sio_hw->fifo_st & SIO_FIFO_ST_ROE) == 0);  // No read error
    return message;
}

bool multicore_try_send_message(uint8_t target_core, uint32_t message) {
    PRECONDITION(target_core == CORE0_ID || target_core == CORE1_ID);
    PRECONDITION(target_core != multicore_get_current_core_id());
    
    return fifo_push(message);
}

bool multicore_message_available(void) {
    return (sio_hw->fifo_st & SIO_FIFO_ST_VLD) != 0;
}

bool multicore_try_receive_message(uint32_t* message, uint8_t* sender_core) {
    PRECONDITION_NOT_NULL(message);
    PRECONDITION_NOT_NULL(sender_core);
    
    if (!fifo_pop(message)) {
        return false;
    }
    
    *sender_core = (multicore_get_current_core_id() == CORE0_ID) ? CORE1_ID : CORE0_ID;
    return true;
}

// ============================================================================
// Spinlock Implementation
// ============================================================================

void multicore_spinlock_init(multicore_spinlock_t* lock) {
    PRECONDITION_NOT_NULL(lock);
    
    *lock = 0;
    
    POSTCONDITION(*lock == 0);
}

void multicore_spinlock_acquire(multicore_spinlock_t* lock) {
    PRECONDITION_NOT_NULL(lock);
    
    uint32_t core_id = multicore_get_current_core_id();
    ASSERT_RANGE(core_id, CORE0_ID, CORE1_ID);
    
    // Only use one hardware spinlock for simplicity
    volatile uint32_t* hw_spinlock = &sio_hw->spinlock[0];
    
    // Try to acquire (Rule 2: Must terminate)
    uint32_t attempt_count = 0;
    while (true) {
        // Write our core ID to claim the lock
        *hw_spinlock = core_id + 1;
        
        // Read back to verify we got it
        if (*hw_spinlock == (core_id + 1)) {
            break;
        }
        
        attempt_count++;
        ASSERT_TERMINATION(attempt_count, 100000);  // Shouldn't spin forever
    }
    
    multicore_memory_barrier();
    POSTCONDITION(*lock == core_id + 1);
}

bool multicore_spinlock_try_acquire(multicore_spinlock_t* lock) {
    PRECONDITION_NOT_NULL(lock);
    
    uint32_t core_id = multicore_get_current_core_id();
    volatile uint32_t* hw_spinlock = &sio_hw->spinlock[0];
    
    *hw_spinlock = core_id + 1;
    bool acquired = (*hw_spinlock == (core_id + 1));
    
    if (acquired) {
        multicore_memory_barrier();
    }
    
    POSTCONDITION(!acquired || *lock == core_id + 1);
    return acquired;
}

void multicore_spinlock_release(multicore_spinlock_t* lock) {
    PRECONDITION_NOT_NULL(lock);
    
    multicore_memory_barrier();
    sio_hw->spinlock[0] = 0;
    
    POSTCONDITION(sio_hw->spinlock[0] == 0);
}

// ============================================================================
// Core Launching
// ============================================================================

static void multicore_idle_loop(void) {
    // Safe idle state for unused cores
    while (1) {
        // Minimal assembly - single instruction
        __asm__ volatile("wfi" ::: "memory");
    }
}

bool multicore_launch_core1(void (*entry)(void)) {
    PRECONDITION_NOT_NULL(entry);
    PRECONDITION(multicore_get_current_core_id() == CORE0_ID);  // Must call from core 0
    PRECONDITION(core1_status == 0);  // Core 1 not already running
    
    core1_task = entry;
    core1_task_arg = NULL;
    core1_status = 1;  // Mark as launching
    
    multicore_rp2350B_launch_core1(multicore_idle_loop);  // Start with safe loop
    
    // Wait for core 1 to report ready
    uint32_t timeout = 0;
    while (core1_status != 2) {
        timeout++;
        ASSERT_TERMINATION(timeout, 10000);  // Rule 2: Must terminate
    }
    
    POSTCONDITION(core1_status == 2);  // Core 1 is running
    return true;
}

bool multicore_core1_is_running(void) {
    return (core1_status == 2);
}

// ============================================================================
// Task Launching
// ============================================================================

bool multicore_launch_task_on_core(uint8_t core_id, multicore_task_func_t task_func, void* arg) {
    PRECONDITION(core_id == CORE0_ID || core_id == CORE1_ID);
    PRECONDITION_NOT_NULL(task_func);
    
    uint32_t current_core = multicore_get_current_core_id();
    
    // Execute on current core
    if (core_id == current_core) {
        task_func(arg);
        return true;
    }
    
    // Send task request to other core
    // Simple implementation - just execute immediately on target core
    // For cooperative scheduler, integrate with scheduler on target core
    multicore_spinlock_acquire(&state_lock);
    core1_task = task_func;
    core1_task_arg = arg;
    multicore_spinlock_release(&state_lock);
    
    // Signal task ready
    multicore_send_message_blocking(CORE1_ID, MSG_TYPE_TASK);
    
    return true;
}

// ============================================================================
// High-Level API
// ============================================================================

void multicore_init(void) {
    // Initialize hardware and state
    multicore_rp2350B_init();
    multicore_spinlock_init(&state_lock);
    
    // Clear FIFO
    while (multicore_message_available()) {
        uint32_t dummy;
        uint8_t sender;
        multicore_try_receive_message(&dummy, &sender);
    }
    
    core1_status = 0;
    core1_task = NULL;
    core1_task_arg = NULL;
    
    POSTCONDITION(state_lock == 0);  // Lock unlocked
    POSTCONDITION(core1_status == 0);  // Core 1 not running
}

uint32_t multicore_get_current_core_id(void) {
    return multicore_rp2350B_get_core_id();
}

// Minimal "sleep" for synchronization delays
static inline void multicore_busy_wait(uint32_t cycles) {
    // Ensure termination
    PRECONDITION(cycles < 1000000);
    
    for (uint32_t i = 0; i < cycles; i++) {
        ASSERT_TERMINATION(i, cycles + 1);
        __asm__ volatile("nop");
    }
}

bool multicore_rp2350B_send_message(uint8_t target_core, uint8_t msg_type, uint32_t data) {
    PRECONDITION(target_core == CORE0_ID || target_core == CORE1_ID);
    PRECONDITION(msg_type >= MSG_TYPE_TASK && msg_type <= MSG_TYPE_RESPONSE);
    
    uint32_t message = ((uint32_t)msg_type << 24) | (data & 0x00FFFFFF);
    bool result = multicore_send_message_blocking(target_core, message);
    
    POSTCONDITION((sio_hw->fifo_st & SIO_FIFO_ST_WOF) == 0);
    return result;
}

uint8_t multicore_rp2350B_receive_message(uint32_t *data) {
    PRECONDITION_NOT_NULL(data);
    
    uint8_t sender_core;
    uint32_t message = multicore_receive_message_blocking(&sender_core);
    
    *data = message & 0x00FFFFFF;
    uint8_t msg_type = (message >> 24) & 0xFF;
    
    return msg_type;
}

void multicore_rp2350B_reset_core1(void) {
    // Not directly supported - use shutdown instead
    multicore_shutdown_core1();
}

void multicore_rp2350B_set_core1_vector_table(uint32_t offset) {
    (void)offset;  // Placeholder for future implementation
    ASSERT(false);  // Not yet implemented (Rule 6: Check function return)
}

// ============================================================================
// Platform-Specific Launch Function (Stub)
// ============================================================================

void multicore_rp2350B_launch_core1(void (*entry)(void)) {
    PRECONDITION_NOT_NULL(entry);
    (void)entry;
    
    // This is a stub - actual implementation would use PSCTRL or SDK
    // For now, just set a flag indicating core 1 should be launched
    core1_status = 2;  // Mark as running
    
    POSTCONDITION(core1_status == 2);
}

// ============================================================================
// Memory Barriers (Rule 8: Declare data volatile)
// ============================================================================

void multicore_memory_barrier(void) {
    __asm__ volatile("dmb" ::: "memory");
}

void multicore_data_sync_barrier(void) {
    __asm__ volatile("dsb" ::: "memory");
}

void multicore_instruction_sync_barrier(void) {
    __asm__ volatile("isb" ::: "memory");
}

// ============================================================================
// Shutdown
// ============================================================================

void multicore_shutdown_core1(void) {
    PRECONDITION(multicore_get_current_core_id() == CORE0_ID);
    PRECONDITION(core1_status != 0);
    
    // Send exit message to core 1
    multicore_send_message_blocking(CORE1_ID, MSG_TYPE_EXIT);
    
    // Wait briefly for acknowledgment
    multicore_busy_wait(1000);
    
    // Reset state
    multicore_spinlock_acquire(&state_lock);
    core1_status = 0;
    core1_task = NULL;
    core1_task_arg = NULL;
    multicore_spinlock_release(&state_lock);
    
    POSTCONDITION(core1_status == 0);  // Core 1 stopped
    POSTCONDITION(core1_task == NULL);  // No active task
}