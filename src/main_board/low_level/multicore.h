#ifndef MULTICORE_H
#define MULTICORE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// Core identifiers for RP2350B
#define CORE0_ID                0
#define CORE1_ID                1
#define NUM_CORES               2

// Message types for inter-core communication
typedef enum {
    MSG_TYPE_TASK = 0x01,      // Task execution request
    MSG_TYPE_DATA = 0x02,      // Generic data transfer
    MSG_TYPE_SYNC = 0x03,      // Synchronization signal
    MSG_TYPE_EXIT = 0x04,      // Exit/shutdown signal
    MSG_TYPE_RESPONSE = 0x05   // Response to a request
} message_type_t;

// Task function signature
typedef void (*multicore_task_func_t)(void* context);

// Message structure for inter-core communication
typedef struct {
    message_type_t type;
    uint32_t data;
    uint32_t sender_core_id;
} multicore_message_t;

// Task descriptor for launching tasks on specific cores
typedef struct {
    multicore_task_func_t function;
    void* context;
    uint32_t stack_size;      // Stack size in bytes (0 for default)
    uint32_t priority;        // Relative priority (0 = normal)
} multicore_task_desc_t;

// ============================================================================
// Multicore System Initialization and Management
// ============================================================================

/**
 * @brief Initialize the multicore system
 * 
 * Must be called on core 0 before launching core 1.
 * Sets up FIFO communication channels and spinlocks.
 */
void multicore_init(void);

/**
 * @brief Launch core 1 with the specified entry point
 * 
 * Starts execution on core 1. Should be called from core 0.
 * 
 * @param entry Entry point function for core 1
 * @return true if successful, false otherwise
+ */
+bool multicore_launch_core1(void (*entry)(void));
+
+/**
+ * @brief Check if core 1 is currently running
+ * 
+ * @return true if core 1 is running, false otherwise
+ */
+bool multicore_core1_is_running(void);
+
+/**
+ * @brief Gracefully shutdown core 1
+ * 
+ * Sends exit message and waits for acknowledgment.
+ */
+void multicore_shutdown_core1(void);
+
+// ============================================================================
+// Core Identification
+// ============================================================================
+
+/**
+ * @brief Get the ID of the current core
+ * 
+ * @return 0 for core 0, 1 for core 1
+ */
+uint32_t multicore_get_current_core_id(void);
+
+/**
+ * @brief Check if running on core 0
+ * 
+ * @return true if on core 0, false otherwise
+ */
+static inline bool multicore_is_core0(void) {
+    return multicore_get_current_core_id() == CORE0_ID;
+}
+
+/**
+ * @brief Check if running on core 1
+ * 
+ * @return true if on core 1, false otherwise
+ */
+static inline bool multicore_is_core1(void) {
+    return multicore_get_current_core_id() == CORE1_ID;
+}
+
+// ============================================================================
+// Task Launching and Management
+// ============================================================================
+
+/**
+ * @brief Launch a task on a specific core
+ * 
+ * @param core_id Target core (CORE0_ID or CORE1_ID)
+ * @param task_func Task function to execute
+ * @param arg Argument passed to task function
+ * @return true if task was queued successfully, false otherwise
+ */
+bool multicore_launch_task_on_core(uint8_t core_id, multicore_task_func_t task_func, void* arg);
+
+/**
+ * @brief Launch a task on core 0
+ * 
+ * Convenience function for launching on core 0.
+ */
+static inline bool multicore_launch_task_on_core0(multicore_task_func_t task_func, void* arg) {
+    return multicore_launch_task_on_core(CORE0_ID, task_func, arg);
+}
+
+/**
+ * @brief Launch a task on core 1
+ * 
+ * Convenience function for launching on core 1.
+ */
+static inline bool multicore_launch_task_on_core1(multicore_task_func_t task_func, void* arg) {
+    return multicore_launch_task_on_core(CORE1_ID, task_func, arg);
+}
+
+// ============================================================================
+// Inter-Core Communication - Blocking APIs
+// ============================================================================
+
+/**
+ * @brief Send a 32-bit message to the other core (blocking)
+ * 
+ * Blocks until the FIFO is ready to accept the message.
+ * 
+ * @param target_core Target core ID
+ * @param message Message data (32 bits)
+ * @return true if sent successfully, false otherwise
+ */
+bool multicore_send_message_blocking(uint8_t target_core, uint32_t message);
+
+/**
+ * @brief Receive a 32-bit message from the other core (blocking)
+ * 
+ * Blocks until a message is available in the FIFO.
+ * 
+ * @param sender_core Output parameter for sender core ID
+ * @return Received message data (32 bits)
+ */
+uint32_t multicore_receive_message_blocking(uint8_t* sender_core);
+
+// ============================================================================
+// Inter-Core Communication - Non-blocking APIs
+// ============================================================================
+
+/**
+ * @brief Send a 32-bit message to the other core (non-blocking)
+ * 
+ * Returns immediately if FIFO is full.
+ * 
+ * @param target_core Target core ID
+ * @param message Message data (32 bits)
+ * @return true if sent successfully, false if FIFO full
+ */
+bool multicore_try_send_message(uint8_t target_core, uint32_t message);
+
+/**
+ * @brief Check if a message is available
+ * 
+ * @return true if message available, false otherwise
+ */
+bool multicore_message_available(void);
+
+/**
+ * @brief Receive a message if available (non-blocking)
+ * 
+ * @param message Output parameter for received message
+ * @param sender_core Output parameter for sender core ID
+ * @return true if message was received, false if none available
+ */
+bool multicore_try_receive_message(uint32_t* message, uint8_t* sender_core);
+
+// ============================================================================
+// Synchronization Primitives
+// ============================================================================
+
+// Spinlock type for simple mutual exclusion between cores
+typedef uint32_t multicore_spinlock_t;
+
+/**
+ * @brief Initialize a spinlock
+ * 
+ * @param lock Pointer to spinlock variable
+ */
+void multicore_spinlock_init(multicore_spinlock_t* lock);
+
+/**
+ * @brief Acquire a spinlock (blocking)
+ * 
+ * Spins until the lock is acquired.
+ * 
+ * @param lock Pointer to spinlock variable
+ */
+void multicore_spinlock_acquire(multicore_spinlock_t* lock);
+
+/**
+ * @brief Try to acquire a spinlock (non-blocking)
+ * 
+ * @param lock Pointer to spinlock variable
+ * @return true if lock was acquired, false otherwise
+ */
+bool multicore_spinlock_try_acquire(multicore_spinlock_t* lock);
+
+/**
+ * @brief Release a spinlock
+ * 
+ * @param lock Pointer to spinlock variable
+ */
+void multicore_spinlock_release(multicore_spinlock_t* lock);
+
+// ============================================================================
+// Memory Barriers (Platform-specific implementation)
+// ============================================================================
+
+/**
+ * @brief Memory barrier to ensure ordering between cores
+ * 
+ * Prevents reordering of memory accesses across this point.
+ */
+static inline void multicore_memory_barrier(void) {
+    __asm__ volatile("dmb" ::: "memory");
+}
+
+/**
+ * @brief Data synchronization barrier
+ * 
+ * Ensures all outstanding memory accesses are complete.
+ */
+static inline void multicore_data_sync_barrier(void) {
+    __asm__ volatile("dsb" ::: "memory");
+}
+
+/**
+ * @brief Instruction synchronization barrier
+ * 
+ * Ensures CPU pipeline is flushed.
+ */
+static inline void multicore_instruction_sync_barrier(void) {
+    __asm__ volatile("isb" ::: "memory");
+}
+
+// ============================================================================
+// Convenience Macros for Task Launching
+// ============================================================================
+
+/**
+ * @brief Launch a scheduler task on a specific core
+ * 
+ * Integrates with the scheduler system to launch tasks on specific cores.
+ * 
+ * @param core_id Target core
+ * @param task_name Name of the task function
+ * @param arg Task argument
+ * @return true if successful, false otherwise
+ */
+#define MULTICORE_LAUNCH_SCHED_TASK(core_id, task_name, arg) \
+    multicore_launch_task_on_core(core_id, (multicore_task_func_t)task_name, (void*)arg)
+
+/**
+ * @brief Launch a scheduler task on core 0
+ * 
+ * Convenience macro for launching scheduler tasks on core 0.
+ */
+#define MULTICORE_LAUNCH_SCHED_TASK_CORE0(task_name, arg) \
+    multicore_launch_task_on_core(CORE0_ID, (multicore_task_func_t)task_name, (void*)arg)
+
+/**
+ * @brief Launch a scheduler task on core 1
+ * 
+ * Convenience macro for launching scheduler tasks on core 1.
+ */
+#define MULTICORE_LAUNCH_SCHED_TASK_CORE1(task_name, arg) \
+    multicore_launch_task_on_core(CORE1_ID, (multicore_task_func_t)task_name, (void*)arg)

#endif // MULTICORE_H
