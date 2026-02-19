/**
 * Multicore Scheduler API
 *
 * Extends the base scheduler with multicore support for RP2350B dual-core M33.
 * Supports task affinity, inter-core communication, and cooperative multitasking.
 *
 * P10 Compliance: This header follows NASA's Power of Ten rules for safety-critical code.
 * All functions include precondition checks and proper assertion handling.
 */

#ifndef SCHEDULER_MULTICORE_H
#define SCHEDULER_MULTICORE_H

#include <stdint.h>
#include <stdbool.h>
#include "scheduler.h"
#include "multicore.h"

// ============================================================================
+// Configuration
+// ============================================================================

#define CORE_ID_INVALID         0xFF    // Task can run on any core
#define CORE0_ID                0       // First M33 core
#define CORE1_ID                1       // Second M33 core
#define NUM_CORES               2       // RP2350B has dual cores

// ============================================================================
+// Task Management
+// ============================================================================

+/**
+ * @brief Create a task that will run on a specific core
+ *
+ * PRECONDITION: func != NULL
+ * PRECONDITION: core_id is CORE0_ID, CORE1_ID, or CORE_ID_INVALID
+ * POSTCONDITION: Returns SCHED_INVALID_TASK if no slots available, otherwise valid task_id
+ *
+ * @param core_id Target core (0, 1, or CORE_ID_INVALID for any)
+ * @param func Task function to execute
+ * @return Task ID or SCHED_INVALID_TASK
+ */
+uint8_t sched_create_on_core(uint8_t core_id, task_func_t func);

+/**
+ * @brief Get the ID of the core currently executing this code
+ *
+ * POSTCONDITION: Returns 0 or 1 (valid core ID)
+ *
+ * @return Core ID (0 or 1)
+ */
+uint8_t sched_current_core(void);

+/**
+ * @brief Check if a task is assigned to the current core
+ *
+ * PRECONDITION: task_id < SCHED_MAX_TASKS
+ *
+ * @param task_id Task ID to check
+ * @return true if task belongs to current core, false otherwise
+ */
+bool sched_task_on_current_core(uint8_t task_id);

+/**
+ * @brief Launch a task on a different core (inter-core task launching)
+ *
+ * PRECONDITION: task_id is a valid task that belongs to target_core
+ * PRECONDITION: target_core is CORE0_ID or CORE1_ID
+ * PRECONDITION: target_core != current_core
+ *
+ * Sends a message to the target core to add the task to its ready queue.
+ *
+ * @param target_core Core to launch task on
+ * @param task_id Task ID to launch
+ * @return true if message sent successfully, false otherwise
+ */
+bool sched_launch_on_core(uint8_t target_core, uint8_t task_id);

+// ============================================================================
+// Inter-Core Communication
+// ============================================================================

+/**
+ * @brief Handle inter-core messages for task management
+ *
+ * PRECONDITION: Called from within scheduler context
+ * POSTCONDITION: Messages processed and tasks queued as appropriate
+ *
+ * Checks for pending messages from other cores and handles them:
+ * - MSG_TYPE_TASK: Add task to this core's ready queue
+ * - MSG_TYPE_EXIT: Shutdown task execution
+ * - MSG_TYPE_SYNC: Synchronization signal
+ *
+ * Should be called periodically in the scheduler main loop.
+ */
+void sched_handle_multicore_message(void);

+/**
+ * @brief Broadcast a synchronization signal to all cores
+ *
+ * PRECONDITION: Called from core 0 (master)
+ *
+ * Sends sync message to core 1, requesting acknowledgment.
+ * Blocks until acknowledgment received.
+ */
+void sched_sync_cores(void);

+// ============================================================================
+// Statistics
+// ============================================================================

+/**
+ * @brief Per-core statistics structure
+ */
+typedef struct {
+    uint32_t tasks_created;      // Tasks created on this core
+    uint32_t tasks_completed;    // Tasks completed on this core
+    uint32_t context_switches;   // Scheduler loop iterations
+    uint32_t messages_sent;      // Inter-core messages sent
+    uint32_t messages_received;  // Inter-core messages received
+} sched_core_stats_t;

+/**
+ * @brief Get statistics for a specific core
+ *
+ * PRECONDITION: core_id is CORE0_ID or CORE1_ID
+ * PRECONDITION: stats != NULL
+ * POSTCONDITION: stats populated with valid data
+ *
+ * @param core_id Core to query
+ * @param stats Output buffer for statistics
+ */
+void sched_get_core_stats(uint8_t core_id, sched_core_stats_t *stats);

+/**
+ * @brief Reset statistics for a specific core
+ *
+ * PRECONDITION: core_id is CORE0_ID or CORE1_ID
+ * POSTCONDITION: Core statistics reset to zero
+ *
+ * @param core_id Core to reset
+ */
+void sched_reset_core_stats(uint8_t core_id);

+// ============================================================================
+// Safety and Compliance Functions
+// ============================================================================

+/**
+ * @brief Validate scheduler integrity for multicore operation
+ *
+ * PRECONDITION: None (safe to call anytime)
+ * POSTCONDITION: If returns true, scheduler state is consistent
+ *
+ * Checks for:
+ * - Task counts match sum of per-core counts
+ * - No tasks marked ready on wrong core
+ * - FIFO state consistent
+ * - Spinlocks not stuck
+ *
+ * @return true if scheduler state is valid, false if corrupted
+ */
+bool sched_validate_multicore_state(void);

+/**
+ * @brief Emergency stop for all cores
+ *
+ * PRECONDITION: Called from core 0 (safety master)
+ * POSTCONDITION: All schedulers stopped, all tasks terminated
+ *
+ * Immediately stops both cores' schedulers and terminates all tasks.
+ * Use only for critical failures or emergency shutdown.
+ */
+void sched_emergency_stop(void);

+// ============================================================================
+// Compatibility Wrappers
+// ============================================================================

+/**
+ * @brief Create a task on the current core (convenience wrapper)
+ *
+ * Identical to sched_create() but logs core assignment.
+ *
+ * @param func Task function
+ * @return Task ID
+ */
+uint8_t sched_create_local(task_func_t func);

+/**
+ * @brief Migrate a task to a different core
+ *
+ * PRECONDITION: task_id is valid
+ * PRECONDITION: target_core is CORE0_ID or CORE1_ID
+ * PRECONDITION: target_core != task's current core
+ *
+ * Stops the task on its current core and recreates it on target core.
+ * Task state is NOT preserved - equivalent to kill and recreate.
+ *
+ * @param task_id Task to migrate
+ * @param target_core Destination core
+ * @return New task ID on target core, or SCHED_INVALID_TASK
+ */
+uint8_t sched_migrate_task(uint8_t task_id, uint8_t target_core);

#endif // SCHEDULER_MULTICORE_H
