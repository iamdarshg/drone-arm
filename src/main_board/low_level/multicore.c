#include "multicore.h"
#include "multicore_rp2350B.h" // Platform-specific includes

// Function to initialize the second core
void multicore_init_core1(void (*entry)(void)) {
    multicore_rp2350B_init_core1(entry);
}

// Function to launch a task on core1
void multicore_launch_task_on_core1(void (*task)(void)) {
    multicore_rp2350B_launch_task_on_core1(task);
}

// Function to get the current core ID
uint32_t multicore_get_current_core_id(void) {
    return multicore_rp2350B_get_current_core_id();
}

// You can add more generic multicore functions here as needed,
// delegating to the platform-specific implementation.