#ifndef MULTICORE_H
#define MULTICORE_H

#include <stdint.h>
#include <stdbool.h>

#include "hardware/multicore_rp2350B.h" // Platform-specific definitions

typedef void (*multicore_task_func_t)(void*);

// Function to initialize the multicore system
void multicore_init(void);

// Function to launch a task on a specific core
void multicore_launch_task(uint8_t core_id, multicore_task_func_t task_func, void* arg);

// Function to send a message to a specific core
void multicore_send_message(uint8_t core_id, uint32_t message);

// Function to receive a message from a specific core (blocking)
uint32_t multicore_receive_message_blocking(uint8_t core_id);

// Function to check if a message is available from a specific core
bool multicore_message_available(uint8_t core_id);

#endif // MULTICORE_H