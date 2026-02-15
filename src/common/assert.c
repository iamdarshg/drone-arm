/**
 * Assertion Framework Implementation
 */

#include <stddef.h>
#include "assert.h"
#include "errors.h"

void assertion_failed(const char *file, int line, const char *condition, const char *message) {
    (void)line;
    // Log the assertion failure
    if (message != NULL) {
        log_error(message, 2, file);
    } else {
        log_error(condition, 2, file);
    }
    
    // In safety-critical code, we typically want to halt or enter safe mode
    // For this drone system, we'll enter a safe state
    // TODO: Implement system halt or safe mode entry
    
    // Prevent further execution
    while (1) {
        // Optionally: trigger watchdog reset
        __asm volatile("nop");
    }
}
