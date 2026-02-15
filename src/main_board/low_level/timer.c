#include "timer.h"
#include <stdio.h>
#include "common/errors.h"

// ============================================================================
// Alarm State Management
// ============================================================================

typedef struct {
    timer_callback_t callback;
    uint32_t period_us;     // 0 = one-shot, non-zero = repeating
    uint64_t target_time;
    bool active;
} timer_alarm_t;

static timer_alarm_t alarms[TIMER_NUM_ALARMS];
static volatile uint32_t alarm_mask = 0;

// ============================================================================
// Timer Initialization
// ============================================================================

void timer_init(void) {
    // Clear all alarms
    for (uint8_t i = 0; i < TIMER_NUM_ALARMS; i++) {
        alarms[i].callback = NULL;
        alarms[i].period_us = 0;
        alarms[i].target_time = 0;
        alarms[i].active = false;
    }
    
    // Disable all interrupts first
    timer_irq_disable_all();
    
    // Clear any pending interrupts
    timer_irq_clear_all();
    
    // Set source to clk_sys for highest accuracy (if needed)
    // Default is tickgen (1MHz), clk_sys can give higher resolution
    #if TIMER_TICK_HZ > 1000000
        timer_set_source_clk_sys(true);
    #endif
    
    // Enable debug pause if desired
    timer_debug_pause(false);
    
    // Make sure timer is running
    timer_resume();
    
    alarm_mask = 0;
}

void timer_deinit(void) {
    // Disarm all alarms
    timer_disarm_all();
    
    // Disable all interrupts
    timer_irq_disable_all();
    
    // Clear all callbacks
    for (uint8_t i = 0; i < TIMER_NUM_ALARMS; i++) {
        alarms[i].callback = NULL;
        alarms[i].active = false;
    }
    
    alarm_mask = 0;
}

// ============================================================================
// Alarm Configuration
// ============================================================================

void timer_set_alarm(uint8_t alarm_num, uint32_t delay_us, timer_callback_t callback) {
    if (alarm_num >= TIMER_NUM_ALARMS) {
        log_error("Invalid alarm number", 2, "timer_set_alarm");
        return;
    }
    
    // Disable interrupt for this alarm during setup
    timer_irq_disable(alarm_num);
    
    // Clear any pending interrupt
    timer_irq_clear(alarm_num);
    
    // Calculate target time
    uint64_t now = timer_get_time();
    uint64_t target = now + delay_us;
    
    // Store alarm info
    alarms[alarm_num].callback = callback;
    alarms[alarm_num].period_us = 0;  // One-shot
    alarms[alarm_num].target_time = target;
    alarms[alarm_num].active = true;
    
    // Set alarm register (32-bit comparison, so use lower 32 bits)
    // For alarms > 32-bit range, need to check high bits in handler
    timer_alarm_set(alarm_num, (uint32_t)target);
    
    // Enable interrupt
    timer_irq_enable(alarm_num);
    
    // Arm the alarm
    timer_alarm_arm(alarm_num);
    
    alarm_mask |= (1U << alarm_num);
}

void timer_set_alarm_repeating(uint8_t alarm_num, uint32_t period_us, timer_callback_t callback) {
    if (alarm_num >= TIMER_NUM_ALARMS) {
        log_error("Invalid alarm number", 2, "timer_set_alarm_repeating");
        return;
    }
    
    // Disable interrupt for this alarm during setup
    timer_irq_disable(alarm_num);
    
    // Clear any pending interrupt
    timer_irq_clear(alarm_num);
    
    // Calculate target time
    uint64_t now = timer_get_time();
    uint64_t target = now + period_us;
    
    // Store alarm info
    alarms[alarm_num].callback = callback;
    alarms[alarm_num].period_us = period_us;
    alarms[alarm_num].target_time = target;
    alarms[alarm_num].active = true;
    
    // Set alarm register
    timer_alarm_set(alarm_num, (uint32_t)target);
    
    // Enable interrupt
    timer_irq_enable(alarm_num);
    
    // Arm the alarm
    timer_alarm_arm(alarm_num);
    
    alarm_mask |= (1U << alarm_num);
}

void timer_cancel_alarm(uint8_t alarm_num) {
    if (alarm_num >= TIMER_NUM_ALARMS) {
        return;
    }
    
    // Disarm the alarm
    timer_alarm_disarm(alarm_num);
    
    // Disable interrupt
    timer_irq_disable(alarm_num);
    
    // Clear any pending
    timer_irq_clear(alarm_num);
    
    // Mark as inactive
    alarms[alarm_num].active = false;
    alarms[alarm_num].callback = NULL;
    
    alarm_mask &= ~(1U << alarm_num);
}

uint64_t timer_get_alarm_time(uint8_t alarm_num) {
    if (alarm_num >= TIMER_NUM_ALARMS) {
        return 0;
    }
    
    return alarms[alarm_num].target_time;
}

int64_t timer_get_alarm_remaining(uint8_t alarm_num) {
    if (alarm_num >= TIMER_NUM_ALARMS || !alarms[alarm_num].active) {
        return -1;
    }
    
    uint64_t now = timer_get_time();
    int64_t remaining = (int64_t)(alarms[alarm_num].target_time - now);
    
    return remaining;
}

bool timer_alarm_fired(uint8_t alarm_num) {
    if (alarm_num >= TIMER_NUM_ALARMS) {
        return false;
    }
    
    return timer_irq_is_pending(alarm_num);
}

// ============================================================================
// Non-blocking Delay
// ============================================================================

static volatile bool delay_complete = false;
static uint8_t delay_alarm_num = 0xFF;

static void delay_callback(void) {
    delay_complete = true;
}

void timer_delay_alarm(uint32_t us) {
    // Find an unused alarm for the delay
    // Use the highest numbered alarm to avoid conflicts
    uint8_t alarm_num = TIMER_NUM_ALARMS - 1;
    
    delay_complete = false;
    delay_alarm_num = alarm_num;
    
    // Save any existing callback
    timer_callback_t saved_callback = alarms[alarm_num].callback;
    bool was_active = alarms[alarm_num].active;
    
    // Set up one-shot alarm
    timer_set_alarm(alarm_num, us, delay_callback);
    
    // Wait for completion
    while (!delay_complete) {
        __asm volatile("wfi");  // Wait for interrupt (low power)
    }
    
    // Restore previous state
    if (was_active) {
        alarms[alarm_num].callback = saved_callback;
    } else {
        timer_cancel_alarm(alarm_num);
    }
    
    delay_alarm_num = 0xFF;
}

// ============================================================================
// IRQ Handler
// ============================================================================

void timer_irq_handler(void) {
    // Get pending interrupts
    uint32_t pending = timer_irq_get_pending();
    
    // Process each pending alarm
    for (uint8_t i = 0; i < TIMER_NUM_ALARMS; i++) {
        if (pending & (1U << i)) {
            // Clear the interrupt
            timer_irq_clear(i);
            
            // Disarm the alarm (hardware auto-disarms, but be explicit)
            timer_alarm_disarm(i);
            
            // Check if this is a valid, active alarm
            if (alarms[i].active && alarms[i].callback != NULL) {
                // Call the callback
                alarms[i].callback();
                
                // If repeating, reschedule
                if (alarms[i].period_us > 0) {
                    uint64_t now = timer_get_time();
                    uint64_t target = alarms[i].target_time + alarms[i].period_us;
                    
                    // If we've fallen behind, catch up
                    while (target < now) {
                        target += alarms[i].period_us;
                    }
                    
                    alarms[i].target_time = target;
                    timer_alarm_set(i, (uint32_t)target);
                    timer_alarm_arm(i);
                } else {
                    // One-shot - mark inactive
                    alarms[i].active = false;
                    alarm_mask &= ~(1U << i);
                }
            }
        }
    }
}
