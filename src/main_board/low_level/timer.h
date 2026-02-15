#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware_defs/timer_config.h"
#include "pico-sdk/src/rp2350/hardware_structs/include/hardware/structs/timer.h"
#include "pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/timer.h"
#include "pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/addressmap.h"

// ============================================================================
// Timer Hardware Selection
// ============================================================================

#if USE_TIMER0
    #define timer_hw    timer0_hw
    #define TIMER_IRQ   0   // TIMER_IRQ_0
#else
    #define timer_hw    timer1_hw
    #define TIMER_IRQ   1   // TIMER_IRQ_1
#endif

// ============================================================================
// Timer Register Bit Definitions
// ============================================================================

// Alarm bits
#define TIMER_ALARM0_BIT    (1U << 0)
#define TIMER_ALARM0_ARMED  (1U << 0)
#define TIMER_ALARM1_BIT    (1U << 1)
#define TIMER_ALARM1_ARMED  (1U << 1)
#define TIMER_ALARM2_BIT    (1U << 2)
#define TIMER_ALARM2_ARMED  (1U << 2)
#define TIMER_ALARM3_BIT    (1U << 3)
#define TIMER_ALARM3_ARMED  (1U << 3)

// Pause bits
#define TIMER_PAUSE_BIT     (1U << 0)
#define TIMER_DBG0_BIT      (1U << 1)
#define TIMER_DBG1_BIT      (1U << 2)

// Source bit
#define TIMER_SOURCE_BIT    (1U << 0)   // 0 = tickgen, 1 = clk_sys

// ============================================================================
// Callback Type Definition
// ============================================================================

typedef void (*timer_callback_t)(void);

// ============================================================================
// Inline Functions - Time Reading (Critical for performance)
// ============================================================================

static inline uint32_t timer_get_low(void) {
    return timer_hw->timelr;
}

static inline uint32_t timer_get_high(void) {
    return timer_hw->timehr;
}

static inline uint64_t timer_get_time(void) {
    // Must read low before high to avoid overflow race
    uint32_t lo = timer_hw->timelr;
    uint32_t hi = timer_hw->timehr;
    return ((uint64_t)hi << 32) | lo;
}

static inline uint32_t timer_get_low_raw(void) {
    return timer_hw->timerawl;
}

static inline uint32_t timer_get_high_raw(void) {
    return timer_hw->timerawh;
}

static inline uint64_t timer_get_time_raw(void) {
    // Raw read - no side effects, but may be inconsistent
    uint32_t lo = timer_hw->timerawl;
    uint32_t hi = timer_hw->timerawh;
    return ((uint64_t)hi << 32) | lo;
}

// ============================================================================
// Inline Functions - Time Setting
// ============================================================================

static inline void timer_set_time(uint64_t time) {
    // Must write low before high
    timer_hw->timelw = (uint32_t)time;
    timer_hw->timehw = (uint32_t)(time >> 32);
}

// ============================================================================
// Inline Functions - Alarm Control
// ============================================================================

static inline void timer_alarm_set(uint8_t alarm_num, uint32_t time) {
    timer_hw->alarm[alarm_num] = time;
}

static inline uint32_t timer_alarm_get(uint8_t alarm_num) {
    return timer_hw->alarm[alarm_num];
}

static inline void timer_alarm_arm(uint8_t alarm_num) {
    timer_hw->armed |= (1U << alarm_num);
}

static inline void timer_alarm_disarm(uint8_t alarm_num) {
    timer_hw->armed &= ~(1U << alarm_num);
}

static inline bool timer_alarm_is_armed(uint8_t alarm_num) {
    return (timer_hw->armed & (1U << alarm_num)) != 0;
}

static inline uint32_t timer_get_armed(void) {
    return timer_hw->armed;
}

// Convenience: arm all configured alarms
static inline void timer_arm_all(void) {
    uint32_t mask = (1U << TIMER_NUM_ALARMS) - 1;
    timer_hw->armed = mask;
}

// Convenience: disarm all alarms
static inline void timer_disarm_all(void) {
    timer_hw->armed = 0;
}

// ============================================================================
// Inline Functions - Interrupt Control
// ============================================================================

static inline void timer_irq_enable(uint8_t alarm_num) {
    timer_hw->inte |= (1U << alarm_num);
}

static inline void timer_irq_disable(uint8_t alarm_num) {
    timer_hw->inte &= ~(1U << alarm_num);
}

static inline void timer_irq_enable_all(void) {
    uint32_t mask = (1U << TIMER_NUM_ALARMS) - 1;
    timer_hw->inte = mask;
}

static inline void timer_irq_disable_all(void) {
    timer_hw->inte = 0;
}

static inline bool timer_irq_is_pending(uint8_t alarm_num) {
    return (timer_hw->ints & (1U << alarm_num)) != 0;
}

static inline uint32_t timer_irq_get_pending(void) {
    return timer_hw->ints;
}

static inline void timer_irq_clear(uint8_t alarm_num) {
    timer_hw->intr = (1U << alarm_num);
}

static inline void timer_irq_clear_all(void) {
    timer_hw->intr = 0x0F;  // Clear all 4 alarms
}

static inline void timer_irq_force(uint8_t alarm_num) {
    timer_hw->intf = (1U << alarm_num);
}

// ============================================================================
// Inline Functions - Timer Control
// ============================================================================

static inline void timer_pause(void) {
    timer_hw->pause |= TIMER_PAUSE_BIT;
}

static inline void timer_resume(void) {
    timer_hw->pause &= ~TIMER_PAUSE_BIT;
}

static inline bool timer_is_paused(void) {
    return (timer_hw->pause & TIMER_PAUSE_BIT) != 0;
}

static inline void timer_set_source_clk_sys(bool use_clk_sys) {
    if (use_clk_sys) {
        timer_hw->source |= TIMER_SOURCE_BIT;
    } else {
        timer_hw->source &= ~TIMER_SOURCE_BIT;
    }
}

static inline void timer_debug_pause(bool pause_on_debug) {
    if (pause_on_debug) {
        timer_hw->dbgpause |= (TIMER_DBG0_BIT | TIMER_DBG1_BIT);
    } else {
        timer_hw->dbgpause &= ~(TIMER_DBG0_BIT | TIMER_DBG1_BIT);
    }
}

// ============================================================================
// Inline Functions - Delay (Busy-wait)
// ============================================================================

static inline void timer_delay_us(uint32_t us) {
    uint64_t target = timer_get_time() + us;
    while (timer_get_time() < target) {
        // Busy wait - tight loop for accuracy
        __asm volatile("nop");
    }
}

static inline void timer_delay_ms(uint32_t ms) {
    timer_delay_us(ms * 1000);
}

// Spin until alarm fires (one-shot wait)
static inline void timer_wait_for_alarm(uint8_t alarm_num) {
    while (!timer_irq_is_pending(alarm_num)) {
        __asm volatile("nop");
    }
}

// ============================================================================
// Function Prototypes
// ============================================================================

/** @brief Standardized timer initialization wrapper. */
void init_timer(void);

void timer_init(void);
void timer_deinit(void);

void timer_set_alarm(uint8_t alarm_num, uint32_t delay_us, timer_callback_t callback);
void timer_set_alarm_repeating(uint8_t alarm_num, uint32_t period_us, timer_callback_t callback);
void timer_cancel_alarm(uint8_t alarm_num);
uint64_t timer_get_alarm_time(uint8_t alarm_num);

// Non-blocking delay using alarm
void timer_delay_alarm(uint32_t us);

// Get remaining time until alarm fires
int64_t timer_get_alarm_remaining(uint8_t alarm_num);

// Check if alarm has fired
bool timer_alarm_fired(uint8_t alarm_num);

// Main IRQ handler (call from your vector table)
void timer_irq_handler(void);

#endif // TIMER_H
