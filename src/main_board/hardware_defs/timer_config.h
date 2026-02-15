#ifndef TIMER_CONFIG_H
#define TIMER_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

// Timer Selection
// TIMER0 is recommended for better accuracy (1MHz tick rate)
// TIMER1 is also available as backup
#define USE_TIMER0              1       // Set to 1 for TIMER0, 0 for TIMER1

// Number of alarms to enable (2 or 3, max 4 per timer)
#define TIMER_NUM_ALARMS        3

// Alarm priorities (lower number = higher priority)
#define TIMER_ALARM0_PRIORITY   0
#define TIMER_ALARM1_PRIORITY   1
#define TIMER_ALARM2_PRIORITY   2
#define TIMER_ALARM3_PRIORITY   3

// Timer tick frequency (1MHz = 1us resolution)
#define TIMER_TICK_HZ           1000000
#define TIMER_US_PER_TICK       1

// Maximum timer value (64-bit, wraps after ~584,000 years at 1MHz)
#define TIMER_MAX_US            0xFFFFFFFFFFFFFFFFULL

// Inline delay loop optimization
// Number of NOPs per microsecond (depends on CPU clock)
// At 150MHz: ~150 cycles per us, assuming 1 cycle per NOP
#define TIMER_NOP_CYCLES        1
#define TIMER_DELAY_LOOP_US     (150 / TIMER_NOP_CYCLES)

// Alarm configuration structure (compile-time defaults)
typedef struct {
    uint8_t alarm_num;
    uint32_t delay_us;
    bool repeating;
} timer_alarm_default_t;

// Pre-configured alarms (modify as needed)
#define TIMER_ALARM0_DEFAULT_US     1000    // 1ms
#define TIMER_ALARM1_DEFAULT_US     10000   // 10ms
#define TIMER_ALARM2_DEFAULT_US     100000  // 100ms

// Interrupt enable mask
#define TIMER_INTE_ALARM0       (1U << 0)
#define TIMER_INTE_ALARM1       (1U << 1)
#define TIMER_INTE_ALARM2       (1U << 2)
#define TIMER_INTE_ALARM3       (1U << 3)
#define TIMER_INTE_ALL          (TIMER_INTE_ALARM0 | TIMER_INTE_ALARM1 | TIMER_INTE_ALARM2 | TIMER_INTE_ALARM3)

// Armed register bits
#define TIMER_ARMED_ALARM0      (1U << 0)
#define TIMER_ARMED_ALARM1      (1U << 1)
#define TIMER_ARMED_ALARM2      (1U << 2)
#define TIMER_ARMED_ALARM3      (1U << 3)

#endif // TIMER_CONFIG_H
