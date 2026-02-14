#include <stdint.h>
#include <stdbool.h>

typedef void (*timer_callback_t)(void);

typedef struct {
    uint8_t alarm_num;
    uint64_t delay_us;
    bool repeating;
    timer_callback_t callback;
} timer_config_t;

void timer_init(void);
void timer_set_alarm(uint8_t alarm_num, uint64_t delay_us, timer_callback_t callback);
void timer_cancel_alarm(uint8_t alarm_num);
uint64_t timer_get_us(void);
void timer_delay_us(uint64_t us);
void timer_sleep_ms(uint32_t ms);
