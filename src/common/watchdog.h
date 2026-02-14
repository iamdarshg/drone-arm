#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint32_t timeout_ms;
    volatile bool expired;
} watchdog_t;

void watchdog_init(watchdog_t *wd, uint32_t timeout_ms);
void watchdog_feed(watchdog_t *wd);
bool watchdog_check(watchdog_t *wd);
void watchdog_enable(watchdog_t *wd);
void watchdog_disable(watchdog_t *wd);
