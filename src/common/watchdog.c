#include "watchdog.h"

void watchdog_init(watchdog_t *wd, uint32_t timeout_ms) {
    wd->timeout_ms = timeout_ms;
    wd->expired = false;
}
void watchdog_feed(watchdog_t *wd) { wd->expired = false; }
bool watchdog_check(watchdog_t *wd) { return wd->expired; }
void watchdog_enable(watchdog_t *wd) { (void)wd; }
void watchdog_disable(watchdog_t *wd) { (void)wd; }
