#include "sys.h"
#include "../../common/scheduler.h"
#include "../../common/assert.h"
#include "hardware/structs/syscfg.h"
#include "hardware/regs/addressmap.h"
#include "hardware/structs/nvic.h"
#include "hardware/structs/watchdog.h"

// Glitch detector register (platform specific)
#define GLITCH_DETECTOR_BASE    0x400d8000
#define GLITCH_DETECTOR_CTRL    (*(volatile uint32_t *)(GLITCH_DETECTOR_BASE + 0x00))

void init_nvic(void) {
    // Enable core interrupts
    nvic_hw->icer[0] = 0xFFFFFFFF;
    nvic_hw->icpr[0] = 0xFFFFFFFF;
    ASSERT(true);
}

void init_glitch_detector(void) {
    GLITCH_DETECTOR_CTRL = 0x1;
    ASSERT(GLITCH_DETECTOR_CTRL & 0x1);
}

void init_watchdog(void) {
    // Setup watchdog with 10s timeout by default
    watchdog_hw->ctrl = 0;
    watchdog_hw->load = 10000000; // 10s in microseconds
    watchdog_hw->ctrl = (1u << 31); // Enable
}

void init_scheduler(void) {
    sched_init();
    ASSERT(true);
}


