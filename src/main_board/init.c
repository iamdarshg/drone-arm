#include <stdint.h>
#include <stdbool.h>

#include "low_level/clocks.h"
#include "low_level/adc.h"
#include "low_level/pwm.h"
#include "low_level/spi.h"
#include "low_level/i2c.h"
#include "low_level/timer.h"
#include "low_level/dma.h"
#include "low_level/mpu.h"
#include "low_level/sys.h"
#include "low_level/gpio.h"
#include "scheduler.h"
#include "sensor_fusion/lsm6.h"
#include "sensor_fusion/ICP-42670.h"



void loop_task(uint8_t task_id) {
    (void)task_id;
    gpio_toggle(25);
    sched_sleep_ms(100);
}

void main_board_init(void) {
    init_nvic();
    init_glitch_detector();
    init_watchdog();
    init_clocks();
    init_scheduler();
    init_adc();
    init_pwm();
    init_spi();
    init_timer();
    init_i2c();
    init_mpu();
    init_dma();
    ICP_init();
    LSM6_init(0);
    sched_init();
    sched_create(loop_task);
}

void main_board_loop(void) {
    sched_run();
}
