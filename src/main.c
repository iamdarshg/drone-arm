#include "board.h"
#include "kernel/scheduler.h"

/*
 * Core1 entry point – runs the scheduler's Core1 work-loop.
 * Declared extern in board.c and defined here so that the application
 * layer (main.c) owns the per-core policy.
 */
void core1_main(void) {
    for (;;) {
        scheduler_run_once(1u);
    }
}

int main(void) {
    board_init();
    scheduler_init();
    for (;;) {
        scheduler_run_once(0u);
    }
}
