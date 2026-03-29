#include "board.h"
#include "kernel/scheduler.h"

int main(void) {
    board_init();
    scheduler_init();
    for (;;) {
        scheduler_run_once(0u);
        scheduler_run_once(1u);
    }
}
