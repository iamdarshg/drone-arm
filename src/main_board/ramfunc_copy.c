/* Copy .ramfunc section from flash (LMA) to RAM (VMA) early in startup */
#include <string.h>
#include "pico/runtime_init.h"

extern uint8_t __sramfunc_load[];
extern uint8_t __sramfunc_start[];
extern uint8_t __sramfunc_end[];

static void copy_ramfuncs(void) {
    size_t sz = (size_t)(&__sramfunc_end - &__sramfunc_start);
    if (sz) {
        memcpy(__sramfunc_start, __sramfunc_load, sz);
    }
}

PICO_RUNTIME_INIT_FUNC_RUNTIME(copy_ramfuncs, PICO_RUNTIME_INIT_BOOTROM_RESET);
