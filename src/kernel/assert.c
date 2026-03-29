#include "assert.h"

void assertion_failed(const char *file, int line, const char *cond, const char *msg) {
    (void)file; (void)line; (void)cond; (void)msg;
    for (;;) {
        __asm__ volatile("bkpt #0");
    }
}
