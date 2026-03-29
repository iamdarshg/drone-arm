#ifndef DRIVERS_CLOCK_H
#define DRIVERS_CLOCK_H

#include <stdint.h>

typedef enum {
    CLOCK_ID_REF = 0,
    CLOCK_ID_SYS = 1,
    CLOCK_ID_PERI = 2,
} clock_id_t;

void clock_init(void);
uint32_t clock_get_hz(clock_id_t id);

#endif
