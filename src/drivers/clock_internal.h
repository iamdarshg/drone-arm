#ifndef DRIVERS_CLOCK_INTERNAL_H
#define DRIVERS_CLOCK_INTERNAL_H

/*
 * Internal clock state API – testable without hardware.
 * Exposed for host-side unit tests only; do not call from normal firmware.
 */

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint32_t sys_clk_hz;
    uint32_t vreg_voltage_mv;
} clock_config_t;

/* Initialise the software state record (does NOT touch hardware registers). */
void clock_state_init(uint32_t sys_hz, uint32_t vreg_mv);

/* Return true when the state record has been initialised with a valid magic. */
bool clock_is_valid(void);

/* Return a pointer to the current config, or NULL if not yet initialised. */
const clock_config_t *clock_get_config(void);

/* Overwrite the config record (does NOT touch hardware registers). */
void clock_set_config(const clock_config_t *cfg);

#endif
