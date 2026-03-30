/* gpio_stub.c – no-op GPIO stubs for host-side unit tests.
 *
 * Provides stand-in definitions for gpio_set_function, gpio_set_dir, and
 * gpio_put so that modules under test can be linked without real hardware.
 */
#include <stdint.h>
#include <stdbool.h>

void gpio_set_function(uint32_t pin, uint32_t fn) { (void)pin; (void)fn; }
void gpio_set_dir(uint32_t pin, bool out)         { (void)pin; (void)out; }
void gpio_put(uint32_t pin, bool val)             { (void)pin; (void)val; }
