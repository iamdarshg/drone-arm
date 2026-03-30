/* device_autodetect.c – firmware-side BOOTSEL / USB MSC detection helpers.
 *
 * On RP2350 the ROM exposes a flag in well-known SRAM that indicates whether
 * the core was launched from the USB BOOTSEL path.  These helpers allow
 * application firmware to detect that condition and take appropriate action
 * (e.g. refuse to arm motors while in bootloader mode).
 *
 * The host-side flashing tool is tools/device_autodetect.py.
 */
#include "kernel/assert.h"
#include <stdbool.h>
#include <stdint.h>

#include "hal/platform.h"

/* The RP2350 ROM stores a boot-type value at a fixed address in SRAM after
 * the boot stage completes.  A value of 1 indicates USB MSC boot (BOOTSEL). */
#define BOOT_TYPE_ADDR   SRAM_SCRATCH_X_BASE
#define BOOT_TYPE_USB_MSC 1u

bool device_was_usb_boot(void) {
    /* Read the boot-type word written by the ROM into Scratch X SRAM. */
    uint32_t boot_type;
    boot_type = REG_RO(BOOT_TYPE_ADDR);
    return boot_type == BOOT_TYPE_USB_MSC;
}

/* Returns true when the device is running application firmware (not bootloader). */
bool device_is_app_running(void) {
    return !device_was_usb_boot();
}

