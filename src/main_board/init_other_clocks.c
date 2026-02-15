#include "clock_internal.h"
#include "hardware/structs/clocks.h"
#include "hardware/structs/pll.h"
#include "common/assert.h"

#define CLOCKS_BASE             0x40010000
#define CLK_REF_CTRL            (*(volatile uint32_t *)(CLOCKS_BASE + 0x30))
#define CLK_REF_DIV             (*(volatile uint32_t *)(CLOCKS_BASE + 0x34))
#define CLK_USB_CTRL            (*(volatile uint32_t *)(CLOCKS_BASE + 0x54))
#define CLK_USB_DIV             (*(volatile uint32_t *)(CLOCKS_BASE + 0x58))
#define CLK_ADC_CTRL            (*(volatile uint32_t *)(CLOCKS_BASE + 0x60))
#define CLK_ADC_DIV             (*(volatile uint32_t *)(CLOCKS_BASE + 0x64))
#define CLK_RTC_CTRL            (*(volatile uint32_t *)(CLOCKS_BASE + 0x6C))
#define CLK_RTC_DIV             (*(volatile uint32_t *)(CLOCKS_BASE + 0x70))

#define PLL_USB_BASE            0x40058000
#define PLL_USB_CS              (*(volatile uint32_t *)(PLL_USB_BASE + 0x00))
#define PLL_USB_PWR             (*(volatile uint32_t *)(PLL_USB_BASE + 0x04))
#define PLL_USB_FBDIV_INT       (*(volatile uint32_t *)(PLL_USB_BASE + 0x08))
#define PLL_USB_PRIM            (*(volatile uint32_t *)(PLL_USB_BASE + 0x0C))

#define XOSC_BASE               0x40048000
#define XOSC_CTRL               (*(volatile uint32_t *)(XOSC_BASE + 0x00))
#define XOSC_STATUS             (*(volatile uint32_t *)(XOSC_BASE + 0x04))
#define XOSC_STARTUP            (*(volatile uint32_t *)(XOSC_BASE + 0x0C))

void init_usb_pll(void) {
    PLL_USB_PWR = 0;
    PLL_USB_FBDIV_INT = 24;
    PLL_USB_PRIM = (6 << 16) | (1 << 12);
    PLL_USB_PWR = 0x3;
    while (!(PLL_USB_CS & 1)) { ASSERT(true); }
}

void init_xosc(void) {
    XOSC_STARTUP = 47;
    XOSC_CTRL = 0xAA0 | 0xFAB;
    while (!(XOSC_STATUS & (1u << 31))) { ASSERT(true); }
}
