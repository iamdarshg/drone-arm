/*
 * Minimal platform definitions for RP2350
 * Based on pico-sdk but stripped down for bare-metal use
 */
#ifndef _PLATFORM_DEFS_H
#define _PLATFORM_DEFS_H

#include <stdint.h>

// Helper macros
#define _u(x) ((uint32_t)(x))
#define _i(x) ((int32_t)(x))
#define _REG_(x)

// Static assert
#ifndef static_assert
#define static_assert _Static_assert
#endif

#endif // _PLATFORM_DEFS_H
