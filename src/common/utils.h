#include <stdint.h>
#include <stdbool.h>

#define REG32_WRITE(addr, val) (*(volatile uint32_t *)(addr) = (val))