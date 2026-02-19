#include <stdint.h>
#include <stdbool.h>

#define REG32_WRITE(addr, val) (*(volatile uint32_t *)(addr) = (val))
typedef struct{
    uint64_t x;
    uint64_t y;
    uint64_t z;
} Vec3;

typedef struct{
    uint64_t d;
    uint64_t c;
    uint64_t b;
    uint64_t a;
} Quarternion;
