#include <stdint.h>
#include <stdbool.h>

#define REG32_WRITE(addr, val) (*(volatile uint32_t *)(addr) = (val))


#define asm  __asm__

typedef struct __attribute__((aligned(16))){
    float x;
    float y;
    float z;
} Vec3;

typedef struct __attribute__((aligned(16))){
    float a;
    float b;
    float c;
    float d;
} Quaternion;
