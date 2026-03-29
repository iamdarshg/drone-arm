#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "../src/hal/uf2.h"

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST_ASSERT(expr) do { \
    if (!(expr)) { \
        printf("FAIL: %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        tests_failed++; \
    } else { \
        tests_passed++; \
    } \
} while (0)

static uf2_block_t make_valid_block(void) {
    uf2_block_t b;
    memset(&b, 0, sizeof(b));
    b.magic_start0 = UF2_MAGIC_START0;
    b.magic_start1 = UF2_MAGIC_START1;
    b.magic_end = UF2_MAGIC_END;
    b.payload_size = 256;
    b.flags = UF2_FLAG_FAMILY_ID_PRESENT;
    b.file_size_or_family_id = UF2_RP2350_ARM_FAMILY_ID;
    b.target_addr = 0x10000000u;
    b.block_no = 0;
    b.num_blocks = 1;
    return b;
}

int main(void) {
    uf2_block_t b = make_valid_block();

    TEST_ASSERT(sizeof(uf2_block_t) == 512);
    TEST_ASSERT(uf2_block_is_valid(&b));
    TEST_ASSERT(uf2_block_has_family_id(&b, UF2_RP2350_ARM_FAMILY_ID));

    b.magic_start0 = 0;
    TEST_ASSERT(!uf2_block_is_valid(&b));

    b = make_valid_block();
    b.magic_end = 0;
    TEST_ASSERT(!uf2_block_is_valid(&b));

    b = make_valid_block();
    b.payload_size = 477;
    TEST_ASSERT(!uf2_block_is_valid(&b));

    b = make_valid_block();
    b.flags = 0;
    TEST_ASSERT(!uf2_block_has_family_id(&b, UF2_RP2350_ARM_FAMILY_ID));

    printf("UF2 tests: %d passed, %d failed\n", tests_passed, tests_failed);
    return tests_failed ? 1 : 0;
}
