#ifndef UF2_H
#define UF2_H

#include <stdint.h>
#include <stdbool.h>

#define UF2_MAGIC_START0 0x0A324655u
#define UF2_MAGIC_START1 0x9E5D5157u
#define UF2_MAGIC_END    0x0AB16F30u
#define UF2_FLAG_FAMILY_ID_PRESENT 0x00002000u
#define UF2_RP2350_ARM_FAMILY_ID 0xE48BFF57u

typedef struct {
    uint32_t magic_start0;
    uint32_t magic_start1;
    uint32_t flags;
    uint32_t target_addr;
    uint32_t payload_size;
    uint32_t block_no;
    uint32_t num_blocks;
    uint32_t file_size_or_family_id;
    uint8_t  data[476];
    uint32_t magic_end;
} uf2_block_t;

bool uf2_block_is_valid(const uf2_block_t *blk);
bool uf2_block_has_family_id(const uf2_block_t *blk, uint32_t family_id);

#endif
