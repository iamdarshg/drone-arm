#include "uf2.h"

bool uf2_block_is_valid(const uf2_block_t *blk) {
    if (!blk) return false;
    if (blk->magic_start0 != UF2_MAGIC_START0) return false;
    if (blk->magic_start1 != UF2_MAGIC_START1) return false;
    if (blk->magic_end != UF2_MAGIC_END) return false;
    if (blk->payload_size == 0 || blk->payload_size > 476u) return false;
    return true;
}

bool uf2_block_has_family_id(const uf2_block_t *blk, uint32_t family_id) {
    if (!uf2_block_is_valid(blk)) return false;
    if ((blk->flags & UF2_FLAG_FAMILY_ID_PRESENT) == 0u) return false;
    return blk->file_size_or_family_id == family_id;
}
