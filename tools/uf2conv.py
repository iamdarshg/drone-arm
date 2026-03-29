#!/usr/bin/env python3
"""UF2 converter for RP2040/RP2350 ELF artifacts."""

import struct
import sys
import argparse
from pathlib import Path

# UF2 constants
UF2_MAGIC_START0 = 0x0A324655
UF2_MAGIC_START1 = 0x9E5D5157
UF2_MAGIC_END = 0x0AB16F30

# Family IDs
FAMILY_IDS = {
    "rp2040": 0xE48BFF56,
    "rp2350-arm": 0xE48BFF57,
    "rp2350-arm-s": 0xE48BFF59,
    "rp2350-riscv": 0xE48BFF5A,
    "rp2350-arm-ns": 0xE48BFF5B,
}

UF2_BLOCK_SIZE = 512
UF2_DATA_SIZE = 476
FLASH_START_ADDR = 0x10000000
FLASH_END_ADDR = 0x20000000


def read_elf_sections(elf_path):
    """Read loadable sections from ELF file."""
    with open(elf_path, "rb") as f:
        data = f.read()

    # Check ELF magic
    if data[:4] != b"\x7fELF":
        raise ValueError("Not a valid ELF file")

    # Check 32-bit ARM
    if data[4] != 1:  # 32-bit
        raise ValueError("Only 32-bit ELF supported")
    if data[5] != 1:  # Little endian
        raise ValueError("Only little-endian ELF supported")

    # Parse ELF header
    e_entry = struct.unpack("<I", data[0x18:0x1C])[0]
    e_phoff = struct.unpack("<I", data[0x1C:0x20])[0]
    e_phentsize = struct.unpack("<H", data[0x2A:0x2C])[0]
    e_phnum = struct.unpack("<H", data[0x2C:0x2E])[0]

    sections = []

    # Parse program headers
    for i in range(e_phnum):
        offset = e_phoff + i * e_phentsize
        p_type = struct.unpack("<I", data[offset : offset + 4])[0]
        p_offset = struct.unpack("<I", data[offset + 4 : offset + 8])[0]
        p_vaddr = struct.unpack("<I", data[offset + 8 : offset + 12])[0]
        p_paddr = struct.unpack("<I", data[offset + 12 : offset + 16])[0]
        p_filesz = struct.unpack("<I", data[offset + 16 : offset + 20])[0]
        p_memsz = struct.unpack("<I", data[offset + 20 : offset + 24])[0]

        # PT_LOAD = 1, loadable segment
        if p_type == 1 and p_filesz > 0:
            # Prefer program virtual address (p_vaddr) for placement; fallback to p_paddr
            load_addr = p_vaddr if p_vaddr != 0 else p_paddr
            # Only handle flash addresses (exclude RAM/other regions)
            if FLASH_START_ADDR <= load_addr < FLASH_END_ADDR:
                section_data = data[p_offset : p_offset + p_filesz]
                sections.append({"addr": load_addr, "data": section_data, "size": p_filesz})

    return sections, e_entry


def create_uf2_block(data, target_addr, block_no, num_blocks, family_id, payload_size=None):
    """Create a single UF2 block.

    `data` should be exactly UF2_DATA_SIZE bytes (padded if necessary).
    `payload_size` is the number of meaningful bytes in `data` (<= UF2_DATA_SIZE).
    """
    block = bytearray(UF2_BLOCK_SIZE)

    # Header (32 bytes)
    struct.pack_into("<I", block, 0, UF2_MAGIC_START0)
    struct.pack_into("<I", block, 4, UF2_MAGIC_START1)
    # Flags: set family ID present if a non-zero family_id is given
    flags = 0x00002000 if family_id else 0
    struct.pack_into("<I", block, 8, flags)
    struct.pack_into("<I", block, 12, target_addr)
    struct.pack_into("<I", block, 16, payload_size if payload_size is not None else len(data))
    struct.pack_into("<I", block, 20, block_no)
    struct.pack_into("<I", block, 24, num_blocks)
    struct.pack_into("<I", block, 28, family_id if family_id else 0)

    # Data (up to 476 bytes) - `data` is already padded to UF2_DATA_SIZE
    block[32 : 32 + len(data)] = data

    # Magic end (4 bytes)
    struct.pack_into("<I", block, 508, UF2_MAGIC_END)

    return block


def elf_to_uf2(elf_path, uf2_path, family_id):
    """Convert ELF to UF2 format."""
    sections, entry = read_elf_sections(elf_path)

    if not sections:
        raise ValueError("No loadable sections found in ELF file")

    # Create UF2 blocks per-section to avoid creating huge images for sparse addresses.
    chunks = []
    for section in sections:
        addr = section["addr"]
        data = section["data"]
        for off in range(0, len(data), UF2_DATA_SIZE):
            piece = data[off : off + UF2_DATA_SIZE]
            payload_size = len(piece)
            if payload_size < UF2_DATA_SIZE:
                piece = piece + bytes(UF2_DATA_SIZE - payload_size)
            chunks.append({"addr": addr + off, "data": piece, "size": payload_size})

    # Sort chunks by target address and create UF2 blocks
    chunks.sort(key=lambda c: c["addr"])  # ascending addresses
    blocks = []
    num_blocks = len(chunks)

    for i, c in enumerate(chunks):
        block = create_uf2_block(c["data"], c["addr"], i, num_blocks, family_id, payload_size=c["size"])
        blocks.append(block)

    # Write UF2 file
    with open(uf2_path, "wb") as f:
        for block in blocks:
            f.write(block)

    return len(blocks)


def main():
    parser = argparse.ArgumentParser(description="Convert ELF to UF2 format")
    parser.add_argument("input", help="Input ELF file")
    parser.add_argument("-o", "--output", help="Output UF2 file")
    parser.add_argument(
        "-f",
        "--family",
        default="rp2350-arm",
        choices=list(FAMILY_IDS.keys()),
        help="Target family (default: rp2350-arm)",
    )

    args = parser.parse_args()

    elf_path = Path(args.input)
    if not elf_path.exists():
        print(f"Error: File not found: {elf_path}")
        sys.exit(1)

    if args.output:
        uf2_path = Path(args.output)
    else:
        uf2_path = elf_path.with_suffix(".uf2")

    family_id = FAMILY_IDS[args.family]

    try:
        num_blocks = elf_to_uf2(elf_path, uf2_path, family_id)
        print(f"Converted {elf_path} -> {uf2_path}")
        print(f"  Family: {args.family} (0x{family_id:08x})")
        print(f"  Blocks: {num_blocks}")
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
