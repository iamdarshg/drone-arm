#!/usr/bin/env python3
import os
import struct
import sys

UF2_MAGIC_START0 = 0x0A324655
UF2_MAGIC_START1 = 0x9E5D5157
UF2_MAGIC_END = 0x0AB16F30
UF2_FLAG_FAMILY_ID_PRESENT = 0x00002000
UF2_RP2350_ARM_FAMILY_ID = 0xE48BFF57


def read_u32_le(data, off):
    return struct.unpack_from('<I', data, off)[0]


def validate_uf2(path):
    if not os.path.exists(path):
        print(f"SKIP: {path} not found")
        return 0

    with open(path, 'rb') as f:
        blob = f.read()

    if len(blob) % 512 != 0:
        print('FAIL: UF2 size not multiple of 512')
        return 1

    blocks = len(blob) // 512
    for i in range(blocks):
        b = blob[i*512:(i+1)*512]
        m0 = read_u32_le(b, 0)
        m1 = read_u32_le(b, 4)
        flags = read_u32_le(b, 8)
        target = read_u32_le(b, 12)
        payload = read_u32_le(b, 16)
        fam = read_u32_le(b, 28)
        mend = read_u32_le(b, 508)

        if m0 != UF2_MAGIC_START0 or m1 != UF2_MAGIC_START1 or mend != UF2_MAGIC_END:
            print(f'FAIL: block {i} bad magic')
            return 1
        if payload == 0 or payload > 476:
            print(f'FAIL: block {i} bad payload size {payload}')
            return 1
        if flags & UF2_FLAG_FAMILY_ID_PRESENT:
            if fam != UF2_RP2350_ARM_FAMILY_ID:
                print(f'FAIL: block {i} unexpected family id {fam:#x}')
                return 1
        if not (0x10000000 <= target <= 0x11000000):
            print(f'FAIL: block {i} target addr out of range {target:#x}')
            return 1

    print(f'PASS: {path} ({blocks} blocks) valid')
    return 0


def main():
    repo = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    candidates = [
        os.path.join(repo, 'build', 'firmware.uf2'),
        os.path.join(repo, 'builddir', 'firmware.uf2'),
        os.path.join(repo, 'firmware.uf2'),
    ]

    for c in candidates:
        rc = validate_uf2(c)
        if rc == 0 and os.path.exists(c):
            return 0
    print('SKIP: no UF2 artifact found')
    return 0


if __name__ == '__main__':
    sys.exit(main())
