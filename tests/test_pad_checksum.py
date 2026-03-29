#!/usr/bin/env python3
import os
import re
import subprocess
import tempfile


def parse_words(asm_path):
    words = []
    with open(asm_path, 'r', encoding='utf-8') as f:
        for line in f:
            m = re.search(r'\.word\s+0x([0-9a-fA-F]{8})', line)
            if m:
                words.append(int(m.group(1), 16))
    return words


def test_pad_checksum_crc32_reference():
    repo = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    tool = os.path.join(repo, 'tools', 'pad_checksum')
    with tempfile.TemporaryDirectory() as td:
        inp = os.path.join(td, 'boot2.bin')
        out = os.path.join(td, 'boot2_padded.S')
        with open(inp, 'wb') as f:
            f.write(bytes([1, 2, 3, 4]))
        subprocess.run(['python3', tool, inp, out], check=True)
        words = parse_words(out)
        assert len(words) == 64
        assert words[0] == 0x04030201
        for w in words[1:63]:
            assert w == 0xFFFFFFFF
        assert words[63] == 0xCF7DD7F3


if __name__ == '__main__':
    test_pad_checksum_crc32_reference()
    print('PASS: test_pad_checksum_crc32_reference')
