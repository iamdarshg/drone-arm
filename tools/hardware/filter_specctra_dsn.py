#!/usr/bin/env python3
"""Filter Specctra DSN networks without changing placement or copper geometry."""
from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

NET_HEAD = re.compile(r'^\(net\s+("(?:[^"\\]|\\.)*"|\S+)')
PINS = re.compile(r'\(pins\s+(.+?)\)', re.S)
REF = re.compile(r'^([A-Za-z]+)(\d+)')


def matching_close(text: str, start: int) -> int:
    depth = 0
    quoted = False
    escaped = False
    for index in range(start, len(text)):
        char = text[index]
        if quoted:
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == '"':
                quoted = False
            continue
        if char == '"':
            quoted = True
        elif char == '(':
            depth += 1
        elif char == ')':
            depth -= 1
            if depth == 0:
                return index
    raise ValueError("unbalanced DSN expression")


def split_children(text: str, start: int, end: int) -> list[tuple[int, int]]:
    children: list[tuple[int, int]] = []
    index = start
    while index < end:
        while index < end and text[index].isspace():
            index += 1
        if index >= end:
            break
        if text[index] != '(':
            index += 1
            continue
        close = matching_close(text, index)
        if close >= end:
            raise ValueError("child extends outside parent")
        children.append((index, close + 1))
        index = close + 1
    return children


def unquote(token: str) -> str:
    token = token.strip()
    if token.startswith('"') and token.endswith('"'):
        return bytes(token[1:-1], "utf-8").decode("unicode_escape")
    return token


def pin_references(block: str) -> list[str]:
    match = PINS.search(block)
    if not match:
        return []
    return [token.split('-', 1)[0] for token in match.group(1).split()]


def numeric_ref(reference: str) -> int | None:
    match = REF.match(reference)
    return int(match.group(2)) if match else None


def is_motor_ref(reference: str) -> bool:
    number = numeric_ref(reference)
    return number is not None and 1100 <= number < 1700


def is_plane_net(name: str) -> bool:
    upper = name.upper()
    if upper in {"/DGND", "/GND", "/3V3", "/5V", "/VBAT", "/VBUS"}:
        return True
    return bool(
        re.search(
            r'(?:^|/)(?:M[1-6]_)?(?:BAT[PN]|PHASE_[ABC]|3V3[AI]?|5V[AI]?|DGND)$',
            upper,
        )
    )


def choose_net(mode: str, name: str, refs: list[str], exact_names: set[str]) -> bool:
    if mode == "names":
        return name in exact_names
    if mode == "shared-esc":
        if is_plane_net(name):
            return False
        shared = [reference for reference in refs if not is_motor_ref(reference)]
        return len(refs) >= 2 and bool(shared)
    raise ValueError(mode)


def filter_dsn(source: Path, output: Path, mode: str, exact_names: set[str]) -> dict:
    text = source.read_text(encoding="utf-8")
    network_start = text.index('(network')
    network_end = matching_close(text, network_start)
    header_end = text.index('\n', network_start) + 1

    kept_blocks: list[str] = []
    kept_nets: list[str] = []
    dropped_nets: list[str] = []
    for start, end in split_children(text, header_end, network_end):
        block = text[start:end]
        match = NET_HEAD.match(block.lstrip())
        if not match:
            kept_blocks.append(block)
            continue
        name = unquote(match.group(1))
        refs = pin_references(block)
        if choose_net(mode, name, refs, exact_names):
            kept_blocks.append(block)
            kept_nets.append(name)
        else:
            dropped_nets.append(name)

    replacement = text[network_start:header_end]
    replacement += ''.join('\n    ' + block.lstrip() for block in kept_blocks)
    replacement += '\n  )'
    output.write_text(
        text[:network_start] + replacement + text[network_end + 1 :],
        encoding="utf-8",
    )
    return {
        "source": str(source),
        "output": str(output),
        "mode": mode,
        "kept_net_count": len(kept_nets),
        "dropped_net_count": len(dropped_nets),
        "kept_nets": kept_nets,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--mode', choices=('names', 'shared-esc'), required=True)
    parser.add_argument('--name', action='append', default=[])
    parser.add_argument('--names-file', type=Path)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()

    names = set(args.name)
    if args.names_file:
        for line in args.names_file.read_text(encoding='utf-8').splitlines():
            line = line.strip()
            if line and not line.startswith('#'):
                names.add(line)
    if args.mode == 'names' and not names:
        parser.error('names mode requires --name or --names-file')

    args.output.parent.mkdir(parents=True, exist_ok=True)
    result = filter_dsn(args.source, args.output, args.mode, names)
    payload = json.dumps(result, indent=2) + '\n'
    print(payload, end='')
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(payload, encoding='utf-8')


if __name__ == '__main__':
    main()
