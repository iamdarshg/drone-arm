#!/usr/bin/env python3
"""Apply deterministic cleanup for router-generated KiCad DRC artifacts.

This operates on top-level copper objects by UUID, using KiCad's own DRC JSON as
its source of truth. It does not alter placement, pads, zones, or footprints.
"""
from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

UUID_RE = re.compile(r'\(uuid\s+"([0-9a-fA-F-]+)"\)')
WIDTH_RE = re.compile(r'(\(width\s+)([-+0-9.eE]+)(\))')
HEAD_RE = re.compile(r'^\s*\((\S+)')
NET_IN_DESC = re.compile(r'\[([^\]]+)\]')


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
    raise ValueError(f"unbalanced expression at offset {start}")


def root_children(text: str) -> list[tuple[int, int]]:
    root_start = text.index('(kicad_pcb')
    root_end = matching_close(text, root_start)
    index = text.index('\n', root_start) + 1
    children: list[tuple[int, int]] = []
    while index < root_end:
        while index < root_end and text[index].isspace():
            index += 1
        if index >= root_end:
            break
        if text[index] != '(':
            index += 1
            continue
        close = matching_close(text, index)
        children.append((index, close + 1))
        index = close + 1
    return children


def object_info(block: str) -> tuple[str | None, str | None]:
    head = HEAD_RE.match(block)
    uuid = UUID_RE.search(block)
    return (head.group(1) if head else None, uuid.group(1) if uuid else None)


def item_net(description: str) -> str | None:
    match = NET_IN_DESC.search(description)
    return match.group(1) if match else None


def cleanup(
    source: Path,
    drc_path: Path,
    output: Path,
    min_width_mm: float,
    remove_dangling_vias: bool,
) -> dict:
    text = source.read_text(encoding='utf-8')
    drc = json.loads(drc_path.read_text(encoding='utf-8'))

    remove_uuids: set[str] = set()
    widen_uuids: set[str] = set()
    skipped: list[dict] = []

    for violation in drc.get('violations', []):
        kind = violation.get('type')
        items = violation.get('items', [])
        if kind == 'track_width':
            for item in items:
                if item.get('uuid'):
                    widen_uuids.add(item['uuid'])
        elif kind == 'via_dangling' and remove_dangling_vias:
            for item in items:
                if item.get('uuid'):
                    remove_uuids.add(item['uuid'])
        elif kind == 'holes_co_located' and len(items) > 1:
            nets = {item_net(item.get('description', '')) for item in items}
            if len(nets) == 1 and None not in nets:
                for item in items[1:]:
                    if item.get('uuid'):
                        remove_uuids.add(item['uuid'])
            else:
                skipped.append({
                    'type': kind,
                    'reason': 'co-located holes are not all on the same named net',
                    'items': items,
                })

    children = root_children(text)
    uuid_to_index: dict[str, int] = {}
    blocks: list[str] = []
    kinds: list[str | None] = []
    for child_index, (start, end) in enumerate(children):
        block = text[start:end]
        kind, uuid = object_info(block)
        blocks.append(block)
        kinds.append(kind)
        if uuid:
            uuid_to_index[uuid] = child_index

    removed = []
    widened = []
    missing = []
    for uuid in sorted(remove_uuids):
        index = uuid_to_index.get(uuid)
        if index is None:
            missing.append({'uuid': uuid, 'action': 'remove'})
            continue
        if kinds[index] != 'via':
            skipped.append({'uuid': uuid, 'action': 'remove', 'kind': kinds[index]})
            continue
        blocks[index] = ''
        removed.append(uuid)

    for uuid in sorted(widen_uuids - remove_uuids):
        index = uuid_to_index.get(uuid)
        if index is None:
            missing.append({'uuid': uuid, 'action': 'widen'})
            continue
        if kinds[index] not in {'segment', 'arc'}:
            skipped.append({'uuid': uuid, 'action': 'widen', 'kind': kinds[index]})
            continue
        match = WIDTH_RE.search(blocks[index])
        if not match:
            skipped.append({'uuid': uuid, 'action': 'widen', 'reason': 'no width field'})
            continue
        old_width = float(match.group(2))
        if old_width >= min_width_mm:
            continue
        blocks[index] = WIDTH_RE.sub(
            lambda width: f"{width.group(1)}{min_width_mm:g}{width.group(3)}",
            blocks[index],
            count=1,
        )
        widened.append({
            'uuid': uuid,
            'old_width_mm': old_width,
            'new_width_mm': min_width_mm,
        })

    pieces: list[str] = []
    cursor = 0
    for (start, end), block in zip(children, blocks):
        pieces.append(text[cursor:start])
        pieces.append(block)
        cursor = end
    pieces.append(text[cursor:])
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(''.join(pieces), encoding='utf-8')

    return {
        'source': str(source),
        'drc': str(drc_path),
        'output': str(output),
        'removed_vias': len(removed),
        'widened_tracks': len(widened),
        'removed_via_uuids': removed,
        'widened': widened,
        'missing': missing,
        'skipped': skipped,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--drc', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--report', type=Path)
    parser.add_argument('--min-width-mm', type=float, default=0.2)
    parser.add_argument('--remove-dangling-vias', action='store_true')
    args = parser.parse_args()

    result = cleanup(
        args.source,
        args.drc,
        args.output,
        args.min_width_mm,
        args.remove_dangling_vias,
    )
    payload = json.dumps(result, indent=2) + '\n'
    print(payload, end='')
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(payload, encoding='utf-8')


if __name__ == '__main__':
    main()
