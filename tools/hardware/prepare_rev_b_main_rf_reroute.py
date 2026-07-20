#!/usr/bin/env python3
"""Remove only the Rev-B main-board RF copper that fails the layout audit."""

from __future__ import annotations

import argparse
import json
import re
from collections import Counter
from pathlib import Path

_TOKEN_RE = re.compile(r'"(?:\\.|[^"\\])*"|[^\s()]+')
_TARGET_NETS = {
    "/915 MHz RF/CC1121_RF",
    "/915 MHz RF/CC1190_VDD15",
    "/915 MHz RF/CC_DCPL_XOSC",
    "/915 MHz RF/CC_EXTCLK",
    "/915 MHz RF/CC_LNA_N",
    "/915 MHz RF/CC_LNA_P",
    "/915 MHz RF/VDD_CC112",
}


def unquote(token: str) -> str:
    if len(token) >= 2 and token[0] == '"' and token[-1] == '"':
        return re.sub(r"\\(.)", r"\1", token[1:-1])
    return token


def matching_paren(text: str, start: int) -> int:
    depth = 0
    quoted = False
    escaped = False
    comment = False
    for index in range(start, len(text)):
        char = text[index]
        if comment:
            if char == "\n":
                comment = False
        elif quoted:
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == '"':
                quoted = False
        else:
            if char == ";":
                comment = True
            elif char == '"':
                quoted = True
            elif char == "(":
                depth += 1
            elif char == ")":
                depth -= 1
                if depth == 0:
                    return index + 1
    raise ValueError("Unclosed S-expression")


def direct_child_spans(block: str) -> list[tuple[int, int]]:
    index = 1
    while index < len(block) and block[index].isspace():
        index += 1
    head = _TOKEN_RE.match(block, index)
    if head is None:
        raise ValueError("Missing root block head")
    index = head.end()
    spans: list[tuple[int, int]] = []
    while index < len(block) - 1:
        char = block[index]
        if char.isspace():
            index += 1
        elif char == ";":
            newline = block.find("\n", index)
            index = len(block) - 1 if newline < 0 else newline + 1
        elif char == "(":
            end = matching_paren(block, index)
            spans.append((index, end))
            index = end
        else:
            token = _TOKEN_RE.match(block, index)
            if token is None:
                raise ValueError(f"Unexpected token at offset {index}")
            index = token.end()
    return spans


def block_head(block: str) -> str | None:
    match = re.match(r"^\(\s*([^\s()]+)", block)
    return None if match is None else match.group(1)


def parse_net_declaration(block: str) -> tuple[int, str] | None:
    match = re.match(
        r'^\(\s*net\s+(\d+)\s+("(?:\\.|[^"\\])*"|[^\s()]+)\s*\)$',
        block,
        re.DOTALL,
    )
    if match is None:
        return None
    return int(match.group(1)), unquote(match.group(2))


def routed_net_code(block: str) -> int | None:
    match = re.search(r"\(\s*net\s+(\d+)\s*\)", block)
    return None if match is None else int(match.group(1))


def segment_layer(block: str) -> str:
    match = re.search(r'\(\s*layer\s+("(?:\\.|[^"\\])*"|[^\s()]+)\s*\)', block)
    return "unknown" if match is None else unquote(match.group(1))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--board", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()

    file_text = args.board.read_text(encoding="utf-8", errors="strict")
    root_start = file_text.find("(")
    if root_start < 0:
        raise SystemExit(f"No S-expression root in {args.board}")
    root_end = matching_paren(file_text, root_start)
    prefix = file_text[:root_start]
    root = file_text[root_start:root_end]
    suffix = file_text[root_end:]
    if suffix.strip():
        raise SystemExit(f"Unexpected content after KiCad PCB root in {args.board}")
    if not re.match(r"^\(kicad_pcb(?=[\s)])", root):
        raise SystemExit(f"Not a KiCad PCB file: {args.board}")

    spans = direct_child_spans(root)
    names_by_code: dict[int, str] = {}
    for start, end in spans:
        declaration = parse_net_declaration(root[start:end])
        if declaration is not None:
            names_by_code[declaration[0]] = declaration[1]

    missing = sorted(_TARGET_NETS - set(names_by_code.values()))
    if missing:
        raise SystemExit(f"Missing expected RF nets: {missing}")
    target_codes = {
        code for code, name in names_by_code.items() if name in _TARGET_NETS
    }

    removals: list[tuple[int, int]] = []
    removed_by_kind: Counter[str] = Counter()
    removed_by_net: Counter[str] = Counter()
    removed_by_layer: Counter[str] = Counter()
    for start, end in spans:
        block = root[start:end]
        head = block_head(block)
        if head not in {"segment", "via"}:
            continue
        code = routed_net_code(block)
        if code not in target_codes:
            continue
        kind = "via" if head == "via" else "segment"
        layer = "through" if kind == "via" else segment_layer(block)
        removals.append((start, end))
        removed_by_kind[kind] += 1
        removed_by_net[names_by_code[code]] += 1
        removed_by_layer[layer] += 1

    if not removals:
        raise SystemExit("No failing RF copper found; refusing no-op reroute base")

    output_root = root
    for start, end in reversed(removals):
        output_root = output_root[:start] + output_root[end:]

    remaining = 0
    for start, end in direct_child_spans(output_root):
        block = output_root[start:end]
        if block_head(block) not in {"segment", "via"}:
            continue
        if routed_net_code(block) in target_codes:
            remaining += 1
    if remaining:
        raise SystemExit(f"RF copper remained after strip: {remaining} objects")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(prefix + output_root + suffix, encoding="utf-8")
    report = {
        "target_net_count": len(_TARGET_NETS),
        "targets": sorted(_TARGET_NETS),
        "removed_item_count": len(removals),
        "removed_by_kind": dict(sorted(removed_by_kind.items())),
        "removed_by_net": dict(sorted(removed_by_net.items())),
        "removed_by_layer": dict(sorted(removed_by_layer.items())),
        "remaining_target_copper_items": remaining,
        "non_target_board_text_preserved": True,
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
