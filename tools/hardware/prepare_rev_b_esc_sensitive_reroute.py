#!/usr/bin/env python3
"""Remove unsafe preserved ESC gate-drive/current-sense copper before rerouting.

This is deliberately a surgical text rewrite rather than a pcbnew mutation. The
KiCad 9 SWIG bindings in Debian trixie can corrupt their track iterator after a
large batch of removals. Direct-child S-expression removal preserves every
non-target board object byte-for-byte and avoids that failure mode entirely.
"""

from __future__ import annotations

import argparse
import json
import re
from collections import Counter
from pathlib import Path

_TOKEN_RE = re.compile(r'"(?:\\.|[^"\\])*"|[^\s()]+')
_GATE_RE = re.compile(r"^/Motor [1-6] power cell/M[1-6]_G[HL][ABC](?:_DRV)?$")
_SENSE_RE = re.compile(
    r"_(?:SH[ABC]_[PN]|BUS_SH_[PN]|CSA_[ABC](?:_RAW)?|BUS_CURRENT(?:_RAW)?)$"
)
_EXPECTED_GATE_NETS = 72
_EXPECTED_SENSE_NETS = 96


def unquote(token: str) -> str:
    if len(token) >= 2 and token[0] == '"' and token[-1] == '"':
        return re.sub(r"\\(.)", r"\1", token[1:-1])
    return token


def matching_paren(text: str, start: int) -> int:
    if text[start] != "(":
        raise ValueError("Block does not start with '('")
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
    head_match = _TOKEN_RE.match(block, index)
    if not head_match:
        raise ValueError("Missing root block head")
    index = head_match.end()
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
            token_match = _TOKEN_RE.match(block, index)
            if not token_match:
                raise ValueError(f"Unexpected token at offset {index}")
            index = token_match.end()
    return spans


def block_head(block: str) -> str | None:
    match = re.match(r"^\(\s*([^\s()]+)", block)
    return None if match is None else match.group(1)


def classify(name: str) -> str | None:
    if _GATE_RE.fullmatch(name):
        return "gate"
    if _SENSE_RE.search(name):
        return "sense"
    return None


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
    source = file_text[root_start:root_end]
    suffix = file_text[root_end:]
    if suffix.strip():
        raise SystemExit(f"Unexpected content after KiCad PCB root in {args.board}")
    if not re.match(r"^\(kicad_pcb(?=[\s)])", source):
        raise SystemExit(f"Not a KiCad PCB file: {args.board}")

    spans = direct_child_spans(source)
    names_by_code: dict[int, str] = {}
    for start, end in spans:
        declaration = parse_net_declaration(source[start:end])
        if declaration is None:
            continue
        code, name = declaration
        if code in names_by_code:
            raise SystemExit(f"Duplicate KiCad net code {code}")
        names_by_code[code] = name

    gate_nets = sorted(name for name in names_by_code.values() if classify(name) == "gate")
    sense_nets = sorted(name for name in names_by_code.values() if classify(name) == "sense")
    if len(gate_nets) != _EXPECTED_GATE_NETS:
        raise SystemExit(
            f"Expected {_EXPECTED_GATE_NETS} gate nets, found {len(gate_nets)}"
        )
    if len(sense_nets) != _EXPECTED_SENSE_NETS:
        raise SystemExit(
            f"Expected {_EXPECTED_SENSE_NETS} sense nets, found {len(sense_nets)}"
        )

    target_codes = {
        code for code, name in names_by_code.items() if classify(name) is not None
    }
    removals: list[tuple[int, int]] = []
    removed_by_kind: Counter[str] = Counter()
    removed_by_group: Counter[str] = Counter()
    removed_by_layer: Counter[str] = Counter()
    removed_by_net: Counter[str] = Counter()

    for start, end in spans:
        block = source[start:end]
        head = block_head(block)
        if head not in {"segment", "via"}:
            continue
        code = routed_net_code(block)
        if code not in target_codes:
            continue
        name = names_by_code[code]
        group = classify(name)
        assert group is not None
        kind = "via" if head == "via" else "track"
        layer = "through" if kind == "via" else segment_layer(block)
        removals.append((start, end))
        removed_by_kind[kind] += 1
        removed_by_group[group] += 1
        removed_by_layer[layer] += 1
        removed_by_net[name] += 1

    if not removals:
        raise SystemExit("No sensitive routed copper found; refusing no-op reroute base")

    output_root = source
    for start, end in reversed(removals):
        output_root = output_root[:start] + output_root[end:]

    remaining_sensitive = 0
    for start, end in direct_child_spans(output_root):
        block = output_root[start:end]
        if block_head(block) not in {"segment", "via"}:
            continue
        code = routed_net_code(block)
        if code in target_codes:
            remaining_sensitive += 1
    if remaining_sensitive:
        raise SystemExit(
            f"Sensitive copper remained after removal: {remaining_sensitive} objects"
        )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(prefix + output_root + suffix, encoding="utf-8")
    report = {
        "source_board": str(args.board),
        "output_board": str(args.output),
        "gate_net_count": len(gate_nets),
        "sense_net_count": len(sense_nets),
        "target_net_count": len(gate_nets) + len(sense_nets),
        "removed_item_count": sum(removed_by_kind.values()),
        "removed_by_kind": dict(sorted(removed_by_kind.items())),
        "removed_by_group": dict(sorted(removed_by_group.items())),
        "removed_by_layer": dict(sorted(removed_by_layer.items())),
        "target_nets_with_removed_copper": len(removed_by_net),
        "target_nets_without_existing_copper": sorted(
            (set(gate_nets) | set(sense_nets)) - set(removed_by_net)
        ),
        "remaining_sensitive_copper_items": remaining_sensitive,
        "non_target_board_text_preserved": True,
        "prefix_bytes_preserved": len(prefix.encode("utf-8")),
        "suffix_bytes_preserved": len(suffix.encode("utf-8")),
        "rewrite_engine": "direct-child-kicad-s-expression",
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
