#!/usr/bin/env python3
"""Transplant selected routed-net copper between topology-identical KiCad boards."""

from __future__ import annotations

import argparse
import json
import re
from collections import Counter
from pathlib import Path

_TOKEN_RE = re.compile(r'"(?:\\.|[^"\\])*"|[^\s()]+')


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


def split_root(file_text: str) -> tuple[str, str, str]:
    root_start = file_text.find("(")
    if root_start < 0:
        raise ValueError("No S-expression root")
    root_end = matching_paren(file_text, root_start)
    prefix = file_text[:root_start]
    root = file_text[root_start:root_end]
    suffix = file_text[root_end:]
    if suffix.strip():
        raise ValueError("Unexpected content after KiCad PCB root")
    if not re.match(r"^\(kicad_pcb(?=[\s)])", root):
        raise ValueError("Root is not a KiCad PCB")
    return prefix, root, suffix


def direct_child_spans(block: str) -> list[tuple[int, int]]:
    index = 1
    while index < len(block) and block[index].isspace():
        index += 1
    head = _TOKEN_RE.match(block, index)
    if head is None:
        raise ValueError("Missing block head")
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


def rewrite_routed_net_code(block: str, code: int) -> str:
    rewritten, count = re.subn(
        r"\(\s*net\s+\d+\s*\)", f"(net {code})", block, count=1
    )
    if count != 1:
        raise ValueError("Routed copper block does not contain exactly one net code")
    return rewritten


def segment_layer(block: str) -> str:
    match = re.search(r'\(\s*layer\s+("(?:\\.|[^"\\])*"|[^\s()]+)\s*\)', block)
    return "unknown" if match is None else unquote(match.group(1))


def board_snapshot(root: str) -> tuple[dict[int, str], dict[str, int], list[tuple[int, int]]]:
    spans = direct_child_spans(root)
    names_by_code: dict[int, str] = {}
    codes_by_name: dict[str, int] = {}
    for start, end in spans:
        declaration = parse_net_declaration(root[start:end])
        if declaration is None:
            continue
        code, name = declaration
        if code in names_by_code or name in codes_by_name:
            raise ValueError(f"Duplicate net declaration for {name!r}/{code}")
        names_by_code[code] = name
        codes_by_name[name] = code
    return names_by_code, codes_by_name, spans


def extract_copper(
    root: str,
    names_by_code: dict[int, str],
    spans: list[tuple[int, int]],
    targets: set[str],
) -> tuple[list[str], list[tuple[int, int]], dict]:
    blocks: list[str] = []
    removals: list[tuple[int, int]] = []
    by_kind: Counter[str] = Counter()
    by_layer: Counter[str] = Counter()
    by_net: Counter[str] = Counter()
    for start, end in spans:
        block = root[start:end]
        head = block_head(block)
        if head not in {"segment", "via"}:
            continue
        code = routed_net_code(block)
        name = names_by_code.get(code)
        if name not in targets:
            continue
        blocks.append(block)
        removals.append((start, end))
        kind = "via" if head == "via" else "segment"
        layer = "through" if kind == "via" else segment_layer(block)
        by_kind[kind] += 1
        by_layer[layer] += 1
        by_net[name] += 1
    return blocks, removals, {
        "item_count": len(blocks),
        "by_kind": dict(sorted(by_kind.items())),
        "by_layer": dict(sorted(by_layer.items())),
        "by_net": dict(sorted(by_net.items())),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--donor", type=Path, required=True)
    parser.add_argument("--targets", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--require-layer", action="append", default=[])
    parser.add_argument("--forbid-vias", action="store_true")
    args = parser.parse_args()

    target_data = json.loads(args.targets.read_text(encoding="utf-8"))
    if isinstance(target_data, dict):
        target_data = target_data.get("targets")
    if not isinstance(target_data, list) or not target_data or not all(
        isinstance(item, str) for item in target_data
    ):
        raise SystemExit("Targets must be a non-empty JSON string list")
    targets = set(target_data)
    if len(targets) != len(target_data):
        raise SystemExit("Targets contain duplicates")

    source_prefix, source_root, source_suffix = split_root(
        args.source.read_text(encoding="utf-8", errors="strict")
    )
    _donor_prefix, donor_root, _donor_suffix = split_root(
        args.donor.read_text(encoding="utf-8", errors="strict")
    )
    source_names, source_codes, source_spans = board_snapshot(source_root)
    donor_names, donor_codes, donor_spans = board_snapshot(donor_root)

    missing_source = sorted(targets - set(source_codes))
    missing_donor = sorted(targets - set(donor_codes))
    if missing_source or missing_donor:
        raise SystemExit(
            f"Missing target declarations: source={missing_source}, donor={missing_donor}"
        )

    source_blocks, source_removals, source_stats = extract_copper(
        source_root, source_names, source_spans, targets
    )
    donor_blocks, _donor_spans, donor_stats = extract_copper(
        donor_root, donor_names, donor_spans, targets
    )
    if not donor_blocks:
        raise SystemExit("Donor contains no routed copper for target nets")
    if args.forbid_vias and donor_stats["by_kind"].get("via", 0):
        raise SystemExit(f"Donor target copper contains vias: {donor_stats}")
    required_layers = set(args.require_layer)
    donor_layers = set(donor_stats["by_layer"])
    donor_layers.discard("through")
    if required_layers and not donor_layers <= required_layers:
        raise SystemExit(
            f"Donor target copper uses disallowed layers {sorted(donor_layers - required_layers)}"
        )

    rewritten_donor = []
    for block in donor_blocks:
        donor_code = routed_net_code(block)
        name = donor_names[donor_code]
        rewritten_donor.append(rewrite_routed_net_code(block, source_codes[name]))

    without_source = source_root
    for start, end in reversed(source_removals):
        without_source = without_source[:start] + without_source[end:]

    insertion = without_source.rfind("\n)")
    if insertion < 0:
        insertion = len(without_source) - 1
    donor_text = "\n" + "\n".join(rewritten_donor) + "\n"
    output_root = without_source[:insertion] + donor_text + without_source[insertion:]

    output_names, _output_codes, output_spans = board_snapshot(output_root)
    output_blocks, _output_removals, output_stats = extract_copper(
        output_root, output_names, output_spans, targets
    )
    if len(output_blocks) != len(donor_blocks):
        raise SystemExit("Output target-copper count does not match donor")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(source_prefix + output_root + source_suffix, encoding="utf-8")
    report = {
        "source": str(args.source),
        "donor": str(args.donor),
        "output": str(args.output),
        "target_net_count": len(targets),
        "targets": sorted(targets),
        "source_target_copper": source_stats,
        "donor_target_copper": donor_stats,
        "output_target_copper": output_stats,
        "forbid_vias": args.forbid_vias,
        "required_layers": sorted(required_layers),
        "non_target_source_text_preserved": True,
        "donor_net_codes_rewritten_to_source": True,
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
