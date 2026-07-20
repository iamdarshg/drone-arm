#!/usr/bin/env python3
"""Limit a Specctra routing pass to an explicit set of named nets.

Target net pin demands are preserved exactly. Every non-target multi-pin net is
collapsed to one pin while all existing wiring remains in the DSN as an
obstacle. With FreeRouting fanout disabled, this prevents unrelated routed nets
from being treated as new routing work.
"""

from __future__ import annotations

import argparse
import json
import re
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


def find_head_block(text: str, head: str, start: int = 0) -> tuple[int, int]:
    match = re.search(r"\(" + re.escape(head) + r"(?=[\s)])", text[start:])
    if match is None:
        raise ValueError(f"Missing ({head} block")
    block_start = start + match.start()
    return block_start, matching_paren(text, block_start)


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


def parse_net(block: str) -> tuple[str, str] | None:
    match = re.match(
        r'^\(\s*net\s+("(?:\\.|[^"\\])*"|[^\s()]+)',
        block,
        re.DOTALL,
    )
    if match is None:
        return None
    return unquote(match.group(1)), block


def pin_tokens(block: str) -> tuple[re.Match[str], list[re.Match[str]]] | None:
    match = re.search(r"\(\s*pins(?P<body>[^()]*)\)", block, re.DOTALL)
    if match is None:
        return None
    return match, list(_TOKEN_RE.finditer(match.group("body")))


def collapse_to_one_pin(block: str) -> tuple[str, int, int]:
    parsed = pin_tokens(block)
    if parsed is None:
        return block, 0, 0
    match, tokens = parsed
    if len(tokens) <= 1:
        return block, len(tokens), len(tokens)
    body = match.group("body")
    leading = re.match(r"\s*", body).group(0)
    trailing = re.search(r"\s*$", body).group(0)
    replacement = "(pins" + leading + tokens[0].group(0) + trailing + ")"
    return block[: match.start()] + replacement + block[match.end() :], len(tokens), 1


def load_targets(path: Path) -> list[str]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if isinstance(data, dict):
        data = data.get("targets")
    if not isinstance(data, list) or not data or not all(isinstance(item, str) for item in data):
        raise ValueError("Target file must be a non-empty JSON list or {'targets': [...]} object")
    if len(set(data)) != len(data):
        raise ValueError("Target file contains duplicate net names")
    return data


def rewrite(source: str, targets: list[str]) -> tuple[str, dict]:
    network_start, network_end = find_head_block(source, "network")
    network = source[network_start:network_end]

    entries: dict[str, str] = {}
    pin_counts: dict[str, int] = {}
    for start, end in direct_child_spans(network):
        parsed = parse_net(network[start:end])
        if parsed is None:
            continue
        name, block = parsed
        if name in entries:
            raise ValueError(f"Duplicate network entry {name!r}")
        entries[name] = block
        pins = pin_tokens(block)
        pin_counts[name] = 0 if pins is None else len(pins[1])

    target_set = set(targets)
    missing = sorted(target_set - set(entries))
    if missing:
        raise ValueError(f"Target nets missing from Specctra network: {missing}")
    empty_targets = sorted(name for name in targets if pin_counts.get(name, 0) < 2)
    if empty_targets:
        raise ValueError(f"Target nets do not have at least two pins: {empty_targets}")

    replacements: list[tuple[int, int, str]] = []
    suppressed: dict[str, dict[str, int]] = {}
    for start, end in direct_child_spans(network):
        parsed = parse_net(network[start:end])
        if parsed is None or parsed[0] in target_set:
            continue
        rewritten, before, after = collapse_to_one_pin(parsed[1])
        if rewritten != parsed[1]:
            replacements.append((start, end, rewritten))
            suppressed[parsed[0]] = {"before": before, "after": after}

    focused_network = network
    for start, end, replacement in reversed(replacements):
        focused_network = focused_network[:start] + replacement + focused_network[end:]
    output = source[:network_start] + focused_network + source[network_end:]

    after_entries: dict[str, str] = {}
    for start, end in direct_child_spans(focused_network):
        parsed = parse_net(focused_network[start:end])
        if parsed is not None:
            after_entries[parsed[0]] = parsed[1]
    for name in targets:
        if entries[name] != after_entries[name]:
            raise ValueError(f"Target net entry changed while focusing {name!r}")

    report = {
        "target_net_count": len(targets),
        "targets": targets,
        "target_pin_counts": {name: pin_counts[name] for name in targets},
        "suppressed_non_target_net_count": len(suppressed),
        "suppressed_non_target_pin_demands": dict(sorted(suppressed.items())),
        "target_network_entries_preserved_byte_for_byte": True,
        "all_existing_wiring_preserved": True,
        "classes_preserved_byte_for_byte": True,
    }
    return output, report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--targets", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()

    targets = load_targets(args.targets)
    output, report = rewrite(
        args.input.read_text(encoding="utf-8", errors="strict"), targets
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(output, encoding="utf-8")
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
