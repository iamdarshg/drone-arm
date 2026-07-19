#!/usr/bin/env python3
"""Suppress zone-backed nets from Specctra routing demand without reformatting.

FreeRouting's DSN reader is sensitive to the exporter layout of ``(net NAME ...)``
entries.  Re-serializing the whole network block can therefore make valid nets
invisible even when the resulting S-expression is structurally correct.  This
filter performs surgical text replacements only inside the ``(pins ...)`` block
of zone-backed nets.  Every other network entry and the complete wiring block
remain byte-for-byte identical to KiCad's export.
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
    index = start
    while index < len(text):
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
        index += 1
    raise ValueError("Unclosed S-expression")


def find_head_block(text: str, block_head: str, start: int = 0) -> tuple[int, int]:
    pattern = re.compile(r"\(" + re.escape(block_head) + r"(?=[\s)])")
    match = pattern.search(text, start)
    if not match:
        raise ValueError(f"Missing ({block_head} block")
    return match.start(), matching_paren(text, match.start())


def direct_child_spans(block: str) -> list[tuple[int, int]]:
    """Return spans for direct child S-expressions of one complete block."""

    index = 1
    while index < len(block) and block[index].isspace():
        index += 1
    head_match = _TOKEN_RE.match(block, index)
    if not head_match:
        raise ValueError("Missing block head")
    index = head_match.end()

    spans: list[tuple[int, int]] = []
    while index < len(block) - 1:
        char = block[index]
        if char.isspace():
            index += 1
            continue
        if char == ";":
            newline = block.find("\n", index)
            index = len(block) - 1 if newline < 0 else newline + 1
            continue
        if char == "(":
            end = matching_paren(block, index)
            spans.append((index, end))
            index = end
            continue
        token_match = _TOKEN_RE.match(block, index)
        if not token_match:
            raise ValueError(f"Unexpected token at offset {index}")
        index = token_match.end()
    return spans


def net_name_and_pins(child: str) -> tuple[str, list[str]] | None:
    match = re.match(
        r'^\(\s*net\s+("(?:\\.|[^"\\])*"|[^\s()]+)',
        child,
        re.DOTALL,
    )
    if not match:
        return None

    name = unquote(match.group(1))
    pins_start, pins_end = find_head_block(child, "pins")
    tokens = _TOKEN_RE.findall(child[pins_start + 1 : pins_end - 1])
    if not tokens or tokens[0] != "pins":
        raise ValueError(f"Malformed pins block for {name!r}")
    return name, tokens[1:]


def zone_net_names(board_text: str) -> set[str]:
    names: set[str] = set()
    for match in re.finditer(r"\(zone\b", board_text):
        header = board_text[match.start() : match.start() + 1600]
        net_match = re.search(r'\(net_name\s+"((?:\\.|[^"\\])*)"\)', header)
        if net_match:
            name = re.sub(r"\\(.)", r"\1", net_match.group(1))
            if name:
                names.add(name)
    return names


def network_snapshot(network_block: str) -> dict[str, tuple[int, str]]:
    snapshot: dict[str, tuple[int, str]] = {}
    for start, end in direct_child_spans(network_block):
        child = network_block[start:end]
        parsed = net_name_and_pins(child)
        if parsed is None:
            continue
        name, pins = parsed
        if name in snapshot:
            raise ValueError(f"Duplicate network net {name!r}")
        snapshot[name] = (len(pins), child)
    return snapshot


def suppress_plane_net_pins(
    dsn_text: str,
    zone_nets: set[str],
) -> tuple[str, dict[str, int], int]:
    network_start, network_end = find_head_block(dsn_text, "network")
    network = dsn_text[network_start:network_end]
    before = network_snapshot(network)

    missing = sorted(zone_nets - set(before))
    if missing:
        raise ValueError(f"Zone-backed nets missing from network block: {missing}")

    replacements: list[tuple[int, int, str]] = []
    original_pin_counts: dict[str, int] = {}
    for child_start, child_end in direct_child_spans(network):
        child = network[child_start:child_end]
        parsed = net_name_and_pins(child)
        if parsed is None:
            continue
        name, pins = parsed
        if name not in zone_nets:
            continue
        if not pins:
            raise ValueError(f"Zone-backed net {name!r} contains no pins")

        pins_start, pins_end = find_head_block(child, "pins")
        replacements.append(
            (child_start + pins_start, child_start + pins_end, f"(pins {pins[0]})")
        )
        original_pin_counts[name] = len(pins)

    filtered_network = network
    for start, end, replacement in reversed(replacements):
        filtered_network = (
            filtered_network[:start] + replacement + filtered_network[end:]
        )
    filtered = dsn_text[:network_start] + filtered_network + dsn_text[network_end:]

    after = network_snapshot(filtered_network)
    if set(before) != set(after):
        raise ValueError("Network names changed during plane-net filtering")
    for name, (before_count, before_text) in before.items():
        after_count, after_text = after[name]
        if name in zone_nets:
            if after_count != 1:
                raise ValueError(
                    f"Plane net {name!r} retained {after_count} pins instead of 1"
                )
        elif before_text != after_text:
            raise ValueError(f"Non-suppressed net {name!r} changed")

    wiring_start, wiring_end = find_head_block(dsn_text, "wiring")
    filtered_wiring_start, filtered_wiring_end = find_head_block(filtered, "wiring")
    if dsn_text[wiring_start:wiring_end] != filtered[
        filtered_wiring_start:filtered_wiring_end
    ]:
        raise ValueError("Wiring block changed during plane-net filtering")

    return filtered, original_pin_counts, len(before)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--board", type=Path, required=True)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()

    zone_nets = zone_net_names(
        args.board.read_text(encoding="utf-8", errors="replace")
    )
    if not zone_nets:
        raise SystemExit("No zone-backed nets found; refusing to emit an unfiltered DSN")

    source = args.input.read_text(encoding="utf-8", errors="strict")
    filtered, original_pin_counts, network_net_count = suppress_plane_net_pins(
        source,
        zone_nets,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(filtered, encoding="utf-8")

    wiring_start, wiring_end = find_head_block(filtered, "wiring")
    wiring = filtered[wiring_start:wiring_end]
    report = {
        "zone_backed_nets_on_board": sorted(zone_nets),
        "zone_backed_net_count_on_board": len(zone_nets),
        "suppressed_router_nets": sorted(original_pin_counts),
        "suppressed_router_net_count": len(original_pin_counts),
        "network_net_count": network_net_count,
        "original_pin_counts": original_pin_counts,
        "removed_pin_references": sum(
            count - 1 for count in original_pin_counts.values()
        ),
        "retained_pin_references": len(original_pin_counts),
        "preserved_plane_wiring_mentions": {
            name: wiring.count(name) for name in sorted(original_pin_counts)
        },
        "non_suppressed_network_entries_preserved": True,
        "wiring_block_preserved_byte_for_byte": True,
        "router_input_dsn": str(args.output),
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
