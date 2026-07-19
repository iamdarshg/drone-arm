#!/usr/bin/env python3
"""Remove zone-backed nets from a Specctra DSN routing demand.

The DSN's existing wiring is preserved so already-routed plane copper remains
visible to FreeRouting as protected geometry. Only the matching ``network/net``
entries and class references are removed.
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Iterator


class Atom(str):
    """An S-expression atom retaining its original quoting."""


Node = Atom | list["Node"]


def tokenize(text: str) -> Iterator[str]:
    index = 0
    while index < len(text):
        char = text[index]
        if char.isspace():
            index += 1
            continue
        if char in "()":
            yield char
            index += 1
            continue
        if char == ";":
            newline = text.find("\n", index)
            index = len(text) if newline < 0 else newline + 1
            continue
        if char == '"':
            end = index + 1
            escaped = False
            while end < len(text):
                current = text[end]
                if escaped:
                    escaped = False
                elif current == "\\":
                    escaped = True
                elif current == '"':
                    end += 1
                    break
                end += 1
            if end > len(text) or text[end - 1] != '"':
                raise ValueError(f"Unterminated quoted token at offset {index}")
            yield text[index:end]
            index = end
            continue
        end = index
        while end < len(text) and not text[end].isspace() and text[end] not in "();":
            end += 1
        yield text[index:end]
        index = end


def parse_one(text: str) -> Node:
    roots: list[Node] = []
    stack: list[list[Node]] = []
    for token in tokenize(text):
        if token == "(":
            node: list[Node] = []
            (stack[-1] if stack else roots).append(node)
            stack.append(node)
        elif token == ")":
            if not stack:
                raise ValueError("Unexpected closing parenthesis")
            stack.pop()
        else:
            (stack[-1] if stack else roots).append(Atom(token))
    if stack or len(roots) != 1:
        raise ValueError("Malformed S-expression")
    return roots[0]


def atom_value(node: Node) -> str | None:
    if not isinstance(node, Atom):
        return None
    raw = str(node)
    if len(raw) >= 2 and raw[0] == '"' and raw[-1] == '"':
        return re.sub(r"\\(.)", r"\1", raw[1:-1])
    return raw


def head(node: Node) -> str | None:
    return atom_value(node[0]) if isinstance(node, list) and node else None


def serialize(node: Node, depth: int = 0) -> str:
    if isinstance(node, Atom):
        return str(node)
    if not node:
        return "()"
    if all(isinstance(child, Atom) for child in node) and sum(
        len(str(child)) + 1 for child in node
    ) < 100:
        return "(" + " ".join(map(str, node)) + ")"

    indent = "  " * depth
    child_indent = "  " * (depth + 1)
    if isinstance(node[0], Atom):
        if len(node) == 1:
            return "(" + str(node[0]) + ")"
        children = ("\n" + child_indent).join(
            serialize(child, depth + 1) for child in node[1:]
        )
        return f"({node[0]}\n{child_indent}{children}\n{indent})"

    children = ("\n" + child_indent).join(
        serialize(child, depth + 1) for child in node
    )
    return f"(\n{child_indent}{children}\n{indent})"


def find_block(text: str, marker: str) -> tuple[int, int]:
    start = text.find(marker)
    if start < 0:
        raise ValueError(f"Missing {marker}")

    depth = 0
    quoted = False
    escaped = False
    index = start
    while index < len(text):
        char = text[index]
        if quoted:
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == '"':
                quoted = False
        else:
            if char == '"':
                quoted = True
            elif char == "(":
                depth += 1
            elif char == ")":
                depth -= 1
                if depth == 0:
                    return start, index + 1
        index += 1
    raise ValueError(f"Unclosed {marker}")


def zone_net_names(board_text: str) -> set[str]:
    names: set[str] = set()
    for match in re.finditer(r"\(zone\b", board_text):
        header = board_text[match.start() : match.start() + 1200]
        net_match = re.search(r'\(net_name\s+"((?:\\.|[^"\\])*)"\)', header)
        if net_match:
            name = re.sub(r"\\(.)", r"\1", net_match.group(1))
            if name:
                names.add(name)
    return names


def filter_plane_nets(dsn_text: str, omitted: set[str]) -> tuple[str, int, int]:
    block_start, block_end = find_block(dsn_text, "(network")
    network = parse_one(dsn_text[block_start:block_end])
    if head(network) != "network" or not isinstance(network, list):
        raise ValueError("Located block is not a network block")

    removed_nets = 0
    removed_class_refs = 0
    filtered: list[Node] = [network[0]]
    for child in network[1:]:
        child_head = head(child)
        if (
            child_head == "net"
            and isinstance(child, list)
            and len(child) > 1
            and atom_value(child[1]) in omitted
        ):
            removed_nets += 1
            continue
        if child_head == "class" and isinstance(child, list) and len(child) > 2:
            rebuilt: list[Node] = child[:2]
            for item in child[2:]:
                if isinstance(item, Atom) and atom_value(item) in omitted:
                    removed_class_refs += 1
                    continue
                rebuilt.append(item)
            child = rebuilt
        filtered.append(child)

    if removed_nets == 0:
        raise ValueError("No zone-backed network entries were removed")
    filtered_block = serialize(filtered)
    return (
        dsn_text[:block_start] + filtered_block + dsn_text[block_end:],
        removed_nets,
        removed_class_refs,
    )


def verify_filtered_network(dsn_text: str, omitted: set[str]) -> None:
    start, end = find_block(dsn_text, "(network")
    network = parse_one(dsn_text[start:end])
    if not isinstance(network, list):
        raise ValueError("Filtered network is malformed")

    leaked_nets = sorted(
        atom_value(child[1])
        for child in network[1:]
        if head(child) == "net"
        and isinstance(child, list)
        and len(child) > 1
        and atom_value(child[1]) in omitted
    )
    leaked_class_refs = sorted(
        atom_value(item)
        for child in network[1:]
        if head(child) == "class" and isinstance(child, list)
        for item in child[2:]
        if isinstance(item, Atom) and atom_value(item) in omitted
    )
    if leaked_nets or leaked_class_refs:
        raise ValueError(
            f"Plane-net filter verification failed: {leaked_nets}, {leaked_class_refs}"
        )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--board", type=Path, required=True)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()

    omitted = zone_net_names(args.board.read_text(encoding="utf-8", errors="replace"))
    if not omitted:
        raise SystemExit("No zone-backed nets found; refusing to emit an unfiltered DSN")

    source = args.input.read_text(encoding="utf-8", errors="strict")
    filtered, removed_nets, removed_class_refs = filter_plane_nets(source, omitted)
    verify_filtered_network(filtered, omitted)
    args.output.write_text(filtered, encoding="utf-8")

    wiring_start = filtered.find("(wiring")
    wiring = filtered[wiring_start:] if wiring_start >= 0 else ""
    report = {
        "omitted_zone_backed_nets": sorted(omitted),
        "omitted_zone_backed_net_count": len(omitted),
        "removed_network_entries": removed_nets,
        "removed_class_references": removed_class_refs,
        "preserved_plane_wiring_mentions": {
            name: wiring.count(name) for name in sorted(omitted)
        },
        "router_input_dsn": str(args.output),
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
