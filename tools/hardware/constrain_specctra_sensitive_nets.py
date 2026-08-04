#!/usr/bin/env python3
"""Force Rev-B ESC gate-drive and current-sense nets onto F.Cu without vias.

The input is a KiCad-exported Specctra DSN. This tool removes preserved wiring
for audit-sensitive nets, removes those nets from their original classes, and
adds dedicated top-layer/no-via classes. Network pin entries are preserved
byte-for-byte so FreeRouting continues to see every connection correctly.
"""

from __future__ import annotations

import argparse
import json
import re
from collections import Counter, defaultdict
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


def quote(value: str) -> str:
    return json.dumps(value, ensure_ascii=False)


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
    if not match:
        raise ValueError(f"Missing ({head} block")
    block_start = start + match.start()
    return block_start, matching_paren(text, block_start)


def direct_child_spans(block: str) -> list[tuple[int, int]]:
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


def net_name_and_text(child: str) -> tuple[str, str] | None:
    match = re.match(
        r'^\(\s*net\s+("(?:\\.|[^"\\])*"|[^\s()]+)', child, re.DOTALL
    )
    if not match:
        return None
    return unquote(match.group(1)), child


def network_snapshot(network: str) -> dict[str, str]:
    result: dict[str, str] = {}
    for start, end in direct_child_spans(network):
        parsed = net_name_and_text(network[start:end])
        if parsed is None:
            continue
        name, text = parsed
        if name in result:
            raise ValueError(f"Duplicate net entry {name!r}")
        result[name] = text
    return result


def classify(name: str) -> str | None:
    if _GATE_RE.fullmatch(name):
        return "gate"
    if _SENSE_RE.search(name):
        return "sense"
    return None


def parse_class(child: str) -> tuple[str, str, list[tuple[str, str]], int] | None:
    if not re.match(r"^\(\s*class(?=[\s)])", child):
        return None
    child_spans = direct_child_spans(child)
    nested_start = child_spans[0][0] if child_spans else len(child) - 1
    header = child[:nested_start]
    tokens = list(_TOKEN_RE.finditer(header))
    if len(tokens) < 2 or tokens[0].group(0) != "class":
        raise ValueError("Malformed class block")
    class_token = tokens[1].group(0)
    class_name = unquote(class_token)
    nets = [(unquote(match.group(0)), match.group(0)) for match in tokens[2:]]
    return class_name, class_token, nets, nested_start


def render_class(child: str, class_token: str, net_tokens: list[str], nested_start: int) -> str:
    lines = [f"(class {class_token}"]
    for token in net_tokens:
        if len(lines[-1]) + 1 + len(token) <= 108:
            lines[-1] += " " + token
        else:
            lines.append("      " + token)
    header = "\n".join(lines)
    return header + child[nested_start:-1] + ")"


def render_restricted_class(name: str, nets: list[str], width: int) -> str:
    lines = [f"    (class {name}"]
    for net in nets:
        token = quote(net)
        if len(lines[-1]) + 1 + len(token) <= 108:
            lines[-1] += " " + token
        else:
            lines.append("      " + token)
    lines.extend(
        [
            "      (circuit",
            '        (use_via "__NO_VIAS__")',
            "        (use_layer F.Cu)",
            "      )",
            "      (rule",
            f"        (width {width})",
            "        (clearance 200)",
            "      )",
            "    )",
        ]
    )
    return "\n".join(lines)


def class_memberships(network: str) -> dict[str, set[str]]:
    memberships: dict[str, set[str]] = defaultdict(set)
    for start, end in direct_child_spans(network):
        parsed = parse_class(network[start:end])
        if parsed is None:
            continue
        class_name, _, nets, _ = parsed
        for net_name, _ in nets:
            memberships[net_name].add(class_name)
    return memberships


def rewrite_classes(network: str, targets: set[str], gate_nets: list[str], sense_nets: list[str]) -> str:
    replacements: list[tuple[int, int, str]] = []
    for start, end in direct_child_spans(network):
        child = network[start:end]
        parsed = parse_class(child)
        if parsed is None:
            continue
        _, class_token, nets, nested_start = parsed
        retained = [token for name, token in nets if name not in targets]
        if len(retained) != len(nets):
            replacements.append(
                (start, end, render_class(child, class_token, retained, nested_start))
            )
    result = network
    for start, end, replacement in reversed(replacements):
        result = result[:start] + replacement + result[end:]
    additions = (
        "\n"
        + render_restricted_class("FAB_GATE_TOP", gate_nets, 500)
        + "\n"
        + render_restricted_class("FAB_SENSE_TOP", sense_nets, 250)
        + "\n  "
    )
    return result[:-1] + additions + ")"


def wiring_net(child: str) -> str | None:
    match = re.search(
        r'\(\s*net\s+("(?:\\.|[^"\\])*"|[^\s()]+)\s*\)', child, re.DOTALL
    )
    return None if match is None else unquote(match.group(1))


def remove_target_wiring(wiring: str, targets: set[str]) -> tuple[str, Counter, Counter]:
    replacements: list[tuple[int, int]] = []
    removed_by_kind: Counter = Counter()
    removed_by_net: Counter = Counter()
    for start, end in direct_child_spans(wiring):
        child = wiring[start:end]
        net = wiring_net(child)
        if net not in targets:
            continue
        head = _TOKEN_RE.search(child[1:])
        kind = "unknown" if head is None else head.group(0)
        replacements.append((start, end))
        removed_by_kind[kind] += 1
        removed_by_net[net] += 1
    result = wiring
    for start, end in reversed(replacements):
        result = result[:start] + result[end:]
    return result, removed_by_kind, removed_by_net


def constrain(source: str) -> tuple[str, dict]:
    network_start, network_end = find_head_block(source, "network")
    network = source[network_start:network_end]
    before_nets = network_snapshot(network)
    gate_nets = sorted(name for name in before_nets if classify(name) == "gate")
    sense_nets = sorted(name for name in before_nets if classify(name) == "sense")
    if len(gate_nets) != _EXPECTED_GATE_NETS:
        raise ValueError(f"Expected {_EXPECTED_GATE_NETS} gate nets, found {len(gate_nets)}")
    if len(sense_nets) != _EXPECTED_SENSE_NETS:
        raise ValueError(f"Expected {_EXPECTED_SENSE_NETS} sense nets, found {len(sense_nets)}")
    targets = set(gate_nets) | set(sense_nets)
    memberships_before = class_memberships(network)

    constrained_network = rewrite_classes(network, targets, gate_nets, sense_nets)
    after_nets = network_snapshot(constrained_network)
    if before_nets != after_nets:
        raise ValueError("Network pin entries changed while applying sensitive-net constraints")
    memberships_after = class_memberships(constrained_network)
    for name in before_nets:
        if name in targets:
            expected = {"FAB_GATE_TOP" if name in gate_nets else "FAB_SENSE_TOP"}
            if memberships_after.get(name, set()) != expected:
                raise ValueError(
                    f"Sensitive net {name!r} has memberships {memberships_after.get(name)}"
                )
        elif memberships_before.get(name, set()) != memberships_after.get(name, set()):
            raise ValueError(f"Non-sensitive class membership changed for {name!r}")

    interim = source[:network_start] + constrained_network + source[network_end:]
    wiring_start, wiring_end = find_head_block(interim, "wiring")
    wiring = interim[wiring_start:wiring_end]
    constrained_wiring, removed_by_kind, removed_by_net = remove_target_wiring(wiring, targets)
    for start, end in direct_child_spans(constrained_wiring):
        net = wiring_net(constrained_wiring[start:end])
        if net in targets:
            raise ValueError(f"Sensitive wiring remained for {net!r}")
    output = interim[:wiring_start] + constrained_wiring + interim[wiring_end:]

    report = {
        "gate_net_count": len(gate_nets),
        "sense_net_count": len(sense_nets),
        "target_net_count": len(targets),
        "removed_wiring_objects": sum(removed_by_kind.values()),
        "removed_wiring_by_kind": dict(sorted(removed_by_kind.items())),
        "target_nets_with_removed_wiring": len(removed_by_net),
        "target_nets_without_existing_wiring": sorted(targets - set(removed_by_net)),
        "classes": {
            "FAB_GATE_TOP": {
                "net_count": len(gate_nets),
                "width_dsn_units": 500,
                "layers": ["F.Cu"],
                "via_rule": "empty via rule via deliberately unresolved __NO_VIAS__ padstack",
            },
            "FAB_SENSE_TOP": {
                "net_count": len(sense_nets),
                "width_dsn_units": 250,
                "layers": ["F.Cu"],
                "via_rule": "empty via rule via deliberately unresolved __NO_VIAS__ padstack",
            },
        },
        "network_pin_entries_preserved_byte_for_byte": True,
        "non_sensitive_class_memberships_preserved": True,
        "sensitive_wiring_removed": True,
    }
    return output, report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    source = args.input.read_text(encoding="utf-8", errors="strict")
    output, report = constrain(source)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(output, encoding="utf-8")
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
