#!/usr/bin/env python3
"""Focus a Specctra pass on unresolved main-board nets and unsafe RF routes.

Non-target pin demands are collapsed to one pin while their existing wiring is
preserved as an obstacle. Seven RF nets are removed from their previous classes
and wiring, then assigned F.Cu-only classes with no available via padstack.
"""

from __future__ import annotations

import argparse
import json
import re
from collections import Counter
from pathlib import Path

_TOKEN_RE = re.compile(r'"(?:\\.|[^"\\])*"|[^\s()]+')
_EXTRA_RF_TARGETS = {
    "/915 MHz RF/CC1121_RF",
    "/915 MHz RF/CC_DCPL_XOSC",
    "/915 MHz RF/CC_EXTCLK",
    "/915 MHz RF/CC_LNA_N",
}
_RF_GROUPS: dict[str, tuple[list[str], int]] = {
    "FAB_RF_250": (
        [
            "/915 MHz RF/CC_DCPL_XOSC",
            "/915 MHz RF/CC_EXTCLK",
        ],
        250,
    ),
    "FAB_RF_350": (
        [
            "/915 MHz RF/CC1121_RF",
            "/915 MHz RF/CC_LNA_N",
            "/915 MHz RF/CC_LNA_P",
        ],
        350,
    ),
    "FAB_RF_800": (
        [
            "/915 MHz RF/VDD_CC112",
            "/915 MHz RF/CC1190_VDD15",
        ],
        800,
    ),
}


def unquote(token: str) -> str:
    if len(token) >= 2 and token[0] == '"' and token[-1] == '"':
        return re.sub(r"\\(.)", r"\1", token[1:-1])
    return token


def quote(value: str) -> str:
    return json.dumps(value, ensure_ascii=False)


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


def suppress_pin_demand(block: str) -> tuple[str, int, int]:
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


def parse_class(
    block: str,
) -> tuple[str, str, list[tuple[str, str]], int] | None:
    if not re.match(r"^\(\s*class(?=[\s)])", block):
        return None
    children = direct_child_spans(block)
    nested_start = children[0][0] if children else len(block) - 1
    header = block[:nested_start]
    tokens = list(_TOKEN_RE.finditer(header))
    if len(tokens) < 2:
        raise ValueError("Malformed class block")
    return (
        unquote(tokens[1].group(0)),
        tokens[1].group(0),
        [(unquote(token.group(0)), token.group(0)) for token in tokens[2:]],
        nested_start,
    )


def render_class(
    block: str, class_token: str, net_tokens: list[str], nested_start: int
) -> str:
    lines = [f"(class {class_token}"]
    for token in net_tokens:
        if len(lines[-1]) + 1 + len(token) <= 108:
            lines[-1] += " " + token
        else:
            lines.append("      " + token)
    return "\n".join(lines) + block[nested_start:-1] + ")"


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


def wiring_net(block: str) -> str | None:
    match = re.search(
        r'\(\s*net\s+("(?:\\.|[^"\\])*"|[^\s()]+)\s*\)',
        block,
        re.DOTALL,
    )
    return None if match is None else unquote(match.group(1))


def targets_from_drc(path: Path) -> Counter[str]:
    data = json.loads(path.read_text(encoding="utf-8"))
    targets: Counter[str] = Counter()
    for unconnected in data.get("unconnected_items", []):
        names: set[str] = set()
        for item in unconnected.get("items", []):
            names.update(re.findall(r"\[([^\]]+)\]", item.get("description", "")))
        for name in names:
            targets[name] += 1
    return targets


def rewrite(source: str, drc_path: Path) -> tuple[str, dict]:
    network_start, network_end = find_head_block(source, "network")
    network = source[network_start:network_end]
    network_entries: dict[str, str] = {}
    for start, end in direct_child_spans(network):
        parsed = parse_net(network[start:end])
        if parsed is not None:
            network_entries[parsed[0]] = parsed[1]

    unconnected_targets = targets_from_drc(drc_path)
    targets = set(unconnected_targets) | _EXTRA_RF_TARGETS
    restricted_rf = set().union(
        *(set(nets) for nets, _width in _RF_GROUPS.values())
    )
    missing = sorted(targets - set(network_entries))
    if missing:
        raise ValueError(f"Target nets missing from Specctra network: {missing}")

    replacements: list[tuple[int, int, str]] = []
    suppressed: dict[str, dict[str, int]] = {}
    for start, end in direct_child_spans(network):
        parsed = parse_net(network[start:end])
        if parsed is None or parsed[0] in targets:
            continue
        rewritten, before, after = suppress_pin_demand(parsed[1])
        if rewritten != parsed[1]:
            replacements.append((start, end, rewritten))
            suppressed[parsed[0]] = {"before": before, "after": after}

    for start, end in direct_child_spans(network):
        block = network[start:end]
        parsed = parse_class(block)
        if parsed is None:
            continue
        _name, class_token, net_tokens, nested_start = parsed
        retained = [token for name, token in net_tokens if name not in restricted_rf]
        if len(retained) != len(net_tokens):
            replacements.append(
                (
                    start,
                    end,
                    render_class(block, class_token, retained, nested_start),
                )
            )

    focused_network = network
    for start, end, replacement in sorted(replacements, reverse=True):
        focused_network = (
            focused_network[:start] + replacement + focused_network[end:]
        )
    additions = "\n"
    for class_name, (nets, width) in _RF_GROUPS.items():
        additions += render_restricted_class(class_name, nets, width) + "\n"
    focused_network = focused_network[:-1] + additions + "  )"

    interim = source[:network_start] + focused_network + source[network_end:]
    wiring_start, wiring_end = find_head_block(interim, "wiring")
    wiring = interim[wiring_start:wiring_end]
    removals: list[tuple[int, int]] = []
    removed_by_kind: Counter[str] = Counter()
    removed_by_net: Counter[str] = Counter()
    for start, end in direct_child_spans(wiring):
        block = wiring[start:end]
        name = wiring_net(block)
        if name not in restricted_rf:
            continue
        head = re.match(r"^\(\s*([^\s()]+)", block)
        kind = "unknown" if head is None else head.group(1)
        removals.append((start, end))
        removed_by_kind[kind] += 1
        removed_by_net[name] += 1

    focused_wiring = wiring
    for start, end in reversed(removals):
        focused_wiring = focused_wiring[:start] + focused_wiring[end:]
    output = interim[:wiring_start] + focused_wiring + interim[wiring_end:]

    report = {
        "unconnected_targets": dict(sorted(unconnected_targets.items())),
        "target_net_count": len(targets),
        "targets": sorted(targets),
        "restricted_rf_nets": sorted(restricted_rf),
        "suppressed_non_target_nets": len(suppressed),
        "suppressed_pin_demands": dict(sorted(suppressed.items())),
        "removed_rf_wiring": len(removals),
        "removed_rf_wiring_by_kind": dict(sorted(removed_by_kind.items())),
        "removed_rf_wiring_by_net": dict(sorted(removed_by_net.items())),
        "target_pin_entries_preserved": True,
        "non_target_wiring_preserved": True,
    }
    return output, report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--drc", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()

    output, report = rewrite(
        args.input.read_text(encoding="utf-8", errors="strict"), args.drc
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(output, encoding="utf-8")
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
