#!/usr/bin/env python3
"""Add generated ESC USB footprints outside Edge.Cuts without moving existing work."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pcbnew


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_NETLIST = ROOT / "hardware/esc/rev_b/reports/esc_usb_netlist.xml"
FOOTPRINT_ROOT = Path(sys.executable).resolve().parents[1] / "share/kicad/footprints"


def component_contract(path: Path):
    root = ET.parse(path).getroot()
    components = {}
    for comp in root.findall("./components/comp"):
        ref = comp.attrib["ref"]
        if ref.startswith("#"):
            continue
        components[ref] = (comp.findtext("footprint", ""), comp.findtext("value", ""))
    pin_nets = {}
    for net in root.findall("./nets/net"):
        name = net.attrib["name"]
        for node in net.findall("node"):
            pin_nets[(node.attrib["ref"], node.attrib["pin"])] = name
    return components, pin_nets


def snapshot(footprint):
    pos = footprint.GetPosition()
    pads = tuple(
        sorted(
            (
                pad.GetNumber(), pad.GetPosition().x, pad.GetPosition().y,
                pad.GetSize().x, pad.GetSize().y, int(pad.GetShape()), pad.GetLayerSet().FmtHex(),
            )
            for pad in footprint.Pads()
        )
    )
    return (pos.x, pos.y, footprint.GetOrientation().AsTenthsOfADegree(), footprint.GetLayer(), pads)


def load_footprint(identifier: str):
    if ":" not in identifier:
        raise ValueError(f"footprint has no library prefix: {identifier!r}")
    library, name = identifier.split(":", 1)
    library_path = FOOTPRINT_ROOT / f"{library}.pretty"
    footprint = pcbnew.FootprintLoad(str(library_path), name)
    if footprint is None:
        raise FileNotFoundError(f"cannot load {identifier} from {library_path}")
    return footprint


def get_or_add_net(board, name: str):
    net = board.FindNet(name)
    if net is None:
        net = pcbnew.NETINFO_ITEM(board, name)
        board.Add(net)
    return net


def reconcile(source: Path, output: Path, netlist: Path) -> dict[str, int]:
    board = pcbnew.LoadBoard(str(source))
    if board is None:
        raise RuntimeError(f"cannot load {source}")
    components, pin_nets = component_contract(netlist)
    existing = {fp.GetReference(): fp for fp in board.GetFootprints()}
    before = {ref: snapshot(fp) for ref, fp in existing.items()}

    stale = sorted(set(existing) - set(components))
    removable_service_headers = {f"J{1000 + cell * 100 + 4}" for cell in range(1, 7)}
    unexpected_stale = sorted(set(stale) - removable_service_headers)
    if unexpected_stale:
        raise RuntimeError(f"refusing to remove non-contract footprints: {unexpected_stale}")
    for ref in stale:
        board.Remove(existing[ref])
        before.pop(ref)

    edge_box = board.GetBoardEdgesBoundingBox()
    edge_right = pcbnew.ToMM(edge_box.GetRight())
    edge_top = pcbnew.ToMM(edge_box.GetTop())
    missing = sorted(set(components) - set(existing))
    for index, ref in enumerate(missing):
        footprint_id, value = components[ref]
        fp = load_footprint(footprint_id)
        fp.SetReference(ref)
        fp.SetValue(value)
        column, row = divmod(index, 12)
        fp.SetPosition(pcbnew.VECTOR2I_MM(edge_right + 20 + column * 32, edge_top + 8 + row * 12))
        board.Add(fp)
        for pad in fp.Pads():
            name = pin_nets.get((ref, pad.GetNumber()))
            if name is not None:
                pad.SetNet(get_or_add_net(board, name))

    output.parent.mkdir(parents=True, exist_ok=True)
    pcbnew.SaveBoard(str(output), board)
    check = pcbnew.LoadBoard(str(output))
    if check is None:
        raise RuntimeError(f"cannot reload {output}")
    after = {fp.GetReference(): fp for fp in check.GetFootprints()}
    changed = sum(snapshot(after[ref]) != snap for ref, snap in before.items())
    missing_after = set(components) - set(after)
    stale_after = set(after) - set(components)
    inside = 0
    for ref in missing:
        if pcbnew.ToMM(after[ref].GetBoundingBox().GetLeft()) <= edge_right:
            inside += 1
    return {
        "source_footprints": len(existing),
        "added_footprints": len(missing),
        "removed_redundant_service_headers": len(stale),
        "output_footprints": len(after),
        "existing_transforms_changed": changed,
        "missing_references": len(missing_after),
        "stale_references": len(stale_after),
        "new_inside_edge_cuts": inside,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--netlist", type=Path, default=DEFAULT_NETLIST)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    report = reconcile(args.source, args.output, args.netlist)
    for key, value in report.items():
        print(f"{key}={value}")
    failed = any(report[key] for key in (
        "existing_transforms_changed", "missing_references", "stale_references", "new_inside_edge_cuts"
    ))
    return 1 if args.check and failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
