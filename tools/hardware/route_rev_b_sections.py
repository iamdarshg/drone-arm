#!/usr/bin/env python3
"""Low-memory Specctra routing helpers for the Rev-B boards.

Run with KiCad's bundled Python.  The ESC contains six geometrically identical
motor cells.  Routing one isolated cell and translating its new tracks/vias to
the other five cells keeps FreeRouting below the deliberately small Java heap
used on the shared workstation.
"""

from __future__ import annotations

import argparse
import json
import os
import re
from pathlib import Path

import pcbnew


MM = pcbnew.FromMM


def net_name(item: pcbnew.BOARD_CONNECTED_ITEM) -> str:
    return str(item.GetNetname())


def item_fingerprint(item: pcbnew.BOARD_ITEM) -> tuple:
    if isinstance(item, pcbnew.PCB_VIA):
        pos = item.GetPosition()
        return (
            "via",
            net_name(item),
            pos.x,
            pos.y,
            item.GetWidth(item.GetLayer()),
            item.GetDrillValue(),
            int(item.GetLayer()),
            int(item.GetViaType()),
        )
    if isinstance(item, pcbnew.PCB_TRACK):
        start = item.GetStart()
        end = item.GetEnd()
        return (
            "track",
            net_name(item),
            start.x,
            start.y,
            end.x,
            end.y,
            item.GetWidth(),
            int(item.GetLayer()),
        )
    raise TypeError(f"Unsupported routed item: {type(item).__name__}")


def add_outline(board: pcbnew.BOARD) -> None:
    corners = (
        (1.0, 41.0),
        (160.0, 41.0),
        (160.0, 142.0),
        (1.0, 142.0),
    )
    for start, end in zip(corners, corners[1:] + corners[:1]):
        segment = pcbnew.PCB_SHAPE(board)
        segment.SetShape(pcbnew.SHAPE_T_SEGMENT)
        segment.SetLayer(pcbnew.Edge_Cuts)
        segment.SetStart(pcbnew.VECTOR2I(MM(start[0]), MM(start[1])))
        segment.SetEnd(pcbnew.VECTOR2I(MM(end[0]), MM(end[1])))
        segment.SetWidth(MM(0.05))
        board.Add(segment)


def is_motor_1_ref(reference: str) -> bool:
    if reference == "TH1":
        return True
    match = re.search(r"(\d+)$", reference)
    return bool(match and 1100 <= int(match.group(1)) < 1200)


def extract_cell(full_path: Path, cell_path: Path, baseline_path: Path, dsn_path: Path) -> None:
    board = pcbnew.LoadBoard(str(full_path))
    zones = [board.GetArea(index) for index in range(board.GetAreaCount())]
    for zone in zones:
        if "M1_" not in str(zone.GetNetname()):
            board.Delete(zone)
    for footprint in list(board.GetFootprints()):
        if not is_motor_1_ref(footprint.GetReference()):
            board.Delete(footprint)
    for item in list(board.GetTracks()):
        if "M1_" not in net_name(item):
            board.Delete(item)
    for drawing in list(board.GetDrawings()):
        board.Delete(drawing)
    add_outline(board)

    baseline = [item_fingerprint(item) for item in board.GetTracks()]
    baseline_path.write_text(json.dumps(baseline, indent=2) + "\n", encoding="utf-8")
    pcbnew.SaveBoard(str(cell_path), board)
    if not pcbnew.ExportSpecctraDSN(board, str(dsn_path)):
        raise RuntimeError(f"Failed to export {dsn_path}")
    print(
        json.dumps(
            {
                "cell": str(cell_path),
                "dsn": str(dsn_path),
                "footprints": len(list(board.GetFootprints())),
                "baseline_routed_items": len(baseline),
            },
            indent=2,
        )
    )


def import_cell(cell_path: Path, ses_path: Path, routed_path: Path) -> None:
    board = pcbnew.LoadBoard(str(cell_path))
    if not pcbnew.ImportSpecctraSES(board, str(ses_path)):
        raise RuntimeError(f"Failed to import {ses_path}")
    pcbnew.SaveBoard(str(routed_path), board)
    print(f"Imported {ses_path} into {routed_path}")


def mapped_net_name(source: str, motor: int) -> str:
    if motor == 1:
        return source
    return source.replace("Motor 1 power cell", f"Motor {motor} power cell").replace(
        "M1_", f"M{motor}_"
    )


def find_net(board: pcbnew.BOARD, name: str) -> pcbnew.NETINFO_ITEM:
    for key, value in board.GetNetsByName().items():
        if str(key) == name:
            return value
    raise KeyError(f"Target board does not contain net {name!r}")


def translated(point: pcbnew.VECTOR2I, dx_mm: float, dy_mm: float) -> pcbnew.VECTOR2I:
    return pcbnew.VECTOR2I(point.x + MM(dx_mm), point.y + MM(dy_mm))


def clone_item(
    source: pcbnew.BOARD_ITEM,
    target: pcbnew.BOARD,
    motor: int,
    dx_mm: float,
    dy_mm: float,
) -> pcbnew.BOARD_ITEM:
    target_net = find_net(target, mapped_net_name(net_name(source), motor))
    if isinstance(source, pcbnew.PCB_VIA):
        clone = pcbnew.PCB_VIA(target)
        clone.SetPosition(translated(source.GetPosition(), dx_mm, dy_mm))
        clone.SetWidth(source.GetWidth(source.GetLayer()))
        clone.SetDrill(source.GetDrillValue())
        clone.SetViaType(source.GetViaType())
        clone.SetLayerSet(source.GetLayerSet())
        clone.SetNet(target_net)
        return clone
    if isinstance(source, pcbnew.PCB_TRACK):
        clone = pcbnew.PCB_TRACK(target)
        clone.SetStart(translated(source.GetStart(), dx_mm, dy_mm))
        clone.SetEnd(translated(source.GetEnd(), dx_mm, dy_mm))
        clone.SetWidth(source.GetWidth())
        clone.SetLayer(source.GetLayer())
        clone.SetNet(target_net)
        return clone
    raise TypeError(f"Unsupported routed item: {type(source).__name__}")


def replicate_cell(
    full_path: Path,
    routed_cell_path: Path,
    baseline_path: Path,
    output_path: Path,
) -> None:
    full = pcbnew.LoadBoard(str(full_path))
    cell = pcbnew.LoadBoard(str(routed_cell_path))
    baseline = {tuple(entry) for entry in json.loads(baseline_path.read_text(encoding="utf-8"))}
    additions = [
        item
        for item in cell.GetTracks()
        if item_fingerprint(item) not in baseline
    ]
    offsets = {
        1: (0.0, 0.0),
        2: (157.0, 0.0),
        3: (314.0, 0.0),
        4: (0.0, 100.0),
        5: (157.0, 100.0),
        6: (314.0, 100.0),
    }
    for motor, (dx_mm, dy_mm) in offsets.items():
        for source in additions:
            full.Add(clone_item(source, full, motor, dx_mm, dy_mm))
    pcbnew.SaveBoard(str(output_path), full)
    print(
        json.dumps(
            {
                "source_items": len(additions),
                "replicated_items": len(additions) * 6,
                "output": str(output_path),
            },
            indent=2,
        )
    )


def import_main(board_path: Path, ses_path: Path, output_path: Path) -> None:
    board = pcbnew.LoadBoard(str(board_path))
    if not pcbnew.ImportSpecctraSES(board, str(ses_path)):
        raise RuntimeError(f"Failed to import {ses_path}")
    pcbnew.SaveBoard(str(output_path), board)
    print(f"Imported {ses_path} into {output_path}")


def export_main(board_path: Path, dsn_path: Path) -> None:
    board = pcbnew.LoadBoard(str(board_path))
    if not pcbnew.ExportSpecctraDSN(board, str(dsn_path)):
        raise RuntimeError(f"Failed to export {dsn_path}")
    print(
        json.dumps(
            {
                "board": str(board_path),
                "dsn": str(dsn_path),
                "footprints": len(list(board.GetFootprints())),
                "routed_items": len(list(board.GetTracks())),
            },
            indent=2,
        )
    )


def fill_zones(board_path: Path, output_path: Path) -> None:
    """Refill every copper zone after an external router session import."""
    board = pcbnew.LoadBoard(str(board_path))
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    pcbnew.SaveBoard(str(output_path), board)
    print(f"Filled zones in {board_path} and saved {output_path}")


def strip_zones(board_path: Path, output_path: Path) -> None:
    """Remove zones, save, then bypass the unstable pcbnew teardown path."""
    board = pcbnew.LoadBoard(str(board_path))
    removed_zones = len(list(board.Zones()))
    for zone in list(board.Zones()):
        board.Remove(zone)
    pcbnew.SaveBoard(str(output_path), board)
    print(
        json.dumps(
            {"removed_zones": removed_zones, "output": str(output_path)},
            indent=2,
        ),
        flush=True,
    )
    os._exit(0)


def strip_tracks_and_export(
    board_path: Path,
    output_path: Path,
    dsn_path: Path,
) -> None:
    """Remove routed copper from a zone-less copy and export the router DSN."""
    board = pcbnew.LoadBoard(str(board_path))
    removed_tracks = len(list(board.GetTracks()))
    for track in list(board.GetTracks()):
        board.Remove(track)
    pcbnew.SaveBoard(str(output_path), board)
    if not pcbnew.ExportSpecctraDSN(board, str(dsn_path)):
        raise RuntimeError(f"Failed to export {dsn_path}")
    print(
        json.dumps(
            {
                "source": str(board_path),
                "output": str(output_path),
                "dsn": str(dsn_path),
                "removed_tracks": removed_tracks,
            },
            indent=2,
        )
    )


def clean_main_router_import(board_path: Path, output_path: Path) -> None:
    """Remove exact router duplicates and repair known mechanical DRC artifacts."""
    board = pcbnew.LoadBoard(str(board_path))
    seen: set[tuple] = set()
    duplicates = []
    cc2_adjustments = 0
    for item in list(board.GetTracks()):
        start = item.GetStart()
        end = item.GetEnd()
        if isinstance(item, pcbnew.PCB_VIA):
            key = (
                "via",
                item.GetNetCode(),
                start.x,
                start.y,
                item.GetLayer(),
                item.GetWidth(),
                item.GetDrillValue(),
            )
        else:
            endpoints = sorted(((start.x, start.y), (end.x, end.y)))
            key = (
                "track",
                item.GetNetCode(),
                item.GetLayer(),
                endpoints[0],
                endpoints[1],
                item.GetWidth(),
            )
        if key in seen:
            duplicates.append(item)
            continue
        seen.add(key)

        if item.GetNetname() == "/Power and USB/USB_CC2":
            changed = False
            if pcbnew.ToMM(start.x) < 0.75:
                start.x = MM(0.75)
                changed = True
            if pcbnew.ToMM(end.x) < 0.75:
                end.x = MM(0.75)
                changed = True
            if changed:
                item.SetStart(start)
                item.SetEnd(end)
                cc2_adjustments += 1

    for item in duplicates:
        board.Remove(item)

    silk_adjustments = 0
    for item in board.GetDrawings():
        if item.GetLayer() != pcbnew.F_SilkS:
            continue
        if isinstance(item, pcbnew.PCB_TEXT) and item.GetText() == "GNSS 50R / 3mm COURTYARD":
            position = item.GetPosition()
            position.y = MM(34.5)
            item.SetPosition(position)
            silk_adjustments += 1
        elif isinstance(item, pcbnew.PCB_SHAPE):
            start = item.GetStart()
            end = item.GetEnd()
            changed = False
            for point in (start, end):
                if pcbnew.ToMM(point.x) >= 139.9:
                    point.x = MM(139.0)
                    changed = True
                if pcbnew.ToMM(point.y) >= 99.9:
                    point.y = MM(99.0)
                    changed = True
            if changed:
                item.SetStart(start)
                item.SetEnd(end)
                silk_adjustments += 1

    pcbnew.SaveBoard(str(output_path), board)
    print(
        json.dumps(
            {
                "removed_duplicate_copper_items": len(duplicates),
                "cc2_adjustments": cc2_adjustments,
                "silk_adjustments": silk_adjustments,
                "output": str(output_path),
            },
            indent=2,
        )
    )


def fix_ina296_reserved_pins(board_path: Path, output_path: Path) -> None:
    board = pcbnew.LoadBoard(str(board_path))
    fixed = []
    for motor in range(1, 7):
        reference = f"U{1000 + motor * 100 + 5}"
        footprint = board.FindFootprintByReference(reference)
        if footprint is None:
            raise KeyError(f"Missing {reference}")
        pad = footprint.FindPadByNumber("4")
        if pad is None:
            raise KeyError(f"Missing {reference}.4")
        target_net = find_net(board, f"/M{motor}_BATN")
        pad.SetNet(target_net)
        fixed.append(f"{reference}.4")
    pcbnew.SaveBoard(str(output_path), board)
    print(json.dumps({"fixed": fixed, "output": str(output_path)}, indent=2))


def main() -> None:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)

    extract = subparsers.add_parser("extract-cell")
    extract.add_argument("--full", type=Path, required=True)
    extract.add_argument("--cell", type=Path, required=True)
    extract.add_argument("--baseline", type=Path, required=True)
    extract.add_argument("--dsn", type=Path, required=True)

    import_cell_parser = subparsers.add_parser("import-cell")
    import_cell_parser.add_argument("--cell", type=Path, required=True)
    import_cell_parser.add_argument("--ses", type=Path, required=True)
    import_cell_parser.add_argument("--output", type=Path, required=True)

    replicate = subparsers.add_parser("replicate-cell")
    replicate.add_argument("--full", type=Path, required=True)
    replicate.add_argument("--cell", type=Path, required=True)
    replicate.add_argument("--baseline", type=Path, required=True)
    replicate.add_argument("--output", type=Path, required=True)

    import_main_parser = subparsers.add_parser("import-main")
    import_main_parser.add_argument("--board", type=Path, required=True)
    import_main_parser.add_argument("--ses", type=Path, required=True)
    import_main_parser.add_argument("--output", type=Path, required=True)

    export_main_parser = subparsers.add_parser("export-main")
    export_main_parser.add_argument("--board", type=Path, required=True)
    export_main_parser.add_argument("--dsn", type=Path, required=True)

    fill_zones_parser = subparsers.add_parser("fill-zones")
    fill_zones_parser.add_argument("--board", type=Path, required=True)
    fill_zones_parser.add_argument("--output", type=Path, required=True)

    strip_zone_parser = subparsers.add_parser("strip-zones")
    strip_zone_parser.add_argument("--board", type=Path, required=True)
    strip_zone_parser.add_argument("--output", type=Path, required=True)

    strip_tracks_parser = subparsers.add_parser("strip-tracks-export")
    strip_tracks_parser.add_argument("--board", type=Path, required=True)
    strip_tracks_parser.add_argument("--output", type=Path, required=True)
    strip_tracks_parser.add_argument("--dsn", type=Path, required=True)

    clean_main_parser = subparsers.add_parser("clean-main-import")
    clean_main_parser.add_argument("--board", type=Path, required=True)
    clean_main_parser.add_argument("--output", type=Path, required=True)

    ina_fix = subparsers.add_parser("fix-ina296-reserved")
    ina_fix.add_argument("--board", type=Path, required=True)
    ina_fix.add_argument("--output", type=Path, required=True)

    args = parser.parse_args()
    if args.command == "extract-cell":
        extract_cell(args.full, args.cell, args.baseline, args.dsn)
    elif args.command == "import-cell":
        import_cell(args.cell, args.ses, args.output)
    elif args.command == "replicate-cell":
        replicate_cell(args.full, args.cell, args.baseline, args.output)
    elif args.command == "import-main":
        import_main(args.board, args.ses, args.output)
    elif args.command == "export-main":
        export_main(args.board, args.dsn)
    elif args.command == "fill-zones":
        fill_zones(args.board, args.output)
    elif args.command == "strip-zones":
        strip_zones(args.board, args.output)
    elif args.command == "strip-tracks-export":
        strip_tracks_and_export(args.board, args.output, args.dsn)
    elif args.command == "clean-main-import":
        clean_main_router_import(args.board, args.output)
    elif args.command == "fix-ina296-reserved":
        fix_ina296_reserved_pins(args.board, args.output)


if __name__ == "__main__":
    main()
