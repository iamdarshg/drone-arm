#!/usr/bin/env python3
"""Export/import Rev-B PCB routing through the Specctra DSN/SES interface."""

from __future__ import annotations

import argparse
from pathlib import Path

import pcbnew


def board_counts(board: pcbnew.BOARD) -> tuple[int, int]:
    tracks = 0
    vias = 0
    for item in board.GetTracks():
        if isinstance(item, pcbnew.PCB_VIA):
            vias += 1
        else:
            tracks += 1
    return tracks, vias


def export_board(board_path: Path, dsn_path: Path) -> None:
    board = pcbnew.LoadBoard(str(board_path))
    dsn_path.parent.mkdir(parents=True, exist_ok=True)
    if not pcbnew.ExportSpecctraDSN(board, str(dsn_path)):
        raise RuntimeError(f"Specctra export failed for {board_path}")
    tracks, vias = board_counts(board)
    print(f"exported {board_path}: {tracks} tracks, {vias} vias -> {dsn_path}")


def import_board(board_path: Path, ses_path: Path) -> None:
    board = pcbnew.LoadBoard(str(board_path))
    before_tracks, before_vias = board_counts(board)
    if not pcbnew.ImportSpecctraSES(board, str(ses_path)):
        raise RuntimeError(f"Specctra session import failed for {ses_path}")

    # Refill all copper zones after the imported routing is applied.
    try:
        filler = pcbnew.ZONE_FILLER(board)
        filler.Fill(board.Zones())
    except Exception as exc:  # KiCad versions expose slightly different APIs.
        print(f"zone refill warning: {exc}")

    pcbnew.SaveBoard(str(board_path), board)
    after_tracks, after_vias = board_counts(board)
    print(
        f"imported {ses_path} -> {board_path}: "
        f"tracks {before_tracks}->{after_tracks}, vias {before_vias}->{after_vias}"
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)

    export_parser = subparsers.add_parser("export")
    export_parser.add_argument("board", type=Path)
    export_parser.add_argument("dsn", type=Path)

    import_parser = subparsers.add_parser("import")
    import_parser.add_argument("board", type=Path)
    import_parser.add_argument("ses", type=Path)

    args = parser.parse_args()
    if args.command == "export":
        export_board(args.board, args.dsn)
    else:
        import_board(args.board, args.ses)


if __name__ == "__main__":
    main()
