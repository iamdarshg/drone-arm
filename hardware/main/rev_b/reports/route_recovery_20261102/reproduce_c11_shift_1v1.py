#!/usr/bin/env python3
"""Replay the C11-shift plus /1V1 In2 route on a pinned 16-open board."""
import argparse
import hashlib
import pathlib
import pcbnew

SOURCE_SHA = "60da183ad175be4e1db9daa57a705714a84dd0d7b1f0b2a942f6beeee3a9a191"
PROJECT_SHA = "f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e"


def mm(value):
    return pcbnew.FromMM(value)


def xy(point):
    return pcbnew.VECTOR2I(mm(point[0]), mm(point[1]))


def pad_map(board):
    return tuple(sorted(
        (fp.GetReference(), fp.GetValue(), fp.GetFPID().GetLibItemName(),
         round(fp.GetOrientationDegrees(), 6),
         tuple(sorted((pad.GetNumber(), pad.GetNetname()) for pad in fp.Pads())) )
        for fp in board.GetFootprints()
    ))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input_board", help="pinned 16-open board")
    parser.add_argument("output_board")
    args = parser.parse_args()
    source = pathlib.Path(args.input_board).resolve()
    project = source.with_name("main_rev_b.kicad_pro")
    assert hashlib.sha256(source.read_bytes()).hexdigest() == SOURCE_SHA, "input board hash differs"
    assert hashlib.sha256(project.read_bytes()).hexdigest() == PROJECT_SHA, "project hash differs"

    board = pcbnew.LoadBoard(str(source))
    before = pad_map(board)
    footprints = {fp.GetReference(): fp for fp in board.GetFootprints()}
    assert len(footprints) == 165
    c11 = footprints["C11"]
    c11.SetPosition(xy((52.0, 46.8)))

    moved_1v1 = moved_gnd = 0
    for item in board.GetTracks():
        if not isinstance(item, pcbnew.PCB_TRACK):
            continue
        start = (round(pcbnew.ToMM(item.GetStart().x), 4), round(pcbnew.ToMM(item.GetStart().y), 4))
        end = (round(pcbnew.ToMM(item.GetEnd().x), 4), round(pcbnew.ToMM(item.GetEnd().y), 4))
        if item.GetNetname() == "/1V1" and {start, end} == {(50.49, 45.97), (51.52, 47.0)}:
            item.SetEnd(xy((51.52, 46.8)))
            moved_1v1 += 1
        if item.GetNetname() == "/GND" and {start, end} == {(52.3078, 46.8278), (52.48, 47.0)}:
            item.SetEnd(xy((52.48, 46.8)))
            moved_gnd += 1
    assert moved_1v1 == moved_gnd == 1, (moved_1v1, moved_gnd)

    net = board.FindNet("/1V1").GetNetCode()
    via = pcbnew.PCB_VIA(board)
    via.SetPosition(xy((50.49, 41.5)))
    via.SetDrill(mm(0.35))
    via.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
    for layer in (pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu):
        via.SetWidth(layer, mm(0.7))
    via.SetNetCode(net)
    board.Add(via)

    path = ((50.49, 41.5), (50.0, 41.5), (50.0, 49.5), (52.5, 49.5), (52.9125, 49.6018))
    for start, end in zip(path, path[1:]):
        track = pcbnew.PCB_TRACK(board)
        track.SetStart(xy(start))
        track.SetEnd(xy(end))
        track.SetLayer(pcbnew.In2_Cu)
        track.SetWidth(mm(0.25))
        track.SetNetCode(net)
        board.Add(track)

    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    output = pathlib.Path(args.output_board)
    output.parent.mkdir(parents=True, exist_ok=True)
    pcbnew.SaveBoard(str(output), board)
    saved = pcbnew.LoadBoard(str(output))
    assert pad_map(saved) == before, "footprint/pad net mappings changed"
    assert len(saved.GetFootprints()) == 165
    print("PASS: replayed C11 0.2 mm move and /1V1 route; all 165 footprint/pad net mappings preserved")


if __name__ == "__main__":
    main()
