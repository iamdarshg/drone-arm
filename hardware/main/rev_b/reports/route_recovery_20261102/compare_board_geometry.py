#!/usr/bin/env python3
"""Compare physical PCB geometry while ignoring generated KiCad UUIDs/order."""
import sys
import pcbnew


def mm(value):
    return round(pcbnew.ToMM(value), 4)


def point(value):
    return (mm(value.x), mm(value.y))


def footprint_signature(board):
    result = []
    for fp in board.GetFootprints():
        pads = tuple(sorted((p.GetNumber(), p.GetNetname(), point(p.GetPosition()),
                             (mm(p.GetSize().x), mm(p.GetSize().y))) for p in fp.Pads()))
        result.append((fp.GetReference(), fp.GetValue(), fp.GetFPID().GetLibItemName(),
                       round(fp.GetOrientationDegrees(), 4), point(fp.GetPosition()), pads))
    return tuple(sorted(result))


def copper_signature(board):
    result = []
    for item in board.GetTracks():
        net = item.GetNetname()
        if isinstance(item, pcbnew.PCB_VIA):
            widths = tuple((int(layer), mm(item.GetWidth(layer))) for layer in
                           (pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu))
            result.append(("via", net, point(item.GetPosition()), mm(item.GetDrill()),
                           int(item.TopLayer()), int(item.BottomLayer()), int(item.GetViaType()), widths))
        else:
            ends = tuple(sorted((point(item.GetStart()), point(item.GetEnd()))))
            result.append(("track", net, int(item.GetLayer()), mm(item.GetWidth()), ends))
    return tuple(sorted(result))


def main():
    if len(sys.argv) != 3:
        raise SystemExit("usage: compare_board_geometry.py candidate.kicad_pcb replay.kicad_pcb")
    a = pcbnew.LoadBoard(sys.argv[1])
    b = pcbnew.LoadBoard(sys.argv[2])
    assert footprint_signature(a) == footprint_signature(b), "footprints/pads differ"
    assert copper_signature(a) == copper_signature(b), "copper geometry differs"
    print("PASS: candidate and replay have matching footprint, pad and copper geometry")


if __name__ == "__main__":
    main()
