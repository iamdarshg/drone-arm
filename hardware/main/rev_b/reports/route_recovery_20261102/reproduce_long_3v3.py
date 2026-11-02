#!/usr/bin/env python3
"""Apply the accepted long /3V3 escape to a board copy using KiCad pcbnew."""
import argparse
import pcbnew


def footprint_pad_map(board):
    result = []
    for fp in board.GetFootprints():
        result.append(("fp", fp.GetReference(), fp.GetValue(),
                       fp.GetPosition().x, fp.GetPosition().y,
                       round(fp.GetOrientationDegrees(), 6), fp.GetFPID().GetLibItemName()))
        for pad in fp.Pads():
            result.append(("pad", fp.GetReference(), pad.GetNumber(), pad.GetNetname(),
                           pad.GetPosition().x, pad.GetPosition().y,
                           pad.GetSize().x, pad.GetSize().y))
    return tuple(sorted(result))


def mm_point(x, y):
    return pcbnew.VECTOR2I(pcbnew.FromMM(x), pcbnew.FromMM(y))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input_board")
    parser.add_argument("output_board")
    args = parser.parse_args()

    board = pcbnew.LoadBoard(args.input_board)
    before = footprint_pad_map(board)
    net_code = board.FindNet("/3V3").GetNetCode()

    # Break out from the existing R121-side F.Cu route onto B.Cu with a
    # standard through-via, then join the existing /3V3 via at the far end.
    via = pcbnew.PCB_VIA(board)
    via.SetPosition(mm_point(27.273, 24.598))
    via.SetDrill(pcbnew.FromMM(0.35))
    via.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
    for layer in (pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu):
        via.SetWidth(layer, pcbnew.FromMM(0.7))
    via.SetNetCode(net_code)
    board.Add(via)

    track = pcbnew.PCB_TRACK(board)
    track.SetStart(mm_point(27.273, 24.598))
    track.SetEnd(mm_point(49.1875, 58.589))
    track.SetWidth(pcbnew.FromMM(0.25))
    track.SetLayer(pcbnew.B_Cu)
    track.SetNetCode(net_code)
    board.Add(track)

    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    pcbnew.SaveBoard(args.output_board, board)
    saved = pcbnew.LoadBoard(args.output_board)
    assert footprint_pad_map(saved) == before, "route altered footprint or pad mappings"
    print("PASS: route added and all footprint/pad mappings are unchanged")


if __name__ == "__main__":
    main()
