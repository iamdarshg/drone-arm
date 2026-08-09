#!/usr/bin/env python3
"""Add six conventional PWM ESC headers to the compressed Main Rev B PCB."""

from __future__ import annotations

import argparse
from pathlib import Path

import pcbnew


FOOTPRINT_LIBRARY = Path(
    r"C:\Program Files\KiCad\9.0\share\kicad\footprints\Connector_PinHeader_2.54mm.pretty"
)
FOOTPRINT_NAME = "PinHeader_1x03_P2.54mm_Vertical"
POSITIONS_MM = [(78.0, 72.5), (87.0, 72.5), (96.0, 72.5),
                (105.0, 72.5), (114.0, 72.5), (123.0, 72.5)]
BOARD_RIGHT_MM = 130.0
BOARD_BOTTOM_MM = 74.52


def find_footprint(board, reference: str):
    for footprint in board.GetFootprints():
        if footprint.GetReference() == reference:
            return footprint
    return None


def get_or_create_net(board, name: str):
    net = board.FindNet(name)
    if net is None:
        net = pcbnew.NETINFO_ITEM(board, name)
        board.Add(net)
    return net


def close_compressed_outline(board) -> None:
    """Close the manually compressed outline at its new 74.52 mm bottom edge."""
    for drawing in board.GetDrawings():
        if drawing.GetLayer() != pcbnew.Edge_Cuts:
            continue
        start = drawing.GetStart()
        end = drawing.GetEnd()
        start_x = pcbnew.ToMM(start.x)
        start_y = pcbnew.ToMM(start.y)
        end_x = pcbnew.ToMM(end.x)
        end_y = pcbnew.ToMM(end.y)

        if abs(start_y - BOARD_BOTTOM_MM) < 0.01 and abs(end_y - BOARD_BOTTOM_MM) < 0.01:
            drawing.SetStart(pcbnew.VECTOR2I_MM(BOARD_RIGHT_MM, BOARD_BOTTOM_MM))
            drawing.SetEnd(pcbnew.VECTOR2I_MM(0.0, BOARD_BOTTOM_MM))
        elif abs(start_x) < 0.01 and abs(end_x) < 0.01:
            if start_y > end_y:
                drawing.SetStart(pcbnew.VECTOR2I_MM(0.0, BOARD_BOTTOM_MM))
            else:
                drawing.SetEnd(pcbnew.VECTOR2I_MM(0.0, BOARD_BOTTOM_MM))
        elif abs(start_x - BOARD_RIGHT_MM) < 0.01 and abs(end_x - BOARD_RIGHT_MM) < 0.01:
            if start_y > end_y:
                drawing.SetStart(pcbnew.VECTOR2I_MM(BOARD_RIGHT_MM, BOARD_BOTTOM_MM))
            else:
                drawing.SetEnd(pcbnew.VECTOR2I_MM(BOARD_RIGHT_MM, BOARD_BOTTOM_MM))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()

    board = pcbnew.LoadBoard(str(args.input))
    close_compressed_outline(board)
    u10 = find_footprint(board, "U10")
    if u10 is None:
        raise RuntimeError("U10 was not found on the board")

    can_gnd = get_or_create_net(board, "/CAN_GND")
    can_5v = get_or_create_net(board, "/CAN_5V")
    pwm_nets = [
        get_or_create_net(board, f"/RP2354B MCU/PWM_ESC{index}")
        for index in range(1, 7)
    ]

    u10_pads = {pad.GetNumber(): pad for pad in u10.Pads()}
    for index, net in enumerate(pwm_nets, start=43):
        pad = u10_pads.get(str(index))
        if pad is None:
            raise RuntimeError(f"U10 pad {index} was not found")
        pad.SetNet(net)

    for index, (x_mm, y_mm) in enumerate(POSITIONS_MM, start=1):
        reference = f"J{59 + index}"
        footprint = find_footprint(board, reference)
        if footprint is None:
            footprint = pcbnew.FootprintLoad(str(FOOTPRINT_LIBRARY), FOOTPRINT_NAME)
            if footprint is None:
                raise RuntimeError(f"Could not load {FOOTPRINT_NAME}")
            footprint.SetReference(reference)
            footprint.SetValue(f"ESC_PWM_{index}")
            footprint.SetPosition(pcbnew.VECTOR2I_MM(x_mm, y_mm))
            footprint.SetOrientationDegrees(90.0)
            board.Add(footprint)

        reference_field = footprint.Reference()
        reference_field.SetPosition(pcbnew.VECTOR2I_MM(x_mm + 2.54, 69.8))
        reference_field.SetTextAngleDegrees(0.0)

        pads = {pad.GetNumber(): pad for pad in footprint.Pads()}
        pads["1"].SetNet(can_gnd)
        pads["2"].SetNet(can_5v)
        pads["3"].SetNet(pwm_nets[index - 1])

    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    pcbnew.SaveBoard(str(args.output), board)


if __name__ == "__main__":
    main()
