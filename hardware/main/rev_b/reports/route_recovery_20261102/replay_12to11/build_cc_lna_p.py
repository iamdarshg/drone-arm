#!/usr/bin/env python3
"""Close CC_LNA_P on the pinned 12-open board without moving footprints."""
import pathlib
import sys
import pcbnew

source, output = map(pathlib.Path, sys.argv[1:3])
board = pcbnew.LoadBoard(str(source))
mm, to = pcbnew.FromMM, pcbnew.ToMM

def point(x, y):
    return pcbnew.VECTOR2I(mm(x), mm(y))

codes = {name: board.FindNet(name).GetNetCode() for name in (
    "/915 MHz RF/CC_LNA_P", "/915 MHz RF/CC_LNA_N", "/915 MHz RF/CC_TRX_SW",
)}
old, new = (97.3142, 61.2381), (99.3, 61.0)
moved = 0
for item in list(board.GetTracks()):
    if isinstance(item, pcbnew.PCB_VIA) and item.GetNetCode() == codes["/915 MHz RF/CC_LNA_N"]:
        p = item.GetPosition()
        if abs(to(p.x) - old[0]) < 0.002 and abs(to(p.y) - old[1]) < 0.002:
            item.SetPosition(point(*new)); moved += 1
    elif isinstance(item, pcbnew.PCB_TRACK) and item.GetNetCode() == codes["/915 MHz RF/CC_LNA_N"]:
        for getter, setter in ((item.GetStart, item.SetStart), (item.GetEnd, item.SetEnd)):
            p = getter()
            if abs(to(p.x) - old[0]) < 0.002 and abs(to(p.y) - old[1]) < 0.002:
                setter(point(*new)); moved += 1
assert moved == 3, f"expected via and two /CC_LNA_N endpoints at source coordinate; got {moved}"
removed = []
for item in list(board.GetTracks()):
    if (isinstance(item, pcbnew.PCB_TRACK) and item.GetNetCode() == codes["/915 MHz RF/CC_TRX_SW"]
            and item.GetLayer() == pcbnew.F_Cu):
        ends = [(round(to(p.x), 4), round(to(p.y), 4)) for p in (item.GetStart(), item.GetEnd())]
        if all(96.4 <= x <= 101.4 and 62.0 <= y <= 62.3 for x, y in ends):
            removed.append(item)
for item in removed: board.Remove(item)
assert len(removed) == 4, f"expected four local /CC_TRX_SW segments; got {len(removed)}"

def add_track(net, start, end, width):
    track = pcbnew.PCB_TRACK(board)
    track.SetStart(point(*start)); track.SetEnd(point(*end)); track.SetLayer(pcbnew.F_Cu)
    track.SetWidth(mm(width)); track.SetNetCode(codes[net]); board.Add(track)

for a, b in zip([(96.4375, 62.25), (96.7, 62.175), (101.24, 62.175)],
                 [(96.7, 62.175), (101.24, 62.175), (101.375, 62.25)]):
    add_track("/915 MHz RF/CC_TRX_SW", a, b, 0.25)
add_track("/915 MHz RF/CC_LNA_P", (96.4375, 61.75), (101.375, 61.75), 0.35)
pcbnew.SaveBoard(str(output), board)
print(f"saved {output}")
