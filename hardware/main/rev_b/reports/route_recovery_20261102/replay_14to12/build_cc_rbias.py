#!/usr/bin/env python3
"""Apply the DRC-clean /CC_RBIAS and VDD_CC112 microvia route."""
import hashlib
import pathlib
import sys
import pcbnew

source, output = map(pathlib.Path, sys.argv[1:3])
expected = "63b00e91cc1ea46e7e539b383c0deb6c7ac4bdfa833dd14cdd59936aef90e0ef"
actual = hashlib.sha256(source.read_bytes()).hexdigest()
if actual != expected:
    raise SystemExit(f"unexpected 14-open source PCB: {actual}")
board = pcbnew.LoadBoard(str(source))
frommm, tomm = pcbnew.FromMM, pcbnew.ToMM
xy = lambda x, y: pcbnew.VECTOR2I(frommm(x), frommm(y))
point = lambda p: (round(tomm(p.x), 4), round(tomm(p.y), 4))
vdd = board.FindNet("/915 MHz RF/VDD_CC112").GetNetCode()
bias = board.FindNet("/915 MHz RF/CC_RBIAS").GetNetCode()
remove = {
    frozenset(((94.7984, 64.3811), (93.7876, 64.3811))),
    frozenset(((94.7984, 64.3811), (95.25, 63.9295))),
}
removed = set()
for item in list(board.GetTracks()):
    if (isinstance(item, pcbnew.PCB_TRACK) and item.GetLayer() == pcbnew.F_Cu
            and item.GetNetCode() == vdd):
        endpoints = frozenset((point(item.GetStart()), point(item.GetEnd())))
        if endpoints in remove:
            removed.add(endpoints)
            board.Remove(item)
if removed != remove:
    raise SystemExit(f"expected exactly two VDD bridge tracks, removed {removed}")

def add_track(net, layer, start, end, width):
    item = pcbnew.PCB_TRACK(board)
    item.SetStart(xy(*start))
    item.SetEnd(xy(*end))
    item.SetLayer(layer)
    item.SetWidth(frommm(width))
    item.SetNetCode(net)
    board.Add(item)

via = pcbnew.PCB_VIA(board)
via.SetPosition(xy(95.15, 64.2))
via.SetDrill(frommm(0.1))
via.SetViaType(pcbnew.VIATYPE_MICROVIA)
via.SetLayerPair(pcbnew.F_Cu, pcbnew.In1_Cu)
for layer in (pcbnew.F_Cu, pcbnew.In1_Cu):
    via.SetWidth(layer, frommm(0.3))
via.SetNetCode(vdd)
board.Add(via)
add_track(vdd, pcbnew.F_Cu, (95.25, 63.9295), (95.15, 64.2), 0.25)
add_track(vdd, pcbnew.In1_Cu, (95.15, 64.2), (95.25, 64.3811), 0.25)
for start, end in (
    ((94.75, 63.4375), (94.65, 64.1)),
    ((94.65, 64.1), (94.65, 65.35)),
    ((94.65, 65.35), (94.4, 65.35)),
    ((94.4, 65.35), (94.4, 66.6)),
    ((94.4, 66.6), (95.0, 67.01)),
):
    add_track(bias, pcbnew.F_Cu, start, end, 0.2)
output.parent.mkdir(parents=True, exist_ok=True)
pcbnew.SaveBoard(str(output), board)
print(f"saved {output}", flush=True)
