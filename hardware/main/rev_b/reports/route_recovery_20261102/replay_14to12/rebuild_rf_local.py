#!/usr/bin/env python3
"""Rebuild the rotated FL40 local signal and ground topology."""
import hashlib
import pathlib
import shutil
import sys
import pcbnew
src, out = map(pathlib.Path, sys.argv[1:3])
board = pcbnew.LoadBoard(str(src))
frommm, tomm = pcbnew.FromMM, pcbnew.ToMM
xy = lambda x, y: pcbnew.VECTOR2I(frommm(x), frommm(y))
codes = {n: board.FindNet(n).GetNetCode() for n in (
    "/915 MHz RF/CC_LNA_N", "/915 MHz RF/CC_TRX_SW", "/915 MHz RF/CC_TXPA",
    "/GND", "/915 MHz RF/CC1121_RF", "/915 MHz RF/CC_LNA_P")}
def point(item, which):
    p = item.GetStart() if which == 0 else item.GetEnd()
    return round(tomm(p.x), 4), round(tomm(p.y), 4)
def add(net, points, layer=pcbnew.F_Cu, width=0.25):
    for start, end in zip(points, points[1:]):
        item = pcbnew.PCB_TRACK(board)
        item.SetStart(xy(*start)); item.SetEnd(xy(*end))
        item.SetLayer(layer); item.SetWidth(frommm(width)); item.SetNetCode(codes[net])
        board.Add(item)
remove = []
for item in list(board.GetTracks()):
    if not isinstance(item, pcbnew.PCB_TRACK):
        continue
    a, z = point(item, 0), point(item, 1)
    net, layer = item.GetNetname(), item.GetLayer()
    if net == "/915 MHz RF/CC_LNA_N" and layer == pcbnew.F_Cu and min(a[0], z[0]) > 101.5 and max(a[0], z[0]) < 103.2 and min(a[1], z[1]) > 62.5:
        remove.append(item)
    if net == "/915 MHz RF/CC_TRX_SW" and layer == pcbnew.F_Cu and min(a[0], z[0]) >= 97.298 and max(a[0], z[0]) > 97.31:
        remove.append(item)
    if net == "/915 MHz RF/CC_TXPA" and layer == pcbnew.F_Cu and min(a[0], z[0]) >= 100.0 and max(a[0], z[0]) <= 103.0 and min(a[1], z[1]) >= 60.0 and max(a[1], z[1]) <= 63.0:
        remove.append(item)
    if net == "/GND" and layer == pcbnew.F_Cu and any(100.5 <= p[0] <= 103.6 and 59.4 <= p[1] <= 64.5 for p in (a, z)):
        remove.append(item)
for item in remove:
    board.Remove(item)
add("/915 MHz RF/CC_LNA_N", [(101.375, 61.25), (101.0, 60.8)], width=0.35)
add("/915 MHz RF/CC1121_RF", [(102.625, 62.25), (103.8, 62.25)], width=0.35)
add("/915 MHz RF/CC_LNA_P", [(96.4375, 61.75), (101.375, 61.75)], width=0.35)
add("/915 MHz RF/CC_TRX_SW", [(97.3073, 62.25), (97.3073, 62.1), (100.8, 62.1), (101.375, 62.25)])
add("/915 MHz RF/CC_TXPA", [(101.375, 62.75), (100.71, 62.75)])
pcbnew.ZONE_FILLER(board).Fill(board.Zones())
out.parent.mkdir(parents=True, exist_ok=True)
pcbnew.SaveBoard(str(out), board)
shutil.copyfile(str(src.with_suffix(".kicad_pro")), str(out.with_suffix(".kicad_pro")))
print(f"removed {len(remove)} local tracks; saved {out}", flush=True)
