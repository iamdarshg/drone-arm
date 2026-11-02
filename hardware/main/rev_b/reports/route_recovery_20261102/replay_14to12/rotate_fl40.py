#!/usr/bin/env python3
"""Rotate FL40 and move its CC_LNA_N In2 endpoint with the rotated pad."""
import pathlib
import shutil
import sys
import pcbnew
src, out = map(pathlib.Path, sys.argv[1:3])
board = pcbnew.LoadBoard(str(src))
frommm, tomm = pcbnew.FromMM, pcbnew.ToMM
xy = lambda x, y: pcbnew.VECTOR2I(frommm(x), frommm(y))
f = next(x for x in board.GetFootprints() if x.GetReference() == "FL40")
f.SetOrientationDegrees(270)
old, new = (102.098, 63.6631), (101.0, 60.8)
for item in list(board.GetTracks()):
    if isinstance(item, pcbnew.PCB_VIA) and item.GetNetname() == "/915 MHz RF/CC_LNA_N":
        p = item.GetPosition()
        if abs(tomm(p.x) - old[0]) < 0.002 and abs(tomm(p.y) - old[1]) < 0.002:
            item.SetPosition(xy(*new))
for item in list(board.GetTracks()):
    if isinstance(item, pcbnew.PCB_TRACK) and item.GetNetname() == "/915 MHz RF/CC_LNA_N" and item.GetLayer() == pcbnew.In2_Cu:
        for which in ("GetStart", "GetEnd"):
            p = getattr(item, which)()
            if abs(tomm(p.x) - old[0]) < 0.002 and abs(tomm(p.y) - old[1]) < 0.002:
                getattr(item, "SetStart" if which == "GetStart" else "SetEnd")(xy(*new))
out.parent.mkdir(parents=True, exist_ok=True)
pcbnew.SaveBoard(str(out), board)
shutil.copyfile(str(src.with_suffix(".kicad_pro")), str(out.with_suffix(".kicad_pro")))
print(f"saved {out}", flush=True)
