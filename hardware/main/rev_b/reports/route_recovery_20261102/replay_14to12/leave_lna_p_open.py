#!/usr/bin/env python3
"""Discard the DRC-regressing direct CC_LNA_P trial; leave that net open."""
import pathlib
import shutil
import sys
import pcbnew
src, out = map(pathlib.Path, sys.argv[1:3])
board = pcbnew.LoadBoard(str(src))
frommm, tomm = pcbnew.FromMM, pcbnew.ToMM
net = board.FindNet("/915 MHz RF/CC_LNA_P").GetNetCode()
removed = 0
for item in list(board.GetTracks()):
    if isinstance(item, pcbnew.PCB_TRACK) and item.GetNetCode() == net and item.GetLayer() == pcbnew.F_Cu:
        points = []
        for p in (item.GetStart(), item.GetEnd()):
            points.append((round(tomm(p.x), 4), round(tomm(p.y), 4)))
        if (96.4375, 61.75) in points or (101.375, 61.75) in points:
            board.Remove(item); removed += 1
pcbnew.ZONE_FILLER(board).Fill(board.Zones())
out.parent.mkdir(parents=True, exist_ok=True)
pcbnew.SaveBoard(str(out), board)
shutil.copyfile(str(src.with_suffix(".kicad_pro")), str(out.with_suffix(".kicad_pro")))
print(f"removed {removed} CC_LNA_P trial segment(s); saved {out}", flush=True)
