
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    g = board.FindNet("/CAN_GND")
    v5 = board.FindNet("/CAN_5V")
    removed = 0

    # The GND-net tracks near R3 belong to R3 pad2's own GND connection -
    # they extend right/down from pad2. My CAN_GND route at y=35.2 and the
    # (23.85, 25.9->35.2) riser collide with them. Check where R3's GND track
    # actually goes: probe all /GND F.Cu tracks in x>18, y>24:
    for t in board.GetTracks():
        if t.GetNetname() == "/GND" and type(t).__name__ == "PCB_TRACK" and t.IsOnLayer(pcbnew.F_Cu):
            s, e = t.GetStart(), t.GetEnd()
            sx, sy = pcbnew.ToMM(s.x), pcbnew.ToMM(s.y)
            ex, ey = pcbnew.ToMM(e.x), pcbnew.ToMM(e.y)
            if max(sx,ex) > 17 and min(sy,ey) > 23:
                print("GND:", (round(sx,2),round(sy,2)), '->', (round(ex,2),round(ey,2)))
    # remove my CAN_GND bottom/riser segments that conflict
    doomed = []
    for t in list(board.GetTracks()):
        if type(t).__name__ != "PCB_TRACK" or t.GetNetname() != "/CAN_GND":
            continue
        s, e = t.GetStart(), t.GetEnd()
        sx, sy, ex, ey = pcbnew.ToMM(s.x), pcbnew.ToMM(s.y), pcbnew.ToMM(e.x), pcbnew.ToMM(e.y)
        if abs(sy - 25.9) < 0.05 and abs(ey - 25.9) < 0.05:
            doomed.append(t)
        elif abs(sx - 21.62) < 0.05 and abs(ex - 21.62) < 0.05 and abs(max(sy,ey) - 25.9) < 0.6:
            doomed.append(t)
        elif abs(sx - 23.85) < 0.05 and abs(ex - 23.85) < 0.05:
            doomed.append(t)
        elif abs(sy - 35.2) < 0.05 and abs(ey - 35.2) < 0.05 and max(sx,ex) > 16:
            doomed.append(t)
    seen = set(); uniq = []
    for t in doomed:
        if id(t) not in seen:
            seen.add(id(t)); uniq.append(t)
    for t in uniq:
        try:
            board.Remove(t); removed += 1
        except Exception:
            pass
    print("removed", removed)
    pcbnew.SaveBoard(sys.argv[1], board)
    return 0


raise SystemExit(main())

