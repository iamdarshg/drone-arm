"""Post-FreeRouting-import cleanup, driven by a DRC JSON report:
1. Widen tracks flagged track_width to their netclass minimum width.
2. Remove vias flagged via_dangling.

Usage: kicad-python cleanup_freeroute.py <board.kicad_pcb> <drc_report.json>
"""
import json
import sys
import pcbnew


def min_width_for(netname, board_min):
    """Netclass track widths from tools/hardware/generate_rev_b_pcbs.py."""
    n = netname.split("/")[-1].upper()
    if "GNSS_RF" in n or "RF_IN" in n or "RF_OUT" in n or "ANT" in n:
        return max(0.35, board_min)
    if any(k in n for k in ("3V3", "5V", "1V1", "2V5", "PWR", "VDD", "VCC", "VBUS", "BATT")):
        return max(0.25, board_min)
    if "GATE" in n or n.startswith(("GH", "GL", "SH")):
        return max(0.50, board_min)
    return board_min


def main():
    path, report = sys.argv[1], sys.argv[2]
    board = pcbnew.LoadBoard(path)
    dsn = board.GetDesignSettings()

    widened, removed_v, removed_t, missed = 0, 0, 0, 0
    with open(report, encoding="utf-8") as fh:
        drc = json.load(fh)

    # index all tracks/vias by uuid string
    by_uuid = {}
    for t in board.GetTracks():
        try:
            by_uuid[t.m_Uuid.AsString()] = t
        except AttributeError:
            pass

    for v in drc.get("violations", []):
        if v.get("type") == "track_width":
            for item in v.get("items", []):
                u = item.get("uuid") or ""
                if isinstance(u, list):
                    u = u[0] if u else ""
                t = by_uuid.get(u)
                if t is None or isinstance(t, pcbnew.PCB_VIA):
                    missed += 1
                    continue
                want_mm = min_width_for(t.GetNetname(), dsn.m_TrackMinWidth / 1e6)
                want = int(want_mm * 1e6)
                if t.GetWidth() < want:
                    t.SetWidth(want)
                    widened += 1
        elif v.get("type") in ("via_dangling", "track_dangling"):
            for item in v.get("items", []):
                u = item.get("uuid") or ""
                if isinstance(u, list):
                    u = u[0] if u else ""
                t = by_uuid.get(u)
                if t is None:
                    missed += 1
                    continue
                if v["type"] == "via_dangling" and not isinstance(t, pcbnew.PCB_VIA):
                    missed += 1
                    continue
                board.Remove(t)
                if isinstance(t, pcbnew.PCB_VIA):
                    removed_v += 1
                else:
                    removed_t += 1

    pcbnew.SaveBoard(path, board)
    print(f"widened {widened} tracks, removed {removed_v} vias, {removed_t} tracks, {missed} unmatched")


raise SystemExit(main())


