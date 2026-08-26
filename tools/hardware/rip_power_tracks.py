"""Rip up power-distribution tracks (and optional vias) to clear the field for
signal routing. Zones and plane copper are untouched. High-power nets
(BATP/BATN/PHASE) and the hand-routed CAN bus are preserved.

Usage: kicad-python rip_power_tracks.py <board.kicad_pcb> [--vias] [--dry]
"""
import sys
import pcbnew

POWER_NETS_EXACT = {
    "GND", "DGND", "AGND", "3V3", "3.3V", "+3V3", "5V", "+5V", "SYS_5V",
    "1V1", "+1V1", "2V5", "+2V5", "AUX_GND", "AUX_BATT", "VDD", "VSS",
    "VBUS", "VREG", "LDO_OUT", "DVDD", "AVDD",
}
KEEP_PREFIXES = ("BATP", "BATN", "PHASE", "CAN_5V", "CAN_GND", "CANH", "CANL")


def main():
    path = sys.argv[1]
    do_vias = "--vias" in sys.argv
    dry = "--dry" in sys.argv
    board = pcbnew.LoadBoard(path)

    removed_t, removed_v = 0, 0
    net_hits = {}
    tracks = list(board.GetTracks())
    for t in tracks:
        is_via = isinstance(t, pcbnew.PCB_VIA)
        is_track = isinstance(t, pcbnew.PCB_TRACK)
        if not (is_track or is_via):
            continue
        if is_via and not do_vias:
            continue
        name = t.GetNetname()
        if not name:
            continue
        base = name.split("/")[-1].strip()
        if base in POWER_NETS_EXACT and not any(base.startswith(k) for k in KEEP_PREFIXES):
            if dry:
                net_hits[name] = net_hits.get(name, 0) + 1
                continue
            board.Remove(t)
            if is_via:
                removed_v += 1
            else:
                removed_t += 1
            net_hits[name] = net_hits.get(name, 0) + 1

    if dry:
        print("DRY RUN - would remove:")
        for n, c in sorted(net_hits.items(), key=lambda kv: -kv[1]):
            print(f"  {n}: {c}")
    else:
        pcbnew.SaveBoard(path, board)
        print(f"removed {removed_t} tracks, {removed_v} vias")
        for n, c in sorted(net_hits.items(), key=lambda kv: -kv[1]):
            print(f"  {n}: {c}")


raise SystemExit(main())
