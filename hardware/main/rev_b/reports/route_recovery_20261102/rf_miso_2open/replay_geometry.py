#!/usr/bin/python3
"""Apply and verify the pinned copper delta from the route snapshot."""
import collections
import hashlib
import pathlib
import sys

import pcbnew

SOURCE_SHA256 = "fb2fdc622895e726b569e55fe97b2abd1d14d2c7e81d8e46601e996b89cbd3c3"
CHANGED_NETS = {
    "/RF_MISO", "/RF_SCLK",
}


def sha256(path):
    return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()


def pad_map(board):
    result = {}
    for fp in board.GetFootprints():
        for index, pad in enumerate(fp.Pads()):
            result[(fp.GetReference(), index)] = (
                pad.GetNumber(), pad.GetNetname(), pad.GetSize().x,
                pad.GetSize().y, pad.GetShape(), pad.GetAttribute(),
                pad.GetPosition().x, pad.GetPosition().y,
            )
    return result


def copper_map(board):
    result = collections.defaultdict(collections.Counter)
    for item in board.GetTracks():
        net = item.GetNetname()
        if isinstance(item, pcbnew.PCB_VIA):
            pos = item.GetPosition()
            shape = (
                "via", pos.x, pos.y, item.GetViaType(), item.TopLayer(),
                item.BottomLayer(), item.GetWidth(item.TopLayer()),
                item.GetDrillValue(),
            )
        else:
            start, end = item.GetStart(), item.GetEnd()
            ends = tuple(sorted(((start.x, start.y), (end.x, end.y))))
            shape = ("track", item.GetLayer(), ends, item.GetWidth())
        result[net][shape] += 1
    return result


def main():
    source_path, snapshot_path, output_path = map(pathlib.Path, sys.argv[1:4])
    actual_source_sha = sha256(source_path)
    if actual_source_sha != SOURCE_SHA256:
        raise SystemExit(f"unexpected pinned source board SHA-256: {actual_source_sha}")

    source = pcbnew.LoadBoard(str(source_path))
    snapshot = pcbnew.LoadBoard(str(snapshot_path))
    source_pads = pad_map(source)
    snapshot_pads = pad_map(snapshot)
    if len(source_pads) != 657 or source_pads != snapshot_pads:
        raise SystemExit("route snapshot changes footprint pads, assignments, or positions")

    additions = []
    for item in snapshot.GetTracks():
        if item.GetNetname() in CHANGED_NETS:
            clone = item.Duplicate()
            clone.SetNetCode(source.FindNet(item.GetNetname()).GetNetCode())
            additions.append(clone)
    for item in list(source.GetTracks()):
        if item.GetNetname() in CHANGED_NETS:
            source.Remove(item)
    for item in additions:
        source.Add(item)

    del additions, snapshot, snapshot_pads
    pcbnew.ZONE_FILLER(source).Fill(source.Zones())
    pcbnew.SaveBoard(str(output_path), source)
    print(f"source={actual_source_sha}")
    print(f"snapshot={sha256(snapshot_path)}")
    print(f"replay={sha256(output_path)}")
    print(f"changed_nets={','.join(sorted(CHANGED_NETS))}")
    # KiCad's SWIG wrappers can segfault at interpreter shutdown after saving.
    sys.stdout.flush()
    import os
    os._exit(0)


if __name__ == "__main__":
    main()
