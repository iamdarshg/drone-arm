#!/usr/bin/env python3
"""Export grouped Rev-B BOM rows from the validated KiCad XML netlists."""

from __future__ import annotations

import argparse
import csv
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path


def field(comp: ET.Element, name: str) -> str:
    for item in comp.findall("./fields/field"):
        if item.attrib.get("name") == name:
            return (item.text or "").strip()
    return ""


def grouped_rows(board_name: str, path: Path) -> list[dict[str, object]]:
    groups: dict[tuple[str, str, str, str], list[str]] = defaultdict(list)
    root = ET.parse(path).getroot()
    for comp in root.findall("./components/comp"):
        ref = comp.attrib["ref"]
        value = comp.findtext("value", default="").strip()
        footprint = comp.findtext("footprint", default="").strip()
        manufacturer = field(comp, "Manufacturer")
        mpn = field(comp, "MPN") or value
        groups[(manufacturer, mpn, value, footprint)].append(ref)
    rows: list[dict[str, object]] = []
    for (manufacturer, mpn, value, footprint), refs in sorted(
        groups.items(), key=lambda item: (item[0][0], item[0][1], item[0][2])
    ):
        rows.append(
            {
                "Board": board_name,
                "Manufacturer": manufacturer or "Commodity / unspecified",
                "MPN": mpn,
                "Value": value,
                "Footprint": footprint,
                "Quantity": len(refs),
                "References": ", ".join(sorted(refs)),
            }
        )
    return rows


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--esc", type=Path, required=True)
    parser.add_argument("--main", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rows = grouped_rows("ESC Rev-B", args.esc) + grouped_rows(
        "Flight Control Rev-B", args.main
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f"Wrote {len(rows)} grouped BOM rows to {args.output}")


if __name__ == "__main__":
    main()
