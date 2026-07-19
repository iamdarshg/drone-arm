#!/usr/bin/env python3
"""Generate the audited custom KiCad footprints used by both Rev-B boards.

The geometry comes from the manufacturer land-pattern drawings mirrored in
docs/datasheets and listed in docs/hardware/SOURCES.md.  The two projects keep
separate copies of the library so they remain independently buildable.
"""

from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
TARGETS = (
    ROOT / "hardware" / "esc" / "rev_b" / "revb.pretty",
    ROOT / "hardware" / "main" / "rev_b" / "revb.pretty",
)


def property_text(kind: str, text: str, y: float, layer: str) -> str:
    return f"""  (property "{kind}" "{text}"
    (at 0 {y:g} 0)
    (layer "{layer}")
    (effects (font (size 1 1) (thickness 0.15)))
  )"""


def pad(
    number: str,
    x: float,
    y: float,
    sx: float,
    sy: float,
    *,
    kind: str = "smd",
    shape: str = "roundrect",
    layers: str = '"F.Cu" "F.Paste" "F.Mask"',
    drill: float | None = None,
    radius: float = 0.12,
) -> str:
    drill_text = f" (drill {drill:g})" if drill is not None else ""
    round_text = f" (roundrect_rratio {radius:g})" if shape == "roundrect" else ""
    return (
        f'  (pad "{number}" {kind} {shape} (at {x:g} {y:g}) '
        f"(size {sx:g} {sy:g}){drill_text} (layers {layers}){round_text})"
    )


def silk_box(x: float, y: float) -> list[str]:
    return [
        f'  (fp_rect (start {-x:g} {-y:g}) (end {x:g} {y:g}) '
        '(stroke (width 0.12) (type solid)) (fill none) (layer "F.SilkS"))',
        f'  (fp_rect (start {-x:g} {-y:g}) (end {x:g} {y:g}) '
        '(stroke (width 0.05) (type solid)) (fill none) (layer "F.CrtYd"))',
    ]


def footprint(name: str, descr: str, body: list[str], ref_y: float, value_y: float) -> str:
    return "\n".join(
        [
            f'(footprint "{name}"',
            "  (version 20240108)",
            '  (generator "drone-arm-rev-b-footprint-generator")',
            '  (layer "F.Cu")',
            f'  (descr "{descr}")',
            property_text("Reference", "REF**", ref_y, "F.SilkS"),
            property_text("Value", name, value_y, "F.Fab"),
            "  (attr smd)",
            *body,
            ")",
            "",
        ]
    )


def wqfn40() -> str:
    # TI RTA0040B: 6 mm body, 0.5 mm pitch, 0.22 x 0.6 mm lands,
    # 4.15 mm exposed pad.  Pin sequence follows the TI bottom view.
    body = silk_box(3.1, 3.1)
    coords = [(-2.25 + 0.5 * i) for i in range(10)]
    for i, x in enumerate(coords, 1):
        body.append(pad(str(i), x, 2.9, 0.22, 0.6))
    for i, y in enumerate(reversed(coords), 11):
        body.append(pad(str(i), 2.9, y, 0.6, 0.22))
    for i, x in enumerate(reversed(coords), 21):
        body.append(pad(str(i), x, -2.9, 0.22, 0.6))
    for i, y in enumerate(coords, 31):
        body.append(pad(str(i), -2.9, y, 0.6, 0.22))
    body.append(pad("41", 0, 0, 4.15, 4.15, layers='"F.Cu" "F.Mask"', radius=0.03))
    for x in (-1.35, -0.45, 0.45, 1.35):
        for y in (-1.35, -0.45, 0.45, 1.35):
            body.append(
                pad(
                    "41", x, y, 0.55, 0.55,
                    kind="thru_hole", shape="circle",
                    layers='"*.Cu" "*.Mask"', drill=0.2,
                )
            )
    for x in (-1.35, -0.45, 0.45, 1.35):
        for y in (-1.35, -0.45, 0.45, 1.35):
            body.append(pad("", x, y, 0.65, 0.65, layers='"F.Paste"', radius=0.08))
    return footprint(
        "WQFN-40-EP_6x6_P0.5",
        "TI RTA0040B WQFN-40 land pattern from DRV8353 datasheet",
        body, -4.0, 4.0,
    )


def tolt() -> str:
    # Infineon Figure 2.  Copper drain slug is 10.2 x 7 mm.  Sixteen
    # 0.8 x 3.375 mm lead lands are on 1.2 mm pitch.
    body = silk_box(5.3, 7.4)
    xs = [(-4.2 + 1.2 * i) for i in range(8)]
    for pin_no, x in enumerate(xs, 1):
        body.append(pad("1" if pin_no < 8 else "8", x, 5.5, 0.8, 3.375))
    for pin_no, x in zip(range(16, 8, -1), xs):
        body.append(pad("9", x, -5.5, 0.8, 3.375))
    body.append(pad("9", 0, 0, 10.2, 7.0, layers='"F.Cu" "F.Mask"', radius=0.02))
    for x in (-3.4, 0, 3.4):
        for y in (-2.25, 0, 2.25):
            body.append(pad("", x, y, 2.75, 1.55, layers='"F.Paste"', radius=0.03))
    body.append(
        '  (fp_circle (center -5.8 6.8) (end -5.5 6.8) '
        '(stroke (width 0.2) (type solid)) (fill none) (layer "F.SilkS"))'
    )
    return footprint(
        "PG-HDSOP-16_TOLT_IPTC",
        "Infineon PG-HDSOP-16-U01 TOLT; pins 1-7 source, 8 gate, 9-16 and slug drain",
        body, -8.4, 8.4,
    )


def css4j() -> str:
    # Bourns page 2: full copper width 10.60, 5.50 mm power-pad gap,
    # 5.60 mm power-pad height, 0.80 mm sense-pad height.
    body = silk_box(5.45, 3.75)
    body.extend(
        [
            pad("1", -4.025, 0, 2.55, 5.6),
            pad("2", 4.025, 0, 2.55, 5.6),
            pad("3", -0.85, -3.25, 0.8, 0.8),
            pad("4", 0.85, -3.25, 0.8, 0.8),
        ]
    )
    return footprint(
        "CSS4J-4026_Kelvin",
        "Bourns CSS4J-4026 four-terminal Kelvin land pattern",
        body, -4.7, 4.7,
    )


def power_lug(count: int) -> str:
    pitch = 20.0
    xs = [pitch * (i - (count - 1) / 2) for i in range(count)]
    span = max(abs(x) for x in xs) + 7.0
    body = silk_box(span, 7.0)
    body[1] = (
        f'  (fp_rect (start {-span:g} -7) (end {span:g} 7) '
        '(stroke (width 0.05) (type solid)) (fill none) (layer "F.CrtYd"))'
    )
    body[0] = (
        f'  (fp_rect (start {-span:g} -7) (end {span:g} 7) '
        '(stroke (width 0.3) (type solid)) (fill none) (layer "F.SilkS"))'
    )
    for i, x in enumerate(xs, 1):
        body.append(
            pad(
                str(i), x, 0, 12.0, 12.0,
                kind="thru_hole", shape="circle",
                layers='"*.Cu" "*.Mask"', drill=5.5,
            )
        )
    name = f"POWER_LUG_M5_{count}P_P20"
    return footprint(
        name,
        "M5 bolted ring-lug/busbar landing; 12 mm annular copper, 20 mm pitch",
        body, -8.5, 8.5,
    ).replace("  (attr smd)", "  (attr through_hole)")


def tps3430() -> str:
    # TI DRC0010J land pattern: 10 x 0.24 x 0.6, 0.5 pitch,
    # exposed pad 1.65 x 2.4 mm.
    body = silk_box(1.65, 1.65)
    ys = [1.0, 0.5, 0, -0.5, -1.0]
    for number, y in enumerate(ys, 1):
        body.append(pad(str(number), -1.4, y, 0.6, 0.24))
    for number, y in enumerate(reversed(ys), 6):
        body.append(pad(str(number), 1.4, y, 0.6, 0.24))
    body.append(pad("11", 0, 0, 1.65, 2.4, layers='"F.Cu" "F.Mask"', radius=0.03))
    for y in (-0.6, 0.6):
        body.append(
            pad(
                "11", 0, y, 0.45, 0.45,
                kind="thru_hole", shape="circle",
                layers='"*.Cu" "*.Mask"', drill=0.2,
            )
        )
    for y in (-0.6, 0.6):
        body.append(pad("", 0, y, 0.95, 0.85, layers='"F.Paste"', radius=0.03))
    return footprint(
        "TPS3430_WSON10_EP",
        "TI DRC0010J VSON/WSON-10 recommended land pattern",
        body, -2.6, 2.6,
    )


def do218ab() -> str:
    # Littelfuse SM8S SMTO-263 / DO-218AB compatible soldering outline.
    # The large terminal uses the manufacturer 10.5 x 8.41 mm copper area;
    # the smaller lead uses 8.13 x 3.0 mm, with 3.3 mm edge gap.
    body = silk_box(5.45, 8.1)
    body.extend(
        [
            pad("1", 0, -3.145, 10.5, 8.41, radius=0.03),
            pad("2", 0, 6.4, 8.13, 3.0, radius=0.03),
        ]
    )
    return footprint(
        "DO-218AB_SM8S",
        "Littelfuse SM8S SMTO-263 / DO-218AB compatible recommended land pattern",
        body, -9.2, 9.2,
    )


def lga14(name: str, maker: str) -> str:
    body = silk_box(1.35, 1.6)
    side_y = [-0.75, -0.25, 0.25, 0.75]
    for number, y in enumerate(side_y, 1):
        body.append(pad(str(number), -1.1625, y, 0.475, 0.25))
    for number, x in enumerate((-0.5, 0, 0.5), 5):
        body.append(pad(str(number), x, 1.4125, 0.25, 0.475))
    for number, y in enumerate(reversed(side_y), 8):
        body.append(pad(str(number), 1.1625, y, 0.475, 0.25))
    for number, x in enumerate((0.5, 0, -0.5), 12):
        body.append(pad(str(number), x, -1.4125, 0.25, 0.475))
    body.append(
        '  (fp_circle (center -1.65 -1.35) (end -1.45 -1.35) '
        '(stroke (width 0.15) (type solid)) (fill none) (layer "F.SilkS"))'
    )
    return footprint(
        name,
        f"{maker} LGA-14 2.5 x 3.0 mm, 0.5 mm pitch manufacturer geometry",
        body, -2.5, 2.5,
    )


def lg77l() -> str:
    body = silk_box(3.6, 3.6)
    # Quectel LG77L(C) Hardware Design V1.3 asks for at least 3 mm of
    # component keepout around the 7 x 7 mm module.  Express that constraint
    # as the courtyard so board DRC catches encroaching support parts.
    body[1] = (
        '  (fp_rect (start -6.6 -6.6) (end 6.6 6.6) '
        '(stroke (width 0.05) (type solid)) (fill none) (layer "F.CrtYd"))'
    )
    perimeter = []
    axis = [-2.6 + 0.65 * i for i in range(9)]
    for number, y in enumerate(axis, 1):
        perimeter.append((number, -3.2, y, 0.65, 0.3))
    for number, x in enumerate(axis, 10):
        perimeter.append((number, x, 3.2, 0.3, 0.65))
    for number, y in enumerate(reversed(axis), 19):
        perimeter.append((number, 3.2, y, 0.65, 0.3))
    for number, x in enumerate(reversed(axis), 28):
        perimeter.append((number, x, -3.2, 0.3, 0.65))
    for number, x, y, sx, sy in perimeter:
        body.append(pad(str(number), x, y, sx, sy))
    interior = {
        37: (-1.3, -1.3), 38: (0, -1.3),
        39: (-1.3, 0), 40: (0, 0),
        41: (-1.3, 1.3), 42: (0, 1.3), 43: (1.3, 1.3),
    }
    for number, (x, y) in interior.items():
        body.append(pad(str(number), x, y, 0.5, 0.5))
    body.append(
        '  (fp_circle (center -4 -2.6) (end -3.75 -2.6) '
        '(stroke (width 0.2) (type solid)) (fill none) (layer "F.SilkS"))'
    )
    return footprint(
        "LG77L_LGA43",
        "Quectel LG77L 7 x 7 mm LGA-43 footprint with V1.3 3 mm component keepout",
        body, -4.8, 4.8,
    )


def rf_filter() -> str:
    # EPCOS DCC6C 3 x 3 mm.  Pads 2/5 are the side RF ports; 1/3/4/6
    # are the four grounded corner terminals.
    body = silk_box(1.6, 1.6)
    body.extend(
        [
            pad("1", -0.9, -1.2, 0.6, 1.5),
            pad("3", 0.9, -1.2, 0.6, 1.5),
            pad("6", -0.9, 1.2, 0.6, 1.5),
            pad("4", 0.9, 1.2, 0.6, 1.5),
            pad("2", -1.2, 0, 1.5, 0.6),
            pad("5", 1.2, 0, 1.5, 0.6),
        ]
    )
    return footprint(
        "RF_Filter_4Port",
        "EPCOS B39921B3588U410 DCC6C 3 x 3 mm six-pad SAW",
        body, -2.5, 2.5,
    )


def lfd21868() -> str:
    # Murata CD386: 2.0 x 1.25 mm, 0.5 mm terminal pitch.  Pins 5
    # and 10 are the two centre side ground pads.
    body = silk_box(1.1, 0.725)
    for number, x in zip((4, 3, 2, 1), (-0.75, -0.25, 0.25, 0.75)):
        body.append(pad(str(number), x, -0.625, 0.3, 0.4))
    for number, x in zip((6, 7, 8, 9), (-0.75, -0.25, 0.25, 0.75)):
        body.append(pad(str(number), x, 0.625, 0.3, 0.4))
    body.extend(
        [
            pad("5", -1.0, 0, 0.4, 0.3),
            pad("10", 1.0, 0, 0.4, 0.3),
        ]
    )
    return footprint(
        "LFD21868MMF1D386",
        "Murata LFD21868MMF1D386 2.0 x 1.25 mm ten-terminal dual matching filter",
        body, -1.7, 1.7,
    )


def rfm_0505s() -> str:
    # RECOM RFM SIP4: 11.5 x 6.0 mm body and four 1.0 mm pins on
    # 2.54 mm pitch.  Pin order is -Vin, +Vin, -Vout, +Vout.
    body = silk_box(5.75, 3.0)
    for number, x in enumerate((-3.81, -1.27, 1.27, 3.81), 1):
        body.append(
            pad(
                str(number), x, 0, 1.9, 1.9,
                kind="thru_hole", shape="circle",
                layers='"*.Cu" "*.Mask"', drill=1.1,
            )
        )
    body.append(
        '  (fp_circle (center -4.9 -2.2) (end -4.55 -2.2) '
        '(stroke (width 0.2) (type solid)) (fill none) (layer "F.SilkS"))'
    )
    return footprint(
        "RFM_SIP4_11.5x6_P2.54",
        "RECOM RFM-0505S SIP4, 11.5 x 6.0 mm, 2.54 mm pitch",
        body, -4.0, 4.0,
    ).replace("  (attr smd)", "  (attr through_hole)")


def generate() -> None:
    parts = {
        "WQFN-40-EP_6x6_P0.5": wqfn40(),
        "PG-HDSOP-16_TOLT_IPTC": tolt(),
        "CSS4J-4026_Kelvin": css4j(),
        "POWER_LUG_M5_2P_P20": power_lug(2),
        "POWER_LUG_M5_3P_P20": power_lug(3),
        "TPS3430_WSON10_EP": tps3430(),
        "DO-218AB_SM8S": do218ab(),
        "ICM42688P_LGA14": lga14("ICM42688P_LGA14", "TDK InvenSense"),
        "LSM6DSO32_LGA14": lga14("LSM6DSO32_LGA14", "STMicroelectronics"),
        "LG77L_LGA43": lg77l(),
        "RF_Filter_4Port": rf_filter(),
        "LFD21868MMF1D386": lfd21868(),
        "RFM_SIP4_11.5x6_P2.54": rfm_0505s(),
    }
    for target in TARGETS:
        target.mkdir(parents=True, exist_ok=True)
        for old in target.glob("*.kicad_mod"):
            old.unlink()
        for name, text in parts.items():
            (target / f"{name}.kicad_mod").write_text(text, encoding="utf-8", newline="\n")
    print(f"Generated {len(parts)} audited footprints in each Rev-B project")


if __name__ == "__main__":
    generate()
