#!/usr/bin/env python3
"""Generate the modular Rev-B ESC and flight-control schematics.

The generated files live beside, but never overwrite, the original KiCad
projects.  Pin maps in the custom library are copied from the source documents
listed in docs/hardware/SOURCES.md.  This script is deliberately deterministic
so that every generated schematic can be reproduced and audited.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import json
import re
import shutil

import kicad_sch_api as ksa


ROOT = Path(__file__).resolve().parents[2]
ESC_DIR = ROOT / "hardware" / "esc" / "rev_b"
MAIN_DIR = ROOT / "hardware" / "main" / "rev_b"


@dataclass(frozen=True)
class SymbolSpec:
    name: str
    reference: str
    footprint: str
    datasheet: str
    pins: tuple[tuple[str, str], ...]


DRV8353_PINS = (
    ("1", "CPL"), ("2", "CPH"), ("3", "VM"), ("4", "VDRAIN"),
    ("5", "VCP"), ("6", "GHA"), ("7", "SHA"), ("8", "GLA"),
    ("9", "SPA"), ("10", "SNA"), ("11", "SNB"), ("12", "SPB"),
    ("13", "GLB"), ("14", "SHB"), ("15", "GHB"), ("16", "GHC"),
    ("17", "SHC"), ("18", "GLC"), ("19", "SPC"), ("20", "SNC"),
    ("21", "SOC"), ("22", "SOB"), ("23", "SOA"), ("24", "VREF"),
    ("25", "AGND"), ("26", "nFAULT"), ("27", "SDO"), ("28", "SDI"),
    ("29", "SCLK"), ("30", "nSCS"), ("31", "ENABLE"), ("32", "INHA"),
    ("33", "INLA"), ("34", "INHB"), ("35", "INLB"), ("36", "INHC"),
    ("37", "INLC"), ("38", "DVDD"), ("39", "GND"), ("40", "VGLS"),
    ("41", "EP"),
)

ADS7038_PINS = (
    ("1", "AIN2_GPIO2"), ("2", "AIN3_GPIO3"), ("3", "AIN4_GPIO4"),
    ("4", "AIN5_GPIO5"), ("5", "AIN6_GPIO6"), ("6", "AIN7_GPIO7"),
    ("7", "AVDD"), ("8", "DECAP"), ("9", "GND"), ("10", "DVDD"),
    ("11", "CS"), ("12", "SDO"), ("13", "SCLK"), ("14", "SDI"),
    ("15", "AIN0_GPIO0"), ("16", "AIN1_GPIO1"), ("17", "EP"),
)

INA296_PINS = (
    ("1", "IN-"), ("2", "GND"), ("3", "REF2"), ("4", "NC"),
    ("5", "OUT"), ("6", "VS"), ("7", "REF1"), ("8", "IN+"),
)

TPS709_PINS = (
    ("1", "IN"), ("2", "GND"), ("3", "EN"), ("4", "NC"), ("5", "OUT"),
)

TPS3430_PINS = (
    ("1", "VDD1"), ("2", "CWD"), ("3", "SET0"), ("4", "CRST"),
    ("5", "GND"), ("6", "SET1"), ("7", "WDI"), ("8", "nWDO"),
    ("9", "NC"), ("10", "VDD2"), ("11", "EP"),
)

TCAN3413_PINS = (
    ("1", "TXD"), ("2", "GND"), ("3", "VCC"), ("4", "RXD"),
    ("5", "VIO"), ("6", "CANL"), ("7", "CANH"), ("8", "STB"),
)

ICM42688_PINS = (
    ("1", "AP_SDO_AD0"), ("2", "RESV"), ("3", "RESV"),
    ("4", "INT1"), ("5", "VDDIO"), ("6", "GND"), ("7", "RESV_GND"),
    ("8", "VDD"), ("9", "INT2_FSYNC_CLKIN"), ("10", "RESV"),
    ("11", "RESV"), ("12", "AP_CS"), ("13", "AP_SCL_SCLK"),
    ("14", "AP_SDA_SDIO_SDI"),
)

LSM6DSO32_PINS = (
    ("1", "SDO_SA0"), ("2", "SDx"), ("3", "SCx"), ("4", "INT1"),
    ("5", "VDDIO"), ("6", "GND"), ("7", "GND"), ("8", "VDD"),
    ("9", "INT2"), ("10", "NC"), ("11", "NC"), ("12", "CS"),
    ("13", "SCL_SPC"), ("14", "SDA_SDI_SDO"),
)

CC1121_PINS = (
    ("1", "VDD_GUARD"), ("2", "RESET_N"), ("3", "GPIO3"),
    ("4", "GPIO2"), ("5", "DVDD"), ("6", "DCPL"), ("7", "SI"),
    ("8", "SCLK"), ("9", "SO_GPIO1"), ("10", "GPIO0"), ("11", "CSn"),
    ("12", "DVDD"), ("13", "AVDD_IF"), ("14", "RBIAS"),
    ("15", "AVDD_RF"), ("16", "NC"), ("17", "PA"), ("18", "TRX_SW"),
    ("19", "LNA_P"), ("20", "LNA_N"), ("21", "DCPL_VCO"),
    ("22", "AVDD_SYNTH1"), ("23", "LPF0"), ("24", "LPF1"),
    ("25", "AVDD_PFD_CHP"), ("26", "DCPL_PFD_CHP"),
    ("27", "AVDD_SYNTH2"), ("28", "AVDD_XOSC"), ("29", "DCPL_XOSC"),
    ("30", "XOSC_Q1"), ("31", "XOSC_Q2"), ("32", "EXT_XOSC"),
    ("33", "EP"),
)

CC1190_PINS = (
    ("1", "GND"), ("2", "PA_OUT"), ("3", "GND"), ("4", "TR_SW"),
    ("5", "LNA_IN"), ("6", "HGM"), ("7", "LNA_EN"), ("8", "PA_EN"),
    ("9", "GND"), ("10", "LNA_OUT"), ("11", "PA_IN"), ("12", "GND"),
    ("13", "VDD_LNA"), ("14", "BIAS"), ("15", "VDD_PA2"),
    ("16", "VDD_PA1"), ("17", "EP"),
)

LG77L_PINS = (
    ("1", "RF_IN"), ("2", "GND"), ("3", "RESERVED"), ("4", "RESERVED"),
    ("5", "RESERVED"), ("6", "RESERVED"), ("7", "RESERVED"),
    ("8", "RTC_O"), ("9", "RTC_I"), ("10", "3D_FIX"),
    ("11", "RESERVED"), ("12", "ANT_DET"), ("13", "WAKEUP"),
    ("14", "RESET_N"), ("15", "RXD"), ("16", "TXD"),
    ("17", "RESERVED"), ("18", "GND"), ("19", "VCC"),
    ("20", "VCC_IO"), ("21", "V_BCKP"), ("22", "RESERVED"),
    ("23", "RESERVED"), ("24", "RESERVED"), ("25", "RESERVED"),
    ("26", "RESERVED"), ("27", "RESERVED"), ("28", "JAM_IND"),
    ("29", "I2C_SCL"), ("30", "I2C_SDA"), ("31", "1PPS"),
    ("32", "RESERVED"), ("33", "RESERVED"), ("34", "ANT_SHORT"),
    ("35", "ANT_OFF"), ("36", "RESERVED"), ("37", "GND"),
    ("38", "GND"), ("39", "GND"), ("40", "GND"), ("41", "GND"),
    ("42", "GND"), ("43", "GND"),
)

def _symbol_block(text: str, symbol_name: str) -> str:
    marker = f'(symbol "{symbol_name}"'
    start = text.find(marker)
    if start < 0:
        raise ValueError(f"Symbol {symbol_name!r} not found")
    depth = 0
    in_string = False
    escaped = False
    for index in range(start, len(text)):
        char = text[index]
        if in_string:
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == '"':
                in_string = False
            continue
        if char == '"':
            in_string = True
        elif char == "(":
            depth += 1
        elif char == ")":
            depth -= 1
            if depth == 0:
                return text[start:index + 1]
    raise ValueError(f"Unterminated symbol {symbol_name!r}")


def _pins_from_committed_symbol(path: Path, symbol_name: str) -> tuple[tuple[str, str], ...]:
    block = _symbol_block(path.read_text(encoding="utf-8"), symbol_name)
    pins: list[tuple[str, str]] = []
    for section in block.split("(pin ")[1:]:
        name_match = re.search(r'\(name "([^"\n]+)"', section)
        number_match = re.search(r'\(number "([^"\n]+)"', section)
        if name_match and number_match:
            pins.append((number_match.group(1), name_match.group(1)))
    if not pins:
        raise ValueError(f"No pins parsed for {symbol_name!r} from {path}")
    return tuple(pins)


def _load_rp2354_pins() -> tuple[tuple[str, str], ...]:
    installed = ksa.get_symbol_info("MCU_RaspberryPi:RP2354B")
    if installed is not None:
        return tuple((pin.number, pin.name) for pin in installed.pins)
    for candidate in (MAIN_DIR / "revb.kicad_sym", ESC_DIR / "revb.kicad_sym"):
        if candidate.exists():
            return _pins_from_committed_symbol(candidate, "RP2354B")
    raise RuntimeError(
        "RP2354B is absent from the installed KiCad library and no committed "
        "Rev-B symbol library is available"
    )


RP2354B_PINS = _load_rp2354_pins()


SYMBOLS = (
    SymbolSpec(
        "RP2354B", "U", "Package_DFN_QFN:QFN-80-1EP_10x10mm_P0.4mm_EP3.4x3.4mm",
        "https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf",
        RP2354B_PINS,
    ),
    SymbolSpec(
        "DRV8353S", "U", "revb:WQFN-40-EP_6x6_P0.5",
        "https://www.ti.com/lit/ds/symlink/drv8353.pdf", DRV8353_PINS
    ),
    SymbolSpec(
        "IPTC014N10NM5", "Q", "revb:PG-HDSOP-16_TOLT_IPTC",
        "https://www.infineon.com/assets/row/public/documents/24/49/infineon-iptc014n10nm5-datasheet-en.pdf",
        (("1", "S"), ("8", "G"), ("9", "D")),
    ),
    SymbolSpec(
        "CSS4J_4026R_L500F", "R", "revb:CSS4J-4026_Kelvin",
        "https://bourns.com/docs/product-datasheets/css4j-4026.pdf",
        (("1", "I_PLUS"), ("2", "I_MINUS"), ("3", "S_PLUS"), ("4", "S_MINUS")),
    ),
    SymbolSpec(
        "ADS7038", "U", "Package_DFN_QFN:WQFN-16-1EP_3x3mm_P0.5mm_EP1.75x1.75mm",
        "https://www.ti.com/lit/ds/symlink/ads7038.pdf", ADS7038_PINS,
    ),
    SymbolSpec(
        "INA296A", "U", "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm",
        "https://www.ti.com/lit/ds/symlink/ina296a.pdf", INA296_PINS,
    ),
    SymbolSpec(
        "TPS70933", "U", "Package_TO_SOT_SMD:SOT-23-5",
        "https://www.ti.com/lit/ds/symlink/tps709.pdf", TPS709_PINS,
    ),
    SymbolSpec(
        "RFM_0505S", "PS", "revb:RFM_SIP4_11.5x6_P2.54",
        "https://recom-power.com/pdf/Econoline/RFM.pdf",
        (("1", "-VIN"), ("2", "+VIN"), ("3", "-VOUT"), ("4", "+VOUT")),
    ),
    SymbolSpec(
        "TPS3430", "U", "revb:TPS3430_WSON10_EP",
        "https://www.ti.com/lit/ds/symlink/tps3430.pdf", TPS3430_PINS,
    ),
    SymbolSpec(
        "TCAN3413", "U", "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm",
        "https://www.ti.com/lit/ds/symlink/tcan3413.pdf", TCAN3413_PINS,
    ),
    SymbolSpec(
        "PESD2CANFD24L", "D", "Package_TO_SOT_SMD:SOT-23",
        "https://assets.nexperia.com/documents/data-sheet/PESD2CANFD24L-T.pdf",
        (("1", "CAN1"), ("2", "CAN2"), ("3", "COMMON")),
    ),
    SymbolSpec(
        "ICM42688P", "U", "revb:ICM42688P_LGA14",
        "https://invensense.tdk.com/wp-content/uploads/2020/04/ds-000347_icm-42688-p-datasheet.pdf",
        ICM42688_PINS,
    ),
    SymbolSpec(
        "LSM6DSO32", "U", "revb:LSM6DSO32_LGA14",
        "https://www.st.com/resource/en/datasheet/lsm6dso32.pdf", LSM6DSO32_PINS,
    ),
    SymbolSpec(
        "CC1121", "U", "Package_DFN_QFN:Texas_RHB0032E_VQFN-32-1EP_5x5mm_P0.5mm_EP3.45x3.45mm_ThermalVias",
        "https://www.ti.com/lit/ds/symlink/cc1121.pdf", CC1121_PINS,
    ),
    SymbolSpec(
        "CC1190", "U", "Package_DFN_QFN:Texas_RGV0016A_VQFN-16-1EP_4x4mm_P0.65mm_EP2.1x2.1mm_ThermalVias",
        "https://www.ti.com/lit/ds/symlink/cc1190.pdf", CC1190_PINS,
    ),
    SymbolSpec(
        "LG77L", "U", "revb:LG77L_LGA43",
        "https://forums.quectel.com/uploads/short-url/9zAmjO9mANigfC05VclI7cICxed.pdf",
        LG77L_PINS,
    ),
    SymbolSpec(
        "AP2112K", "U", "Package_TO_SOT_SMD:SOT-23-5",
        "https://www.diodes.com/assets/Datasheets/AP2112.pdf",
        (("1", "VIN"), ("2", "GND"), ("3", "EN"), ("4", "NC"), ("5", "VOUT")),
    ),
    SymbolSpec(
        "TLV62569", "U", "Package_TO_SOT_SMD:SOT-23-5",
        "https://www.ti.com/lit/ds/symlink/tlv62569.pdf",
        (("1", "EN"), ("2", "GND"), ("3", "SW"), ("4", "VIN"), ("5", "FB")),
    ),
    SymbolSpec(
        "TLV755P", "U", "Package_TO_SOT_SMD:SOT-23-5",
        "https://www.ti.com/lit/ds/symlink/tlv755p.pdf",
        (("1", "IN"), ("2", "GND"), ("3", "EN"), ("4", "NC"), ("5", "OUT")),
    ),
    SymbolSpec(
        "USBLC6-2SC6", "U", "Package_TO_SOT_SMD:SOT-23-6",
        "https://www.st.com/resource/en/datasheet/usblc6-2.pdf",
        (("1", "IO1"), ("2", "GND"), ("3", "IO2"), ("4", "IO2"), ("5", "VBUS"), ("6", "IO1")),
    ),
    SymbolSpec(
        "SN74LVC1G08", "U", "Package_TO_SOT_SMD:SOT-23-5",
        "https://www.ti.com/lit/ds/symlink/sn74lvc1g08.pdf",
        (("1", "A"), ("2", "B"), ("3", "GND"), ("4", "Y"), ("5", "VCC")),
    ),
    SymbolSpec(
        "RF_FILTER_4PORT", "FL", "revb:RF_Filter_4Port",
        "TI BOOSTXL-CC1120-90 Rev A",
        (
            ("1", "GND"), ("2", "RF2"), ("3", "GND"),
            ("4", "GND"), ("5", "RF1"), ("6", "GND"),
        ),
    ),
    SymbolSpec(
        "LFD21868MMF1D386", "FL", "revb:LFD21868MMF1D386",
        "TI BOOSTXL-CC1120-90 Rev A",
        (
            ("1", "GND"), ("2", "ANT"), ("3", "NC"), ("4", "GND"),
            ("5", "GND"), ("6", "RX1LNA_N"), ("7", "RX2LNA_P"),
            ("8", "TRX"), ("9", "TXPA"), ("10", "GND"),
        ),
    ),
    SymbolSpec(
        "UFL", "J", "Connector_Coaxial:U.FL_Hirose_U.FL-R-SMT-1_Vertical",
        "https://www.hirose.com/product/document?clcode=&productname=&series=U.FL&documenttype=Catalog&lang=en",
        (("1", "SIGNAL"), ("2", "GND")),
    ),
)


def _q(text: str) -> str:
    return text.replace("\\", "\\\\").replace('"', '\\"')


def _symbol_text(spec: SymbolSpec) -> str:
    count = len(spec.pins)
    split = (count + 1) // 2
    height = max(12.7, (split - 1) * 2.54 + 5.08)
    half_h = height / 2
    body_half_w = 12.7
    lines = [
        f'  (symbol "{_q(spec.name)}"',
        "    (pin_names (offset 0.8))",
        "    (exclude_from_sim no)",
        "    (in_bom yes)",
        "    (on_board yes)",
        f'    (property "Reference" "{_q(spec.reference)}" (at 0 {half_h + 2.54:.3f} 0)',
        "      (effects (font (size 1.27 1.27))))",
        f'    (property "Value" "{_q(spec.name)}" (at 0 {half_h:.3f} 0)',
        "      (effects (font (size 1.27 1.27))))",
        f'    (property "Footprint" "{_q(spec.footprint)}" (at 0 0 0)',
        "      (effects (font (size 1.27 1.27)) hide))",
        f'    (property "Datasheet" "{_q(spec.datasheet)}" (at 0 0 0)',
        "      (effects (font (size 1.27 1.27)) hide))",
        f'    (symbol "{_q(spec.name)}_0_1"',
        f"      (rectangle (start {-body_half_w:.3f} {half_h:.3f})"
        f" (end {body_half_w:.3f} {-half_h:.3f})",
        "        (stroke (width 0.254) (type default)) (fill (type background))))",
        f'    (symbol "{_q(spec.name)}_1_1"',
    ]
    for index, (number, name) in enumerate(spec.pins):
        if index < split:
            x = -body_half_w - 5.08
            y = half_h - 2.54 - index * 2.54
            rotation = 0
        else:
            x = body_half_w + 5.08
            y = half_h - 2.54 - (index - split) * 2.54
            rotation = 180
        lines.extend(
            (
                f"      (pin passive line (at {x:.3f} {y:.3f} {rotation}) (length 5.08)",
                f'        (name "{_q(name)}" (effects (font (size 1.0 1.0))))',
                f'        (number "{_q(number)}" (effects (font (size 1.0 1.0)))))',
            )
        )
    lines.extend(("    )", "  )"))
    return "\n".join(lines)


def write_symbol_library(directory: Path) -> Path:
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / "revb.kicad_sym"
    text = (
        "(kicad_symbol_lib (version 20231120) (generator codex)\n"
        + "\n".join(_symbol_text(spec) for spec in SYMBOLS)
        + "\n)\n"
    )
    path.write_text(text, encoding="utf-8")
    return path


def write_tables(directory: Path) -> None:
    (directory / "sym-lib-table").write_text(
        '(sym_lib_table\n  (lib (name "revb")(type "KiCad")'
        '(uri "${KIPRJMOD}/revb.kicad_sym")(options "")(descr "Rev-B audited symbols"))\n)\n',
        encoding="utf-8",
    )
    (directory / "fp-lib-table").write_text(
        '(fp_lib_table\n  (lib (name "revb")(type "KiCad")'
        '(uri "${KIPRJMOD}/revb.pretty")(options "")(descr "Rev-B audited footprints"))\n)\n',
        encoding="utf-8",
    )


def add_part(
    sch: ksa.Schematic,
    lib_id: str,
    ref: str,
    value: str,
    position: tuple[float, float],
    footprint: str | None = None,
    manufacturer: str = "",
    mpn: str = "",
):
    return sch.components.add(
        lib_id=lib_id,
        reference=ref,
        value=value,
        position=position,
        footprint=footprint,
        Manufacturer=manufacturer,
        MPN=mpn,
    )


def label_pin(sch: ksa.Schematic, ref: str, pin: str, net: str) -> None:
    sch.add_label(text=net, pin=(ref, str(pin)), size=0.9)


def no_connect_pin(sch: ksa.Schematic, ref: str, pin: str) -> None:
    position = sch.get_component_pin_position(ref, str(pin))
    if position is None:
        raise ValueError(f"Cannot find {ref} pin {pin} for no-connect marker")
    sch.no_connects.add(position=position)


def connect_pins(
    sch: ksa.Schematic, ref: str, pin_nets: dict[str, str | None], all_pins: tuple[tuple[str, str], ...]
) -> None:
    for pin, _name in all_pins:
        net = pin_nets.get(pin)
        if net is None:
            no_connect_pin(sch, ref, pin)
        else:
            label_pin(sch, ref, pin, net)


def connect_rp2354(
    sch: ksa.Schematic,
    ref: str,
    gpio_nets: dict[int, str],
    power_map: dict[str, str],
) -> None:
    for pin_number, pin_name in RP2354B_PINS:
        if pin_name.startswith("GPIO"):
            match = re.match(r"GPIO(\d+)", pin_name)
            net = gpio_nets.get(int(match.group(1))) if match else None
        else:
            net = power_map.get(pin_name)
        if net is None:
            no_connect_pin(sch, ref, pin_number)
        else:
            label_pin(sch, ref, pin_number, net)


def add_two_pin(
    sch: ksa.Schematic,
    lib_id: str,
    ref: str,
    value: str,
    position: tuple[float, float],
    net1: str,
    net2: str,
    footprint: str,
    manufacturer: str = "",
    mpn: str = "",
) -> None:
    add_part(sch, lib_id, ref, value, position, footprint, manufacturer, mpn)
    label_pin(sch, ref, "1", net1)
    label_pin(sch, ref, "2", net2)


def add_custom(
    sch: ksa.Schematic,
    name: str,
    ref: str,
    value: str,
    position: tuple[float, float],
    pin_nets: dict[str, str | None],
    manufacturer: str,
    mpn: str,
) -> None:
    spec = next(item for item in SYMBOLS if item.name == name)
    add_part(sch, f"revb:{name}", ref, value, position, spec.footprint, manufacturer, mpn)
    connect_pins(sch, ref, pin_nets, spec.pins)


def add_hlabels(sch: ksa.Schematic, nets: list[str]) -> None:
    for index, net in enumerate(nets):
        position = (20.32, 20.32 + index * 2.54)
        stub_end = (25.40, position[1])
        sch.add_hierarchical_label(net, position, shape="bidirectional", size=0.9)
        sch.add_wire(position, stub_end, grid_units=False)
        sch.add_label(net, stub_end, size=0.8, grid_units=False)


def add_sheet_with_pins(
    top: ksa.Schematic,
    name: str,
    filename: str,
    position: tuple[float, float],
    size: tuple[float, float],
    nets: list[str],
    project: str,
) -> str:
    snap = lambda value: round(value / 2.54) * 2.54
    position = (snap(position[0]), snap(position[1]))
    size = (snap(size[0]), snap(size[1]))
    sheet_uuid = top.add_sheet(name, filename, position, size, project_name=project)
    left = []
    right = []
    for index, net in enumerate(nets):
        (left if index % 2 == 0 else right).append(net)
    for index, net in enumerate(left):
        offset = 2.54 + index * 2.54
        top.add_sheet_pin(sheet_uuid, net, "bidirectional", "left", size[1] - offset)
        pin_position = (position[0], position[1] + offset)
        stub_end = (pin_position[0] - 2.54, pin_position[1])
        top.add_wire(pin_position, stub_end, grid_units=False)
        top.add_label(net, stub_end, size=0.8, grid_units=False)
    for index, net in enumerate(right):
        offset = 2.54 + index * 2.54
        top.add_sheet_pin(sheet_uuid, net, "bidirectional", "right", offset)
        pin_position = (position[0] + size[0], position[1] + offset)
        stub_end = (pin_position[0] + 2.54, pin_position[1])
        top.add_wire(pin_position, stub_end, grid_units=False)
        top.add_label(net, stub_end, size=0.8, grid_units=False)
    return sheet_uuid


def add_power_flag(sch: ksa.Schematic, ref: str, net: str, position: tuple[float, float]) -> None:
    add_part(sch, "power:PWR_FLAG", ref, "PWR_FLAG", position)
    label_pin(sch, ref, "1", net)


def assert_no_label_collisions(sch: ksa.Schematic, context: str) -> None:
    """Reject unlike net labels placed on the same electrical coordinate."""
    labels_by_position: dict[tuple[float, float], set[str]] = {}
    for label in sch.labels:
        point = (round(label.position.x, 3), round(label.position.y, 3))
        labels_by_position.setdefault(point, set()).add(label.text)
    collisions = {
        point: sorted(names)
        for point, names in labels_by_position.items()
        if len(names) > 1
    }
    if collisions:
        details = ", ".join(f"{point}: {names}" for point, names in collisions.items())
        raise RuntimeError(f"{context}: unlike net labels collide: {details}")


def _make_motor_sheet_legacy(index: int, parent_uuid: str, sheet_uuid: str) -> None:
    n = index
    # Motor-cell references occupy dedicated 11xx..16xx blocks so the six
    # replicated sheets cannot collide with controller (2xx) or PSU (7xx)
    # annotations in the flattened PCB netlist.
    base = 1000 + n * 100
    sch = ksa.create_schematic("esc_rev_b")
    sch.set_hierarchy_context(parent_uuid, sheet_uuid)
    interface = [
        f"M{n}_BATP", f"M{n}_BATN", f"M{n}_PHASE_A", f"M{n}_PHASE_B", f"M{n}_PHASE_C",
        f"M{n}_PWM_A", f"M{n}_PWM_B", f"M{n}_PWM_C", "DRV_SCLK", "DRV_SDI", "DRV_SDO",
        f"M{n}_nCS", "DRV_ENABLE", "DRV_3X_EN", "DRV_nFAULT", f"M{n}_CSA_A", f"M{n}_CSA_B",
        f"M{n}_NTC", f"M{n}_BUS", "3V3", "DGND",
    ]
    add_hlabels(sch, interface)
    sch.add_text(
        f"MOTOR {n}: 12S / 60 A continuous design cell\n"
        "IPTC014N10NM5 TOLT switches require isolated top-side heat spreader.\n"
        "BATP/BATN arrive on their own fused busbar tap; no 360 A PCB trunk.",
        (80, 17), size=1.1, bold=True,
    )

    driver_ref = f"U{base + 1}"
    driver_nets = {
        "1": f"M{n}_CPL", "2": f"M{n}_CPH", "3": f"M{n}_BATP",
        "4": f"M{n}_BATP", "5": f"M{n}_VCP",
        "6": f"M{n}_GHA_DRV", "7": f"M{n}_PHASE_A", "8": f"M{n}_GLA_DRV",
        "9": f"M{n}_SHA_P", "10": f"M{n}_SHA_N",
        "11": f"M{n}_SHB_N", "12": f"M{n}_SHB_P",
        "13": f"M{n}_GLB_DRV", "14": f"M{n}_PHASE_B", "15": f"M{n}_GHB_DRV",
        "16": f"M{n}_GHC_DRV", "17": f"M{n}_PHASE_C", "18": f"M{n}_GLC_DRV",
        "19": f"M{n}_BATN", "20": f"M{n}_BATN", "21": None,
        "22": f"M{n}_CSA_B_RAW", "23": f"M{n}_CSA_A_RAW", "24": "3V3",
        "25": f"M{n}_BATN", "26": "DRV_nFAULT", "27": "DRV_SDO",
        "28": "DRV_SDI", "29": "DRV_SCLK", "30": f"M{n}_nCS",
        "31": "DRV_ENABLE", "32": f"M{n}_PWM_A", "33": "DRV_3X_EN",
        "34": f"M{n}_PWM_B", "35": "DRV_3X_EN", "36": f"M{n}_PWM_C",
        "37": "DRV_3X_EN", "38": f"M{n}_DVDD", "39": f"M{n}_BATN",
        "40": f"M{n}_VGLS", "41": f"M{n}_BATN",
    }
    add_custom(sch, "DRV8353S", driver_ref, "DRV8353SRTAR", (90, 72), driver_nets, "Texas Instruments", "DRV8353SRTAR")

    # Charge-pump and local driver bypass network from the DRV8353 datasheet.
    cap_index = base + 1
    caps = [
        ("47n 100V", f"M{n}_CPH", f"M{n}_CPL", "Capacitor_SMD:C_0603_1608Metric"),
        ("1u 16V", f"M{n}_VCP", f"M{n}_BATP", "Capacitor_SMD:C_0603_1608Metric"),
        ("1u 16V", f"M{n}_VGLS", f"M{n}_BATN", "Capacitor_SMD:C_0603_1608Metric"),
        ("1u 10V", f"M{n}_DVDD", f"M{n}_BATN", "Capacitor_SMD:C_0603_1608Metric"),
        ("100n 100V", f"M{n}_BATP", f"M{n}_BATN", "Capacitor_SMD:C_0603_1608Metric"),
        ("10u 100V", f"M{n}_BATP", f"M{n}_BATN", "Capacitor_SMD:C_1210_3225Metric"),
        ("100n 10V", "3V3", f"M{n}_BATN", "Capacitor_SMD:C_0603_1608Metric"),
    ]
    for value, net1, net2, footprint in caps:
        add_two_pin(sch, "Device:C", f"C{cap_index}", value, (145, 32 + (cap_index - base) * 5),
                    net1, net2, footprint)
        cap_index += 1

    phases = ("A", "B", "C")
    for phase_i, phase in enumerate(phases):
        x = 160 + phase_i * 38
        high_ref = f"Q{base + phase_i * 2 + 1}"
        low_ref = f"Q{base + phase_i * 2 + 2}"
        gate_h = f"M{n}_GH{phase}"
        gate_l = f"M{n}_GL{phase}"
        phase_net = f"M{n}_PHASE_{phase}"
        low_source = f"M{n}_SH{phase}_I" if phase in ("A", "B") else f"M{n}_BATN"
        add_custom(
            sch, "IPTC014N10NM5", high_ref, "IPTC014N10NM5", (x, 70),
            {"1": phase_net, "8": gate_h, "9": f"M{n}_BATP"}, "Infineon", "IPTC014N10NM5ATMA1",
        )
        add_custom(
            sch, "IPTC014N10NM5", low_ref, "IPTC014N10NM5", (x, 105),
            {"1": low_source, "8": gate_l, "9": phase_net}, "Infineon", "IPTC014N10NM5ATMA1",
        )
        add_two_pin(sch, "Device:R", f"R{base + phase_i * 4 + 1}", "2.2",
                    (x - 13, 63), f"M{n}_GH{phase}_DRV", gate_h,
                    "Resistor_SMD:R_0603_1608Metric")
        add_two_pin(sch, "Device:R", f"R{base + phase_i * 4 + 2}", "100k",
                    (x - 13, 72), gate_h, phase_net,
                    "Resistor_SMD:R_0603_1608Metric")
        add_two_pin(sch, "Device:R", f"R{base + phase_i * 4 + 3}", "2.2",
                    (x - 13, 98), f"M{n}_GL{phase}_DRV", gate_l,
                    "Resistor_SMD:R_0603_1608Metric")
        add_two_pin(sch, "Device:R", f"R{base + phase_i * 4 + 4}", "100k",
                    (x - 13, 107), gate_l, low_source,
                    "Resistor_SMD:R_0603_1608Metric")

    for shunt_i, phase in enumerate(("A", "B")):
        ref = f"R{base + 20 + shunt_i}"
        add_custom(
            sch, "CSS4J_4026R_L500F", ref, "0.5m 1% 5W", (166 + shunt_i * 42, 135),
            {
                "1": f"M{n}_SH{phase}_I", "2": f"M{n}_BATN",
                "3": f"M{n}_SH{phase}_P", "4": f"M{n}_SH{phase}_N",
            },
            "Bourns", "CSS4J-4026R-L500F",
        )

    # CSA anti-alias filters: the capacitor is on the ADC side of the series resistor.
    for sense_i, phase in enumerate(("A", "B")):
        out_net = f"M{n}_CSA_{phase}"
        raw_net = f"M{n}_CSA_{phase}_RAW"
        add_two_pin(sch, "Device:R", f"R{base + 30 + sense_i}", "47",
                    (112, 128 + sense_i * 12), raw_net, out_net,
                    "Resistor_SMD:R_0603_1608Metric")
        add_two_pin(sch, "Device:C", f"C{base + 20 + sense_i}", "10n C0G",
                    (130, 128 + sense_i * 12), out_net, f"M{n}_BATN",
                    "Capacitor_SMD:C_0603_1608Metric")

    # Local DC link and transient suppression.
    add_two_pin(
        sch, "Device:C_Polarized", f"C{base + 30}", "820u 100V",
        (238, 58), f"M{n}_BATP", f"M{n}_BATN",
        "Capacitor_THT:CP_Radial_D18.0mm_P7.50mm", "Rubycon", "100PX820MEFC18X35.5",
    )
    for k in range(6):
        add_two_pin(
            sch, "Device:C", f"C{base + 31 + k}", "10u 100V",
            (238, 72 + k * 10), f"M{n}_BATP", f"M{n}_BATN",
            "Capacitor_SMD:C_1210_3225Metric",
        )
    add_two_pin(
        sch, "Device:D_TVS", f"D{base + 1}", "SM8S51A",
        (238, 123), f"M{n}_BATN", f"M{n}_BATP",
        "revb:DO-218AB_SM8S", "Littelfuse", "SM8S51A",
    )

    # 50.4 V bus monitor: 600k/33k gives 2.63 V at full charge.
    previous = f"M{n}_BATP"
    for k in range(4):
        next_net = f"M{n}_BUS_DIV_{k}" if k < 3 else f"M{n}_BUS"
        add_two_pin(sch, "Device:R", f"R{base + 40 + k}", "150k 0.1%",
                    (70 + k * 18, 160), previous, next_net,
                    "Resistor_SMD:R_1206_3216Metric")
        previous = next_net
    add_two_pin(sch, "Device:R", f"R{base + 44}", "33k 0.1%",
                (145, 160), f"M{n}_BUS", f"M{n}_BATN",
                "Resistor_SMD:R_0603_1608Metric")
    add_two_pin(sch, "Device:C", f"C{base + 50}", "10n 50V",
                (162, 160), f"M{n}_BUS", f"M{n}_BATN",
                "Capacitor_SMD:C_0603_1608Metric")

    # Thermal monitor and logic/power star.  RSTAR is a deliberate Kelvin-only
    # connection; load current returns through the external BATN terminal.
    add_two_pin(sch, "Device:Thermistor", f"TH{n}", "NTCG163JF103FT1",
                (192, 160), "3V3", f"M{n}_NTC",
                "Resistor_SMD:R_0603_1608Metric", "TDK", "NTCG163JF103FT1")
    add_two_pin(sch, "Device:R", f"R{base + 45}", "10k 0.1%",
                (212, 160), f"M{n}_NTC", f"M{n}_BATN",
                "Resistor_SMD:R_0603_1608Metric")
    add_two_pin(sch, "Device:R", f"R{base + 46}", "0 DNP/STAR",
                (232, 160), "DGND", f"M{n}_BATN",
                "Resistor_SMD:R_1206_3216Metric")

    add_part(
        sch, "Connector_Generic:Conn_01x02", f"J{base + 1}", "FUSED_BATTERY_TAP",
        (45, 70), "revb:POWER_LUG_M5_2P_P20",
        "Generic", "M5_RING_LUG_PAIR",
    )
    label_pin(sch, f"J{base + 1}", "1", f"M{n}_BATP")
    label_pin(sch, f"J{base + 1}", "2", f"M{n}_BATN")
    add_two_pin(sch, "Device:R", f"R{base + 47}", "10k", (52, 82),
                "3V3", f"M{n}_nCS", "Resistor_SMD:R_0603_1608Metric")
    add_part(
        sch, "Connector_Generic:Conn_01x03", f"J{base + 2}", "MOTOR_PHASES",
        (45, 105), "revb:POWER_LUG_M5_3P_P20",
        "Generic", "M5_RING_LUG_TRIPLE",
    )
    for pin, phase in zip(("1", "2", "3"), phases):
        label_pin(sch, f"J{base + 2}", pin, f"M{n}_PHASE_{phase}")

    assert_no_label_collisions(sch, f"ESC motor {n}")
    sch.save(ESC_DIR / f"motor_{n}.kicad_sch")


def make_motor_sheet(index: int, parent_uuid: str, sheet_uuid: str) -> None:
    """Create one electrically isolated 60 A motor-control/power cell."""
    n = index
    base = 1000 + n * 100
    sch = ksa.create_schematic("esc_rev_b")
    sch.set_hierarchy_context(parent_uuid, sheet_uuid)
    interface = [
        f"M{n}_BATP", f"M{n}_BATN", f"M{n}_PHASE_A", f"M{n}_PHASE_B", f"M{n}_PHASE_C",
        f"M{n}_CMD", f"M{n}_STATUS", "ARM_SAFE", "5V", "3V3", "DGND",
    ]
    add_hlabels(sch, interface)
    sch.add_text(
        f"MOTOR {n}: 12S / 60 A CONTINUOUS CELL\n"
        "Local STM32G431: 3 leg shunts + DC-input shunt, sampled every PWM cycle.\n"
        "Command/arm/status are isolated; DGND never bonds to this cell's BATN.",
        (80, 17), size=1.1, bold=True,
    )

    batn = f"M{n}_BATN"
    local_3v3 = f"M{n}_3V3I"
    local_3v3a = f"M{n}_3V3A"
    local_5v = f"M{n}_5VI"

    # Functional-isolation supply.  The unregulated module feeds a 30 V-input
    # 3.3 V LDO so its light-load tolerance cannot over-volt local logic.
    add_custom(
        sch, "RFM_0505S", f"PS{base + 1}", "RFM-0505S", (48, 190),
        {"1": "DGND", "2": "5V", "3": batn, "4": local_5v},
        "RECOM", "RFM-0505S",
    )
    add_custom(
        sch, "TPS70933", f"U{base + 4}", "TPS70933DBVR", (78, 190),
        {
            "1": local_5v, "2": batn, "3": f"M{n}_LDO_EN",
            "4": None, "5": local_3v3,
        },
        "Texas Instruments", "TPS70933DBVR",
    )
    # RFM is an unregulated converter.  Its specified load-regulation envelope
    # is safe for TPS709 IN, but TPS709 EN is only rated to 6.5 V.  Divide the
    # raw isolated output so an unloaded/start-up overshoot cannot stress EN.
    add_two_pin(
        sch, "Device:R", f"R{base + 58}", "100k",
        (78, 176), local_5v, f"M{n}_LDO_EN",
        "Resistor_SMD:R_0603_1608Metric",
    )
    add_two_pin(
        sch, "Device:R", f"R{base + 59}", "68k",
        (94, 176), f"M{n}_LDO_EN", batn,
        "Resistor_SMD:R_0603_1608Metric",
    )
    # The isolated converter and LDO custom symbols intentionally use passive
    # pins.  Explicit flags document the three locally generated supply rails
    # and let ERC verify the MCU power pins without weakening global rules.
    add_power_flag(sch, f"#FLG{base + 1}", batn, (28, 180))
    add_power_flag(sch, f"#FLG{base + 2}", local_3v3, (34, 180))
    add_power_flag(sch, f"#FLG{base + 3}", local_3v3a, (40, 180))
    for ref, value, rail, ground, pos, fp in (
        (base + 60, "10u 10V", "5V", "DGND", (45, 205), "Capacitor_SMD:C_0805_2012Metric"),
        (base + 61, "100n", local_5v, batn, (60, 205), "Capacitor_SMD:C_0603_1608Metric"),
        (base + 62, "2.2u 10V", local_5v, batn, (75, 205), "Capacitor_SMD:C_0603_1608Metric"),
        (base + 63, "2.2u 10V", local_3v3, batn, (90, 205), "Capacitor_SMD:C_0603_1608Metric"),
    ):
        add_two_pin(sch, "Device:C", f"C{ref}", value, pos, rail, ground, fp)

    # Two forward channels carry command and independent ARM permission; the
    # reverse channel returns status.  The F suffix guarantees fail-low.
    iso_ref = f"U{base + 3}"
    add_part(
        sch, "Isolator:ISO6731", iso_ref, "ISO6731FDWR", (55, 235),
        "Package_SO:SOIC-16W_7.5x10.3mm_P1.27mm",
        "Texas Instruments", "ISO6731FDWR",
    )
    iso_nets = {
        "1": "3V3", "2": "DGND", "3": f"M{n}_CMD", "4": "ARM_SAFE",
        "5": f"M{n}_STATUS", "6": None, "7": "3V3", "8": "DGND",
        "9": batn, "10": local_3v3, "11": None,
        "12": f"M{n}_UART_TX", "13": f"M{n}_ARM_ISO",
        "14": f"M{n}_UART_RX", "15": batn, "16": local_3v3,
    }
    iso_info = ksa.get_symbol_info("Isolator:ISO6731")
    assert iso_info is not None
    connect_pins(sch, iso_ref, iso_nets, tuple((pin.number, pin.name) for pin in iso_info.pins))
    add_two_pin(sch, "Device:C", f"C{base + 64}", "100n", (80, 225),
                "3V3", "DGND", "Capacitor_SMD:C_0603_1608Metric")
    add_two_pin(sch, "Device:C", f"C{base + 65}", "100n", (80, 240),
                local_3v3, batn, "Capacitor_SMD:C_0603_1608Metric")

    # TIM1 provides six complementary PWM outputs with hardware dead time.
    # PA11 is TIM1_BKIN2, so a DRV fault can stop PWM without firmware.
    mcu_ref = f"U{base + 2}"
    add_part(
        sch, "MCU_ST_STM32G4:STM32G431CBTx", mcu_ref, "STM32G431CBT6",
        (120, 220), "Package_QFP:LQFP-48_7x7mm_P0.5mm",
        "STMicroelectronics", "STM32G431CBT6",
    )
    mcu_nets = {
        "1": local_3v3, "2": None, "3": None, "4": None,
        "5": f"M{n}_OSC_IN", "6": f"M{n}_OSC_OUT", "7": f"M{n}_NRST",
        "8": f"M{n}_CSA_A", "9": f"M{n}_CSA_B", "10": f"M{n}_CSA_C",
        "11": f"M{n}_BUS_CURRENT", "12": f"M{n}_BUS",
        "13": f"M{n}_DRV_SCLK", "14": f"M{n}_DRV_SDO", "15": f"M{n}_DRV_SDI",
        "16": f"M{n}_NTC", "17": None, "18": None,
        "19": batn, "20": local_3v3a, "21": local_3v3a,
        "22": None, "23": batn, "24": local_3v3,
        "25": None, "26": f"M{n}_DRV_nCS",
        "27": f"M{n}_PWM_AL", "28": f"M{n}_PWM_BL", "29": f"M{n}_PWM_CL",
        "30": f"M{n}_PWM_AH", "31": f"M{n}_PWM_BH", "32": f"M{n}_PWM_CH",
        "33": f"M{n}_DRV_nFAULT", "34": f"M{n}_ARM_ISO",
        "35": batn, "36": local_3v3,
        "37": f"M{n}_SWDIO", "38": f"M{n}_SWCLK", "39": None,
        "40": None, "41": f"M{n}_MCU_ARM", "42": f"M{n}_STATUS_LED",
        "43": f"M{n}_UART_TX", "44": f"M{n}_UART_RX",
        "45": f"M{n}_BOOT0", "46": None, "47": batn, "48": local_3v3,
    }
    stm_info = ksa.get_symbol_info("MCU_ST_STM32G4:STM32G431CBTx")
    assert stm_info is not None
    connect_pins(sch, mcu_ref, mcu_nets, tuple((pin.number, pin.name) for pin in stm_info.pins))
    add_two_pin(sch, "Device:FerriteBead", f"FB{base + 1}", "600R@100MHz",
                (145, 188), local_3v3, local_3v3a,
                "Inductor_SMD:L_0603_1608Metric", "Murata", "BLM18AG601SN1D")
    for offset, (value, rail) in enumerate((
        ("4.7u", local_3v3), ("100n", local_3v3), ("100n", local_3v3),
        ("100n", local_3v3), ("100n", local_3v3), ("1u", local_3v3a),
        ("100n", local_3v3a),
    )):
        add_two_pin(
            sch, "Device:C", f"C{base + 66 + offset}", value,
            (158 + (offset % 4) * 16, 188 + (offset // 4) * 12),
            rail, batn,
            "Capacitor_SMD:C_0603_1608Metric" if value != "100n" else "Capacitor_SMD:C_0402_1005Metric",
        )
    add_part(
        sch, "Device:Crystal", f"Y{base + 1}", "8MHz ABM8G", (170, 225),
        "Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm",
        "Abracon", "ABM8G-8.000MHZ-18-D2Y-T",
    )
    label_pin(sch, f"Y{base + 1}", "1", f"M{n}_OSC_IN")
    label_pin(sch, f"Y{base + 1}", "2", f"M{n}_OSC_OUT")
    add_two_pin(sch, "Device:C", f"C{base + 73}", "12p C0G", (192, 220),
                f"M{n}_OSC_IN", batn, "Capacitor_SMD:C_0402_1005Metric")
    add_two_pin(sch, "Device:C", f"C{base + 74}", "12p C0G", (192, 232),
                f"M{n}_OSC_OUT", batn, "Capacitor_SMD:C_0402_1005Metric")
    add_two_pin(sch, "Device:R", f"R{base + 53}", "10k", (210, 210),
                local_3v3, f"M{n}_NRST", "Resistor_SMD:R_0603_1608Metric")
    add_two_pin(sch, "Device:C", f"C{base + 75}", "100n", (225, 210),
                f"M{n}_NRST", batn, "Capacitor_SMD:C_0603_1608Metric")
    add_two_pin(sch, "Device:R", f"R{base + 54}", "100k", (210, 222),
                f"M{n}_BOOT0", batn, "Resistor_SMD:R_0603_1608Metric")
    add_two_pin(sch, "Device:R", f"R{base + 55}", "4.7k", (225, 222),
                local_3v3, f"M{n}_DRV_nFAULT", "Resistor_SMD:R_0603_1608Metric")
    add_two_pin(sch, "Device:R", f"R{base + 56}", "10k", (240, 222),
                local_3v3, f"M{n}_DRV_nCS", "Resistor_SMD:R_0603_1608Metric")
    add_part(
        sch, "Connector_Generic:Conn_01x05", f"J{base + 3}", "LOCAL_SWD",
        (255, 230), "Connector_PinHeader_1.27mm:PinHeader_1x05_P1.27mm_Vertical",
    )
    for pin, net in {
        "1": local_3v3, "2": f"M{n}_SWDIO", "3": f"M{n}_SWCLK",
        "4": f"M{n}_NRST", "5": batn,
    }.items():
        label_pin(sch, f"J{base + 3}", pin, net)

    # Both the isolated central watchdog and the local MCU must grant ARM.
    add_custom(
        sch, "SN74LVC1G08", f"U{base + 6}", "SN74LVC1G08DBVR", (235, 245),
        {
            "1": f"M{n}_ARM_ISO", "2": f"M{n}_MCU_ARM",
            "3": batn, "4": f"M{n}_DRV_ENABLE", "5": local_3v3,
        },
        "Texas Instruments", "SN74LVC1G08DBVR",
    )
    for offset, net in enumerate((f"M{n}_ARM_ISO", f"M{n}_MCU_ARM", f"M{n}_DRV_ENABLE")):
        add_two_pin(sch, "Device:R", f"R{base + 50 + offset}", "100k",
                    (205 + offset * 17, 245), net, batn,
                    "Resistor_SMD:R_0603_1608Metric")
    add_two_pin(sch, "Device:C", f"C{base + 77}", "100n", (260, 245),
                local_3v3, batn, "Capacitor_SMD:C_0603_1608Metric")

    driver_ref = f"U{base + 1}"
    driver_nets = {
        "1": f"M{n}_CPL", "2": f"M{n}_CPH", "3": f"M{n}_BATP",
        "4": f"M{n}_BATP", "5": f"M{n}_VCP",
        "6": f"M{n}_GHA_DRV", "7": f"M{n}_PHASE_A", "8": f"M{n}_GLA_DRV",
        "9": f"M{n}_SHA_P", "10": f"M{n}_SHA_N",
        "11": f"M{n}_SHB_N", "12": f"M{n}_SHB_P",
        "13": f"M{n}_GLB_DRV", "14": f"M{n}_PHASE_B", "15": f"M{n}_GHB_DRV",
        "16": f"M{n}_GHC_DRV", "17": f"M{n}_PHASE_C", "18": f"M{n}_GLC_DRV",
        "19": f"M{n}_SHC_P", "20": f"M{n}_SHC_N",
        "21": f"M{n}_CSA_C_RAW", "22": f"M{n}_CSA_B_RAW", "23": f"M{n}_CSA_A_RAW",
        "24": local_3v3a, "25": batn, "26": f"M{n}_DRV_nFAULT",
        "27": f"M{n}_DRV_SDO", "28": f"M{n}_DRV_SDI", "29": f"M{n}_DRV_SCLK",
        "30": f"M{n}_DRV_nCS", "31": f"M{n}_DRV_ENABLE",
        "32": f"M{n}_PWM_AH", "33": f"M{n}_PWM_AL",
        "34": f"M{n}_PWM_BH", "35": f"M{n}_PWM_BL",
        "36": f"M{n}_PWM_CH", "37": f"M{n}_PWM_CL",
        "38": f"M{n}_DVDD", "39": batn, "40": f"M{n}_VGLS", "41": batn,
    }
    add_custom(
        sch, "DRV8353S", driver_ref, "DRV8353SRTAR", (90, 72),
        driver_nets, "Texas Instruments", "DRV8353SRTAR",
    )

    caps = [
        ("47n 100V", f"M{n}_CPH", f"M{n}_CPL", "Capacitor_SMD:C_0603_1608Metric"),
        ("1u 16V", f"M{n}_VCP", f"M{n}_BATP", "Capacitor_SMD:C_0603_1608Metric"),
        ("1u 16V", f"M{n}_VGLS", batn, "Capacitor_SMD:C_0603_1608Metric"),
        ("1u 10V", f"M{n}_DVDD", batn, "Capacitor_SMD:C_0603_1608Metric"),
        ("100n 100V", f"M{n}_BATP", batn, "Capacitor_SMD:C_0603_1608Metric"),
        ("10u 100V", f"M{n}_BATP", batn, "Capacitor_SMD:C_1210_3225Metric"),
        ("100n 10V", local_3v3a, batn, "Capacitor_SMD:C_0603_1608Metric"),
    ]
    for offset, (value, net1, net2, footprint) in enumerate(caps, 1):
        add_two_pin(sch, "Device:C", f"C{base + offset}", value,
                    (145, 32 + offset * 5), net1, net2, footprint)

    phases = ("A", "B", "C")
    for phase_i, phase in enumerate(phases):
        x = 160 + phase_i * 38
        gate_h = f"M{n}_GH{phase}"
        gate_l = f"M{n}_GL{phase}"
        phase_net = f"M{n}_PHASE_{phase}"
        low_source = f"M{n}_SH{phase}_I"
        add_custom(
            sch, "IPTC014N10NM5", f"Q{base + phase_i * 2 + 1}", "IPTC014N10NM5", (x, 70),
            {"1": phase_net, "8": gate_h, "9": f"M{n}_BATP"},
            "Infineon", "IPTC014N10NM5ATMA1",
        )
        add_custom(
            sch, "IPTC014N10NM5", f"Q{base + phase_i * 2 + 2}", "IPTC014N10NM5", (x, 105),
            {"1": low_source, "8": gate_l, "9": phase_net},
            "Infineon", "IPTC014N10NM5ATMA1",
        )
        add_two_pin(sch, "Device:R", f"R{base + phase_i * 4 + 1}", "2.2",
                    (x - 13, 63), f"M{n}_GH{phase}_DRV", gate_h,
                    "Resistor_SMD:R_0603_1608Metric")
        add_two_pin(sch, "Device:R", f"R{base + phase_i * 4 + 2}", "100k",
                    (x - 13, 72), gate_h, phase_net,
                    "Resistor_SMD:R_0603_1608Metric")
        add_two_pin(sch, "Device:R", f"R{base + phase_i * 4 + 3}", "2.2",
                    (x - 13, 98), f"M{n}_GL{phase}_DRV", gate_l,
                    "Resistor_SMD:R_0603_1608Metric")
        add_two_pin(sch, "Device:R", f"R{base + phase_i * 4 + 4}", "100k",
                    (x - 13, 107), gate_l, low_source,
                    "Resistor_SMD:R_0603_1608Metric")

    # All three phase-leg currents are directly measured.
    for shunt_i, phase in enumerate(phases):
        add_custom(
            sch, "CSS4J_4026R_L500F", f"R{base + 20 + shunt_i}",
            "0.5m 1% 5W", (154 + shunt_i * 38, 135),
            {
                "1": f"M{n}_SH{phase}_I", "2": batn,
                "3": f"M{n}_SH{phase}_P", "4": f"M{n}_SH{phase}_N",
            },
            "Bourns", "CSS4J-4026R-L500F",
        )

    # Separate high-side DC-input current monitor for per-motor power and
    # regenerative-current telemetry.
    add_custom(
        sch, "CSS4J_4026R_L500F", f"R{base + 23}", "0.5m 1% 5W", (32, 35),
        {
            "1": f"M{n}_BATP_IN", "2": f"M{n}_BATP",
            "3": f"M{n}_BUS_SH_P", "4": f"M{n}_BUS_SH_N",
        },
        "Bourns", "CSS4J-4026R-L500F",
    )
    add_custom(
        sch, "INA296A", f"U{base + 5}", "INA296A2IDR", (115, 145),
        {
            "1": f"M{n}_BUS_SH_N", "2": batn, "3": local_3v3a, "4": batn,
            "5": f"M{n}_BUS_CURRENT_RAW", "6": local_3v3a,
            "7": batn, "8": f"M{n}_BUS_SH_P",
        },
        "Texas Instruments", "INA296A2IDR",
    )
    add_two_pin(sch, "Device:C", f"C{base + 76}", "100n", (137, 145),
                local_3v3a, batn, "Capacitor_SMD:C_0603_1608Metric")

    # 338 kHz output filters retain fast control bandwidth and attenuate PWM
    # edges before the dual 4 MSPS ADCs.
    for sense_i, name in enumerate(("CSA_A", "CSA_B", "CSA_C", "BUS_CURRENT")):
        out_net = f"M{n}_{name}"
        raw_net = f"M{n}_{name}_RAW"
        add_two_pin(sch, "Device:R", f"R{base + 30 + sense_i}", "47",
                    (105, 160 + sense_i * 10), raw_net, out_net,
                    "Resistor_SMD:R_0603_1608Metric")
        add_two_pin(sch, "Device:C", f"C{base + 20 + sense_i}", "10n C0G",
                    (125, 160 + sense_i * 10), out_net, batn,
                    "Capacitor_SMD:C_0603_1608Metric")

    # Local DC-link capacitance and clamping; the commutation loop stays inside
    # this cell and never traverses the external distribution harness.
    add_two_pin(sch, "Device:C_Polarized", f"C{base + 30}", "820u 100V",
                (238, 58), f"M{n}_BATP", batn,
                "Capacitor_THT:CP_Radial_D18.0mm_P7.50mm",
                "Rubycon", "100PX820MEFC18X35.5")
    for k in range(6):
        add_two_pin(sch, "Device:C", f"C{base + 31 + k}", "10u 100V",
                    (238, 72 + k * 10), f"M{n}_BATP", batn,
                    "Capacitor_SMD:C_1210_3225Metric")
    add_two_pin(sch, "Device:D_TVS", f"D{base + 1}", "SM8S51A",
                (238, 123), batn, f"M{n}_BATP",
                "revb:DO-218AB_SM8S", "Littelfuse", "SM8S51A")

    # 600k/33k bus divider gives 2.63 V at a 50.4 V fully charged pack.
    previous = f"M{n}_BATP"
    for k in range(4):
        next_net = f"M{n}_BUS_DIV_{k}" if k < 3 else f"M{n}_BUS"
        add_two_pin(sch, "Device:R", f"R{base + 40 + k}", "150k 0.1%",
                    (70 + k * 18, 155), previous, next_net,
                    "Resistor_SMD:R_1206_3216Metric")
        previous = next_net
    add_two_pin(sch, "Device:R", f"R{base + 44}", "33k 0.1%",
                (145, 155), f"M{n}_BUS", batn,
                "Resistor_SMD:R_0603_1608Metric")
    add_two_pin(sch, "Device:C", f"C{base + 50}", "10n 50V",
                (162, 155), f"M{n}_BUS", batn,
                "Capacitor_SMD:C_0603_1608Metric")
    add_two_pin(sch, "Device:Thermistor", f"TH{n}", "NTCG163JF103FT1",
                (192, 155), local_3v3a, f"M{n}_NTC",
                "Resistor_SMD:R_0603_1608Metric",
                "TDK", "NTCG163JF103FT1")
    add_two_pin(sch, "Device:R", f"R{base + 45}", "10k 0.1%",
                (212, 155), f"M{n}_NTC", batn,
                "Resistor_SMD:R_0603_1608Metric")

    add_part(
        sch, "Connector_Generic:Conn_01x02", f"J{base + 1}", "FUSED_BATTERY_TAP",
        (45, 70), "revb:POWER_LUG_M5_2P_P20",
        "Generic", "M5_RING_LUG_PAIR",
    )
    label_pin(sch, f"J{base + 1}", "1", f"M{n}_BATP_IN")
    label_pin(sch, f"J{base + 1}", "2", batn)
    add_part(
        sch, "Connector_Generic:Conn_01x03", f"J{base + 2}", "MOTOR_PHASES",
        (45, 105), "revb:POWER_LUG_M5_3P_P20",
        "Generic", "M5_RING_LUG_TRIPLE",
    )
    for pin, phase in zip(("1", "2", "3"), phases):
        label_pin(sch, f"J{base + 2}", pin, f"M{n}_PHASE_{phase}")

    add_two_pin(sch, "Device:R", f"R{base + 57}", "1k", (250, 205),
                f"M{n}_STATUS_LED", f"M{n}_LED_A",
                "Resistor_SMD:R_0603_1608Metric")
    add_part(sch, "Device:LED", f"D{base + 2}", "GREEN", (270, 205),
             "LED_SMD:LED_0603_1608Metric")
    label_pin(sch, f"D{base + 2}", "1", batn)
    label_pin(sch, f"D{base + 2}", "2", f"M{n}_LED_A")

    assert_no_label_collisions(sch, f"ESC motor {n}")
    sch.save(ESC_DIR / f"motor_{n}.kicad_sch")


def _make_esc_controller_legacy(parent_uuid: str, sheet_uuid: str) -> None:
    sch = ksa.create_schematic("esc_rev_b")
    sch.set_hierarchy_context(parent_uuid, sheet_uuid)
    nets = ["5V", "3V3", "DGND", "CANH", "CANL", "DRV_SCLK", "DRV_SDI", "DRV_SDO",
            "DRV_ENABLE", "DRV_3X_EN", "DRV_nFAULT"]
    for n in range(1, 7):
        nets += [f"M{n}_PWM_A", f"M{n}_PWM_B", f"M{n}_PWM_C", f"M{n}_nCS",
                 f"M{n}_CSA_A", f"M{n}_CSA_B", f"M{n}_NTC", f"M{n}_BUS"]
    add_hlabels(sch, nets)
    sch.add_text(
        "CENTRAL ESC CONTROL\n"
        "Six independent 3xPWM groups; driver SPI and ADC SPI are separate.\n"
        "TPS3430 + ARM gate hold all six ENABLE pins low until firmware is healthy.",
        (85, 16), size=1.1, bold=True,
    )

    mcu_ref = "U201"
    mcu = add_part(
        sch, "revb:RP2354B", mcu_ref, "RP2354B", (95, 80),
        "Package_DFN_QFN:QFN-80-1EP_10x10mm_P0.4mm_EP3.4x3.4mm",
        "Raspberry Pi", "RP2354B",
    )
    gpio_nets: dict[int, str] = {}
    for motor in range(1, 7):
        for phase_i, phase in enumerate(("A", "B", "C")):
            gpio_nets[(motor - 1) * 3 + phase_i] = f"M{motor}_PWM_{phase}"
    gpio_nets.update(
        {
            18: "DRV_SCLK", 19: "DRV_SDI", 20: "DRV_SDO",
            21: "M1_nCS", 22: "M2_nCS", 23: "M3_nCS", 24: "M4_nCS",
            25: "M5_nCS", 26: "M6_nCS", 27: "DRV_nFAULT",
            28: "ADC_SCLK", 29: "ADC_SDI", 30: "ADC_SDO_A",
            31: "ADC_SDO_B", 32: "ADC_nCS", 33: "CAN_TX",
            34: "CAN_RX", 35: "WDI", 36: "ARM", 37: "STATUS_LED",
            38: "DRV_3X_EN",
            40: "M5_NTC", 41: "M6_NTC", 42: "M1_BUS", 43: "M2_BUS",
            44: "M3_BUS", 45: "M4_BUS", 46: "M5_BUS", 47: "M6_BUS",
        }
    )
    power_map = {
        "VREG_AVDD": "VREG_AVDD", "USB_OTP_VDD": "3V3", "QSPI_IOVDD": "3V3",
        "IOVDD": "3V3", "VREG_PGND": "DGND", "GND": "DGND",
        "DVDD": "1V1", "VREG_VIN": "3V3", "VREG_LX": "VREG_LX",
        "VREG_FB": "1V1", "ADC_AVDD": "3V3_ADC", "RUN": "ESC_RUN", "SWCLK": "SWCLK",
        "SWDIO": "SWDIO", "XIN": "ESC_XIN", "XOUT": "ESC_XOUT",
    }
    connect_rp2354(sch, mcu_ref, gpio_nets, power_map)

    # Recommended RP2350 internal switching regulator network.
    add_two_pin(sch, "Device:L", "L201", "3.3u AOTA-B201610S3R3-101-T",
                (155, 42), "VREG_LX", "1V1", "Inductor_SMD:L_0805_2012Metric",
                "Abracon", "AOTA-B201610S3R3-101-T")
    add_two_pin(sch, "Device:R", "R201", "33",
                (155, 52), "1V1", "VREG_AVDD", "Resistor_SMD:R_0402_1005Metric")
    for ref, value, net in (
        ("C201", "4.7u", "3V3"), ("C202", "4.7u", "1V1"),
        ("C203", "4.7u", "VREG_AVDD"), ("C204", "4.7u", "3V3_ADC"),
    ):
        add_two_pin(sch, "Device:C", ref, value, (175, 40 + (int(ref[1:]) - 201) * 12),
                    net, "DGND", "Capacitor_SMD:C_0402_1005Metric")
    for k in range(8):
        add_two_pin(sch, "Device:C", f"C{205 + k}", "100n",
                    (195 + (k % 4) * 15, 40 + (k // 4) * 10),
                    "3V3" if k < 5 else "1V1", "DGND",
                    "Capacitor_SMD:C_0402_1005Metric")
    # RP2350B/RP2354B has eight IOVDD pins, three DVDD pins, and adjacent
    # USB_OTP_VDD/QSPI_IOVDD pins.  The first loop supplies five IOVDD and all
    # three DVDD bypasses; these complete the remaining I/O, USB/QSPI and ADC
    # bypass positions required by the Raspberry Pi hardware guide.
    for ref, net, pos in (
        ("C215", "3V3", (195, 62)),
        ("C216", "3V3", (210, 62)),
        ("C217", "3V3", (225, 62)),
        ("C218", "3V3", (240, 62)),
        ("C219", "3V3_ADC", (255, 62)),
    ):
        add_two_pin(
            sch, "Device:C", ref, "100n", pos, net, "DGND",
            "Capacitor_SMD:C_0402_1005Metric",
        )
    add_two_pin(sch, "Device:R", "R202", "10k", (150, 75), "3V3", "ESC_RUN",
                "Resistor_SMD:R_0603_1608Metric")
    add_two_pin(sch, "Device:R", "R204", "100k", (150, 84), "DRV_3X_EN", "DGND",
                "Resistor_SMD:R_0603_1608Metric")
    add_two_pin(sch, "Device:R", "R205", "4.7k", (165, 84), "3V3", "DRV_SDO",
                "Resistor_SMD:R_0603_1608Metric")
    add_two_pin(sch, "Device:R", "R206", "4.7k", (180, 84), "3V3", "DRV_nFAULT",
                "Resistor_SMD:R_0603_1608Metric")
    add_part(sch, "Device:Crystal", "Y201", "12MHz ABM8-272-T3", (160, 95),
             "Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm", "Abracon", "ABM8-272-T3")
    label_pin(sch, "Y201", "1", "ESC_XIN")
    label_pin(sch, "Y201", "2", "ESC_XOUT")
    add_two_pin(sch, "Device:C", "C213", "15p C0G", (185, 90), "ESC_XIN", "DGND",
                "Capacitor_SMD:C_0402_1005Metric")
    add_two_pin(sch, "Device:C", "C214", "15p C0G", (185, 100), "ESC_XOUT", "DGND",
                "Capacitor_SMD:C_0402_1005Metric")
    add_part(sch, "Connector_Generic:Conn_01x03", "J201", "ESC_SWD", (60, 175),
             "Connector_JST:JST_SH_SM03B-SRSS-TB_1x03-1MP_P1.00mm_Horizontal")
    for pin, net in {"1": "SWCLK", "2": "SWDIO", "3": "DGND"}.items():
        label_pin(sch, "J201", pin, net)
    add_two_pin(sch, "Device:R", "R203", "1k", (58, 192), "STATUS_LED", "ESC_LED_A",
                "Resistor_SMD:R_0603_1608Metric")
    add_part(sch, "Device:LED", "D201", "GREEN", (82, 192), "LED_SMD:LED_0603_1608Metric")
    label_pin(sch, "D201", "1", "DGND")
    label_pin(sch, "D201", "2", "ESC_LED_A")
    add_power_flag(sch, "#FLG201", "1V1", (205, 42))
    add_power_flag(sch, "#FLG202", "VREG_AVDD", (220, 42))
    add_power_flag(sch, "#FLG203", "3V3_ADC", (235, 42))
    add_two_pin(sch, "Device:FerriteBead", "FB201", "600R@100MHz",
                (250, 52), "3V3", "3V3_ADC", "Inductor_SMD:L_0603_1608Metric",
                "Murata", "BLM18AG601SN1D")

    # Two synchronized ADCs: one samples each low-side shunt at the same instant.
    for adc_i, suffix in enumerate(("A", "B")):
        ref = f"U{202 + adc_i}"
        pin_nets = {
            "1": f"M3_CSA_{suffix}", "2": f"M4_CSA_{suffix}",
            "3": f"M5_CSA_{suffix}", "4": f"M6_CSA_{suffix}",
            "5": "M1_NTC" if suffix == "A" else "M3_NTC",
            "6": "M2_NTC" if suffix == "A" else "M4_NTC",
            "7": "3V3_ADC", "8": f"ADC_{suffix}_DECAP", "9": "DGND",
            "10": "3V3", "11": "ADC_nCS", "12": f"ADC_SDO_{suffix}",
            "13": "ADC_SCLK", "14": "ADC_SDI", "15": f"M1_CSA_{suffix}",
            "16": f"M2_CSA_{suffix}", "17": "DGND",
        }
        add_custom(sch, "ADS7038", ref, "ADS7038IRTER", (92 + adc_i * 58, 145),
                   pin_nets, "Texas Instruments", "ADS7038IRTER")
        add_two_pin(sch, "Device:C", f"C{220 + adc_i * 2}", "1u",
                    (92 + adc_i * 58, 180), f"ADC_{suffix}_DECAP", "DGND",
                    "Capacitor_SMD:C_0603_1608Metric")
        add_two_pin(sch, "Device:C", f"C{221 + adc_i * 2}", "1u",
                    (106 + adc_i * 58, 180), "3V3_ADC", "DGND",
                    "Capacitor_SMD:C_0603_1608Metric")
    add_two_pin(sch, "Device:R", "R220", "10k", (205, 180),
                "3V3", "ADC_nCS", "Resistor_SMD:R_0603_1608Metric")

    # Watchdog output is ANDed with a pulled-down ARM command.
    add_custom(
        sch, "TPS3430", "U204", "TPS3430WDRCR", (220, 95),
        {
            "1": "3V3", "2": "WD_CWD", "3": "DGND", "4": "WD_CRST",
            "5": "DGND", "6": "DGND", "7": "WDI", "8": "ESC_RUN",
            "9": None, "10": "3V3", "11": "DGND",
        },
        "Texas Instruments", "TPS3430WDRCR",
    )
    add_two_pin(sch, "Device:C", "C230", "1n C0G", (242, 85), "WD_CWD", "DGND",
                "Capacitor_SMD:C_0603_1608Metric")
    add_two_pin(sch, "Device:C", "C231", "10n", (242, 95), "WD_CRST", "DGND",
                "Capacitor_SMD:C_0603_1608Metric")
    add_two_pin(sch, "Device:C", "C232", "100n", (258, 95), "3V3", "DGND",
                "Capacitor_SMD:C_0603_1608Metric")
    add_two_pin(sch, "Device:R", "R231", "100k", (242, 115), "ARM", "DGND",
                "Resistor_SMD:R_0603_1608Metric")
    add_custom(
        sch, "SN74LVC1G08", "U205", "SN74LVC1G08DBVR", (220, 135),
        {"1": "ESC_RUN", "2": "ARM", "3": "DGND", "4": "DRV_ENABLE", "5": "3V3"},
        "Texas Instruments", "SN74LVC1G08DBVR",
    )

    # Fault-protected 3.3 V CAN FD interface.
    add_custom(
        sch, "TCAN3413", "U206", "TCAN3413DR", (220, 165),
        {"1": "CAN_TX", "2": "DGND", "3": "3V3", "4": "CAN_RX", "5": "3V3",
         "6": "CANL", "7": "CANH", "8": "DGND"},
        "Texas Instruments", "TCAN3413DR",
    )
    add_two_pin(sch, "Device:R", "R240", "120 DNP", (252, 162), "CANH", "CANL",
                "Resistor_SMD:R_0603_1608Metric")
    add_two_pin(sch, "Device:C", "C240", "100n", (252, 173), "3V3", "DGND",
                "Capacitor_SMD:C_0603_1608Metric")

    assert_no_label_collisions(sch, "ESC controller")
    sch.save(ESC_DIR / "controller.kicad_sch")


def add_can_fd_interface(
    sch: ksa.Schematic,
    *,
    mcp_ref: str,
    xcvr_ref: str,
    choke_ref: str,
    esd_ref: str,
    crystal_ref: str,
    resistor_base: int,
    capacitor_base: int,
    ground: str,
    rail: str,
    bus_h: str,
    bus_l: str,
    spi_sck: str,
    spi_mosi: str,
    spi_miso: str,
    spi_ncs: str,
    irq: str,
    position: tuple[float, float],
) -> None:
    """Add a hardware CAN-FD controller, protected transceiver and bus filter."""
    x, y = position
    add_part(
        sch, "Interface_CAN_LIN:MCP2518FD-xQBB", mcp_ref, "MCP2518FD-H/QBB",
        (x, y), "Package_DFN_QFN:DFN-14-1EP_3x4.5mm_P0.65mm_EP1.65x4.25mm",
        "Microchip", "MCP2518FD-H/QBB",
    )
    mcp_nets = {
        "1": f"{mcp_ref}_TXCAN", "2": f"{mcp_ref}_RXCAN", "3": None,
        "4": irq, "5": f"{mcp_ref}_OSC2", "6": f"{mcp_ref}_OSC1",
        "7": ground, "8": None, "9": None, "10": spi_sck,
        "11": spi_mosi, "12": spi_miso, "13": spi_ncs,
        "14": rail, "15": ground,
    }
    mcp_info = ksa.get_symbol_info("Interface_CAN_LIN:MCP2518FD-xQBB")
    assert mcp_info is not None
    connect_pins(sch, mcp_ref, mcp_nets, tuple((pin.number, pin.name) for pin in mcp_info.pins))
    add_part(
        sch, "Device:Crystal", crystal_ref, "40MHz ABM8G", (x + 40, y - 12),
        "Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm",
        "Abracon", "ABM8G-40.000MHZ-10-D2Y-T",
    )
    label_pin(sch, crystal_ref, "1", f"{mcp_ref}_OSC1")
    label_pin(sch, crystal_ref, "2", f"{mcp_ref}_OSC2")
    add_two_pin(sch, "Device:C", f"C{capacitor_base}", "15p C0G",
                (x + 58, y - 18), f"{mcp_ref}_OSC1", ground,
                "Capacitor_SMD:C_0402_1005Metric")
    add_two_pin(sch, "Device:C", f"C{capacitor_base + 1}", "15p C0G",
                (x + 58, y - 7), f"{mcp_ref}_OSC2", ground,
                "Capacitor_SMD:C_0402_1005Metric")
    add_two_pin(sch, "Device:C", f"C{capacitor_base + 2}", "100n",
                (x + 18, y + 22), rail, ground,
                "Capacitor_SMD:C_0603_1608Metric")
    add_two_pin(sch, "Device:R", f"R{resistor_base}", "10k",
                (x + 35, y + 22), rail, spi_ncs,
                "Resistor_SMD:R_0603_1608Metric")
    add_two_pin(sch, "Device:R", f"R{resistor_base + 1}", "10k",
                (x + 50, y + 22), rail, irq,
                "Resistor_SMD:R_0603_1608Metric")

    add_custom(
        sch, "TCAN3413", xcvr_ref, "TCAN3413DR", (x + 78, y),
        {
            "1": f"{mcp_ref}_TXCAN", "2": ground, "3": rail,
            "4": f"{mcp_ref}_RXCAN", "5": rail,
            "6": f"{xcvr_ref}_CANL", "7": f"{xcvr_ref}_CANH", "8": ground,
        },
        "Texas Instruments", "TCAN3413DR",
    )
    add_two_pin(sch, "Device:C", f"C{capacitor_base + 3}", "100n",
                (x + 98, y + 22), rail, ground,
                "Capacitor_SMD:C_0603_1608Metric")
    add_part(
        sch, "Device:L_Coupled", choke_ref, "ACT45B-110-2P-TL003",
        (x + 120, y), "revb:L_CommonModeChoke_Coilank_ACM4532",
        "TDK", "ACT45B-110-2P-TL003",
    )
    for pin, net in {
        "1": f"{xcvr_ref}_CANH", "2": bus_h,
        "3": f"{xcvr_ref}_CANL", "4": bus_l,
    }.items():
        label_pin(sch, choke_ref, pin, net)
    add_two_pin(sch, "Device:R", f"R{resistor_base + 2}", "120 DNP",
                (x + 105, y - 18), f"{xcvr_ref}_CANH", f"{xcvr_ref}_CANL",
                "Resistor_SMD:R_0603_1608Metric")
    add_custom(
        sch, "PESD2CANFD24L", esd_ref, "PESD2CANFD24L-T", (x + 150, y),
        {"1": bus_h, "2": bus_l, "3": ground},
        "Nexperia", "PESD2CANFD24L-T",
    )


def make_esc_controller(parent_uuid: str, sheet_uuid: str) -> None:
    sch = ksa.create_schematic("esc_rev_b")
    sch.set_hierarchy_context(parent_uuid, sheet_uuid)
    nets = ["5V", "3V3", "DGND", "CANH", "CANL", "ARM_SAFE"]
    for n in range(1, 7):
        nets += [f"M{n}_CMD", f"M{n}_STATUS"]
    add_hlabels(sch, nets)
    sch.add_text(
        "CENTRAL ESC SUPERVISOR\n"
        "Six isolated command/status links; local STM32G431s perform PWM and ADC sampling.\n"
        "TPS3430 gates ARM_SAFE low on reset, stalled firmware, or lost central power.",
        (85, 16), size=1.1, bold=True,
    )

    mcu_ref = "U201"
    add_part(
        sch, "revb:RP2354B", mcu_ref, "RP2354B", (95, 80),
        "Package_DFN_QFN:QFN-80-1EP_10x10mm_P0.4mm_EP3.4x3.4mm",
        "Raspberry Pi", "RP2354B",
    )
    gpio_nets = {
        **{motor - 1: f"M{motor}_CMD" for motor in range(1, 7)},
        **{motor + 5: f"M{motor}_STATUS" for motor in range(1, 7)},
        12: "CAN_SCK", 13: "CAN_MOSI", 14: "CAN_MISO",
        15: "CAN_nCS", 16: "CAN_INT", 17: "WDI",
        18: "ARM_CMD", 19: "STATUS_LED",
    }
    power_map = {
        "VREG_AVDD": "VREG_AVDD", "USB_OTP_VDD": "3V3", "QSPI_IOVDD": "3V3",
        "IOVDD": "3V3", "VREG_PGND": "DGND", "GND": "DGND",
        "DVDD": "1V1", "VREG_VIN": "3V3", "VREG_LX": "VREG_LX",
        "VREG_FB": "1V1", "ADC_AVDD": "3V3_ADC", "RUN": "ESC_RUN",
        "SWCLK": "SWCLK", "SWDIO": "SWDIO",
        "XIN": "ESC_XIN", "XOUT": "ESC_XOUT",
    }
    connect_rp2354(sch, mcu_ref, gpio_nets, power_map)

    add_two_pin(sch, "Device:L", "L201", "3.3u AOTA-B201610S3R3-101-T",
                (155, 42), "VREG_LX", "1V1", "Inductor_SMD:L_0805_2012Metric",
                "Abracon", "AOTA-B201610S3R3-101-T")
    add_two_pin(sch, "Device:R", "R201", "33", (155, 52),
                "1V1", "VREG_AVDD", "Resistor_SMD:R_0402_1005Metric")
    for ref, value, net in (
        ("C201", "4.7u", "3V3"), ("C202", "4.7u", "1V1"),
        ("C203", "4.7u", "VREG_AVDD"), ("C204", "4.7u", "3V3_ADC"),
    ):
        add_two_pin(sch, "Device:C", ref, value,
                    (175, 40 + (int(ref[1:]) - 201) * 12), net, "DGND",
                    "Capacitor_SMD:C_0402_1005Metric")
    for k in range(8):
        add_two_pin(sch, "Device:C", f"C{205 + k}", "100n",
                    (195 + (k % 4) * 15, 40 + (k // 4) * 10),
                    "3V3" if k < 5 else "1V1", "DGND",
                    "Capacitor_SMD:C_0402_1005Metric")
    for ref, net, pos in (
        ("C215", "3V3", (195, 62)),
        ("C216", "3V3", (210, 62)),
        ("C217", "3V3", (225, 62)),
        ("C218", "3V3", (240, 62)),
        ("C219", "3V3_ADC", (255, 62)),
    ):
        add_two_pin(
            sch, "Device:C", ref, "100n", pos, net, "DGND",
            "Capacitor_SMD:C_0402_1005Metric",
        )
    add_two_pin(sch, "Device:R", "R202", "10k", (150, 75),
                "3V3", "ESC_RUN", "Resistor_SMD:R_0603_1608Metric")
    add_part(sch, "Device:Crystal", "Y201", "12MHz ABM8-272-T3", (160, 95),
             "Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm",
             "Abracon", "ABM8-272-T3")
    label_pin(sch, "Y201", "1", "ESC_XIN")
    label_pin(sch, "Y201", "2", "ESC_XOUT")
    add_two_pin(sch, "Device:C", "C213", "15p C0G", (185, 90),
                "ESC_XIN", "DGND", "Capacitor_SMD:C_0402_1005Metric")
    add_two_pin(sch, "Device:C", "C214", "15p C0G", (185, 100),
                "ESC_XOUT", "DGND", "Capacitor_SMD:C_0402_1005Metric")
    add_two_pin(sch, "Device:FerriteBead", "FB201", "600R@100MHz",
                (250, 52), "3V3", "3V3_ADC",
                "Inductor_SMD:L_0603_1608Metric",
                "Murata", "BLM18AG601SN1D")
    for ref, net, pos in (
        ("#FLG201", "1V1", (205, 42)),
        ("#FLG202", "VREG_AVDD", (220, 42)),
        ("#FLG203", "3V3_ADC", (235, 42)),
    ):
        add_power_flag(sch, ref, net, pos)

    add_part(sch, "Connector_Generic:Conn_01x03", "J201", "ESC_SWD",
             (60, 175),
             "Connector_JST:JST_SH_SM03B-SRSS-TB_1x03-1MP_P1.00mm_Horizontal")
    for pin, net in {"1": "SWCLK", "2": "SWDIO", "3": "DGND"}.items():
        label_pin(sch, "J201", pin, net)
    add_two_pin(sch, "Device:R", "R203", "1k", (58, 192),
                "STATUS_LED", "ESC_LED_A", "Resistor_SMD:R_0603_1608Metric")
    add_part(sch, "Device:LED", "D201", "GREEN", (82, 192),
             "LED_SMD:LED_0603_1608Metric")
    label_pin(sch, "D201", "1", "DGND")
    label_pin(sch, "D201", "2", "ESC_LED_A")

    add_custom(
        sch, "TPS3430", "U204", "TPS3430WDRCR", (220, 95),
        {
            "1": "3V3", "2": "WD_CWD", "3": "DGND", "4": "WD_CRST",
            "5": "DGND", "6": "DGND", "7": "WDI", "8": "ESC_RUN",
            "9": None, "10": "3V3", "11": "DGND",
        },
        "Texas Instruments", "TPS3430WDRCR",
    )
    add_two_pin(sch, "Device:C", "C230", "1n C0G", (242, 85),
                "WD_CWD", "DGND", "Capacitor_SMD:C_0603_1608Metric")
    add_two_pin(sch, "Device:C", "C231", "10n", (242, 95),
                "WD_CRST", "DGND", "Capacitor_SMD:C_0603_1608Metric")
    add_two_pin(sch, "Device:C", "C232", "100n", (258, 95),
                "3V3", "DGND", "Capacitor_SMD:C_0603_1608Metric")
    add_two_pin(sch, "Device:R", "R231", "100k", (242, 115),
                "ARM_CMD", "DGND", "Resistor_SMD:R_0603_1608Metric")
    add_custom(
        sch, "SN74LVC1G08", "U205", "SN74LVC1G08DBVR", (220, 135),
        {"1": "ESC_RUN", "2": "ARM_CMD", "3": "DGND", "4": "ARM_SAFE", "5": "3V3"},
        "Texas Instruments", "SN74LVC1G08DBVR",
    )
    add_two_pin(sch, "Device:C", "C233", "100n", (245, 135),
                "3V3", "DGND", "Capacitor_SMD:C_0603_1608Metric")

    add_can_fd_interface(
        sch, mcp_ref="U207", xcvr_ref="U206", choke_ref="FL201",
        esd_ref="D202", crystal_ref="Y202",
        resistor_base=240, capacitor_base=240,
        ground="DGND", rail="3V3", bus_h="CANH", bus_l="CANL",
        spi_sck="CAN_SCK", spi_mosi="CAN_MOSI", spi_miso="CAN_MISO",
        spi_ncs="CAN_nCS", irq="CAN_INT", position=(95, 150),
    )

    assert_no_label_collisions(sch, "ESC controller")
    sch.save(ESC_DIR / "controller.kicad_sch")


def make_esc_power(parent_uuid: str, sheet_uuid: str) -> None:
    sch = ksa.create_schematic("esc_rev_b")
    sch.set_hierarchy_context(parent_uuid, sheet_uuid)
    nets = ["AUX_BATT", "AUX_GND", "5V", "3V3", "DGND", "CANH", "CANL"]
    add_hlabels(sch, nets)
    sch.add_text(
        "TWO INDEPENDENT 1 A AUXILIARY RAILS\n"
        "One LM5164 powers ESC logic/cells; the other powers the flight board over CAN.\n"
        "The fused auxiliary input never carries motor current.",
        (85, 18), size=1.1, bold=True,
    )
    add_part(
        sch, "Connector_Generic:Conn_01x02", "J701", "AUX_BATTERY",
        (35, 60), "Connector_Molex:Molex_Micro-Fit_3.0_43650-0200_1x02_P3.00mm_Horizontal",
        "Molex", "43650-0200",
    )
    label_pin(sch, "J701", "1", "AUX_BATT")
    label_pin(sch, "J701", "2", "AUX_GND")
    add_part(
        sch, "Regulator_Switching:LM5164DDA", "U701", "LM5164DDA", (92, 68),
        "Package_SO:HSOP-8-1EP_3.9x4.9mm_P1.27mm_EP2.41x3.1mm_ThermalVias",
        "Texas Instruments", "LM5164DDAR",
    )
    lm_nets = {
        "1": "AUX_GND", "2": "AUX_BATT", "3": "AUX_BATT", "4": "LM_RON",
        "5": "LM_FB", "7": "LM_BST", "8": "LM_SW", "9": "AUX_GND",
    }
    for pin, net in lm_nets.items():
        label_pin(sch, "U701", pin, net)
    no_connect_pin(sch, "U701", "6")
    add_two_pin(sch, "Device:R", "R701", "41.2k 1%", (58, 90), "LM_RON", "AUX_GND",
                "Resistor_SMD:R_0603_1608Metric")
    add_two_pin(sch, "Device:C", "C701", "2.2u 100V X7R", (58, 105), "AUX_BATT", "AUX_GND",
                "Capacitor_SMD:C_1210_3225Metric")
    add_two_pin(sch, "Device:C", "C702", "10u 100V", (75, 105), "AUX_BATT", "AUX_GND",
                "Capacitor_SMD:C_1210_3225Metric")
    add_two_pin(sch, "Device:C", "C700", "2.2u 100V X7R", (92, 105), "AUX_BATT", "AUX_GND",
                "Capacitor_SMD:C_1210_3225Metric")
    add_two_pin(sch, "Device:C", "C703", "2.2n 50V X7R", (115, 52), "LM_BST", "LM_SW",
                "Capacitor_SMD:C_0603_1608Metric")
    add_two_pin(sch, "Device:L", "L701", "33u 1.8A", (120, 72), "LM_SW", "5V",
                "Inductor_SMD:L_Bourns_SRR1260")
    add_two_pin(sch, "Device:C", "C704", "22u 10V", (142, 83), "5V", "AUX_GND",
                "Capacitor_SMD:C_1210_3225Metric")
    add_two_pin(sch, "Device:C", "C705", "22u 10V", (158, 83), "5V", "AUX_GND",
                "Capacitor_SMD:C_1210_3225Metric")
    add_two_pin(sch, "Device:R", "R702", "45.3k 1%", (142, 60), "5V", "LM_FB",
                "Resistor_SMD:R_0603_1608Metric")
    add_two_pin(sch, "Device:R", "R703", "14.3k 1%", (158, 60), "LM_FB", "AUX_GND",
                "Resistor_SMD:R_0603_1608Metric")
    # Type-3 ripple injection follows TI SNVA874's 15-100 V to 5 V example.
    add_two_pin(sch, "Device:R", "R705", "226k 1%", (105, 92), "LM_SW", "LM_RIPPLE",
                "Resistor_SMD:R_0603_1608Metric")
    add_two_pin(sch, "Device:C", "C708", "3.3n X7R", (122, 92), "LM_RIPPLE", "5V",
                "Capacitor_SMD:C_0603_1608Metric")
    add_two_pin(sch, "Device:C", "C709", "470p C0G", (139, 92), "LM_RIPPLE", "LM_FB",
                "Capacitor_SMD:C_0603_1608Metric")

    add_custom(
        sch, "AP2112K", "U702", "AP2112K-3.3TRG1", (205, 70),
        {"1": "5V", "2": "DGND", "3": "5V", "4": None, "5": "3V3"},
        "Diodes Inc.", "AP2112K-3.3TRG1",
    )
    add_two_pin(sch, "Device:C", "C706", "1u", (230, 58), "5V", "DGND",
                "Capacitor_SMD:C_0603_1608Metric")
    add_two_pin(sch, "Device:C", "C707", "1u", (230, 72), "3V3", "DGND",
                "Capacitor_SMD:C_0603_1608Metric")
    add_two_pin(sch, "Device:R", "R704", "0 STAR", (195, 105), "AUX_GND", "DGND",
                "Resistor_SMD:R_1206_3216Metric")
    add_two_pin(sch, "Device:D_TVS", "D701", "SM8S51A", (48, 120), "AUX_GND", "AUX_BATT",
                "revb:DO-218AB_SM8S", "Littelfuse", "SM8S51A")

    # A second converter prevents the six local-controller isolation supplies
    # and the RF-enabled flight board from sharing one marginal 1 A budget.
    add_part(
        sch, "Regulator_Switching:LM5164DDA", "U703", "LM5164DDA", (92, 175),
        "Package_SO:HSOP-8-1EP_3.9x4.9mm_P1.27mm_EP2.41x3.1mm_ThermalVias",
        "Texas Instruments", "LM5164DDAR",
    )
    for pin, net in {
        "1": "AUX_GND", "2": "AUX_BATT", "3": "AUX_BATT",
        "4": "LM2_RON", "5": "LM2_FB", "7": "LM2_BST",
        "8": "LM2_SW", "9": "AUX_GND",
    }.items():
        label_pin(sch, "U703", pin, net)
    no_connect_pin(sch, "U703", "6")
    add_two_pin(sch, "Device:R", "R710", "41.2k 1%", (58, 190),
                "LM2_RON", "AUX_GND", "Resistor_SMD:R_0603_1608Metric")
    add_two_pin(sch, "Device:C", "C710", "2.2u 100V X7R", (58, 205),
                "AUX_BATT", "AUX_GND", "Capacitor_SMD:C_1210_3225Metric")
    add_two_pin(sch, "Device:C", "C711", "10u 100V", (75, 205),
                "AUX_BATT", "AUX_GND", "Capacitor_SMD:C_1210_3225Metric")
    add_two_pin(sch, "Device:C", "C712", "2.2n 50V X7R", (115, 162),
                "LM2_BST", "LM2_SW", "Capacitor_SMD:C_0603_1608Metric")
    add_two_pin(sch, "Device:L", "L702", "33u 1.8A", (120, 177),
                "LM2_SW", "CAN_5V_RAW", "Inductor_SMD:L_Bourns_SRR1260")
    add_two_pin(sch, "Device:C", "C713", "22u 10V", (142, 188),
                "CAN_5V_RAW", "AUX_GND", "Capacitor_SMD:C_1210_3225Metric")
    add_two_pin(sch, "Device:C", "C714", "22u 10V", (158, 188),
                "CAN_5V_RAW", "AUX_GND", "Capacitor_SMD:C_1210_3225Metric")
    add_two_pin(sch, "Device:R", "R711", "45.3k 1%", (142, 165),
                "CAN_5V_RAW", "LM2_FB", "Resistor_SMD:R_0603_1608Metric")
    add_two_pin(sch, "Device:R", "R712", "14.3k 1%", (158, 165),
                "LM2_FB", "AUX_GND", "Resistor_SMD:R_0603_1608Metric")
    add_two_pin(sch, "Device:R", "R713", "226k 1%", (105, 197),
                "LM2_SW", "LM2_RIPPLE", "Resistor_SMD:R_0603_1608Metric")
    add_two_pin(sch, "Device:C", "C715", "3.3n X7R", (122, 197),
                "LM2_RIPPLE", "CAN_5V_RAW", "Capacitor_SMD:C_0603_1608Metric")
    add_two_pin(sch, "Device:C", "C716", "470p C0G", (139, 197),
                "LM2_RIPPLE", "LM2_FB", "Capacitor_SMD:C_0603_1608Metric")
    add_two_pin(sch, "Device:Fuse", "F701", "PTC 0.75A", (188, 177),
                "CAN_5V_RAW", "CAN_5V_OUT", "Fuse:Fuse_1206_3216Metric",
                "Littelfuse", "1206L075/24WR")
    add_part(
        sch, "Connector_Generic:Conn_01x04", "J702", "FLIGHT_CAN_POWER",
        (230, 177), "Connector_JST:JST_GH_SM04B-GHS-TB_1x04-1MP_P1.25mm_Horizontal",
        "JST", "SM04B-GHS-TB",
    )
    for pin, net in {
        "1": "CAN_5V_OUT", "2": "DGND", "3": "CANH", "4": "CANL",
    }.items():
        label_pin(sch, "J702", pin, net)

    for ref, net, pos in (
        ("#FLG701", "AUX_BATT", (35, 140)), ("#FLG702", "AUX_GND", (55, 140)),
        ("#FLG703", "5V", (75, 140)), ("#FLG704", "3V3", (95, 140)),
        ("#FLG705", "DGND", (115, 140)), ("#FLG706", "CAN_5V_RAW", (135, 140)),
    ):
        add_power_flag(sch, ref, net, pos)
    assert_no_label_collisions(sch, "ESC auxiliary power")
    sch.save(ESC_DIR / "power.kicad_sch")


def make_esc_top() -> None:
    top = ksa.create_schematic("esc_rev_b")
    top.add_text(
        "REV-B SIX-CHANNEL 12S / 60 A ESC\n"
        "Distinct project from the flight-control board. Six fused power inputs; external busbar required.",
        (45, 12), size=1.4, bold=True,
    )
    power_nets = ["AUX_BATT", "AUX_GND", "5V", "3V3", "DGND", "CANH", "CANL"]
    controller_nets = ["5V", "3V3", "DGND", "CANH", "CANL", "ARM_SAFE"]
    for n in range(1, 7):
        controller_nets += [f"M{n}_CMD", f"M{n}_STATUS"]
    power_uuid = add_sheet_with_pins(top, "Auxiliary power", "power.kicad_sch",
                                     (15.24, 25.40), (71.12, 40.64), power_nets, "esc_rev_b")
    controller_uuid = add_sheet_with_pins(top, "Controller and ADC", "controller.kicad_sch",
                                          (93.98, 20.32), (195.58, 81.28), controller_nets, "esc_rev_b")
    motor_uuids: list[str] = []
    for n in range(1, 7):
        motor_nets = [
            f"M{n}_BATP", f"M{n}_BATN", f"M{n}_PHASE_A", f"M{n}_PHASE_B", f"M{n}_PHASE_C",
            f"M{n}_CMD", f"M{n}_STATUS", "ARM_SAFE", "5V", "3V3", "DGND",
        ]
        x = 15.24 + ((n - 1) % 3) * 93.98
        y = 109.22 + ((n - 1) // 3) * 50.80
        motor_uuids.append(
            add_sheet_with_pins(top, f"Motor {n} power cell", f"motor_{n}.kicad_sch",
                                (x, y), (83.82, 45.72), motor_nets, "esc_rev_b")
        )
    top.save(ESC_DIR / "esc_rev_b.kicad_sch")
    make_esc_power(top.uuid, power_uuid)
    make_esc_controller(top.uuid, controller_uuid)
    for n, uuid in enumerate(motor_uuids, start=1):
        make_motor_sheet(n, top.uuid, uuid)


def make_main_power(parent_uuid: str, sheet_uuid: str) -> None:
    sch = ksa.create_schematic("main_rev_b")
    sch.set_hierarchy_context(parent_uuid, sheet_uuid)
    nets = [
        "CAN_5V", "CAN_GND", "CANH", "CANL", "SYS_5V", "3V3",
        "3V3_GNSS", "2V8_GNSS", "GNSS_PWR_EN", "1V1",
        "VREG_LX", "VREG_AVDD", "3V3_ADC", "GND", "USB_DP", "USB_DM",
    ]
    add_hlabels(sch, nets)
    sch.add_text(
        "FLIGHT-CONTROL POWER + REAL USB-C\n"
        "CAN cable power and USB VBUS are diode-ORed; neither may backfeed the other.",
        (85, 17), size=1.1, bold=True,
    )
    add_part(
        sch, "Connector:USB_C_Receptacle_USB2.0_16P", "J1", "USB-C USB2",
        (45, 65), "Connector_USB:USB_C_Receptacle_GCT_USB4105-xx-A_16P_TopMnt_Horizontal",
        "GCT", "USB4105-GF-A",
    )
    usb_nets = {
        "A1": "GND", "A12": "GND", "B1": "GND", "B12": "GND", "S1": "CHASSIS",
        "A4": "USB_VBUS", "A9": "USB_VBUS", "B4": "USB_VBUS", "B9": "USB_VBUS",
        "A5": "USB_CC1", "B5": "USB_CC2", "A7": "USB_DN_CONN", "B7": "USB_DN_CONN",
        "A6": "USB_DP_CONN", "B6": "USB_DP_CONN",
    }
    for pin, net in usb_nets.items():
        label_pin(sch, "J1", pin, net)
    no_connect_pin(sch, "J1", "A8")
    no_connect_pin(sch, "J1", "B8")
    add_two_pin(sch, "Device:R", "R5", "1M", (75, 32), "CHASSIS", "GND",
                "Resistor_SMD:R_0603_1608Metric")
    add_two_pin(sch, "Device:C", "C5", "4.7n 2kV", (95, 32), "CHASSIS", "GND",
                "Capacitor_SMD:C_1206_3216Metric")
    add_two_pin(sch, "Device:R", "R1", "5.1k", (75, 45), "USB_CC1", "GND",
                "Resistor_SMD:R_0603_1608Metric")
    add_two_pin(sch, "Device:R", "R2", "5.1k", (75, 55), "USB_CC2", "GND",
                "Resistor_SMD:R_0603_1608Metric")
    add_custom(
        sch, "USBLC6-2SC6", "U1", "USBLC6-2SC6", (95, 73),
        {"1": "USB_DN_CONN", "2": "GND", "3": "USB_DP_CONN", "4": "USB_DP",
         "5": "USB_VBUS", "6": "USB_DM"},
        "STMicroelectronics", "USBLC6-2SC6",
    )
    add_two_pin(sch, "Device:D_Schottky", "D1", "PMEG2010ER", (128, 45),
                "USB_VBUS", "SYS_5V", "Diode_SMD:D_SOD-123", "Nexperia", "PMEG2010ER")

    add_part(
        sch, "Connector_Generic:Conn_01x04", "J2", "ESC_CAN_POWER",
        (45, 120), "Connector_JST:JST_GH_SM04B-GHS-TB_1x04-1MP_P1.25mm_Horizontal",
        "JST", "SM04B-GHS-TB",
    )
    for pin, net in {"1": "CAN_5V", "2": "CAN_GND", "3": "CANH", "4": "CANL"}.items():
        label_pin(sch, "J2", pin, net)
    add_two_pin(sch, "Device:Fuse", "F1", "PTC 0.75A", (78, 110),
                "CAN_5V", "CAN_5V_FUSED", "Fuse:Fuse_1206_3216Metric",
                "Littelfuse", "1206L075/24WR")
    add_two_pin(sch, "Device:D_Schottky", "D2", "PMEG2010ER", (108, 110),
                "CAN_5V_FUSED", "SYS_5V", "Diode_SMD:D_SOD-123", "Nexperia", "PMEG2010ER")
    add_two_pin(sch, "Device:R", "R3", "0 STAR", (78, 130), "CAN_GND", "GND",
                "Resistor_SMD:R_1206_3216Metric")

    add_custom(
        sch, "TLV62569", "U2", "TLV62569DBVR", (155, 72),
        {"1": "SYS_5V", "2": "GND", "3": "BUCK_SW", "4": "SYS_5V", "5": "BUCK_FB"},
        "Texas Instruments", "TLV62569DBVR",
    )
    add_two_pin(sch, "Device:L", "L2", "2.2u 3.5A",
                (178, 72), "BUCK_SW", "3V3", "Inductor_SMD:L_APV_ANR4018",
                "Taiyo Yuden", "NR4018T2R2M")
    add_two_pin(sch, "Device:R", "R6", "453k 1%",
                (178, 84), "3V3", "BUCK_FB", "Resistor_SMD:R_0402_1005Metric")
    add_two_pin(sch, "Device:R", "R7", "100k 1%",
                (195, 84), "BUCK_FB", "GND", "Resistor_SMD:R_0402_1005Metric")
    add_custom(
        sch, "TLV755P", "U3", "TLV75528PDBVR", (165, 115),
        {"1": "SYS_5V", "2": "GND", "3": "GNSS_PWR_EN", "4": None, "5": "2V8_GNSS"},
        "Texas Instruments", "TLV75528PDBVR",
    )
    add_custom(
        sch, "TLV755P", "U4", "TLV75533PDBVR", (165, 135),
        {"1": "SYS_5V", "2": "GND", "3": "GNSS_PWR_EN", "4": None, "5": "3V3_GNSS"},
        "Texas Instruments", "TLV75533PDBVR",
    )
    # Default-on so GNSS starts even before firmware configures GPIO33; the
    # MCU can pull this node low to perform the manufacturer-recommended
    # controlled restart without removing the always-on backup-domain rail.
    add_two_pin(sch, "Device:R", "R8", "100k", (138, 125), "3V3", "GNSS_PWR_EN",
                "Resistor_SMD:R_0402_1005Metric")
    for ref, net, pos in (
        ("C1", "SYS_5V", (210, 55)), ("C2", "3V3", (225, 55)),
        ("C6", "3V3", (240, 55)),
        ("C3", "SYS_5V", (195, 105)), ("C4", "2V8_GNSS", (210, 105)),
        ("C7", "SYS_5V", (225, 105)), ("C8", "3V3_GNSS", (240, 105)),
    ):
        value = "4.7u 10V" if ref == "C1" else ("10u 10V" if ref in ("C2", "C6") else "1u")
        footprint = "Capacitor_SMD:C_0805_2012Metric" if ref in ("C1", "C2", "C6") else "Capacitor_SMD:C_0603_1608Metric"
        add_two_pin(sch, "Device:C", ref, value, pos, net, "GND", footprint)

    add_two_pin(sch, "Device:L", "L1", "3.3u AOTA-B201610S3R3-101-T",
                (150, 155), "VREG_LX", "1V1", "Inductor_SMD:L_0805_2012Metric",
                "Abracon", "AOTA-B201610S3R3-101-T")
    add_two_pin(sch, "Device:R", "R4", "33", (180, 155), "1V1", "VREG_AVDD",
                "Resistor_SMD:R_0402_1005Metric")
    add_two_pin(sch, "Device:FerriteBead", "FB1", "600R@100MHz", (215, 155),
                "3V3", "3V3_ADC", "Inductor_SMD:L_0603_1608Metric",
                "Murata", "BLM18AG601SN1D")
    for k, net in enumerate(("3V3", "1V1", "VREG_AVDD", "3V3_ADC")):
        add_two_pin(sch, "Device:C", f"C{10 + k}", "4.7u", (150 + k * 22, 175),
                    net, "GND", "Capacitor_SMD:C_0402_1005Metric")
    for ref, net, pos in (
        ("#FLG1", "SYS_5V", (40, 175)), ("#FLG2", "3V3", (60, 175)),
        ("#FLG3", "2V8_GNSS", (80, 175)), ("#FLG4", "1V1", (100, 175)),
        ("#FLG5", "GND", (120, 175)), ("#FLG6", "3V3_GNSS", (140, 175)),
    ):
        add_power_flag(sch, ref, net, pos)
    sch.save(MAIN_DIR / "power_usb.kicad_sch")


def make_main_mcu(parent_uuid: str, sheet_uuid: str) -> None:
    sch = ksa.create_schematic("main_rev_b")
    sch.set_hierarchy_context(parent_uuid, sheet_uuid)
    nets = ["3V3", "1V1", "VREG_LX", "VREG_AVDD", "3V3_ADC", "GND", "USB_DP", "USB_DM",
            "SENS_SCLK", "SENS_MOSI", "SENS_MISO", "ICM_nCS", "LSM_nCS",
            "ICM_INT1", "ICM_INT2", "LSM_INT1", "LSM_INT2", "BARO_SCL", "BARO_SDA",
            "BARO_DRDY", "GNSS_TX", "GNSS_RX", "GNSS_PPS", "GNSS_RESET_N",
            "GNSS_PWR_EN",
            "RF_SCLK", "RF_MOSI", "RF_MISO", "RF_nCS", "RF_RESET_N", "RF_GPIO0",
            "RF_GPIO2", "RF_GPIO3", "PA_EN", "LNA_EN", "HGM",
            "CAN_SCK", "CAN_MOSI", "CAN_MISO", "CAN_nCS", "CAN_INT"]
    add_hlabels(sch, nets)
    sch.add_text("RP2354B FLIGHT MCU\nInternal switcher and decoupling follow the Raspberry Pi hardware guide.",
                 (80, 16), size=1.1, bold=True)
    add_part(
        sch, "revb:RP2354B", "U10", "RP2354B", (105, 92),
        "Package_DFN_QFN:QFN-80-1EP_10x10mm_P0.4mm_EP3.4x3.4mm",
        "Raspberry Pi", "RP2354B",
    )
    gpio = {
        0: "SENS_SCLK", 1: "SENS_MOSI", 2: "SENS_MISO", 3: "ICM_nCS",
        4: "LSM_nCS", 5: "ICM_INT1", 6: "ICM_INT2", 7: "LSM_INT1",
        8: "LSM_INT2", 9: "BARO_SCL", 10: "BARO_SDA", 11: "BARO_DRDY",
        12: "GNSS_TX", 13: "GNSS_RX", 14: "GNSS_PPS", 15: "GNSS_RESET_N",
        16: "RF_SCLK", 17: "RF_MOSI", 18: "RF_MISO", 19: "RF_nCS",
        20: "RF_RESET_N", 21: "RF_GPIO0", 22: "RF_GPIO2", 23: "RF_GPIO3",
        24: "PA_EN", 25: "LNA_EN", 26: "HGM",
        27: "CAN_SCK", 28: "CAN_MOSI", 29: "CAN_MISO",
        30: "CAN_nCS", 31: "CAN_INT", 32: "STATUS_LED", 33: "GNSS_PWR_EN",
    }
    power_map = {
        "VREG_AVDD": "VREG_AVDD", "USB_OTP_VDD": "3V3", "QSPI_IOVDD": "3V3",
        "IOVDD": "3V3", "VREG_PGND": "GND", "GND": "GND", "DVDD": "1V1",
        "VREG_VIN": "3V3", "VREG_LX": "VREG_LX", "VREG_FB": "1V1",
        "ADC_AVDD": "3V3_ADC", "USB_DM": "USB_DM_MCU", "USB_DP": "USB_DP_MCU",
        "RUN": "RUN", "SWCLK": "SWCLK", "SWDIO": "SWDIO",
        "XIN": "XIN", "XOUT": "XOUT",
    }
    connect_rp2354(sch, "U10", gpio, power_map)
    add_two_pin(sch, "Device:R", "R10", "27", (165, 65), "USB_DP", "USB_DP_MCU",
                "Resistor_SMD:R_0402_1005Metric")
    add_two_pin(sch, "Device:R", "R11", "27", (165, 75), "USB_DM", "USB_DM_MCU",
                "Resistor_SMD:R_0402_1005Metric")
    add_two_pin(sch, "Device:R", "R12", "10k", (165, 88), "3V3", "RUN",
                "Resistor_SMD:R_0603_1608Metric")
    add_two_pin(sch, "Device:R", "R13", "1k", (205, 88), "STATUS_LED", "MAIN_LED_A",
                "Resistor_SMD:R_0603_1608Metric")
    add_part(sch, "Device:LED", "D10", "GREEN", (230, 88), "LED_SMD:LED_0603_1608Metric")
    label_pin(sch, "D10", "1", "GND")
    label_pin(sch, "D10", "2", "MAIN_LED_A")
    add_part(sch, "Device:Crystal", "Y1", "12MHz ABM8-272-T3", (165, 115),
             "Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm", "Abracon", "ABM8-272-T3")
    label_pin(sch, "Y1", "1", "XIN")
    label_pin(sch, "Y1", "2", "XOUT")
    add_two_pin(sch, "Device:C", "C20", "15p C0G", (195, 110), "XIN", "GND",
                "Capacitor_SMD:C_0402_1005Metric")
    add_two_pin(sch, "Device:C", "C21", "15p C0G", (195, 122), "XOUT", "GND",
                "Capacitor_SMD:C_0402_1005Metric")
    for k in range(12):
        add_two_pin(sch, "Device:C", f"C{30 + k}", "100n", (160 + (k % 4) * 20, 145 + (k // 4) * 10),
                    "3V3" if k < 9 else "1V1", "GND",
                    "Capacitor_SMD:C_0402_1005Metric")
    add_two_pin(sch, "Device:C", "C42", "100n", (240, 165),
                "3V3_ADC", "GND", "Capacitor_SMD:C_0402_1005Metric")
    add_power_flag(sch, "#FLG11", "VREG_AVDD", (220, 42))
    add_power_flag(sch, "#FLG12", "3V3_ADC", (235, 42))
    add_part(sch, "Connector_Generic:Conn_01x03", "J10", "SWD", (45, 150),
             "Connector_JST:JST_SH_SM03B-SRSS-TB_1x03-1MP_P1.00mm_Horizontal")
    for pin, net in {"1": "SWCLK", "2": "SWDIO", "3": "GND"}.items():
        label_pin(sch, "J10", pin, net)
    sch.save(MAIN_DIR / "mcu.kicad_sch")


def make_main_sensors(parent_uuid: str, sheet_uuid: str) -> None:
    sch = ksa.create_schematic("main_rev_b")
    sch.set_hierarchy_context(parent_uuid, sheet_uuid)
    nets = ["3V3", "GND", "SENS_SCLK", "SENS_MOSI", "SENS_MISO", "ICM_nCS", "LSM_nCS",
            "ICM_INT1", "ICM_INT2", "LSM_INT1", "LSM_INT2", "BARO_SCL", "BARO_SDA", "BARO_DRDY"]
    add_hlabels(sch, nets)
    sch.add_text(
        "DISSIMILAR REDUNDANT IMUs\nKeepout from regulator/RF; solid quiet ground, matched axes and mechanical isolation.",
        (80, 16), size=1.1, bold=True,
    )
    add_two_pin(sch, "Device:FerriteBead", "FB20", "600R@100MHz", (55, 42),
                "3V3", "3V3_SENS", "Inductor_SMD:L_0603_1608Metric")
    add_custom(
        sch, "ICM42688P", "U20", "ICM-42688-P", (85, 80),
        {
            "1": "SENS_MISO", "2": None, "3": None, "4": "ICM_INT1",
            "5": "3V3_SENS", "6": "GND", "7": "GND", "8": "3V3_SENS",
            "9": "ICM_INT2", "10": None, "11": None, "12": "ICM_nCS",
            "13": "SENS_SCLK", "14": "SENS_MOSI",
        },
        "TDK InvenSense", "ICM-42688-P",
    )
    add_custom(
        sch, "LSM6DSO32", "U21", "LSM6DSO32TR", (155, 80),
        {
            "1": "SENS_MISO", "2": "3V3_SENS", "3": "3V3_SENS", "4": "LSM_INT1",
            "5": "3V3_SENS", "6": "GND", "7": "GND", "8": "3V3_SENS",
            "9": "LSM_INT2", "10": None, "11": None, "12": "LSM_nCS",
            "13": "SENS_SCLK", "14": "SENS_MOSI",
        },
        "STMicroelectronics", "LSM6DSO32TR",
    )
    add_part(
        sch, "Sensor_Pressure:LPS22DF", "U22", "LPS22DFTR", (225, 80),
        "Package_LGA:ST_HLGA-10_2x2mm_P0.5mm_LayoutBorder3x2y",
        "STMicroelectronics", "LPS22DFTR",
    )
    for pin, net in {
        "1": "3V3_SENS", "2": "BARO_SCL", "3": "GND", "4": "BARO_SDA",
        "5": "GND", "6": "3V3_SENS", "7": "BARO_DRDY", "8": "GND",
        "9": "GND", "10": "3V3_SENS",
    }.items():
        label_pin(sch, "U22", pin, net)
    for ref, net, pos in (
        ("R20", "BARO_SCL", (220, 125)), ("R21", "BARO_SDA", (240, 125)),
    ):
        add_two_pin(sch, "Device:R", ref, "4.7k", pos, "3V3_SENS", net,
                    "Resistor_SMD:R_0603_1608Metric")
    add_two_pin(sch, "Device:R", "R22", "10k", (185, 125), "3V3_SENS", "ICM_nCS",
                "Resistor_SMD:R_0603_1608Metric")
    add_two_pin(sch, "Device:R", "R23", "10k", (200, 125), "3V3_SENS", "LSM_nCS",
                "Resistor_SMD:R_0603_1608Metric")
    for k in range(8):
        add_two_pin(sch, "Device:C", f"C{50 + k}", "100n" if k % 2 == 0 else "2.2u",
                    (60 + k * 25, 150), "3V3_SENS", "GND",
                    "Capacitor_SMD:C_0402_1005Metric" if k % 2 == 0 else "Capacitor_SMD:C_0603_1608Metric")
    add_power_flag(sch, "#FLG20", "3V3_SENS", (245, 42))
    assert_no_label_collisions(sch, "main-board sensors")
    sch.save(MAIN_DIR / "sensors.kicad_sch")


def make_main_gnss(parent_uuid: str, sheet_uuid: str) -> None:
    sch = ksa.create_schematic("main_rev_b")
    sch.set_hierarchy_context(parent_uuid, sheet_uuid)
    nets = [
        "3V3", "3V3_GNSS", "2V8_GNSS", "GND",
        "GNSS_TX", "GNSS_RX", "GNSS_PPS", "GNSS_RESET_N",
    ]
    add_hlabels(sch, nets)
    sch.add_text(
        "LG77L GNSS - QUIET SUPPLY + PASSIVE-ANTENNA REFERENCE\n"
        "VCC=quiet 3.3 V LDO; VCC_IO=2.8 V; V_BCKP remains powered through bead.",
        (70, 16), size=1.1, bold=True,
    )
    gnss_map: dict[str, str | None] = {str(i): None for i in range(1, 44)}
    for pin in ("2", "18", "37", "38", "39", "40", "41", "42", "43"):
        gnss_map[pin] = "GND"
    gnss_map.update(
        {
            "1": "GNSS_RF_IN", "14": "GNSS_RESET_OD", "15": "GNSS_UART_RX_2V8",
            "16": "GNSS_UART_TX_2V8", "19": "3V3_GNSS", "20": "2V8_GNSS",
            "21": "GNSS_VBCKP", "31": "GNSS_PPS",
        }
    )
    add_custom(sch, "LG77L", "U30", "LG77LICMD", (105, 95), gnss_map,
               "Quectel", "LG77LICMD")
    # Cheapest robust UART translation: divider in the 3.3->2.8 direction;
    # 2.8 V output directly meets the RP2354 VIH requirement in the reverse direction.
    add_two_pin(sch, "Device:R", "R30", "1k", (55, 65), "GNSS_TX", "GNSS_UART_RX_2V8",
                "Resistor_SMD:R_0603_1608Metric")
    add_two_pin(sch, "Device:R", "R31", "5.6k", (75, 65), "GNSS_UART_RX_2V8", "GND",
                "Resistor_SMD:R_0603_1608Metric")
    add_two_pin(sch, "Device:R", "R32", "33", (55, 80), "GNSS_UART_TX_2V8", "GNSS_RX",
                "Resistor_SMD:R_0603_1608Metric")
    # RESET_N is driven open drain and cannot overdrive the 2.8 V domain.
    add_part(sch, "Transistor_FET:2N7002", "Q30", "2N7002", (55, 105),
             "Package_TO_SOT_SMD:SOT-23", "Nexperia", "2N7002P")
    label_pin(sch, "Q30", "1", "GNSS_RESET_N")
    label_pin(sch, "Q30", "2", "GND")
    label_pin(sch, "Q30", "3", "GNSS_RESET_OD")
    add_two_pin(sch, "Device:R", "R33", "100k", (75, 105), "GNSS_RESET_N", "GND",
                "Resistor_SMD:R_0603_1608Metric")
    add_two_pin(sch, "Device:R", "R34", "10k", (95, 105), "2V8_GNSS", "GNSS_RESET_OD",
                "Resistor_SMD:R_0603_1608Metric")
    # Quectel passive-antenna reference: DNP/0R/DNP pi matching and an
    # antenna-rated TVS whose maximum capacitance is below 0.6 pF.
    add_custom(sch, "UFL", "J30", "GNSS_UFL", (220, 75),
               {"1": "GNSS_RF_ANT", "2": "GND"}, "Hirose", "U.FL-R-SMT-1(10)")
    add_two_pin(sch, "Device:R", "R35", "0", (190, 75),
                "GNSS_RF_ANT", "GNSS_RF_IN", "Resistor_SMD:R_0402_1005Metric")
    add_two_pin(sch, "Device:C", "C77", "DNP", (185, 92),
                "GNSS_RF_ANT", "GND", "Capacitor_SMD:C_0402_1005Metric")
    add_two_pin(sch, "Device:C", "C78", "DNP", (205, 92),
                "GNSS_RF_IN", "GND", "Capacitor_SMD:C_0402_1005Metric")
    add_part(
        sch, "Device:D_TVS", "D30", "TPD1E0B04DPYR", (225, 92),
        "Package_SON:Texas_DPY0002A_0.6x1mm_P0.65mm",
        "Texas Instruments", "TPD1E0B04DPYR",
    )
    label_pin(sch, "D30", "1", "GNSS_RF_ANT")
    label_pin(sch, "D30", "2", "GND")

    # VCC and backup-domain networks follow Quectel's 10u/100n/33p and
    # 4.7u/100n/33p recommendations.  The rail TVS parts are inexpensive,
    # stocked devices; the RF TVS above is the ultra-low-capacitance part.
    add_two_pin(sch, "Device:FerriteBead", "FB30", "600R@100MHz", (155, 120),
                "3V3", "GNSS_VBCKP", "Inductor_SMD:L_0603_1608Metric",
                "Murata", "BLM18AG601SN1D")
    for ref, value, net, pos in (
        ("C70", "10u", "3V3_GNSS", (170, 120)),
        ("C71", "100n", "3V3_GNSS", (185, 120)),
        ("C72", "33p C0G", "3V3_GNSS", (200, 120)),
        ("C73", "1u", "2V8_GNSS", (215, 120)),
        ("C74", "4.7u", "GNSS_VBCKP", (170, 140)),
        ("C75", "100n", "GNSS_VBCKP", (185, 140)),
        ("C76", "33p C0G", "GNSS_VBCKP", (200, 140)),
    ):
        footprint = (
            "Capacitor_SMD:C_0805_2012Metric"
            if value in ("10u", "4.7u")
            else "Capacitor_SMD:C_0402_1005Metric"
        )
        add_two_pin(sch, "Device:C", ref, value, pos, net, "GND",
                    footprint)
    for ref, rail, pos in (
        ("D31", "3V3_GNSS", (220, 140)),
        ("D32", "GNSS_VBCKP", (235, 140)),
    ):
        add_part(
            sch, "Device:D_TVS", ref, "ESD9B3.3ST5G", pos,
            "Diode_SMD:D_SOD-923", "onsemi", "ESD9B3.3ST5G",
        )
        label_pin(sch, ref, "1", rail)
        label_pin(sch, ref, "2", "GND")
    assert_no_label_collisions(sch, "main-board GNSS")
    sch.save(MAIN_DIR / "gnss.kicad_sch")


def make_main_rf(parent_uuid: str, sheet_uuid: str) -> None:
    sch = ksa.create_schematic("main_rev_b")
    sch.set_hierarchy_context(parent_uuid, sheet_uuid)
    nets = ["3V3", "GND", "RF_SCLK", "RF_MOSI", "RF_MISO", "RF_nCS", "RF_RESET_N",
            "RF_GPIO0", "RF_GPIO2", "RF_GPIO3", "PA_EN", "LNA_EN", "HGM"]
    add_hlabels(sch, nets)
    sch.add_text(
        "915 MHz CC1121 + CC1190\n"
        "Topology and values reproduce TI BOOSTXL-CC1120-90 Rev A. Keep U.FL and DNP tune pads.",
        (70, 16), size=1.1, bold=True,
    )
    cc1121 = {
        "1": "VDD_CC112", "2": "RF_RESET_N", "3": "RF_GPIO3", "4": "RF_GPIO2",
        "5": "VDD_CC112", "6": "CC_DCPL", "7": "RF_MOSI", "8": "RF_SCLK",
        "9": "RF_MISO", "10": "RF_GPIO0", "11": "RF_nCS", "12": "VDD_CC112",
        "13": "VDD_CC112", "14": "CC_RBIAS", "15": "VDD_CC112", "16": None,
        "17": "CC_PA", "18": "CC_TRX_SW", "19": "CC_LNA_P", "20": "CC_LNA_N",
        "21": "CC_DCPL_VCO", "22": "VDD_CC112", "23": "CC_LPF0", "24": "CC_LPF1",
        "25": "VDD_CC112", "26": "CC_DCPL_PFD", "27": "VDD_CC112", "28": "VDD_CC112",
        "29": "CC_DCPL_XOSC", "30": None, "31": None,
        "32": "CC_EXTCLK", "33": "GND",
    }
    add_custom(sch, "CC1121", "U40", "CC1121RHBR", (78, 82), cc1121,
               "Texas Instruments", "CC1121RHBR")
    cc1190 = {
        "1": "GND", "2": "RF_PA_OUT", "3": "GND", "4": "RF_TR_SW",
        "5": "RF_LNA_IN", "6": "HGM", "7": "LNA_EN", "8": "PA_EN",
        "9": "GND", "10": "RF_PA_INOUT", "11": "RF_PA_INOUT", "12": "GND",
        "13": "CC1190_VDD13", "14": "CC1190_BIAS", "15": "CC1190_VDD15",
        "16": "CC1190_VDD16", "17": "GND",
    }
    add_custom(sch, "CC1190", "U41", "CC1190RGVR", (168, 82), cc1190,
               "Texas Instruments", "CC1190RGVR")

    # Independently filtered transceiver and PA rails from the TI reference.
    add_two_pin(sch, "Device:FerriteBead", "L40", "1k@100MHz", (42, 42),
                "3V3", "VDD_CC112", "Inductor_SMD:L_0402_1005Metric",
                "Murata", "BLM15HG102SN1D")
    add_two_pin(sch, "Device:L", "L45", "10n", (62, 42),
                "3V3", "VDD_CC119", "Inductor_SMD:L_0402_1005Metric",
                "Murata", "LQW15AN10NJ00D")
    add_two_pin(sch, "Device:R", "R40", "56k", (45, 125), "CC_RBIAS", "GND",
                "Resistor_SMD:R_0402_1005Metric")
    for ref, source, target, pos in (
        ("R43", "VDD_CC112", "RF_nCS", (45, 145)),
        ("R44", "VDD_CC112", "RF_RESET_N", (60, 145)),
        ("R45", "PA_EN", "GND", (75, 145)),
        ("R46", "LNA_EN", "GND", (90, 145)),
        ("R47", "HGM", "GND", (105, 145)),
    ):
        add_two_pin(sch, "Device:R", ref, "10k" if ref in ("R43", "R44") else "100k",
                    pos, source, target, "Resistor_SMD:R_0402_1005Metric")
    for ref, value, net, pos in (
        ("C80", "220n", "CC_DCPL", (60, 165)),
        ("C81", "10n", "CC_DCPL_VCO", (78, 165)),
        ("C82", "47n", "CC_DCPL_PFD", (96, 165)),
        ("C83", "47n", "CC_DCPL_XOSC", (114, 165)),
    ):
        add_two_pin(sch, "Device:C", ref, value, pos, net, "GND",
                    "Capacitor_SMD:C_0402_1005Metric")

    # CC1121 clock input uses the reference's series 22 pF coupling capacitor.
    add_part(sch, "Oscillator:ASCO", "Y40", "32MHz NT2016SA-32M-TEE3017A",
             (45, 190), "Oscillator:Oscillator_SMD_SiT_PQFN-4Pin_2.0x1.6mm",
             "NDK", "NT2016SA-32M-TEE3017A")
    no_connect_pin(sch, "Y40", "1")
    for pin, net in {"2": "GND", "3": "TCXO_RAW", "4": "VDD_CC112"}.items():
        label_pin(sch, "Y40", pin, net)
    add_two_pin(sch, "Device:C", "C84", "22p C0G", (72, 190), "TCXO_RAW", "CC_EXTCLK",
                "Capacitor_SMD:C_0402_1005Metric")

    # CC1121 PA bias and transmit branch, copied net-for-net from Rev A.
    add_two_pin(sch, "Device:L", "L41", "8.2n", (110, 52), "CC_PA", "CC_PA_BIAS",
                "Inductor_SMD:L_0402_1005Metric", "Murata", "LQW15AN8N2G00D")
    add_two_pin(sch, "Device:R", "R41", "10", (128, 52), "CC_PA_BIAS", "VDD_CC112",
                "Resistor_SMD:R_0402_1005Metric")
    add_two_pin(sch, "Device:C", "C85", "33p C0G", (146, 52), "CC_PA_BIAS", "VDD_CC112",
                "Capacitor_SMD:C_0402_1005Metric")
    add_two_pin(sch, "Device:C", "C86", "15p C0G", (110, 62), "CC_PA", "CC_TX_PRE",
                "Capacitor_SMD:C_0402_1005Metric")
    add_two_pin(sch, "Device:R", "R42", "0 (non-SigFox)", (128, 62), "CC_TX_PRE", "CC_TXPA",
                "Resistor_SMD:R_0402_1005Metric")
    add_custom(
        sch, "LFD21868MMF1D386", "FL40", "LFD21868MMF1D386", (118, 108),
        {
            "1": "GND", "2": "CC1121_RF", "3": None, "4": "GND", "5": "GND",
            "6": "CC_LNA_N", "7": "CC_LNA_P", "8": "CC_TRX_SW",
            "9": "CC_TXPA", "10": "GND",
        },
        "Murata", "LFD21868MMF1D386",
    )
    add_two_pin(sch, "Device:C", "C87", "1.8n U2J", (82, 122), "CC_LPF0", "CC_LPF1",
                "Capacitor_SMD:C_0402_1005Metric")

    # Six-pad EPCOS SAW: RF is on pins 5 and 2; every other pad is ground.
    add_custom(
        sch, "RF_FILTER_4PORT", "FL41", "B39921B3588U410", (145, 132),
        {"1": "GND", "2": "SAW_PA", "3": "GND", "4": "GND",
         "5": "CC1121_RF", "6": "GND"},
        "TDK EPCOS", "B39921B3588U410",
    )
    add_two_pin(sch, "Device:C", "C88", "0.8p C0G", (168, 132), "SAW_PA", "GND",
                "Capacitor_SMD:C_0402_1005Metric")
    add_two_pin(sch, "Device:L", "L42", "10n", (185, 132), "SAW_PA", "RF_PA_INOUT",
                "Inductor_SMD:L_0402_1005Metric", "Murata", "LQW15AN10NJ00D")

    # CC1190 bias, isolated supply pins, and reference decoupling.
    add_two_pin(sch, "Device:R", "R50", "3.3k", (195, 145), "CC1190_BIAS", "GND",
                "Resistor_SMD:R_0402_1005Metric")
    add_two_pin(sch, "Device:L", "L43", "1.5n", (205, 155), "VDD_CC119", "CC1190_VDD13",
                "Inductor_SMD:L_0402_1005Metric", "Murata", "LQG15HS1N5S02D")
    add_two_pin(sch, "Device:R", "R51", "47", (223, 155), "VDD_CC119", "CC1190_VDD15",
                "Resistor_SMD:R_0402_1005Metric")
    add_two_pin(sch, "Device:L", "L44", "15n", (241, 155), "VDD_CC119", "CC1190_VDD16",
                "Inductor_SMD:L_0402_1005Metric", "Murata", "LQW15AN15NJ00D")
    add_two_pin(sch, "Device:C", "C89", "15p C0G", (205, 168), "CC1190_VDD13", "GND",
                "Capacitor_SMD:C_0402_1005Metric")

    # Characterized 915 MHz PA output and LNA input network, net-for-net.
    add_two_pin(sch, "Device:L", "L46", "22n", (205, 52), "VDD_CC119", "RF_PA_OUT",
                "Inductor_SMD:L_0603_1608Metric", "Murata", "LQW18AN22NJ10D")
    add_two_pin(sch, "Device:C", "C90", "47p C0G", (223, 52), "RF_PA_OUT", "MATCH_A",
                "Capacitor_SMD:C_0402_1005Metric")
    add_two_pin(sch, "Device:C", "C91", "3.3p C0G", (241, 62), "MATCH_A", "GND",
                "Capacitor_SMD:C_0402_1005Metric")
    add_two_pin(sch, "Device:L", "L47", "7.5n", (259, 62), "MATCH_A", "RX_COMBINE",
                "Inductor_SMD:L_0402_1005Metric", "Murata", "LQW15AN7N5G00D")
    add_two_pin(sch, "Device:C", "C92", "12p C0G", (277, 62), "RX_COMBINE", "RF_TR_SW",
                "Capacitor_SMD:C_0402_1005Metric")
    add_two_pin(sch, "Device:C", "C93", "12p C0G", (295, 62), "RF_TR_SW", "RF_LNA_IN",
                "Capacitor_SMD:C_0402_1005Metric")
    add_two_pin(sch, "Device:L", "L48", "2.9n", (259, 82), "MATCH_A", "MATCH_B",
                "Inductor_SMD:L_0402_1005Metric", "Murata", "LQW15AN2N9C00D")
    add_two_pin(sch, "Device:C", "C94", "2.4p DNP", (277, 82), "MATCH_A", "MATCH_B",
                "Capacitor_SMD:C_0402_1005Metric")
    add_two_pin(sch, "Device:C", "C95", "7.5p C0G", (295, 82), "MATCH_B", "GND",
                "Capacitor_SMD:C_0402_1005Metric")
    add_two_pin(sch, "Device:L", "L49", "9.1n", (313, 82), "MATCH_B", "ANT_915",
                "Inductor_SMD:L_0402_1005Metric", "Murata", "LQW15AN9N1G00D")
    add_two_pin(sch, "Device:C", "C96", "2.7p DNP", (331, 82), "ANT_915", "GND",
                "Capacitor_SMD:C_0402_1005Metric")
    add_custom(sch, "UFL", "J40", "915MHz_ANT", (340, 62),
               {"1": "ANT_915", "2": "GND"}, "Hirose", "U.FL-R-SMT-1(10)")

    # Local rail decoupling values from the TI BOM.
    for k, value in enumerate(("2.2u", "100n", "47n", "47n", "47n", "10n", "100p")):
        add_two_pin(sch, "Device:C", f"C{100 + k}", value,
                    (105 + k * 18, 205), "VDD_CC112", "GND",
                    "Capacitor_SMD:C_0603_1608Metric" if value == "2.2u" else "Capacitor_SMD:C_0402_1005Metric")
    for k, value in enumerate(("10u", "27p", "47p", "1n", "1u", "100n")):
        add_two_pin(sch, "Device:C", f"C{110 + k}", value,
                    (235 + k * 18, 205), "VDD_CC119", "GND",
                    "Capacitor_SMD:C_0603_1608Metric" if value in ("10u", "1u") else "Capacitor_SMD:C_0402_1005Metric")

    add_power_flag(sch, "#FLG40", "VDD_CC112", (325, 145))
    assert_no_label_collisions(sch, "main-board 915 MHz RF")
    sch.save(MAIN_DIR / "rf_915.kicad_sch")


def _make_main_interfaces_legacy(parent_uuid: str, sheet_uuid: str) -> None:
    sch = ksa.create_schematic("main_rev_b")
    sch.set_hierarchy_context(parent_uuid, sheet_uuid)
    nets = ["3V3", "GND", "CANH", "CANL", "CAN_TX", "CAN_RX"]
    add_hlabels(sch, nets)
    add_custom(
        sch, "TCAN3413", "U50", "TCAN3413DR", (95, 75),
        {"1": "CAN_TX", "2": "GND", "3": "3V3", "4": "CAN_RX", "5": "3V3",
         "6": "CANL", "7": "CANH", "8": "GND"},
        "Texas Instruments", "TCAN3413DR",
    )
    add_two_pin(sch, "Device:R", "R120", "120 DNP", (140, 65), "CANH", "CANL",
                "Resistor_SMD:R_0603_1608Metric")
    add_two_pin(sch, "Device:C", "C120", "100n", (140, 85), "3V3", "GND",
                "Capacitor_SMD:C_0603_1608Metric")
    sch.add_text("CAN FD physical layer; termination populated only at a bus end.", (70, 25), size=1.1, bold=True)
    sch.save(MAIN_DIR / "interfaces.kicad_sch")


def make_main_interfaces(parent_uuid: str, sheet_uuid: str) -> None:
    sch = ksa.create_schematic("main_rev_b")
    sch.set_hierarchy_context(parent_uuid, sheet_uuid)
    nets = [
        "3V3", "GND", "CANH", "CANL",
        "CAN_SCK", "CAN_MOSI", "CAN_MISO", "CAN_nCS", "CAN_INT",
    ]
    add_hlabels(sch, nets)
    sch.add_text(
        "HARDWARE CAN-FD\n"
        "MCP2518FD provides the protocol controller; TCAN3413, CMC and TVS protect the cable interface.",
        (70, 25), size=1.1, bold=True,
    )
    add_can_fd_interface(
        sch, mcp_ref="U51", xcvr_ref="U50", choke_ref="FL50",
        esd_ref="D50", crystal_ref="Y50",
        resistor_base=120, capacitor_base=120,
        ground="GND", rail="3V3", bus_h="CANH", bus_l="CANL",
        spi_sck="CAN_SCK", spi_mosi="CAN_MOSI", spi_miso="CAN_MISO",
        spi_ncs="CAN_nCS", irq="CAN_INT", position=(80, 75),
    )
    assert_no_label_collisions(sch, "main-board CAN-FD interface")
    sch.save(MAIN_DIR / "interfaces.kicad_sch")


def make_main_top() -> None:
    top = ksa.create_schematic("main_rev_b")
    top.add_text(
        "REV-B FLIGHT-CONTROL BOARD\n"
        "Physically and electrically distinct from the ESC; CAN + protected 5 V are the only normal interconnects.",
        (42, 12), size=1.4, bold=True,
    )
    sheet_specs = [
        ("Power and USB", "power_usb.kicad_sch", (15.24, 25.40), (83.82, 45.72),
         ["CAN_5V", "CAN_GND", "CANH", "CANL", "SYS_5V", "3V3",
          "3V3_GNSS", "2V8_GNSS", "GNSS_PWR_EN", "1V1",
          "VREG_LX", "VREG_AVDD", "3V3_ADC", "GND", "USB_DP", "USB_DM"]),
        ("RP2354B MCU", "mcu.kicad_sch", (106.68, 22.86), (180.34, 58.42),
         ["3V3", "1V1", "VREG_LX", "VREG_AVDD", "3V3_ADC", "GND", "USB_DP", "USB_DM",
          "SENS_SCLK", "SENS_MOSI", "SENS_MISO", "ICM_nCS", "LSM_nCS",
          "ICM_INT1", "ICM_INT2", "LSM_INT1", "LSM_INT2", "BARO_SCL", "BARO_SDA",
          "BARO_DRDY", "GNSS_TX", "GNSS_RX", "GNSS_PPS", "GNSS_RESET_N",
          "GNSS_PWR_EN",
          "RF_SCLK", "RF_MOSI", "RF_MISO", "RF_nCS", "RF_RESET_N", "RF_GPIO0",
          "RF_GPIO2", "RF_GPIO3", "PA_EN", "LNA_EN", "HGM",
          "CAN_SCK", "CAN_MOSI", "CAN_MISO", "CAN_nCS", "CAN_INT"]),
        ("Redundant sensors", "sensors.kicad_sch", (15.24, 91.44), (83.82, 45.72),
         ["3V3", "GND", "SENS_SCLK", "SENS_MOSI", "SENS_MISO", "ICM_nCS", "LSM_nCS",
          "ICM_INT1", "ICM_INT2", "LSM_INT1", "LSM_INT2", "BARO_SCL", "BARO_SDA", "BARO_DRDY"]),
        ("GNSS", "gnss.kicad_sch", (106.68, 91.44), (83.82, 45.72),
         ["3V3", "3V3_GNSS", "2V8_GNSS", "GND",
          "GNSS_TX", "GNSS_RX", "GNSS_PPS", "GNSS_RESET_N"]),
        ("915 MHz RF", "rf_915.kicad_sch", (198.12, 91.44), (88.90, 45.72),
         ["3V3", "GND", "RF_SCLK", "RF_MOSI", "RF_MISO", "RF_nCS", "RF_RESET_N",
          "RF_GPIO0", "RF_GPIO2", "RF_GPIO3", "PA_EN", "LNA_EN", "HGM"]),
        ("CAN interface", "interfaces.kicad_sch", (106.68, 149.86), (83.82, 35.56),
         ["3V3", "GND", "CANH", "CANL",
          "CAN_SCK", "CAN_MOSI", "CAN_MISO", "CAN_nCS", "CAN_INT"]),
    ]
    uuids: dict[str, str] = {}
    for name, filename, pos, size, nets in sheet_specs:
        uuids[filename] = add_sheet_with_pins(top, name, filename, pos, size, nets, "main_rev_b")
    top.save(MAIN_DIR / "main_rev_b.kicad_sch")
    make_main_power(top.uuid, uuids["power_usb.kicad_sch"])
    make_main_mcu(top.uuid, uuids["mcu.kicad_sch"])
    make_main_sensors(top.uuid, uuids["sensors.kicad_sch"])
    make_main_gnss(top.uuid, uuids["gnss.kicad_sch"])
    make_main_rf(top.uuid, uuids["rf_915.kicad_sch"])
    make_main_interfaces(top.uuid, uuids["interfaces.kicad_sch"])


def write_projects(board: str) -> None:
    # Empty KiCad project JSON is valid and lets KiCad apply its current defaults.
    # Net classes and board-specific rules are written by the PCB generator.
    projects = []
    if board in ("esc", "both"):
        projects.append((ESC_DIR, "esc_rev_b"))
    if board in ("main", "both"):
        projects.append((MAIN_DIR, "main_rev_b"))
    for directory, name in projects:
        project = directory / f"{name}.kicad_pro"
        if not project.exists():
            project.write_text("{}\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--board", choices=("esc", "main", "both"), default="both")
    args = parser.parse_args()
    directories = []
    if args.board in ("esc", "both"):
        directories.append(ESC_DIR)
    if args.board in ("main", "both"):
        directories.append(MAIN_DIR)
    for directory in directories:
        directory.mkdir(parents=True, exist_ok=True)
        write_symbol_library(directory)
        write_tables(directory)
    # Register both generated libraries for the current Python process.
    cache = ksa.get_symbol_cache()
    # The helper library keeps a persistent symbol-content cache.  Clear only
    # those cached symbol bodies so generated schematics always embed the
    # installed KiCad 9 library revision (not a stale pre-upgrade RP2354B).
    cache.clear_cache()
    if args.board in ("esc", "both"):
        cache.add_library_path(ESC_DIR / "revb.kicad_sym")
        make_esc_top()
    if args.board in ("main", "both"):
        cache.add_library_path(MAIN_DIR / "revb.kicad_sym")
        make_main_top()
    write_projects(args.board)
    manifest = {
        "generator": str(Path(__file__).relative_to(ROOT)),
        "esc_root": "hardware/esc/rev_b/esc_rev_b.kicad_sch",
        "main_root": "hardware/main/rev_b/main_rev_b.kicad_sch",
        "note": "Generated Rev-B projects are distinct and do not overwrite original designs.",
    }
    (ROOT / "docs" / "hardware" / "REV_B_GENERATION.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
