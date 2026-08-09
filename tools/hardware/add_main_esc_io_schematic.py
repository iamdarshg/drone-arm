#!/usr/bin/env python3
"""Surgically add six standard PWM ESC connectors to Main Rev B schematics."""

from __future__ import annotations

import re
import uuid
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
MAIN_DIR = ROOT / "hardware" / "main" / "rev_b"
MCU_PATH = MAIN_DIR / "mcu.kicad_sch"
TOP_PATH = MAIN_DIR / "main_rev_b.kicad_sch"


def new_uuid() -> str:
    return str(uuid.uuid4())


def number(value: float) -> str:
    rounded = round(value, 4)
    if rounded == int(rounded):
        return str(int(rounded))
    return f"{rounded:.4f}".rstrip("0").rstrip(".")


def sexpr_end(text: str, start: int) -> int:
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
                return index + 1
    raise ValueError("Unbalanced KiCad S-expression")


def find_block(text: str, start_pattern: str, contains: str) -> tuple[int, int, str]:
    for match in re.finditer(start_pattern, text, flags=re.MULTILINE):
        start = match.start()
        end = sexpr_end(text, start)
        block = text[start:end]
        if contains in block:
            return start, end, block
    raise ValueError(f"Could not find block containing {contains!r}")


def wire_block(x1: float, y1: float, x2: float, y2: float) -> str:
    return (
        "\t(wire\n"
        "\t\t(pts\n"
        f"\t\t\t(xy {number(x1)} {number(y1)}) (xy {number(x2)} {number(y2)})\n"
        "\t\t)\n"
        "\t\t(stroke\n"
        "\t\t\t(width 0)\n"
        "\t\t\t(type default)\n"
        "\t\t)\n"
        f"\t\t(uuid \"{new_uuid()}\")\n"
        "\t)\n"
    )


def label_block(name: str, x: float, y: float, rotation: int = 0, size: float = 0.9) -> str:
    return (
        f"\t(label \"{name}\"\n"
        f"\t\t(at {number(x)} {number(y)} {rotation})\n"
        "\t\t(effects\n"
        "\t\t\t(font\n"
        f"\t\t\t\t(size {number(size)} {number(size)})\n"
        "\t\t\t)\n"
        "\t\t\t(justify left bottom)\n"
        "\t\t)\n"
        f"\t\t(uuid \"{new_uuid()}\")\n"
        "\t)\n"
    )


def hierarchical_label_block(name: str, y: float) -> str:
    return (
        f"\n\t(hierarchical_label \"{name}\"\n"
        "\t\t(shape input)\n"
        f"\t\t(at 20.32 {number(y)} 0.0000)\n"
        "\t\t(effects\n"
        "\t\t\t(font\n"
        "\t\t\t\t(size 0.9 0.9)\n"
        "\t\t\t)\n"
        "\t\t\t(justify left)\n"
        "\t\t)\n"
        f"\t\t(uuid \"{new_uuid()}\")\n"
        "\t)"
    )


def sheet_pin_block(name: str, x: float, y: float, rotation: int, justify: str) -> str:
    return (
        f"\t\t(pin \"{name}\" bidirectional\n"
        f"\t\t\t(at {number(x)} {number(y)} {rotation})\n"
        f"\t\t\t(uuid \"{new_uuid()}\")\n"
        "\t\t\t(effects\n"
        "\t\t\t\t(font\n"
        "\t\t\t\t\t(size 1.27 1.27)\n"
        "\t\t\t\t)\n"
        f"\t\t\t\t(justify {justify})\n"
        "\t\t\t)\n"
        "\t\t)\n"
    )


AT_PATTERN = re.compile(r"\(at (-?\d+(?:\.\d+)?) (-?\d+(?:\.\d+)?) ([^)]+)\)")
UUID_PATTERN = re.compile(r'\(uuid "[0-9a-f-]+"\)')


def clone_connector(template: str, reference: str, value: str, x: float, y: float) -> str:
    dx = x - 44.45
    dy = y - 149.86

    def shift(match: re.Match[str]) -> str:
        return (
            f"(at {number(float(match.group(1)) + dx)} "
            f"{number(float(match.group(2)) + dy)} {match.group(3)})"
        )

    block = AT_PATTERN.sub(shift, template)
    block = block.replace('"J10"', f'"{reference}"')
    block = block.replace('"SWD"', f'"{value}"')
    block = block.replace(
        '"Connector_JST:JST_SH_SM03B-SRSS-TB_1x03-1MP_P1.00mm_Horizontal"',
        '"Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical"',
    )
    return UUID_PATTERN.sub(lambda _: f'(uuid "{new_uuid()}")', block)


def add_mcu_connectors() -> None:
    text = MCU_PATH.read_text(encoding="utf-8")
    if 'property "Reference" "J60"' in text:
        raise RuntimeError("PWM ESC connectors are already present in mcu.kicad_sch")

    symbol_start = r'^\t\(symbol\s*$'
    _, j10_end, j10 = find_block(text, symbol_start, 'property "Reference" "J10"')
    symbols = []
    connector_labels = []
    for index in range(1, 7):
        x = 35.56 + (index - 1) * 35.56
        y = 195.58
        symbols.append(clone_connector(j10, f"J{59 + index}", f"ESC_PWM_{index}", x, y))
        pin_x = x - 5.08
        connector_labels.extend(
            (
                label_block("CAN_GND", pin_x, y - 2.54, 180),
                label_block("CAN_5V", pin_x, y, 180),
                label_block(f"PWM_ESC{index}", pin_x, y + 2.54, 180),
            )
        )
    text = text[:j10_end] + "\n" + "\n".join(symbols) + text[j10_end:]

    for y in (93.98, 96.52, 99.06, 101.6, 104.14, 106.68):
        pattern = re.compile(
            rf"\n\t\(no_connect\n\t\t\(at 87\.63 {re.escape(number(y))}\)\n"
            rf"\t\t\(uuid \"[0-9a-f-]+\"\)\n\t\)",
        )
        text, count = pattern.subn("", text, count=1)
        if count != 1:
            raise RuntimeError(f"Expected one no-connect marker at GPIO position {y}")

    first_label = text.index("\n\t(label ") + 1
    extra_wires = wire_block(20.32, 124.46, 25.40, 124.46)
    extra_wires += wire_block(20.32, 127.00, 25.40, 127.00)
    text = text[:first_label] + extra_wires + text[first_label:]

    first_hlabel = text.index("\n\t(hierarchical_label ") + 1
    pwm_labels = "".join(
        label_block(f"PWM_ESC{index}", 87.63, y, 180)
        for index, y in enumerate((93.98, 96.52, 99.06, 101.6, 104.14, 106.68), start=1)
    )
    local_labels = (
        label_block("CAN_5V", 25.40, 124.46, 0, 0.8)
        + label_block("CAN_GND", 25.40, 127.00, 0, 0.8)
        + pwm_labels
        + "".join(connector_labels)
    )
    text = text[:first_hlabel] + local_labels + text[first_hlabel:]

    can_int_start = text.index('\n\t(hierarchical_label "CAN_INT"') + 1
    can_int_end = sexpr_end(text, can_int_start)
    hlabels = hierarchical_label_block("CAN_5V", 124.46)
    hlabels += hierarchical_label_block("CAN_GND", 127.00)
    text = text[:can_int_end] + hlabels + text[can_int_end:]

    MCU_PATH.write_text(text, encoding="utf-8")


def add_top_sheet_pins() -> None:
    text = TOP_PATH.read_text(encoding="utf-8")
    old_text = (
        "Physically and electrically distinct from the ESC; CAN + protected 5 V are the only normal interconnects."
    )
    new_text = "CAN + protected 5 V serve the custom ESC; six PWM outputs support standard ESCs."
    if old_text not in text:
        raise RuntimeError("Expected Main Rev B title text was not found")
    text = text.replace(old_text, new_text, 1)

    sheet_start_pattern = r'^\t\(sheet\s*$'
    sheet_start, sheet_end, sheet = find_block(
        text, sheet_start_pattern, 'property "Sheetname" "RP2354B MCU"'
    )
    if 'pin "CAN_5V"' in sheet or 'pin "CAN_GND"' in sheet:
        raise RuntimeError("CAN power pins are already present on the MCU sheet")
    insert_at = sheet.index("\t\t(instances")
    pins = sheet_pin_block("CAN_5V", 287.02, 76.20, 0, "right")
    pins += sheet_pin_block("CAN_GND", 106.68, 78.74, 180, "left")
    sheet = sheet[:insert_at] + pins + sheet[insert_at:]
    text = text[:sheet_start] + sheet + text[sheet_end:]

    first_label = text.index("\n\t(label ") + 1
    wires = wire_block(287.02, 76.20, 289.56, 76.20)
    wires += wire_block(106.68, 78.74, 104.14, 78.74)
    text = text[:first_label] + wires + text[first_label:]

    title_index = text.index('\n\t(text "REV-B FLIGHT-CONTROL BOARD') + 1
    labels = label_block("CAN_5V", 289.56, 76.20, 0, 0.8)
    labels += label_block("CAN_GND", 104.14, 78.74, 0, 0.8)
    text = text[:title_index] + labels + text[title_index:]

    TOP_PATH.write_text(text, encoding="utf-8")


def main() -> None:
    add_mcu_connectors()
    add_top_sheet_pins()


if __name__ == "__main__":
    main()
