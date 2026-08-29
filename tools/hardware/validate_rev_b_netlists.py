#!/usr/bin/env python3
"""Deterministic connectivity checks for the generated Rev-B KiCad netlists.

ERC catches pin-level electrical-rule problems.  These checks cover the
design-intent relationships that ERC cannot infer: Kelvin shunt polarity,
timer-to-driver assignments, local ADC channels, hardware break/arm paths,
and isolation between the supervisor ground and each motor-cell return.
"""

from __future__ import annotations

import argparse
import json
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Node:
    ref: str
    pin: str
    function: str


class Netlist:
    def __init__(self, path: Path) -> None:
        self.path = path
        root = ET.parse(path).getroot()
        self.source = root.findtext("./design/source", default="")
        self.components: dict[str, str] = {}
        for comp in root.findall("./components/comp"):
            self.components[comp.attrib["ref"]] = comp.findtext("value", default="")
        self.nets: dict[str, set[Node]] = {}
        for net in root.findall("./nets/net"):
            self.nets[net.attrib["name"]] = {
                Node(
                    node.attrib["ref"],
                    node.attrib["pin"],
                    node.attrib.get("pinfunction", ""),
                )
                for node in net.findall("node")
            }

    def suffix(self, suffix: str) -> set[Node]:
        matches = [nodes for name, nodes in self.nets.items() if name.endswith(suffix)]
        if len(matches) != 1:
            raise AssertionError(
                f"{self.path.name}: expected exactly one net ending {suffix!r}, "
                f"found {len(matches)}"
            )
        return matches[0]

    def value(self, ref: str) -> str:
        if ref not in self.components:
            raise AssertionError(f"{self.path.name}: missing component {ref}")
        return self.components[ref]


class Audit:
    def __init__(self) -> None:
        self.checks: list[dict[str, object]] = []

    def check(self, name: str, condition: bool, detail: str) -> None:
        self.checks.append(
            {"name": name, "status": "PASS" if condition else "FAIL", "detail": detail}
        )

    def require_nodes(
        self, netlist: Netlist, suffix: str, expected: set[tuple[str, str]], name: str
    ) -> None:
        try:
            nodes = netlist.suffix(suffix)
            actual = {(node.ref, node.pin) for node in nodes}
            missing = sorted(expected - actual)
            self.check(name, not missing, f"{suffix}: missing={missing or 'none'}")
        except AssertionError as exc:
            self.check(name, False, str(exc))

    @property
    def passed(self) -> bool:
        return all(check["status"] == "PASS" for check in self.checks)


def audit_esc(netlist: Netlist, audit: Audit) -> None:
    audit.check(
        "ESC source identity",
        netlist.source.lower().endswith(r"hardware\esc\rev_b\esc_rev_b.kicad_sch"),
        netlist.source,
    )
    expected_counts = {
        "STM32G431CBT6": 6,
        "DRV8353SRTAR": 6,
        "IPTC014N10NM5": 36,
        "0.5m 1% 5W": 24,
        "INA296A2IDR": 6,
        "ISO6731FDWR": 6,
        "RFM-0505S": 6,
        "TPS70933DBVR": 6,
        "SM8S51A": 7,
        "USBLC6-2SC6": 8,
        "6P6T BBM SERVICE SELECTOR": 1,
    }
    for value, expected in expected_counts.items():
        actual = sum(component == value for component in netlist.components.values())
        audit.check(
            f"ESC population: {value}",
            actual == expected,
            f"expected={expected}, actual={actual}",
        )

    obsolete_caps = {f"C{1000 + motor * 100 + 30}" for motor in range(1, 7)}
    present_obsolete = sorted(obsolete_caps & netlist.components.keys())
    audit.check(
        "External motor bulk capacitors are absent",
        not present_obsolete,
        f"present={present_obsolete or 'none'}",
    )
    audit.check(
        "Controller and service USB interfaces are populated",
        {"J201", "J202", "J203", "J204", "SW201", "U208", "U209"}
        <= netlist.components.keys(),
        "requires controller SWD, two USB-C ports, selected-cell control header, selector, and ESD",
    )
    audit.require_nodes(
        netlist, "/CTRL_USB_DM_MCU", {("U201", "66"), ("R206", "2")},
        "ESC supervisor: controller USB DM reaches RP2354B",
    )
    audit.require_nodes(
        netlist, "/CTRL_USB_DP_MCU", {("U201", "67"), ("R207", "2")},
        "ESC supervisor: controller USB DP reaches RP2354B",
    )

    for motor in range(1, 7):
        base = 1000 + motor * 100
        driver = f"U{base + 1}"
        mcu = f"U{base + 2}"
        isolator = f"U{base + 3}"
        ldo = f"U{base + 4}"
        bus_amp = f"U{base + 5}"
        arm_gate = f"U{base + 6}"
        shunts = [f"R{base + offset}" for offset in range(20, 24)]

        expected_values = {
            driver: "DRV8353SRTAR",
            mcu: "STM32G431CBT6",
            isolator: "ISO6731FDWR",
            ldo: "TPS70933DBVR",
            bus_amp: "INA296A2IDR",
            arm_gate: "SN74LVC1G08DBVR",
            f"PS{base + 1}": "RFM-0505S",
            **{ref: "0.5m 1% 5W" for ref in shunts},
        }
        actual_values = {ref: netlist.value(ref) for ref in expected_values}
        audit.check(
            f"M{motor}: local controller/current/isolation population",
            actual_values == expected_values,
            f"{len(actual_values)} required parts present",
        )

        phase_data = {
            "A": (23, 8, 30, 27, 32, 33),
            "B": (22, 9, 31, 28, 34, 35),
            "C": (21, 10, 32, 29, 36, 37),
        }
        for index, (phase, pins) in enumerate(phase_data.items()):
            drv_csa_pin, adc_pin, pwm_hi_pin, pwm_lo_pin, drv_hi_pin, drv_lo_pin = pins
            shunt = f"R{base + 20 + index}"
            audit.require_nodes(
                netlist,
                f"/M{motor}_SH{phase}_P",
                {(shunt, "3"), (driver, str(9 if phase == "A" else 12 if phase == "B" else 19))},
                f"M{motor}: phase {phase} Kelvin plus",
            )
            audit.require_nodes(
                netlist,
                f"/M{motor}_SH{phase}_N",
                {(shunt, "4"), (driver, str(10 if phase == "A" else 11 if phase == "B" else 20))},
                f"M{motor}: phase {phase} Kelvin minus",
            )
            audit.require_nodes(
                netlist,
                f"/M{motor}_CSA_{phase}_RAW",
                {(driver, str(drv_csa_pin)), (f"R{base + 30 + index}", "1")},
                f"M{motor}: phase {phase} amplifier output",
            )
            audit.require_nodes(
                netlist,
                f"/M{motor}_CSA_{phase}",
                {(mcu, str(adc_pin)), (f"R{base + 30 + index}", "2"),
                 (f"C{base + 20 + index}", "1")},
                f"M{motor}: phase {phase} filtered ADC",
            )
            audit.require_nodes(
                netlist,
                f"/M{motor}_PWM_{phase}H",
                {(mcu, str(pwm_hi_pin)), (driver, str(drv_hi_pin))},
                f"M{motor}: phase {phase} high PWM",
            )
            audit.require_nodes(
                netlist,
                f"/M{motor}_PWM_{phase}L",
                {(mcu, str(pwm_lo_pin)), (driver, str(drv_lo_pin))},
                f"M{motor}: phase {phase} low PWM",
            )

        bus_shunt = f"R{base + 23}"
        audit.require_nodes(
            netlist,
            f"/M{motor}_BUS_SH_P",
            {(bus_shunt, "3"), (bus_amp, "8")},
            f"M{motor}: bus Kelvin plus",
        )
        audit.require_nodes(
            netlist,
            f"/M{motor}_BUS_SH_N",
            {(bus_shunt, "4"), (bus_amp, "1")},
            f"M{motor}: bus Kelvin minus",
        )
        audit.require_nodes(
            netlist,
            f"/M{motor}_BUS_CURRENT_RAW",
            {(bus_amp, "5"), (f"R{base + 33}", "1")},
            f"M{motor}: bus amplifier output",
        )
        audit.require_nodes(
            netlist,
            f"/M{motor}_BUS_CURRENT",
            {(mcu, "11"), (f"R{base + 33}", "2"), (f"C{base + 23}", "1")},
            f"M{motor}: filtered bus-current ADC",
        )
        audit.require_nodes(
            netlist,
            f"/M{motor}_BATN",
            {(bus_amp, "2"), (bus_amp, "4"), (bus_amp, "7")},
            f"M{motor}: INA296 ground, reserved pin, and REF1",
        )
        audit.require_nodes(
            netlist,
            f"/M{motor}_3V3A",
            {(bus_amp, "3"), (bus_amp, "6")},
            f"M{motor}: INA296 REF2 and supply",
        )
        audit.require_nodes(
            netlist,
            f"/M{motor}_DRV_nFAULT",
            {(driver, "26"), (mcu, "22")},
            f"M{motor}: driver fault to TIM1 break",
        )
        audit.require_nodes(
            netlist, f"/M{motor}_USB_DM", {(mcu, "33")},
            f"M{motor}: native USB DM",
        )
        audit.require_nodes(
            netlist, f"/M{motor}_USB_DP", {(mcu, "34")},
            f"M{motor}: native USB DP",
        )
        audit.require_nodes(
            netlist, f"/M{motor}_ARM_ISO", {(mcu, "25"), (arm_gate, "1")},
            f"M{motor}: isolated arm permission moved to PB11",
        )
        audit.check(
            f"M{motor}: SWD retained and cell-side USB ESD present",
            {f"J{base + 3}", f"U{base + 7}"} <= netlist.components.keys()
            and f"J{base + 4}" not in netlist.components,
            "local SWD and cell-side ESD present; no redundant per-cell USB connector",
        )
        audit.require_nodes(
            netlist,
            f"/M{motor}_DRV_ENABLE",
            {(driver, "31"), (arm_gate, "4")},
            f"M{motor}: hardware-gated driver enable",
        )
        audit.require_nodes(
            netlist,
            f"/M{motor}_LDO_EN",
            {
                (ldo, "3"),
                (f"R{base + 58}", "2"),
                (f"R{base + 59}", "1"),
            },
            f"M{motor}: divided isolated-LDO enable",
        )
        audit.require_nodes(
            netlist,
            f"/M{motor}_5VI",
            {(ldo, "1"), (f"R{base + 58}", "1")},
            f"M{motor}: raw isolated rail reaches TPS709 IN, not EN",
        )
        audit.require_nodes(
            netlist,
            f"/M{motor}_3V3A",
            {(driver, "24")},
            f"M{motor}: DRV8353 VREF uses quiet local 3.3 V",
        )

        # No high-current or local-control device is allowed on supervisor DGND.
        dgnd_nodes = netlist.suffix("/DGND")
        forbidden_prefixes = (
            driver,
            mcu,
            ldo,
            bus_amp,
            arm_gate,
            f"Q{base + 1}",
            f"Q{base + 2}",
            f"Q{base + 3}",
            f"Q{base + 4}",
            f"Q{base + 5}",
            f"Q{base + 6}",
            *shunts,
        )
        offenders = sorted(
            {(node.ref, node.pin) for node in dgnd_nodes if node.ref in forbidden_prefixes}
        )
        audit.check(
            f"M{motor}: no local power/control bond to DGND",
            not offenders,
            f"offenders={offenders or 'none'}",
        )

    for left in range(1, 7):
        for right in range(left + 1, 7):
            combined = [
                name
                for name in netlist.nets
                if f"M{left}_" in name and f"M{right}_" in name
            ]
            audit.check(
                f"M{left}/M{right}: motor-cell nets remain separate",
                not combined,
                f"shared={combined or 'none'}",
            )

    for motor in range(1, 7):
        for signal, pole in (("DP", 1), ("DM", 2), ("GND", 3), ("VBUS", 4), ("BOOT0", 5), ("NRST", 6)):
            selector_pin = str(6 + (motor - 1) * 6 + pole)
            audit.require_nodes(
                netlist, f"/M{motor}_USB_{signal}", {("SW201", selector_pin)},
                f"Selector position {motor}: {signal} reaches only motor {motor}",
            )

    try:
        ctrl_vbus = {(node.ref, node.pin) for node in netlist.suffix("/CTRL_USB_VBUS")}
        forbidden = {node for node in ctrl_vbus if node[0] in {"U201", "L201", "U204", "U205", "U206", "U207"}}
        audit.check("Controller USB VBUS cannot power ESC rails", not forbidden,
                    f"forbidden={sorted(forbidden) or 'none'}")
    except AssertionError as exc:
        audit.check("Controller USB VBUS cannot power ESC rails", False, str(exc))

    audit.require_nodes(
        netlist,
        "/3V3",
        {(f"C{ref}", "1") for ref in (*range(205, 210), *range(215, 219))},
        "ESC supervisor: nine RP2354B 3.3 V bypass positions",
    )
    audit.require_nodes(
        netlist,
        "/1V1",
        {(f"C{ref}", "1") for ref in range(210, 213)},
        "ESC supervisor: three RP2354B DVDD bypass positions",
    )
    audit.require_nodes(
        netlist,
        "/3V3_ADC",
        {("C219", "1")},
        "ESC supervisor: local ADC_AVDD high-frequency bypass",
    )


def audit_main(netlist: Netlist, audit: Audit) -> None:
    audit.check(
        "Control-board source identity",
        netlist.source.lower().endswith(r"hardware\main\rev_b\main_rev_b.kicad_sch"),
        netlist.source,
    )
    required = {
        "RP2354B": 1,
        "MCP2518FD-H/QBB": 1,
        "TCAN3413DR": 1,
        "PESD2CANFD24L-T": 1,
        "LG77LICMD": 1,
        "TLV75528PDBVR": 1,
        "TLV75533PDBVR": 1,
        "TPD1E0B04DPYR": 1,
        "ESD9B3.3ST5G": 2,
    }
    for value, expected in required.items():
        actual = sum(component == value for component in netlist.components.values())
        audit.check(
            f"Control-board population: {value}",
            actual == expected,
            f"expected={expected}, actual={actual}",
        )
    esc_power_parts = {
        "DRV8353SRTAR",
        "IPTC014N10NM5",
        "INA296A2IDR",
        "RFM-0505S",
    }
    offenders = sorted(
        ref for ref, value in netlist.components.items() if value in esc_power_parts
    )
    audit.check(
        "Control board remains distinct from ESC power stages",
        not offenders,
        f"ESC-only parts={offenders or 'none'}",
    )
    audit.require_nodes(
        netlist,
        "/3V3",
        {(f"C{ref}", "1") for ref in range(30, 39)},
        "Control-board MCU: nine RP2354B 3.3 V bypass positions",
    )
    audit.require_nodes(
        netlist,
        "/1V1",
        {(f"C{ref}", "1") for ref in range(39, 42)},
        "Control-board MCU: three RP2354B DVDD bypass positions",
    )
    audit.require_nodes(
        netlist,
        "/3V3_ADC",
        {("C42", "1")},
        "Control-board MCU: local ADC_AVDD high-frequency bypass",
    )
    audit.require_nodes(
        netlist,
        "/U51_TXCAN",
        {("U51", "1"), ("U50", "1")},
        "Control-board CAN TX path",
    )
    audit.require_nodes(
        netlist,
        "/U51_RXCAN",
        {("U51", "2"), ("U50", "4")},
        "Control-board CAN RX path",
    )
    audit.require_nodes(
        netlist,
        "/3V3_GNSS",
        {("U4", "5"), ("U30", "19"), ("C70", "1"), ("C71", "1"),
         ("C72", "1"), ("D31", "1")},
        "GNSS quiet VCC rail and local 10u/100n/33p protection",
    )
    audit.require_nodes(
        netlist,
        "/2V8_GNSS",
        {("U3", "5"), ("U30", "20"), ("C73", "1"), ("R34", "1")},
        "GNSS 2.8 V I/O rail",
    )
    audit.require_nodes(
        netlist,
        "/GNSS_VBCKP",
        {("FB30", "2"), ("U30", "21"), ("C74", "1"), ("C75", "1"),
         ("C76", "1"), ("D32", "1")},
        "GNSS always-on backup rail and local 4.7u/100n/33p protection",
    )
    audit.require_nodes(
        netlist,
        "/GNSS_RF_ANT",
        {("J30", "1"), ("R35", "1"), ("C77", "1"), ("D30", "1")},
        "GNSS antenna connector, pi pad, and ultra-low-C TVS",
    )
    audit.require_nodes(
        netlist,
        "/GNSS_RF_IN",
        {("U30", "1"), ("R35", "2"), ("C78", "1")},
        "GNSS module-side pi matching node",
    )
    audit.require_nodes(
        netlist,
        "/GNSS_PWR_EN",
        {("U3", "3"), ("U4", "3"), ("R8", "2")},
        "MCU-controlled dual GNSS LDO enable",
    )
    try:
        gnss_enable_nodes = netlist.suffix("/GNSS_PWR_EN")
        audit.check(
            "GNSS power enable reaches flight MCU",
            any(node.ref == "U10" for node in gnss_enable_nodes),
            f"nodes={sorted((node.ref, node.pin) for node in gnss_enable_nodes)}",
        )
    except AssertionError as exc:
        audit.check("GNSS power enable reaches flight MCU", False, str(exc))

    connected_u30_pins = {
        node.pin
        for name, nodes in netlist.nets.items()
        if not name.startswith("unconnected-")
        for node in nodes
        if node.ref == "U30"
    }
    reserved_u30_pins = {
        "3", "4", "5", "6", "7", "11", "17",
        "22", "23", "24", "25", "26", "27", "32", "33", "36",
    }
    audit.check(
        "LG77L reserved pins remain NC",
        not (connected_u30_pins & reserved_u30_pins),
        f"connected reserved pins={sorted(connected_u30_pins & reserved_u30_pins) or 'none'}",
    )
    audit.require_nodes(
        netlist,
        "/GND",
        {("U30", pin) for pin in ("2", "18", "37", "38", "39", "40", "41", "42", "43")},
        "All LG77L ground pads connect to the ground plane",
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("esc_xml", type=Path)
    parser.add_argument("main_xml", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    audit = Audit()
    try:
        audit_esc(Netlist(args.esc_xml), audit)
        audit_main(Netlist(args.main_xml), audit)
    except (ET.ParseError, OSError, AssertionError) as exc:
        audit.checks.append({"name": "audit execution", "status": "FAIL", "detail": str(exc)})

    report = {
        "status": "PASS" if audit.passed else "FAIL",
        "checks": len(audit.checks),
        "failures": sum(check["status"] == "FAIL" for check in audit.checks),
        "results": audit.checks,
    }
    text = json.dumps(report, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0 if audit.passed else 1


if __name__ == "__main__":
    sys.exit(main())
