"""Electrical contract for the generated Rev-B ESC USB programming interfaces."""

from pathlib import Path
import xml.etree.ElementTree as ET


REPO = Path(__file__).resolve().parents[1]
NETLIST = REPO / "hardware/esc/rev_b/reports/esc_rev_b_netlist.xml"
REMOVED_BULK_CAPS = {f"C{1000 + cell * 100 + 30}" for cell in range(1, 7)}


def load_netlist():
    root = ET.parse(NETLIST).getroot()
    refs = {comp.attrib["ref"] for comp in root.findall("./components/comp")}
    nets = {}
    for net in root.findall("./nets/net"):
        nets[net.attrib["name"]] = {
            (node.attrib["ref"], node.attrib["pin"])
            for node in net.findall("node")
        }
    return refs, nets


def net_for_pin(nets, ref, pin):
    matches = [name for name, nodes in nets.items() if (ref, str(pin)) in nodes]
    assert len(matches) == 1, f"expected one net for {ref}.{pin}, got {matches}"
    return matches[0]


def test_external_bulk_capacitors_are_not_in_authoritative_netlist():
    refs, _ = load_netlist()
    assert not (refs & REMOVED_BULK_CAPS), f"obsolete PCB capacitors remain: {sorted(refs & REMOVED_BULK_CAPS)}"


def test_controller_has_native_usb_c_and_keeps_swd():
    refs, nets = load_netlist()
    assert {"J201", "J202", "U201"} <= refs
    assert net_for_pin(nets, "U201", 66).endswith("CTRL_USB_DM")
    assert net_for_pin(nets, "U201", 67).endswith("CTRL_USB_DP")


def test_service_port_and_break_before_make_selector_exist():
    refs, _ = load_netlist()
    assert {"J203", "SW201"} <= refs


def test_each_motor_mcu_uses_usb_pins_and_preserves_debug_and_safety():
    refs, nets = load_netlist()
    for cell in range(1, 7):
        mcu = f"U{cell}102"
        assert {mcu, f"J{cell}103"} <= refs
        assert net_for_pin(nets, mcu, 33).endswith(f"M{cell}_USB_DM")
        assert net_for_pin(nets, mcu, 34).endswith(f"M{cell}_USB_DP")
        assert net_for_pin(nets, mcu, 22).endswith(f"M{cell}_DRV_nFAULT")
        assert net_for_pin(nets, mcu, 25).endswith(f"M{cell}_ARM_ISO")


def test_motor_cell_ground_domains_remain_distinct():
    _, nets = load_netlist()
    for name, nodes in nets.items():
        cells = {
            int(ref[1])
            for ref, _ in nodes
            if len(ref) >= 5 and ref[0] in "RUJCDQLYFBTH" and ref[1] in "123456"
        }
        assert len(cells) <= 1, f"net {name} joins motor-cell populations {sorted(cells)}"
