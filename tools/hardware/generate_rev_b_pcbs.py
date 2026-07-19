#!/usr/bin/env python3
"""Generate placed four-layer Rev-B PCBs from the validated KiCad XML netlists.

Run this script with KiCad's bundled Python, not the system Python:
  "C:/Program Files/KiCad/9.0/bin/python.exe" tools/hardware/generate_rev_b_pcbs.py

The ESC is deliberately large enough for six physically partitioned 60 A
cells.  Major current paths are copper areas on the 4 oz outer layers with
1.5 oz inner-layer reinforcement; small-signal placement is local to each
motor controller.  The flight-control PCB is a separate board.
"""

from __future__ import annotations

import argparse
import math
import os
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import pcbnew


ROOT = Path(__file__).resolve().parents[2]
ESC_DIR = ROOT / "hardware" / "esc" / "rev_b"
MAIN_DIR = ROOT / "hardware" / "main" / "rev_b"
def _discover_footprint_root() -> Path:
    candidates: list[Path] = []
    for name in ("KICAD_FOOTPRINT_DIR", "KICAD9_FOOTPRINT_DIR", "KICAD8_FOOTPRINT_DIR"):
        value = os.environ.get(name)
        if value:
            candidates.append(Path(value))
    candidates.extend(
        (
            Path(r"C:\Program Files\KiCad\9.0\share\kicad\footprints"),
            Path("/usr/share/kicad/footprints"),
            Path("/usr/local/share/kicad/footprints"),
        )
    )
    for candidate in candidates:
        if candidate.is_dir():
            return candidate
    attempted = ", ".join(str(path) for path in candidates)
    raise RuntimeError(f"Unable to locate KiCad footprint libraries; tried: {attempted}")


KICAD_FP_ROOT = _discover_footprint_root()
MM = pcbnew.FromMM


@dataclass
class Component:
    ref: str
    value: str
    footprint: str
    sheet: str


@dataclass
class ParsedNetlist:
    components: dict[str, Component]
    nets: dict[str, list[tuple[str, str]]]


def parse_netlist(path: Path) -> ParsedNetlist:
    root = ET.parse(path).getroot()
    components: dict[str, Component] = {}
    for comp in root.findall("./components/comp"):
        ref = comp.attrib["ref"]
        components[ref] = Component(
            ref=ref,
            value=comp.findtext("value", default=""),
            footprint=comp.findtext("footprint", default=""),
            sheet=comp.find("./sheetpath").attrib.get("names", ""),
        )
    nets: dict[str, list[tuple[str, str]]] = {}
    for net in root.findall("./nets/net"):
        nets[net.attrib["name"]] = [
            (node.attrib["ref"], node.attrib["pin"]) for node in net.findall("node")
        ]
    return ParsedNetlist(components, nets)


def vec(x: float, y: float) -> pcbnew.VECTOR2I:
    return pcbnew.VECTOR2I(MM(x), MM(y))


def load_footprint(project_dir: Path, footprint_id: str) -> pcbnew.FOOTPRINT:
    library, name = footprint_id.split(":", 1)
    library_dir = (
        project_dir / "revb.pretty"
        if library == "revb"
        else KICAD_FP_ROOT / f"{library}.pretty"
    )
    if not library_dir.is_dir():
        raise RuntimeError(f"Footprint library does not exist: {library_dir}")
    footprint_file = library_dir / f"{name}.kicad_mod"
    if not footprint_file.is_file():
        raise RuntimeError(f"Footprint file does not exist: {footprint_file}")
    footprint = None
    try:
        footprint = pcbnew.FootprintLoad(str(library_dir), name)
    except (AttributeError, RuntimeError):
        plugin_type = getattr(pcbnew, "PCB_IO_KICAD_SEXPR", None)
        if plugin_type is None:
            raise
        footprint = plugin_type().FootprintLoad(str(library_dir), name, False)
    if footprint is None:
        raise RuntimeError(f"Unable to load {footprint_id} from {library_dir}")
    return footprint


def create_board(
    netlist: ParsedNetlist, project_dir: Path
) -> tuple[pcbnew.BOARD, dict[str, pcbnew.FOOTPRINT], dict[str, pcbnew.NETINFO_ITEM]]:
    board = pcbnew.BOARD()
    board.SetCopperLayerCount(4)
    settings = board.GetDesignSettings()
    base_clearance = 0.20 if project_dir == ESC_DIR else 0.125
    settings.m_HasStackup = True
    settings.m_MinClearance = MM(base_clearance)
    settings.m_TrackMinWidth = MM(0.20)
    settings.m_ViasMinSize = MM(0.60)
    settings.m_ViasMinDrill = MM(0.30)
    settings.m_MinThroughDrill = MM(0.20)
    settings.m_HoleClearance = MM(0.20)
    settings.m_HoleToHoleMin = MM(0.20)

    default = settings.m_NetSettings.GetNetClassByName("Default")
    default.SetClearance(MM(base_clearance))
    default.SetTrackWidth(MM(0.25))
    default.SetViaDiameter(MM(0.70))
    default.SetViaDrill(MM(0.35))

    netclasses = {
        "POWER": (base_clearance, 0.80, 0.80, 0.40),
        "GATE_DRIVE": (base_clearance, 0.50, 0.70, 0.35),
        "CURRENT_SENSE": (base_clearance, 0.25, 0.60, 0.30),
        "CAN_DIFF": (base_clearance, 0.25, 0.60, 0.30),
        "RF_50R": (base_clearance, 0.35, 0.60, 0.30),
        # Fine-pitch driver/control pads share these nets, so the class
        # clearance must remain compatible with their land patterns.  The
        # large current-carrying zones enforce 0.50 mm locally.
        "HIGH_CURRENT": (base_clearance, 6.00, 1.00, 0.50),
    }
    class_objects: dict[str, pcbnew.NETCLASS] = {"Default": default}
    for name, (clearance, width, via, drill) in netclasses.items():
        netclass = pcbnew.NETCLASS(name)
        netclass.SetClearance(MM(clearance))
        netclass.SetTrackWidth(MM(width))
        netclass.SetViaDiameter(MM(via))
        netclass.SetViaDrill(MM(drill))
        settings.m_NetSettings.SetNetclass(name, netclass)
        class_objects[name] = netclass

    board_nets: dict[str, pcbnew.NETINFO_ITEM] = {}
    for name in netlist.nets:
        item = pcbnew.NETINFO_ITEM(board, name)
        board.Add(item)
        board_nets[name] = item

    high_current = re.compile(
        r"/M[1-6]_(?:BATP(?:_IN)?|BATN|PHASE_[ABC]|SH[ABC]_I)$"
    )
    for name, item in board_nets.items():
        assigned_class = "Default"
        if high_current.search(name):
            assigned_class = "HIGH_CURRENT"
        elif any(token in name for token in ("GH", "GL", "PWM_")):
            assigned_class = "GATE_DRIVE"
        elif any(token in name for token in ("CSA_", "BUS_SH_", "BUS_CURRENT")):
            assigned_class = "CURRENT_SENSE"
        elif name.endswith(("CANH", "CANL")) or "_CANH" in name or "_CANL" in name:
            assigned_class = "CAN_DIFF"
        elif any(
            token in name
            for token in (
                "ANT_915",
                "CC1121_RF",
                "CC_LNA_",
                "CC_PA",
                "RF_LNA_IN",
                "RF_PA_",
                "RF_TR_SW",
                "RX_COMBINE",
                "SAW_PA",
                "MATCH_",
                "GNSS_RF_",
            )
        ):
            assigned_class = "RF_50R"
        elif any(
            token in name
            for token in ("3V3", "5V", "1V1", "2V8", "VBUS", "AUX_", "VDD")
        ):
            assigned_class = "POWER"
        item.SetNetClass(class_objects[assigned_class])
        if assigned_class != "Default":
            settings.m_NetSettings.SetNetclassPatternAssignment(name, assigned_class)

    node_to_net: dict[tuple[str, str], pcbnew.NETINFO_ITEM] = {}
    for name, nodes in netlist.nets.items():
        for node in nodes:
            node_to_net[node] = board_nets[name]

    footprints: dict[str, pcbnew.FOOTPRINT] = {}
    for component in netlist.components.values():
        footprint = load_footprint(project_dir, component.footprint)
        footprint.SetReference(component.ref)
        footprint.SetValue(component.value)
        footprint.SetPosition(vec(0, 0))
        for field in (footprint.Reference(), footprint.Value()):
            field.SetTextSize(vec(0.8, 0.8))
            field.SetTextThickness(MM(0.12))
        board.Add(footprint)
        footprints[component.ref] = footprint
        for pad in footprint.Pads():
            net = node_to_net.get((component.ref, pad.GetNumber()))
            if net is not None:
                pad.SetNet(net)

    board.BuildListOfNets()
    board.SynchronizeNetsAndNetClasses(False)
    return board, footprints, board_nets


def place(
    footprints: dict[str, pcbnew.FOOTPRINT],
    ref: str,
    x: float,
    y: float,
    rotation: float = 0.0,
) -> None:
    fp = footprints[ref]
    fp.SetPosition(vec(x, y))
    fp.SetOrientationDegrees(rotation)


def move_footprint_silks_to_fab(board: pcbnew.BOARD) -> None:
    """Keep dense boards DRC-clean while retaining an assembly reference layer."""
    for footprint in board.GetFootprints():
        reference = footprint.Reference()
        reference.SetLayer(pcbnew.F_Fab)
        reference.SetVisible(True)
        value = footprint.Value()
        value.SetLayer(pcbnew.F_Fab)
        value.SetVisible(False)
        for item in footprint.GraphicalItems():
            if item.GetLayer() == pcbnew.F_SilkS:
                item.SetLayer(pcbnew.F_Fab)


def courtyard_size(fp: pcbnew.FOOTPRINT) -> tuple[float, float]:
    pad_bbox = fp.GetFpPadsLocalBbox()
    pad_width = pcbnew.ToMM(pad_bbox.GetWidth())
    pad_height = pcbnew.ToMM(pad_bbox.GetHeight())
    try:
        bbox = fp.GetCourtyard(pcbnew.F_CrtYd).BBox()
        width = pcbnew.ToMM(bbox.GetWidth())
        height = pcbnew.ToMM(bbox.GetHeight())
    except Exception:
        width = pad_width
        height = pad_height
    # Extra margin avoids borderline courtyard collisions from cached polygon
    # bounds in KiCad's SWIG API.
    return max(width, pad_width, 1.2) + 1.0, max(height, pad_height, 1.2) + 1.0


def pack_region(
    footprints: dict[str, pcbnew.FOOTPRINT],
    refs: list[str],
    x0: float,
    y0: float,
    x1: float,
    y1: float,
    gap: float = 0.55,
) -> None:
    """Simple deterministic courtyard-aware row packer for support passives."""
    x = x0
    y = y0
    row_height = 0.0
    for ref in refs:
        fp = footprints[ref]
        width, height = courtyard_size(fp)
        if x + width / 2 > x1:
            x = x0
            y += row_height + gap
            row_height = 0.0
        if y + height / 2 > y1:
            raise RuntimeError(
                f"Placement region exhausted at {ref}: ({x0},{y0})-({x1},{y1})"
            )
        place(footprints, ref, x + width / 2, y + height / 2)
        x += width + gap
        row_height = max(row_height, height)


def add_outline(board: pcbnew.BOARD, width: float, height: float) -> None:
    corners = [(0, 0), (width, 0), (width, height), (0, height), (0, 0)]
    for start, end in zip(corners, corners[1:]):
        line = pcbnew.PCB_SHAPE(board)
        line.SetShape(pcbnew.SHAPE_T_SEGMENT)
        line.SetStart(vec(*start))
        line.SetEnd(vec(*end))
        line.SetLayer(pcbnew.Edge_Cuts)
        line.SetWidth(MM(0.15))
        board.Add(line)


def add_silk_line(
    board: pcbnew.BOARD,
    start: tuple[float, float],
    end: tuple[float, float],
    width: float = 0.30,
) -> None:
    line = pcbnew.PCB_SHAPE(board)
    line.SetShape(pcbnew.SHAPE_T_SEGMENT)
    line.SetStart(vec(*start))
    line.SetEnd(vec(*end))
    line.SetLayer(pcbnew.F_SilkS)
    line.SetWidth(MM(width))
    board.Add(line)


def add_text(
    board: pcbnew.BOARD,
    text: str,
    x: float,
    y: float,
    size: float = 1.5,
    layer: int = pcbnew.F_SilkS,
) -> None:
    item = pcbnew.PCB_TEXT(board)
    item.SetText(text)
    item.SetPosition(vec(x, y))
    item.SetLayer(layer)
    item.SetTextSize(vec(size, size))
    item.SetTextThickness(MM(max(0.18, size * 0.12)))
    board.Add(item)


def add_mounting_holes(
    board: pcbnew.BOARD,
    project_dir: Path,
    positions: list[tuple[float, float]],
    start_index: int = 1,
) -> None:
    for index, (x, y) in enumerate(positions, start_index):
        fp = load_footprint(project_dir, "MountingHole:MountingHole_4.3mm_M4")
        fp.SetReference(f"H{index}")
        fp.SetValue("M4")
        fp.SetPosition(vec(x, y))
        board.Add(fp)


def net_ending(
    board_nets: dict[str, pcbnew.NETINFO_ITEM], suffix: str
) -> pcbnew.NETINFO_ITEM:
    matches = [item for name, item in board_nets.items() if name.endswith(suffix)]
    if len(matches) != 1:
        raise RuntimeError(f"Expected one net ending {suffix!r}; got {len(matches)}")
    return matches[0]


def add_zone(
    board: pcbnew.BOARD,
    net: pcbnew.NETINFO_ITEM,
    layer: int,
    points: list[tuple[float, float]],
    *,
    clearance: float = 0.50,
    priority: int = 0,
) -> pcbnew.ZONE:
    zone = pcbnew.ZONE(board)
    zone.SetNet(net)
    zone.SetLayer(layer)
    zone.SetLocalClearance(MM(clearance))
    zone.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
    zone.SetAssignedPriority(priority)
    outline = zone.Outline()
    outline.NewOutline()
    for x, y in points:
        outline.Append(MM(x), MM(y))
    board.Add(zone)
    return zone


def add_rect_zone(
    board: pcbnew.BOARD,
    net: pcbnew.NETINFO_ITEM,
    layer: int,
    x0: float,
    y0: float,
    x1: float,
    y1: float,
    **kwargs: object,
) -> pcbnew.ZONE:
    return add_zone(
        board,
        net,
        layer,
        [(x0, y0), (x1, y0), (x1, y1), (x0, y1)],
        **kwargs,
    )


def add_via(
    board: pcbnew.BOARD,
    net: pcbnew.NETINFO_ITEM,
    x: float,
    y: float,
    diameter: float = 1.0,
    drill: float = 0.5,
) -> None:
    via = pcbnew.PCB_VIA(board)
    via.SetPosition(vec(x, y))
    via.SetWidth(MM(diameter))
    via.SetDrill(MM(drill))
    via.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
    via.SetNet(net)
    board.Add(via)


def add_via_grid(
    board: pcbnew.BOARD,
    net: pcbnew.NETINFO_ITEM,
    center: tuple[float, float],
    columns: int,
    rows: int,
    pitch: float = 1.35,
) -> None:
    cx, cy = center
    for col in range(columns):
        for row in range(rows):
            add_via(
                board,
                net,
                cx + (col - (columns - 1) / 2) * pitch,
                cy + (row - (rows - 1) / 2) * pitch,
            )


def place_esc_motor(
    footprints: dict[str, pcbnew.FOOTPRINT], motor: int, x0: float, y0: float
) -> None:
    base = 1000 + motor * 100
    explicit: set[str] = set()

    def p(ref: str, x: float, y: float, rotation: float = 0.0) -> None:
        place(footprints, ref, x0 + x, y0 + y, rotation)
        explicit.add(ref)

    # The three half bridges are staggered around a central gate driver.  This
    # keeps all six gate/source loops below about 30 mm without putting logic
    # in a switch-node copper area.
    phase_x = (60.0, 80.0, 100.0)
    high_y = (32.0, 24.0, 32.0)
    low_y = (52.0, 60.0, 52.0)
    shunt_y = (69.0, 79.0, 69.0)

    p(f"J{base + 2}", 80, 8)
    p(f"J{base + 1}", 17, 86)
    p(f"R{base + 23}", 17, 69, 90)
    p(f"C{base + 30}", 31, 69)
    p(f"D{base + 1}", 42, 88)
    for phase_index in range(3):
        x = phase_x[phase_index]
        yh = high_y[phase_index]
        yl = low_y[phase_index]
        p(f"Q{base + phase_index * 2 + 1}", x, yh)
        p(f"Q{base + phase_index * 2 + 2}", x, yl)
        p(f"R{base + 20 + phase_index}", x, shunt_y[phase_index], 270)
        gate_refs = [base + phase_index * 4 + offset for offset in range(1, 5)]
        # TOLT pad 8 is at the lower-right source edge.  Put both the series
        # resistor and gate-source pull-down directly outside that edge.
        p(f"R{gate_refs[0]}", x + 7.5, yh + 5.5)
        p(f"R{gate_refs[1]}", x + 7.5, yh + 8.5, 90)
        p(f"R{gate_refs[2]}", x + 7.5, yl + 5.5)
        p(f"R{gate_refs[3]}", x + 7.5, yl + 8.5, 90)

    p(f"U{base + 1}", 90, 43)
    p(f"U{base + 2}", 130, 45)
    p(f"U{base + 3}", 118, 84)
    p(f"U{base + 4}", 143, 76)
    p(f"U{base + 5}", 18, 57)
    p(f"U{base + 6}", 138, 63)
    p(f"PS{base + 1}", 143, 89)
    p(f"J{base + 3}", 150, 56, 90)
    p(f"Y{base + 1}", 118, 38)
    p(f"FB{base + 1}", 136, 51)
    p(f"TH{motor}", 108, 66)
    p(f"D{base + 2}", 151, 70)

    # DRV8353 charge-pump, DVDD, VGLS and analog bypass components are kept
    # against their associated driver edges.  The two high-voltage bypass
    # parts sit at the nearby BATP/BATN corridor.
    driver_caps = {
        1: (87.0, 48.5, 90),
        2: (90.0, 49.5, 90),
        3: (86.0, 45.0, 90),
        4: (86.0, 42.0, 90),
        5: (93.0, 56.5, 90),
        6: (90.0, 53.0, 0),
        7: (92.0, 38.0, 90),
    }
    for offset, (x, y, rotation) in driver_caps.items():
        p(f"C{base + offset}", x, y, rotation)
    for phase_index in range(3):
        x = phase_x[phase_index]
        if phase_index == 1:
            # The middle high-side MOSFET is raised, so its two MLCCs occupy
            # the clear corridors on either side instead of its courtyard.
            p(f"C{base + 31 + phase_index * 2}", 70.0, 20.5, 90)
            p(f"C{base + 32 + phase_index * 2}", 90.0, 20.5, 90)
        else:
            p(f"C{base + 31 + phase_index * 2}", x - 3.2, 20.5, 90)
            p(f"C{base + 32 + phase_index * 2}", x + 3.2, 20.5, 90)
    # Fast current-sense filters terminate at the MCU ADC side.
    for offset in range(4):
        p(f"R{base + 30 + offset}", 118, 44 + offset * 3.0)
        p(f"C{base + 20 + offset}", 123, 44 + offset * 3.0)
    for offset in range(5):
        p(f"R{base + 40 + offset}", 110 + offset * 9.0, 8)
    p(f"R{base + 45}", 112, 66)
    p(f"C{base + 50}", 146, 12)
    # Isolation supply/LDO bypass and both sides of the digital isolator.
    isolation_caps = {
        60: (137.0, 94.0, 0),
        61: (151.5, 85.0, 90),
        62: (151.5, 89.0, 90),
        63: (149.0, 76.0, 90),
        64: (110.0, 82.0, 90),
        65: (126.0, 82.0, 90),
    }
    for offset, (x, y, rotation) in isolation_caps.items():
        p(f"C{base + offset}", x, y, rotation)

    # MCU bulk/bypass, oscillator, reset and analog-supply components.
    mcu_caps = {
        66: (126.0, 37.0, 0),
        67: (129.0, 37.0, 0),
        68: (133.0, 38.0, 90),
        69: (136.0, 40.0, 90),
        70: (136.0, 43.0, 90),
        71: (136.0, 46.0, 90),
        72: (136.0, 49.0, 90),
        73: (122.5, 36.5, 0),
        74: (122.5, 40.5, 0),
        75: (128.0, 55.5, 0),
        76: (23.0, 57.0, 90),
        77: (143.0, 63.0, 90),
    }
    for offset, (x, y, rotation) in mcu_caps.items():
        ref = f"C{base + offset}"
        if ref in footprints:
            p(ref, x, y, rotation)
    for offset in range(50, 58):
        ref = f"R{base + offset}"
        if ref in footprints and ref not in explicit:
            p(ref, 120 + ((offset - 50) % 4) * 5.0, 58 + ((offset - 50) // 4) * 5.0)
    # Status LED resistor belongs with the LED, outside the ARM-gate cluster.
    p(f"R{base + 57}", 147, 67)
    # TPS709 enable-divider resistors belong at the LDO, not in the generic
    # control-area leftovers row.
    p(f"R{base + 58}", 138, 70)
    p(f"R{base + 59}", 146, 70)

    # Determine membership by the numeric reference block; TH and PS were
    # already explicitly placed.
    leftovers: list[str] = []
    for ref in footprints:
        if ref in explicit:
            continue
        match = re.search(r"(\d+)$", ref)
        if match and base <= int(match.group(1)) < base + 100:
            leftovers.append(ref)
    if leftovers:
        pack_region(footprints, sorted(leftovers), x0 + 104, y0 + 12, x0 + 154, y0 + 21)


def add_esc_power_copper(
    board: pcbnew.BOARD,
    board_nets: dict[str, pcbnew.NETINFO_ITEM],
    motor: int,
    x0: float,
    y0: float,
) -> None:
    batp_in = net_ending(board_nets, f"/M{motor}_BATP_IN")
    batp = net_ending(board_nets, f"/M{motor}_BATP")
    batn = net_ending(board_nets, f"/M{motor}_BATN")
    phases = {
        phase: net_ending(board_nets, f"/M{motor}_PHASE_{phase}")
        for phase in ("A", "B", "C")
    }
    low_sources = {
        phase: net_ending(board_nets, f"/M{motor}_SH{phase}_I")
        for phase in ("A", "B", "C")
    }

    add_rect_zone(board, batp_in, pcbnew.F_Cu, x0 + 3, y0 + 70.5, x0 + 22, y0 + 92)
    add_rect_zone(board, batp, pcbnew.In1_Cu, x0 + 12, y0 + 18, x0 + 98, y0 + 72)
    phase_x = (60.0, 80.0, 100.0)
    high_y = (32.0, 24.0, 32.0)
    low_y = (52.0, 60.0, 52.0)
    shunt_y = (69.0, 79.0, 69.0)
    for phase_index, phase in enumerate(("A", "B", "C")):
        x = x0 + phase_x[phase_index]
        yh = high_y[phase_index]
        yl = low_y[phase_index]
        add_rect_zone(
            board, batp, pcbnew.F_Cu,
            x - 6, y0 + yh - 9, x + 6, y0 + yh + 3,
            priority=3,
        )
        # Keep the F.Cu switch-node copper only at the two MOSFET landings.
        # In2 carries the phase to the output lug.  The clear horizontal band
        # between these two islands is reserved for tightly paired gate/source
        # routing, so phase-A drive never has to cross a switch-node polygon.
        add_rect_zone(
            board, phases[phase], pcbnew.F_Cu,
            x - 5.2, y0 + yh + 3.8, x + 5.2, y0 + yh + 7.2,
            priority=4,
        )
        add_rect_zone(
            board, phases[phase], pcbnew.F_Cu,
            x - 5.2, y0 + yl - 7.2, x + 5.2, y0 + yl - 3,
            priority=4,
        )
        add_rect_zone(
            board, phases[phase], pcbnew.In2_Cu,
            x - 5.2, y0 + 2, x + 5.2, y0 + yl - 3,
            priority=4,
        )
        add_rect_zone(
            board, low_sources[phase], pcbnew.F_Cu,
            x - 4.5, y0 + yl + 3, x + 4.5, y0 + shunt_y[phase_index] - 3.5,
            priority=4,
        )
        add_via_grid(board, batp, (x, y0 + yh - 8.5), 8, 3)
        # Ten-via phase transitions fit inside each landing island.  Bias the
        # field toward the source/drain pad bank and away from TOLT gate pad 8
        # on the right edge; a centered six-via row violated the gate-pad hole
        # clearance by 0.025 mm.
        add_via_grid(board, phases[phase], (x - 1.35, y0 + yh + 5.5), 5, 2)
        add_via_grid(board, phases[phase], (x - 1.35, y0 + yl - 5.1), 5, 2)
        add_via_grid(
            board,
            batn,
            (x, y0 + shunt_y[phase_index] + 6),
            8,
            3,
        )

    add_rect_zone(board, batn, pcbnew.F_Cu, x0 + 24, y0 + 72, x0 + 100, y0 + 94)
    add_zone(
        board,
        batn,
        pcbnew.B_Cu,
        [
            (x0 + 3, y0 + 45),
            (x0 + 100, y0 + 45),
            (x0 + 100, y0 + 94),
            (x0 + 24, y0 + 94),
            (x0 + 24, y0 + 64),
            (x0 + 3, y0 + 64),
        ],
    )
    add_rect_zone(board, batn, pcbnew.B_Cu, x0 + 102, y0 + 3, x0 + 154, y0 + 97)
    add_via_grid(board, batp, (x0 + 22, y0 + 64), 5, 3)


def generate_esc() -> Path:
    netlist = parse_netlist(ESC_DIR / "reports" / "esc_rev_b_netlist.xml")
    board, footprints, board_nets = create_board(netlist, ESC_DIR)
    width, height = 474.0, 246.0
    add_outline(board, width, height)
    cell_origins: dict[int, tuple[float, float]] = {}
    for motor in range(1, 7):
        col = (motor - 1) % 3
        row = (motor - 1) // 3
        origin = (3 + col * 157, 43 + row * 100)
        cell_origins[motor] = origin
        place_esc_motor(footprints, motor, *origin)
        add_esc_power_copper(board, board_nets, motor, *origin)
        add_text(board, f"MOTOR {motor} - 60 A CELL", origin[0] + 75, origin[1] + 97, 1.4)
        add_silk_line(board, (origin[0], origin[1]), (origin[0] + 155, origin[1]))
        add_silk_line(board, (origin[0], origin[1]), (origin[0], origin[1] + 98))

    controller_refs = sorted(
        ref for ref, comp in netlist.components.items() if "Controller and ADC" in comp.sheet
    )
    power_refs = sorted(
        ref for ref, comp in netlist.components.items() if "Auxiliary power" in comp.sheet
    )
    # Edge connectors and major ICs first; all support parts are packed into
    # their own controller/power regions in the non-isolated supervisor strip.
    explicit = {
        "J701": (12, 20, 90),
        "J702": (172, 6, 0),
        "U701": (42, 20, 0),
        "U703": (105, 20, 0),
        "L701": (62, 20, 0),
        "L702": (125, 20, 0),
        "U702": (150, 20, 0),
        "F701": (165, 20, 0),
        "D701": (180, 20, 0),
        "U201": (340, 21, 0),
        # RP2354B switcher, bulk and per-domain bypass parts stay within a few
        # millimetres of the QFN.  The regulator loop must not be left to the
        # generic support-part packer.
        "L201": (344, 13, 0),
        "R201": (337, 13, 0),
        "C201": (331.5, 17, 0),
        "C202": (331.5, 20, 0),
        "C203": (331.5, 23, 0),
        "C204": (331.5, 26, 0),
        "FB201": (349, 29, 0),
        "C205": (334, 14.5, 0),
        "C206": (337, 14.5, 0),
        "C207": (340, 14.5, 0),
        "C208": (343, 14.5, 0),
        "C209": (346, 15.5, 90),
        "C210": (346.5, 19, 90),
        "C211": (346.5, 22, 90),
        "C212": (349, 25.5, 90),
        "C215": (343, 27.5, 0),
        "C216": (340, 27.5, 0),
        "C217": (337, 27.5, 0),
        "C218": (334, 27.5, 0),
        "C219": (332.5, 29.5, 0),
        "U204": (385, 21, 0),
        "U205": (400, 21, 0),
        "U206": (435, 21, 0),
        "U207": (420, 21, 0),
        "J201": (365, 6, 0),
        "FL201": (452, 21, 0),
        "D202": (462, 21, 0),
        "Y201": (355, 21, 0),
        "Y202": (420, 10, 0),
    }
    for ref, (x, y, rotation) in explicit.items():
        place(footprints, ref, x, y, rotation)
    power_support = [ref for ref in power_refs if ref not in explicit]
    split = math.ceil(len(power_support) / 2)
    pack_region(footprints, power_support[:split], 22, 4, 158, 13, 1.00)
    pack_region(footprints, power_support[split:], 22, 28, 158, 39, 1.00)
    pack_region(
        footprints,
        [ref for ref in controller_refs if ref not in explicit],
        195,
        5,
        330,
        38,
        1.00,
    )
    add_text(board, "AUX POWER - TWO INDEPENDENT 1 A RAILS", 95, 40, 1.4)
    add_text(board, "ESC SUPERVISOR / CAN FD", 370, 40, 1.4)
    add_silk_line(board, (0, 42), (width, 42), 0.5)
    add_silk_line(board, (157, 42), (157, height), 0.5)
    add_silk_line(board, (314, 42), (314, height), 0.5)
    add_silk_line(board, (0, 143), (width, 143), 0.5)
    add_text(board, "REV-B  |  4 oz OUTER / 1.5 oz INNER  |  6 x 60 A", width / 2, 2.5, 1.6)
    add_mounting_holes(
        board,
        ESC_DIR,
        [(5, 5), (width - 5, 5), (5, height - 5), (width - 5, height - 5),
         (157, 40), (314, 40), (157, height - 5), (314, height - 5)],
    )
    move_footprint_silks_to_fab(board)

    # Supervisor ground planes do not enter local BATN areas.  Narrow primary-
    # side power corridors to the ISO6731/RFM inputs are routed conventionally.
    dgnd = net_ending(board_nets, "/DGND")
    add_rect_zone(board, dgnd, pcbnew.B_Cu, 2, 2, width - 2, 41, clearance=0.30)

    output = ESC_DIR / "esc_rev_b.kicad_pcb"
    pcbnew.SaveBoard(str(output), board)
    apply_stackup(output, outer_oz=4.0, inner_oz=1.5)
    return output


def main_key_positions() -> dict[str, tuple[float, float, float]]:
    return {
        "J1": (6, 76, 90),
        "J2": (6, 20, 90),
        # USB protection remains at the connector.  The buck input/output and
        # feedback components form a compact loop in the lower-left region.
        "U1": (17, 76, 0),
        "U2": (29, 76, 0),
        "L2": (35, 76, 0),
        "C1": (26, 71.5, 0),
        "C2": (40, 75, 90),
        "C6": (40, 79, 90),
        "R6": (34, 82, 0),
        "R7": (38, 82, 0),
        "R1": (11, 69, 0),
        "R2": (11, 83, 0),
        "R5": (16, 69, 0),
        "C5": (20, 69, 0),
        "D1": (15, 72, 0),
        "F1": (11, 31, 0),
        "D2": (17, 31, 0),
        "R3": (23, 31, 0),
        # Put the quiet GNSS regulators at the load.  Only raw SYS_5V crosses
        # the board; the 2.8 V and 3.3 V GNSS rails stay local and short.
        "U3": (126, 17, 0),
        "C3": (122, 17, 90),
        "C4": (130, 17, 90),
        "U4": (126, 27, 0),
        "C7": (122, 27, 90),
        "C8": (130, 27, 90),
        "R8": (134, 22, 90),
        "U10": (59, 50, 0),
        "J10": (59, 32, 0),
        "Y1": (70, 50, 0),
        # RP2354B regulator and decoupling placement follows the RP2350B
        # minimal-board guidance: tight switch loop, local AVDD RC, and one
        # 100 nF bypass per supply position.
        "L1": (68, 42.5, 0),
        "R4": (51, 41.5, 0),
        "FB1": (51.5, 50, 90),
        "C10": (52, 44, 0),
        "C11": (52, 47, 0),
        "C12": (52, 53, 0),
        "C13": (52, 56, 0),
        "C30": (54, 43.5, 0),
        "C31": (57, 43.5, 0),
        "C32": (60, 43.5, 0),
        "C33": (63, 43.5, 0),
        "C34": (65.5, 46, 90),
        "C35": (65.5, 49, 90),
        "C36": (65.5, 52, 90),
        "C37": (65.5, 55, 90),
        "C38": (63, 56.5, 0),
        "C39": (60, 56.5, 0),
        "C40": (57, 56.5, 0),
        "C41": (54, 56.5, 0),
        "C42": (51.5, 58.5, 0),
        # Sensor rail entry, local bulk and one high-frequency bypass at each
        # sensor.  Keep the two dissimilar IMUs mechanically separated.
        "FB20": (43, 83, 90),
        "C56": (45.5, 79, 0),
        "C57": (43, 88, 0),
        "U20": (50, 83, 0),
        "C50": (47, 80, 0),
        "C51": (47, 86, 0),
        "R22": (53.5, 88, 0),
        "U21": (60, 83, 0),
        "C52": (57, 80, 0),
        "C53": (57, 86, 0),
        "R23": (63.5, 88, 0),
        "U22": (70, 83, 0),
        "C54": (67, 80, 0),
        "C55": (67, 86, 0),
        "R20": (73.5, 80, 90),
        "R21": (73.5, 86, 90),
        "U30": (101, 19, 0),
        # Passive-GNSS RF chain is kept short and all support components
        # remain outside the module's manufacturer-required 3 mm courtyard.
        "J30": (84, 19, 0),
        "D30": (89, 23, 0),
        "C77": (89, 15, 0),
        "R35": (91.5, 19, 0),
        "C78": (92, 23, 0),
        "D31": (110.5, 27, 0),
        "D32": (110.5, 11, 0),
        "FB30": (112, 19, 0),
        "C70": (115, 23, 0),
        "C71": (115, 20, 0),
        "C72": (115, 17, 0),
        "C73": (115, 14, 0),
        "C74": (120, 23, 0),
        "C75": (120, 20, 0),
        "C76": (120, 17, 0),
        # CAN connector-side ESD, common-mode choke, transceiver, controller
        # and crystal are placed in bus order.
        "D50": (11, 27, 0),
        "FL50": (15, 20, 0),
        "R122": (20, 24.5, 90),
        "U50": (24.5, 20, 180),
        "C123": (24.5, 15, 0),
        "U51": (32, 20, 0),
        "C122": (32, 15, 0),
        "R120": (36, 16.5, 90),
        "R121": (28, 24.5, 90),
        "Y50": (33, 27, 0),
        "C120": (29, 28, 0),
        "C121": (37, 28, 0),
        # CC1121 RF pins 17..20 face the balun.  Rotating FL40 by 90 degrees
        # aligns all four RF pins; the SAW and CC1190 then form one short,
        # monotonic RF chain toward the edge connector.
        "U40": (94, 61, 0),
        "FL40": (102, 62, 90),
        "FL41": (105, 62.25, 180),
        "U41": (111.5, 62.25, 180),
        "Y40": (87, 54, 0),
        "J40": (136, 62.5, 0),
        # CC1121 DCPL/oscillator/bias network.
        "L40": (83, 61, 0),
        "C100": (86, 61, 0),
        "C101": (88.5, 57, 0),
        "C102": (90.5, 55.5, 90),
        "C103": (92.5, 55.5, 90),
        "C104": (94.5, 55.5, 90),
        "C105": (89, 65.5, 0),
        "C106": (92, 66.5, 0),
        "C80": (88.5, 61.75, 0),
        "C81": (98.75, 60, 0),
        "C82": (98.5, 55.5, 90),
        "C83": (96.5, 55.5, 90),
        "C84": (89.8, 54, 0),
        "C85": (97, 67.5, 0),
        "C86": (98.2, 62.75, 0),
        "C87": (98.75, 58, 90),
        "L41": (96, 65.5, 90),
        "R40": (95, 66.5, 90),
        "R41": (98.5, 65.5, 90),
        "R42": (100.2, 62.75, 0),
        "R43": (86, 65.5, 90),
        "R44": (84, 65.5, 90),
        # SAW-to-CC1190 input and local bias/decoupling.
        "C88": (106.5, 59, 90),
        "L42": (107.8, 62.25, 0),
        "R45": (109.5, 57, 90),
        "R46": (111.5, 57, 90),
        "R47": (113.5, 57, 90),
        "C89": (109.5, 66, 90),
        "L43": (107, 66, 90),
        "R50": (111.5, 66, 90),
        "R51": (112.5, 67.5, 90),
        "L44": (114.5, 68.5, 90),
        "L45": (103.5, 70, 0),
        "C110": (106.5, 70, 0),
        "C111": (109.5, 70, 0),
        "C112": (112, 70, 0),
        "C113": (114.5, 70, 0),
        "C114": (117, 70, 0),
        "C115": (119.5, 70, 0),
        # CC1190 PA/LNA combining and harmonic-match network.  DNP parts stay
        # physically present so the manufactured board can be VNA retuned.
        "C93": (114.7, 60.3, 90),
        "C92": (116.2, 61.3, 180),
        "L46": (116.5, 64.5, 0),
        "C90": (115.5, 62.6, 0),
        "C91": (117, 67.5, 90),
        "L47": (117.5, 59.5, 90),
        "L48": (119, 62.6, 0),
        "C94": (119, 64.2, 0),
        "C95": (121.5, 65, 90),
        "L49": (123, 62.6, 0),
        "C96": (126, 65, 90),
    }


def generate_main() -> Path:
    netlist = parse_netlist(MAIN_DIR / "reports" / "main_rev_b_netlist.xml")
    board, footprints, board_nets = create_board(netlist, MAIN_DIR)
    width, height = 140.0, 100.0
    add_outline(board, width, height)
    key_positions = main_key_positions()
    for ref, (x, y, rotation) in key_positions.items():
        place(footprints, ref, x, y, rotation)

    regions = {
        "/Power and USB/": [(9, 59, 42, 69), (9, 84, 42, 97)],
        "/Redundant sensors/": [(44, 73, 74, 78), (44, 89, 74, 97)],
        "/RP2354B MCU/": [(44, 36, 76, 43), (44, 59, 76, 71)],
        "/GNSS/": [(80, 5, 106, 10), (124, 12, 136, 34)],
        "/CAN interface/": [(9, 5, 42, 11), (9, 26, 42, 34)],
        "/915 MHz RF/": [(79, 38, 136, 48), (79, 72, 136, 97)],
    }
    for sheet, sheet_regions in regions.items():
        refs = sorted(
            ref
            for ref, component in netlist.components.items()
            if component.sheet == sheet and ref not in key_positions
        )
        split = math.ceil(len(refs) / len(sheet_regions))
        for index, bounds in enumerate(sheet_regions):
            chunk = refs[index * split : (index + 1) * split]
            if chunk:
                pack_region(footprints, chunk, *bounds, gap=0.75)

    ground = net_ending(board_nets, "/GND")
    add_rect_zone(board, ground, pcbnew.In1_Cu, 1, 1, width - 1, height - 1, clearance=0.20)
    add_rect_zone(board, ground, pcbnew.B_Cu, 1, 1, width - 1, height - 1, clearance=0.20)
    for x in range(10, 130, 5):
        add_via(board, ground, x, 3, 0.7, 0.35)
        add_via(board, ground, x, height - 3, 0.7, 0.35)
    for y in range(8, 96, 5):
        add_via(board, ground, 77, y, 0.7, 0.35)
        # The edge-launch 915 MHz connector includes a local keepout.
        if not 58 <= y <= 68:
            add_via(board, ground, 137, y, 0.7, 0.35)
    # Ground-via fences bound both RF areas.  The GNSS row pair forms a
    # grounded corridor around the short connector-to-module 50-ohm route;
    # the 915 MHz section also has a continuous In1 ground reference.
    for x in range(80, 96, 2):
        add_via(board, ground, x, 12, 0.7, 0.35)
        add_via(board, ground, x, 26, 0.7, 0.35)
    for x in range(80, 137, 3):
        add_via(board, ground, x, 36, 0.7, 0.35)
    for x in range(80, 137, 3):
        add_via(board, ground, x, 48, 0.7, 0.35)
        add_via(board, ground, x, 74, 0.7, 0.35)

    add_text(board, "REV-B FLIGHT CONTROL - DISTINCT FROM ESC", width / 2, 2.5, 1.5)
    add_text(board, "CAN FD", 22, 36, 1.2)
    add_text(board, "REDUNDANT IMU / BARO", 59, 98, 1.1)
    add_text(board, "GNSS 50R / 3mm COURTYARD", 103, 34.5, 1.0)
    add_text(board, "915 MHz RF - 50R / VIA FENCE", 108, 98, 1.1)
    add_silk_line(board, (77, 36), (77, height - 1.0), 0.4)
    add_silk_line(board, (77, 36), (width - 1.0, 36), 0.4)
    add_mounting_holes(
        board,
        MAIN_DIR,
        [(4, 4), (width - 4, 4), (4, height - 4), (width - 4, height - 4)],
    )
    move_footprint_silks_to_fab(board)
    output = MAIN_DIR / "main_rev_b.kicad_pcb"
    pcbnew.SaveBoard(str(output), board)
    apply_stackup(output, outer_oz=1.0, inner_oz=1.0)
    return output


def apply_stackup(path: Path, outer_oz: float, inner_oz: float) -> None:
    """Persist an explicit 1.6 mm four-layer stackup in KiCad's board file."""
    outer = 0.035 * outer_oz
    inner = 0.035 * inner_oz
    dielectric_total = 1.6 - 2 * outer - 2 * inner - 0.02
    prepreg = 0.20
    core = dielectric_total - 2 * prepreg
    stackup = f"""
\t\t(stackup
\t\t\t(layer \"F.SilkS\" (type \"Top Silk Screen\"))
\t\t\t(layer \"F.Paste\" (type \"Top Solder Paste\"))
\t\t\t(layer \"F.Mask\" (type \"Top Solder Mask\") (thickness 0.01))
\t\t\t(layer \"F.Cu\" (type \"copper\") (thickness {outer:.4f}))
\t\t\t(layer \"dielectric 1\" (type \"prepreg\") (thickness {prepreg:.4f}) (material \"FR4\") (epsilon_r 4.2) (loss_tangent 0.02))
\t\t\t(layer \"In1.Cu\" (type \"copper\") (thickness {inner:.4f}))
\t\t\t(layer \"dielectric 2\" (type \"core\") (thickness {core:.4f}) (material \"FR4\") (epsilon_r 4.2) (loss_tangent 0.02))
\t\t\t(layer \"In2.Cu\" (type \"copper\") (thickness {inner:.4f}))
\t\t\t(layer \"dielectric 3\" (type \"prepreg\") (thickness {prepreg:.4f}) (material \"FR4\") (epsilon_r 4.2) (loss_tangent 0.02))
\t\t\t(layer \"B.Cu\" (type \"copper\") (thickness {outer:.4f}))
\t\t\t(layer \"B.Mask\" (type \"Bottom Solder Mask\") (thickness 0.01))
\t\t\t(layer \"B.Paste\" (type \"Bottom Solder Paste\"))
\t\t\t(layer \"B.SilkS\" (type \"Bottom Silk Screen\"))
\t\t\t(copper_finish \"ENIG\")
\t\t\t(dielectric_constraints yes)
\t\t)
"""
    text = path.read_text(encoding="utf-8")
    marker = "\t(setup\n"
    if marker not in text:
        raise RuntimeError(f"Could not locate setup block in {path}")
    text = text.replace(marker, marker + stackup, 1)
    path.write_text(text, encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--board", choices=("esc", "main", "both"), default="both")
    args = parser.parse_args()
    if args.board in ("esc", "both"):
        esc = generate_esc()
        print(f"Generated {esc}")
    if args.board in ("main", "both"):
        control = generate_main()
        print(f"Generated {control}")


if __name__ == "__main__":
    main()
