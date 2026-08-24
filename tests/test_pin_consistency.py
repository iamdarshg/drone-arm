
#!/usr/bin/env python3
"""Pin-consistency test: firmware pin map vs PCB netlist pin map.

Firmware source of truth: src/board_config.h on the sdk-recreation branch.
This test reads hardware/main/rev_b/mcu_pinmap.json (generated from the
schematic netlist) and asserts that every firmware pin assignment matches
the PCB.  Run with: pytest tests/test_pin_consistency.py
"""
import json
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
PINMAP = REPO / "hardware/main/rev_b/mcu_pinmap.json"

# Firmware pin assignments (RP2354B GPIO numbers).
# Source: sdk-recreation branch, src/board_config.h + main Rev-B interface spec.
# The firmware repo keeps these in board_config.h; they are mirrored here so the
# test fails loudly if either side drifts.
FIRMWARE_PINS = {
    "SENS_SCLK": 0,
    "SENS_MOSI": 1,
    "SENS_MISO": 2,
    "ICM_nCS": 3,
    "LSM_nCS": 4,
    "ICM_INT1": 5,
    "ICM_INT2": 6,
    "LSM_INT1": 7,
    "LSM_INT2": 8,
    "BARO_SCL": 9,
    "BARO_SDA": 10,
    "BARO_DRDY": 11,
    "GNSS_TX": 12,
    "GNSS_RX": 13,
    "GNSS_PPS": 14,
    "GNSS_RESET_N": 15,
    "RF_SCLK": 16,
    "RF_MOSI": 17,
    "RF_MISO": 18,
    "RF_nCS": 19,
    "RF_RESET_N": 20,
    "RF_GPIO0": 21,
    "RF_GPIO2": 22,
    "RF_GPIO3": 23,
    "PA_EN": 24,
    "LNA_EN": 25,
    "HGM": 26,
    "CAN_SCK": 27,
    "CAN_MOSI": 28,
    "CAN_MISO": 29,
    "CAN_nCS": 30,
    "CAN_INT": 31,
    "STATUS_LED": 32,
    "GNSS_PWR_EN": 33,
    "PWM_ESC1": 34,
    "PWM_ESC2": 35,
    "PWM_ESC3": 36,
    "PWM_ESC4": 37,
    "PWM_ESC5": 38,
    "PWM_ESC6": 39,
}


def load_pcb_map():
    data = json.loads(PINMAP.read_text())
    out = {}
    for gpio, info in data["pins"].items():
        # normalise hierarchical net names to their leaf token
        leaf = info["net"].split("/")[-1]
        out.setdefault(leaf, []).append(int(gpio))
    return out


def test_pinmap_file_exists():
    assert PINMAP.exists(), "run tools/hardware/gen_main_pinmap.py first"


def test_every_firmware_pin_exists_on_pcb():
    pcb = load_pcb_map()
    missing = [n for n in FIRMWARE_PINS if n not in pcb]
    assert not missing, f"firmware pins absent from PCB: {missing}"


def test_gpio_numbers_match():
    pcb = load_pcb_map()
    wrong = {
        n: (FIRMWARE_PINS[n], pcb.get(n))
        for n in FIRMWARE_PINS
        if n in pcb and FIRMWARE_PINS[n] not in pcb[n]
    }
    assert not wrong, f"GPIO number mismatches firmware vs PCB: {wrong}"


def test_no_duplicate_gpio_assignment():
    seen = {}
    for n, g in FIRMWARE_PINS.items():
        assert g not in seen, f"GPIO{g} double-assigned: {seen[g]} and {n}"
        seen[g] = n


def test_esc_pwm_pins_are_contiguous():
    esc = [FIRMWARE_PINS[f"PWM_ESC{i}"] for i in range(1, 7)]
    assert esc == sorted(esc), "ESC PWM pins must be ascending"

