# Bounded high-value ratsnest audit from `f653d8c`

The source PCB is unchanged at SHA-256 `095aa3fd44285b885c2a4ca68c05633b169dd053fbc8407d2160e8585e73336b`; matching project SHA-256 `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`. A fresh explicit `pcbnew.ZONE_FILLER.Fill(board.Zones())` and KiCad 9.0.2 `pcb drc --severity-all --format json` baseline gives **7 DRC violations / 42 unconnected items**.

## Remaining airwires by net class

| Net class | Open count | High-value links in the remaining set |
| --- | ---: | --- |
| `POWER` | 4 | `/1V1` U10.65 `(61.2,45.05)`→U10.51 `(63.95,49.8)` and an existing `/1V1` track→U10.51; `/3V3_ADC` existing track `(50.8132,48.5257)`→U10.59 `(63.95,46.6)`; `/VREG_AVDD` U10.61 `(62.8,45.05)`→existing In1 track endpoint `(52.3067,41.5)`. |
| `Default` | 37 | Includes two GND plane/pad opens; USB MCU-side links R11.2→U10.66 (`/USB_DM_MCU`) and R10.2→U10.67 (`/USB_DP_MCU`); the remaining CAN, sensor, GNSS, debug, MCU/RF control and reset links. |
| `RF_50R` | 1 | `/915 MHz RF/CC_LNA_P` U40.19→FL40.7. |

The board netlist contains no servo, motor, ESC, or PWM-labelled nets, so there is no servo ratline to route in this project revision.

## Focused USB probe

I tested one alternate `/USB_DP_MCU` topology on a disposable source copy: U10.67 `(60.4,45.05)` F.Cu to a 0.60/0.30 mm through-via at `(61.2,43.8)`; R10.2 `(56.53,37.1)` F.Cu to a through-via at `(55.3,37.1)`; a 0.20 mm B.Cu span joins the vias. Source and candidate were separately zone-refilled using `pcbnew.ZONE_FILLER.Fill(board.Zones())` and checked with KiCad CLI 9.0.2 `pcb drc --severity-all --format json` and the same-stem matching project. Baseline JSON SHA-256: [`route_probe_f653d8c_usb_dp_focus_before.json`](route_probe_f653d8c_usb_dp_focus_before.json), `eecc8de27a0a3762ae415728c80307f969c27aae58ed65a4d13687a01c16a941`.

Candidate board SHA-256 `8f06bb96148e7f30ef961d13576d6847ca3283b677b859b46eeb860b0d370a48`; candidate JSON [`route_probe_f653d8c_usb_dp_focus_candidate.json`](route_probe_f653d8c_usb_dp_focus_candidate.json), SHA-256 `460f6fe1032f66e45bc5930a6da4a3c977e22667f37e4426c9b1bc23556dfca2`. DRC is **18 violations / 41 unconnected**, with 11 new signatures. New issues include shorts between the candidate and `/3V3`, GND, `/USB_DP`, and `/USB_DM_MCU`, plus crossing, clearance and three solder-mask bridges. It is rejected; no USB route or PCB change was promoted.

## Handoff recommendation

The high-value power and USB links do not have a legal corridor in the current MCU escape area under the unchanged seven-signature DRC baseline. Existing reports cover multiple failed `/1V1`, `/3V3_ADC`, `/VREG_AVDD`, `/USB_DM_MCU`, and `/USB_DP_MCU` routes; see the 1V1, sensor-access, USB and zone-refill reports alongside this file. Keep the MCU USB pair unrouted for a board/layout engineer to rework as a controlled differential pair. Rework `/1V1`, `/3V3_ADC`, and `/VREG_AVDD` as a coordinated power escape, and relocate the GND plane stitch outside R50’s courtyard/pad-mask region. Leave the remaining U10-to-peripheral nets open for that same layout pass. No board or design expectation was changed.
