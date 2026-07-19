# Hardware validation sources

Primary manufacturer documents used for the review are listed below. Most are
mirrored under `docs/datasheets` and `docs/reference-designs`; the listed
manufacturer URL remains authoritative and should be checked for a newer
revision before a production spin.

## Motor cell and current acquisition

| Local file | Primary source |
| --- | --- |
| `docs/datasheets/esc/ti-drv8353.pdf` | https://www.ti.com/lit/ds/symlink/drv8353.pdf |
| `docs/datasheets/esc/infineon-iptc014n10nm5.pdf` | https://www.infineon.com/assets/row/public/documents/24/49/infineon-iptc014n10nm5-datasheet-en.pdf |
| `docs/datasheets/esc/bourns-css4j-4026.pdf` | https://www.bourns.com/docs/product-datasheets/css4j-4026.pdf |
| `docs/datasheets/esc/ti-ina296a.pdf` | https://www.ti.com/lit/ds/symlink/ina296a.pdf |
| `docs/datasheets/esc/st-stm32g431cb.pdf` | https://www.st.com/resource/en/datasheet/stm32g431cb.pdf |
| `docs/reference-designs/esc/st-an5093-stm32g4-hardware.pdf` | https://www.st.com/resource/en/application_note/an5093-getting-started-with-stm32g4-series-hardware-development-stmicroelectronics.pdf |
| `docs/datasheets/esc/ti-iso6731.pdf` | https://www.ti.com/lit/ds/symlink/iso6731.pdf |
| `docs/datasheets/esc/recom-rfm.pdf` | https://recom-power.com/pdf/Econoline/RFM.pdf |
| `docs/datasheets/esc/ti-tps709.pdf` | https://www.ti.com/lit/ds/symlink/tps709.pdf |
| `docs/datasheets/esc/littelfuse-sm8s.pdf` | https://www.littelfuse.com/assetdocs/littelfuse-tvs-diode-sm8s-datasheet |
| `docs/reference-designs/esc/infineon-eval-tolt-dc48v-3kw.pdf` | https://www.infineon.com/assets/row/public/documents/24/44/infineon-evaluation-board-eval-tolt-dc48v-3kw-usermanual-en.pdf |
| `docs/reference-designs/esc/ti-motor-driver-layout.pdf` | https://www.ti.com/lit/an/slva959b/slva959b.pdf |

## ESC auxiliary power and supervision

| Local file | Primary source |
| --- | --- |
| `docs/datasheets/esc/ti-lm5164.pdf` | https://www.ti.com/lit/ds/symlink/lm5164.pdf |
| `docs/datasheets/esc/ti-tps3430.pdf` | https://www.ti.com/lit/ds/symlink/tps3430.pdf |
| `docs/datasheets/esc/diodes-ap2112.pdf` | https://www.diodes.com/assets/Datasheets/AP2112.pdf |
| `docs/datasheets/esc/ti-sn74lvc1g08.pdf` | https://www.ti.com/lit/ds/symlink/sn74lvc1g08.pdf |

## CAN FD on both boards

| Local file | Primary source |
| --- | --- |
| `docs/datasheets/common-microchip-mcp2518fd.pdf` | https://ww1.microchip.com/downloads/en/DeviceDoc/External-CAN-FD-Controller-with-SPI-Interface-DS20006027B.pdf |
| `docs/datasheets/common-ti-tcan3413.pdf` | https://www.ti.com/lit/ds/symlink/tcan3413.pdf |
| `docs/datasheets/common-nexperia-pesd2canfd24l-t.pdf` | https://assets.nexperia.com/documents/data-sheet/PESD2CANFD24L-T.pdf |

## Flight-control MCU, USB, sensors, and GNSS

| Local file | Primary source |
| --- | --- |
| `docs/datasheets/main/rp2350-datasheet.pdf` | https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf |
| `docs/datasheets/main/rp2350-hardware-design.pdf` | https://datasheets.raspberrypi.com/rp2350/hardware-design-with-rp2350.pdf |
| `docs/datasheets/main/ti-tlv62569.pdf` | https://www.ti.com/lit/ds/symlink/tlv62569.pdf |
| `docs/datasheets/main/ti-tlv755p.pdf` | https://www.ti.com/lit/ds/symlink/tlv755p.pdf |
| `docs/datasheets/main/tdk-icm42688p.pdf` | https://invensense.tdk.com/wp-content/uploads/2020/04/ds-000347_icm-42688-p-datasheet.pdf |
| `docs/datasheets/main/st-lsm6dso32.pdf` | https://www.st.com/resource/en/datasheet/lsm6dso32.pdf |
| `docs/datasheets/main/st-lps22df.pdf` | https://www.st.com/resource/en/datasheet/lps22df.pdf |
| `docs/datasheets/main/quectel-lg77l-hardware-design.pdf` | https://forums.quectel.com/uploads/short-url/9zAmjO9mANigfC05VclI7cICxed.pdf |

## 915 MHz radio

| Local file | Primary source |
| --- | --- |
| `docs/datasheets/main/ti-cc1121.pdf` | https://www.ti.com/lit/ds/symlink/cc1121.pdf |
| `docs/datasheets/main/ti-cc1190.pdf` | https://www.ti.com/lit/ds/symlink/cc1190.pdf |
| `docs/reference-designs/main/ti-swrc224.zip` | https://www.ti.com/lit/zip/swrc224 |
| `docs/reference-designs/main/ti-swrr145a.zip` | https://www.ti.com/lit/zip/swrr145 |
| `docs/reference-designs/main/ti-cc112x-cc1190-boosterpack.pdf` | https://www.ti.com/lit/an/swra492/swra492.pdf |

TI explicitly recommends following the characterized CC112x/CC1190 reference
layout. The checked-in component values and placement are therefore only the
starting point: the production stackup requires VNA tuning and conducted/
radiated compliance testing.

## Legacy documents retained for the failure audit

| Local file | Primary source |
| --- | --- |
| `docs/datasheets/esc/infineon-ir2136.pdf` | https://www.infineon.com/assets/row/public/documents/24/49/infineon-ir213-ds-en.pdf |
| `docs/datasheets/esc/ti-ina240.pdf` | https://www.ti.com/lit/ds/symlink/ina240.pdf |
| `docs/datasheets/esc/ti-tla2528.pdf` | https://www.ti.com/lit/ds/symlink/tla2528.pdf |
| `docs/datasheets/esc/nexperia-buk9k12-80l.pdf` | https://assets.nexperia.com/documents/data-sheet/BUK9K12-80L.pdf |
| `docs/datasheets/esc/ti-lm5148-q1.pdf` | https://www.ti.com/lit/ds/symlink/lm5148-q1.pdf |
| `docs/datasheets/esc/infineon-iptc015n10nm5.pdf` | https://www.infineon.com/dgdl/Infineon-IPTC015N10NM5-DataSheet-v02_00-EN.pdf |
