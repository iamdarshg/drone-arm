# Rev-B manual route probes on `c42c8ea`

Report-only routing experiments using KiCad pcbnew and CLI 9.0.2. The source board and matching project are hash-pinned below. Baseline DRC had 7 violations and 45 unconnected items. Each candidate connected one net but introduced new DRC violations, so both are rejected.

- Source commit: `c42c8eaca8bbc723ceeeea45a1228361e6c86b86`
- Source board SHA-256: `a21b010eba158e7b3ecfeb0bfdb8c98de9c8d62212f9f7958ffbba0e6b945824`
- Matching project SHA-256: `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`
- Baseline DRC JSON SHA-256: `a3c1d57c6869e33a0768e90939fb52c445a42d5c4465ad4cff0711eb18138213`

| Probe | Candidate board SHA-256 | DRC JSON SHA-256 | Result |
| --- | --- | --- | --- |
| `/RP2354B MCU/XIN`, pad U10.30 to a new via placed at endpoint of existing In1.Cu track | `c9a1bdca17d1bbccd73015170c8ad9987581597aef15912821cb77b516de3e88` | `05e2bb9f89e1fbe34d56a4e264a1040bbb2e017a4d717e8c413c02e38ad2208c` | 26 violations, 44 unconnected. Rejected due new shorts to `/3V3` and `/XOUT`, crossings, clearance/zone/hole-clearance, annular-width, via-diameter and mask-bridge issues. |
| `/RP2354B MCU/XOUT`, straight F.Cu U10.31 to existing F.Cu track | `b8e9f012b0731866cacbc4504434af6d7a02a97c2385dc40ea3cd4774566f602` | `78bed2e1251046e8509d05c5256da36f854840d47d1c6a5117ffa798d6e78bbe` | 23 violations, 44 unconnected. Rejected due new shorts to `/1V1`, `/GNSS_PWR_EN`, `/3V3`, crossings, clearance/zone and mask-bridge issues. |

The complete before and candidate DRC JSON files accompany this record. No candidate PCB was promoted.
