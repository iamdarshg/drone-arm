# Final Retail BOM — Four Prototype Motors

Date: 2026-08-26

Scope: four 12-slot / 8-pole outrunner prototype motors. Stator electrical-steel sheets and stator-lamination cutting are intentionally excluded. This BOM follows the current conversation-level geometry (42 mm stator OD, 20 mm active stack, 62 mm maximum motor OD, 45 mm maximum mechanical height, 8 mm shaft) and supersedes the obsolete 51 mm OD / 56 mm stack quantities in the older release package.

## Frozen or selected items

| Item | Specification | Purchase quantity for four motors | Unit price (INR) | Extended price (INR) | Status / notes |
|---|---|---:|---:|---:|---|
| Rotor magnets | 20 x 10 x 2 mm, N42 NdFeB, one layer, 8 poles per motor | 50 pieces | 42.80 | 2,140 | 32 installed; 18 spare for grading, matching and breakage. Measure Br consistency and reject damaged plating. |
| Rotor flux-liner sheet | 0.5 mm ungalvanized cold-rolled mild steel, approximately 300 x 600 mm | 1 sheet | 1,400 | 1,400 | Frozen choice. Form two nested liner layers per rotor with offset seams. One sheet is ample for four rotors and coupons. |
| Shaft stock | 8 mm ground steel round, 500 mm long | 1 rod | 315 | 315 | Free shaft machining assumed. Verify straightness, hardness, bearing fits and final shoulder geometry. |
| Bearings | 608ZZ, 8 x 22 x 7 mm | 10 pieces | 30 | 300 | Eight installed plus two spare. Generic bearings require speed, grease, clearance and runout screening before 35 krpm use. |
| Slot insulation | Nomex, 7 mil (approximately 0.178 mm), 900 x 300 mm | 2 sheets | 270 | 540 | One may be enough geometrically; two allows forming trials, slot liners, phase separators and rejects. |
| Winding impregnation varnish | Elantas V2603 or exact selected Class-H-compatible grade | 1 litre | 1,000 | 1,000 | Confirm exact TDS, cure schedule and compatibility with Nomex and wire enamel. |
| Winding thermistors | 10 k NTC, selected B-value; three per motor | 12 pieces | 6 | 72 | Add four spares if the seller permits. Electrically isolate and calibrate assembled sensors. |
| Hall sensors | Exact selected Hall switch, three per motor | 12 pieces | 62 | 744 | Product identity and electrical interface must be frozen before purchase. Omit if the final controller is sensorless. |
| End-cap / fillet adhesive | Lapox Ultra, 180 g | 1 kit | 349 Amazon / about 211 alternate | 349 budgeted | For keyed end-cap joints and fillets, not fibre impregnation. Maintain 100:80 mass ratio and thin controlled bond lines. |
| Thin magnet adhesive | User-selected small-pack Araldite from Amazon | 1 kit | TBD | TBD | Use only after steel-to-NdFeB lap/shear and thermal-cycle coupons. Remove plating contamination and control bond-line thickness. |
| Composite matrix | Lapox L-12 resin + AH-411 hardener, 100:22 by mass | 1.0 kg resin + 0.22-0.25 kg hardener | 800-1,800 per usable set | 800-1,800 | Primary inner-glass / UD-carbon / outer-glass matrix. Post-cure empty liner/sleeve before magnets are installed. |
| UD hoop reinforcement | 12K UD carbon cloth/tape, approximately 300 gsm | Minimum 1 m x 0.3-0.5 m usable area | TBD | TBD | Circumferential primary retention reinforcement. Purchase enough for four rings plus at least three witness rings. |
| Glass isolation and outer ply | Light woven E-glass cloth, approximately 100-200 gsm | Approximately 1 square metre | TBD | TBD | One inner electrical-isolation ply and one outer handling/damage-indication ply minimum. |
| Phase leads and connectors | Existing stock | 4 motor sets | Free | 0 | Three phase leads per motor; exact gauge and strain relief remain dependent on current and termination geometry. |
| Magnet installation adhesive already in stock | Existing stock, if retained as alternate | As available | Free | 0 | Compare against Araldite coupons; do not stack incompatible adhesive systems in one bond line. |
| Aluminium stock | 6061-T6 for four stator carriers and four rotor end caps | Final CAD-dependent | TBD | TBD | Obtain after corrected CAD fixes blank diameters, axial lengths, bearing seats and cooling ducts. |
| Stator-carrier machining | Four identical 6061-T6 parts | 4 | Quote pending | TBD | Seek one setup plus four-piece batch pricing. |
| Rotor-end-cap machining | Four identical 6061-T6 parts | 4 | Quote pending | TBD | Carbon/steel liner design removes the former fully machined rotor bell. |
| Composite spacers / assembly fixtures | High-temperature printed polymer or machined fixture material | 1 set | TBD | TBD | Printed magnet spacers are retained in the rotor; cure and balance fixtures must tolerate the selected post-cure. |
| Balancing material | Small removable balance weights or qualified balancing compound | 1 kit | TBD | TBD | Do not use random resin blobs as the first balancing method. |
| Shaft retaining hardware | Prop adapter/nut, spacers, washers and retention feature | 4 sets | TBD | TBD | Final shaft and prop interface dependent. |

## Geometry-dependent items that are not yet safe to purchase in bulk

| Item | Present requirement | Why quantity remains open |
|---|---|---|
| Magnet wire | Class H or better, likely parallel 27-32 AWG enamelled copper strands | Turns, strands per turn, mean turn length, phase resistance and slot fill must be recomputed from the corrected 42 mm / 20 mm active geometry. The obsolete 12-AWG and 51 mm-stack figures must not be reused. |
| Aluminium blanks | 6061-T6 | Corrected carrier/end-cap STEP files must freeze bearing seats, ducts, shaft interface and composite overlap before stock size is ordered. |
| Fasteners and retaining rings | Alloy/stainless as appropriate | Sizes depend on the corrected shaft and end-cap drawings. |
| Propeller adapter | For selected 5.5-inch high-speed propeller | Requires final prop hub dimensions and verified shaft/end-cap load path. |

## Explicitly excluded

- CRNO/CRNGO stator electrical-steel sheets.
- Cutting/punching of stator laminations.
- Batteries, ESC/controller, airframe and propellers.
- Free dyno testing, free shaft machining, free phase leads/connectors and any free adhesive already in stock.

## Cost snapshot

Known-price subtotal, excluding the stator core, wire, carbon/glass fabric, aluminium and machining: **INR 6,860**.

Including the expected Lapox L-12/AH-411 matrix purchase: **INR 7,660-8,660**.

This is not the finished-project total. The dominant unresolved costs are aluminium/CNC work, winding copper, carbon/glass reinforcement, fixtures and high-speed-rated bearing replacement if the INR 30 608ZZ samples fail screening.

## Loctite EA E-30CL disposition

Do not use EA E-30CL as the sleeve laminating resin. Henkel identifies it as a 2:1 cartridge structural epoxy with approximately 10,000 cP mixed viscosity, 30-minute work life and approximately 70 C glass-transition temperature. It is useful only as an optional clear joint adhesive after coupon testing. It also requires the correct 2:1 cartridge gun and static mixing nozzles, adding cost and waste.

