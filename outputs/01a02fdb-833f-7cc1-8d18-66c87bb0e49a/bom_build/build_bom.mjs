import fs from "node:fs/promises";
import { Workbook, SpreadsheetFile } from "@oai/artifact-tool";

const outDir = "D:/CodeProjects/drone-arm/outputs/01a02fdb-833f-7cc1-8d18-66c87bb0e49a";
const outFile = `${outDir}/FINAL_MOTOR_BOM_4_UNITS.xlsx`;
const previewDir = `${outDir}/bom_previews`;

const wb = Workbook.create();
const summary = wb.worksheets.add("Summary");
const bom = wb.worksheets.add("Purchase BOM");
const cnc = wb.worksheets.add("CNC Stock");
const wire = wb.worksheets.add("Wire Estimate");
const infusion = wb.worksheets.add("Infusion Budget");
const sources = wb.worksheets.add("Sources & Gates");
wb.comments.setSelf({ displayName: "Darsh Gupta" });

const navy = "#17324D";
const blue = "#2F75B5";
const paleBlue = "#D9EAF7";
const green = "#DDEBF7";
const paleGreen = "#E2F0D9";
const amber = "#FFF2CC";
const red = "#F4CCCC";
const gray = "#E7E6E6";
const white = "#FFFFFF";
const inputBlue = "#EAF3F8";

function title(sheet, range, text) {
  sheet.getRange(range).merge();
  sheet.getRange(range).values = [[text]];
  sheet.getRange(range).format = {
    fill: navy,
    font: { bold: true, color: white, size: 16 },
    verticalAlignment: "center",
  };
  sheet.getRange(range).format.rowHeight = 30;
}

function header(range) {
  range.format = {
    fill: blue,
    font: { bold: true, color: white },
    wrapText: true,
    verticalAlignment: "center",
    borders: { preset: "outside", style: "thin", color: "#9EADBA" },
  };
  range.format.rowHeight = 30;
}

function section(range) {
  range.format = { fill: paleBlue, font: { bold: true, color: navy } };
}

for (const sh of [summary, bom, cnc, wire, infusion, sources]) sh.showGridLines = false;

// ---------------- Summary ----------------
title(summary, "A1:H1", "Four-Motor Retail BOM — Current Selected Path");
summary.getRange("A3:B11").values = [
  ["Design basis", "Current value"],
  ["Motor quantity", 4],
  ["Stator topology", "12-slot / 8-pole"],
  ["Stator OD", "42 mm"],
  ["Active stack", "20 mm"],
  ["Maximum motor envelope", "62 mm OD × 45 mm high"],
  ["Shaft", "8 mm ground steel"],
  ["Rotor liner", "Two nested 0.5 mm mild-steel layers"],
  ["Stator core path", "3× REES52 4007 stacks per motor"],
];
header(summary.getRange("A3:B3"));
summary.getRange("A4:B11").format.borders = { preset: "inside", style: "thin", color: "#D9E1E8" };

summary.getRange("D3:F8").values = [
  ["Cost summary", "Formula-driven value", "Interpretation"],
  ["Selected purchase subtotal", null, "Includes twelve donor stators, provisional wire and 6061 stock; excludes CNC labour"],
  ["Known/frozen retail items", null, "Rows currently marked BUY in Purchase BOM"],
  ["6061-T6 stock allowance", null, "Material only for three CNC parts × four motors"],
  ["Provisional 24 SWG wire", null, "Replace checkout price and winding assumptions before buying"],
  ["Unpriced HOLD rows", null, "Carbon and E-glass are priced through the separate infusion-cart subtotal"],
];
header(summary.getRange("D3:F3"));
summary.getRange("E4").formulas = [["=ROUND(SUM('Purchase BOM'!$J$5:$J$18),0)"]];
summary.getRange("E5").formulas = [["=ROUND(SUMIF('Purchase BOM'!$K$5:$K$18,\"BUY\",'Purchase BOM'!$J$5:$J$18),0)"]];
summary.getRange("E6").formulas = [["=ROUND('CNC Stock'!$J$14,0)"]];
summary.getRange("E7").formulas = [["=ROUND('Wire Estimate'!$H$12,0)"]];
summary.getRange("E8").formulas = [["=COUNTIF('Purchase BOM'!$I$5:$I$18,0)"]];
summary.getRange("E4:E7").format.numberFormat = "₹#,##0";
summary.getRange("E8").format.numberFormat = "0";
summary.getRange("D4:F8").format.borders = { preset: "inside", style: "thin", color: "#D9E1E8" };
summary.getRange("E4:E7").format.fill = paleGreen;
summary.getRange("E8").format.fill = amber;

summary.getRange("D10:F13").values = [
  ["Confirmed infusion-path cash", "Value", "Treatment"],
  ["User-confirmed materials subtotal", 4271, "Includes the Carbon Fiber India cart; do not add its visible lines again"],
  ["Vacuum pump", 6800, "Reusable capital/tooling purchase"],
  ["Immediate cash outlay", null, "Materials subtotal plus pump"],
];
header(summary.getRange("D10:F10"));
summary.getRange("E13").formulas = [["=ROUND('Infusion Budget'!$E$20,0)"]];
summary.getRange("E11:E13").format.numberFormat = "₹#,##0";
summary.getRange("D11:F13").format.borders = { preset: "inside", style: "thin", color: "#D9E1E8" };
summary.getRange("E11:E12").format.fill = inputBlue;
summary.getRange("D13:F13").format = { fill: paleGreen, font: { bold: true, color: navy } };

summary.getRange("D14:F14").values = [["Grand project cash", null, "Main motor BOM plus infusion materials and reusable pump"]];
summary.getRange("E14").formulas = [["=ROUND(E4+'Infusion Budget'!$E$20,0)"]];
summary.getRange("E14").format.numberFormat = "₹#,##0";
summary.getRange("D14:F14").format = { fill: navy, font: { bold: true, color: white } };

summary.getRange("A16:H20").values = [
  ["Critical checkout gates", null, null, null, null, null, null, null],
  ["1", "Do not buy the linked B0CVS4KBZ6 magnets: that listing is ceramic/ferrite, not the agreed N42/N52 NdFeB rotor magnet.", null, null, null, null, null, null],
  ["2", "The selected 24 SWG ART IFACT wire is only provisionally listed. The public description says modified-polyurethane enamel, not verified Class H. Obtain thermal-class and dielectric data first.", null, null, null, null, null, null],
  ["3", "₹30 608ZZ bearings are prototype samples. Verify manufacturer speed rating, grease, clearance, temperature rise and runout before any 35 krpm or overspeed test.", null, null, null, null, null, null],
  ["4", "CNC stock dimensions are planning blanks, not released manufacturing dimensions. Update them from corrected STEP/drawings before ordering cut pieces.", null, null, null, null, null, null],
];
summary.getRange("A16:H16").merge();
summary.getRange("A16:H16").format = { fill: red, font: { bold: true, color: "#7F0000" } };
for (let r = 17; r <= 20; r++) {
  summary.getRange(`B${r}:H${r}`).merge();
  summary.getRange(`A${r}:H${r}`).format = { fill: amber, wrapText: true, verticalAlignment: "top" };
  summary.getRange(`A${r}:H${r}`).format.rowHeight = 38;
}
summary.getRange("A:A").format.columnWidth = 18;
summary.getRange("B:B").format.columnWidth = 38;
summary.getRange("C:C").format.columnWidth = 3;
summary.getRange("D:D").format.columnWidth = 26;
summary.getRange("E:E").format.columnWidth = 18;
summary.getRange("F:F").format.columnWidth = 48;
summary.getRange("G:H").format.columnWidth = 12;
summary.freezePanes.freezeRows(1);

// ---------------- Purchase BOM ----------------
title(bom, "A1:M1", "Purchase BOM — Four Motors with REES52 Donor Stator Cores");
bom.getRange("A2:M2").merge();
bom.getRange("A2:M2").values = [["Blue cells are editable inputs. Extended costs and stock quantities are formulas. Prices are retail estimates and should be overwritten with checkout values."]];
bom.getRange("A2:M2").format = { fill: inputBlue, font: { italic: true, color: navy }, wrapText: true };
bom.getRange("A4:M4").values = [["ID", "Category", "Item", "Selected specification", "Pack / stock unit", "Installed qty", "Spare qty", "Purchase qty", "Unit price INR", "Extended INR", "Status", "Retail source", "Engineering note"]];
header(bom.getRange("A4:M4"));
const rows = [
  [1,"Rotor","NdFeB magnets","20×10×2 mm; N42/N52; thickness magnetized","piece",32,18,50,42.8,null,"BUY","User retail quote; Amazon ferrite link rejected","Grade and magnetization direction require verification; 50 pieces allows 18 spares."],
  [2,"Rotor","Flux-liner sheet","0.5 mm cold-rolled ungalvanized mild steel; approx. 300×600 mm","sheet",1,0,1,1400,null,"BUY","https://www.amazon.in/dp/B0F2NBWV58","Frozen choice; form two nested rings per rotor with seams offset."],
  [3,"Machined stock","6061-T6 stock for three CNC parts","Four stator carriers + four rotor end caps + four prop hubs","lot",1,0,1,null,null,"PROVISIONAL","See CNC Stock sheet","Material only; CNC labour excluded."],
  [4,"Adhesive","Lapox Ultra","180 g two-part epoxy; general joints, magnet coupons and fillets","kit",1,0,1,349,null,"BUY","https://www.amazon.in/dp/B0B6C4BTZ2","Use thin controlled bond lines and keyed/mechanical joints; coupon-test magnets first."],
  [5,"Bearing","608ZZ bearings","8×22×7 mm, metal shielded","piece",8,2,10,30,null,"PROVISIONAL","https://www.amazon.in/dp/B01LXLWLFK","Prototype stock only until speed/grease/runout are verified."],
  [6,"Shaft","Ground shaft stock","8 mm diameter × 500 mm long","rod",1,0,1,315,null,"BUY","User retail quote","One rod is sufficient for four shafts plus setup allowance; machining assumed free."],
  [7,"Insulation","Nomex sheet","7 mil (0.178 mm), 900×300 mm","sheet",1,1,2,270,null,"BUY","https://www.amazon.in/dp/B09WKV41J1","Second sheet covers forming trials, phase barriers and rejects."],
  [8,"Winding","Winding varnish","ELANTAS V2603 or exact selected Class-H-compatible varnish; 1 L","litre",1,0,1,1000,null,"BUY","User retail quote; VASK listing retained only as unapproved alternative","Do not substitute electronic potting resin without exact thermal/electrical TDS."],
  [9,"Sensor","NTC thermistor","10 kΩ; selected B-value; three per motor","piece",12,4,16,6,null,"BUY","User retail quote","Calibrate and electrically isolate; four spares included."],
  [10,"Sensor","Hall-effect sensors","Exact switch/interface part to be frozen; three per motor","piece",12,3,15,62,null,"PROVISIONAL","User retail quote","Delete if final commutation is sensorless; verify voltage and magnetic polarity."],
  [11,"Winding","24 SWG magnet wire","ART IFACT; 0.56 mm conductor; 50 m roll","roll",null,null,null,596,null,"PROVISIONAL","https://www.amazon.in/dp/B07LCRM82G","Recommended of the two linked sizes for bundle practicality; Class H is not verified."],
  [12,"Composite","UD carbon reinforcement","12K UD carbon, approx. 300 gsm","lot",1,0,1,0,null,"HOLD","Retail source/price not frozen","Primary circumferential hoop reinforcement; price intentionally blank."],
  [13,"Composite","Woven E-glass","100–200 gsm; approx. 1 m²","lot",1,0,1,0,null,"HOLD","Retail source/price not frozen","Inner galvanic isolation and outer protection plies."],
  [14,"Stator core","REES52 5010-750Kv donor motors","Three 40.6×7.4 mm 12-slot stators per motor; 22.2 mm combined stack; retain custom 8-pole rotor","motor",12,0,12,1399,null,"BUY","https://rees52.com/products/5010-750kv-brushless-motor-5010-750kv-high-torque-brushless-motor-for-drone-rs2931","Twelve donors for four motors. Verify lamination thickness and align all three slot stacks before final winding."],
];
bom.getRange("A5:M18").values = rows;
for (let r = 5; r <= 18; r++) bom.getRange(`J${r}`).formulas = [[`=ROUND(H${r}*I${r},0)`]];
bom.getRange("I5:I18").format.fill = inputBlue;
bom.getRange("F5:I18").format.numberFormat = "#,##0.00";
bom.getRange("J5:J18").format.numberFormat = "₹#,##0";
bom.getRange("H7").formulas = [["=1"]];
bom.getRange("I7").formulas = [["=ROUND('CNC Stock'!$J$14,0)"]];
bom.getRange("H15").formulas = [["='Wire Estimate'!$G$12"]];
bom.getRange("K5:K18").dataValidation = { rule: { type: "list", values: ["BUY", "PROVISIONAL", "HOLD", "REJECT"] } };
bom.getRange("A4:M18").format.borders = { preset: "inside", style: "thin", color: "#D9E1E8" };
bom.getRange("A19:I19").merge();
bom.getRange("A19:I19").values = [["Selected BOM subtotal"]];
bom.getRange("J19").formulas = [["=ROUND(SUM(J5:J18),0)"]];
bom.getRange("A19:J19").format = { fill: navy, font: { bold: true, color: white } };
bom.getRange("J19").format.numberFormat = "₹#,##0";
bom.getRange("K5:K18").conditionalFormats.add("containsText", { text: "BUY", format: { fill: paleGreen, font: { color: "#215E21", bold: true } } });
bom.getRange("K5:K18").conditionalFormats.add("containsText", { text: "PROVISIONAL", format: { fill: amber, font: { color: "#7F6000", bold: true } } });
bom.getRange("K5:K18").conditionalFormats.add("containsText", { text: "HOLD", format: { fill: gray, font: { color: "#666666", bold: true } } });
bom.getRange("A:A").format.columnWidth = 6;
bom.getRange("B:B").format.columnWidth = 15;
bom.getRange("C:C").format.columnWidth = 24;
bom.getRange("D:D").format.columnWidth = 42;
bom.getRange("E:E").format.columnWidth = 14;
bom.getRange("F:J").format.columnWidth = 13;
bom.getRange("K:K").format.columnWidth = 15;
bom.getRange("L:L").format.columnWidth = 42;
bom.getRange("M:M").format.columnWidth = 52;
bom.getRange("A5:M18").format.wrapText = true;
bom.getRange("A5:M18").format.verticalAlignment = "top";
bom.getRange("A5:M18").format.rowHeight = 46;
bom.freezePanes.freezeRows(4);
bom.freezePanes.freezeColumns(3);

// ---------------- CNC Stock ----------------
title(cnc, "A1:K1", "6061-T6 Material Estimate — Three CNC Parts × Four Motors");
cnc.getRange("A3:B7").values = [
  ["Editable assumption", "Value"],
  ["6061-T6 density, kg/m³", 2700],
  ["Retail/cut-piece price, INR/kg", 500],
  ["Saw/face allowance per blank, mm", 3],
  ["Procurement contingency", 0.15],
];
header(cnc.getRange("A3:B3"));
cnc.getRange("B4:B7").format.fill = inputBlue;
cnc.getRange("B4:B6").format.numberFormat = "#,##0.00";
cnc.getRange("B7").format.numberFormat = "0%";
cnc.getRange("A9:K9").values = [["Part", "Material", "Blank diameter mm", "Finished axial envelope mm", "Qty", "Cut length incl. saw mm", "Stock mass kg", "Base stock cost INR", "Cost with contingency INR", "Selected lot cost INR", "Planning note"]];
header(cnc.getRange("A9:K9"));
cnc.getRange("A10:K12").values = [
  ["Stator carrier","6061-T6 round bar",50,32,4,null,null,null,null,null,"Planning blank only; bearing seat and duct geometry must come from corrected CAD."],
  ["Rotor end cap","6061-T6 round bar",65,15,4,null,null,null,null,null,"65 mm stock gives machining allowance around the 62 mm motor envelope."],
  ["Prop hub / shaft adapter","6061-T6 round bar",30,25,4,null,null,null,null,null,"Keep only if the final shaft/end-cap design uses a separate hub."],
];
for (let r = 10; r <= 12; r++) {
  cnc.getRange(`F${r}`).formulas = [[`=E${r}*(D${r}+$B$6)`]];
  cnc.getRange(`G${r}`).formulas = [[`=PI()*(C${r}/2000)^2*(F${r}/1000)*$B$4`]];
  cnc.getRange(`H${r}`).formulas = [[`=ROUND(G${r}*$B$5,0)`]];
  cnc.getRange(`I${r}`).formulas = [[`=ROUND(H${r}*(1+$B$7),0)`]];
  cnc.getRange(`J${r}`).formulas = [[`=I${r}`]];
}
cnc.getRange("A14:F14").merge();
cnc.getRange("A14:F14").values = [["Totals"]];
cnc.getRange("G14").formulas = [["=SUM(G10:G12)"]];
cnc.getRange("H14").formulas = [["=ROUND(SUM(H10:H12),0)"]];
cnc.getRange("I14").formulas = [["=ROUND(SUM(I10:I12),0)"]];
cnc.getRange("J14").formulas = [["=ROUND(SUM(J10:J12),0)"]];
cnc.getRange("A14:J14").format = { fill: navy, font: { bold: true, color: white } };
cnc.getRange("G10:G14").format.numberFormat = "0.000";
cnc.getRange("H10:J14").format.numberFormat = "₹#,##0";
cnc.getRange("C10:E12").format.fill = inputBlue;
cnc.getRange("A9:K14").format.borders = { preset: "inside", style: "thin", color: "#D9E1E8" };
cnc.getRange("A16:K18").merge(true);
cnc.getRange("A16:K18").values = [
  ["Suggested cut-stock order: Ø50 × 140 mm, Ø65 × 72 mm and Ø30 × 112 mm, all certified 6061-T6/T6511. These are estimates, not release dimensions."],
  ["Price basis is ₹500/kg, selected as a conservative small-cut retail assumption from current Indian 6061-T6 listings. Replace B5 with the vendor quote."],
  ["CNC labour, setup, tooling, inspection, anodizing and shipping are excluded."],
];
cnc.getRange("A16:K18").format = { fill: amber, wrapText: true };
cnc.getRange("A:A").format.columnWidth = 25;
cnc.getRange("B:B").format.columnWidth = 24;
cnc.getRange("C:J").format.columnWidth = 15;
cnc.getRange("K:K").format.columnWidth = 52;
cnc.getRange("A10:K12").format.wrapText = true;
cnc.getRange("A10:K12").format.rowHeight = 48;
cnc.freezePanes.freezeRows(9);

// ---------------- Wire Estimate ----------------
title(wire, "A1:J1", "Magnet-Wire Stock Estimate — Editable Screening Model");
wire.getRange("A3:B12").values = [
  ["Editable winding assumption", "Value"],
  ["Motors", 4],
  ["Coils/teeth per motor", 12],
  ["Turns per coil", 4],
  ["Mean turn length, m", 0.075],
  ["Required copper area per turn bundle, mm²", 6],
  ["Manufacturing/waste allowance", 0.15],
  ["24 SWG conductor diameter, mm", 0.56],
  ["24 SWG roll length, m", 50],
  ["24 SWG roll price, INR", 596],
];
header(wire.getRange("A3:B3"));
wire.getRange("B4:B12").format.fill = inputBlue;
wire.getRange("B9").format.numberFormat = "0%";
wire.getRange("A14:J14").values = [["Option", "Diameter mm", "Area/strand mm²", "Parallel strands", "Path length/motor m", "Total wire incl. waste m", "Rolls required", "Estimated cost INR", "Enamel evidence", "Disposition"]];
header(wire.getRange("A14:J14"));
wire.getRange("A15:J16").values = [
  ["24 SWG ART IFACT",0.56,null,null,null,null,null,null,"Modified polyurethane; thermal class not published","Preferred geometry, HOLD for Class-H evidence"],
  ["32 SWG ART IFACT",0.27,null,null,null,null,null,null,"Modified polyurethane; thermal class not published","Not preferred: excessive strand count and enamel fraction"],
];
wire.getRange("C15").formulas = [["=PI()*(B15^2)/4"]];
wire.getRange("C16").formulas = [["=PI()*(B16^2)/4"]];
wire.getRange("D15").formulas = [["=CEILING($B$8/C15,1)"]];
wire.getRange("D16").formulas = [["=CEILING($B$8/C16,1)"]];
wire.getRange("E15").formulas = [["=$B$5*$B$6*$B$7*D15"]];
wire.getRange("E16").formulas = [["=$B$5*$B$6*$B$7*D16"]];
wire.getRange("F15").formulas = [["=E15*$B$4*(1+$B$9)"]];
wire.getRange("F16").formulas = [["=E16*$B$4*(1+$B$9)"]];
wire.getRange("G15").formulas = [["=CEILING(F15/$B$11,1)"]];
wire.getRange("G16").formulas = [["=CEILING(F16/100,1)"]];
wire.getRange("H15").formulas = [["=G15*$B$12"]];
wire.getRange("H16").formulas = [["=G16*524"]];
wire.getRange("B15:H16").format.numberFormat = "0.00";
wire.getRange("D15:D16").format.numberFormat = "0";
wire.getRange("G15:G16").format.numberFormat = "0";
wire.getRange("H15:H16").format.numberFormat = "₹#,##0";
wire.getRange("A18:H18").values = [["Selected 24 SWG result", null, null, null, "Total metres", null, "Rolls", "Cost"]];
wire.getRange("F18").formulas = [["=F15"]];
wire.getRange("G18").formulas = [["=G15"]];
wire.getRange("H18").formulas = [["=H15"]];
wire.getRange("A18:J18").format = { fill: navy, font: { bold: true, color: white } };
wire.getRange("F18").format.numberFormat = "0.0";
wire.getRange("G18").format.numberFormat = "0";
wire.getRange("H18").format.numberFormat = "₹#,##0";
// Mirror selected outputs in row 12 columns G/H for simple cross-sheet references.
wire.getRange("G12").formulas = [["=G15"]];
wire.getRange("H12").formulas = [["=H15"]];
wire.getRange("G11:H11").values = [["Selected rolls", "Selected cost"]];
wire.getRange("G11:H11").format = { fill: paleBlue, font: { bold: true, color: navy } };
wire.getRange("G12").format.numberFormat = "0";
wire.getRange("H12").format.numberFormat = "₹#,##0";
wire.getRange("A14:J16").format.borders = { preset: "inside", style: "thin", color: "#D9E1E8" };
wire.getRange("A20:J23").merge(true);
wire.getRange("A20:J23").values = [
  ["This is a stock estimate, not a released winding. The assumed 4 turns, 75 mm mean turn length and 6 mm² copper bundle must be replaced by the corrected electromagnetic/slot-fill solution."],
  ["The calculation currently yields a large parallel bundle. This is expected for a 60 A burst target; it also makes insulation thickness and termination repeatability critical."],
  ["Do not buy all calculated rolls until one coil is wound, measured for mean turn length, weighed, checked for slot fit and resistance-tested."],
  ["Both ART IFACT listings use SWG-style diameters in their text. Do not interpret the gauge number as AWG."],
];
wire.getRange("A20:J23").format = { fill: amber, wrapText: true };
wire.getRange("A:A").format.columnWidth = 28;
wire.getRange("B:H").format.columnWidth = 16;
wire.getRange("I:I").format.columnWidth = 34;
wire.getRange("J:J").format.columnWidth = 44;
wire.getRange("A15:J16").format.wrapText = true;
wire.getRange("A15:J16").format.rowHeight = 48;
wire.freezePanes.freezeRows(14);

// ---------------- Infusion Budget ----------------
title(infusion, "A1:G1", "Vacuum-Infusion Materials and Coverage — Four Rotors");
infusion.getRange("A3:B11").values = [
  ["Editable geometry/process assumption", "Value"],
  ["Rotor outside diameter, m", 0.062],
  ["Reinforced axial band, m", 0.020],
  ["Rotor quantity", 4],
  ["UD-carbon hoop plies", 2],
  ["UD-glass isolation/protection plies", 2],
  ["Cutting/process waste", 0.30],
  ["Carbon fabric width, m", 0.50],
  ["Purchased linear length, m", 1.00],
];
header(infusion.getRange("A3:B3"));
infusion.getRange("B4:B11").format.fill = inputBlue;
infusion.getRange("B4:B5").format.numberFormat = "0.000";
infusion.getRange("B6:B8").format.numberFormat = "0";
infusion.getRange("B9").format.numberFormat = "0%";
infusion.getRange("B10:B11").format.numberFormat = "0.00";

infusion.getRange("D3:G3").values = [["Material", "Required area incl. waste m²", "Purchased area m²", "Coverage multiple"]];
header(infusion.getRange("D3:G3"));
infusion.getRange("D4:D5").values = [["400 gsm 12K UD carbon"], ["400 gsm UD glass"]];
infusion.getRange("E4").formulas = [["=PI()*$B$4*$B$5*$B$6*$B$7*(1+$B$9)"]];
infusion.getRange("E5").formulas = [["=PI()*$B$4*$B$5*$B$6*$B$8*(1+$B$9)"]];
infusion.getRange("F4").formulas = [["=$B$10*$B$11"]];
infusion.getRange("F5").formulas = [["=1"]];
infusion.getRange("G4:G5").formulas = [["=F4/E4"], ["=F5/E5"]];
infusion.getRange("E4:F5").format.numberFormat = "0.000";
infusion.getRange("G4:G5").format.numberFormat = "0.0x";
infusion.getRange("D3:G5").format.borders = { preset: "inside", style: "thin", color: "#D9E1E8" };
infusion.getRange("G4:G5").format.fill = paleGreen;

infusion.getRange("A13:E13").values = [["Cash item", "Observed pack/length", "Visible line price INR", "Included in ₹4,271 subtotal?", "Cash-flow value INR"]];
header(infusion.getRange("A13:E13"));
infusion.getRange("A14:E18").values = [
  ["Carbon Fiber India materials cart", "User-confirmed cart subtotal", 4271, "YES — controlling subtotal", 4271],
  ["Vacuum pump", "Reusable tooling", 6800, "NO", 6800],
  ["400 gsm UD glass", "1,000 mm wide; listing appears per 1 m²", 330, "YES — shown inside cart", 0],
  ["LY556 + HY951", "Listing says per kg; exact resin/hardener split must be confirmed", 920, "YES — shown inside cart", 0],
  ["400 gsm 12K UD carbon", "500 mm wide × apparently 1 linear metre", 930, "YES — shown inside cart", 0],
];
infusion.getRange("E15").formulas = [["=C15"]];
infusion.getRange("A20:D20").merge();
infusion.getRange("A20:D20").values = [["Immediate cash outlay (confirmed materials subtotal + pump)"]];
infusion.getRange("E20").formulas = [["=SUM(E14:E18)"]];
infusion.getRange("A20:E20").format = { fill: navy, font: { bold: true, color: white } };
infusion.getRange("C14:C18").format.numberFormat = "₹#,##0";
infusion.getRange("E14:E20").format.numberFormat = "₹#,##0";
infusion.getRange("A13:E18").format.borders = { preset: "inside", style: "thin", color: "#D9E1E8" };
infusion.getRange("A14:E18").format.wrapText = true;
infusion.getRange("A14:E18").format.rowHeight = 42;

infusion.getRange("A22:G25").merge(true);
infusion.getRange("A22:A25").values = [
  ["Quantity result: one 500 mm × 1 m carbon cut supplies about 0.500 m²; the four-rotor, two-carbon-ply estimate uses about 0.041 m² including 30% waste, so nominal area coverage is about 12.3×."],
  ["One square metre of 400 gsm UD glass supplies about 24.7× the calculated two-ply requirement. One unit is ample for the four rotors and trials."],
  ["A 1 kg LY556/HY951 kit is far more resin than the laminate fibre mass requires, but line/cup waste and trial panels dominate at this scale. Confirm the actual package split and mix ratio before ordering."],
  ["Process and packaging gate: the vendor mentions RTM but publishes no trustworthy mixed-viscosity/pot-life table. Flow-test first. Two nominal 0.38 mm carbon plies plus two glass plies may add roughly 2.5–3.5 mm to diameter; confirm on a cured coupon before freezing pre-wrap OD."],
];
infusion.getRange("A22:G25").format = { fill: amber, wrapText: true, verticalAlignment: "top" };
infusion.getRange("A22:G25").format.rowHeight = 38;
infusion.getRange("A27:G27").merge();
infusion.getRange("A27:G27").values = [["Supplier product pages"]];
header(infusion.getRange("A27:G27"));
infusion.getRange("A28:G30").merge(true);
infusion.getRange("A28:A30").values = [
  ["UD carbon: https://carbonfiberindia.com/product/12k-carbon-fabric-unidirectional-400-gsm-width-500mm-mtr-2/"],
  ["UD glass: https://carbonfiberindia.com/product/glass-fabric-ud-400-gsm-width-1000-mm-sqm/"],
  ["Resin system: https://carbonfiberindia.com/product/araldite-ly556-aradur-hy951-kg/"],
];
infusion.getRange("A28:G30").format = { fill: inputBlue, wrapText: true };
infusion.getRange("A:A").format.columnWidth = 38;
infusion.getRange("B:B").format.columnWidth = 22;
infusion.getRange("C:C").format.columnWidth = 18;
infusion.getRange("D:D").format.columnWidth = 27;
infusion.getRange("E:G").format.columnWidth = 21;
infusion.freezePanes.freezeRows(3);

// ---------------- Sources & Gates ----------------
title(sources, "A1:F1", "Sources, Product Identity Checks and Acceptance Gates");
sources.getRange("A3:F3").values = [["Item", "URL", "Observed listing/specification", "Workbook treatment", "Confidence", "Required action"]];
header(sources.getRange("A3:F3"));
sources.getRange("A4:F16").values = [
  ["24 SWG wire","https://www.amazon.in/dp/B07LCRM82G","50 m; 24 gauge/SWG; 0.56 mm; 99.9% copper; modified-polyurethane enamel","Provisional selected wire","Medium","Obtain insulation thermal class, enamel build and measured spool mass/length."],
  ["32 SWG wire","https://www.amazon.in/dp/B07KSVHYN1","100 m; 32 gauge/SWG; 0.27 mm; 99.9% copper; modified-polyurethane enamel","Alternative only","Medium","Do not select unless slot-fill study benefits from very fine strands."],
  ["Linked magnet listing","https://www.amazon.in/dp/B0CVS4KBZ6","Advertised as ceramic/ferrite 20×10×2 mm","REJECT","High","Buy the previously selected N42/N52 NdFeB magnets instead."],
  ["Alternate NdFeB retail lead","https://www.magneticks.com/products/20-x-10-x-2mm-neodymium-block-magnets-pack-of-20-30-50-magneticks","20×10×2 mm NdFeB; seller claims N52; pack options 20/30/50","Possible verification source","Medium","Request Br/Hcj evidence and thickness magnetization confirmation."],
  ["Mild-steel liner","https://www.amazon.in/dp/B0F2NBWV58","Cold-rolled mild-steel retail sheet; user fixed 0.5 mm liner","BUY","Medium","Measure actual thickness and reject galvanized, warped or heavily scaled sheet."],
  ["Lapox Ultra","https://www.amazon.in/dp/B0B6C4BTZ2","180 g two-part modified epoxy; 100:80 mass ratio; approx. 30,000–35,000 cP","BUY","High","Run magnet/steel and aluminium/composite coupons plus thermal cycles."],
  ["608ZZ bearing","https://www.amazon.in/dp/B01LXLWLFK","8×22×7 mm hobby/robotics bearing listing","PROVISIONAL","High identity / low suitability","Require speed, grease, clearance and runout evidence or replace."],
  ["Nomex listing","https://www.amazon.in/dp/B09WKV41J1","Retail insulation sheet listing","BUY with verification","Medium","Confirm genuine aramid paper, 7 mil thickness and temperature class."],
  ["VASK Electrofill","https://www.amazon.in/dp/B0H3W1MSPK","Electronic encapsulation/potting product; exact winding-varnish suitability not established","Not selected","Low","Do not substitute for V2603 without full TDS and cure compatibility."],
  ["6061-T6 price basis","https://www.tradeindia.com/products/aluminium-6061-t6-round-bar-c13037141.html","Current Indian listing around ₹500/kg","Planning input","Medium","Replace with cut-piece delivered quotation."],
  ["6061-T6 lower benchmark","https://www.sachinsteelcentre.in/stainless-steel-round-bars.html","Current listing around ₹349/kg","Reference only","Medium","Small cut pieces may cost more than bulk rate."],
  ["Lapox technical data","https://www.atul.co.in/wp-content/uploads/2017/05/PO_Lapox-Ultra-1.pdf","Manufacturer product family and packaging data","Supporting source","High","Follow current batch TDS/SDS."],
  ["Stator electrical steel","Excluded","CRNO/CRNGO sheets and lamination manufacture intentionally omitted","Excluded","High","Add in a later revision after sourcing is frozen."],
];
sources.getRange("A3:F16").format.borders = { preset: "inside", style: "thin", color: "#D9E1E8" };
sources.getRange("A4:F16").format.wrapText = true;
sources.getRange("A4:F16").format.verticalAlignment = "top";
sources.getRange("A4:F16").format.rowHeight = 52;
sources.getRange("D4:D16").conditionalFormats.add("containsText", { text: "REJECT", format: { fill: red, font: { color: "#7F0000", bold: true } } });
sources.getRange("D4:D16").conditionalFormats.add("containsText", { text: "BUY", format: { fill: paleGreen, font: { color: "#215E21", bold: true } } });
sources.getRange("D4:D16").conditionalFormats.add("containsText", { text: "PROVISIONAL", format: { fill: amber, font: { color: "#7F6000", bold: true } } });
sources.getRange("A:A").format.columnWidth = 24;
sources.getRange("B:B").format.columnWidth = 56;
sources.getRange("C:C").format.columnWidth = 48;
sources.getRange("D:D").format.columnWidth = 24;
sources.getRange("E:E").format.columnWidth = 16;
sources.getRange("F:F").format.columnWidth = 52;
sources.freezePanes.freezeRows(3);

// Comments on key assumptions.
wb.comments.addThread({ cell: cnc.getRange("B5") }, "Planning price only. Replace with the delivered small-cut 6061-T6 quote.");
wb.comments.addThread({ cell: wire.getRange("B8") }, "Required copper area is provisional until Kv, resistance, slot fill and current-density calculations are rerun on corrected CAD.");
wb.comments.addThread({ cell: bom.getRange("I5") }, "User-provided price per NdFeB magnet. The Amazon ceramic/ferrite link is not used for this row.");

await fs.mkdir(outDir, { recursive: true });
await fs.mkdir(previewDir, { recursive: true });

for (const sh of [summary, bom, cnc, wire, infusion, sources]) {
  const image = await wb.render({ sheetName: sh.name, autoCrop: "all", scale: 1, format: "png" });
  await fs.writeFile(`${previewDir}/${sh.name.replaceAll(" ", "_")}.png`, new Uint8Array(await image.arrayBuffer()));
}

const inspect = await wb.inspect({ kind: "table", range: "Summary!A1:H20", include: "values,formulas", tableMaxRows: 22, tableMaxCols: 10 });
console.log(inspect.ndjson);
const errors = await wb.inspect({ kind: "match", searchTerm: "#REF!|#DIV/0!|#VALUE!|#NAME\\?|#N/A", options: { useRegex: true, maxResults: 100 }, summary: "final formula error scan" });
console.log(errors.ndjson);

const xlsx = await SpreadsheetFile.exportXlsx(wb);
await xlsx.save(outFile);
console.log(`SAVED ${outFile}`);
