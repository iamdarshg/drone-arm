
import json
import re
import sys

text = open(sys.argv[1], encoding="utf-8").read()

nets_start = text.index("(nets")
starts = [m.start() for m in re.finditer(r'\(net \(code "\d+"\)', text)]
ends = [m2.start() for m2 in re.finditer(r'\n    \(net \(code', text)] + [len(text)]

net_of = {}
for i, st in enumerate(starts):
    en = min([e for e in ends if e > st] + [len(text)])
    chunk = text[st:en]
    nm = re.match(r'\(net \(code "\d+"\) \(name "([^"]*)"', chunk)
    if not nm:
        continue
    netname = nm.group(1)
    if netname.startswith("unconnected"):
        continue
    for m in re.finditer(r'\(node \(ref "U10"\) \(pin "(\d+)"\) \(pinfunction "(GPIO\d+)"', chunk):
        pin, fn = m.group(1), m.group(2)
        net_of[int(fn[4:])] = {"pad": pin, "pinfunction": fn, "net": netname}

pinmap = {
    "board": "main_rev_b",
    "mcu": "RP2354B (U10, QFN-80)",
    "source_netlist": "hardware/main/rev_b/reports/main_net.xml",
    "generated": sys.argv[2] if len(sys.argv) > 2 else "",
    "pins": {str(g): v for g, v in sorted(net_of.items())},
}
with open("hardware/main/rev_b/mcu_pinmap.json", "w") as f:
    json.dump(pinmap, f, indent=2)
print("wrote mcu_pinmap.json with", len(net_of), "GPIO nets")

