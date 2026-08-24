
import re
import sys

text = open(sys.argv[1], encoding="utf-8").read()

# Extract pinfunction names for U10 from the nets section
nodes = re.findall(r'\(node \(ref "U10"\) \(pin "(\d+)"\) \(pinfunction "([^"]*)"\)', text)
pinfunc = {num: fn for num, fn in nodes}

# Also parse net membership per U10 pin
net_of = {}
for nm in re.finditer(r'\(net \(code "\d+"\) \(name "([^"]*)"[^\)]*\)(.*?)\n    \)', text, re.S):
    netname = nm.group(1)
    for m in re.finditer(r'\(node \(ref "U10"\) \(pin "(\d+)"\)[^)]*\)', nm.group(2)):
        net_of[m.group(1)] = netname

rows = []
for num, fn in pinfunc.items():
    gm = re.match(r"GPIO(\d+)", fn)
    if not gm:
        continue
    gpio = int(gm.group(1))
    net = net_of.get(num, "?")
    if net.startswith("unconnected") or net == "?":
        continue
    rows.append((gpio, fn, net, num))

rows.sort()
print("U10 GPIO pins with connections:")
for gpio, fn, net, num in rows:
    print(f"  pad {num:>3}  {fn:<24}  GPIO{gpio:<3} -> {net}")

