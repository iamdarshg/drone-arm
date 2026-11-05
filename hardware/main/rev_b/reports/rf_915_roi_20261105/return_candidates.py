import pcbnew as p,json,hashlib,math
from pathlib import Path
source=Path('/work/main_rev_b_source.kicad_pcb')
sha=hashlib.sha256(source.read_bytes()).hexdigest()
assert sha=='b500f5fecd2299cdca0af12a05424e820ed845e1bb5ab35568e35cc38e826d43'
b=p.LoadBoard(str(source))
def pos(item):
 q=item.GetPosition();return [q.x/1e6,q.y/1e6]
def dist(x,y):
 px=max(123.585,min(134.95,x));return math.hypot(x-px,y-62.5)
items=[]
for fp in b.GetFootprints():
 pads=[{'pad':a.GetNumber(),'net':a.GetNetname(),'position_mm':pos(a)} for a in fp.Pads()]
 nets={x['net'] for x in pads};x,y=pos(fp);d=dist(x,y)
 if d<=30 and '/3V3' in nets and '/GND' in nets:
  items.append({'reference':fp.GetReference(),'value':fp.GetValue(),'position_mm':[x,y],'distance_to_feed_mm':d,'pads':pads})
vias=[]
for v in b.GetTracks():
 if isinstance(v,p.PCB_VIA):
  x,y=pos(v);d=dist(x,y)
  if d<=30 and v.GetNetname() in ['/3V3','/GND']:vias.append({'net':v.GetNetname(),'position_mm':[x,y],'distance_to_feed_mm':d})
items.sort(key=lambda x:x['distance_to_feed_mm']);vias.sort(key=lambda x:x['distance_to_feed_mm'])
out={'board_sha256':sha,'search_radius_mm':30,'candidate_footprints':items,'candidate_vias':vias,'scope':'geometric footprint and via inventory, not evidence of assembled components, actual connection paths, impedance, current or return continuity'}
Path('/work/rf_return_candidates.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
