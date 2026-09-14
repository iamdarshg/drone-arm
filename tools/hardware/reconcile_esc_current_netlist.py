#!/usr/bin/env python3
from __future__ import annotations
import argparse, hashlib, json, sys, xml.etree.ElementTree as ET
from pathlib import Path
import pcbnew

FOOTPRINT_ROOT = Path(sys.executable).resolve().parents[1] / 'share/kicad/footprints'
EXPECTED_SOURCE_SHA256 = 'AD26EC66883FD67D09BD64A5D32F40EA232C75AA35557BCF5C4E5ACFAE2BA2F5'

POSITIONS = {
 'J202': (103.0, 6.80, 0), 'U208': (103.0, 14.0, 0),
 'R204': (94.5, 14.0, 0), 'R205': (98.0, 14.0, 0),
 'R208': (107.0, 14.0, 0), 'R209': (110.5, 14.0, 0),
 'C234': (114.5, 14.0, 0), 'R210': (119.0, 14.0, 0),
 'R206': (118.0, 21.5, 90), 'R207': (120.0, 21.5, 90),
 'J203': (130.0, 6.80, 0), 'U209': (130.0, 14.0, 0),
 'R211': (122.5, 14.0, 0), 'R212': (126.0, 14.0, 0),
 'C235': (134.5, 14.0, 0), 'R213': (139.0, 14.0, 0),
 'SW201': (155.0, 18.0, 90),
 'R214': (138.0, 22.0, 0), 'R215': (141.5, 22.0, 0),
 'R216': (145.0, 22.0, 0), 'R217': (148.5, 22.0, 0),
 'R218': (152.0, 22.0, 0), 'R219': (155.5, 22.0, 0),
}
POSITIONS.update({
 'U1107': (84.5,84.5,0), 'R1180': (84.5,80.5,0), 'R1181': (84.5,77.0,0),
 'U1207': (172.0,104.5,90), 'R1280': (172.0,100.5,0), 'R1281': (172.0,97.0,0),
 'U1307': (84.5,167.0,0), 'R1380': (84.5,163.0,0), 'R1381': (84.5,159.5,0),
 'U1407': (172.0,187.0,90), 'R1480': (172.0,183.0,0), 'R1481': (172.0,179.5,0),
 'U1507': (84.5,249.5,0), 'R1580': (84.5,245.5,0), 'R1581': (84.5,242.0,0),
 'U1607': (172.0,269.5,90), 'R1680': (172.0,265.5,0), 'R1681': (172.0,262.0,0),
})
def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest().upper()
def contract(path):
 root=ET.parse(path).getroot(); comps={}
 for comp in root.findall('./components/comp'):
  ref=comp.attrib['ref']
  if not ref.startswith('#'): comps[ref]=(comp.findtext('footprint',''),comp.findtext('value',''))
 pins={}
 for net in root.findall('./nets/net'):
  for node in net.findall('node'): pins[(node.attrib['ref'],node.attrib['pin'])]=net.attrib['name']
 return comps,pins

def load_fp(identifier):
 lib,name=identifier.split(':',1); p=FOOTPRINT_ROOT/f'{lib}.pretty'; fp=pcbnew.FootprintLoad(str(p),name)
 if fp is None: raise RuntimeError(f'cannot load {identifier} from {p}')
 return fp

def get_net(board,name):
 net=board.FindNet(name)
 if net is None:
  net=pcbnew.NETINFO_ITEM(board,name); board.Add(net)
 return net

def trans(fp):
 p=fp.GetPosition(); return [p.x,p.y,fp.GetOrientation().AsTenthsOfADegree(),fp.GetLayer()]
def via_map(board):
 out={}
 for t in board.GetTracks():
  if t.Type()==pcbnew.PCB_VIA_T:
   p=t.GetPosition(); key=(p.x,p.y,t.GetWidth(pcbnew.F_Cu),t.GetDrillValue(),t.GetLayerSet().FmtHex())
   out.setdefault(key,[]).append(t.GetNetname())
 return {str(k):sorted(v) for k,v in sorted(out.items(),key=lambda kv:kv[0])}

def main():
 ap=argparse.ArgumentParser(); ap.add_argument('--source',type=Path,required=True); ap.add_argument('--netlist',type=Path,required=True); ap.add_argument('--output',type=Path,required=True); ap.add_argument('--report',type=Path,required=True); a=ap.parse_args()
 if sha(a.source)!=EXPECTED_SOURCE_SHA256: raise RuntimeError(f'source SHA changed: {sha(a.source)}')
 b=pcbnew.LoadBoard(str(a.source)); comps,pins=contract(a.netlist)
 existing={f.GetReference():f for f in b.GetFootprints()}; before_trans={r:trans(f) for r,f in existing.items()}; vias_before=via_map(b)
 missing=sorted(set(comps)-set(existing)); unexpected=sorted(set(missing)-set(POSITIONS))
 if unexpected: raise RuntimeError(f'missing placements for {unexpected}')
 pad_updates=[]
 for ref in sorted(set(comps)&set(existing)):
  for pad in existing[ref].Pads():
   num=pad.GetNumber()
   if not num: continue
   expected=pins.get((ref,num))
   if expected is None: continue
   actual=pad.GetNetname()
   if actual!=expected:
    pad.SetNet(get_net(b,expected)); pad_updates.append({'ref':ref,'pad':num,'from':actual,'to':expected})
 for ref in missing:
  ident,val=comps[ref]; fp=load_fp(ident); fp.SetReference(ref); fp.SetValue(val)
  x,y,deg=POSITIONS[ref]; fp.SetPosition(pcbnew.VECTOR2I_MM(x,y)); fp.SetOrientationDegrees(deg); fp.SetLayer(pcbnew.F_Cu)
  try: fp.Reference().SetVisible(False); fp.Value().SetVisible(False)
  except Exception: pass
  for graphic in list(fp.GraphicalItems()):
   if graphic.GetLayer() in (pcbnew.F_SilkS, pcbnew.B_SilkS): graphic.SetLayer(pcbnew.F_Fab)
  b.Add(fp)
  for pad in fp.Pads():
   expected=pins.get((ref,pad.GetNumber()))
   if expected is not None: pad.SetNet(get_net(b,expected))
 a.output.parent.mkdir(parents=True,exist_ok=True); pcbnew.SaveBoard(str(a.output),b)
 c=pcbnew.LoadBoard(str(a.output)); after={f.GetReference():f for f in c.GetFootprints()}; vias_after=via_map(c)
 transform_changes=[r for r,v in before_trans.items() if r not in after or trans(after[r])!=v]
 back=[r for r,f in after.items() if f.GetLayer()!=pcbnew.F_Cu]
 report={'source':str(a.source),'source_sha256':sha(a.source),'output':str(a.output),'output_sha256':sha(a.output),'existing_footprints':len(existing),'output_footprints':len(after),'missing_added':missing,'added_count':len(missing),'stale_retained':sorted(set(after)-set(comps)),'pad_updates':pad_updates,'pad_update_count':len(pad_updates),'existing_transform_changes':transform_changes,'back_side_footprints':back,'via_count_before':sum(len(v) for v in vias_before.values()),'via_count_after':sum(len(v) for v in vias_after.values()),'via_geometry_or_net_changed':vias_before!=vias_after,'zones_before':len(list(b.Zones())),'zones_after':len(list(c.Zones())),'tracks_before':sum(1 for t in b.GetTracks() if t.Type()==pcbnew.PCB_TRACE_T),'tracks_after':sum(1 for t in c.GetTracks() if t.Type()==pcbnew.PCB_TRACE_T),'positions':{r:POSITIONS[r] for r in missing}}
 a.report.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8'); print(json.dumps(report,indent=2))
 if transform_changes or back or vias_before!=vias_after or len(after)!=len(existing)+len(missing): return 2
 return 0
if __name__=='__main__': raise SystemExit(main())
