import pcbnew as p,json,hashlib,collections
from pathlib import Path
root=Path('/work');source=root/'main_rev_b_source.kicad_pcb'
sha=hashlib.sha256(source.read_bytes()).hexdigest()
assert sha=='b500f5fecd2299cdca0af12a05424e820ed845e1bb5ab35568e35cc38e826d43'
board=p.LoadBoard(str(source));via=[]
for v in board.GetTracks():
 if not isinstance(v,p.PCB_VIA):continue
 q=v.GetPosition();x=q.x/1e6;y=q.y/1e6
 if not (100.075<x<139.925 and 45.075<y<77.925):continue
 if v.TopLayer()!=p.F_Cu or v.BottomLayer()!=p.B_Cu:continue
 diameter=v.GetDrillValue()/1e6
 assert diameter>0
 via.append({'board_x_mm':x,'board_y_mm':y,'local_x_mm':x-100.075,'local_y_mm':77.925-y,'drill_mm':diameter,'net':v.GetNetname()})
via.sort(key=lambda a:(a['drill_mm'],a['local_x_mm'],a['local_y_mm']))
groups=sorted({v['drill_mm'] for v in via});assert via
lines=['M48','METRIC']
for i,d in enumerate(groups,1):lines.append(f'T{i}C{d:.6f}')
lines+=['%','G90','G05']
for i,d in enumerate(groups,1):
 lines.append(f'T{i}')
 for a in via:
  if a['drill_mm']==d:lines.append('X%.6fY%.6f'%(a['local_x_mm'],a['local_y_mm']))
lines.append('M30')
dst=root/'rf_gerber_roi_expanded_20261105/fab/roi-PTH.drl'
old=dst.with_name('original-mixed-drill.txt');assert not old.exists();old.write_bytes(dst.read_bytes())
dst.write_text('\n'.join(lines)+'\n')
out={'board_sha256':sha,'selection':'actual through-vias within expanded rectangle; component through-holes, blind/buried vias and plating/microvia details excluded','transform':'local x=board x-100.075, local y=77.925-board y (mm); derived from prior port coordinate convention, independently verify against Gerber pixels','count':len(via),'net_counts':dict(collections.Counter(v['net'] for v in via)),'vias':via,'excellon_sha256':hashlib.sha256(dst.read_bytes()).hexdigest()}
(root/'rf_gerber_roi_expanded_20261105/native_via_manifest.json').write_text(json.dumps(out,indent=2));print(json.dumps({k:v for k,v in out.items() if k!='vias'},indent=2))
