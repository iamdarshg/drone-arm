from pathlib import Path
import pcbnew as p,hashlib,json,math
src=Path('/work/main_rev_b_source.kicad_pcb');sha=hashlib.sha256(src.read_bytes()).hexdigest()
assert sha=='b500f5fecd2299cdca0af12a05424e820ed845e1bb5ab35568e35cc38e826d43'
b=p.LoadBoard(str(src))
target='/915 MHz RF/ANT_915'
def coord(v):return (v.x/1e6,v.y/1e6)
tracks=[t for t in b.GetTracks() if not isinstance(t,p.PCB_VIA) and t.GetLayer()==p.F_Cu]
def separation(x,y,t):
 a=coord(t.GetStart());q=coord(t.GetEnd());dx=q[0]-a[0];dy=q[1]-a[1];n=dx*dx+dy*dy
 f=max(0,min(1,((x-a[0])*dx+(y-a[1])*dy)/n)) if n else 0
 return math.hypot(x-a[0]-f*dx,y-a[1]-f*dy)
samples=[]
for label,x0,sign in [('SP1',123.585,1),('SP2',134.95,-1)]:
 for j in range(11):
  x=x0+sign*j/10;y=62.5
  hits=[t for t in tracks if t.GetNetname()==target and separation(x,y,t)<=t.GetWidth()/2e6+1e-7]
  samples.append({'port':label,'x_mm':x,'y_mm':y,'matching_target_segments':len(hits)})
out={'board_sha256':sha,'samples':samples,'all_samples_in_target_copper':all(x['matching_target_segments']>0 for x in samples),'scope':'native F.Cu segment centerline inclusion only; no ground reference, nearby copper clearance, multilayer connectivity or calibration'}
Path('/work/rf_port_native_alignment.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
if not out['all_samples_in_target_copper']:raise SystemExit('port centerline leaves target track')
