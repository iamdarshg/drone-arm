import pcbnew as p
from PIL import Image
import random,json,hashlib
from pathlib import Path
root=Path('/work');src=root/'main_rev_b_source.kicad_pcb';assert hashlib.sha256(src.read_bytes()).hexdigest()=='b500f5fecd2299cdca0af12a05424e820ed845e1bb5ab35568e35cc38e826d43'
b=p.LoadBoard(str(src));zones=[z for z in b.Zones() if z.GetNetname()=='/GND' and z.GetLayerSet().Contains(p.F_Cu)]
im=Image.open(root/'rf_gerber_roi_expanded_20261105/ems/geometry/F_Cu.png')
rng=random.Random(829);samples=[]
for _ in range(20000):
 x=rng.uniform(100.5,139.5);y=rng.uniform(45.5,77.5);pt=p.VECTOR2I(round(x*1e6),round(y*1e6))
 gold=any(z.HitTestFilledArea(p.F_Cu,pt) for z in zones)
 samples.append((x,y,gold))
def calc(dx,dy):
 tp=tn=pos=neg=0
 for x,y,gold in samples:
  lx=x-100.075+dx;ly=77.925-y+dy;px=int(round(lx/.025));py=im.height-1-int(round(ly/.025));color=im.getpixel((px,py));green=color[1]>230
  if gold:pos+=1;tp+=green
  else:neg+=1;tn+=not green
 return {'dx_mm':dx,'dy_mm':dy,'zone_positive':pos,'zone_negative':neg,'recall':tp/max(pos,1),'specificity':tn/max(neg,1),'balanced_score':.5*(tp/max(pos,1)+tn/max(neg,1))}
result=[calc(i*.025,j*.025) for i in range(0,7) for j in range(0,7)]
result.sort(key=lambda x:x['balanced_score'],reverse=True)
out={'image':str(im.size),'n_samples':len(samples),'method':'native F.Cu ground-zone point inclusion versus bitmap copper, random fixed-seed board points; other-net copper and raster discretization are confounders','top':result[:10],'unshifted':next(a for a in result if a['dx_mm']==0 and a['dy_mm']==0)}
(root/'rf_gerber_roi_expanded_20261105/zone_alignment_fine.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
