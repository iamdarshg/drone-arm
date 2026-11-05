from PIL import Image
from pathlib import Path
import json
r=Path('/work/rf_gerber_roi_expanded_20261105');m=json.loads((r/'native_via_manifest.json').read_text());ims=[Image.open(r/'ems/geometry'/n) for n in ['F_Cu.png','B_Cu.png']]
def copper(im,x,y):
 px=int(round(x/.025));py=im.height-1-int(round(y/.025))
 if not (0<=px<im.width and 0<=py<im.height):return False
 c=im.getpixel((px,py));return c[1]>c[2] and c[1]>c[0]
out=[]
for dx in [-.15,0,.15]:
 for dy in [-.15,0,.15]:
  v=[(copper(im,a['local_x_mm']+dx,a['local_y_mm']+dy)) for im in ims for a in m['vias']]
  out.append({'dx':dx,'dy':dy,'front_hits':sum(v[:len(m['vias'])]),'back_hits':sum(v[len(m['vias']):])})
print(json.dumps(out,indent=2));(r/'via_pixel_alignment.json').write_text(json.dumps(out,indent=2))
