from pathlib import Path
import xml.etree.ElementTree as E,json,hashlib
p=Path('/work/rf_gerber_roi_expanded_v2_20261105/ems/geometry/geometry.xml')
r=E.parse(p).getroot();props={}
for el in r.iter():
 n=el.attrib.get('Name')
 if n:
  prim=el.find('Primitives')
  if prim is not None:props[n]=len(prim)
grid={n:int(r.find('.//'+n).attrib['Qty']) for n in ['XLines','YLines','ZLines']}
out={'geometry_sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'grid_lines':grid,'approximate_cells':(grid['XLines']-1)*(grid['YLines']-1)*(grid['ZLines']-1),'primitive_counts':props,'limitations':'geometry import check only, does not establish actual port positions or return continuity; loss tangent still needs patch; no EM solve'}
Path('/work/rf_gerber_roi_expanded_v2_20261105/geometry_audit.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
