from pathlib import Path
import json,shutil,hashlib,math,xml.etree.ElementTree as E
r=Path('/work');src=r/'rf_gerber_roi_expanded_v2_20261105';dst=r/'rf_gerber_roi_expanded_915_pml_20261105'
assert not dst.exists()
(dst/'ems').mkdir(parents=True);shutil.copytree(src/'fab',dst/'fab');shutil.copytree(src/'ems/geometry',dst/'ems/geometry')
d=json.loads((src/'simulation.json').read_text());d['max_steps']=300000
(dst/'simulation.json').write_text(json.dumps(d,indent=2))
p=dst/'ems/geometry/geometry.xml';before=hashlib.sha256(p.read_bytes()).hexdigest()
tree=E.parse(p);k=2*math.pi*915e6*8.8541878128e-12*4.2*.02;count=0
for el in tree.getroot().iter('Material'):
 if el.attrib.get('Name','').startswith('Substrate_'):
  prop=el.find('Property');assert prop is not None
  assert all(float(x)==0 for x in prop.attrib['Kappa'].split(','))
  prop.set('Kappa',f'{k:.12g},{k:.12g},{k:.12g}');count+=1
assert count==3
tree.write(p,encoding='utf-8',xml_declaration=True)
out={'source_geometry_sha256':before,'patched_geometry_sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'constant_conductivity_s_per_m':k,'substrates':count,'warning':'FR4 loss tangent fit at 915 MHz, no dispersion; no antenna, blind vias, matching components or PA; PML via run_rf_pml_v2'}
(dst/'loss_patch.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
