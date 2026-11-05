from pathlib import Path
import json,shutil,hashlib
r=Path('/work');src=r/'rf_gerber_roi_expanded_915_pml_20261105';dst=r/'rf_solid_via_smoke'
assert not dst.exists()
(dst/'ems/geometry').mkdir(parents=True);shutil.copytree(src/'fab',dst/'fab')
shutil.copyfile(r/'rf_solid_via_geometry.xml',dst/'ems/geometry/geometry.xml')
d=json.loads((src/'simulation.json').read_text());d['max_steps']=200
(dst/'simulation.json').write_text(json.dumps(d,indent=2))
print('Prepared 200-step geometry voxelization diagnostic; no meaningful S-parameters')
