from pathlib import Path
import json, shutil, hashlib
root=Path('/work')
src=root/'rf_gerber_roi_v4_x2'
dst=root/'rf_gerber_roi_broadband_20261105'
assert not dst.exists()
(dst/'ems').mkdir(parents=True)
shutil.copytree(src/'fab',dst/'fab')
shutil.copytree(src/'ems/geometry',dst/'ems/geometry')
shutil.copyfile(src/'loss_patch.json',dst/'loss_patch.json')
data=json.loads((src/'simulation.json').read_text())
data['frequency']={'start':100000000,'stop':10000000000}
(dst/'simulation.json').write_text(json.dumps(data,indent=2))
man=json.loads((src/'manifest.json').read_text())
man['frequency_note']='Broadband passive linear ROI only. Includes neither RF IC nonlinearity nor matching components or antenna. Existing clipped and reference-plane limitations remain.'
man['geometry_sha256']=hashlib.sha256((dst/'ems/geometry/geometry.xml').read_bytes()).hexdigest()
(dst/'manifest.json').write_text(json.dumps(man,indent=2))
print(json.dumps(man,indent=2))
