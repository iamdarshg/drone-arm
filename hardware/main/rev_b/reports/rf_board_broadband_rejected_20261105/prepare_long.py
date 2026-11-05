from pathlib import Path
import json,shutil
r=Path('/work');src=r/'rf_gerber_roi_broadband_20261105';dst=r/'rf_gerber_roi_broadband_long_20261105'
assert not dst.exists()
(dst/'ems').mkdir(parents=True)
shutil.copytree(src/'fab',dst/'fab')
shutil.copytree(src/'ems/geometry',dst/'ems/geometry')
for x in ['loss_patch.json','manifest.json']:shutil.copyfile(src/x,dst/x)
d=json.loads((src/'simulation.json').read_text());d['max_steps']=300000
(dst/'simulation.json').write_text(json.dumps(d,indent=2))
print('prepared independent long run; max_steps=300000')
