from pathlib import Path
import json,shutil,hashlib
r=Path('/work');src=r/'rf_solid_via_full';dst=r/'rf_solid_via_fixed_duration'
assert not dst.exists()
(dst/'ems/geometry').mkdir(parents=True)
shutil.copytree(src/'fab',dst/'fab')
shutil.copyfile(src/'ems/geometry/geometry.xml',dst/'ems/geometry/geometry.xml')
g=dst/'ems/geometry/geometry.xml';assert hashlib.sha256(g.read_bytes()).hexdigest()=='cfc5dc7d90380c8966bfc41a39083fe6b4ae925497cc411c3e2c256cec21bdcb'
d=json.loads((src/'simulation.json').read_text());d['max_steps']=400000
(dst/'simulation.json').write_text(json.dumps(d,indent=2))
print('Prepared 400000-step solid-PEC sensitivity run; manual energy/port review required')
