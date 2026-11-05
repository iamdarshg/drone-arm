import inspect,hashlib,json
from pathlib import Path
import gerber2ems.main as m
from gerber2ems.simulation import Simulation
from openEMS import openEMS
geometry=inspect.getsource(m.geometry);simulate=inspect.getsource(m.simulate);init=inspect.getsource(Simulation.__init__)
bc_doc=openEMS.SetBoundaryCond.__doc__
assert 'set_boundary_conditions(pml=False)' in geometry
assert 'set_boundary_conditions' not in simulate
assert 'sim = Simulation(GEOMETRY_FILE)' in simulate
assert 'self.fdtd = openEMS.openEMS' in init
assert 'PEC' in bc_doc and 'default' in bc_doc
p=Path(inspect.getfile(m));q=Path(inspect.getfile(Simulation))
out={'wrapper_main_sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'wrapper_simulation_sha256':hashlib.sha256(q.read_bytes()).hexdigest(),'geometry_sets_mur':True,'simulate_constructs_new_solver':True,'simulate_explicitly_sets_boundary':False,'openems_api_default':'PEC','interpretation':'-s on fresh solver uses documented default; geometry-stage MUR setting does not carry to new fdtd instance'}
Path('/work/rf_boundary_wrapper_audit.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
