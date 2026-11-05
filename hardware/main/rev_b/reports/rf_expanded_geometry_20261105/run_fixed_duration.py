import sys
from gerber2ems.simulation import Simulation
from gerber2ems.main import main
original=Simulation.run
def run_fixed_duration(self,excited_port_number):
 self.set_boundary_conditions(pml=True)
 self.fdtd.SetEndCriteria(0.0)
 print('Explicit PML_8; automatic energy stop disabled; fixed timestep cap and post-run energy review required',flush=True)
 return original(self,excited_port_number)
Simulation.run=run_fixed_duration
sys.argv=['gerber2ems','-s']
sys.exit(main())
