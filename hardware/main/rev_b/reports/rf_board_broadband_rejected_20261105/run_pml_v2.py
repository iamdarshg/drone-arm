import sys
from gerber2ems.simulation import Simulation
from gerber2ems.main import main
original=Simulation.run
def run_with_pml(self,excited_port_number):
 self.set_boundary_conditions(pml=True)
 print('Explicit PML_8 applied to solver immediately before run',flush=True)
 return original(self,excited_port_number)
Simulation.run=run_with_pml
sys.argv=['gerber2ems','-s']
sys.exit(main())
