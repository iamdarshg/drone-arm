import sys
from gerber2ems.simulation import Simulation
from gerber2ems.main import main
original=Simulation.set_boundary_conditions
def use_pml(self,pml=False):
 return original(self,pml=True)
Simulation.set_boundary_conditions=use_pml
sys.argv=['gerber2ems','-s']
sys.exit(main())
