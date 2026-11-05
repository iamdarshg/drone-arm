from pathlib import Path
import numpy as np
from CSXCAD import ContinuousStructure
from openEMS import openEMS
out=Path('/work/rf_zero_endcriterion_probe');out.mkdir(exist_ok=False)
csx=ContinuousStructure();g=csx.GetGrid();g.SetDeltaUnit(1e-3)
for axis in ['x','y','z']:g.AddLine(axis,np.linspace(-10,10,21))
metal=csx.AddMetal('metal')
fdtd=openEMS(NrTS=1000,EndCriteria=0.0);fdtd.SetCSX(csx);fdtd.SetBoundaryCond(['MUR']*6);fdtd.SetGaussExcite(3e9,2e9)
fdtd.AddLumpedPort(1,50,[-1,0,0],[1,0,0],'x',excite=1)
fdtd.Run(str(out),cleanup=False,numThreads=1)
print('TOY RUN RETURNED: inspect solver timestep and energy log; this is not a board simulation.')
