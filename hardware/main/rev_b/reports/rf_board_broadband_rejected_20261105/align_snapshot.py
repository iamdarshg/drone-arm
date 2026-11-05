from pathlib import Path
import numpy as np
src=Path('/work/rf_pml_snapshot/ems/simulation/0')
dst=Path('/work/rf_pml_snapshot_aligned/ems/simulation/0');dst.mkdir(parents=True,exist_ok=False)
files=list(src.glob('port_*')); ends=[float(np.loadtxt(x,comments='%')[-1,0]) for x in files];end=min(ends)
for p in files:
 lines=p.read_text().splitlines()
 out=[s for s in lines if s.startswith('%') or (s.strip() and float(s.split()[0])<=end)]
 (dst/p.name).write_text('\n'.join(out)+'\n')
print('Snapshot signals truncated to common time',end)
