from pathlib import Path
import shutil
r=Path('/work');src=r/'rf_gerber_roi_broadband_pml_v2_20261105/ems/simulation/0';dst=r/'rf_pml_snapshot/ems/simulation/0';assert not dst.exists();dst.mkdir(parents=True)
for p in src.glob('port_*'):
 b=p.read_bytes()
 if not b.endswith(b'\n'):b=b[:b.rfind(b'\n')+1]
 (dst/p.name).write_bytes(b)
print('Copied running probe snapshot; provisional only')
