from pathlib import Path
import shutil,tarfile
r=Path('/work');dst=r/'rf_evidence_replay'
assert not dst.exists()
dst.mkdir()
with tarfile.open(r/'rf_port_probes.tar.gz') as t:t.extractall(dst,filter='data')
for s in ['rf_both_incidents.py','rf_915_roi_50ohm_post.py','rf_roi_50ohm_post.py']:shutil.copyfile(r/s,dst/s)
print('probe replay workspace prepared')
