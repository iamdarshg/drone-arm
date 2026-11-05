import json,hashlib
import numpy as np
from pathlib import Path
root=Path('/work')
files=[root/'rf_microstrip_broadband_0p1_10_run/sparameters.csv',root/'rf_microstrip_broadband_0p1_10_refined_run/sparameters.csv']
a,b=[np.genfromtxt(f,delimiter=',',names=True) for f in files]
assert len(a)==len(b)==992 and np.allclose(a['frequency_hz'],b['frequency_hz'],rtol=0,atol=0)
assert all(np.all(np.isfinite(z)) for z in [a['s11_db'],b['s11_db'],a['s21_db'],b['s21_db']])
d11=np.abs(a['s11_db']-b['s11_db']);d21=np.abs(a['s21_db']-b['s21_db']);i=int(np.argmin(abs(a['frequency_hz']-915e6)))
out={'source_csv_sha256':[hashlib.sha256(f.read_bytes()).hexdigest() for f in files],'frequencies':len(a),'worst_absolute_s11_difference_db':float(np.max(d11)),'worst_s11_difference_frequency_hz':float(a['frequency_hz'][np.argmax(d11)]),'worst_absolute_s21_difference_db':float(np.max(d21)),'s11_difference_915_db':float(d11[i]),'s21_difference_915_db':float(d21[i]),'limitations':'mesh sensitivity for ideal trace geometry only; no predefined convergence threshold, validation, real antenna or active RF'}
(root/'rf_broadband_mesh_comparison.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
