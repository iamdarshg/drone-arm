import numpy as np, json, hashlib
from pathlib import Path
root=Path('/work/rf_microstrip_broadband_0p1_10_run')
x=np.genfromtxt(root/'sparameters.csv',delimiter=',',names=True)
f=x['frequency_hz'];a=x['s11_abs'];b=x['s21_abs']
assert len(f)==992 and np.all(np.isfinite(a)) and np.all(np.isfinite(b))
rows=[]
for requested in [0.1e9,915e6]+[915e6*n for n in range(2,11)]+[10e9]:
 i=int(np.argmin(abs(f-requested)))
 rows.append(dict(requested_hz=requested,sampled_hz=float(f[i]),s11_db=float(x['s11_db'][i]),s21_db=float(x['s21_db'][i]),passive_power_sum=float(a[i]**2+b[i]**2)))
out={'points':len(f),'maximum_passive_power_sum':float(np.max(a*a+b*b)),'minimum_s11_db':float(np.min(x['s11_db'])),'maximum_s11_db':float(np.max(x['s11_db'])),'points_of_interest':rows,'csv_sha256':hashlib.sha256((root/'sparameters.csv').read_bytes()).hexdigest(),'interpretation':'linear idealized straight feed only; cannot predict transmitter-generated harmonics; no broadband mesh convergence or physical validation','caveats':'constant conductivity fitted at 915 MHz; no material dispersion, actual ground plane, components, nonlinear active device, antenna or enclosure'}
(root/'broadband_summary.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
