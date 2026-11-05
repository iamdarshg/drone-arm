import csv,math,json,hashlib
from pathlib import Path
root=Path('/work/rf_gerber_roi_broadband_20261105')
p=root/'ems/simulation/Sx0.csv'
with p.open() as f:
 header=next(f).strip()
 rows=[[float(c.strip()) for c in row[:5]] for row in csv.reader(f) if row]
assert rows and all(len(r)==5 and all(math.isfinite(x) for x in r) for r in rows)
def summary(r):
 freq,a,b,c,d=r
 return {'frequency_mhz':freq,'s11_db':20*math.log10(max(abs(complex(a,c)),1e-15)),'s21_db':20*math.log10(max(abs(complex(b,d)),1e-15)),'power_sum':abs(complex(a,c))**2+abs(complex(b,d))**2}
worst=max(rows,key=lambda r:summary(r)['power_sum'])
selected=[summary(min(rows,key=lambda r:abs(r[0]-f))) for f in [100,915]+[915*i for i in range(2,11)]+[10000]]
out={'header':header,'points':len(rows),'min_frequency_mhz':min(r[0] for r in rows),'max_frequency_mhz':max(r[0] for r in rows),'worst_power_sum':summary(worst),'selected':selected,'csv_sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'status':'Exploratory clipped board ROI only; full-band mesh sensitivity and port validity unverified; excludes nonlinear RF components and antenna. No harmonic emission or match conclusion.'}
(root/'broadband_sanity.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
if out['worst_power_sum']['power_sum']>1.05:raise RuntimeError('Reject broadband ROI: passive energy balance')
