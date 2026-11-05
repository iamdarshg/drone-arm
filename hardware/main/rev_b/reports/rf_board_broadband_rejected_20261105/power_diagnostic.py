import csv,json,math
from pathlib import Path
p=Path('/work/rf_gerber_roi_broadband_20261105/ems/simulation/Sx0.csv')
with p.open() as f:
 next(f);rows=[[float(c) for c in r[:5]] for r in csv.reader(f) if r]
regions=[(100,1000),(1000,3000),(3000,5000),(5000,7000),(7000,10000.1)]
out=[]
for lo,hi in regions:
 sample=[(r[0],r[1]**2+r[2]**2+r[3]**2+r[4]**2) for r in rows if lo<=r[0]<hi]
 out.append({'mhz':[lo,hi],'n':len(sample),'over_1_05':sum(v>1.05 for f,v in sample),'maximum':max(sample,key=lambda x:x[1])})
print(json.dumps(out,indent=2))
Path('/work/rf_gerber_roi_broadband_20261105/power_diagnostic.json').write_text(json.dumps(out,indent=2))
