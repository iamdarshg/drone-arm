from pathlib import Path
import numpy as np,json
from openEMS.ports import UI_data
freq=np.linspace(600e6,1300e6,1001);results=[]
for name in ['rf_gerber_roi_v4_x2','rf_gerber_roi_v5_x2_refined']:
 p=Path('/work')/name/'ems/simulation/0';u=[];i=[]
 for n in [0,1]:
  u.append(UI_data([f'port_ut_{n}B'],str(p),freq,'pulse').ui_f_val[0])
  z=UI_data([f'port_it_{n}A',f'port_it_{n}B'],str(p),freq,'pulse').ui_f_val;i.append(.5*(z[0]+z[1]))
 inc=.5*(u[0]+50*i[0]);s11=.5*(u[0]-50*i[0])/inc;s21=.5*(u[1]-50*i[1])/inc
 j=int(np.argmin(abs(freq-915e6)));results.append({'case':name,'frequency_hz':float(freq[j]),'s11_db':float(20*np.log10(abs(s11[j]))),'s21_db':float(20*np.log10(abs(s21[j]))),'power_sum':float(abs(s11[j])**2+abs(s21[j])**2)})
out={'reference_ohm':50,'results':results,'method':'explicit real 50-ohm reference from original probe signals, identical time-domain fields','limitations':'clipped ROI, uncertain launches and return geometry; no matching parts, antenna or active device'}
Path('/work/rf_915_roi_50ohm_comparison.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
