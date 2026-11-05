from pathlib import Path
import numpy as np,json
from openEMS.ports import UI_data
results=[]
for name in ['rf_gerber_roi_broadband_20261105','rf_gerber_roi_broadband_long_20261105']:
 root=Path('/work')/name;p=root/'ems/simulation/0';freq=np.linspace(100e6,10e9,1001);u=[];i=[]
 for n in [0,1]:
  u.append(UI_data([f'port_ut_{n}B'],str(p),freq,'pulse').ui_f_val[0]);z=UI_data([f'port_it_{n}A',f'port_it_{n}B'],str(p),freq,'pulse').ui_f_val;i.append(.5*(z[0]+z[1]))
 a0=.5*(u[0]+50*i[0]);a1=.5*(u[1]+50*i[1]);b0=.5*(u[0]-50*i[0]);b1=.5*(u[1]-50*i[1])
 inc=abs(a0)**2+abs(a1)**2;out=abs(b0)**2+abs(b1)**2
 ratio=out/inc;relative=abs(a1/a0)**2;j=int(np.argmax(ratio))
 results.append({'case':name,'max_out_over_total_incident':float(ratio[j]),'frequency_hz':float(freq[j]),'total_incident_fail_points_above_1_05':int(np.sum(ratio>1.05)),'max_port2_incident_relative':float(np.max(relative)),'interpretation':'uses both recorded port incident waves and explicit real 50-ohm references; fields remain non-converged'})
p=Path('/work/rf_broadband_both_incidents.json');p.write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))
