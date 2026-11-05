from pathlib import Path
import numpy as np,json
from openEMS.ports import UI_data
root=Path('/work/rf_gerber_roi_broadband_pml_v2_20261105');p=root/'ems/simulation/0';freq=np.unique(np.concatenate((np.linspace(100e6,10e9,1001),915e6*np.arange(1,11))));u=[];i=[]
for n in [0,1]:
 u.append(UI_data([f'port_ut_{n}B'],str(p),freq,'pulse').ui_f_val[0]);z=UI_data([f'port_it_{n}A',f'port_it_{n}B'],str(p),freq,'pulse').ui_f_val;i.append(.5*(z[0]+z[1]))
a0=.5*(u[0]+50*i[0]);a1=.5*(u[1]+50*i[1]);b0=.5*(u[0]-50*i[0]);b1=.5*(u[1]-50*i[1])
s11=b0/a0;s21=b1/a0;ratio=(abs(b0)**2+abs(b1)**2)/(abs(a0)**2+abs(a1)**2)
assert np.all(np.isfinite(ratio)) and np.all(np.isfinite(s11)) and np.all(np.isfinite(s21))
incident_db=20*np.log10(np.maximum(abs(a0)/np.max(abs(a0)),1e-300))
np.savetxt(root/'sparams_50ohm_and_incident.csv',np.column_stack((freq,s11.real,s11.imag,s21.real,s21.imag,abs(a1/a0)**2,ratio,incident_db)),delimiter=',',header='frequency_hz,s11_real,s11_imag,s21_real,s21_imag,port2_incident_over_port1,both_incident_power_ratio,port1_incident_db_relative_to_max',comments='')
j=int(np.argmax(ratio));j915=int(np.argmin(abs(freq-915e6)))
out={'minimum_relative_incident_db':float(np.min(incident_db)),'freq_points':len(freq),'reference_ohm':50,'max_power_ratio':float(ratio[j]),'max_ratio_frequency_hz':float(freq[j]),'power_ratio_fail_count_above_1_05':int(np.sum(ratio>1.05)),'nearest_915_frequency_hz':float(freq[j915]),'nearest_915_s11_db':float(20*np.log10(abs(s11[j915]))),'nearest_915_s21_db':float(20*np.log10(abs(s21[j915]))),'nearest_915_port2_incident_ratio':float(abs(a1[j915]/a0[j915])**2),'status':'Port and boundary diagnostic only; inspect energy termination and convergence separately; excludes PA, antenna and matching network'}
(root/'pml_50ohm_diagnostic.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
if out['power_ratio_fail_count_above_1_05']:raise RuntimeError('Failed explicit 50-ohm both-port power diagnostic')
