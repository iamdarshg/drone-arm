from pathlib import Path
import numpy as np,json,csv,hashlib
from openEMS.ports import UI_data
root=Path('/work/rf_gerber_roi_broadband_20261105');p=root/'ems/simulation/0';freq=np.linspace(100e6,10e9,1001)
u=[];i=[]
for n in [0,1]:
 v=UI_data([f'port_ut_{n}B'],str(p),freq,'pulse').ui_f_val[0]
 z=UI_data([f'port_it_{n}A',f'port_it_{n}B'],str(p),freq,'pulse').ui_f_val
 u.append(v);i.append(.5*(z[0]+z[1]))
inc=.5*(u[0]+50*i[0]);s11=.5*(u[0]-50*i[0])/inc;s21=.5*(u[1]-50*i[1])/inc
assert np.all(np.isfinite(s11)) and np.all(np.isfinite(s21))
power=abs(s11)**2+abs(s21)**2
np.savetxt(root/'sparams_50ohm_repost.csv',np.column_stack((freq,s11.real,s11.imag,s21.real,s21.imag,power)),delimiter=',',header='frequency_hz,s11_real,s11_imag,s21_real,s21_imag,power_sum',comments='')
idx=int(np.argmax(power));idx915=int(np.argmin(abs(freq-915e6)))
info={'reference_ohm':50,'method':'recomputed from openEMS port voltage B and average of currents A/B using UI_data and explicit real 50-ohm traveling-wave definitions; same FDTD outputs','max_power_sum':float(power[idx]),'max_frequency_hz':float(freq[idx]),'point_near_915_hz':float(freq[idx915]),'point_near_915_power_sum':float(power[idx915]),'points_over_1_05':int(np.sum(power>1.05)),'source_model_warning':'Original FDTD hit maximum timesteps without satisfying energy criterion; re-normalization cannot cure that, nor omitted antenna/devices/return path.'}
(root/'postprocess_50ohm_summary.json').write_text(json.dumps(info,indent=2));print(json.dumps(info,indent=2))
