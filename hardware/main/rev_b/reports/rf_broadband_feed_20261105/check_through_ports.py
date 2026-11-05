import numpy as np,json
from pathlib import Path
from openEMS.ports import UI_data
p=Path('/work/rf_microstrip_broadband_0p1_10_run');data=np.genfromtxt(p/'sparameters.csv',delimiter=',',names=True);freq=data['frequency_hz'];u=[];i=[]
for n in [1,2]:
 u.append(UI_data([f'port_ut_{n}B'],str(p),freq,'pulse').ui_f_val[0])
 vals=UI_data([f'port_it_{n}A',f'port_it_{n}B'],str(p),freq,'pulse').ui_f_val
 i.append(.5*(vals[0]+vals[1]))
a0=.5*(u[0]+50*i[0]);a1=.5*(u[1]+50*i[1]);b0=.5*(u[0]-50*i[0]);b1=.5*(u[1]-50*i[1])
s11=abs(b0/a0);s21=abs(b1/a0)
ratio=(abs(b0)**2+abs(b1)**2)/(abs(a0)**2+abs(a1)**2)
e1=float(np.max(abs(s11-data['s11_abs'])));e2=float(np.max(abs(s21-data['s21_abs'])))
assert e1<1e-10 and e2<1e-10
out={'samples':len(freq),'max_abs_s11_difference':e1,'max_abs_s21_difference':e2,'max_both_incident_power_ratio':float(np.max(ratio)),'count_over_1_05':int(np.sum(ratio>1.05)),'scope':'confirms identical numerical wave extraction for ideal matched through-line model; does not validate real-board port orientation, reference conductor or geometry'}
Path('/work/rf_ideal_through_postprocessing_check.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
