import os,json,hashlib
import numpy as np
from CSXCAD import ContinuousStructure
from openEMS import openEMS
from openEMS.physical_constants import C0, EPS0
# Board source SHA256 b500f5fecd2299cdca0af12a05424e820ed845e1bb5ab35568e35cc38e826d43
# /915 MHz RF/ANT_915 straight F.Cu segment (123.585,62.5) to (134.95,62.5) mm.
# Idealised uniform microstrip; continuous PEC ground, no antenna, vias, bends, pads or components.
u=1e-3; w=.35; h=.2; L=11.365; extension=10.0; half=L/2+extension
f0=915e6; eps=4.2; tand=.02; fmax=12e9
out='/work/rf_microstrip_broadband_0p1_10_run'
os.makedirs(out,exist_ok=True)
fdtd=openEMS(NrTS=1000000,EndCriteria=1e-5)
fdtd.SetGaussExcite(5.05e9,4.95e9)
fdtd.SetBoundaryCond(['PML_8','PML_8','MUR','MUR','PEC','MUR'])
csx=ContinuousStructure();fdtd.SetCSX(csx);mesh=csx.GetGrid();mesh.SetDeltaUnit(u)
res=C0/(fmax*np.sqrt(eps))/u/35
anchors=[-half-10,-half,-half+3,-L/2,0,L/2,half-3,half,half+10]
for a,b in zip(anchors,anchors[1:]): mesh.AddLine('x',np.linspace(a,b,int(np.ceil((b-a)/.25))+1))
mesh.SmoothMeshLines('x',res,1.3)
mesh.AddLine('y',[-8,8]);mesh.AddLine('y',np.linspace(-w/2,w/2,13))
mesh.SmoothMeshLines('y',res,1.3)
mesh.AddLine('z',np.linspace(0,h,11));mesh.AddLine('z',15);mesh.SmoothMeshLines('z',res,1.3)
sub=csx.AddMaterial('FR4',epsilon=eps,kappa=2*np.pi*f0*EPS0*eps*tand)
sub.AddBox([-half-10,-8,0],[half+10,8,h])
metal=csx.AddMetal('PEC')
p1=fdtd.AddMSLPort(1,metal,[-half-10,-w/2,h],[0,w/2,0],'x','z',excite=-1,FeedShift=13,MeasPlaneShift=extension+10,priority=10)
p2=fdtd.AddMSLPort(2,metal,[half+10,-w/2,h],[0,w/2,0],'x','z',MeasPlaneShift=extension+10,priority=10)
csx.Write2XML(out+'/model.xml')
fdtd.Run(out,cleanup=False,numThreads=2)
freq=np.unique(np.concatenate((np.linspace(0.1e9,10e9,991),[915e6])))
for port in (p1,p2):port.CalcPort(out,freq,ref_impedance=50)
s11=p1.uf_ref/p1.uf_inc;s21=p2.uf_ref/p1.uf_inc
np.savetxt(out+'/sparameters.csv',np.column_stack((freq,np.abs(s11),20*np.log10(np.maximum(np.abs(s11),1e-15)),np.abs(s21),20*np.log10(np.maximum(np.abs(s21),1e-15)))),delimiter=',',header='frequency_hz,s11_abs,s11_db,s21_abs,s21_db',comments='')
i=np.argmin(abs(freq-f0))
power=float(abs(s11[i])**2+abs(s21[i])**2)
summary={'power_balance_at_915':power,'passivity_gate':power <= 1.05,'model':'idealised straight ANT_915 segment only','board_sha256':'b500f5fecd2299cdca0af12a05424e820ed845e1bb5ab35568e35cc38e826d43','trace_start_mm':[123.585,62.5],'trace_end_mm':[134.95,62.5],'width_mm':w,'length_mm':L,'substrate_height_mm':h,'epsilon_r':eps,'loss_tangent':tand,'conductor':'zero-thickness PEC; no copper loss','ground':'assumed uninterrupted PEC','frequency_hz':float(freq[i]),'s11_db':float(20*np.log10(abs(s11[i]))),'s21_db':float(20*np.log10(abs(s21[i]))),'limitations':'No actual board ground geometry, soldermask, component models, antenna, discontinuities, enclosure or radiation assessment. Not a fab acceptance.'}
with open(out+'/summary.json','w') as f:json.dump(summary,f,indent=2)
print(json.dumps(summary,indent=2))
if not np.all(np.isfinite(s11)) or not np.all(np.isfinite(s21)) or np.max(np.abs(s11)**2+np.abs(s21)**2)>1.05: raise RuntimeError('failed broadband finite/passivity gate')
