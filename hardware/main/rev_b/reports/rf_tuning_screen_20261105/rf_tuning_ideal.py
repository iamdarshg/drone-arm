import math,cmath,csv,json,hashlib
from pathlib import Path
board=Path('/work/main_rev_b_source.kicad_pcb')
expected='b500f5fecd2299cdca0af12a05424e820ed845e1bb5ab35568e35cc38e826d43'
assert hashlib.sha256(board.read_bytes()).hexdigest()==expected
out=Path('/work/rf_tuning_ideal');out.mkdir(exist_ok=False)
def mul(a,b):
 return (a[0]*b[0]+a[1]*b[2],a[0]*b[1]+a[1]*b[3],a[2]*b[0]+a[3]*b[2],a[2]*b[1]+a[3]*b[3])
def response(f,c):
 w=2*math.pi*f
 # Ideal, restricted two-port from MATCH_A to ANT_915; omitted receive branch and CC1190 complex output.
 M=(1+0j,0j,0j,1+0j)
 for element in [(1+0j,0j,1j*w*3.3e-12,1+0j),(1+0j,1j*w*2.9e-9,0j,1+0j),(1+0j,0j,1j*w*7.5e-12,1+0j),(1+0j,1j*w*9.1e-9,0j,1+0j),(1+0j,0j,1j*w*c*1e-12,1+0j)]:
  M=mul(M,element)
 A,B,C,D=M;den=A+B/50+C*50+D;s11=(A+B/50-C*50-D)/den;s21=2/den
 assert abs(abs(s11)**2+abs(s21)**2-1)<1e-9
 return s11,s21
rows=[]
for mhz in range(850,951):
 candidates=[]
 for i in range(10,501):
  cap=i/100;s11,s21=response(mhz*1e6,cap);candidates.append((abs(s11),cap,s21))
 mag,cap,s21=min(candidates,key=lambda x:x[0])
 rows.append({'frequency_mhz':mhz,'optimal_ideal_C96_pF':cap,'s11_db':20*math.log10(max(mag,1e-15)),'s21_db':20*math.log10(max(abs(s21),1e-15))})
with (out/'sweep.csv').open('w',newline='') as f:
 writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
result={'board_sha256':expected,'model':'ideal lumped MATCH_A to ANT_915 two-port, 50-ohm ports; C91=3.3pF, L48=2.9nH, C95=7.5pF, L49=9.1nH, virtual C96=0.10-5.00pF, C94 DNP','range_mhz':[850,950],'selected_915':rows[65],'physical_tuner_installed':False,'status':'exploratory circuit math only; no PCB hardware/firmware tuning implemented and no EM or measured validation','limitations':['C96 is DNP footprint, not adjustable capacitor','omits CC1190 complex source/load, receive branch, package/parasitics, diode bias, power handling, trace EM and antenna','arbitrary tuning range is not established']}
(out/'summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
