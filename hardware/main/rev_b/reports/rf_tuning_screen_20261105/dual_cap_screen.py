import math, cmath, csv, json, hashlib
from pathlib import Path
board=Path('/work/main_rev_b_source.kicad_pcb')
sha='b500f5fecd2299cdca0af12a05424e820ed845e1bb5ab35568e35cc38e826d43'
assert hashlib.sha256(board.read_bytes()).hexdigest()==sha
out=Path('/work/rf_915_dual_cap_ideal');out.mkdir(exist_ok=False)
w=2*math.pi*915e6
def mul(a,b):
 return (a[0]*b[0]+a[1]*b[2],a[0]*b[1]+a[1]*b[3],a[2]*b[0]+a[3]*b[2],a[2]*b[1]+a[3]*b[3])
def run(c94,c96):
 z48=1/(1/(1j*w*2.9e-9)+1j*w*c94*1e-12)
 m=(1+0j,0j,0j,1+0j)
 for el in [(1+0j,0j,1j*w*3.3e-12,1+0j),(1+0j,z48,0j,1+0j),(1+0j,0j,1j*w*7.5e-12,1+0j),(1+0j,1j*w*9.1e-9,0j,1+0j),(1+0j,0j,1j*w*c96*1e-12,1+0j)]:m=mul(m,el)
 a,b,c,d=m;den=a+b/50+50*c+d;s11=(a+b/50-50*c-d)/den;s21=2/den
 assert abs(abs(s11)**2+abs(s21)**2-1)<1e-9
 return s11,s21
best=None
for x in range(101):
 for y in range(101):
  c94=x/20;c96=y/20;s11,s21=run(c94,c96)
  v=(abs(s11),c94,c96,abs(s21))
  if best is None or v<best:best=v
s0,t0=run(0,0)
data={'board_sha256':sha,'frequency_hz':915000000,'ideal_50ohm_source_and_load':True,'baseline_no_dnp_c94_c96':{'s11_db':20*math.log10(abs(s0)),'s21_db':20*math.log10(abs(t0))},'grid_best':{'virtual_c94_pf':best[1],'virtual_c96_pf':best[2],'s11_db':20*math.log10(best[0]),'s21_db':20*math.log10(best[3])},'grid_step_pf':0.05,'candidate_grid_range_pf':[0,5],'limitation':'Pure idealized MATCH_A to ANT_915 50-ohm network, no actual CC1190 complex source impedance, receive branch, component parasitics, nonlinear power, traces or antenna. Not recommended component values; no physical tuning implemented.'}
(out/'result.json').write_text(json.dumps(data,indent=2));print(json.dumps(data,indent=2))
