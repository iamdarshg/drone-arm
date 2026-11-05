from pathlib import Path
import csv,math,json
root=Path(r'E:\1vm\TMP');out=[]
for name in ['rf_gerber_roi_v4_x2','rf_gerber_roi_v5_x2_refined']:
 p=root/name/'ems/simulation/Sx0.csv'
 if not p.exists():out.append({'case':name,'status':'missing'});continue
 with p.open() as f:
  next(f);rows=[[float(c.strip()) for c in row[:5]] for row in csv.reader(f) if row]
 f,a,b,c,d=min(rows,key=lambda x:abs(x[0]-915));s11=complex(a,c);s21=complex(b,d);power=abs(s11)**2+abs(s21)**2
 out.append({'case':name,'freq_mhz':f,'s11_db':20*math.log10(abs(s11)),'s21_db':20*math.log10(abs(s21)),'power_balance':power,'strict_passivity':power<=1.000001})
print(json.dumps(out,indent=2));(root/'rf_x2_mesh_comparison.json').write_text(json.dumps(out,indent=2))
