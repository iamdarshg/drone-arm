import pcbnew as p,hashlib,json
board='/work/main_rev_b_source.kicad_pcb'
b=p.LoadBoard(board)
sha=hashlib.sha256(open(board,'rb').read()).hexdigest()
assert sha=='b500f5fecd2299cdca0af12a05424e820ed845e1bb5ab35568e35cc38e826d43'
samples=[]
for i in range(11):
 x=123.585+(134.95-123.585)*i/10;y=62.5;pt=p.VECTOR2I(round(x*1e6),round(y*1e6))
 layers={}
 for layer in [p.F_Cu,p.In1_Cu,p.In2_Cu,p.B_Cu]:
  layers[b.GetLayerName(layer)]=[z.GetNetname() for z in b.Zones() if z.GetLayerSet().Contains(layer) and z.HitTestFilledArea(layer,pt)]
 samples.append({'x_mm':x,'y_mm':y,'filled_zone_nets':layers})
vias=[]
for t in b.GetTracks():
 if isinstance(t,p.PCB_VIA):
  q=t.GetPosition();x=q.x/1e6;y=q.y/1e6
  if 120<=x<=140 and 56<=y<=70:vias.append({'x_mm':x,'y_mm':y,'net':t.GetNetname()})
result={'board_sha256':sha,'segment':'/915 MHz RF/ANT_915 F.Cu','sampling':'11 centreline points; filled zone point containment only','samples':samples,'vias_in_roi_120_140_x_56_70_mm':vias}
with open('/work/rf_ground_audit.json','w') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
