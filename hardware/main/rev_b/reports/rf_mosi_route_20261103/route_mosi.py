import pcbnew as p,hashlib
src='hardware/main/rev_b/reports/rf_mosi_route_20261103/source.kicad_pcb';out='hardware/main/rev_b/reports/rf_mosi_route_20261103/replayed.kicad_pcb'
assert hashlib.sha256(open(src,'rb').read()).hexdigest()=='e73000b8edbb97769f9436f48b41fd33304c518db2bf9bf047b1120717e1664f'
b=p.LoadBoard(src);mm=p.FromMM;V=lambda x,y:p.VECTOR2I(round(x*1e6),round(y*1e6));net=next(t.GetNetCode() for t in b.GetTracks() if t.GetNetname()=='/3V3')
rm=[]
for t in list(b.GetTracks()):
 if isinstance(t,p.PCB_TRACK) and not isinstance(t,p.PCB_VIA) and t.GetNetCode()==net and t.GetLayer()==p.F_Cu:
  a=(t.GetStart().x/1e6,t.GetStart().y/1e6);z=(t.GetEnd().x/1e6,t.GetEnd().y/1e6)
  if min(a[0],z[0])>=56.39 and max(a[0],z[0])<=56.41 and min(a[1],z[1])>=53.84 and max(a[1],z[1])<=54.96:rm.append(t)
for t in rm:b.Remove(t)
assert len(rm)==1,len(rm)
for a,z in [((56.4,54.95),(56.4,55.64)),((56.4,55.64),(58.4,55.64))]:
 t=p.PCB_TRACK(b);t.SetStart(V(*a));t.SetEnd(V(*z));t.SetLayer(p.F_Cu);t.SetWidth(mm(.2));t.SetNetCode(net);b.Add(t)
# Remove the three obsolete /3V3 fanout segments at the previous dangling spur.
remove_uuids={'a0b6c2a7-072e-48e2-a83e-26d284034e25','a4c59b5b-938d-4674-88ac-6d3f998de255','abf1c49e-aae1-4ff7-a47f-b0f46d21d408'}
found=set()
for t in list(b.GetTracks()):
 if t.m_Uuid.AsString() in remove_uuids:
  b.Remove(t);found.add(t.m_Uuid.AsString())
assert found==remove_uuids,(found,remove_uuids)
p.ZONE_FILLER(b).Fill(b.Zones());assert p.SaveBoard(out,b);print(hashlib.sha256(open(out,'rb').read()).hexdigest())
