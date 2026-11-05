from pathlib import Path
import json,shutil,hashlib
root=Path('/work');src=root/'rf_gerber_roi_v4_x2';dst=root/'rf_gerber_roi_expanded_20261105'
assert not dst.exists()
shutil.copytree(src/'fab',dst/'fab')
edge='%FSLAX46Y46*%\n%MOMM*%\n%LPD*%\n%ADD10C,0.150000*%\nD10*\nX100000000Y-78000000D02*\nX140000000Y-78000000D01*\nX140000000Y-45000000D01*\nX100000000Y-45000000D01*\nX100000000Y-78000000D01*\nM02*\n'
(dst/'fab/roi-Edge_Cuts.gbr').write_text(edge)
ports='Ref,Val,Package,PosX,PosY,Rot,Side\nSP1,Simulation_Port,Simulation_Port,23.510,15.425,270,top\nSP2,Simulation_Port,Simulation_Port,34.875,15.425,90,top\n'
(dst/'fab/roi-top-pos.csv').write_text(ports)
d=json.loads((src/'simulation.json').read_text());d['grid']['margin']['from_trace']=False
(dst/'simulation.json').write_text(json.dumps(d,indent=2))
m=json.loads((src/'manifest.json').read_text());m['roi_board_mm']={'x':[100,140],'y':[45,78]}
m['limitations']='Expanded diagnostic region; includes more ground via geometry, but still omits blind microvia drills, lumped components, chip, antenna and enclosure; power plane is port reference, unvalidated; not a tuned system or emissions model.'
m['prepared_from_geometry']=hashlib.sha256((src/'ems/geometry/geometry.xml').read_bytes()).hexdigest()
(dst/'manifest.json').write_text(json.dumps(m,indent=2))
print(json.dumps(m,indent=2))
