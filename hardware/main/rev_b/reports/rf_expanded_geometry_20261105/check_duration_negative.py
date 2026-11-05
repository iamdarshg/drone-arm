import re
from pathlib import Path
log=Path('/work/expanded_915_solver.log').read_text(encoding='utf-8-sig',errors='replace')
energy=re.findall(r'Timestep:\s*(\d+).*?Energy:.*?\(\s*-\s*([\d.]+)dB\)',log)
it=re.search(r'Time for\s+(\d+)\s+iterations',log);exc=re.search(r'Excitation signal length is:\s*(\d+)',log)
assert energy and it and exc
total=int(it.group(1));pulse=int(exc.group(1))
assert total==103599 and pulse==108890 and total<pulse
assert log.count('Unused primitive')==21
print('PASS negative fixture: rejected prior completed run because total steps',total,'< excitation',pulse,'and unused vias=21; parsed energy samples:',len(energy))
