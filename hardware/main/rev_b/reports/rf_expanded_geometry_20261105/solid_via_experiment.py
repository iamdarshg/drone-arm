"""Prepare a source-pinned solid-PEC via sensitivity model; never alter input."""
from pathlib import Path
import hashlib
import json
import sys
import xml.etree.ElementTree as ET

if len(sys.argv) != 3:
    raise SystemExit('usage: python solid_via_experiment.py INPUT_XML OUTPUT_XML')
source, output = map(Path, sys.argv[1:])
assert not output.exists(), 'refusing to overwrite an existing geometry'
digest = hashlib.sha256(source.read_bytes()).hexdigest()
expected = '9b8a1782e6fa254344bfdd8f06a261b51b708fcf83d91d35fa520e8d2591b746'
assert digest == expected, 'unexpected source geometry'
tree = ET.parse(source)
properties = {x.attrib.get('Name'): x for x in tree.getroot().iter()
              if x.attrib.get('Name') in ('Via', 'ViaFilling')}
assert set(properties) == {'Via', 'ViaFilling'}
via = properties['Via'].find('Primitives')
filling = properties['ViaFilling'].find('Primitives')
assert via is not None and filling is not None
assert len(via) == len(filling) == 36
for child in list(filling):
    filling.remove(child)
tree.write(output, encoding='utf-8', xml_declaration=True)
summary = {
    'source_sha256': digest,
    'output_sha256': hashlib.sha256(output.read_bytes()).hexdigest(),
    'via_count': len(via),
    'change': 'PEC via outer polygons retained; dielectric hole-fill primitives removed, making via cross-sections solid PEC',
    'limitations': 'sensitivity approximation, not actual plated barrel or physical model validation; requires unused-primitive and mesh-convergence checks',
}
print(json.dumps(summary, indent=2))
