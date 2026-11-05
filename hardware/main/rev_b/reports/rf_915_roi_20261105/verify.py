from pathlib import Path
import csv
import json
import math
import sys

here = Path(__file__).resolve().parent
paths = [Path(p) for p in sys.argv[1:]] or [here / 'coarse_Sx0.csv', here / 'refined_Sx0.csv']
if len(paths) != 2:
    raise SystemExit('supply exactly two CSV paths, or none')
expected = [(-9.383865381847333, -2.9468063675548213),
            (-8.706339394897643, -2.4770546022313895)]
for path, target in zip(paths, expected):
    with path.open(newline='') as f:
        next(f)
        rows = [[float(v.strip()) for v in row[:5]] for row in csv.reader(f) if row]
    freq, a, b, c, d = min(rows, key=lambda row: abs(row[0] - 915))
    assert freq == 915
    s11, s21 = complex(a, c), complex(b, d)
    values = (20 * math.log10(abs(s11)), 20 * math.log10(abs(s21)))
    assert all(abs(x-y) < 1e-9 for x, y in zip(values, target))
    print(json.dumps({'file': str(path), 'frequency_mhz': freq,
                      's11_db': values[0], 's21_db': values[1],
                      'power_sum': abs(s11)**2 + abs(s21)**2}))
