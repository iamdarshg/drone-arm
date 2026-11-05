import numpy as np,json
from pathlib import Path
root=Path('/work/rf_gerber_roi_broadband_20261105/ems/simulation/0')
results={}
for name in ['port_ut_0A','port_ut_0B','port_ut_1A']:
 d=np.loadtxt(root/name,comments='%'); t=d[:,0];v=d[:,1];cut=t>1e-9
 dt=float(np.median(np.diff(t)));tail=v[cut];tail=tail-np.mean(tail);win=np.hanning(len(tail))
 f=np.fft.rfftfreq(len(tail),dt);amp=abs(np.fft.rfft(tail*win));idx=int(np.argmax(amp[1:])+1)
 results[name]={'sample_count':int(len(t)),'dt_seconds':dt,'time_span_s':float(t[-1]-t[0]),'tail_from_s':1e-9,'dominant_tail_bin_hz':float(f[idx]),'nominal_bin_width_hz':float(f[1]-f[0]),'tail_peak_voltage':float(np.max(abs(tail))),'overall_peak_voltage':float(np.max(abs(v)))}
out={'results':results,'interpretation':'exploratory probe-tail spectrum; bin is resolution-limited and does not identify physical resonance or cause'}
Path('/work/rf_gerber_roi_broadband_20261105/tail_spectrum_diagnostic.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
