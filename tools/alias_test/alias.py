# Aliasing in each test case: every case repeats exactly every 1/F s (F below), so
# its true spectrum has lines only at multiples of F. Energy anywhere else between
# 20 Hz and 20 kHz is aliasing. Prints that energy relative to the wanted lines,
# and the loudest single alias relative to the loudest line.
import numpy as np, sys
from scipy.signal import windows
F={'saw_1009':1009,'saw_3001':3001,'tri_3001':3001,'square_1499':1499,'sine_fold8_1009':1009,'sine_fold20_2003':2003,
   'saw_fold6_1009':1009,'mix_fold12_521':521,'am_saw_fold4':101,'fm_sine':101,'fm_sine_fold6':101,'fm_saw_deep':101}
def metric(x, f):
    x=x[48000:96000].astype(np.float64)
    w=windows.blackmanharris(len(x)); X=np.abs(np.fft.rfft(x*w))**2
    fr=np.arange(len(X))
    band=(fr>=20)&(fr<=20000)
    k=np.round(fr/f); near=np.abs(fr-k*f)<=4
    harm=X[band&near].sum(); inh=X[band&~near].sum()
    peak=X[band&near].max(); worst=X[band&~near].max()
    return 10*np.log10(inh/harm+1e-30), 10*np.log10(worst/peak+1e-30)
dirs=sys.argv[1:]
names={'out1':'1.1.x (no OS)','out2':'2x oversampled','out4':'4x oversampled'}
print('%-18s'%'case'+''.join('%22s'%names.get(d,d) for d in dirs))
print('%-18s'%''+''.join('%22s'%'alias energy / peak' for d in dirs))
for n,f in F.items():
    row='%-18s'%n
    for d in dirs:
        a,b=metric(np.fromfile(f'{d}/{n}.f32',np.float32),f); row+='%11.1f dB %7.1f dB'%(a,b)
    print(row)
