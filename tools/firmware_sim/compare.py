# Compares the reference and new runs of each session: audio sample for sample, and
# the LED log (every I2C write to the AW9523s).
import numpy as np, sys, glob, os
d = sys.argv[1]
ok = True
for f in sorted(glob.glob(d + '/*.ref.f32')):
    n = os.path.basename(f).split('.')[0]
    a = np.fromfile(f, np.float32); b = np.fromfile(f'{d}/{n}.new.f32', np.float32)
    la = open(f'{d}/{n}.ref.led').read(); lb = open(f'{d}/{n}.new.led').read()
    same = len(a) == len(b) and np.array_equal(a, b)
    if same: audio = 'IDENTICAL'
    elif len(a) != len(b): audio = 'DIFFERENT LENGTH'
    else:
        first = int(np.argmax(a != b)) // 2
        rms = lambda x: 20 * np.log10(np.sqrt(np.mean(x.astype(float) ** 2)) + 1e-12)
        audio = 'differs from %.3f s (level %.2f dB vs %.2f dB)' % (first / 48000, rms(a), rms(b))
    print(f'{n:8s} audio {audio:45s} leds {"IDENTICAL" if la == lb else "DIFFER"}')
    ok &= same and la == lb
print('ALL IDENTICAL' if ok else 'NOT IDENTICAL (expected where the sound was changed on purpose)')
