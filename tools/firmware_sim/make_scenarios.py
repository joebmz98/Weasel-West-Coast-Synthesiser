# Writes the scripted sessions for the firmware simulator (sim.cpp) into <out dir>.
#
# Scenario lines (times in ms):
#   <ms> pot <index> <0..1>          pot position (index as in hardware.h PotIndex)
#   <ms> btn <bit> <0|1>             button down / up (bit as in hardware.h ButtonBit)
#   <ms> patch <src> <dst> <0|1>     patch cable in / out (src 0 SEQ 1 EG 2 PLSR 3 RNDM)
#   <ms> midi <hex bytes...>         bytes arriving on the MIDI input
#   <ms> end                         end of the session
import math, os, random, shutil, sys

out = sys.argv[1]
P = dict(S1=0, S2=1, S3=2, S4=3, S5=4, ATT=5, SUS=6, DEC=7, PLS_CTRL=8, PLS_PER=9, MOD_FC=10, MOD_F=11, MOD_FINE=12,
         MOD_MC=13, MOD_M=14, COMP_FC=15, COMP_F=16, COMP_FINE=17, WF_C=18, WF=19, TIMB=20, L1C=21, L1L=22, L2C=23,
         L2L=24, CLK=25, REV=26, VOL=27)
def SR(sr, d): return 32 - 8 * sr + d
B = dict(SEQ_TRIG=SR(1, 0), SEQ_STAGES=SR(1, 1), ST1=SR(1, 2), ST2=SR(1, 3), ST3=SR(1, 4), ST4=SR(1, 5), ST5=SR(1, 6),
         ENV_TRIG=SR(1, 7), ENV_MODE=SR(2, 0), PLS_MODE=SR(2, 1), PLS_MAN=SR(2, 2), AMFM=SR(2, 3), MOD_WAVE=SR(2, 4),
         POL=SR(2, 5), COMP_WAVE=SR(2, 6), MIDI=SR(2, 7), LPG1=SR(3, 0), LPG2=SR(3, 1), RND=SR(3, 2), SYNC=SR(3, 3))

def base(lines, **kw):
    d = dict(S1=.2, S2=.5, S3=.8, S4=.4, S5=.6, ATT=.1, SUS=.3, DEC=.4, PLS_CTRL=.3, PLS_PER=.3, MOD_FC=.2, MOD_F=.35,
             MOD_FINE=.5, MOD_MC=.2, MOD_M=.3, COMP_FC=.3, COMP_F=.5, COMP_FINE=.5, WF_C=.2, WF=.3, TIMB=.5, L1C=.4,
             L1L=.7, L2C=.3, L2L=.5, CLK=.6, REV=.3, VOL=.8)
    d.update(kw)
    for k, v in d.items(): lines.append(f"0 pot {P[k]} {v}")
def press(lines, t, b, hold=40):
    lines.append(f"{t} btn {B[b]} 1"); lines.append(f"{t + hold} btn {B[b]} 0")
def save(name, lines, end):
    lines.append(f"{end} end")
    lines.sort(key=lambda l: float(l.split()[0]))
    open(os.path.join(out, name), 'w').write("\n".join(lines) + "\n")

# pots: sweeps of pitch, fold, timbre and mod depth, a few mode changes
L = []; base(L)
for i in range(400):
    t = 50 + i * 10
    L.append(f"{t} pot {P['COMP_F']} {0.5 + 0.5 * math.sin(i / 30)}")
    L.append(f"{t} pot {P['WF']} {(i % 100) / 100}")
    L.append(f"{t} pot {P['TIMB']} {(i % 57) / 57}")
    L.append(f"{t} pot {P['MOD_M']} {(i % 80) / 80}")
press(L, 1500, 'AMFM'); press(L, 2500, 'COMP_WAVE'); press(L, 3000, 'MOD_WAVE'); press(L, 3500, 'POL')
save('pots.sc', L, 4500)

# buttons: every button in turn, step switches, the MIDI shift
L = []; base(L); t = 100
order = ['SEQ_TRIG', 'SEQ_STAGES', 'ENV_TRIG', 'ENV_MODE', 'PLS_MODE', 'PLS_MAN', 'AMFM', 'MOD_WAVE', 'POL',
         'COMP_WAVE', 'MIDI', 'LPG1', 'LPG2', 'RND', 'SYNC']
for rep in range(4):
    for b in order: press(L, t, b, 30 + rep * 20); t += 170
for i, s in enumerate(['ST1', 'ST3', 'ST5']): L.append(f"{t + i * 100} btn {B[s]} 1")
t += 600
L.append(f"{t} btn {B['MIDI']} 1"); press(L, t + 700, 'LPG1'); press(L, t + 900, 'LPG2'); L.append(f"{t + 1300} btn {B['MIDI']} 0"); t += 1600
L.append(f"{t} btn {B['MIDI']} 1"); press(L, t + 100, 'ENV_MODE'); L.append(f"{t + 300} btn {B['MIDI']} 0"); t += 600
save('buttons.sc', L, t + 1000)

# patch: every source to every destination, then a few patches with mode changes
L = []; base(L, CLK=.75); t = 100
for s in range(4):
    for d in range(7):
        L.append(f"{t} patch {s} {d} 1"); L.append(f"{t + 350} patch {s} {d} 0"); t += 300
L.append(f"{t} patch 1 3 1"); L.append(f"{t} patch 2 4 1"); L.append(f"{t} patch 0 1 1"); L.append(f"{t} patch 3 5 1")
press(L, t + 200, 'PLS_MODE'); press(L, t + 700, 'ENV_MODE'); press(L, t + 1200, 'AMFM')
save('patch.sc', L, t + 2500)

# midi: notes, held chords, all notes off, clock with start / stop / continue, divisions
L = []; base(L); t = 100
press(L, t, 'SEQ_TRIG'); press(L, t + 60, 'SEQ_TRIG'); press(L, t + 120, 'ENV_TRIG'); press(L, t + 180, 'ENV_TRIG'); press(L, t + 240, 'MIDI'); t = 500
random.seed(3)
for i in range(60):
    n = random.randint(36, 84)
    L.append(f"{t} midi 90 {n:02X} 64"); L.append(f"{t + random.randint(20, 150)} midi 80 {n:02X} 00"); t += random.randint(30, 120)
press(L, t, 'ENV_MODE'); t += 100
for i in range(20):
    L.append(f"{t} midi 90 {40 + i:02X} 50"); L.append(f"{t + 60} midi 90 {50 + i:02X} 50")
    L.append(f"{t + 120} midi 80 {50 + i:02X} 00"); L.append(f"{t + 200} midi 80 {40 + i:02X} 00"); t += 250
L.append(f"{t} midi B0 7B 00"); t += 50
press(L, t, 'SEQ_TRIG'); press(L, t + 60, 'SYNC'); t += 200
L.append(f"{t} midi FA")
for i in range(400): L.append(f"{t + i * 20.833:.3f} midi F8")
L.append(f"{t + 3000} midi FC"); L.append(f"{t + 3500} midi FB")
for i in range(50): L.append(f"{t + 4000 + i * 15:.3f} pot {P['CLK']} {i / 50}")
save('midi.sc', L, t + 9000)

# demo: complex oscillator at its harshest (fold sweep, folded saw sweep, FM, AM + fold)
shutil.copy(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'demo.sc'), os.path.join(out, 'demo.sc'))
