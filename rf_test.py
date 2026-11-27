"""RF section / squelch / AF volume test of the whole radio (CPU + DSP firmware in djxemu.dll) with an IQ WAV.

    python rf_test.py data/test_145000000Hz.wav [out_dir]

Uses a new EEPROM image (factory state). Script (radio time): tune 145.1 (signal) by keypad, volume ring +12,
SQL to 3 (FUNC, MONI, dial +3), then 145.05 (no signal) and back to 145.1. Prints level, S-meter (from the CPU's RAM),
BUSY / AF amplifier and the audio RMS each second; exit code 1 if the expected behaviour is not seen.
"""
import os
import shutil
import sys
import tempfile
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import djxcore as D  # noqa: E402

wav = sys.argv[1] if len(sys.argv) > 1 else os.path.join(D.ROOT, 'data', 'test_145000000Hz.wav')
out = sys.argv[2] if len(sys.argv) > 2 else tempfile.mkdtemp()
os.makedirs(out, exist_ok=True)
ee = os.path.join(out, 'eeprom.bin')
if os.path.exists(ee):
    os.remove(ee)                            # a new radio: the emulator creates the EEPROM (markers + calibration)
src = D.WavSource(wav)
core = D.Core(src.rate, 0.0)
err = core.load_radio(eeprom=ee)
if err:
    sys.exit('load_radio: ' + err)
core.set_mode(D.MODE_FM)
core.set_volume(1.0)


def keys(names, t):
    s = []
    for n in names:
        s += [(t, 'k', n, True), (t + 0.15, 'k', n, False)]
        t += 0.4
    return s, t


script, t = keys(list('145.1') + ['SET'], 8.0)
script += [(t + 0.5, 'e', 1, 12)]                                    # volume ring
s2, t2 = keys(['FUNC', 'MONI/SQL'], t + 1.5)
script += s2 + [(t2 + 0.3, 'e', 0, 3)]                               # SQL 0 -> 3
s3, t3 = keys(list('145.05') + ['SET'], t2 + 4.0)
script += s3
s4, t4 = keys(list('145.1') + ['SET'], t3 + 4.0)
script += s4
end_t = t4 + 4.0
script.sort(key=lambda a: a[0])
mask = 0
tuned_n = -1
t0 = time.time()
log = []
blk = int(src.rate * 0.02)
next_print = 0
abuf = np.zeros(0, np.int16)
while core.radio_time() < end_t and time.time() - t0 < 400:
    x = src.read(blk)
    core.process(x)
    rt = core.radio_time()
    while script and script[0][0] <= rt:
        a = script.pop(0)
        if a[1] == 'k':
            c = 1 << D.KEY_CODES[a[2]]
            mask = mask | c if a[3] else mask & ~c
            core.keys(mask)
        else:
            core.encoder(a[2], a[3])
    hz, n = core.tuned()
    if n != tuned_n and hz:
        tuned_n = n
        core.set_offset(hz - src.center)
    for ev in core.events():
        if os.environ.get('RF_EVLOG') and ('CPU>DSP' in ev or 'DSP>CPU' in ev or 'DSP reset' in ev):
            print(f'{core.radio_time():6.2f} {ev}')
    a = core.read_audio()
    if len(a):
        abuf = np.concatenate([abuf, a])[-12000:]      # the latest 0.5 s of audio
    if rt >= next_print:
        next_print = rt + 1.0
        dbm, snr, af, vol = core.radio_rf()
        rms = float(np.sqrt(np.mean(abuf.astype(np.float64) ** 2))) if len(abuf) else 0.0
        log.append((rt, hz, dbm, snr, af, vol, rms))
        print(f'{rt:6.1f} s  tuned {hz / 1e6:9.4f}  RF {dbm:6.1f} dBm  SNR {snr:5.1f}  AF {"on " if af else "off"}'
              f'  vol {vol}  audio rms {rms:7.1f}', flush=True)
    time.sleep(0.004)
n, img = core.lcd()
try:
    from PIL import Image
    Image.fromarray(img[:, :, :3].copy()).resize((640, 480), Image.NEAREST).save(os.path.join(out, 'lcd.png'))
except ImportError:
    pass
sig = [r for r in log if abs(r[1] - 145.1e6) < 1]
nos = [r for r in log if abs(r[1] - 145.05e6) < 1]
nos = [r for r in nos if nos[0][0] + 0.9 <= r[0] <= t3 + 3.9]        # settled, before the next key beeps
good = len(sig) >= 2 and len(nos) >= 2 and all(r[4] and r[6] > 50 and r[2] > -80 for r in sig[-2:]) and \
    all((not r[4]) and r[6] < 5 and r[2] < -115 for r in nos)
print('PASS' if good else 'FAIL', '- signal: AF on with audio; no signal: squelch closed, silent;', 'lcd', out)
sys.exit(0 if good else 1)
