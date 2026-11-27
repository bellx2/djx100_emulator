"""Broadcast FM (BK1068 "RADIO" mode) test of the whole radio with an IQ WAV that holds a WFM station.

    python wfm_test.py station.wav MHz [out_dir]

Types the frequency + SET on the keypad (76..108 MHz switches the firmware to its BK1068 receiver), turns the
volume ring up and checks: the SDR channel follows the BK1068 tuning, the channel runs the WFM model, the S-meter
register gets a level and audio comes out. PASS / FAIL.
"""
import os
import shutil
import sys
import tempfile
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import djxcore as D  # noqa: E402

wav, mhz = sys.argv[1], sys.argv[2]
out = sys.argv[3] if len(sys.argv) > 3 else tempfile.mkdtemp()
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
script, t = [], 8.0
for k in list(mhz) + ['SET']:
    script += [(t, 'k', k, True), (t + 0.15, 'k', k, False)]
    t += 0.4
script += [(t + 1.0, 'e', 1, 12)]
end_t = t + 6.0
mask, tuned_n, log, nxt = 0, -1, [], 0
blk = int(src.rate * 0.02)
t0 = time.time()
abuf = np.zeros(0, np.int16)
while core.radio_time() < end_t and time.time() - t0 < 300:
    core.process(src.read(blk))
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
    a = core.read_audio()
    if len(a):
        abuf = np.concatenate([abuf, a])[-12000:]      # the latest 0.5 s of audio
    if rt >= nxt:
        nxt = rt + 1.0
        dbm, snr, af, vol = core.radio_rf()
        rms = float(np.sqrt(np.mean(abuf.astype(np.float64) ** 2))) if len(abuf) else 0.0
        wfm = core.radio_wfm()
        log.append((rt, hz, wfm, dbm, af, rms))
        print(f'{rt:6.1f} s  tuned {hz / 1e6:9.4f} {"WFM/BK1068" if wfm else "main rx   "}  RF {dbm:6.1f} dBm  SNR {snr:5.1f}'
              f'  AF {"on " if af else "off"}  audio rms {rms:7.1f}', flush=True)
    time.sleep(0.002)
try:
    from PIL import Image
    n, img = core.lcd()
    Image.fromarray(img[:, :, :3].copy()).resize((640, 480), Image.NEAREST).save(os.path.join(out, 'lcd.png'))
except ImportError:
    pass
want = round(float(mhz) * 1e6)
last = log[-2:]
good = len(last) == 2 and all(abs(r[1] - want) < 1 and r[2] and r[4] and r[5] > 50 for r in last)
print('PASS' if good else 'FAIL', '- BK1068 tuned, WFM channel, audio;', 'lcd', out)
sys.exit(0 if good else 1)
