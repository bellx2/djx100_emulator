"""Whole-radio smoke test without SDR/GUI: boot the stock CPU + DSP firmware in djxemu.dll, press keys, save the LCD.

    python radio_headless.py out_dir [seconds]
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import djxcore  # noqa: E402
from PIL import Image  # noqa: E402

out = sys.argv[1] if len(sys.argv) > 1 else '.'
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 12
os.makedirs(out, exist_ok=True)
core = djxcore.Core(2.4e6, 0.0)
e = core.load_radio()
if e:
    sys.exit('load_radio: ' + e)
t0 = time.time()
script = [(8.0, 'enc', 0, 5), (9.0, 'key', 'MODE', 0.3), (10.5, 'key', 'MODE', 0.3)]   # radio-time actions
done = set()
held_until = None
while time.time() - t0 < secs:
    time.sleep(0.05)
    rt = core.radio_time()
    for i, a in enumerate(script):
        if i not in done and rt >= a[0]:
            done.add(i)
            if a[1] == 'enc':
                core.encoder(a[2], a[3])
            else:
                core.keys(1 << djxcore.KEY_CODES[a[2]])
                held_until = rt + a[3]
    if held_until and rt >= held_until:
        core.keys(0)
        held_until = None
    for ev in core.events():
        print(f'{rt:6.2f} {ev}')
    core.read_audio()
n, img = core.lcd()
hz, cnt = core.tuned()
st = core.status()
print(f'wall {time.time() - t0:.1f} s, radio time {core.radio_time():.2f} s, lcd frames {n}, tuned {hz} Hz ({cnt} calls)'
      f', error "{core.radio_error()}"')
Image.fromarray(img[:, :, :3].copy()).resize((640, 480), Image.NEAREST).save(os.path.join(out, 'lcd.png'))
core.radio_save()
