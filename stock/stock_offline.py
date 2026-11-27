"""Run an IQ recording through the emulator with the stock DSP firmware (DigRxx107.bin) in one mode.
usage: stock_offline.py iq.wav freq_hz mode seconds [start_s] [out.wav] [--sql N] [--vol G]"""
import argparse, os, sys, time, wave
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import djxcore as D

ap = argparse.ArgumentParser()
ap.add_argument('wav'); ap.add_argument('freq', type=float); ap.add_argument('mode'); ap.add_argument('secs', type=float)
ap.add_argument('start', type=float, nargs='?', default=0); ap.add_argument('out', nargs='?', default='stock_offline.wav')
ap.add_argument('--sql', type=int, default=0); ap.add_argument('--vol', type=int, default=2048)
ap.add_argument('--if', dest='ifm', default='MCF 8pole +-7.5k (default)')
ap.add_argument('--no-invert', action='store_true', help='IF chain without spectrum inversion')
ap.add_argument('--loop', action='store_true', help='repeat the recording')
a = ap.parse_args()
src = D.WavSource(a.wav, loop=a.loop)
src.seek(a.start)
c = D.Core(src.rate, a.freq - src.center, stock=True)
c.set_if_model(a.ifm)
if a.no_invert:
    c.set_if_invert(False)
print('fs_in', src.rate, 'fs1', c.fs1, 'offset', a.freq - src.center, 'fw', c.fw_name, c.fw_error or '')
modes = dict(D.STOCK_MODES)
c.set_mode(D.MODE_FM)
blk = max(1, int(src.rate // 50))
out, n, ready, t0 = [], 0, False, time.time()
lines = []
while n < a.secs * src.rate:
    x = src.read(blk)
    if x is None:
        break
    c.process(x); n += len(x)
    out.append(c.read_audio())
    for l in c.cpu_lines():
        lines.append((n / src.rate, l))
        if l.startswith('sr') and not ready:
            ready = True
            for cmd in (modes[a.mode], f'vl{a.vol:05d}', f'sl{a.sql:02d}'):
                c.lib.djx_cpu_send(cmd.encode())
time.sleep(1.0)
out.append(c.read_audio())
dt = time.time() - t0
au = np.concatenate(out)
w = wave.open(a.out, 'wb'); w.setnchannels(1); w.setsampwidth(2); w.setframerate(24000); w.writeframes(au.tobytes()); w.close()
st = c.status()
print(f'processed {n / src.rate:.1f} s in {dt:.1f} s; audio {len(au) / 24000:.1f} s rms {au.astype(float).std():.0f} -> {a.out}')
print(f'DSP load {st.dsp_load:.1f}% peak {st.dsp_load_peak:.1f}%  lag {st.fw_queue * 10} ms  UART {st.uart_tx}/{st.uart_rx} B  '
      f'AMBE channel packets {st.frames_to_ambe} decoded {st.speech_packets}  halted {st.halted} {c.lib.djx_fw_error().decode() if st.halted else ""}')
seen = {}
for t, l in lines:
    k = l[:2]
    seen[k] = seen.get(k, 0) + 1
    if seen[k] <= 6:
        print(f'  {t:7.2f} DSP->CPU {l}')
print('line types:', seen)
