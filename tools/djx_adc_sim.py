"""DJ-X100 receive-chain model -> ADC word stream exactly as the DSP sees it (McBSP odd words).

input : complex IQ, channel centred at 0 Hz: cf32 at 96 kS/s, or a 2-channel IQ WAV at any
        rate, resampled to 96 kS/s
output: uint16 little-endian, 12-bit offset-binary ADC words (0..4095), real IF 18 kHz @ 72 kS/s

model : IF crystal filter (Chebyshev lowpass equivalent, N poles, +-fc) -> real IF at 18 kHz ->
        sampled at 72 kS/s with no further anti-alias filter -> 12-bit quantiser (+ optional noise
        added before the crystal filter, given as Es/N0-ish SNR in a 16 kHz band).
usage : djx_adc_sim.py in.cf32 out.u16 [--poles 8] [--fc 7500] [--ripple 0.5] [--snr 99]
                                        [--peak-dbfs -6] [--freq-offset 0]
"""
import argparse
import numpy as np
from scipy import signal

ap = argparse.ArgumentParser()
ap.add_argument('inp'); ap.add_argument('out')
ap.add_argument('--poles', type=int, default=8)
ap.add_argument('--fc', type=float, default=7500)
ap.add_argument('--ripple', type=float, default=0.5)
ap.add_argument('--snr', type=float, default=99)
ap.add_argument('--peak-dbfs', type=float, default=-6)
ap.add_argument('--freq-offset', type=float, default=0.0, help='carrier offset to simulate [Hz]')
ap.add_argument('--seed', type=int, default=1)
a = ap.parse_args()

FS_IN = 96000
if a.inp.lower().endswith('.wav'):
    from fractions import Fraction
    from scipy.io import wavfile
    fs, d = wavfile.read(a.inp)
    d = d.astype(np.float64)
    if d.dtype.kind in 'iu' or np.max(np.abs(d)) > 2:
        d = d / 32768.0
    x = d[:, 0] + 1j * d[:, 1]
    r = Fraction(FS_IN, fs).limit_denominator(1000)
    x = signal.resample_poly(x, r.numerator, r.denominator)
else:
    x = np.fromfile(a.inp, np.complex64).astype(np.complex128)
if a.freq_offset:
    x *= np.exp(2j * np.pi * a.freq_offset * np.arange(len(x)) / FS_IN)
if a.snr < 99:
    rng = np.random.default_rng(a.seed)
    p = np.mean(np.abs(x) ** 2)
    n = (rng.standard_normal(len(x)) + 1j * rng.standard_normal(len(x))) / np.sqrt(2)
    x = x + n * np.sqrt(p / 10 ** (a.snr / 10) * FS_IN / 16000)
b, aa = (signal.cheby1(a.poles, a.ripple, a.fc, fs=FS_IN) if a.ripple > 0
         else signal.butter(a.poles, a.fc, fs=FS_IN))
y = signal.lfilter(b, aa, x)
y = signal.resample_poly(y, 3, 1)                     # 288 kS/s
t = np.arange(len(y)) / 288000.0
r = np.real(y * np.exp(2j * np.pi * 18000.0 * t))[::4]  # real IF 18 kHz sampled at 72 kS/s (aliasing kept)
r = r / np.max(np.abs(r)) * 10 ** (a.peak_dbfs / 20)
w = np.clip(np.round(r * 2047) + 2048, 0, 4095).astype('<u2')
w.tofile(a.out)
print(f'{a.out}: {len(w)} words, {len(w)/72000:.2f} s')
