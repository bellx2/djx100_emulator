"""Synthetic IQ test recordings for the emulator (SDR# style WAV, centre frequency in the file name).
  make_test_iq.py analog  -> data/test_145000000Hz.wav : NFM 1 kHz (+-2.5 kHz) at 145.100, AM 800 Hz (60 %) at 144.800
  make_test_iq.py wfm     -> data/wfm_80000000Hz.wav   : broadcast FM 1 kHz (+-45 kHz) at 80.300 MHz, 2.4 MS/s u8
"""
import os, struct, sys
import numpy as np

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')


def wav(path, x, fs, u8=False):
    if u8:
        iq = np.empty(2 * len(x), np.uint8)
        iq[0::2] = np.clip(x.real * 127 + 127.5, 0, 255); iq[1::2] = np.clip(x.imag * 127 + 127.5, 0, 255)
        bits, ba = 8, 2
    else:
        iq = np.empty(2 * len(x), np.int16)
        iq[0::2] = np.clip(x.real * 32767, -32767, 32767); iq[1::2] = np.clip(x.imag * 32767, -32767, 32767)
        bits, ba = 16, 4
    d = iq.tobytes()
    hdr = (b'RIFF' + struct.pack('<I', 36 + len(d)) + b'WAVEfmt ' +
           struct.pack('<IHHIIHH', 16, 1, 2, fs, fs * ba, ba, bits) + b'data' + struct.pack('<I', len(d)))
    open(path, 'wb').write(hdr + d)
    print('wrote', path)


what = sys.argv[1] if len(sys.argv) > 1 else 'analog'
rng = np.random.default_rng(1)
if what == 'analog':
    fs, dur = 960000, 6
    t = np.arange(int(fs * dur)) / fs
    m = np.sin(2 * np.pi * 1000 * t)
    x = 0.3 * np.exp(1j * (2 * np.pi * 2500 * np.cumsum(m) / fs + 2 * np.pi * 100e3 * t))
    x += 0.2 * (1 + 0.6 * np.sin(2 * np.pi * 800 * t)) * np.exp(1j * 2 * np.pi * (-200e3) * t)
    x += 0.003 * (rng.standard_normal(len(t)) + 1j * rng.standard_normal(len(t)))
    wav(os.path.join(ROOT, 'data', 'test_145000000Hz.wav'), x, fs)
elif what == 'wfm':
    fs, dur = 2400000, 3
    t = np.arange(int(fs * dur)) / fs
    m = 0.6 * np.sin(2 * np.pi * 1000 * t)
    x = 0.4 * np.exp(1j * (2 * np.pi * 75000 * np.cumsum(m) / fs + 2 * np.pi * 300e3 * t))
    wav(os.path.join(ROOT, 'data', 'wfm_80000000Hz.wav'), x, fs, u8=True)
