"""Minimal rtl_tcp server for testing: serves an IQ WAV recording as 8-bit IQ, honours set-frequency by shifting
(within the recording's band), paced in real time."""
import socket, struct, sys, threading, time
import numpy as np
sys.path.insert(0, __import__('os').path.dirname(__file__))
import djxcore as D

def serve(path, port=1234, once=True):
    src = D.WavSource(path)
    srv = socket.socket(); srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('127.0.0.1', port)); srv.listen(1)
    print('fake rtl_tcp on', port, 'serving', path, flush=True)
    conn, _ = srv.accept()
    conn.sendall(b'RTL0' + struct.pack('>II', 5, 29))           # R820T, 29 gains
    state = {'freq': src.center, 'rate': src.rate}
    def rx():
        try:
            while True:
                c = conn.recv(5)
                if len(c) < 5: return
                cmd, val = struct.unpack('>BI', c)
                if cmd == 1: state['freq'] = val
                if cmd == 2: state['rate'] = val
                print('cmd', cmd, val, flush=True)
        except OSError:
            pass
    threading.Thread(target=rx, daemon=True).start()
    ph = 0.0
    try:
        while True:
            n = int(src.rate * 0.02)
            x = src.read(n)
            shift = state['freq'] - src.center                  # emulate retuning inside the recorded band
            t = (np.arange(n) + ph) / src.rate
            x = x * np.exp(-2j * np.pi * shift * t).astype(np.complex64)
            ph += n
            iq = np.empty(2 * n, np.uint8)
            iq[0::2] = np.clip(np.round(x.real * 128 + 127.5), 0, 255)
            iq[1::2] = np.clip(np.round(x.imag * 128 + 127.5), 0, 255)
            conn.sendall(iq.tobytes())
            time.sleep(0.02)
    except OSError:
        pass

if __name__ == '__main__':
    serve(sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else 1234)
