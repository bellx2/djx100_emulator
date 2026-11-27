"""C5517 (C5515 family) unencrypted boot image tool (format: TI SPRABD7 table 3).

  0x09AA | entry (32, byte address) | regcfg count N (16) | N x (io addr, value) |
  sections: word count (16) | dest word address (32) | data words | pad to (count + 2) rounded up to 4 words |
  0x0000 end

usage:
  bootimg.py list  image.bin
  bootimg.py check image.bin                 rebuild from the parse and compare byte for byte
  bootimg.py extract image.bin outdir        one file per section: <word addr>.bin (big-endian words)
  bootimg.py build out.bin --entry 0x... [--reg 0x1C1F=0x0000 ...] sec1.bin@0x... sec2.bin@0x...
"""
import os
import struct
import sys


def parse(data):
    w = lambda o: struct.unpack_from('>H', data, o)[0]
    if w(0) != 0x09AA:
        raise ValueError('bad signature 0x%04X' % w(0))
    entry = struct.unpack_from('>I', data, 2)[0]
    n = w(6)
    o = 8
    regs = []
    for _ in range(n):
        regs.append((w(o), w(o + 2)))
        o += 4
    secs = []
    while True:
        cnt = w(o)
        if cnt == 0:
            o += 2
            break
        addr = struct.unpack_from('>I', data, o + 2)[0]
        payload = data[o + 6:o + 6 + 2 * cnt]
        m = (cnt + 2 + 3) // 4 * 4                       # words after the count field
        pad = data[o + 6 + 2 * cnt:o + 2 + 2 * m]
        secs.append({'addr': addr, 'data': payload, 'pad': pad, 'off': o})
        o += 2 + 2 * m
    trailer = data[o:]
    return {'entry': entry, 'regs': regs, 'sections': secs, 'trailer': trailer}


def build(entry, regs, sections, pad_byte=0x20, trailer=b''):
    out = bytearray(struct.pack('>HIH', 0x09AA, entry, len(regs)))
    for a, v in regs:
        out += struct.pack('>HH', a, v)
    for s in sections:
        d = s['data']
        if len(d) % 2:
            d += b'\x00'
        cnt = len(d) // 2
        if not 0 < cnt < 0x10000:
            raise ValueError('section size')
        m = (cnt + 2 + 3) // 4 * 4
        out += struct.pack('>HI', cnt, s['addr']) + d
        padlen = 2 * (m - 2 - cnt)
        out += s.get('pad', bytes([pad_byte]) * padlen)[:padlen].ljust(padlen, bytes([pad_byte]))
    out += b'\x00\x00' + trailer
    return bytes(out)


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    cmd, path = sys.argv[1], sys.argv[2]
    if cmd == 'build':
        entry, regs, secs = None, [], []
        args = sys.argv[3:]
        i = 0
        while i < len(args):
            a = args[i]
            if a == '--entry':
                entry = int(args[i + 1], 0); i += 2; continue
            if a == '--reg':
                k, v = args[i + 1].split('='); regs.append((int(k, 0), int(v, 0))); i += 2; continue
            f, addr = a.rsplit('@', 1)
            secs.append({'addr': int(addr, 0), 'data': open(f, 'rb').read()}); i += 1
        if entry is None:
            raise SystemExit('--entry required')
        open(path, 'wb').write(build(entry, regs, secs))
        print(f'{path}: {len(secs)} sections, entry 0x{entry:X}')
        return 0
    data = open(path, 'rb').read()
    img = parse(data)
    if cmd == 'list':
        print(f'entry 0x{img["entry"]:06X} (byte address), {len(img["regs"])} register writes, '
              f'{len(img["sections"])} sections, trailer {img["trailer"].hex()}')
        for a, v in img['regs']:
            print(f'  reg  io 0x{a:04X} = 0x{v:04X}')
        for s in img['sections']:
            n = len(s['data']) // 2
            print(f'  sec  file 0x{s["off"]:06X}  word addr 0x{s["addr"]:06X} .. 0x{s["addr"] + n - 1:06X}'
                  f'  ({n} words, byte addr 0x{2 * s["addr"]:06X})')
    elif cmd == 'check':
        again = build(img['entry'], img['regs'], img['sections'], trailer=img['trailer'])
        print('round trip', 'OK' if again == data else 'DIFFERENT', len(again), len(data))
        return 0 if again == data else 1
    elif cmd == 'extract':
        outdir = sys.argv[3]
        os.makedirs(outdir, exist_ok=True)
        for s in img['sections']:
            open(os.path.join(outdir, f'{s["addr"]:06X}.bin'), 'wb').write(s['data'])
    return 0


if __name__ == '__main__':
    sys.exit(main())
