# Map PyMethodDef / Cython method tables in a 32-bit LE ELF to function addresses.
# Usage: map_methods.py <so> <index.tsv>
import struct, sys
so, idx = sys.argv[1], sys.argv[2]
data = open(so, 'rb').read()
funcs = {int(l.split('\t')[0], 16): l.split('\t')[2].strip() for l in open(idx)}
# section headers
e_shoff, = struct.unpack_from('<I', data, 0x20)
e_shentsize, e_shnum, e_shstrndx = struct.unpack_from('<HHH', data, 0x2e)
secs = []
for i in range(e_shnum):
    name, typ, flags, addr, off, size = struct.unpack_from('<IIIIII', data, e_shoff + i * e_shentsize)
    secs.append([name, typ, flags, addr, off, size])
shstr = secs[e_shstrndx]
for s in secs:
    n = data[shstr[4] + s[0]:].split(b'\0', 1)[0].decode()
    s[0] = n
def v2o(va):
    for n, t, f, a, o, sz in secs:
        if a and a <= va < a + sz and t != 8:
            return o + va - a
def cstr(va):
    o = v2o(va)
    if o is None: return None
    s = data[o:o + 200].split(b'\0', 1)[0]
    try:
        s = s.decode()
    except UnicodeDecodeError:
        return None
    return s if s and all(32 <= ord(c) < 127 for c in s) else None
for n, t, f, a, o, sz in secs:
    if n not in ('.data', '.data.rel.ro', '.rodata', '.data.rel'):
        continue
    for off in range(0, sz - 16, 4):
        w0, w1, w2 = struct.unpack_from('<III', data, o + off)
        if w1 in funcs:
            name = cstr(w0)
            if name and name.replace('_', '').isalnum():
                print('%08x\t%s\t%s\tflags=%#x' % (w1, name, funcs[w1], w2))
