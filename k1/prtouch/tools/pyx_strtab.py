# Recover Cython 0.29 __Pyx_StringTabEntry table: {PyObject **p; char *s; ssize_t n; char *enc; char is_unicode, is_str, intern}
# from a 32-bit LE MIPS .so. Prints "slot_va\tkind\tstring".
import struct, sys
data = open(sys.argv[1], 'rb').read()
RODATA = (0x1182b0, 0x1182b0 + 0x7040)
DATA_VA, DATA_OFF, DATA_SZ = 0x12f340, 0x11f340, 0x5a20
BSS = (0x135010, 0x135010 + 0x1d70)
def va2off(va):
    if RODATA[0] <= va < RODATA[1]: return va
    if DATA_VA <= va < DATA_VA + DATA_SZ: return va - 0x10000
seen = {}
for off in range(0, DATA_SZ - 20, 4):
    p, s, n, enc = struct.unpack_from('<IIiI', data, DATA_OFF + off)
    is_u, is_s, intern = data[DATA_OFF + off + 16:DATA_OFF + off + 19]
    if not (BSS[0] <= p < BSS[1]) or n <= 0 or n > 4000: continue
    so = va2off(s)
    if so is None: continue
    raw = data[so:so + n]
    if len(raw) != n or raw[-1] != 0 or b'\0' in raw[:-1]: continue
    if is_u > 1 or is_s > 1 or intern > 1: continue
    kind = 'n' if intern else ('kp_u' if is_u else 'kp_b' if not is_s else 'kp_s')
    seen[p] = (kind, raw[:-1].decode('utf-8', 'replace'))
for p in sorted(seen):
    k, s = seen[p]
    print('%08x\t%s\t%s' % (p, k, s.replace('\n', '\\n').replace('\t', '\\t')))
