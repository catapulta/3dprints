# Resolve MIPS-PIC GOT-relative expressions in Ghidra C dumps to Cython names/constants.
# usage: annotate_c.py <so> <decomp_dir>   -> writes <decomp_dir>/annot/*.c and consts.tsv
import os, re, struct, sys
so, dd = sys.argv[1], sys.argv[2]
data = open(so, 'rb').read()
SHIFT = 0x10000
# file VA -> file offset (segments: text/rodata identity, data at -0x10000)
def foff(va):
    if 0 <= va < 0x11f330: return va
    if 0x12f330 <= va < 0x135010: return va - 0x10000
    return None
def u32(va):
    o = foff(va); return None if o is None else struct.unpack_from('<I', data, o)[0]
def f64(va):
    o = foff(va); return None if o is None else struct.unpack_from('<d', data, o)[0]
def cstr(va):
    o = foff(va)
    if o is None: return None
    s = data[o:o + 300].split(b'\0', 1)[0]
    if len(s) < 2: return None
    try: s = s.decode()
    except UnicodeDecodeError: return None
    return s if all(32 <= ord(c) < 127 or c in '\n\t' for c in s) else None
strtab = {}
for l in open(os.path.join(dd, 'strtab.tsv')):
    a, k, s = l.rstrip('\n').split('\t', 2)
    strtab[int(a, 16)] = ('n_' if k == 'n' else 's_') + s
funcs = {}
for l in open(os.path.join(dd, 'index.tsv')):
    a, sz, n = l.rstrip('\n').split('\t')
    funcs[int(a, 16) - SHIFT] = n
consts = {}
def describe(t):
    if t in strtab: return strtab[t]
    if t in consts: return consts[t]
    if t in funcs: return 'FN:' + funcs[t]
    s = cstr(t)
    if s is not None: return 'str:' + repr(s)[:120]
    return None
PAT = re.compile(r'\(?\b(PTR_[A-Za-z_]*?_?)(00144[0-9a-f]{3})\s*\+\s*(-?0x[0-9a-f]+|-?\d+)\)?')
def resolve(m):
    got_g = int(m.group(2), 16)
    base = u32(got_g - SHIFT)
    if base is None: return None
    off = int(m.group(3), 0)
    return base + off
def sub(text):
    def r(m):
        t = resolve(m)
        if t is None: return m.group(0)
        d = describe(t)
        return '%s/*@%x%s*/' % (m.group(0), t, (' ' + d) if d else '')
    return PAT.sub(r, text)
# pass 1: constants from module init (PyFloat_FromDouble / PyLong_FromLong stores)
init = [f for f in os.listdir(os.path.join(dd, 'funcs'))]
best = max(init, key=lambda f: open(os.path.join(dd, 'funcs', f)).read().count('PyFloat_FromDouble'))
lines = open(os.path.join(dd, 'funcs', best)).read().split('\n')
for i, l in enumerate(lines):
    m = re.search(r'(PyFloat_FromDouble|PyLong_FromLong)_[0-9a-f]+\)\((.*)\);', l)
    if not m: continue
    arg = m.group(2).strip()
    val = None
    mm = PAT.search(arg)
    if mm:
        t = resolve(mm)
        val = f64(t) if m.group(1) == 'PyFloat_FromDouble' else struct.unpack('<i', struct.pack('<I', u32(t)))[0]
    else:
        try:
            v = int(arg, 0); v = v - (1 << 32) if v >= 1 << 31 else v
            val = v if m.group(1) == 'PyLong_FromLong' else float(v)
        except ValueError: pass
    for l2 in lines[i + 1:i + 3]:
        m2 = re.search(r'\*\(\w+ \*\)\((PTR_[A-Za-z_]*?_?00144[0-9a-f]{3}\s*\+\s*-?0x[0-9a-f]+)\)\s*=', l2)
        if m2 and val is not None:
            t = resolve(PAT.search(m2.group(1)))
            consts[t] = ('float_' if m.group(1) == 'PyFloat_FromDouble' else 'int_') + repr(val)
            break
os.makedirs(os.path.join(dd, 'annot'), exist_ok=True)
with open(os.path.join(dd, 'consts.tsv'), 'w') as f:
    for t in sorted(consts): f.write('%x\t%s\n' % (t, consts[t]))
for fn in os.listdir(os.path.join(dd, 'funcs')):
    src = open(os.path.join(dd, 'funcs', fn)).read()
    open(os.path.join(dd, 'annot', fn), 'w').write(sub(src))
print('init fn', best, 'consts', len(consts))
