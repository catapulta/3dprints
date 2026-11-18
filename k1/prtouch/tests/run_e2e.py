"""Offline end-to-end replay of k1/prtouch/prtouch_v1_wrapper.py against the
stock module's traces (k1/prtouch/traces/*.log.gz).

Run from anywhere:  python3 k1/prtouch/tests/run_e2e.py [-v] [scenario ...]
(scenarios: boot G28 G29 AHZ CBM NCLR RP; default all). Also runs under 3.8:
  uv run --no-project --python 3.8 python k1/prtouch/tests/run_e2e.py
E2E_WRAPPER=<path> replays another copy of the module instead.

Each scenario builds a fresh fake printer (fake_klippy.Env), constructs the
wrapper class through the boot path (fake config from k1/printer.cfg), applies
the instance STATE the earlier traces left behind, then drives the traced
top-level command exactly as klippy does, injecting the traced MCU responses
keyed on the command sequence. Both sides are normalised to event lists and
diffed; the first divergence is printed in context.
"""
import ast
import collections
import gzip
import importlib.util
import logging
import os
import re
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
TRACES = os.path.join(ROOT, 'traces')
K1 = os.path.dirname(ROOT)
MERGED = os.environ.get('E2E_WRAPPER') or os.path.join(ROOT, 'prtouch_v1_wrapper.py')
sys.path.insert(0, HERE)
import fake_klippy as fk  # noqa: E402

CFG_FILES = [os.path.join(K1, 'printer.cfg'),
             os.path.join(K1, 'Helper-Script', 'nozzle-cleaning-fan-control.cfg')]

PRTOUCH_CMDS = ('write_swap_prtouch', 'read_swap_prtouch', 'deal_avgs_prtouch', 'read_pres_prtouch',
                'start_pres_prtouch', 'start_step_prtouch', 'manual_get_steps', 'manual_get_pres')
PRTOUCH_REPLIES = ('resault_write_swap_prtouch', 'result_read_swap_prtouch', 'result_deal_avgs_prtouch',
                   'result_manual_get_steps', 'resault_manual_get_pres')
TH_KEEP = ('get_position', 'set_position', 'wait_moves', 'dwell', 'manual_move')
STP_KEEP = ('get_name', 'get_step_dist')
POLL_DELAY = 'delay_s(0.01,)'

LINE = re.compile(r'^\s*([0-9.]+) (c\d+)  ( *)(\S+) ?(.*)$')
TRUNC = re.compile(r'\.\.\.\((\d+)\)$')
DUR = re.compile(r' \(\d+\.\d+s\)$')


# ----------------------------------------------------------------------------
# trace parsing
# ----------------------------------------------------------------------------
class Trace:
    def __init__(self, path):
        self.path = path
        self.events = []        # (kind, depth, text, lineno)
        self.schedule = []      # (k, s, handler, params)
        self.replies = collections.defaultdict(collections.deque)
        self.states = []        # per top-level call: [(attr, valtext, lineno)]
        self.top_names = []
        self.first_pos = None
        self.misc = []          # (kind, text) e.g. MODVAR / INFO for boot
        self._parse()

    def _parse(self):
        main = None
        k = s = 0
        with gzip.open(self.path, 'rt') as f:
            for lineno, raw in enumerate(f, 1):
                m = LINE.match(raw.rstrip('\n'))
                if not m:
                    continue
                _, ctx, indent, tag, msg = m.groups()
                msg = msg.rstrip()
                if main is None:
                    main = ctx
                depth = len(indent) // 2
                if ctx != main:
                    if tag == 'CALL' and msg.startswith('_handle_'):
                        name = msg.split('(', 1)[0]
                        args = ast.literal_eval(msg[len(name):])
                        self.schedule.append((k, s, name, args[0]))
                    continue
                if tag in ('MODVAR', 'INFO', 'DICT'):
                    self.misc.append((tag, msg))
                    continue
                if tag == 'CALL':
                    if msg.startswith('delay_s('):
                        s += 1
                    self.events.append(('CALL', depth, msg, lineno))
                elif tag == 'RET':
                    self.events.append(('RET', depth, DUR.sub('', msg), lineno))
                    if depth == 0:
                        self.top_names.append(msg.split(' ', 1)[0])
                        self.states.append([])
                elif tag == 'EXC':
                    self.events.append(('EXC', depth, DUR.sub('', msg), lineno))
                elif tag == 'STATE':
                    mm = re.match(r'after (\S+) (\S+) = (.*)$', msg)
                    if not self.states:
                        self.states.append([])
                        self.top_names.append(mm.group(1))
                    self.states[-1].append((mm.group(2), mm.group(3), lineno))
                elif tag == 'MCU>':
                    if msg.split(' ', 1)[0] in PRTOUCH_CMDS:
                        k += 1
                        s = 0
                        self.events.append(('MCU>', depth, msg, lineno))
                elif tag == 'MCU<':
                    d = ast.literal_eval(msg)
                    if d.get('#name') in PRTOUCH_REPLIES:
                        self.replies[d['#name']].append(d)
                        s += 1
                elif tag == 'TH':
                    meth = msg.split('(', 1)[0].split('.', 1)[1]
                    if meth == 'wait_moves':
                        s += 1
                    if meth in TH_KEEP:
                        self.events.append(('TH', depth, msg, lineno))
                        if meth == 'get_position' and self.first_pos is None:
                            self.first_pos = ast.literal_eval(msg.split(' -> ', 1)[1])
                elif tag == 'STP':
                    meth = msg.split('(', 1)[0].split('.', 1)[1]
                    if meth in STP_KEEP:
                        self.events.append(('STP', depth, msg, lineno))
                elif tag == 'GC':
                    if (msg.startswith('GCodeDispatch.respond_info(') and self.events
                            and self.events[-1][0] == 'CALL' and self.events[-1][2].startswith('print_msg(')):
                        # the wrapper's own console output (print_msg with is_print)
                        arg = ast.literal_eval(msg[len('GCodeDispatch.respond_info'):msg.rindex(' -> ')])[0]
                        self.events.append(('GCR', depth, 'respond_info(%r)' % arg, lineno))
                        continue
                    if not msg.startswith('GCodeDispatch.run_script_from_command('):
                        continue          # respond_info/raw: from bed_mesh/probe/macros, not the wrapper
                    script = ast.literal_eval(msg[len('GCodeDispatch.run_script_from_command'):msg.rindex(' -> ')])[0]
                    if script.startswith('\n'):
                        # a macro body (M106 -> SET_PIN, RESTORE_E_CURRENT, _BED_MESH_CALIBRATE):
                        # it and the toolhead calls it made (logged just before it) are klippy's
                        while self.events and self.events[-1][0] in ('TH', 'STP'):
                            self.events.pop()
                        continue
                    self.events.append(('GC', depth, msg, lineno))
                elif tag in ('CFG', 'MCUCFG', 'LOOKUP', 'LOOKUPQ', 'REGRESP'):
                    self.events.append((tag, depth, msg, lineno))
        self.main = main

    def calls(self, name):
        return [e for e in self.events if e[0] == 'CALL' and e[2].startswith(name + '(')]


# ----------------------------------------------------------------------------
# normalisation and comparison
# ----------------------------------------------------------------------------
def drop_polls(events):
    """Drop delay_s(0.01) polling calls (pure timing) with their RETs."""
    out, stack = [], []
    for ev in events:
        kind, depth, text = ev[0], ev[1], ev[2]
        if kind == 'CALL':
            poll = text == POLL_DELAY
            stack.append(poll)
            if not poll:
                out.append(ev)
        elif kind in ('RET', 'EXC'):
            poll = stack.pop() if stack else False
            if not poll:
                out.append(ev)
        else:
            out.append(ev)
    return out


NUM = re.compile(r'-?\d+(?:\.\d+)?(?:e[-+]?\d+)?')


def _tok(s):
    parts, last = [], 0
    for m in NUM.finditer(s):
        parts.append(('t', s[last:m.start()]))
        parts.append(('n', m.group()))
        last = m.end()
    parts.append(('t', s[last:]))
    return parts


MATCH = collections.Counter()
TOL_EXAMPLES = []


def text_eq(exp, got):
    """exp (trace) vs got (fake). Exact, else numbers within 1e-9 relative
    (same int/float spelling), else, for a tracer-truncated '...(N)' line,
    the same on the kept prefix."""
    if exp == got:
        MATCH['exact'] += 1
        return True
    m = TRUNC.search(exp)
    if m:
        pre = exp[:m.start()]
        if got.startswith(pre):
            MATCH['truncated-prefix'] += 1
            return True
        ok = _tol_eq(pre, got[:len(pre)], drop_last=True)
        if ok:
            MATCH['truncated-prefix+tolerance'] += 1
            TOL_EXAMPLES.append((exp, got))
        return ok
    ok = _tol_eq(exp, got)
    if ok:
        MATCH['tolerance'] += 1
        TOL_EXAMPLES.append((exp, got))
    return ok


def _tol_eq(a, b, drop_last=False):
    ta, tb = _tok(a), _tok(b)
    if drop_last:
        n = min(len(ta), len(tb)) - 2
        if n < 1:
            return False
        ta, tb = ta[:n], tb[:n]
    if len(ta) != len(tb):
        return False
    for (ka, va), (kb, vb) in zip(ta, tb):
        if ka != kb:
            return False
        if ka == 't':
            if va != vb:
                return False
            continue
        if ('.' in va) != ('.' in vb) or ('e' in va) != ('e' in vb):
            return False
        fa, fb = float(va), float(vb)
        if abs(fa - fb) > 1e-9 * max(1.0, abs(fa), abs(fb)):
            return False
    return True


def first_divergence(exp, got):
    n = min(len(exp), len(got))
    for i in range(n):
        e, g = exp[i], got[i]
        if e[0] != g[0] or e[1] != g[1] or not text_eq(fk.norm_obj(e[2]), fk.norm_obj(g[2])):
            return i
    if len(exp) != len(got):
        return n
    return None


def poll_runs(events):
    """Each polling loop = a maximal run of delay_s(0.01) calls under the same
    caller with nothing but their RETs in between: (caller, count, lineno)."""
    runs, stack, cur = [], [], None
    for ev in events:
        kind, depth, text = ev[0], ev[1], ev[2]
        if kind == 'CALL' and text == POLL_DELAY:
            owner = stack[-1] if stack else '?'
            if cur and cur[0] == owner and cur[3] == depth:
                cur[1] += 1
            else:
                if cur:
                    runs.append(tuple(cur[:3]))
                cur = [owner, 1, ev[3] if len(ev) > 3 else 0, depth]
            stack.append(None)
            continue
        if kind == 'CALL':
            if cur:
                runs.append(tuple(cur[:3]))
                cur = None
            stack.append(text.split('(', 1)[0])
        elif kind in ('RET', 'EXC'):
            top = stack.pop() if stack else None
            if top is not None and cur:
                runs.append(tuple(cur[:3]))
                cur = None
        elif cur and kind == 'MCU>':
            runs.append(tuple(cur[:3]))
            cur = None
    if cur:
        runs.append(tuple(cur[:3]))
    return runs


def short(t, n=230):
    return t if len(t) <= n else t[:n] + '...'


def show_divergence(exp, got, i, ctx=6):
    lines = ['  first divergence at event #%d' % i]
    for j in range(max(0, i - ctx), i):
        e = exp[j]
        lines.append('     ok  L%-6d %s %s%s' % (e[3], e[0], '  ' * e[1], short(e[2])))
    if i < len(exp):
        e = exp[i]
        lines.append('  > trace L%-6d %s %s%s' % (e[3], e[0], '  ' * e[1], short(e[2], 600)))
    else:
        lines.append('  > trace: <end of trace>')
    if i < len(got):
        g = got[i]
        lines.append('  > fake         %s %s%s' % (g[0], '  ' * g[1], short(g[2], 600)))
    else:
        lines.append('  > fake: <end of run>')
    for j in range(i + 1, min(len(exp), i + 4)):
        e = exp[j]
        lines.append('   trace L%-6d %s %s%s' % (e[3], e[0], '  ' * e[1], short(e[2])))
    for j in range(i + 1, min(len(got), i + 4)):
        g = got[j]
        lines.append('   fake          %s %s%s' % (g[0], '  ' * g[1], short(g[2])))
    return '\n'.join(lines)


# ----------------------------------------------------------------------------
# environment / object construction
# ----------------------------------------------------------------------------
def load_module(env):
    for k in [k for k in sys.modules if k == 'extras' or k.startswith('extras.')]:
        del sys.modules[k]
    pkg = types.ModuleType('extras')
    pkg.__path__ = [ROOT]
    sys.modules['extras'] = pkg
    probe = types.ModuleType('extras.probe')
    sys.modules['extras.probe'] = probe
    pkg.probe = probe
    spec = importlib.util.spec_from_file_location('extras.prtouch_v1_wrapper', MERGED)
    mod = importlib.util.module_from_spec(spec)
    sys.modules['extras.prtouch_v1_wrapper'] = mod
    spec.loader.exec_module(mod)
    mod.time = env.fake_time
    mod.random = env.fake_random
    return mod


def snapshot(obj):
    out = {}
    for k, v in vars(obj).items():
        if callable(v) and not isinstance(v, (list, dict, tuple)):
            continue
        out[k] = fk.norm_obj(repr(v))
    return out


def build(sections, boot_only=False):
    """Boot path: __init__ with the fake config, MCU config callbacks
    (_build_step_config, _build_pres_config), klippy:ready (prtouch_v2_fan's
    set_fan_speed hijack)."""
    env = fk.Env(sections)
    mod = load_module(env)
    Traced = fk.trace_class(mod.PRTouchEndstopWrapper, env.rec)
    env.mod, env.Traced = mod, Traced
    snaps = []
    env.rec.top_hook = lambda name, inst: snaps.append((name, snapshot(inst)))
    obj = Traced(env.config())
    env.printer.add_object('probe', fk.Obj(mcu_probe=obj, name='probe'))
    for mcu in (env.mcu, env.leveling_mcu):
        for cb in mcu.config_callbacks:
            cb()
    # klippy:ready -> prtouch_v2_fan.PRTouchFan.hijack_set_fan_speed
    max_speed = float(sections.get('prtouch_v2_fan', {}).get('max_speed', 0.4))
    # hijack() saves the (traced) bound method as set_fan_speed__ and installs an
    # untraced clamp in front of it, so the traced CALL set_fan_speed carries the
    # clamped speed (trace_NCLR: the wrapper asks for 1.0, the trace shows 0.5).
    obj.set_fan_speed__ = obj.set_fan_speed

    def set_fan_speed(self, fan_name='None', fan_spd=0.0):
        if fan_name == 'fan' and fan_spd > max_speed:
            fan_spd = max_speed
        return self.set_fan_speed__(fan_name, fan_spd)
    obj.set_fan_speed = types.MethodType(set_fan_speed, obj)
    env.boot_snaps = snaps
    return env, obj


# ----------------------------------------------------------------------------
# scenario plans derived from the trace (environment behaviour only)
# ----------------------------------------------------------------------------
def heater_plan(tr):
    """Per heater: number of delay_s(0.1) polls inside each traced
    set_*_temps(wait=True) call, in order."""
    plans = {'set_hot_temps': collections.deque(), 'set_bed_temps': collections.deque()}
    ev = tr.events
    for i, e in enumerate(ev):
        if e[0] != 'CALL':
            continue
        name = e[2].split('(', 1)[0]
        if name not in plans:
            continue
        txt = e[2]
        if "'wait': True" not in txt and not re.match(r'\w+\([^,]+, True', txt):
            continue
        n = 0
        for f in ev[i + 1:]:
            if f[1] <= e[1]:
                break
            if f[0] == 'CALL' and f[1] == e[1] + 1 and f[2] == 'delay_s(0.1,)':
                n += 1
        plans[name].append(n)
    return plans


def shake_plan(tr):
    """len(toolhead.move_queue.queue) per evaluation in shake_motor's
    `while len(queue) > 4: delay_s(0.01)`: 5 for every traced poll, then 0."""
    out = []
    ev = tr.events
    for i, e in enumerate(ev):
        if e[0] == 'CALL' and e[2].startswith('shake_motor('):
            g, n, counting = 0, 0, False
            for f in ev[i + 1:]:
                if f[1] <= e[1]:
                    break
                if f[1] != e[1] + 1:
                    continue
                if f[0] == 'GC' and "('G1 X" in f[2]:
                    if counting:
                        out.extend([5] * n + [0])
                        counting = False
                    g += 1
                    if g % 4 == 0:
                        counting, n = True, 0
                elif f[0] == 'CALL' and f[2] == POLL_DELAY and counting:
                    n += 1
                elif f[0] == 'CALL' and counting:
                    out.extend([5] * n + [0])
                    counting = False
    return out


def call_args(text):
    name = text.split('(', 1)[0]
    rest = text[len(name):]
    kw = {}
    if rest.endswith('}') and ') {' in rest:
        rest, kws = rest.split(') {', 1)
        rest += ')'
        kw = ast.literal_eval('{' + kws)
    return ast.literal_eval(fk.norm_obj(rest).replace('<obj>', 'None')), kw


def g28_random_plan(tr, env, obj):
    """run_G28_Z: now_pos_sta0 = home +/- uniform(4, 8) with the sign from
    time parity, pos2 = home + uniform(-1, 1). Recover both from the traced
    move() calls."""
    rg = tr.calls('run_G28_Z')[0]
    d = rg[1] + 1
    moves = [e for e in tr.events if e[0] == 'CALL' and e[1] == d and e[2].startswith('move(')
             and e[3] > rg[3]]
    sta0 = call_args(moves[0][2])[0][0]
    hx, hy = obj.home_x, obj.home_y
    env.fake_time.parity_plan = {1000: 0 if sta0[0] >= hx else 1, 100: 0 if sta0[1] >= hy else 1}
    plan = [abs(sta0[0] - hx), abs(sta0[1] - hy)]
    pos2 = None
    for e in moves[1:]:
        args = call_args(e[2])[0]
        if len(args[0]) == 4 and args[0][2] == obj.best_above_z * 2 and args[1] == obj.rdy_xy_spd / 5:
            pos2 = args[0]
            break
    plan += [pos2[0] - hx, pos2[1] - hy] if pos2 else [0.0, 0.0]
    env.fake_random.plan.extend(plan)


def nclr_random_plan(tr, env, obj):
    cn = tr.calls('clear_nozzle')[0]
    mv = [e for e in tr.events if e[0] == 'CALL' and e[1] == cn[1] + 1 and e[2].startswith('move(')
          and e[3] > cn[3]][0]
    src = call_args(mv[2])[0][0]
    env.fake_random.plan.extend([src[0] - obj.clr_noz_start_x, src[1] - obj.clr_noz_start_y])


def mesh_from_trace(tr, env):
    """The bed mesh a G29 run left behind: bed_mesh_post_proc's 'After=' list
    (25 corrected points incl. the appended last point)."""
    for e in tr.events:
        if e[0] == 'CALL' and e[2].startswith("print_msg('BED_MESH_POST_PROC', 'After="):
            msg = call_args(e[2])[0][1]
            pts = ast.literal_eval(msg[len('After='):])
            bm = env.bed_mesh
            return fk.zmesh_from_results(pts, bm.bmc.mesh_config['x_count'], bm.bmc.mesh_config['y_count'],
                                         bm.bmc.mesh_min, bm.bmc.mesh_max)
    return None


def run_mesh(env, obj):
    """probe.py ProbePointsHelper.start_probe + PrinterProbe.run_probe/_probe +
    homing.py probing_move (prtouch_v2 branch) + bed_mesh finalize, as the
    stock K1 klippy runs them for BED_MESH_CALIBRATE (sample_count 1)."""
    bm, th = env.bed_mesh, env.toolhead
    bm.set_mesh(None)                                   # cmd_BED_MESH_CALIBRATE
    points = bm.probe_points()
    results = []
    bm.bmc.probe_helper.results = results
    obj.multi_probe_begin()                             # probe.multi_probe_begin()
    while True:
        if len(results) >= len(points):                 # _move_next: done
            th.get_last_move_time()
            bm.finalize(results)
            break
        obj.run_to_next(list(points[len(results)]))     # _move_next (prtouch_v2)
        th.get_position()                               # run_probe: probexy
        th.get_position()                               # _probe: pos
        epos = obj.run_G29_Z()                          # probing_move
        pos = epos[:3]
        results.append([sum([p[i] for p in [pos]]) / 1.0 for i in range(3)])   # _calc_mean
    obj.multi_probe_end()


# ----------------------------------------------------------------------------
# scenarios
# ----------------------------------------------------------------------------
ORDER = ['G28', 'G29', 'AHZ', 'CBM', 'NCLR']
FILES = {'boot': 'prtouch_trace.boot.log.gz', 'G28': 'trace_G28.log.gz', 'G29': 'trace_G29.log.gz',
         'AHZ': 'trace_AHZ.log.gz', 'CBM': 'trace_CBM.log.gz', 'NCLR': 'trace_NCLR.log.gz',
         'RP': 'trace_READ_PRES.log.gz'}
_TRACES = {}


def trace(name):
    if name not in _TRACES:
        _TRACES[name] = Trace(os.path.join(TRACES, FILES[name]))
    return _TRACES[name]


def literal(text):
    if TRUNC.search(text) or '<' in text:
        return None, False
    try:
        return ast.literal_eval(text), True
    except Exception:
        return None, False


def apply_prior_state(obj, env, name):
    """Instance state = boot + every STATE change recorded by the traces that
    ran before this one (they are slices of one klippy session)."""
    applied = 0
    if name not in ORDER:          # standalone slice right after a fresh boot
        return applied
    for prev in ORDER[:ORDER.index(name)]:
        for st in trace(prev).states:
            for attr, val, _ in st:
                v, ok = literal(val)
                if ok:
                    setattr(obj, attr, v)
                    applied += 1
    # No slice lists ver_pres as changed although boot left it 'V0.0': a
    # leveling_mcu debug_prtouch (sets ver_pres) arrived between the boot slice
    # and the G28 slice.
    obj.ver_pres = 'V3.07'
    if getattr(obj, 'has_save_sys_acc', False):
        th = env.toolhead
        th.max_velocity, th.max_accel = obj.run_max_velocity, obj.run_max_accel
        th.kin.max_z_velocity, th.kin.max_z_accel = obj.run_max_z_velocity, obj.run_max_z_accel
    return applied


class Result:
    def __init__(self, name):
        self.name = name
        self.ok = True
        self.lines = []
        self.notes = []

    def fail(self, text):
        self.ok = False
        self.lines.append(text)


FAN_HIJACK = ('set_fan_speed', 'set_fan_speed__')


def compare_states(res, tr, snaps, initial):
    prev = initial
    n = min(len(tr.states), len(snaps))
    if len(tr.states) != len(snaps):
        res.fail('  STATE: trace has %d top-level calls, fake %d' % (len(tr.states), len(snaps)))
    for i in range(n):
        name, snap = snaps[i]
        traced = tr.states[i]
        tnames = set()
        for attr, val, lineno in traced:
            tnames.add(attr)
            if attr in FAN_HIJACK and not snap.get(attr):
                continue          # set on the instance by prtouch_v2_fan at klippy:ready
            got = snap.get(attr, '<missing>')
            if not text_eq(fk.norm_obj(val), got):
                res.fail('  STATE after %s #%d (trace L%d): %s\n      trace: %s\n      fake : %s'
                         % (name, i, lineno, attr, short(val, 300), short(got, 300)))
        changed = {k for k, v in snap.items() if prev.get(k) != v}
        extra = sorted(changed - tnames)
        # an attribute whose value is unchanged on the printer but differs in the
        # fake's starting state shows up here too
        if extra:
            res.fail('  STATE after %s #%d: fake changed attributes the trace did not: %s'
                     % (name, i, ', '.join(extra)))
        prev = snap


def run_boot(verbose):
    res = Result('boot')
    tr = trace('boot')
    sections = fk.parse_cfg_files(CFG_FILES)
    env, obj = build(sections)
    got = env.rec.events
    exp = tr.events
    i = first_divergence(exp, got)
    if i is not None:
        res.fail(show_divergence(exp, got, i))
    # STATE after __init__ is a full dump; the two _build_* dumps are changes
    if len(env.boot_snaps) != len(tr.states):
        res.fail('  STATE: trace has %d boot calls, fake %d' % (len(tr.states), len(env.boot_snaps)))
    for (name, snap), st in zip(env.boot_snaps, tr.states):
        for attr, val, lineno in st:
            got_v = snap.get(attr, '<missing>')
            if not text_eq(fk.norm_obj(val), got_v):
                res.fail('  STATE after %s (L%d): %s trace=%s fake=%s' % (name, lineno, attr, val, got_v))
    full = dict((a, v) for a, v, _ in tr.states[0])
    extra = sorted(set(env.boot_snaps[0][1]) - set(full))
    if extra:
        res.fail('  STATE after __init__: attributes not in the trace: %s' % ', '.join(extra))
    # module globals (MODVAR) and public names
    mod = env.mod
    nmod = 0
    for tag, msg in tr.misc:
        if tag == 'MODVAR' and msg.startswith('extras.prtouch_v1_wrapper.'):
            attr, val = msg[len('extras.prtouch_v1_wrapper.'):].split(' = ', 1)
            if attr in ('time', 'random'):
                continue      # replaced by the harness fakes
            got_v = fk.norm_obj(repr(getattr(mod, attr, '<missing>')))
            want = fk.norm_obj(val)
            if attr in ('datetime', 'logging', 'math', 'os', 'probe', 'random', 'socket', 'threading', 'time'):
                ok = isinstance(getattr(mod, attr, None), types.ModuleType)
            else:
                ok = text_eq(want, got_v)
            nmod += 1
            if not ok:
                res.fail('  MODVAR %s trace=%s fake=%s' % (attr, val, got_v))
        if tag == 'INFO' and msg.startswith('extras.prtouch_v1_wrapper attrs: '):
            want = ast.literal_eval(msg.split(': ', 1)[1])
            have = sorted(a for a in dir(mod) if not a.startswith('__'))
            if have != want:
                res.fail('  module names differ: missing %s extra %s' % (
                    sorted(set(want) - set(have)), sorted(set(have) - set(want))))
    res.notes.append('%d boot events, %d MODVARs compared' % (len(exp), nmod))
    return res


def run_scenario(name, verbose):
    res = Result(name)
    tr = trace(name)
    sections = fk.parse_cfg_files(CFG_FILES)
    env, obj = build(sections)
    applied = apply_prior_state(obj, env, name)
    # environment
    env.rec.events = []
    env.inj.schedule = list(tr.schedule)
    env.replies = {k: collections.deque(v) for k, v in tr.replies.items()}
    if tr.first_pos:
        env.toolhead.pos = list(tr.first_pos)
    bm = env.bed_mesh
    bm.pmgr.profiles['default'] = {'mesh': fk.FakeZMesh([[0.0, 0.0], [0.0, 0.0]], 5, 5, 215, 215)}
    if name in ('AHZ', 'CBM'):
        bm.z_mesh = mesh_from_trace(trace('G29'), env)
    elif name == 'NCLR':
        bm.z_mesh = mesh_from_trace(trace('CBM'), env)
    env.gcode.macro_handlers['BED_MESH_CALIBRATE'] = lambda script: run_mesh(env, obj)
    env.toolhead.move_queue.queue.plan.extend(shake_plan(tr))
    hp = heater_plan(tr)
    Traced = env.Traced
    for meth, heater in (('set_hot_temps', env.heater_hot), ('set_bed_temps', env.heater_bed)):
        traced_fn = getattr(Traced, meth)

        def wrapped(self, *a, _fn=traced_fn, _h=heater, _q=hp[meth], **kw):
            wait = kw.get('wait', a[1] if len(a) > 1 else False)
            if wait:
                _h.begin_wait(_q.popleft() if _q else 0)
            return _fn(self, *a, **kw)
        setattr(Traced, meth, wrapped)
    if name in ('G28', 'AHZ'):
        g28_random_plan(tr, env, obj)
    elif name == 'NCLR':
        nclr_random_plan(tr, env, obj)

    snaps = []
    env.rec.top_hook = lambda n, inst: snaps.append((n, snapshot(inst)))
    initial = snapshot(obj)
    err = None
    try:
        if name == 'G28':
            obj.run_G28_Z()                                    # homing.py: mcu_probe.run_G28_Z()
        elif name == 'G29':
            run_mesh(env, obj)
        elif name == 'AHZ':
            obj.cmd_ACCURATE_HOME_Z(fk.FakeGCmd({}))
        elif name == 'CBM':
            obj.cmd_CHECK_BED_MESH(fk.FakeGCmd({}))
        elif name == 'RP':
            obj.cmd_READ_PRES(fk.FakeGCmd({'C': 10}))
        elif name == 'NCLR':
            obj.cmd_NOZZLE_CLEAR(fk.FakeGCmd({'HOT_MIN_TEMP': 180, 'HOT_MAX_TEMP': 220, 'BED_MAX_TEMP': 50}))
    except Exception as e:  # noqa: BLE001
        import traceback
        err = traceback.format_exc()
    exp = drop_polls(tr.events)
    got = drop_polls(env.rec.events)
    i = first_divergence(exp, got)
    if i is not None:
        res.fail(show_divergence(exp, got, i))
    if err:
        res.fail('  exception in the fake run:\n' + '\n'.join('    ' + l for l in err.rstrip().splitlines()[-8:]))
    compare_states(res, tr, snaps, initial)
    # timing-only information
    tpolls = sum(1 for e in tr.events if e[0] == 'CALL' and e[2] == POLL_DELAY)
    fpolls = sum(1 for e in env.rec.events if e[0] == 'CALL' and e[2] == POLL_DELAY)
    truns, fruns = poll_runs(tr.events), poll_runs(env.rec.events)
    if len(truns) != len(fruns):
        res.fail('  polling loops: trace %d, fake %d' % (len(truns), len(fruns)))
    diffs = [(t, f) for t, f in zip(truns, fruns) if t[1] != f[1]]
    if diffs:
        byloc = collections.Counter('%s %+d' % (t[0], f[1] - t[1]) for t, f in diffs)
        res.notes.append('polling loops with a different poll count (loop owner, fake-trace): %s'
                         % ', '.join('%s x%d' % kv for kv in sorted(byloc.items())))
    if verbose:
        for t, f in diffs:
            res.notes.append('   poll run %s: trace %d (from L%d), fake %d' % (t[0], t[1], t[2], f[1]))
    res.notes.append('%d events compared (%d trace / %d fake), %d STATE values applied from earlier traces'
                     % (min(len(exp), len(got)), len(exp), len(got), applied))
    res.notes.append('delay_s(0.01) polls: trace %d, fake %d; handlers delivered %d/%d'
                     % (tpolls, fpolls, env.inj.delivered, len(tr.schedule)))
    left = {k: len(v) for k, v in env.replies.items() if v}
    if left:
        res.notes.append('unused traced MCU replies: %s' % left)
    if env.fake_random.out_of_range:
        res.fail('  random.uniform(a, b) asked for a range that cannot produce the traced value: %s'
                 % env.fake_random.out_of_range)
    if env.fake_random.used:
        res.notes.append('random.uniform calls (a, b, value from trace): %s' % ', '.join(
            '(%g, %g, %.6f)' % u for u in env.fake_random.used))
    if env.fake_random.unplanned:
        res.notes.append('unplanned random.uniform calls: %s' % env.fake_random.unplanned)
    for n in env.notes:
        res.notes.append(n)
    if env.configfile_sets:
        res.notes.append('configfile.set: %s' % env.configfile_sets)
    return res


def main(argv):
    logging.disable(logging.CRITICAL)
    verbose = '-v' in argv
    names = [a for a in argv if not a.startswith('-')] or ['boot'] + ORDER + ['RP']
    allok = True
    print('python %s' % sys.version.split()[0])
    for n in names:
        res = run_boot(verbose) if n == 'boot' else run_scenario(n, verbose)
        allok &= res.ok
        print('%-5s %s' % (n, 'PASS' if res.ok else 'FAIL'))
        for line in res.lines:
            print(line)
        for line in res.notes:
            print('      note: ' + line)
        print('      matches: %s' % dict(MATCH))
        if verbose:
            for e, g in TOL_EXAMPLES[:5]:
                print('      tolerance match:\n        trace %s\n        fake  %s' % (short(e, 300), short(g, 300)))
        MATCH.clear()
        del TOL_EXAMPLES[:]
    return 0 if allok else 1


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
