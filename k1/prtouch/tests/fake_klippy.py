"""Fake Klipper environment for the offline end-to-end replay of
k1/prtouch/prtouch_v1_wrapper.py (driven by run_e2e.py).

Only the Klipper side is faked. Everything the wrapper touches records the
same kind of line the on-printer tracer (extras.prtouch_trace) wrote, so the
two event streams can be diffed:

  CALL/RET  wrapper methods (installed by trace_class() on a subclass)
  MCU>      CommandWrapper / CommandQueryWrapper .send()
  TH        ToolHead calls the wrapper (or a faked klippy module) makes
  STP       MCU_stepper.get_name / get_step_dist
  GC        GCodeDispatch.run_script_from_command (logged on return, like
            the tracer, so nested scripts come first)
  CFG/MCUCFG/LOOKUP/LOOKUPQ/REGRESP   boot-time lines

Like the real tracer, non-CALL/RET lines are only recorded while a wrapper
method is active (depth > 0) and only on the main context; MCU response
handlers are delivered on a 'handler' context and are not recorded.

MCU responses are injected by Injector, keyed on the command sequence: every
traced handler call carries (k, s) = (number of prtouch MCU> sends so far,
number of sync points since that send). Sync points are reactor.pause()
(delay_s), ToolHead.wait_moves() and query sends. A handler keyed (k, 0) is
delivered right after send k; one keyed (k, s>0) at the end of sync point s
after send k. Anything still pending when send k+1 happens is flushed first.
"""
import collections
import re
import sys

PAT_OBJ = re.compile(r'<[^<>]*? at 0x[0-9a-fA-F]+>')


def norm_obj(text):
    return PAT_OBJ.sub('<obj>', text)


def fmt_call(name, args, kwargs):
    s = '%s%r' % (name, tuple(args))
    if kwargs:
        s += ' %r' % (dict(kwargs),)
    return s


# ----------------------------------------------------------------------------
# recording
# ----------------------------------------------------------------------------
class Recorder:
    def __init__(self):
        self.events = []          # (kind, depth, text)
        self.depth = 0
        self.ctx = 'main'
        self.top_hook = None      # called after every depth-0 RET (STATE snapshot)

    def active(self):
        return self.ctx == 'main' and self.depth > 0

    def rec(self, kind, text, force=False):
        if self.ctx != 'main':
            return
        if self.depth > 0 or force:
            self.events.append((kind, self.depth, text))

    def call(self, name, args, kwargs):
        if self.ctx != 'main':
            return
        self.events.append(('CALL', self.depth, fmt_call(name, args, kwargs)))
        self.depth += 1

    def ret(self, name, value, inst=None):
        if self.ctx != 'main':
            return
        self.depth -= 1
        self.events.append(('RET', self.depth, '%s -> %r' % (name, value)))
        if self.depth == 0 and self.top_hook:
            self.top_hook(name, inst)

    def exc(self, name, e, inst=None):
        if self.ctx != 'main':
            return
        self.depth -= 1
        self.events.append(('EXC', self.depth, '%s -> %s: %s' % (name, type(e).__name__, e)))
        if self.depth == 0 and self.top_hook:
            self.top_hook(name, inst)


def trace_class(cls, rec, skip=()):
    """Subclass of cls whose every method records CALL/RET like the tracer."""
    ns = {}
    for name, fn in list(vars(cls).items()):
        if not callable(fn) or isinstance(fn, type) or name in skip:
            continue
        if name.startswith('__') and name != '__init__':
            continue
        ns[name] = _traced(name, fn, rec)
    return type(cls.__name__, (cls,), ns)


def _traced(name, fn, rec):
    def traced(self, *args, **kwargs):
        rec.call(name, args, kwargs)
        try:
            ret = fn(self, *args, **kwargs)
        except BaseException as e:
            rec.exc(name, e, self)
            raise
        rec.ret(name, ret, self)
        return ret
    traced.__name__ = name
    traced.__wrapped__ = fn
    return traced


# ----------------------------------------------------------------------------
# response injection / reactor / clock
# ----------------------------------------------------------------------------
class Injector:
    def __init__(self, rec):
        self.rec = rec
        self.schedule = []        # [(k, s, handler_name, params)]
        self.pos = 0
        self.k = 0
        self.s = 0
        self.handlers = {}        # name -> callable (registered via register_response)
        self.delivered = 0

    def _deliver(self, pred):
        while self.pos < len(self.schedule):
            k, s, name, params = self.schedule[self.pos]
            if not pred(k, s):
                break
            self.pos += 1
            cb = self.handlers[name]
            saved = self.rec.ctx
            self.rec.ctx = 'handler'
            try:
                cb(dict(params))
            finally:
                self.rec.ctx = saved
            self.delivered += 1

    def on_send(self):
        cur = self.k
        self._deliver(lambda k, s: k <= cur)
        self.k += 1
        self.s = 0
        cur = self.k
        self._deliver(lambda k, s: k < cur or (k == cur and s == 0))

    def sync(self):
        self.s += 1
        cur, cs = self.k, self.s
        self._deliver(lambda k, s: k < cur or (k == cur and s <= cs))


class FakeReactor:
    def __init__(self, inj):
        self.inj = inj
        self.now = 1000.0
        self.pauses = 0

    def monotonic(self):
        return self.now

    def pause(self, waketime):
        self.pauses += 1
        self.inj.sync()
        if waketime > self.now:
            self.now = waketime
        return self.now

    def register_timer(self, cb, waketime=None):
        return cb

    def completion(self):
        return None


class FakeTime:
    """Stands in for the `time` module inside the wrapper's globals.

    time() is the fake reactor clock plus an epoch. run_G28_Z picks the sign
    of its random X/Y offsets from int(time.time()*1000) % 2 and
    int(time.time()*100) % 2; parity_plan maps the multiplier (1000 / 100)
    to the parity the trace implies, applied when the calling source line
    contains that expression."""

    def __init__(self, reactor):
        self.reactor = reactor
        self.epoch = 1700000000.0
        self.parity_plan = {}

    def time(self):
        t = self.epoch + self.reactor.now
        f = sys._getframe(1)
        line = _source_line(f)
        for mult, parity in self.parity_plan.items():
            if ('* %d) %% 2' % mult) in line:
                v = int(t * mult)
                if v % 2 != parity:
                    v += 1
                return (v + 0.5) / mult
        return t

    def strftime(self, fmt, *a):
        return '00:00:00 '

    def sleep(self, s):
        pass


_LINES = {}


def _source_line(frame):
    fn = frame.f_code.co_filename
    if fn not in _LINES:
        try:
            with open(fn) as f:
                _LINES[fn] = f.read().splitlines()
        except OSError:
            _LINES[fn] = []
    lines = _LINES[fn]
    n = frame.f_lineno
    return lines[n - 1] if 0 < n <= len(lines) else ''


class FakeRandom:
    """Stands in for `random`: uniform() returns planned values (derived from
    the trace by the scenario), seed() is ignored."""

    def __init__(self):
        self.plan = collections.deque()
        self.unplanned = []
        self.out_of_range = []
        self.used = []

    def seed(self, *a, **k):
        pass

    def uniform(self, a, b):
        if self.plan:
            v = self.plan.popleft()
            if not min(a, b) <= v <= max(a, b):
                self.out_of_range.append((a, b, v))
            self.used.append((a, b, v))
            return v
        self.unplanned.append((a, b))
        return (a + b) / 2.0

    def random(self):
        return 0.5


# ----------------------------------------------------------------------------
# config
# ----------------------------------------------------------------------------
class ConfigError(Exception):
    pass


def parse_cfg_files(paths):
    """Klipper-style config: sections, 'key: value' / 'key = value', '#' and
    ';' comments, the '#*#' SAVE_CONFIG block, multi-line values. Later
    duplicates win."""
    sections = collections.OrderedDict()
    for path in paths:
        cur = None
        last_key = None
        with open(path) as f:
            for raw in f:
                line = raw.rstrip('\n')
                if line.startswith('#*#'):
                    line = line[3:]
                    if line.startswith(' '):
                        line = line[1:]
                else:
                    line = re.split(r'\s[#;]', line, 1)[0] if not line.lstrip().startswith(('#', ';')) else ''
                if not line.strip():
                    continue
                m = re.match(r'^\[([^\]]+)\]', line)
                if m:
                    cur = m.group(1).strip()
                    sections.setdefault(cur, collections.OrderedDict())
                    last_key = None
                    continue
                if cur is None:
                    continue
                if line[0] in ' \t' and last_key is not None:
                    sections[cur][last_key] += '\n' + line.strip()
                    continue
                m = re.match(r'^([^:=]+?)\s*[:=]\s*(.*)$', line)
                if m:
                    last_key = m.group(1).strip().lower()
                    sections[cur][last_key] = m.group(2).strip()
    return sections


_SENTINEL = object()


class FakeConfig:
    def __init__(self, printer, sections, name, rec):
        self.printer = printer
        self.sections = sections
        self.section = name
        self.rec = rec

    def _log(self, meth, args, kwargs, ret):
        self.rec.events.append(('CFG', self.rec.depth, '[%s] %s -> %s' % (
            self.section, fmt_call(meth, args, kwargs), norm_obj(repr(ret)))))

    def get_name(self):
        return self.section

    def get_printer(self):
        self._log('get_printer', (), {}, self.printer)
        return self.printer

    def _raw(self, option, default):
        sec = self.sections.get(self.section, {})
        if option.lower() in sec:
            return True, sec[option.lower()]
        if default is _SENTINEL:
            raise ConfigError("Option '%s' in section '%s' must be specified" % (option, self.section))
        return False, default

    def _get(self, meth, conv, option, default=_SENTINEL, minval=None, maxval=None,
             above=None, below=None, _args=None, _kwargs=None):
        found, v = self._raw(option, default)
        if found:
            v = conv(v)
            if minval is not None and v < minval:
                raise ConfigError('%s below minimum' % option)
            if maxval is not None and v > maxval:
                raise ConfigError('%s above maximum' % option)
        self._log(meth, _args, _kwargs, v)
        return v

    def get(self, option, *args, **kwargs):
        default = args[0] if args else kwargs.get('default', _SENTINEL)
        return self._get('get', str, option, default, _args=(option,) + args, _kwargs=kwargs)

    def getint(self, option, *args, **kwargs):
        names = ['default', 'minval', 'maxval']
        kw = dict(zip(names, args))
        kw.update(kwargs)
        return self._get('getint', int, option, _args=(option,) + args, _kwargs=kwargs, **kw)

    def getfloat(self, option, *args, **kwargs):
        names = ['default', 'minval', 'maxval', 'above', 'below']
        kw = dict(zip(names, args))
        kw.update(kwargs)
        return self._get('getfloat', float, option, _args=(option,) + args, _kwargs=kwargs, **kw)

    def getboolean(self, option, *args, **kwargs):
        kw = dict(zip(['default'], args))
        kw.update(kwargs)

        def conv(s):
            s = s.strip().lower()
            if s in ('1', 'yes', 'true', 'on'):
                return True
            if s in ('0', 'no', 'false', 'off'):
                return False
            raise ConfigError('bad boolean %r' % s)
        return self._get('getboolean', conv, option, _args=(option,) + args, _kwargs=kwargs, **kw)

    def _getlist(self, meth, conv, option, args, kwargs):
        kw = dict(zip(['default'], args))
        kw.update({k: v for k, v in kwargs.items() if k == 'default'})
        return self._get(meth, lambda s: tuple(conv(p.strip()) for p in s.split(',') if p.strip()),
                         option, kw.get('default', _SENTINEL), _args=(option,) + args, _kwargs=kwargs)

    def getintlist(self, option, *args, **kwargs):
        return self._getlist('getintlist', int, option, args, kwargs)

    def getfloatlist(self, option, *args, **kwargs):
        return self._getlist('getfloatlist', float, option, args, kwargs)

    def has_section(self, name):
        ret = name in self.sections
        self._log('has_section', (name,), {}, ret)
        return ret

    def getsection(self, name):
        ret = FakeConfig(self.printer, self.sections, name, self.rec)
        self._log('getsection', (name,), {}, ret)
        return ret

    def get_prefix_sections(self, prefix):
        ret = [FakeConfig(self.printer, self.sections, s, self.rec)
               for s in self.sections if s.startswith(prefix)]
        self._log('get_prefix_sections', (prefix,), {}, ret)
        return ret


# ----------------------------------------------------------------------------
# MCU
# ----------------------------------------------------------------------------
class FakeCommand:
    def __init__(self, mcu, fmt, resp=None):
        self.mcu, self.fmt, self.resp = mcu, fmt, resp
        self.resp_name = resp.split()[0] if resp else None

    def send(self, data=(), minclock=0, reqclock=0):
        env = self.mcu.env
        env.inj.on_send()
        env.rec.rec('MCU>', '%s %r' % (self.fmt, list(data)))
        env.sends.append((self.fmt, list(data)))
        if self.resp is None:
            return None
        env.inj.sync()
        q = env.replies.get(self.resp_name)
        if not q:
            env.notes.append('no traced reply left for %s %r' % (self.fmt, list(data)))
            return None
        return dict(q.popleft())


class FakeMCU:
    def __init__(self, env, name, first_oid):
        self.env, self.name = env, name
        self._oid = first_oid
        self.config_callbacks = []

    def create_oid(self):
        o = self._oid
        self._oid += 1
        return o

    def register_config_callback(self, cb):
        self.config_callbacks.append(cb)

    def add_config_cmd(self, cmd, **kwargs):
        self.env.rec.events.append(('MCUCFG', self.env.rec.depth, '%s %s %r' % (self.name, cmd, kwargs)))

    def lookup_command(self, fmt, cq=None):
        self.env.rec.events.append(('LOOKUP', self.env.rec.depth, '%s %s' % (self.name, fmt)))
        return FakeCommand(self, fmt)

    def lookup_query_command(self, fmt, resp, oid=None, cq=None, is_async=False):
        kw = {'oid': oid}
        self.env.rec.events.append(('LOOKUPQ', self.env.rec.depth, '%s %s | %s %r' % (self.name, fmt, resp, kw)))
        return FakeCommand(self, fmt, resp)

    def register_response(self, cb, msg, oid=None):
        self.env.rec.events.append(('REGRESP', self.env.rec.depth, '%s %s oid=%s cb=%s' % (
            self.name, msg, oid, getattr(cb, '__name__', cb))))
        self.env.inj.handlers[cb.__name__] = cb

    def get_printer(self):
        return self.env.printer


class FakePins:
    def __init__(self, chips):
        self.chips = chips

    def parse_pin(self, desc, can_invert=False, can_pullup=False):
        d = desc.strip()
        pullup = invert = 0
        while d and d[0] in '^~!':
            if d[0] == '!':
                invert = 1
            else:
                pullup = 1
            d = d[1:].strip()
        chip_name, pin = ('mcu', d)
        if ':' in d:
            chip_name, pin = [s.strip() for s in d.split(':', 1)]
        return {'chip': self.chips[chip_name], 'chip_name': chip_name, 'pin': pin,
                'invert': invert, 'pullup': pullup}


# ----------------------------------------------------------------------------
# motion
# ----------------------------------------------------------------------------
class FakeStepper:
    def __init__(self, env, name, step_dist):
        self.env, self.name, self.step_dist = env, name, step_dist

    def get_name(self, short=False):
        self.env.rec.rec('STP', 'MCU_stepper.get_name() -> %r' % self.name)
        return self.name

    def get_step_dist(self):
        self.env.rec.rec('STP', 'MCU_stepper.get_step_dist() -> %r' % self.step_dist)
        return self.step_dist

    def is_active_axis(self, axis):
        return self.name == 'stepper_' + axis


class FakeKin:
    def __init__(self, env):
        self.steppers = [FakeStepper(env, 'stepper_x', 0.0125), FakeStepper(env, 'stepper_y', 0.0125),
                         FakeStepper(env, 'stepper_z', 0.00078125)]
        self.limits = [(0.0, 220.0), (0.0, 220.0), (-5.0, 255.0)]
        self.max_z_velocity = 20.0
        self.max_z_accel = 300.0

    def get_steppers(self):
        return list(self.steppers)


class PlannedQueue:
    """toolhead.move_queue.queue: len() follows the traced number of
    `while len(queue) > 4: delay_s(0.01)` iterations in shake_motor."""

    def __init__(self):
        self.plan = collections.deque()

    def __len__(self):
        return self.plan.popleft() if self.plan else 0


class FakeToolhead:
    Z_BASE = -0.010000000000000004   # gcode_move base Z: G1 Zv -> toolhead v + this (trace-exact)

    def __init__(self, env):
        self.env = env
        self.kin = FakeKin(env)
        self.pos = [0.0, 0.0, 0.0, 0.0]
        self.max_velocity = 800.0
        self.max_accel = 20000.0
        self.max_accel_to_decel = 20000.0
        self.square_corner_velocity = 5.0
        self.move_queue = type('MQ', (), {})()
        self.move_queue.queue = PlannedQueue()

    def _th(self, name, args, kwargs, ret):
        self.env.rec.rec('TH', 'ToolHead.%s -> %r' % (fmt_call(name, args, kwargs), ret))

    def get_position(self):
        ret = list(self.pos)
        self._th('get_position', (), {}, ret)
        return ret

    def set_position(self, newpos, homing_axes=()):
        kw = {'homing_axes': homing_axes} if homing_axes != () else {}
        self.pos = list(newpos)
        # toolhead:set_position event -> gcode_move.reset_last_position -> get_position
        self.get_position()
        self._th('set_position', (newpos,), kw, None)

    def wait_moves(self):
        self.env.inj.sync()
        self._th('wait_moves', (), {}, None)

    def dwell(self, delay):
        self._th('dwell', (delay,), {}, None)

    def manual_move(self, coord, speed):
        for i, c in enumerate(coord):
            if c is not None:
                self.pos[i] = c
        self._th('manual_move', (coord, speed), {}, None)

    def move(self, newpos, speed):
        self.pos = list(newpos)

    def get_last_move_time(self):
        return self.env.reactor.now

    def flush_step_generation(self):
        pass

    def get_kinematics(self):
        return self.kin


# ----------------------------------------------------------------------------
# heaters, bed mesh, misc objects
# ----------------------------------------------------------------------------
class FakeHeater:
    """smoothed_temp follows the trace: while a set_*_temps(wait=True) loop is
    active it reads 'far from target' for exactly the traced number of
    delay_s(0.1) iterations (begin_wait(n)), then 'at target'."""

    def __init__(self, name, idle=25.0):
        self.name = name
        self.target_temp = 0.0
        self.idle = idle
        self.wait_left = None

    def begin_wait(self, n):
        self.wait_left = n

    @property
    def smoothed_temp(self):
        if self.wait_left is None:
            return self.idle
        if self.wait_left > 0:
            self.wait_left -= 1
            return self.target_temp + 1000.0
        self.wait_left = None
        return self.target_temp


class FakeHeaters:
    def set_temperature(self, heater, temp, wait=False):
        heater.target_temp = temp


class FakeZMesh:
    def __init__(self, grid, min_x, min_y, max_x, max_y):
        self.grid = grid       # grid[row(y)][col(x)]
        self.min_x, self.min_y, self.max_x, self.max_y = min_x, min_y, max_x, max_y

    def calc_z(self, x, y):
        ny, nx = len(self.grid), len(self.grid[0])
        fx = (x - self.min_x) / (self.max_x - self.min_x) * (nx - 1)
        fy = (y - self.min_y) / (self.max_y - self.min_y) * (ny - 1)
        ix, iy = min(int(fx), nx - 2), min(int(fy), ny - 2)
        tx, ty = fx - ix, fy - iy
        z0 = self.grid[iy][ix] * (1 - tx) + self.grid[iy][ix + 1] * tx
        z1 = self.grid[iy + 1][ix] * (1 - tx) + self.grid[iy + 1][ix + 1] * tx
        return z0 * (1 - ty) + z1 * ty


def zmesh_from_results(results, x_cnt, y_cnt, mn, mx):
    grid = []
    for i in range(y_cnt):
        row = [p[2] for p in results[i * x_cnt:(i + 1) * x_cnt]]
        if i % 2 == 1:
            row = row[::-1]
        grid.append(row)
    return FakeZMesh(grid, mn[0], mn[1], mx[0], mx[1])


class FakeBedMesh:
    def __init__(self, env):
        self.env = env
        self.bmc = type('BMC', (), {})()
        self.bmc.mesh_min = (5.0, 5.0)
        self.bmc.mesh_max = (215.0, 215.0)
        self.bmc.mesh_config = {'x_count': 5, 'y_count': 5}
        self.bmc.probe_helper = type('PH', (), {})()
        self.bmc.probe_helper.results = []
        self.pmgr = type('PM', (), {})()
        self.pmgr.profiles = {}
        self.z_mesh = None

    def get_mesh(self):
        return self.z_mesh

    def set_mesh(self, mesh):
        self.z_mesh = mesh
        # bed_mesh.set_mesh -> gcode_move.reset_last_position -> toolhead.get_position
        self.env.toolhead.get_position()

    def probe_points(self):
        (mnx, mny), (mxx, mxy) = self.bmc.mesh_min, self.bmc.mesh_max
        xc, yc = self.bmc.mesh_config['x_count'], self.bmc.mesh_config['y_count']
        dx, dy = (mxx - mnx) / (xc - 1), (mxy - mny) / (yc - 1)
        pts = []
        for i in range(yc):
            y = mny + i * dy
            xs = [mnx + j * dx for j in range(xc)]
            if i % 2 == 1:
                xs = xs[::-1]
            pts.extend((x, y) for x in xs)
        return pts

    def finalize(self, results):
        mesh = zmesh_from_results(results, self.bmc.mesh_config['x_count'], self.bmc.mesh_config['y_count'],
                                  self.bmc.mesh_min, self.bmc.mesh_max)
        self.set_mesh(mesh)
        self.pmgr.profiles['default'] = {'mesh': mesh}


class FakeGCode:
    def __init__(self, env):
        self.env = env
        self.commands = {}
        self.scripts = []
        self.macro_handlers = {}

    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        self.commands[cmd] = (func, desc)

    def respond_info(self, msg, log=True):
        self.env.rec.rec('GCR', 'respond_info(%r)' % msg)

    def respond_raw(self, msg):
        pass

    def run_script_from_command(self, script):
        env = self.env
        self.scripts.append(script)
        word = script.strip().split()[0] if script.strip() else ''
        if word == 'G1':
            th = env.toolhead
            newpos = list(th.pos)
            for tok in script.split()[1:]:
                axis, val = tok[0].upper(), tok[1:]
                if axis in 'XYZ':
                    i = 'XYZ'.index(axis)
                    newpos[i] = float(val) + (FakeToolhead.Z_BASE if i == 2 else 0.0)
            th.move(newpos, 0)
        elif word == 'G28':
            env.toolhead.kin.limits = [(0.0, 220.0), (0.0, 220.0), (-5.0, 255.0)]
        elif word == 'BED_MESH_PROFILE' and 'LOAD' in script:
            prof = env.bed_mesh.pmgr.profiles.get('default')
            env.bed_mesh.set_mesh(prof['mesh'] if prof else None)
        elif word == 'M140':
            env.heater_bed.target_temp = float(script.split('S')[1])
        elif word in self.macro_handlers:
            self.macro_handlers[word](script)
        env.rec.rec('GC', 'GCodeDispatch.run_script_from_command(%r,) -> None' % script)


class FakeGCmd:
    def __init__(self, params):
        self.params = {k.upper(): str(v) for k, v in params.items()}

    def get(self, name, default=_SENTINEL, **kw):
        if name in self.params:
            return self.params[name]
        if default is _SENTINEL:
            raise ConfigError('missing ' + name)
        return default

    def get_int(self, name, default=_SENTINEL, minval=None, maxval=None):
        v = self.get(name, default)
        return int(v) if v is not None else v

    def get_float(self, name, default=_SENTINEL, minval=None, maxval=None, above=None, below=None):
        v = self.get(name, default)
        return float(v) if v is not None else v


class CommandError(Exception):
    pass


class FakePrinter:
    command_error = CommandError

    def __init__(self, env):
        self.env = env
        self.objects = collections.OrderedDict()
        self.event_handlers = collections.defaultdict(list)

    def lookup_object(self, name, default=_SENTINEL):
        if name in self.objects:
            return self.objects[name]
        if default is _SENTINEL:
            raise ConfigError('Unknown object %r' % name)
        return default

    def add_object(self, name, obj):
        self.objects[name] = obj

    def get_reactor(self):
        return self.env.reactor

    def is_shutdown(self):
        return False

    def register_event_handler(self, event, cb):
        self.event_handlers[event].append(cb)


class Obj:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class Env:
    """One fake printer. Build, then construct the wrapper with env.config()."""

    def __init__(self, cfg_sections):
        self.rec = Recorder()
        self.inj = Injector(self.rec)
        self.reactor = FakeReactor(self.inj)
        self.fake_time = FakeTime(self.reactor)
        self.fake_random = FakeRandom()
        self.replies = {}
        self.sends = []
        self.notes = []
        self.sections = cfg_sections
        p = self.printer = FakePrinter(self)
        self.mcu = FakeMCU(self, 'mcu', 5)
        self.leveling_mcu = FakeMCU(self, 'leveling_mcu', 0)
        self.nozzle_mcu = FakeMCU(self, 'nozzle_mcu', 0)
        self.gcode = FakeGCode(self)
        self.toolhead = FakeToolhead(self)
        self.bed_mesh = FakeBedMesh(self)
        self.heater_hot = FakeHeater('extruder', idle=25.0)
        self.heater_bed = FakeHeater('heater_bed', idle=25.0)
        p.add_object('gcode', self.gcode)
        p.add_object('pins', FakePins({'mcu': self.mcu, 'leveling_mcu': self.leveling_mcu,
                                       'nozzle_mcu': self.nozzle_mcu}))
        p.add_object('virtual_sdcard', Obj(run_dis=0))
        p.add_object('toolhead', self.toolhead)
        p.add_object('bed_mesh', self.bed_mesh)
        p.add_object('heaters', FakeHeaters())
        p.add_object('extruder', Obj(heater=self.heater_hot))
        p.add_object('heater_bed', Obj(heater=self.heater_bed))
        p.add_object('heater_fan hotend_fan', Obj())
        p.add_object('stepper_enable', Obj(lookup_enable=lambda name: Obj(
            is_motor_enabled=lambda: True, motor_enable=lambda t: None, motor_disable=lambda t: None)))
        self.configfile_sets = []
        p.add_object('configfile', Obj(set=lambda s, o, v: self.configfile_sets.append((s, o, v))))

    def config(self, section='prtouch_v2'):
        return FakeConfig(self.printer, self.sections, section, self.rec)
