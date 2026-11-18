# Black-box tracer for the K1 prtouch probe module (prtouch_v1_wrapper and
# prtouch_v2_wrapper: Creality's compiled .so or the k1/prtouch rewrite).
#
# Must be loaded BEFORE [prtouch_v2] in printer.cfg (an empty [prtouch_trace]
# section above it): it patches PRTouchEndstopWrapper at import time so the
# bound methods prtouch_v2 registers in __init__ are the traced ones.
#
# Logs to <logs>/prtouch_trace.log:
#   CALL/RET/EXC  every PRTouchEndstopWrapper method (args, return, duration)
#   CFG           every config read in __init__ (option, default, bounds, value)
#   STATE         vars(self) after __init__ and after each top-level call
#   MCU>          every command sent / config cmd added while inside a call
#   TH/GC/STP     toolhead, gcode and stepper calls made while inside a call
#   DICT          prtouch-related MCU dictionary entries at connect
import logging, os, sys, time, functools, threading

LOG_PATH = '/usr/data/printer_data/logs/prtouch_trace.log'
MAXREPR = 4000

_log = logging.getLogger('prtouch_trace')
_log.propagate = False
_log.setLevel(logging.DEBUG)
if not _log.handlers:
    _h = logging.FileHandler(LOG_PATH)
    _h.setFormatter(logging.Formatter('%(message)s'))
    _log.addHandler(_h)

_t0 = time.monotonic()
_depths = {}
_ctx_ids = {}
_snap = {}
try:
    import greenlet as _gl
except ImportError:
    _gl = None


def _ctx():
    g = id(_gl.getcurrent()) if _gl else 0
    return (threading.get_ident(), g)


def _depth():
    return _depths.get(_ctx(), 0)


def _set_depth(d):
    _depths[_ctx()] = d


def _ctx_tag():
    k = _ctx()
    if k not in _ctx_ids:
        _ctx_ids[k] = len(_ctx_ids)
    return 'c%d' % _ctx_ids[k]


def _r(v):
    try:
        s = repr(v)
    except Exception as e:
        s = '<repr error %s>' % e
    return s if len(s) <= MAXREPR else s[:MAXREPR] + '...(%d)' % len(s)


def _emit(tag, msg):
    _log.debug('%10.4f %-3s %s%s %s', time.monotonic() - _t0, _ctx_tag(),
               '  ' * _depth(), tag, msg)


def _flush():
    for h in _log.handlers:
        h.flush()


def _state_dump(obj, label):
    try:
        items = sorted(vars(obj).items())
    except TypeError:
        return
    prev = _snap.setdefault(id(obj), {})
    for k, v in items:
        rv = _r(v)
        if prev.get(k) != rv:
            prev[k] = rv
            _emit('STATE', '%s %s = %s' % (label, k, rv))


class _CfgProxy:
    """Logs every config read prtouch makes."""
    def __init__(self, cfg):
        self._cfg = cfg

    def __getattr__(self, name):
        attr = getattr(self._cfg, name)
        if not callable(attr):
            return attr

        def wrapped(*a, **kw):
            res = attr(*a, **kw)
            sect = self._cfg.get_name()
            _emit('CFG', '[%s] %s%s%s -> %s' % (sect, name, _r(a),
                  (' ' + _r(kw)) if kw else '', _r(res)))
            if name in ('getsection',) and res is not None:
                return _CfgProxy(res)
            return res
        return wrapped


def _trace_method(cls, name, fn):
    @functools.wraps(fn)
    def wrapper(self, *a, **kw):
        if name == '__init__' and a:
            a = (_CfgProxy(a[0]),) + a[1:]
        _emit('CALL', '%s%s%s' % (name, _r(a), (' ' + _r(kw)) if kw else ''))
        _set_depth(_depth() + 1)
        start = time.monotonic()
        try:
            res = fn(self, *a, **kw)
        except BaseException as e:
            _set_depth(_depth() - 1)
            _emit('EXC', '%s %s: %s (%.3fs)' % (name, type(e).__name__, e,
                                               time.monotonic() - start))
            _flush()
            raise
        _set_depth(_depth() - 1)
        _emit('RET', '%s -> %s (%.3fs)' % (name, _r(res),
                                          time.monotonic() - start))
        if _depth() == 0 and not name.startswith('_handle'):
            _state_dump(self, 'after ' + name)
            _flush()
        return res
    return wrapper


def _wrap_when_inside(cls, names, tag):
    for name in names:
        fn = getattr(cls, name, None)
        if fn is None or getattr(fn, '_ptrace', False):
            continue

        def make(fn, name):
            @functools.wraps(fn)
            def w(*a, **kw):
                if _depth() == 0:
                    return fn(*a, **kw)
                res = fn(*a, **kw)
                _emit(tag, '%s.%s%s%s -> %s' % (
                    cls.__name__, name, _r(a[1:]),
                    (' ' + _r(kw)) if kw else '', _r(res)))
                return res
            w._ptrace = True
            return w
        setattr(cls, name, make(fn, name))


def _patch_mcu():
    import mcu, serialhdl

    def wrap_send(cls, meth):
        orig = getattr(cls, meth)

        @functools.wraps(orig)
        def w(self, data=(), *a, **kw):
            if _depth():
                fmt = getattr(getattr(self, '_cmd', None), 'msgformat', '?')
                _emit('MCU>', '%s %s %s' % (fmt, _r(data), _r(a) if a else ''))
            res = orig(self, data, *a, **kw)
            if _depth() and cls is mcu.CommandQueryWrapper:
                _emit('MCU<', _r(res))
            return res
        setattr(cls, meth, w)
    wrap_send(mcu.CommandWrapper, 'send')
    wrap_send(mcu.CommandQueryWrapper, 'send')

    orig_add = mcu.MCU.add_config_cmd

    def add_config_cmd(self, cmd, *a, **kw):
        if 'prtouch' in cmd or _depth():
            _emit('MCUCFG', '%s %s %s' % (self.get_name(), cmd, _r(kw)))
        return orig_add(self, cmd, *a, **kw)
    mcu.MCU.add_config_cmd = add_config_cmd

    orig_lookup = mcu.MCU.lookup_command
    orig_lookup_q = mcu.MCU.lookup_query_command

    def lookup_command(self, msgformat, *a, **kw):
        if _depth():
            _emit('LOOKUP', '%s %s' % (self.get_name(), msgformat))
        return orig_lookup(self, msgformat, *a, **kw)

    def lookup_query_command(self, msgformat, respformat, *a, **kw):
        if _depth():
            _emit('LOOKUPQ', '%s %s | %s %s' % (self.get_name(), msgformat,
                                                respformat, _r(kw)))
        return orig_lookup_q(self, msgformat, respformat, *a, **kw)
    mcu.MCU.lookup_command = lookup_command
    mcu.MCU.lookup_query_command = lookup_query_command

    orig_reg = mcu.MCU.register_response

    def register_response(self, cb, msg, oid=None):
        if _depth():
            _emit('REGRESP', '%s %s oid=%s cb=%s' % (
                self.get_name(), msg, oid, getattr(cb, '__name__', cb)))
        return orig_reg(self, cb, msg, oid)
    mcu.MCU.register_response = register_response


def _patch_misc():
    import toolhead, stepper
    from extras import homing
    _wrap_when_inside(toolhead.ToolHead, [
        'move', 'manual_move', 'dwell', 'wait_moves', 'set_position',
        'get_position', 'drip_move', 'flush_step_generation',
        'get_last_move_time', 'note_kinematic_activity', 'register_lookahead_callback',
        'get_max_velocity'], 'TH')
    _wrap_when_inside(stepper.MCU_stepper, [
        'get_mcu_position', 'get_commanded_position', 'set_position',
        'get_past_mcu_position', 'mcu_to_commanded_position',
        'get_step_dist', 'get_oid', 'get_name', 'set_trapq',
        'get_trapq', 'add_active_callback', 'get_dir_inverted',
        'setup_itersolve', 'generate_steps', 'note_homing_end',
        'calc_position_from_coord'], 'STP')
    import gcode
    _wrap_when_inside(gcode.GCodeDispatch, [
        'run_script_from_command', 'run_script', 'respond_info',
        'respond_raw'], 'GC')
    for cls_name in ('HomingMove', 'PrinterHoming', 'Homing'):
        cls = getattr(homing, cls_name, None)
        if cls is not None:
            _wrap_when_inside(cls, [n for n in vars(cls)
                                    if not n.startswith('__')], 'HOM')


def _patch_wrapper():
    from extras import prtouch_v1_wrapper, prtouch_v2_wrapper
    for w in (prtouch_v1_wrapper, prtouch_v2_wrapper):
        _patch_wrapper_mod(w)


def _patch_wrapper_mod(w):
    cls = w.PRTouchEndstopWrapper
    if getattr(cls, '_ptraced', False):
        return
    for name, fn in list(vars(cls).items()):
        if callable(fn) and (not name.startswith('__') or name == '__init__'):
            setattr(cls, name, _trace_method(cls, name, fn))
    cls._ptraced = True
    _emit('INFO', '%s attrs: %s' % (w.__name__, _r(sorted(
        k for k in vars(w) if not k.startswith('__')))))
    for k, v in sorted(vars(w).items()):
        if not k.startswith('__') and not callable(v):
            _emit('MODVAR', '%s.%s = %s' % (w.__name__, k, _r(v)))


class PRTouchTrace:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.register_event_handler('klippy:connect', self._connect)
        _emit('INFO', '==== klippy start pid=%d ====' % os.getpid())
        _emit('INFO', 'preloaded: objects=%s mods=%s' % (
            sorted(self.printer.objects.keys()),
            sorted(m for m in sys.modules if 'prtouch' in m)))
        _patch_mcu()
        _patch_misc()
        _patch_wrapper()
        _flush()

    def _connect(self):
        for name, obj in self.printer.lookup_objects('mcu'):
            try:
                mp = obj._serial.get_msgparser()
            except Exception:
                continue
            for tag, mtype, fmt in mp.get_messages():
                if any(s in fmt for s in ('prtouch', 'pres', 'swap', 'hx711',
                                          'step_', 'dirzctl')):
                    _emit('DICT', '%s %s %s %s' % (name, tag, mtype, fmt))
            for k, v in sorted(mp.get_constants().items()):
                _emit('DICT', '%s const %s=%s' % (name, k, v))
            _emit('DICT', '%s version %s' % (name, _r(mp.get_version_info())))
        p = self.printer.lookup_object('prtouch_v2', None)
        if p is None:
            p = self.printer.lookup_object('probe').mcu_probe
        _emit('INFO', 'connect: type=%r run_G28_Z=%r init=%r' % (
            type(p),
            vars(type(p)).get('run_G28_Z'), vars(type(p)).get('__init__')))
        _state_dump(p, 'connect')
        _flush()


def load_config(config):
    return PRTouchTrace(config)
