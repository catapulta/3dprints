# prtouch: open-source K1 strain-gauge probe module

`prtouch_v1_wrapper.py` is a drop-in Python replacement for Creality's closed
`/usr/share/klipper/klippy/extras/prtouch_v1_wrapper.cpython-38-mipsel-linux-gnu.so`
(Cython 0.29.32, PR_VERSION 307). That module is the K1's load-cell Z probe:
`G28` Z, `ACCURATE_HOME_Z`, `BED_MESH_CALIBRATE` probing, `CHECK_BED_MESH`,
`NOZZLE_CLEAR` and the prtouch debug commands. `prtouch_v2.py` loads it because
the `[prtouch_v2]` section sets `pr_version: 1`.

The module keeps the original's class (`PRTouchEndstopWrapper`), all 71 methods
with their signatures, every config option with its default and bounds, the
G-code commands and help texts, the error dicts, and the MCU protocol. The MCU
firmware is closed and unchanged; the module sends the same command sequences,
parameters, thresholds, timings and toolhead moves as the original. Original
behaviour is kept even where it looks wrong (see below). Comments give the
original `.py` line range of each method and, for paths no trace covers, the
binary function they were reconstructed from.

## Install and restore

Python loads a `.so` before a `.py` of the same name, so the stock module has
to move out of `extras/`:

    ssh k1 'mkdir -p /usr/data/prtouch_stock && mv /usr/share/klipper/klippy/extras/prtouch_v1_wrapper.cpython-38-mipsel-linux-gnu.so /usr/data/prtouch_stock/'
    scp prtouch/prtouch_v1_wrapper.py k1:/usr/share/klipper/klippy/extras/prtouch_v1_wrapper.py
    ssh k1 /etc/init.d/S55klipper_service restart

The patched loader (`k1/system-patches/prtouch_v2.py`) drops the import of
Creality's unused v2 module, so its `.so` leaves `extras/` too and no closed
probe code remains in Klipper.

Restore the stock modules and loader:

    scp system-patches/prtouch_v2.py.orig k1:/usr/share/klipper/klippy/extras/prtouch_v2.py
    ssh k1 'E=/usr/share/klipper/klippy/extras; rm $E/prtouch_v1_wrapper.py $E/prtouch_v1_wrapper.pyc $E/prtouch_v2.pyc; mv /usr/data/prtouch_stock/*.so $E/; /etc/init.d/S55klipper_service restart'

A Creality firmware update puts both `.so` files and the stock loader back, and
the `.so` wins over the `.py`. `/usr/data/prtouch_stock/` on the printer holds
both stock `.so` files and the private archive (below).

## Verification

Offline, `tests/run_e2e.py` builds a fake Klipper around the module, replays
the MCU responses recorded from the stock module on this printer, and diffs
every wrapper call, MCU command, toolhead move, G-code, console message and
attribute change against the trace:

    python3 k1/prtouch/tests/run_e2e.py            # all scenarios
    uv run --no-project --python 3.8 python k1/prtouch/tests/run_e2e.py
    python3 k1/prtouch/tests/run_e2e.py -v G28     # one scenario, verbose

Scenarios: `boot`, `G28`, `G29` (5x5 `BED_MESH_CALIBRATE`), `AHZ`
(`ACCURATE_HOME_Z`), `CBM` (`CHECK_BED_MESH`), `NCLR` (`NOZZLE_CLEAR
HOT_MIN_TEMP=180 HOT_MAX_TEMP=220 BED_MAX_TEMP=50`) and `RP` (`READ_PRES C=10`).
All match exactly; the only differences are a few `delay_s(0.01)` polling
loops that poll once less, an artifact of when the fake delivers MCU
responses. `E2E_WRAPPER=<path>` replays another copy of the module.

On the printer, the module ran boot, `READ_PRES`, `G28`, `ACCURATE_HOME_Z`,
the 5x5 mesh, `NOZZLE_CLEAR` (the moves, at the configured wipe band), a print
start (`START_PRINT`: rough `G28`, `ACCURATE_G28`, adaptive mesh, purge, first
layers), and a Mainsail cancel during mesh probing that `homing_abort` stops
between points. Its boot trace is identical to the stock module's, and every
flow follows the stock call sequence; probe results agree within normal probe
repeatability.

## Tracing

`tools/prtouch_trace.py` is a Klipper extra that wraps every
`PRTouchEndstopWrapper` method, the MCU command and response paths, and the
toolhead, stepper, G-code and homing calls made inside them. It logs to
`/usr/data/printer_data/logs/prtouch_trace.log` and works with the stock `.so`
and with this module.

    scp prtouch/tools/prtouch_trace.py k1:/usr/share/klipper/klippy/extras/
    scp prtouch/tools/ptrace_run.sh prtouch/tools/ptrace_post.py k1:/usr/data/

Enable it by making an empty `[prtouch_trace]` section the first line of
`printer.cfg` (it must load before `[prtouch_v2]`) and restarting Klipper; the
log grows with every probe, so remove the section when not diagnosing.
`/usr/data/ptrace_run.sh <name> <gcode...>` runs G-code through Moonraker,
waits for idle and saves the slice as `trace_<name>.log`.

`traces/` holds the recorded traces (gzip). The ones without a suffix are the
stock module; `_recon` are this module on the same printer; `stockboot2` /
`reconboot` are the two boot traces; `PRINT_cancel` and `PRINT_full` are the
print-start runs with this module.

## Original behaviour kept on purpose

None of these paths runs during normal homing, meshing or printing.

| where | behaviour |
|---|---|
| `cmd_TEST_SWAP` | sends `read_swap_prtouch` with `pres_oid` to the step MCU; may shut the MCU down |
| `cmd_SAFE_DOWN_Z` | passes config lists where numbers are expected; every leg raises `TypeError` on this config |
| `cmd_TRIG_BED_TEST` | raises `AttributeError` with `F=0` and leaves speeds and mesh unrestored |
| `ck_and_manual_get_pres` | resends lost pressure data with `step_oid` on the pressure MCU, so the resend fails |
| `env_self_check` | passes when all 4 load cells are unsensed; a swap-pin failure returns False without an error |
| `run_G28_Z` | plain `G28` (not accurate) returns with the bed mesh unloaded |
| `run_G29_Z` | `BED_MESH_CALIBRATE` before any Z homing raises `AttributeError` (`g29_cnt`) |
| `bed_mesh_post_proc` | neighbour bounds swap x and y counts (harmless on square meshes) |
| `clear_nozzle` | wipes even when all 3 probe attempts read too low |
| `clear_nozzle_quick` | the stroke parity uses a stale index; the fallback leaves accel raised |
| `add_stepper` | indexes `steppers[0]` before appending (nothing calls it) |
| `run_re_g29s` | nothing calls it |
| `run_step_prtouch`, `run_G28_Z` | two error messages print an unformatted `{}` |

## Paths no trace covers

These are reconstructed from the binary alone: the error, timeout and
no-trigger branches of `run_step_prtouch`, `run_G28_Z` and `run_G29_Z`; the
lost-data resend paths; `env_self_check`'s failure branches and fallback;
`bed_mesh_post_proc`'s spike re-probe; `safe_move_z` and `SAFE_MOVE_Z`;
`clear_nozzle_quick` (`clr_noz_quick` is off); `PRTOUCH_READY`; the debug
commands other than `READ_PRES`; and the `use_adc` and corexz branches, which
the K1 does not use.

## Hazard: wipe band

The extruder motor carries a taller heatsink, and the head hits the rear frame
at the `[prtouch_v2]` wipe band (`clr_noz_start_y: 221`, `clr_noz_len_y: 2`,
Y 221-223). `NOZZLE_CLEAR` must not run until the wipe band moves forward.
`CX_NOZZLE_CLEAR` in `gcode_macro.cfg` is an empty override, so `START_PRINT`
does not wipe.

## Decompile and private archive

The decompiled vendor code and the binaries are not in this public repo.
`private/prtouch-private.tar.xz` (git-ignored) holds them: the Ghidra output
(`decomp/funcs`, annotated `decomp/annot`, `index.tsv`, `methods.tsv`,
`strtab.tsv`, `consts.tsv`), both wrapper `.so` files, the per-group
reconstruction shards with their notes and checkers, the build notes, and the
agent briefs. A copy sits on the printer in `/usr/data/prtouch_stock/`.

The decompile regenerates from the `.so` with the scripts in `tools/`
(Ghidra from Homebrew, `JAVA_HOME=$(brew --prefix openjdk@25)/libexec/openjdk.jdk/Contents/Home`):

1. `analyzeHeadless <proj> prtouch -import <so> -scriptPath tools -postScript DumpDecomp.java <out> -deleteProject`
   writes `<out>/funcs/*.c` and `<out>/index.tsv`.
2. `python3 tools/pyx_strtab.py <so> > <out>/strtab.tsv` recovers the Cython
   string table (interned names and constants).
3. `python3 tools/map_methods.py <so> <out>/index.tsv > <out>/methods.tsv`
   maps the Python methods to their entry points.
4. Re-run step 1 as `... -postScript Annotate.java <out>/methods.tsv <out>/strtab.tsv 10000 -postScript DumpDecomp.java <out>`:
   it labels the string-table slots and creates the method entry functions
   (`M_<method>`) at the +0x10000 load shift, then dumps again.
5. `python3 tools/annotate_c.py <so> <out>` resolves the MIPS PIC GOT accesses
   into names and constants and writes `<out>/annot/*.c`.

Ghidra loads the `.so` at +0x10000: Ghidra addresses are file VAs + 0x10000.
`__init__`'s body (`FUN_0007b720`) needs a decompile timeout above
`DumpDecomp.java`'s 300 s.
