# System patches

Modifications to files outside `printer_data/config`. These live in the stock
firmware tree, so a Creality firmware update silently reverts them.

The probe module replacement (`prtouch_v1_wrapper.py`) has its own directory,
`k1/prtouch/`, with install and restore steps.

## heater_bed.py

`/usr/share/klipper/klippy/extras/heater_bed.py`

Stock `cmd_M140` clamps every requested bed temperature to `max_temp - 15`
(115 - 15 = 100C), so the bed never reaches its configured maximum. The patch
clamps to `max_temp` instead.

Files here: the patched `heater_bed.py`, the untouched `heater_bed.py.orig`,
and `heater_bed.py.patch` (the diff between them).

Re-apply after a firmware update:

    scp system-patches/heater_bed.py k1:/usr/share/klipper/klippy/extras/heater_bed.py
    ssh k1 'rm /usr/share/klipper/klippy/extras/heater_bed.pyc; /etc/init.d/S55klipper_service restart'

The stale `.pyc` must go, otherwise Klipper keeps running the old bytecode.

Symptom that the patch is gone: the bed stops going above 100C.

## prtouch_v2.py

`/usr/share/klipper/klippy/extras/prtouch_v2.py`

The `[prtouch_v2]` loader imports both of Creality's closed probe modules and
builds the one `pr_version` selects; the K1 uses `pr_version: 1`. The patch
drops the import of the unused `prtouch_v2_wrapper` `.so` and makes any other
`pr_version` a config error, so the `.so` can leave `extras/`. It sits in
`/usr/data/prtouch_stock/` on the printer.

Files here: the patched `prtouch_v2.py`, the untouched `prtouch_v2.py.orig`,
and `prtouch_v2.py.patch`.

Re-apply after a firmware update (together with `k1/prtouch/`):

    scp system-patches/prtouch_v2.py k1:/usr/share/klipper/klippy/extras/prtouch_v2.py
    ssh k1 'E=/usr/share/klipper/klippy/extras; rm $E/prtouch_v2.pyc; mv $E/prtouch_v2_wrapper.cpython-38-mipsel-linux-gnu.so /usr/data/prtouch_stock/; /etc/init.d/S55klipper_service restart'

## homing_abort.py

`/usr/share/klipper/klippy/extras/homing_abort.py` (new file, loaded by the
`[homing_abort]` section in `printer.cfg`)

G28 holds the gcode mutex until homing finishes, so a cancel from Mainsail or
anything else using Moonraker's cancel API normally waits out the whole homing
sequence and the rest of the running macro. The module wraps Klippy's `pause_resume/cancel`
webhook, the endpoint Moonraker's cancel calls. While a cancel is pending it
fires the active homing move's trsync with the host-request reason, the same
MCU path an endstop hit takes. The steppers stop, the TMC homing-end handlers
run, and G28 fails with "No trigger on <axis> after full movement" (preceded by
"Homing aborted by cancel request"). That error ends the print, then the queued
`CANCEL_PRINT` runs. Klipper stays up; no `FIRMWARE_RESTART`. Axes are left
unhomed.

Probing (KAMP's `BED_MESH_CALIBRATE`) runs inside the prtouch probe module's
`run_G29_Z()` (stock `.so` or `k1/prtouch/`), which never fires the homing-move events, so a probe in progress
can't be interrupted. The module wraps `homing.probing_move` instead: while a
cancel is pending, the next probe point fails with "Probing aborted by cancel
request", so the mesh stops after the point in progress.

Heat waits (`M109`, `M190`) poll Creality's `can_break` flag in `heaters.py`
once a second, the flag its `breakheater` webhook sets. A cancel sets it, and
the module turns the early return into "Heat wait aborted by cancel request",
so the macro stops instead of continuing at the wrong temperature.
`END_PRINT`, run by `CANCEL_PRINT`, turns the heaters off.

Scope: X/Y homing, the gaps between probe points, and `M109`/`M190` waits.
`G28 Z` runs the prtouch probe module's `run_G28_Z()` (self-test plus three bed
descents onto the nozzle), which never fires the homing-move events, so a
cancel during Z homing waits for it to finish. A cancel during
`TEMPERATURE_WAIT`, a dwell, an ordinary move, `ACCURATE_HOME_Z` or the nozzle
wipe also waits for that command to finish. A `CANCEL_PRINT` typed in the
console bypasses the webhook and is not covered.

Install or re-apply after a firmware update:

    scp system-patches/homing_abort.py k1:/usr/share/klipper/klippy/extras/homing_abort.py
    ssh k1 /etc/init.d/S55klipper_service restart

Symptom that the module is gone: Klipper fails to start with an unknown
`[homing_abort]` config section.
