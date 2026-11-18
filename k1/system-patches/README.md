# System patches

Modifications to files outside `printer_data/config`. These live in the stock
firmware tree, so a Creality firmware update silently reverts them.

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

Probing (KAMP's `BED_MESH_CALIBRATE`) runs inside prtouch_v2's closed
`run_G29_Z()`, which never fires the homing-move events, so a probe in progress
can't be interrupted. The module wraps `homing.probing_move` instead: while a
cancel is pending, the next probe point fails with "Probing aborted by cancel
request", so the mesh stops after the point in progress.

Heat waits (`M109`, `M190`) poll Creality's `can_break` flag in `heaters.py`
once a second, the flag its `breakheater` webhook sets. A cancel sets it, and
the module turns the early return into "Heat wait aborted by cancel request",
so the macro stops instead of continuing at the wrong temperature.
`END_PRINT`, run by `CANCEL_PRINT`, turns the heaters off.

Scope: homing moves, the gaps between probe points, and `M109`/`M190` waits.
A cancel during `TEMPERATURE_WAIT`, a dwell, an ordinary move,
`ACCURATE_HOME_Z` or the nozzle wipe still waits for that command to finish. A `CANCEL_PRINT` typed in the console
bypasses the webhook and is not covered.

Install or re-apply after a firmware update:

    scp system-patches/homing_abort.py k1:/usr/share/klipper/klippy/extras/homing_abort.py
    ssh k1 /etc/init.d/S55klipper_service restart

Symptom that the module is gone: Klipper fails to start with an unknown
`[homing_abort]` config section.
