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
