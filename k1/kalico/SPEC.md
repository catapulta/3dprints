# K1 migration to Simple AF (Kalico)

Spec for moving this K1 from Creality's Klipper fork with Helper-Script to
Simple AF (pellcorp/creality) on Kalico, using the bed load cells as the probe.

## Why migrate

The current stack works: probing runs on the open `k1/prtouch/` module and
prints come out well. Migrating trades a working, frozen setup for a
maintained one. The case for it:

- Klipper core. The printer runs Creality's fork of Klipper v0.11 (host
  `v0.11.0-372`, board firmware `1.3.0.40`, built July 2023). Kalico tracks
  current upstream Klipper and adds its own features (MPC heater control,
  multiple probes, sensorless homing without a homing override, non-critical
  MCUs).
- Open firmware on the boards. Simple AF flashes Kalico firmware onto `mcu`,
  `nozzle_mcu` and `leveling_mcu`. Creality's closed board firmware, and the
  probe protocol `k1/prtouch/` reimplements, go away.
- Probing quality. Kalico's `load_cell_probe` analyses each tap's force curve
  and rejects bad taps (ooze, debris such as the TPU residue on this bed);
  Creality's probe triggers on a threshold.
- Updates that stick. A Creality firmware update reverts every patch in
  `k1/system-patches/` and restores the closed `.so` files. Simple AF updates
  through its installer and keeps local changes as config overrides.
- Fewer Creality services. Simple AF removes Creality's display server, cloud
  and AI services, freeing RAM on a 214 MB board.
- Upgrade path. External probes (Cartographer, Beacon, Eddy) and Simple AF for
  a Raspberry Pi, which leaves Creality's OS behind, use the same stack.

Against it:

- All three boards are reflashed (through Creality's own bootloader and
  `S13mcu_update`; recoverable unless power is cut mid-flash).
- A factory reset is required first: Helper-Script, Entware, GuppyScreen and
  everything in `/usr/data` are wiped.
- The four-load-cell driver (`hx711s`, 2026) is new, and its calibration
  format changed in Kalico nov2026. HX711 probing runs at about 1 mm/s.
- Creality Cloud, Creality Print LAN printing and the stock screen go.
- Single maintainer (pellcorp); OrcaSlicer is the only supported slicer.

No current print-quality problem requires the migration.

## Target state

- Creality OS 1.3.3.8 (current firmware; Simple AF needs 1.3.3.5 or higher),
  Simple AF installed to `/usr/data/pellcorp`, Kalico host and Kalico firmware
  on all three boards.
- Probe: `--install loadcells`. `[load_cell_probe]` with `sensor_type: hx711s`
  reads the four bed corners as one load cell:

      dout_pins: leveling_mcu:PA4, leveling_mcu:PA3, leveling_mcu:PA0, leveling_mcu:PA1
      sclk_pins: leveling_mcu:PA7, leveling_mcu:PA6, leveling_mcu:PA2, leveling_mcu:PA5

  These are the same pins as `pres0..3_clk_pins` / `pres0..3_sdo_pins` in the
  current `[prtouch_v2]` section.
- Nozzle wipe disabled or moved (see Hazards).
- GrumpyScreen on the touchscreen, Mainsail or Fluidd in the browser.
- Local changes kept as Simple AF config overrides
  (`/usr/data/pellcorp-overrides`) and hook macros (`_SAF_*`), mirrored in
  this repo.

## Hazards

- Rear collision. The extruder motor carries a taller heatsink and hits the
  rear frame near Y 221. Simple AF's load-cell wipe (`loadcells_wipe.cfg`) runs
  before every bed mesh at the back of the bed, middle third of X, 3 to 5 mm
  in from Y max. It must not run as shipped: remove `[include
  loadcells_wipe.cfg]` or define a `_SAF_NOZZLE_WIPE` that stays clear of the
  rear, before the first `START_PRINT`.
- Power during flashing and factory reset. Do not power-cycle until the screen
  confirms; an interrupted flash or reset is the brick path.
- First load-cell probe. The nozzle is pushed into the bed with a force the
  load cells measure; uncalibrated cells can damage the bed. Stay at the
  printer with the e-stop ready during the first homing and mesh.

## Not carried over

- `k1/prtouch/` (the open probe module) and `k1/system-patches/prtouch_v2.py`:
  Kalico's probe and firmware replace them. They stay in the repo for running
  stock firmware.
- `k1/system-patches/heater_bed.py`: Creality's `max_temp - 15` clamp is in
  Creality's fork only. Verify the bed reaches its configured maximum.
- Helper-Script and Entware: Simple AF ships its own `bash`, `sftp-server`,
  `curl`, `ffmpeg` and `mjpg-streamer`, and refuses to install over
  Helper-Script.

## TODO

### Before

- [ ] Decide to migrate (see Why migrate).
- [ ] Back up all of `/usr/data` to the Mac (4.3 GB; gcodes can be skipped or
      kept separately), including `/usr/data/prtouch_stock/`,
      `printer_data/config/`, `macaddr.txt` and `wpa_supplicant.conf`.
- [ ] Copy `k1/prtouch/private/prtouch-private.tar.xz` to Google Drive.
- [ ] Record the current state: board firmware versions (`1.3.0.40` on all
      three), `printer.cfg` and its `SAVE_CONFIG` block, z-offset, input shaper
      results, bed mesh.
- [ ] Check that a USB stick triggers the emergency factory reset (Simple AF
      wiki, Verify USB).
- [ ] Run Creality's self-check (input shaping and auto leveling) on stock, so
      Switch to Stock has calibrated configs to fall back to.
- [ ] List the local customisations to port:
  - `gcode_macro.cfg` overrides (`CX_NOZZLE_CLEAR` emptied, others)
  - `homing_abort` cancel behaviour (`k1/system-patches/homing_abort.py`)
  - `M600-support.cfg`, `buzzer-support.cfg`, `fans-control.cfg`,
    `nozzle-cleaning-fan-control.cfg`, `save-zoffset.cfg`, `timelapse.cfg`,
    `useful-macros.cfg` from Helper-Script
  - `sensorless.cfg`, `printer_params.cfg`
  - both filament sensors (`encoder_sensor`, `filament_sensor`)
  - `S60wifi_pm` (Wi-Fi power save off)
- [ ] Check the Orca start and end G-code against Simple AF's slicer settings
      page.

### Install

- [ ] Simple AF factory reset (`S58factoryreset reset` from the Simple AF
      repo, not the Creality screen); wait for the stock screen.
- [ ] Restore Wi-Fi and SSH access.
- [ ] Clone `https://github.com/pellcorp/creality.git` to `/usr/data/pellcorp`
      and run `installer.sh --install loadcells` (confirm the exact arguments
      on the load cells wiki page first).
- [ ] Before the first print: disable or replace the nozzle wipe (Hazards).
- [ ] Power-cycle so `S13mcu_update` flashes the three boards; wait for the
      screen.
- [ ] Confirm `mcu`, `nozzle_mcu` and `leveling_mcu` report Kalico firmware in
      `klippy.log`.

### Calibrate

- [ ] `LOAD_CELL_CALIBRATE` with a known weight.
- [ ] `LOAD_CELL_PROBE_CALIBRATE CALIBRATION=TAP_PERFORMANCE` with a clean
      nozzle, then `SAVE_CONFIG`.
- [ ] Home and probe at the printer: G28, `PROBE_ACCURACY`, 5x5 mesh. Compare
      the mesh with the current one (repo `k1/printer.cfg`).
- [ ] Z-offset and first layer: print a single-layer test.
- [ ] `SHAPER_CALIBRATE`.
- [ ] PID or MPC tune for the hotend and bed.

### Port

- [ ] Wi-Fi power save: `wl PM 0` at boot from `/etc/init.d` (Entware is
      gone); confirm with `wl PM` after a reboot.
- [ ] Cancel behaviour: test cancel during homing, mesh and heat waits on
      Kalico; port `homing_abort.py` only for the parts Kalico lacks.
- [ ] Bed max temperature: confirm `M140 S115` reaches 115 C.
- [ ] Filament sensors, M600, buzzer, fan control, timelapse: re-create with
      Simple AF macros or hooks.
- [ ] Move the repo's K1 config to the Simple AF layout (overrides plus
      custom `.cfg`), and document it in `k1/`.

### After

- [ ] A full TPU print and a full PLA or ABS print.
- [ ] Document the installed state in `k1/` (replace this spec with a README).
- [ ] Decide whether to keep `k1/prtouch/` and `k1/system-patches/` as the
      stock-firmware fallback.
- [ ] Later: Simple AF for Raspberry Pi to leave Creality OS entirely.

## Rollback

- Switch to Stock (`/usr/data/pellcorp/k1/switch-to-stock.sh`, then
  power-cycle) restores Creality's Klipper, screen and board firmware for
  temporary use; `--revert` returns to Simple AF.
- A full return to the current setup: Creality factory reset, then reinstall
  Helper-Script, Entware and GuppyScreen, restore the `/usr/data` backup, and
  re-apply `k1/prtouch/` and `k1/system-patches/`.

## References

- Simple AF: https://github.com/pellcorp/creality, wiki
  https://pellcorp.github.io/creality-wiki/ (getting-started, loadcells,
  factory_reset, kalico, kalico_nov2026, nozzle_wipe, custom_hooks,
  config_overrides, switch_to_stock, emergency_factory_reset)
- Kalico fork for the K1: https://github.com/pellcorp/kalico
- Board firmware builds: https://github.com/pellcorp/k1-kalico-firmware
