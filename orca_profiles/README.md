# OrcaSlicer user profiles

Backup of the personalized OrcaSlicer presets (machines, filaments, processes).

Source on macOS:

    ~/Library/Application Support/OrcaSlicer/user/default/

Layout mirrors that directory:

- `machine/` — printer presets (Creality K1 0.2/0.4/0.6 nozzle, Anycubic Chiron, Ender-3 0.6)
- `filament/` — filament presets (Kingroon ABS, Sunlu/Duramic/3DHoJor PETG, various PLA, TPU)
- `process/` — layer-height/quality presets per printer and nozzle

Each preset is a `.json` holding the settings plus a `.info` with its sync metadata.

## Restore

Close OrcaSlicer, then copy the directories back:

    rsync -a orca_profiles/ ~/Library/Application\ Support/OrcaSlicer/user/default/

Presets inherit from Orca's bundled system profiles, so the matching vendor
profiles must be installed in OrcaSlicer for them to load.

`print_host` fields point at printers on the local network (192.168.1.x).
