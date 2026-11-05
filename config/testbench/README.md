# Toucan3D test bench configuration

Snapshot of the configuration files of the Toucan3D Klipper test bench
(Raspberry Pi with a BTT Octopus V1.1, `kinematics: none`). On the Pi they
live in `~/printer_data/config/`.

| File | Content |
|---|---|
| `printer.cfg` | Thermistors, fan, BTT SFS V2.0 runout switch and `[filament_encoder sfs]`, `sfs_flow` display sensor, load cell calibration in the SAVE_CONFIG block |
| `loadcell.cfg` | HX711 load cell and `nozzle_force` display sensor |
| `moonraker.conf` | Trusted clients, update manager entries, `[testdata]` file root |
| `KlipperScreen.conf` | Bench dashboard layout switch |

The load cell calibration in `printer.cfg` is nominal (datasheet value), not
measured. `mm_per_edge` of the SFS is the BTT value, not yet calibrated.

Related branches: `test-bench` of Toucan3d/KlipperScreen (bench dashboard)
and Toucan3d/mainsail (bench web page).
