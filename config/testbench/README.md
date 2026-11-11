# Toucan3D test bench configuration

Snapshot of the configuration files of the Toucan3D Klipper test bench
(Raspberry Pi with a BTT Octopus V1.1, `kinematics: none`). On the Pi they
live in `~/printer_data/config/`.

| File | Content |
|---|---|
| `printer.cfg` | Thermistors, fan, BTT SFS V2.0 runout switch and `[filament_encoder sfs]`, `sfs_flow` display sensor, load cell calibration in the SAVE_CONFIG block |
| `loadcell.cfg` | HX711 load cell and `nozzle_force` display sensor |
| `hcu_v03_toolhead.cfg` | Toucan3D toolhead on the HCU V0.3: induction heater (`resonance_frequency` 147000 for the coil fitted 2026-11-10), two TMC2130 extruder motors with `driver_SGT: 10`, A axis, probe, fans |
| `moonraker.conf` | Trusted clients, update manager entries, `[testdata]` file root |
| `KlipperScreen.conf` | Bench dashboard layout switch |

The load cell calibration in `printer.cfg` is nominal (datasheet value), not
measured. `mm_per_edge` 1.483 of the SFS was derived on 2026-11-11 from the
calibrated extruder at 15-40 mm^3/s with the new filament path (1.506 with
the old path); the marked 500 mm test is still open.

The Pi 3 runs its USB on the `dwc2` driver (`dtoverlay=dwc2,dr_mode=host` at
the end of `/boot/firmware/config.txt`, not part of this snapshot). With the
default `dwc_otg` driver the HCU's CH340 adapter logged
`urb stopped: -32` errors and Klipper lost the HCU at the second one.

Related branches: `test-bench` of Toucan3d/KlipperScreen (bench dashboard)
and Toucan3d/mainsail (bench web page).
