# CTC calibration binaries

This directory holds the standalone calibration programs launched from
the Klipper console:

| Program | Klipper module | Command |
|---|---|---|
| `ctc_ilc_auto-<arch>` | `[ctc_ilc]` (`klippy/extras/ctc_ilc.py`) | `CTC_ILC_CALIBRATE` — concentricity lookup table |
| `varoline_offset-<arch>` | `[varoline_offset]` (`klippy/extras/varoline_offset.py`) | `VAROLINE_OFFSET_CALIBRATE` — slot-angle offset |

Both modules share the process supervision and operator prompts in
`klippy/extras/ctc_launcher.py`.

The programs are **proprietary Toucan3D software and not part of
Klipper**. They run as separate host processes and communicate with
Klipper exclusively through the Klipper API server Unix domain socket
(the path klippy is started with via `-a`). They are distributed here in
compiled form only; the sources live in the private `CTC_Eddy_SW`
repository (`scripts/ctc_ilc_auto.py`, `scripts/angle_calibration.py`).

## Files

- `*-aarch64` — 64-bit ARM Linux hosts (Raspberry Pi 3/4/5 with 64-bit
  OS and most SBCs)
- `*-armv7l` — 32-bit ARM Linux hosts (32-bit Raspberry Pi OS)

The launcher selects the binary matching the host userland (pointer
size, so a 32-bit OS on a 64-bit kernel gets the armv7l build)
automatically; an explicit `program:` option in the module's config
section overrides the selection.

## Compatibility

The binaries are built with Nuitka (onefile) in Debian Bullseye
containers and run on any glibc-based distribution with glibc >= 2.31
(Bullseye, Bookworm and newer; verified on both). Exception: the
current `ctc_ilc_auto-armv7l` was built natively on a 32-bit Raspberry
Pi OS Bookworm host and needs glibc >= 2.36 (Bookworm or newer); it is
built without onefile compression (`--onefile-no-compression`) because
the zstd step runs out of memory in a 32-bit process on a 2 GB Pi. A
Docker rebuild lifts that restriction. The CPython runtime is bundled -
no Python installation is required on the host. Not supported:
musl-based distributions (e.g. Alpine) and glibc older than Bullseye.

The binaries self-extract to `~/.cache/ctc_ilc/` on start, which avoids
`noexec`-mounted `/tmp` filesystems.

## Building

Built from the `CTC_Eddy_SW` repository. On any machine with Docker and
qemu binfmt handlers (`docker run --privileged --rm tonistiigi/binfmt
--install arm64,arm`):

Bullseye has moved to `archive.debian.org`, and its security pool is no
longer consistent with the package indexes, so the recipe uses the
official `python:3.9-bullseye` images (Python and gcc preinstalled)
and fetches only `patchelf` from the archive:

```bash
cd CTC_Eddy_SW
for img in arm64v8/python:3.9-bullseye arm32v7/python:3.9-bullseye; do
  docker run --rm -v "$PWD":/src -w /src/scripts "$img" bash -c '
    echo "deb http://archive.debian.org/debian bullseye main" > /etc/apt/sources.list
    rm -f /etc/apt/sources.list.d/*
    apt-get -o Acquire::Check-Valid-Until=false update
    apt-get install -y --no-install-recommends patchelf ccache
    python3 -m pip install nuitka pyserial zstandard ordered-set
    for src in ctc_ilc_auto:ctc_ilc_auto angle_calibration:varoline_offset; do
      python3 -m nuitka --onefile --assume-yes-for-downloads \
          --product-version=0.1.0 \
          --onefile-tempdir-spec="{HOME}/.cache/ctc_ilc/{VERSION}" \
          --output-filename=${src#*:}-$(uname -m) \
          --output-dir=build ${src%:*}.py
    done'
done
```

Copy the four `scripts/build/*-aarch64` / `*-armv7l` files into this
directory (`git update-index --chmod=+x` when committing from Windows)
and record the `CTC_Eddy_SW` commit they were built from in the commit
message.

Native build on a Raspberry Pi instead of Docker (`sudo apt install
patchelf python3-dev ccache zlib1g-dev`, Nuitka in a venv): use the same
commands plus `--onefile-no-compression` on 32-bit hosts. Note that
`uname -m` reports the kernel architecture; a 32-bit OS on a 64-bit
kernel produces the armv7l binary.

## Configuration

```ini
[ctc_ilc]
#program:                     # default: this directory's binary for the host arch
#output_dir: ~/ctc_ilc_sessions
#serial_port: /dev/ttyACM1
#timeout: 21600               # seconds; the run is killed when exceeded
#result_variant: sign_inverted   # which final lookup file is persisted

[varoline_offset]
#program:                     # default: this directory's binary for the host arch
#output_dir: ~/varoline_offset_sessions
#serial_port: /dev/ttyACM1
#timeout: 21600
#initial_a: 0                 # expected X-parallel machine A position; the zero must lie within +/-15 deg
```

## Operation

Both commands first ask to unload the filament, then to place the CTC
calibration device under the CTC beacon with 1-2 mm clearance; each
step is confirmed with `<PREFIX>_CONTINUE` (or the Continue button of
the dialog Mainsail/Fluidd show) and can be cancelled with
`<PREFIX>_ABORT`. The head must already be at the measurement position:
the calibrations capture the current X/Y/Z position as their measurement
base and start measuring right after the second confirmation. An active
bed mesh aborts the command unless `DISABLE_MESH=1` is given, which runs
`BED_MESH_CLEAR` for this session. `<PREFIX>_STATUS` reports progress,
`<PREFIX>_ABORT` stops a run. The console only shows a few progress
lines and the result; the complete program output is kept in
`console.log` inside the session directory (and in `klippy.log`).

`CTC_ILC_CALIBRATE` refuses to run while a `[ctc]` lookup table is
active unless `DISABLE_LOOKUP=1` is given, which clears it. On success
the sign-inverted lookup - the form `[ctc]` stores - is applied to
`[ctc]` immediately and staged for `SAVE_CONFIG`.

`VAROLINE_OFFSET_CALIBRATE [INITIAL_A=<deg>]` needs the concentricity
lookup measured by `CTC_ILC_CALIBRATE`: it requires an active `[ctc]`
table, exports it for the program, switches `[ctc]` compensation off
for the duration of the run (the program applies the lookup itself) and
restores it afterwards. The result - the machine A position at which the
slot is parallel to X - is reported on the console, exposed as
`printer.varoline_offset.recommended_a`, and kept in the session's
`angle_calibration.json`; nothing is written to the config.

While a calibration is running, do not send motion G-Code - the
calibration owns the printer.
