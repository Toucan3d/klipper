# CTC ILC calibration binaries

This directory holds the standalone CTC ILC concentricity-calibration
program launched by the `[ctc_ilc]` Klipper module
(`klippy/extras/ctc_ilc.py`) via the `CTC_ILC_CALIBRATE` G-Code command.

The program is **proprietary Toucan3D software and not part of
Klipper**. It runs as a separate host process and communicates with
Klipper exclusively through the Klipper API server Unix domain socket
(the path klippy is started with via `-a`). It is distributed here in
compiled form only; the source lives in the private `CTC_Eddy_SW`
repository (`scripts/ctc_ilc_auto.py`).

## Files

- `ctc_ilc_auto-aarch64` — 64-bit ARM Linux hosts (Raspberry Pi 3/4/5
  with 64-bit OS and most SBCs)
- `ctc_ilc_auto-armv7l` — 32-bit ARM Linux hosts (32-bit Raspberry Pi OS)

The launcher selects the binary matching `os.uname().machine`
automatically; an explicit `program:` option in the `[ctc_ilc]` config
section overrides the selection.

## Compatibility

The binaries are built with Nuitka (onefile) inside Debian Bullseye
containers, so they run on any glibc-based distribution with glibc >=
Bullseye's (Bullseye, Bookworm and newer). The CPython runtime is
bundled - no Python installation is required on the host. Not supported:
musl-based distributions (e.g. Alpine) and glibc older than Bullseye.

The binary self-extracts to `~/.cache/ctc_ilc/` on start, which avoids
`noexec`-mounted `/tmp` filesystems.

## Building

Built from the `CTC_Eddy_SW` repository. On any machine with Docker and
qemu binfmt handlers (`docker run --privileged --rm tonistiigi/binfmt
--install arm64,arm`):

```bash
cd CTC_Eddy_SW
for img in arm64v8/debian:bullseye arm32v7/debian:bullseye; do
  docker run --rm -v "$PWD":/src -w /src/scripts "$img" bash -c '
    apt-get update && apt-get install -y gcc python3-dev python3-pip patchelf ccache
    python3 -m pip install nuitka pyserial
    python3 -m nuitka --onefile \
        --onefile-tempdir-spec="{HOME}/.cache/ctc_ilc/{VERSION}" \
        --output-filename=ctc_ilc_auto-$(uname -m) \
        --output-dir=build ctc_ilc_auto.py'
done
```

Copy `scripts/build/ctc_ilc_auto-aarch64` and
`scripts/build/ctc_ilc_auto-armv7l` into this directory and record the
`CTC_Eddy_SW` commit they were built from in the commit message.

## Configuration

```ini
[ctc_ilc]
#program:                     # default: this directory's binary for the host arch
#output_dir: ~/ctc_ilc_sessions
#serial_port: /dev/ttyACM1
#timeout: 21600               # seconds; the run is killed when exceeded
#result_variant: sign_inverted   # which final lookup file is persisted
```

`CTC_ILC_CALIBRATE` starts a run. The head must already be at the
measurement position with the eddy-sensor load cell in place - the
calibration captures the current X/Y/Z position as its measurement base
and starts measuring immediately (use `DISABLE_LOOKUP=1` to clear an
active `[ctc]` table first). `CTC_ILC_STATUS` reports progress, `CTC_ILC_ABORT` stops the
run. On success the new lookup is applied to `[ctc]` immediately and
staged for `SAVE_CONFIG`.

While a calibration is running, do not send motion G-Code - the
calibration owns the printer.
