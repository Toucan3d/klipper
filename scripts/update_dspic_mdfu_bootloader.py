#!/usr/bin/env python3
# Tool to flash a dsPIC MDFU bootloader when no Klipper executable is running
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import argparse
import os
import shutil
import subprocess
import sys
import time

try:
    import serial
except ModuleNotFoundError:
    serial = None

BOOT_TOKEN = b"KLIPPER_BOOT33\n"
BOOT_TOKEN_BURST_COUNT = 10
BOOT_TOKEN_BURST_DELAY = 0.02
RAW_SERIAL_TIMEOUT = 0.5
DEFAULT_BOOT_WAIT = 1.2
DEFAULT_PORT_WAIT = 5.0


class UpdateError(Exception):
    pass


def output_line(msg):
    sys.stdout.write("%s\n" % (msg,))
    sys.stdout.flush()


def require_pyserial():
    if serial is None:
        raise UpdateError(
            "Python's pyserial module is required. Install it with:\n"
            "  %s -m pip install pyserial" % (sys.executable,)
        )


def find_pymdfu_command():
    env_script = os.path.join(os.path.dirname(sys.executable), "pymdfu")
    if os.name == "nt":
        env_script += ".exe"
    if os.path.exists(env_script):
        return [env_script]
    path_script = shutil.which("pymdfu")
    if path_script is not None:
        return [path_script]
    raise UpdateError(
        "Unable to find 'pymdfu' in the current Python environment or PATH."
    )


def wait_for_port(port, baud, timeout):
    deadline = time.monotonic() + timeout
    last_error = None

    while time.monotonic() < deadline:
        try:
            with serial.Serial(
                port=port, baudrate=baud, timeout=RAW_SERIAL_TIMEOUT,
                write_timeout=RAW_SERIAL_TIMEOUT, exclusive=True
            ) as raw_serial:
                raw_serial.reset_input_buffer()
                raw_serial.reset_output_buffer()
                return
        except (OSError, IOError, serial.SerialException) as e:
            last_error = e
            time.sleep(0.1)

    if last_error is None:
        raise UpdateError("Unable to open serial port %s" % (port,))
    raise UpdateError("Unable to access serial port %s: %s" % (port, last_error))


def send_boot_token(port, baud):
    with serial.Serial(
        port=port, baudrate=baud, timeout=RAW_SERIAL_TIMEOUT,
        write_timeout=RAW_SERIAL_TIMEOUT, exclusive=True
    ) as raw_serial:
        raw_serial.reset_input_buffer()
        raw_serial.reset_output_buffer()
        output_line("Sending bootloader token")
        for _ in range(BOOT_TOKEN_BURST_COUNT):
            raw_serial.write(BOOT_TOKEN)
            raw_serial.flush()
            time.sleep(BOOT_TOKEN_BURST_DELAY)


def run_pymdfu(image, port, baud):
    args = find_pymdfu_command() + [
        "update", "--verbose", "debug", "--tool", "serial",
        "--image", image, "--port", port, "--baudrate", str(baud)
    ]
    output_line("Running: %s" % (" ".join(args),))
    try:
        subprocess.run(args, check=True)
    except subprocess.CalledProcessError as e:
        raise UpdateError("pymdfu failed with exit code %d" % (e.returncode,))


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Flash a dsPIC MDFU bootloader when no Klipper application is "
            "available on the MCU."
        )
    )
    parser.add_argument(
        "--port", required=True,
        help="Serial device path, for example /dev/ttyUSB0"
    )
    parser.add_argument(
        "--image", required=True,
        help="Firmware image path, for example app.X.production.bin"
    )
    parser.add_argument(
        "--baud", type=int, default=460800,
        help="Serial baudrate (default: 460800)"
    )
    parser.add_argument(
        "--boot-wait", type=float, default=DEFAULT_BOOT_WAIT,
        help=(
            "Seconds to wait before probing the port and starting pymdfu "
            "(default: 1.2)"
        )
    )
    parser.add_argument(
        "--port-wait", type=float, default=DEFAULT_PORT_WAIT,
        help=(
            "Seconds to wait for the serial port to become available "
            "(default: 5.0)"
        )
    )
    parser.add_argument(
        "--send-token", action="store_true",
        help=(
            "Send the raw bootloader token before flashing. Use this only if "
            "you are manually resetting into the 1s boot-entry window."
        )
    )
    args = parser.parse_args()

    try:
        require_pyserial()
        if not os.path.exists(args.image):
            raise UpdateError("Firmware image not found: %s" % (args.image,))
        output_line(
            "Waiting %.2fs for the bootloader to become ready" % (args.boot_wait,)
        )
        time.sleep(args.boot_wait)
        wait_for_port(args.port, args.baud, args.port_wait)
        if args.send_token:
            send_boot_token(args.port, args.baud)
        run_pymdfu(args.image, args.port, args.baud)
    except UpdateError as e:
        output_line("Update error: %s" % (e,))
        sys.exit(-1)


if __name__ == "__main__":
    main()
