#!/usr/bin/env python3
# Tool to reset a Klipper MCU, enter the dsPIC MDFU bootloader, and flash it
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import argparse
import logging
import os
import select
import shutil
import subprocess
import sys
import time

try:
    import serial
except ModuleNotFoundError:
    serial = None

KLIPPER_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../"))
sys.path.append(os.path.join(KLIPPER_DIR, "klippy"))

RESET_CMD = "reset"
BOOT_TOKEN = b"KLIPPER_BOOT33\n"
BOOT_TOKEN_BURST_COUNT = 10
BOOT_TOKEN_BURST_DELAY = 0.02
RAW_SERIAL_OPEN_TIMEOUT = 0.5
RESET_SETTLE_TIME = 0.05
DEFAULT_KLIPPER_BAUD = 250000
DEFAULT_BOOTLOADER_BAUD = 460800
DEFAULT_CONNECT_TIMEOUT = 5.0


class UpdateError(Exception):
    pass


def output_line(msg):
    sys.stdout.write("%s\n" % (msg,))
    sys.stdout.flush()


def output(msg):
    sys.stdout.write("%s" % (msg,))
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


def send_klipper_reset(port, klipper_baud, connect_timeout):
    console_script = os.path.join(KLIPPER_DIR, "klippy", "console.py")
    cmd = [sys.executable, console_script, "-b", str(klipper_baud), port]
    recent_lines = []

    output_line(
        "Connecting to Klipper MCU on %s at %d baud" % (port, klipper_baud)
    )
    proc = subprocess.Popen(
        cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True, bufsize=1
    )
    try:
        deadline = time.monotonic() + connect_timeout
        connected = False
        while time.monotonic() < deadline:
            ready, _, _ = select.select([proc.stdout], [], [], 0.2)
            if not ready:
                if proc.poll() is not None:
                    break
                output(".")
                continue
            line = proc.stdout.readline()
            if not line:
                if proc.poll() is not None:
                    break
                continue
            recent_lines.append(line.rstrip())
            recent_lines = recent_lines[-10:]
            if "connected" in line.lower():
                connected = True
                break

        if not connected:
            raise UpdateError(
                "Unable to connect via console.py%s" % (
                    "" if not recent_lines else ": " + recent_lines[-1],)
            )

        output_line(" connected")
        output_line("Sending Klipper reset command")
        proc.stdin.write(RESET_CMD + "\n")
        proc.stdin.flush()
        time.sleep(RESET_SETTLE_TIME)
    except BrokenPipeError as e:
        raise UpdateError("Unable to send Klipper reset command: %s" % (e,))
    finally:
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=1.0)


def send_boot_token(port, bootloader_baud):
    deadline = time.monotonic() + 1.0
    last_error = None

    while time.monotonic() < deadline:
        try:
            with serial.Serial(
                port=port, baudrate=bootloader_baud,
                timeout=RAW_SERIAL_OPEN_TIMEOUT,
                write_timeout=RAW_SERIAL_OPEN_TIMEOUT,
                exclusive=True
            ) as raw_serial:
                raw_serial.reset_input_buffer()
                output_line("Sending bootloader token")
                for _ in range(BOOT_TOKEN_BURST_COUNT):
                    raw_serial.write(BOOT_TOKEN)
                    raw_serial.flush()
                    time.sleep(BOOT_TOKEN_BURST_DELAY)
                return
        except (OSError, IOError, serial.SerialException) as e:
            last_error = e
            time.sleep(0.02)

    if last_error is None:
        raise UpdateError("Unable to open serial port %s for raw token send" % (
            port,))
    raise UpdateError("Unable to send bootloader token: %s" % (last_error,))


def run_pymdfu(image, port, bootloader_baud):
    args = find_pymdfu_command() + [
        "update", "--verbose", "debug", "--tool", "serial",
        "--image", image, "--port", port, "--baudrate", str(bootloader_baud)
    ]
    output_line("Running: %s" % (" ".join(args),))
    try:
        subprocess.run(args, check=True)
    except subprocess.CalledProcessError as e:
        raise UpdateError("pymdfu failed with exit code %d" % (e.returncode,))


def main():
    parser = argparse.ArgumentParser(
        description="Reset a Klipper dsPIC MCU, enter MDFU mode, and flash it."
    )
    parser.add_argument(
        "--port", required=True, help="Serial device path, for example /dev/ttyUSB0"
    )
    parser.add_argument(
        "--image", required=True, help="Firmware image path, for example app.X.production.bin"
    )
    parser.add_argument(
        "--klipper-baud", type=int, default=DEFAULT_KLIPPER_BAUD,
        help="Klipper protocol baudrate for the reset command (default: 250000)"
    )
    parser.add_argument(
        "--baud", type=int, default=DEFAULT_BOOTLOADER_BAUD,
        help="Bootloader and pymdfu baudrate (default: 460800)"
    )
    parser.add_argument(
        "--connect-timeout", type=float, default=DEFAULT_CONNECT_TIMEOUT,
        help="Seconds to wait for console.py to connect to Klipper (default: 5.0)"
    )
    args = parser.parse_args()

    logging.basicConfig(level=logging.CRITICAL)

    try:
        require_pyserial()
        if not os.path.exists(args.image):
            raise UpdateError("Firmware image not found: %s" % (args.image,))
        send_klipper_reset(args.port, args.klipper_baud, args.connect_timeout)
        send_boot_token(args.port, args.baud)
        run_pymdfu(args.image, args.port, args.baud)
    except UpdateError as e:
        output_line("Update error: %s" % (e,))
        sys.exit(-1)


if __name__ == "__main__":
    main()
