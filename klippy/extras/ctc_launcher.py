# Shared launcher for the external CTC calibration programs
#
# The calibration algorithms run as separate host processes (compiled
# standalone binaries, see ctc/README.md at the repository root). They
# talk to Klipper through the API server Unix domain socket, so the
# launcher only walks the operator through the preparation prompts,
# starts the process, relays its report lines to the console and hands
# the result to the module-specific staging hook.
#
# The G-Code handlers here must never block on the subprocess: its
# G-Code arrives through the same gcode mutex the handlers hold, so a
# blocking handler would deadlock the calibration's first move. All
# supervision happens through reactor fd and timer callbacks.
#
# Copyright (C) 2026
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os, sys, subprocess, logging

ABORT_GRACE_TIME = 5.
CHECK_INTERVAL = 1.
SESSION_DIR_SENTINEL = 'ILC_SESSION_DIR '
REPORT_PREFIX = 'ILC_REPORT '
PENDING_STATES = ('await_filament', 'await_device')

def host_arch():
    # Name of the binary variant matching the *userland*: a 32-bit OS on a
    # 64-bit kernel (common on Raspberry Pi) reports aarch64 from uname but
    # needs the armv7l build.
    machine = os.uname().machine
    if machine.startswith(('aarch64', 'arm')):
        return 'aarch64' if sys.maxsize > 2**32 else 'armv7l'
    return machine

class CalibrationLauncher:
    # Subclasses set these and implement the hooks below.
    command_prefix = None   # e.g. 'CTC_ILC' -> CTC_ILC_CALIBRATE, ...
    program_stem = None     # binary name in ctc/, without -<arch>
    default_output_dir = None
    label = None            # human name used in console messages
    console_tag = None      # prefix of relayed report lines
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        default_program = os.path.normpath(os.path.join(
            os.path.dirname(__file__), '..', '..', 'ctc',
            self.program_stem + '-' + host_arch()))
        self.program = os.path.expanduser(
            config.get('program', default_program))
        self.output_dir = os.path.expanduser(
            config.get('output_dir', self.default_output_dir))
        self.serial_port = config.get('serial_port', '/dev/ttyACM1')
        self.run_timeout = config.getfloat('timeout', 21600., above=0.)
        self.proc = None
        self.fd_handle = None
        self.check_timer = None
        self.partial_output = b""
        self.state = 'idle'
        self.session_dir = None
        self.last_line = ''
        self.returncode = None
        self.start_time = self.end_time = 0.
        self.abort_requested = self.timed_out = False
        self.abort_deadline = None
        self.apiserver = None
        self.printer.register_event_handler('klippy:disconnect',
                                            self._handle_disconnect)
        for name in ('CALIBRATE', 'CONTINUE', 'STATUS', 'ABORT'):
            cmd = self.command_prefix + '_' + name
            self.gcode.register_command(
                cmd, getattr(self, 'cmd_' + name),
                desc=getattr(self, 'cmd_%s_help' % (name,)) % (self.label,))
    # Hooks for subclasses
    def prepare(self, gcmd):
        # Called by CALIBRATE after the generic checks; raise gcmd.error
        # to refuse the run. Store any command parameters needed later.
        pass
    def program_args(self):
        # Extra command line arguments for the calibration program.
        return []
    def before_start(self):
        # Called right before the process is spawned.
        pass
    def stage_result(self):
        # Called when the process exited with code 0 (session_dir is
        # known). Raise to report a staging failure.
        pass
    def cleanup(self):
        # Called once after every run end (complete, error, aborted).
        pass
    def get_status(self, eventtime):
        elapsed = 0.
        if self.proc is not None:
            elapsed = self.reactor.monotonic() - self.start_time
        elif self.end_time:
            elapsed = self.end_time - self.start_time
        return {'state': self.state, 'session_dir': self.session_dir,
                'last_line': self.last_line,
                'returncode': self.returncode, 'elapsed': elapsed}
    cmd_CALIBRATE_help = "Start the %s calibration"
    def cmd_CALIBRATE(self, gcmd):
        if self.proc is not None or self.state in PENDING_STATES:
            raise gcmd.error("A %s calibration is already in progress"
                             " (%s_STATUS / %s_ABORT)"
                             % (self.label, self.command_prefix,
                                self.command_prefix))
        bed_mesh = self.printer.lookup_object('bed_mesh', None)
        if bed_mesh is not None and bed_mesh.get_mesh() is not None:
            if not gcmd.get_int('DISABLE_MESH', 0):
                raise gcmd.error(
                    "A bed mesh is active. The calibration must run"
                    " without mesh compensation - rerun with"
                    " DISABLE_MESH=1 to clear it for this session")
            self.gcode.run_script_from_command("BED_MESH_CLEAR")
            gcmd.respond_info(
                "bed mesh compensation disabled for this run")
        apiserver = self.printer.get_start_args().get('apiserver')
        if not apiserver:
            raise gcmd.error("Klipper was started without an API server"
                             " socket (-a); the calibration cannot"
                             " connect")
        if not (os.path.isfile(self.program)
                and os.access(self.program, os.X_OK)):
            raise gcmd.error("Calibration program not found or not"
                             " executable: %s" % (self.program,))
        try:
            os.makedirs(self.output_dir, exist_ok=True)
        except OSError as e:
            raise gcmd.error("Unable to create output directory %s: %s"
                             % (self.output_dir, e))
        self.prepare(gcmd)
        self.apiserver = apiserver
        self.state = 'await_filament'
        self._prompt("Unload filament",
                     "Unload the filament before the calibration.",
                     "When it is unloaded, continue.")
    def _prompt(self, title, *lines):
        # Mainsail/Fluidd render action prompts as a dialog; the plain
        # text covers the console and other front ends.
        raw = self.gcode.respond_raw
        raw("// action:prompt_begin " + title)
        for line in lines:
            raw("// action:prompt_text " + line)
        raw("// action:prompt_footer_button Continue|%s_CONTINUE"
            % (self.command_prefix,))
        raw("// action:prompt_footer_button Cancel|%s_ABORT|error"
            % (self.command_prefix,))
        raw("// action:prompt_show")
        self._respond("%s: %s Send %s_CONTINUE to continue or"
                      " %s_ABORT to cancel."
                      % (title, " ".join(lines), self.command_prefix,
                         self.command_prefix))
    def _prompt_end(self):
        self.gcode.respond_raw("// action:prompt_end")
    cmd_CONTINUE_help = "Confirm the current %s calibration step"
    def cmd_CONTINUE(self, gcmd):
        if self.state == 'await_filament':
            self.state = 'await_device'
            self._prompt("Place the calibration device",
                         "Place the CTC calibration device under the CTC"
                         " beacon with 1-2 mm clearance.",
                         "The measurement starts when you continue.")
            return
        if self.state != 'await_device':
            raise gcmd.error("No %s calibration step is waiting for"
                             " confirmation" % (self.label,))
        self._prompt_end()
        self._start_process(gcmd)
    def _start_process(self, gcmd):
        argv = [self.program, '--non-interactive',
                '--socket', self.apiserver,
                '--serial-port', self.serial_port,
                '--output-dir', self.output_dir]
        argv.extend(self.program_args())
        self.before_start()
        try:
            self.proc = subprocess.Popen(
                argv, cwd=self.output_dir, stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        except OSError as e:
            self.state = 'idle'
            self.cleanup()
            raise gcmd.error("Unable to start calibration: %s" % (e,))
        os.set_blocking(self.proc.stdout.fileno(), False)
        self.fd_handle = self.reactor.register_fd(
            self.proc.stdout.fileno(), self._handle_output)
        self.partial_output = b""
        self.state = 'running'
        self.session_dir = None
        self.last_line = ''
        self.returncode = None
        self.abort_requested = self.timed_out = False
        self.abort_deadline = None
        self.start_time = self.reactor.monotonic()
        self.end_time = 0.
        self.check_timer = self.reactor.register_timer(
            self._check_event, self.start_time + CHECK_INTERVAL)
        gcmd.respond_info(
            "%s calibration started. It moves the printer - do not"
            " send motion G-Code until it finishes." % (self.label,))
    cmd_STATUS_help = "Report the %s calibration state"
    def cmd_STATUS(self, gcmd):
        status = self.get_status(self.reactor.monotonic())
        msg = ["%s state: %s" % (self.label, status['state'])]
        if status['elapsed']:
            msg.append("elapsed: %.0fs" % (status['elapsed'],))
        if status['session_dir'] is not None:
            msg.append("session: %s" % (status['session_dir'],))
        if status['returncode'] is not None:
            msg.append("exit code: %d" % (status['returncode'],))
        if status['last_line']:
            msg.append("last output: %s" % (status['last_line'],))
        gcmd.respond_info("\n".join(msg))
    cmd_ABORT_help = "Abort a running %s calibration"
    def cmd_ABORT(self, gcmd):
        if self.state in PENDING_STATES:
            self._prompt_end()
            self.state = 'idle'
            gcmd.respond_info("%s calibration cancelled" % (self.label,))
            return
        if self.proc is None:
            raise gcmd.error("No %s calibration is running" % (self.label,))
        self.abort_requested = True
        self.state = 'aborting'
        self.abort_deadline = self.reactor.monotonic() + ABORT_GRACE_TIME
        self._terminate()
        gcmd.respond_info(
            "Aborting %s calibration. Queued moves still finish;"
            " reposition the printer manually afterwards." % (self.label,))
    def _respond(self, msg):
        self.gcode.respond_info(msg)
    def _terminate(self, kill=False):
        if self.proc is None:
            return
        try:
            if kill:
                self.proc.kill()
            else:
                self.proc.terminate()
        except OSError:
            pass
    def _handle_disconnect(self):
        self._terminate(kill=True)
    def _handle_output(self, eventtime):
        if self.proc is None:
            return
        try:
            data = os.read(self.proc.stdout.fileno(), 4096)
        except BlockingIOError:
            return
        except OSError:
            data = b""
        if not data:
            # EOF - the check timer reaps the process
            if self.fd_handle is not None:
                self.reactor.unregister_fd(self.fd_handle)
                self.fd_handle = None
            return
        self.partial_output += data
        self._emit_lines()
    def _emit_lines(self):
        while b'\n' in self.partial_output:
            line, self.partial_output = self.partial_output.split(b'\n', 1)
            text = line.decode('utf-8', 'replace').rstrip()
            if not text:
                continue
            logging.info("%s: %s", self.command_prefix.lower(), text)
            if text.startswith(SESSION_DIR_SENTINEL):
                self.session_dir = text[len(SESSION_DIR_SENTINEL):].strip()
                continue
            self.last_line = text
            if text.startswith(REPORT_PREFIX):
                self._respond(self.console_tag + ": "
                              + text[len(REPORT_PREFIX):])
    def _check_event(self, eventtime):
        if self.proc is None:
            return self.reactor.NEVER
        if self.proc.poll() is not None:
            self._finalize()
            return self.reactor.NEVER
        if self.state == 'aborting':
            if (self.abort_deadline is not None
                    and eventtime >= self.abort_deadline):
                self._terminate(kill=True)
                self.abort_deadline = None
        elif eventtime - self.start_time > self.run_timeout:
            self.timed_out = True
            self.state = 'aborting'
            self.abort_deadline = eventtime + ABORT_GRACE_TIME
            self._respond("%s: maximum runtime (%.0fs) exceeded;"
                          " terminating" % (self.console_tag,
                                            self.run_timeout))
            self._terminate()
        return eventtime + CHECK_INTERVAL
    def _drain_output(self):
        if self.proc is None or self.proc.stdout is None:
            return
        while True:
            try:
                data = os.read(self.proc.stdout.fileno(), 4096)
            except (BlockingIOError, OSError):
                break
            if not data:
                break
            self.partial_output += data
        if self.partial_output:
            self.partial_output += b'\n'
            self._emit_lines()
            self.partial_output = b""
        try:
            self.proc.stdout.close()
        except OSError:
            pass
    def _finalize(self):
        rc = self.proc.returncode
        self._drain_output()
        if self.fd_handle is not None:
            self.reactor.unregister_fd(self.fd_handle)
            self.fd_handle = None
        if self.check_timer is not None:
            self.reactor.unregister_timer(self.check_timer)
            self.check_timer = None
        self.returncode = rc
        self.end_time = self.reactor.monotonic()
        self.proc = None
        if self.timed_out:
            self.state = 'error'
            self._respond("%s calibration killed after exceeding the"
                          " configured timeout" % (self.label,))
        elif self.abort_requested:
            self.state = 'aborted'
            self._respond("%s calibration aborted (exit code %d)"
                          % (self.label, rc))
        elif rc == 0:
            try:
                if self.session_dir is None:
                    raise ValueError("the calibration never reported its"
                                     " session directory")
                self.stage_result()
                self.state = 'complete'
            except Exception as e:
                logging.exception("%s: result staging failed",
                                  self.command_prefix.lower())
                self.state = 'error'
                self._respond("%s: calibration finished but the result"
                              " could not be applied: %s"
                              % (self.console_tag, e))
        else:
            self.state = 'error'
            self._respond("%s calibration failed (exit code %d)."
                          " Last output: %s"
                          % (self.label, rc, self.last_line))
        try:
            self.cleanup()
        except Exception:
            logging.exception("%s: cleanup failed",
                              self.command_prefix.lower())
