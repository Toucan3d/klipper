# Launch and supervise the external CTC ILC concentricity calibration
#
# The calibration algorithm runs as a separate host process (a compiled
# standalone binary, see ctc/README.md at the repository root). It talks
# to Klipper through the API server Unix domain socket, so this module
# only starts the process, relays its progress output to the console and
# stages the resulting lookup table into the [ctc] config section.
#
# The G-Code handlers here must never block on the subprocess: its
# G-Code arrives through the same gcode mutex the handlers hold, so a
# blocking handler would deadlock the calibration's first move. All
# supervision happens through reactor fd and timer callbacks.
#
# Copyright (C) 2026
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os, subprocess, json, logging

ABORT_GRACE_TIME = 5.
CHECK_INTERVAL = 1.
SESSION_DIR_SENTINEL = 'ILC_SESSION_DIR '
LOOKUP_OPTIONS = ('lookup_a', 'lookup_dx', 'lookup_dy')

class CTCILCLauncher:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        default_program = os.path.normpath(os.path.join(
            os.path.dirname(__file__), '..', '..', 'ctc',
            'ctc_ilc_auto-' + os.uname().machine))
        self.program = os.path.expanduser(
            config.get('program', default_program))
        self.output_dir = os.path.expanduser(
            config.get('output_dir', '~/ctc_ilc_sessions'))
        self.serial_port = config.get('serial_port', '/dev/ttyACM1')
        self.run_timeout = config.getfloat('timeout', 21600., above=0.)
        self.result_variant = config.getchoice(
            'result_variant', ['sign_inverted', 'direct'],
            'sign_inverted')
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
        self.printer.register_event_handler('klippy:disconnect',
                                            self._handle_disconnect)
        for name in ('CALIBRATE', 'STATUS', 'ABORT'):
            cmd = 'CTC_ILC_' + name
            self.gcode.register_command(
                cmd, getattr(self, 'cmd_' + cmd),
                desc=getattr(self, 'cmd_%s_help' % (cmd,)))
    def get_status(self, eventtime):
        elapsed = 0.
        if self.proc is not None:
            elapsed = self.reactor.monotonic() - self.start_time
        elif self.end_time:
            elapsed = self.end_time - self.start_time
        return {'state': self.state, 'session_dir': self.session_dir,
                'last_line': self.last_line,
                'returncode': self.returncode, 'elapsed': elapsed}
    cmd_CTC_ILC_CALIBRATE_help = (
        "Start the autonomous CTC ILC concentricity calibration")
    def cmd_CTC_ILC_CALIBRATE(self, gcmd):
        if self.proc is not None:
            raise gcmd.error("An ILC calibration is already running"
                             " (CTC_ILC_STATUS / CTC_ILC_ABORT)")
        ctc = self.printer.lookup_object('ctc', None)
        if ctc is not None and ctc.is_active:
            if not gcmd.get_int('DISABLE_LOOKUP', 0):
                raise gcmd.error(
                    "ctc lookup compensation is active. The calibration"
                    " must run without compensation - rerun with"
                    " DISABLE_LOOKUP=1 to disable it for this session")
            ctc.clear_table()
            gcmd.respond_info(
                "ctc lookup compensation disabled for this run")
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
        argv = [self.program, '--non-interactive',
                '--socket', apiserver,
                '--serial-port', self.serial_port,
                '--output-dir', self.output_dir]
        try:
            self.proc = subprocess.Popen(
                argv, cwd=self.output_dir, stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        except OSError as e:
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
            "ILC calibration started (pid %d). The calibration moves"
            " the printer - do not send motion G-Code until it"
            " finishes." % (self.proc.pid,))
    cmd_CTC_ILC_STATUS_help = "Report the ILC calibration state"
    def cmd_CTC_ILC_STATUS(self, gcmd):
        status = self.get_status(self.reactor.monotonic())
        msg = ["ILC state: %s" % (status['state'],)]
        if status['elapsed']:
            msg.append("elapsed: %.0fs" % (status['elapsed'],))
        if status['session_dir'] is not None:
            msg.append("session: %s" % (status['session_dir'],))
        if status['returncode'] is not None:
            msg.append("exit code: %d" % (status['returncode'],))
        if status['last_line']:
            msg.append("last output: %s" % (status['last_line'],))
        gcmd.respond_info("\n".join(msg))
    cmd_CTC_ILC_ABORT_help = "Abort a running ILC calibration"
    def cmd_CTC_ILC_ABORT(self, gcmd):
        if self.proc is None:
            raise gcmd.error("No ILC calibration is running")
        self.abort_requested = True
        self.state = 'aborting'
        self.abort_deadline = self.reactor.monotonic() + ABORT_GRACE_TIME
        self._terminate()
        gcmd.respond_info(
            "Aborting ILC calibration. Queued moves still finish;"
            " reposition the printer manually afterwards.")
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
            if text.startswith(SESSION_DIR_SENTINEL):
                self.session_dir = text[len(SESSION_DIR_SENTINEL):].strip()
            self.last_line = text
            self._respond("ILC: " + text)
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
            self._respond("ILC: maximum runtime (%.0fs) exceeded;"
                          " terminating" % (self.run_timeout,))
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
            self._respond("ILC calibration killed after exceeding the"
                          " configured timeout")
        elif self.abort_requested:
            self.state = 'aborted'
            self._respond("ILC calibration aborted (exit code %d)"
                          % (rc,))
        elif rc == 0:
            try:
                self._stage_result()
                self.state = 'complete'
            except Exception as e:
                logging.exception("ctc_ilc: result staging failed")
                self.state = 'error'
                self._respond("ILC: calibration finished but the result"
                              " could not be applied: %s" % (e,))
        else:
            self.state = 'error'
            self._respond("ILC calibration failed (exit code %d)."
                          " Last output: %s" % (rc, self.last_line))
    def _parse_lookup_file(self, path):
        options = {}
        with open(path, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or ':' not in line:
                    continue
                name, value = line.split(':', 1)
                options[name.strip()] = value.strip()
        missing = [n for n in LOOKUP_OPTIONS if n not in options]
        if missing:
            raise ValueError("%s is missing options: %s"
                             % (path, ", ".join(missing)))
        parsed = {n: [float(p) for p in options[n].split(',')]
                  for n in LOOKUP_OPTIONS}
        counts = {len(v) for v in parsed.values()}
        if len(counts) != 1 or not min(counts):
            raise ValueError("%s has inconsistent or empty lookup"
                             " columns" % (path,))
        return options, parsed
    def _stage_result(self):
        if self.session_dir is None:
            raise ValueError("the calibration never reported its"
                             " session directory")
        summary_path = os.path.join(self.session_dir,
                                    'session_summary.json')
        try:
            with open(summary_path, 'r') as f:
                summary = json.load(f)
        except (IOError, OSError, ValueError):
            logging.exception("ctc_ilc: unable to read %s", summary_path)
            summary = {}
        msg = ["ILC calibration complete"]
        if summary.get('stop_reason'):
            msg.append("stop reason: %s" % (summary['stop_reason'],))
        if summary.get('raw_reduction_percent') is not None:
            msg.append("control-node raw symmetry reduction: %.2f%%"
                       % (summary['raw_reduction_percent'],))
        if summary.get('full_5deg_raw_reduction_percent') is not None:
            msg.append("full 5-degree raw symmetry reduction: %.2f%%"
                       % (summary['full_5deg_raw_reduction_percent'],))
        msg.append("session data: %s" % (self.session_dir,))
        self._respond("\n".join(msg))
        lookup_path = os.path.join(
            self.session_dir,
            'final_lookup_%s.cfg' % (self.result_variant,))
        raw_options, parsed = self._parse_lookup_file(lookup_path)
        ctc = self.printer.lookup_object('ctc', None)
        if ctc is not None:
            ctc.set_table(parsed['lookup_a'], parsed['lookup_dx'],
                          parsed['lookup_dy'])
            self._respond("New lookup table applied to [ctc] for this"
                          " session (%d points)"
                          % (len(parsed['lookup_a']),))
        else:
            self._respond("Warning: no [ctc] section is configured;"
                          " the lookup takes effect after SAVE_CONFIG"
                          " and restart")
        configfile = self.printer.lookup_object('configfile')
        for name in LOOKUP_OPTIONS:
            configfile.set('ctc', name, raw_options[name])
        self._respond(
            "The SAVE_CONFIG command will update the printer config"
            " file with the new lookup table and restart the printer."
            " If SAVE_CONFIG reports a conflict, remove the lookup_a/"
            "lookup_dx/lookup_dy options from the [ctc] section of"
            " printer.cfg first.")

def load_config(config):
    return CTCILCLauncher(config)
