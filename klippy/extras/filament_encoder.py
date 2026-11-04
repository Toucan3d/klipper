# Filament length, speed and flow measurement with a motion encoder
#
# Copyright (C) 2026  Toucan3D
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import collections, logging, math, os, re, time
from . import pulse_counter

DEFAULT_ENCODER_KEY = None
WAIT_TIMEOUT = 2.


######################################################################
# Monitor run
######################################################################

class MonitorRun:
    def __init__(self, label, interval):
        self.label = label
        self.interval = interval
        self.start_time = self.stop_time = None
        self.start_edges = self.start_cmd = None
        self.flow_min = self.flow_max = None
        self.done = False
        self.result = None
        self.status_timer = None
        self.log_file = None
        self.log_name = None
    def is_started(self):
        return self.start_edges is not None


######################################################################
# Encoder
######################################################################

class FilamentEncoder:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name().split()[-1]
        self.mm_per_edge = config.getfloat('mm_per_edge', above=0.)
        diameter = config.getfloat('filament_diameter', 1.75, above=0.)
        self.area = math.pi * diameter * diameter / 4.
        self.speed_window = config.getfloat('speed_window', 1., above=0.)
        self.sample_time = config.getfloat('sample_time', 0.010, above=0.)
        poll_time = config.getfloat('poll_time', 0.0002, above=0.)
        self.history_time = config.getfloat(
            'history_time', 60., above=self.speed_window)
        self.extruder_name = config.get('extruder', None)
        self.extruder = None
        # Sample state
        self.history = collections.deque()
        self.base_count = None
        self.abs_base = self.rel_base = 0.
        self.last_epos = None
        self.cmd_total = 0.
        # Monitor state
        self.run = None
        self.finishing = []
        self.last_result = None
        # Counter on the micro controller
        self.counter = pulse_counter.MCU_counter(
            self.printer, config.get('pin'), self.sample_time, poll_time)
        self.counter.setup_callback(self._counter_callback)
        # Events and commands
        self.printer.register_event_handler("klippy:ready",
                                            self._handle_ready)
        self.printer.register_event_handler("klippy:shutdown",
                                            self._handle_shutdown)
        self.gcode = self.printer.lookup_object('gcode')
        cmds = [('ENCODER_QUERY', self.cmd_ENCODER_QUERY,
                 self.cmd_ENCODER_QUERY_help),
                ('ENCODER_RESET', self.cmd_ENCODER_RESET,
                 self.cmd_ENCODER_RESET_help),
                ('FLOW_MONITOR_START', self.cmd_FLOW_MONITOR_START,
                 self.cmd_FLOW_MONITOR_START_help),
                ('FLOW_MONITOR_STOP', self.cmd_FLOW_MONITOR_STOP,
                 self.cmd_FLOW_MONITOR_STOP_help)]
        mux = self.gcode.mux_commands.get('ENCODER_QUERY')
        is_first = mux is None or DEFAULT_ENCODER_KEY not in mux[1]
        for cmd, func, desc in cmds:
            self.gcode.register_mux_command(cmd, 'ENCODER', self.name,
                                            func, desc=desc)
            if is_first:
                self.gcode.register_mux_command(cmd, 'ENCODER',
                                                DEFAULT_ENCODER_KEY, func)
    def _handle_ready(self):
        if self.extruder_name is None:
            return
        extruder = self.printer.lookup_object(self.extruder_name, None)
        if extruder is None or not hasattr(extruder, 'find_past_position'):
            raise self.printer.config_error(
                "filament_encoder %s: unknown extruder '%s'"
                % (self.name, self.extruder_name))
        self.extruder = extruder
    def _handle_shutdown(self):
        for run in [self.run] + self.finishing:
            if run is not None:
                self._stop_status_timer(run)
                self._close_log(run)
    # Sample handling
    def _counter_callback(self, print_time, count, count_time):
        # Called in the serial thread, hand over to the main thread
        self.reactor.register_async_callback(
            (lambda e, t=print_time, c=count: self.handle_sample(t, c)))
    def handle_sample(self, print_time, count):
        if self.base_count is None:
            # The first report may contain an edge from the initial pin
            # level, it only sets the baseline
            self.base_count = count
        edges = count - self.base_count
        if self.extruder is not None:
            epos = self.extruder.find_past_position(print_time)
            if self.last_epos is not None:
                self.cmd_total += abs(epos - self.last_epos)
            self.last_epos = epos
        history = self.history
        history.append((print_time, edges, self.cmd_total))
        min_time = print_time - self.history_time
        while len(history) > 2 and history[0][0] < min_time:
            history.popleft()
        for run in [self.run] + list(self.finishing):
            if run is not None:
                self._update_run(run, print_time)
    def count_at(self, print_time):
        # Interpolated (edges, commanded) at a print time
        history = self.history
        if not history:
            return None
        if print_time >= history[-1][0]:
            return history[-1][1], history[-1][2]
        if print_time <= history[0][0]:
            return history[0][1], history[0][2]
        later = history[-1]
        for i in range(len(history) - 2, -1, -1):
            sample = history[i]
            if sample[0] <= print_time:
                dt = later[0] - sample[0]
                if dt <= 0.:
                    return sample[1], sample[2]
                f = (print_time - sample[0]) / dt
                return (sample[1] + f * (later[1] - sample[1]),
                        sample[2] + f * (later[2] - sample[2]))
            later = sample
        return history[0][1], history[0][2]
    def get_edges(self):
        if not self.history:
            return 0
        return self.history[-1][1]
    def get_speed(self):
        history = self.history
        if len(history) < 2:
            return 0.
        t1, e1, c1 = history[-1]
        older = history[0]
        min_time = t1 - self.speed_window
        for i in range(len(history) - 2, -1, -1):
            if history[i][0] <= min_time:
                older = history[i]
                break
        dt = t1 - older[0]
        if dt <= 0.:
            return 0.
        return (e1 - older[1]) * self.mm_per_edge / dt
    def get_flow(self):
        return self.get_speed() * self.area
    # Monitor handling
    def _run_values(self, run, print_time):
        edges, cmd = self.count_at(print_time)
        length = (edges - run.start_edges) * self.mm_per_edge
        elapsed = print_time - run.start_time
        flow_avg = None
        if elapsed > 0.:
            flow_avg = length * self.area / elapsed
        commanded = ratio = None
        if self.extruder is not None:
            commanded = cmd - run.start_cmd
            if commanded > 0.:
                ratio = 100. * length / commanded
        return {'edges': edges - run.start_edges, 'length': length,
                'elapsed': elapsed, 'flow_avg': flow_avg,
                'commanded': commanded, 'ratio': ratio}
    def _update_run(self, run, print_time):
        if run.done or run.start_time is None:
            return
        if print_time < run.start_time:
            return
        if not run.is_started():
            run.start_edges, run.start_cmd = self.count_at(run.start_time)
            if run is self.run:
                self.rel_base = run.start_edges
        if run.stop_time is not None and print_time >= run.stop_time:
            self._finish_run(run)
            return
        flow = self.get_flow()
        if print_time - self.speed_window >= run.start_time:
            if run.flow_min is None or flow < run.flow_min:
                run.flow_min = flow
            if run.flow_max is None or flow > run.flow_max:
                run.flow_max = flow
        if run.log_file is not None:
            vals = self._run_values(run, print_time)
            self._write_log(run, "%.6f,%d,%.3f,%.3f,%.3f,%s\n" % (
                print_time, round(vals['edges']), vals['length'],
                self.get_speed(), flow, self._fmt_csv(vals['commanded'])))
    def _finish_run(self, run):
        vals = self._run_values(run, run.stop_time)
        flow_min, flow_max = run.flow_min, run.flow_max
        if flow_min is None:
            # Run shorter than speed_window, use the average instead
            flow_min = flow_max = vals['flow_avg']
        result = {'active': False, 'label': run.label,
                  'elapsed': vals['elapsed'], 'duration': vals['elapsed'],
                  'length': vals['length'],
                  'edges': int(round(vals['edges'])),
                  'flow_avg': vals['flow_avg'], 'flow_min': flow_min,
                  'flow_max': flow_max, 'commanded': vals['commanded'],
                  'ratio': vals['ratio']}
        run.result = result
        run.done = True
        self.last_result = result
        self._stop_status_timer(run)
        self._close_log(run)
        if run in self.finishing:
            self.finishing.remove(run)
        if run is self.run:
            self.run = None
        msg = "FLOW %sDONE t=%.2fs len=%.1fmm edges=%d" % (
            self._fmt_label(run), result['duration'], result['length'],
            result['edges'])
        if result['commanded'] is not None:
            msg += " cmd=%.1fmm ratio=%s" % (
                result['commanded'], self._fmt_ratio(result['ratio']))
        msg += " flow_avg=%s flow_min=%s flow_max=%s" % (
            self._fmt_num(result['flow_avg']),
            self._fmt_num(flow_min), self._fmt_num(flow_max))
        self.gcode.respond_info(msg)
    def _status_event(self, eventtime):
        run = self.run
        if run is None or run.done:
            return self.reactor.NEVER
        if run.is_started() and self.history:
            print_time = self.history[-1][0]
            if print_time > run.start_time:
                vals = self._run_values(run, print_time)
                msg = "FLOW %st=%.1fs len=%.1fmm speed=%.1fmm/s" \
                      " flow=%.1fmm3/s" % (
                          self._fmt_label(run), vals['elapsed'],
                          vals['length'], self.get_speed(),
                          self.get_flow())
                if vals['commanded'] is not None:
                    msg += " cmd=%.1fmm ratio=%s" % (
                        vals['commanded'], self._fmt_ratio(vals['ratio']))
                self.gcode.respond_info(msg)
        return eventtime + run.interval
    def _stop_status_timer(self, run):
        if run.status_timer is not None:
            self.reactor.unregister_timer(run.status_timer)
            run.status_timer = None
    def _set_start_time(self, run, print_time):
        run.start_time = print_time
        if self.history and self.history[-1][0] >= print_time:
            self._update_run(run, self.history[-1][0])
    def _set_stop_time(self, run, print_time):
        if run.start_time is None:
            run.start_time = print_time
        run.stop_time = max(print_time, run.start_time)
        if self.history and self.history[-1][0] >= run.stop_time:
            self._update_run(run, self.history[-1][0])
    def _end_active_run(self, toolhead):
        run = self.run
        if run is None:
            return None
        self.run = None
        self.finishing.append(run)
        toolhead.register_lookahead_callback(
            (lambda pt, r=run: self._set_stop_time(r, pt)))
        return run
    # CSV log
    def _open_log(self, run):
        log_file = self.printer.get_start_args().get('log_file')
        if not log_file:
            self.gcode.respond_info(
                "FLOW log disabled, Klipper runs without a log file")
            return
        label = re.sub(r'[^A-Za-z0-9_.-]', '_', run.label or '')
        stamp = time.strftime("%Y%m%d_%H%M%S")
        fname = "flow_%s_%s.csv" % (label, stamp) if label \
            else "flow_%s.csv" % (stamp,)
        path = os.path.join(os.path.dirname(os.path.abspath(log_file)),
                            fname)
        try:
            run.log_file = open(path, 'w')
            run.log_file.write("print_time,edges,length_mm,speed_mm_s,"
                               "flow_mm3_s,commanded_mm\n")
            run.log_name = path
        except IOError as e:
            logging.exception("filament_encoder: unable to open log")
            self.gcode.respond_info("FLOW log error: %s" % (str(e),))
            run.log_file = None
    def _write_log(self, run, line):
        try:
            run.log_file.write(line)
        except IOError as e:
            logging.exception("filament_encoder: unable to write log")
            self.gcode.respond_info("FLOW log error: %s" % (str(e),))
            self._close_log(run)
    def _close_log(self, run):
        if run.log_file is None:
            return
        try:
            run.log_file.close()
        except IOError:
            logging.exception("filament_encoder: unable to close log")
        run.log_file = None
    # Formatting helpers
    def _fmt_label(self, run):
        if run.label:
            return run.label + " "
        return ""
    def _fmt_num(self, val):
        if val is None:
            return "-"
        return "%.1f" % (val,)
    def _fmt_ratio(self, val):
        if val is None:
            return "-"
        return "%.1f%%" % (val,)
    def _fmt_csv(self, val):
        if val is None:
            return ""
        return "%.3f" % (val,)
    # Commands
    cmd_ENCODER_QUERY_help = "Report the filament encoder length and flow"
    def cmd_ENCODER_QUERY(self, gcmd):
        edges = self.get_edges()
        gcmd.respond_info(
            "ENCODER %s absolute=%.1fmm relative=%.1fmm edges=%d"
            " relative_edges=%d speed=%.1fmm/s flow=%.1fmm3/s" % (
                self.name, (edges - self.abs_base) * self.mm_per_edge,
                (edges - self.rel_base) * self.mm_per_edge,
                round(edges - self.abs_base), round(edges - self.rel_base),
                self.get_speed(), self.get_flow()))
    cmd_ENCODER_RESET_help = "Restart the relative filament length"
    def cmd_ENCODER_RESET(self, gcmd):
        edges = self.get_edges()
        self.rel_base = edges
        if gcmd.get_int('ALL', 0, minval=0, maxval=1):
            self.abs_base = edges
        gcmd.respond_info("ENCODER %s reset" % (self.name,))
    cmd_FLOW_MONITOR_START_help = "Start a synced filament flow measurement"
    def cmd_FLOW_MONITOR_START(self, gcmd):
        interval = gcmd.get_float('INTERVAL', 1., minval=0.)
        label = gcmd.get('LABEL', '')
        do_log = gcmd.get_int('LOG', 0, minval=0, maxval=1)
        toolhead = self.printer.lookup_object('toolhead')
        old = self._end_active_run(toolhead)
        if old is not None:
            gcmd.respond_info("FLOW %sended by a new FLOW_MONITOR_START"
                              % (self._fmt_label(old),))
        run = MonitorRun(label, interval)
        self.run = run
        if do_log:
            self._open_log(run)
        if interval > 0.:
            run.status_timer = self.reactor.register_timer(
                self._status_event,
                self.reactor.monotonic() + interval)
        toolhead.register_lookahead_callback(
            (lambda pt, r=run: self._set_start_time(r, pt)))
        msg = "FLOW %sstarted" % (self._fmt_label(run),)
        if run.log_name is not None:
            msg += " log=%s" % (run.log_name,)
        gcmd.respond_info(msg)
    cmd_FLOW_MONITOR_STOP_help = "Stop the synced filament flow measurement"
    def cmd_FLOW_MONITOR_STOP(self, gcmd):
        wait = gcmd.get_int('WAIT', 0, minval=0, maxval=1)
        toolhead = self.printer.lookup_object('toolhead')
        run = self._end_active_run(toolhead)
        if run is None:
            gcmd.respond_info("FLOW no active monitor on encoder %s"
                              % (self.name,))
            return
        if not wait:
            return
        toolhead.wait_moves()
        if not getattr(toolhead, 'can_pause', True):
            return
        eventtime = self.reactor.monotonic()
        deadline = eventtime + WAIT_TIMEOUT
        while not run.done and eventtime < deadline:
            eventtime = self.reactor.pause(eventtime + self.sample_time)
        if not run.done:
            raise gcmd.error("FLOW %sno result within %.0fs, encoder"
                             " samples missing"
                             % (self._fmt_label(run), WAIT_TIMEOUT))
    def get_status(self, eventtime):
        edges = self.get_edges()
        monitor = {'active': False, 'label': None, 'elapsed': 0.,
                   'length': 0., 'flow_avg': None, 'commanded': None,
                   'ratio': None}
        run = self.run
        if run is not None:
            monitor['active'] = True
            monitor['label'] = run.label
            if run.is_started() and self.history \
                    and self.history[-1][0] > run.start_time:
                vals = self._run_values(run, self.history[-1][0])
                for key in ('elapsed', 'length', 'flow_avg', 'commanded',
                            'ratio'):
                    monitor[key] = vals[key]
        return {'absolute': (edges - self.abs_base) * self.mm_per_edge,
                'relative': (edges - self.rel_base) * self.mm_per_edge,
                'edges': int(round(edges - self.abs_base)),
                'relative_edges': int(round(edges - self.rel_base)),
                'speed': self.get_speed(), 'flow': self.get_flow(),
                'mm_per_edge': self.mm_per_edge, 'monitor': monitor,
                'last_result': self.last_result}

######################################################################
# Display sensor for [temperature_sensor]
######################################################################

class FilamentFlowSensor:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        encoder_name = config.get('encoder')
        self.value = config.getchoice('value', {'flow': 'flow',
                                                'speed': 'speed'}, 'flow')
        self.report_time = config.getfloat('report_time', 0.25, above=0.)
        self.encoder = self.printer.load_object(
            config, 'filament_encoder ' + encoder_name)
        self.min_temp = self.max_temp = 0.
        self._callback = None
        self.timer = None
        self.printer.register_event_handler("klippy:ready",
                                            self._handle_ready)
        self.printer.register_event_handler("klippy:shutdown",
                                            self._handle_shutdown)
    def _handle_ready(self):
        if self.printer.get_start_args().get('debugoutput') is not None:
            return
        self.timer = self.reactor.register_timer(self._sample_event,
                                                 self.reactor.NOW)
    def _handle_shutdown(self):
        if self.timer is not None:
            self.reactor.unregister_timer(self.timer)
            self.timer = None
    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp
    def setup_callback(self, cb):
        self._callback = cb
    def get_report_time_delta(self):
        return self.report_time
    def _sample_event(self, eventtime):
        if self.value == 'speed':
            value = self.encoder.get_speed()
        else:
            value = self.encoder.get_flow()
        if self._callback is not None:
            mcu = self.printer.lookup_object('mcu')
            self._callback(mcu.estimated_print_time(eventtime), value)
        return eventtime + self.report_time

def load_config(config):
    pheaters = config.get_printer().load_object(config, "heaters")
    pheaters.add_sensor_factory("filament_flow", FilamentFlowSensor)

def load_config_prefix(config):
    return FilamentEncoder(config)
