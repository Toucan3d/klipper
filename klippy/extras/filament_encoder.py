# Filament length, speed and flow measurement with a motion encoder or
# an angle sensor on the filament wheel
#
# Copyright (C) 2026  Toucan3D
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import collections, json, logging, math, os, re, time
from . import bulk_sensor, pulse_counter

DEFAULT_ENCODER_KEY = None
WAIT_TIMEOUT = 2.
FORCE_DELAY_BASE = 1.
# Stall samples arrive in batches every 0.5 s (bulk_sensor.BATCH_INTERVAL)
STALL_DELAY = 1.5
# Heater temperatures arrive every 0.1 to 0.3 s
TEMP_DELAY = 0.5
MIN_PULSES = 4
META_SCHEMA = 1

def trapq_state(trapq, print_time):
    # Commanded (position, velocity) of a trapq at a print time, from the
    # move that covers it or the last move before it
    import chelper
    ffi_main, ffi_lib = chelper.get_ffi()
    data = ffi_main.new('struct pull_move[1]')
    count = ffi_lib.trapq_extract_old(trapq, data, 1, 0., print_time)
    if not count:
        return None, None
    move = data[0]
    move_time = max(0., min(move.move_t, print_time - move.print_time))
    dist = (move.start_v + .5 * move.accel * move_time) * move_time
    velocity = (move.start_v + move.accel * move_time) * move.x_r
    return move.start_x + move.x_r * dist, velocity

def history_step(history, print_time):
    # Newest entry at or before a print time, else the oldest entry
    for entry in reversed(history):
        if entry[0] <= print_time:
            return entry
    if history:
        return history[0]
    return None

def history_interp(history, print_time):
    # Value linearly interpolated between the entries around a print
    # time, the nearest entry outside the covered time
    if not history:
        return None
    if print_time >= history[-1][0]:
        return history[-1][1]
    later = history[-1]
    for i in range(len(history) - 2, -1, -1):
        sample = history[i]
        if sample[0] <= print_time:
            dt = later[0] - sample[0]
            if dt <= 0.:
                return sample[1]
            f = (print_time - sample[0]) / dt
            return sample[1] + f * (later[1] - sample[1])
        later = sample
    return history[0][1]

def trim_history(history, keep_time):
    if history:
        min_time = history[-1][0] - keep_time
        while len(history) > 2 and history[0][0] < min_time:
            history.popleft()


######################################################################
# Angle sensor stream
######################################################################

ANGLE_COUNTS = 65536.
# Noise needs this many second differences per batch
MIN_NOISE_SAMPLES = 10

class AngleTrack:
    # Turns the samples of an [angle] sensor (print time, unwrapped angle
    # with 65536 counts per turn) into wheel angles averaged over a fixed
    # time grid and into virtual edges like those of an SFS: one edge per
    # deg_per_edge of travel in either direction. Travel only counts once
    # the averaged angle leaves a band of +-hysteresis degrees, so sensor
    # noise adds no edges. All angles in degrees from the first sample.
    def __init__(self, bucket_time, deg_per_edge, hysteresis):
        self.bucket_time = bucket_time
        self.deg_per_edge = deg_per_edge
        self.hysteresis = hysteresis
        self.base = None
        self.bucket_end = None
        self.sum_time = self.sum_angle = 0.
        self.count = 0
        self.angle = None
        # Hysteresis follower, its unsigned travel and the next edge
        self.follow = None
        self.travel = 0.
        self.next_edge = deg_per_edge
        self.prev_time = None
        # Noise (deg rms) from second differences of the raw samples
        self.prev = []
        self.noise = None
        self.samples = 0
    def feed(self, samples):
        # Returns [(print_time, angle, [edge_time, ...]), ...], one entry
        # per closed time bucket
        out = []
        k = 360. / ANGLE_COUNTS
        d2_sum = 0.
        d2_count = 0
        prev = self.prev
        for sample_time, counts in samples:
            angle = counts * k
            if self.base is None:
                self.base = angle
                self.bucket_end = sample_time + self.bucket_time
            angle -= self.base
            self.samples += 1
            # Second difference of evenly spaced samples: zero for a
            # constant speed, 6 times the noise variance for white noise
            prev.append((sample_time, angle))
            if len(prev) > 3:
                del prev[0]
            if len(prev) == 3:
                (t0, a0), (t1, a1), (t2, a2) = prev
                dt0, dt1 = t1 - t0, t2 - t1
                if dt0 > 0. and abs(dt1 - dt0) <= .2 * dt0:
                    d2 = a2 - 2. * a1 + a0
                    d2_sum += d2 * d2
                    d2_count += 1
            if sample_time >= self.bucket_end and self.count:
                out.append(self._close_bucket())
                while self.bucket_end <= sample_time:
                    self.bucket_end += self.bucket_time
            self.sum_time += sample_time
            self.sum_angle += angle
            self.count += 1
        if d2_count >= MIN_NOISE_SAMPLES:
            self.noise = math.sqrt(d2_sum / d2_count / 6.)
        return out
    def _close_bucket(self):
        # Mean time and mean angle of the bucket: no lag at a constant
        # speed
        bucket_time = self.sum_time / self.count
        angle = self.sum_angle / self.count
        self.sum_time = self.sum_angle = 0.
        self.count = 0
        self.angle = angle
        edges = []
        if self.follow is None:
            self.follow = angle
        else:
            diff = angle - self.follow
            step = 0.
            if diff > self.hysteresis:
                step = diff - self.hysteresis
            elif diff < -self.hysteresis:
                step = diff + self.hysteresis
            if step:
                old = self.travel
                self.follow += step
                self.travel += abs(step)
                while self.travel >= self.next_edge:
                    f = (self.next_edge - old) / (self.travel - old)
                    edges.append(self.prev_time
                                 + f * (bucket_time - self.prev_time))
                    self.next_edge += self.deg_per_edge
        self.prev_time = bucket_time
        return bucket_time, angle, edges


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
        # Log rows waiting for the delayed load cell, stall and heater
        # samples
        self.pending = collections.deque()
        # (print time of the newest edge, edges) of each encoder sample
        # with new edges, the single edge times are unknown when one
        # sample holds more than one edge
        self.edge_times = []
        # Side files next to the run CSV
        self.base_name = None
        self.force_file = self.edges_file = self.angle_file = None
        self.force_cursor = None
        self.anchors = []
        # Pulse counter edges and angle read errors at the run start, when
        # the angle sensor is the length source
        self.start_ref = None
        self.start_errors = 0
    def is_started(self):
        return self.start_edges is not None
    def in_run(self, print_time):
        return (self.start_time is not None and not self.done
                and print_time >= self.start_time
                and (self.stop_time is None or print_time <= self.stop_time))


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
        self.heater = None
        # Force source: a [load_cell] name or the full name of any object
        # with the load_cell client interface
        self.load_cell_name = config.get('load_cell', None)
        self.load_cell = None
        self.force_sps = None
        # Additional logged values
        self.mosfet_name = config.get('mosfet_sensor', None)
        self.mosfet = None
        self.a_stepper_name = config.get('a_stepper', None)
        self.a_stepper = None
        self.temp_history = collections.deque()
        self.heat_steps = collections.deque()
        self.mosfet_history = collections.deque()
        self.log_path = config.get('log_path', None)
        self.force_history = collections.deque()
        self.force_delay = 0.
        # TMC stall data of the extruder motors, streamed during logged runs
        self.stall_names = list(config.getlist('stall_steppers', (),
                                               sep=','))
        self.stall_interval = config.getfloat('stall_interval', 0.020,
                                              minval=0.001)
        self.stall_dumps = []
        self.stall_cbs = []
        self.stall_history = []
        # Sample state
        self.last_edge_time = None
        self.history = collections.deque()
        self.base_count = None
        self.abs_base = self.rel_base = 0.
        self.last_epos = None
        self.cmd_total = 0.
        # Monitor state
        self.run = None
        self.finishing = []
        self.last_result = None
        # Length source: an [angle] sensor on the filament wheel, or the
        # pulse counter of an encoder such as the SFS. With both, the
        # pulse counter edges are logged for comparison.
        pin = config.get('pin', None)
        self.angle_name = config.get('angle_sensor', None)
        if pin is None and self.angle_name is None:
            raise config.error("filament_encoder %s: set pin or angle_sensor"
                               % (self.name,))
        self.angle = self.track = None
        self.mm_per_rev = self.angle_hysteresis = None
        self.angle_errors = 0
        self.ref_history = collections.deque()
        self.ref_base = None
        if self.angle_name is not None:
            self.mm_per_rev = config.getfloat('mm_per_rev')
            if not self.mm_per_rev:
                raise config.error("filament_encoder %s: mm_per_rev must not"
                                   " be 0" % (self.name,))
            self.angle_hysteresis = config.getfloat('angle_hysteresis', 0.3,
                                                    minval=0.)
            deg_per_edge = 360. * self.mm_per_edge / abs(self.mm_per_rev)
            self.track = AngleTrack(self.sample_time, deg_per_edge,
                                    self.angle_hysteresis)
        # Counter on the micro controller
        self.counter = None
        if pin is not None:
            self.counter = pulse_counter.MCU_counter(
                self.printer, pin, self.sample_time, poll_time)
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
        if self.angle_name is not None:
            obj_name = self.angle_name
            if ' ' not in obj_name:
                obj_name = 'angle ' + obj_name
            angle = self.printer.lookup_object(obj_name, None)
            if angle is None or not hasattr(angle, 'add_client'):
                raise self.printer.config_error(
                    "filament_encoder %s: unknown angle_sensor '%s'"
                    % (self.name, self.angle_name))
            self.angle = angle
            # Starting the stream waits for SPI replies, which is not
            # allowed in a ready handler. The stream stays on: the angle
            # is the length source.
            self.reactor.register_callback(
                lambda e: angle.add_client(self._angle_callback))
        if self.load_cell_name is not None:
            # Force source: a [load_cell] or any object with add_client(cb)
            # and get_samples_per_second() (on itself or on get_sensor()).
            # cb gets {'data': [[print_time, grams, counts, ...], ...]}.
            # Optional: get_max_delay() is the time in seconds until the
            # samples of a print time have arrived, and
            # get_sync_report(start, stop) describes the timestamp quality
            # of a run for the meta file.
            obj_name = self.load_cell_name
            if ' ' not in obj_name:
                obj_name = 'load_cell ' + obj_name
            load_cell = self.printer.lookup_object(obj_name, None)
            rate_obj = load_cell
            if hasattr(load_cell, 'get_sensor'):
                rate_obj = load_cell.get_sensor()
            if (load_cell is None or not hasattr(load_cell, 'add_client')
                    or not hasattr(rate_obj, 'get_samples_per_second')):
                raise self.printer.config_error(
                    "filament_encoder %s: unknown load_cell '%s'"
                    % (self.name, self.load_cell_name))
            self.load_cell = load_cell
            sps = rate_obj.get_samples_per_second()
            self.force_sps = sps
            if hasattr(load_cell, 'get_max_delay'):
                self.force_delay = (FORCE_DELAY_BASE
                                    + load_cell.get_max_delay())
            else:
                # Bulk sensor samples arrive in blocks, wait for at most
                # two blocks
                block = bulk_sensor.MAX_BULK_MSG_SIZE // 3
                self.force_delay = FORCE_DELAY_BASE + 2. * block / sps
            load_cell.add_client(self._load_cell_callback)
        if self.mosfet_name is not None:
            mosfet = self.printer.lookup_object(self.mosfet_name, None)
            if mosfet is None or not hasattr(mosfet, 'get_status'):
                raise self.printer.config_error(
                    "filament_encoder %s: unknown mosfet_sensor '%s'"
                    % (self.name, self.mosfet_name))
            self.mosfet = mosfet
        if self.a_stepper_name is not None:
            a_stepper = self.printer.lookup_object(self.a_stepper_name, None)
            if (a_stepper is None or not hasattr(a_stepper, 'get_trapq')
                    or not hasattr(a_stepper, 'get_axis_gcode_id')):
                raise self.printer.config_error(
                    "filament_encoder %s: unknown a_stepper '%s', use a"
                    " manual_stepper" % (self.name, self.a_stepper_name))
            self.a_stepper = a_stepper
        for name in self.stall_names:
            dump = self.printer.lookup_object('tmc_stallguard ' + name, None)
            if dump is None:
                raise self.printer.config_error(
                    "filament_encoder %s: no TMC driver with stall data for"
                    " stepper '%s'" % (self.name, name))
            index = len(self.stall_dumps)
            self.stall_dumps.append(dump)
            self.stall_cbs.append(
                (lambda msg, i=index: self._stall_callback(i, msg)))
            self.stall_history.append(collections.deque())
        if self.extruder_name is None:
            return
        extruder = self.printer.lookup_object(self.extruder_name, None)
        if extruder is None or not hasattr(extruder, 'find_past_position'):
            raise self.printer.config_error(
                "filament_encoder %s: unknown extruder '%s'"
                % (self.name, self.extruder_name))
        self.extruder = extruder
        if hasattr(extruder, 'get_heater'):
            self.heater = extruder.get_heater()
    def _handle_shutdown(self):
        for run in [self.run] + self.finishing:
            if run is not None:
                self._stop_status_timer(run)
                self._flush_log(run, force=True)
                self._end_log(run)
    # Sample handling
    def _counter_callback(self, print_time, count, count_time):
        # Called in the serial thread, hand over to the main thread
        if self.track is not None:
            self.reactor.register_async_callback(
                (lambda e, t=print_time, c=count:
                 self.handle_ref_sample(t, c)))
            return
        self.reactor.register_async_callback(
            (lambda e, t=print_time, c=count, ct=count_time:
             self.handle_sample(t, c, ct)))
    def handle_sample(self, print_time, count, count_time=None):
        if self.base_count is None:
            # The first report may contain an edge from the initial pin
            # level, it only sets the baseline
            self.base_count = count
        edges = count - self.base_count
        if self.history and count_time is not None:
            new_edges = edges - self.history[-1][1]
            if new_edges > 0:
                # count_time is the MCU time of the last edge
                self._record_edges(count_time, int(round(new_edges)))
        self._add_history(print_time, edges)
    def handle_ref_sample(self, print_time, count):
        # Pulse counter next to the angle sensor, kept for comparison
        if self.ref_base is None:
            self.ref_base = count
        self.ref_history.append((print_time, count - self.ref_base))
        trim_history(self.ref_history, self.history_time)
    def ref_at(self, print_time):
        entry = history_step(self.ref_history, print_time)
        if entry is None:
            return None
        return entry[1]
    def _angle_callback(self, msg):
        # Batches of [print_time, angle] about every 0.1 s
        self.angle_errors += msg.get('errors', 0)
        samples = msg.get('data', [])
        if not samples:
            return True
        self._write_angle_samples(samples)
        scale = self.mm_per_rev / (360. * self.mm_per_edge)
        for print_time, angle, edge_times in self.track.feed(samples):
            for edge_time in edge_times:
                self._record_edges(edge_time, 1)
            # Signed length in edge units
            self._add_history(print_time, angle * scale)
        return True
    def _write_angle_samples(self, samples):
        # Copy every native angle sample of a logged run to its side file
        for run in [self.run] + self.finishing:
            if (run is None or run.angle_file is None
                    or run.start_time is None):
                continue
            lines = ["%.6f,%d\n" % (t, a) for t, a in samples
                     if t >= run.start_time
                     and (run.stop_time is None or t <= run.stop_time)]
            if lines:
                self._write_side(run, 'angle_file', "".join(lines))
    def _record_edges(self, edge_time, count):
        self.last_edge_time = edge_time
        for run in [self.run] + self.finishing:
            if run is not None and run.in_run(edge_time):
                run.edge_times.append((edge_time, count))
                if run.edges_file is not None:
                    self._write_side(run, 'edges_file',
                                     "%.6f,%d\n" % (edge_time, count))
    def _add_history(self, print_time, edges):
        self._poll_sensors(print_time)
        if self.extruder is not None:
            epos = self.extruder.find_past_position(print_time)
            if self.last_epos is not None:
                # The angle sensor knows the direction, so its length and
                # the commanded length are both signed
                delta = epos - self.last_epos
                if self.track is None:
                    delta = abs(delta)
                self.cmd_total += delta
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
    # Heater, MOSFET and motion values for the log rows
    def _poll_sensors(self, print_time):
        heater = self.heater
        if heater is not None:
            read_time = getattr(heater, 'last_temp_time', None)
            temps = self.temp_history
            if read_time and (not temps or read_time > temps[-1][0]):
                temps.append((read_time, heater.last_temp))
                trim_history(temps, self.history_time)
            step = (heater.target_temp, heater.last_pwm_value)
            steps = self.heat_steps
            if not steps or steps[-1][1:] != step:
                steps.append((print_time,) + step)
                trim_history(steps, self.history_time)
        if self.mosfet is not None:
            temp = self.mosfet.get_status(
                self.reactor.monotonic()).get('temperature')
            hist = self.mosfet_history
            if temp is not None and (not hist or hist[-1][1] != temp):
                hist.append((print_time, temp))
                trim_history(hist, self.history_time)
    def _temp_ready(self, print_time):
        # True once a heater reading at or after the print time exists,
        # or when it is overdue
        if self.heater is None:
            return True
        temps = self.temp_history
        if temps and temps[-1][0] >= print_time:
            return True
        if not self.history:
            return False
        return self.history[-1][0] - print_time > TEMP_DELAY
    def _motion_values(self, print_time):
        # Commanded extruder velocity and A axis position and velocity
        e_vel = a_pos = a_vel = None
        if self.extruder is not None and hasattr(self.extruder, 'get_trapq'):
            pos, e_vel = trapq_state(self.extruder.get_trapq(), print_time)
        a_stepper = self.a_stepper
        if a_stepper is not None and a_stepper.get_axis_gcode_id():
            a_pos, a_vel = trapq_state(a_stepper.get_trapq(), print_time)
        return e_vel, a_pos, a_vel
    def _has_extra_columns(self):
        return (self.extruder is not None or self.mosfet is not None
                or self.a_stepper is not None)
    def _has_ref(self):
        return self.track is not None and self.counter is not None
    def _extra_columns(self, run, print_time, motion):
        # CSV text of the columns after the stall columns
        e_vel, a_pos, a_vel = motion
        cols = []
        if self.extruder is not None:
            cols.append(self._fmt_val(e_vel, "%.4f"))
            temp = target = power = None
            if self.heater is not None:
                temp = history_interp(self.temp_history, print_time)
                step = history_step(self.heat_steps, print_time)
                if step is not None:
                    target, power = step[1], step[2]
            cols += [self._fmt_val(temp, "%.2f"),
                     self._fmt_val(target, "%.1f"),
                     self._fmt_val(power, "%.4f")]
        if self.mosfet is not None:
            step = history_step(self.mosfet_history, print_time)
            cols.append(self._fmt_val(None if step is None else step[1],
                                      "%.2f"))
        if self.a_stepper is not None:
            cols += [self._fmt_val(a_pos, "%.4f"),
                     self._fmt_val(a_vel, "%.3f")]
        if self._has_ref():
            ref = self.ref_at(print_time)
            if ref is not None and run.start_ref is not None:
                ref -= run.start_ref
            cols.append(self._fmt_val(ref, "%d"))
        if not cols:
            return ""
        return "," + ",".join(cols)
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
    # Load cell handling
    def _load_cell_callback(self, msg):
        # Rows of [print_time, grams, counts, tare_counts], grams is None
        # until the load cell is calibrated and tared
        history = self.force_history
        for row in msg.get('data', []):
            if history and row[0] <= history[-1][0]:
                continue
            history.append((row[0], row[1], row[2]))
        if history:
            min_time = history[-1][0] - self.history_time
            while len(history) > 2 and history[0][0] < min_time:
                history.popleft()
        for run in [self.run] + list(self.finishing):
            if run is not None:
                self._flush_log(run)
                self._try_finish(run)
        return True
    def force_at(self, print_time):
        # Interpolated (grams, counts) at a print time, None if the load
        # cell samples do not cover that time
        history = self.force_history
        if (not history or print_time > history[-1][0]
                or print_time < history[0][0]):
            return None, None
        later = history[-1]
        for i in range(len(history) - 2, -1, -1):
            sample = history[i]
            if sample[0] <= print_time:
                dt = later[0] - sample[0]
                f = 0.
                if dt > 0.:
                    f = (print_time - sample[0]) / dt
                counts = sample[2] + f * (later[2] - sample[2])
                grams = None
                if sample[1] is not None and later[1] is not None:
                    grams = sample[1] + f * (later[1] - sample[1])
                return grams, counts
            later = sample
        return history[0][1], history[0][2]
    def get_force(self):
        if not self.force_history:
            return None
        return self.force_history[-1][1]
    def _force_stats(self, start_time, stop_time):
        total = 0.
        count = 0
        fmin = fmax = None
        for sample_time, grams, counts in reversed(self.force_history):
            if sample_time < start_time:
                break
            if grams is None or sample_time > stop_time:
                continue
            total += grams
            count += 1
            if fmin is None or grams < fmin:
                fmin = grams
            if fmax is None or grams > fmax:
                fmax = grams
        if not count:
            return {'force_avg': None, 'force_min': None, 'force_max': None}
        return {'force_avg': total / count, 'force_min': fmin,
                'force_max': fmax}
    # Stall data handling
    def _stall_callback(self, index, msg):
        history = self.stall_history[index]
        for row in msg.get('data', []):
            if history and row[0] <= history[-1][0]:
                continue
            history.append((row[0], row[1], row[2]))
        if history:
            min_time = history[-1][0] - self.history_time
            while len(history) > 2 and history[0][0] < min_time:
                history.popleft()
        for run in [self.run] + list(self.finishing):
            if run is not None:
                self._flush_log(run)
                self._try_finish(run)
        return self._stall_wanted()
    def _stall_wanted(self):
        return any(run is not None and not run.done
                   and run.log_file is not None
                   for run in [self.run] + self.finishing)
    def _start_stall(self):
        for dump, cb in zip(self.stall_dumps, self.stall_cbs):
            if dump.has_client(cb):
                continue
            try:
                dump.add_client(cb, self.stall_interval)
            except self.printer.command_error as e:
                self.gcode.respond_info("FLOW stall data unavailable: %s"
                                        % (str(e),))
    def stall_at(self, index, print_time):
        # Newest (sg_result, cs_actual) at or before a print time
        for sample_time, sg, cs in reversed(self.stall_history[index]):
            if sample_time <= print_time:
                return sg, cs
        return None, None
    def _stall_ready(self, print_time):
        for dump, cb, history in zip(self.stall_dumps, self.stall_cbs,
                                     self.stall_history):
            if not dump.has_client(cb):
                continue
            if history and history[-1][0] >= print_time:
                continue
            if self.history and self.history[-1][0] - print_time > STALL_DELAY:
                continue
            return False
        return True
    def _stall_stats(self, start_time, stop_time):
        stats = {}
        for name, history in zip(self.stall_names, self.stall_history):
            values = [sg for t, sg, cs in history
                      if start_time <= t <= stop_time and sg >= 0]
            stats[name] = {'samples': len(values),
                           'sg_avg': (sum(values) / float(len(values))
                                      if values else None),
                           'sg_min': min(values) if values else None}
        return stats
    def _pulse_stats(self, run):
        # Scatter of the time between full pulses (two edges), the SFS
        # edges themselves are asymmetric
        times = [t if n == 1 else None for t, n in run.edge_times]
        durations = []
        for i in range(len(times) - 2):
            if None in (times[i], times[i + 1], times[i + 2]):
                continue
            durations.append(times[i + 2] - times[i])
        if len(durations) < MIN_PULSES:
            return {'pulse_cv': None, 'pulses': len(durations)}
        mean = sum(durations) / len(durations)
        var = sum((d - mean) ** 2 for d in durations) / len(durations)
        return {'pulse_cv': 100. * math.sqrt(var) / mean,
                'pulses': len(durations)}
    def _data_ready(self, print_time):
        return (self._force_ready(print_time) and self._stall_ready(print_time)
                and self._temp_ready(print_time))
    def _force_ready(self, print_time):
        # True once the load cell samples cover the print time, or when
        # they are overdue
        if self.load_cell is None:
            return True
        history = self.force_history
        if history and history[-1][0] >= print_time:
            return True
        if not self.history:
            return False
        return self.history[-1][0] - print_time > self.force_delay
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
            run.start_errors = self.angle_errors
            if self._has_ref():
                run.start_ref = self.ref_at(run.start_time)
            if run is self.run:
                self.rel_base = run.start_edges
        if run.stop_time is not None and print_time >= run.stop_time:
            self._try_finish(run)
            return
        flow = self.get_flow()
        if print_time - self.speed_window >= run.start_time:
            if run.flow_min is None or flow < run.flow_min:
                run.flow_min = flow
            if run.flow_max is None or flow > run.flow_max:
                run.flow_max = flow
        if run.log_file is not None:
            vals = self._run_values(run, print_time)
            edge_time = ""
            if self.last_edge_time is not None:
                edge_time = "%.6f" % (self.last_edge_time,)
            # The angle length has more resolution than one edge
            if self.track is not None:
                edges = "%.3f" % (vals['edges'],)
            else:
                edges = "%d" % (round(vals['edges']),)
            row = "%.6f,%s,%.3f,%.3f,%.3f,%s,%s" % (
                print_time, edges, vals['length'],
                self.get_speed(), flow, self._fmt_csv(vals['commanded']),
                edge_time)
            # Motion is final at the row time, the axis registration may
            # change before the delayed row is written
            motion = self._motion_values(print_time)
            if (self.load_cell is None and not self.stall_dumps
                    and self.heater is None and not self._has_ref()):
                self._write_log(run, row + self._extra_columns(
                    run, print_time, motion) + "\n")
            else:
                run.pending.append((print_time, row, motion))
                self._flush_log(run)
    def _try_finish(self, run):
        if run.done or run.stop_time is None or not self.history:
            return
        if self.history[-1][0] < run.stop_time:
            return
        if self._data_ready(run.stop_time):
            self._finish_run(run)
    def _finish_run(self, run):
        self._flush_log(run, force=True)
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
        result.update(self._run_force(run, run.stop_time))
        result.update(self._pulse_stats(run))
        result['stall'] = None
        if self.stall_dumps:
            result['stall'] = self._stall_stats(run.start_time,
                                                run.stop_time)
        result.update(self._angle_result(run, vals))
        run.result = result
        run.done = True
        self.last_result = result
        self._stop_status_timer(run)
        self._end_log(run)
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
        if self.load_cell is not None:
            msg += " force_avg=%sg force_min=%sg force_max=%sg" % (
                self._fmt_num(result['force_avg']),
                self._fmt_num(result['force_min']),
                self._fmt_num(result['force_max']))
        if self.track is not None:
            msg += " revs=%.3f" % (result['revolutions'],)
            if result['angle_errors']:
                msg += " angle_errors=%d" % (result['angle_errors'],)
            if result['sfs_edges'] is not None:
                msg += " sfs=%.1fmm" % (result['sfs_length'],)
        if result['pulse_cv'] is not None:
            msg += " pulse_cv=%.1f%%" % (result['pulse_cv'],)
        for name, stats in sorted((result['stall'] or {}).items()):
            if stats['samples']:
                msg += " sg[%s]=%.0f/%d" % (name, stats['sg_avg'],
                                            stats['sg_min'])
        self.gcode.respond_info(msg)
    def _angle_result(self, run, vals):
        # Wheel turns of the run (mm_per_rev = commanded / revolutions at
        # a low force), angle read errors and the pulse counter length
        if self.track is None:
            return {}
        result = {'revolutions': vals['length'] / self.mm_per_rev,
                  'angle_errors': self.angle_errors - run.start_errors,
                  'sfs_edges': None, 'sfs_length': None}
        if self._has_ref() and run.start_ref is not None:
            stop = self.ref_at(run.stop_time)
            if stop is not None:
                result['sfs_edges'] = stop - run.start_ref
                result['sfs_length'] = result['sfs_edges'] * self.mm_per_edge
        return result
    def _run_force(self, run, print_time):
        if self.load_cell is None or not run.is_started():
            return {'force_avg': None, 'force_min': None, 'force_max': None}
        return self._force_stats(run.start_time, print_time)
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
                msg += self._fmt_force_info()
                self.gcode.respond_info(msg)
        return eventtime + run.interval
    def _stop_status_timer(self, run):
        if run.status_timer is not None:
            self.reactor.unregister_timer(run.status_timer)
            run.status_timer = None
    def _set_start_time(self, run, print_time):
        run.start_time = print_time
        if run.log_file is not None:
            self._add_anchor(run, 'start')
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
    def _get_log_dir(self):
        if self.log_path is not None:
            return os.path.normpath(os.path.expanduser(self.log_path))
        log_file = self.printer.get_start_args().get('log_file')
        if not log_file:
            return None
        # Prefer the "testdata" folder next to the Klipper log folder
        log_dir = os.path.dirname(os.path.abspath(log_file))
        testdata = os.path.join(os.path.dirname(log_dir), 'testdata')
        if os.path.isdir(testdata):
            return testdata
        return log_dir
    def _open_log(self, run):
        log_dir = self._get_log_dir()
        if log_dir is None:
            self.gcode.respond_info(
                "FLOW log disabled, set log_path or run Klipper with a"
                " log file")
            return
        label = re.sub(r'[^A-Za-z0-9_.-]', '_', run.label or '')
        stamp = time.strftime("%Y%m%d_%H%M%S")
        fname = "flow_%s_%s.csv" % (label, stamp) if label \
            else "flow_%s.csv" % (stamp,)
        path = os.path.join(log_dir, fname)
        base = path[:-len(".csv")]
        try:
            if not os.path.isdir(log_dir):
                os.makedirs(log_dir)
            run.log_file = open(path, 'w')
            run.log_file.write(",".join(self._log_columns()) + "\n")
            run.log_name = path
            run.base_name = base
            run.edges_file = open(base + "_edges.csv", 'w')
            run.edges_file.write("print_time,edges\n")
            if self.load_cell is not None:
                run.force_file = open(base + "_force.csv", 'w')
                run.force_file.write("print_time,force_counts,force_g\n")
            if self.track is not None:
                run.angle_file = open(base + "_angle.csv", 'w')
                run.angle_file.write("print_time,angle_counts\n")
        except (IOError, OSError) as e:
            logging.exception("filament_encoder: unable to open log")
            self.gcode.respond_info("FLOW log error: %s" % (str(e),))
            self._close_log(run)
            run.base_name = run.log_name = None
            return
        self._add_anchor(run, 'open')
        self._write_meta(run)
    def _log_columns(self):
        cols = ["print_time", "edges", "length_mm", "speed_mm_s",
                "flow_mm3_s", "commanded_mm", "edge_time"]
        if self.load_cell is not None:
            cols += ["force_g", "force_counts"]
        for name in self.stall_names:
            short = re.sub(r'[^A-Za-z0-9]+', '_', name)
            cols += ["sg_" + short, "cs_" + short]
        if self.extruder is not None:
            cols += ["e_cmd_vel_mm_s", "nozzle_c", "target_c",
                     "heater_power"]
        if self.mosfet is not None:
            cols.append("mosfet_c")
        if self.a_stepper is not None:
            cols += ["a_pos_deg", "a_vel_deg_s"]
        if self._has_ref():
            cols.append("sfs_edges")
        return cols
    # Time anchors and run description for external data streams
    def _add_anchor(self, run, event):
        eventtime = self.reactor.monotonic()
        unix_time = time.time()
        print_time = None
        mcu = self.printer.lookup_object('mcu', None)
        if mcu is not None:
            print_time = mcu.estimated_print_time(eventtime)
        run.anchors.append({'event': event, 'monotonic': eventtime,
                            'unix_time': unix_time,
                            'print_time': print_time})
    def _force_sync_report(self, run):
        # Timestamp quality of the force samples in the run, from sources
        # that rebuild sample times on the host
        report = getattr(self.load_cell, 'get_sync_report', None)
        if report is None or run.start_time is None:
            return None
        try:
            return report(run.start_time, run.stop_time)
        except Exception:
            logging.exception("filament_encoder: force sync report failed")
            return None
    def _write_meta(self, run):
        if run.base_name is None:
            return
        base = os.path.basename(run.base_name)
        meta = {
            'schema': META_SCHEMA, 'encoder': self.name,
            'label': run.label,
            'software_version': self.printer.get_start_args().get(
                'software_version'),
            'files': {
                'rows': base + ".csv", 'edges': base + "_edges.csv",
                'force': (base + "_force.csv"
                          if self.load_cell is not None else None),
                'angle': (base + "_angle.csv"
                          if self.track is not None else None)},
            'columns': self._log_columns(),
            # 'angle': signed length and commanded length, edges are the
            # length in mm_per_edge units and edge times are virtual
            'length_source': 'angle' if self.track is not None else 'pulse',
            'angle_sensor': self.angle_name,
            'mm_per_rev': self.mm_per_rev,
            'angle_hysteresis_deg': self.angle_hysteresis,
            'start_print_time': run.start_time,
            'stop_print_time': run.stop_time,
            'anchors': run.anchors,
            'mm_per_edge': self.mm_per_edge,
            'filament_area_mm2': self.area,
            'sample_time': self.sample_time,
            'speed_window': self.speed_window,
            'force_source': self.load_cell_name,
            'force_samples_per_second': self.force_sps,
            'stall_steppers': self.stall_names,
            'stall_interval': self.stall_interval,
            'extruder': self.extruder_name,
            'mosfet_sensor': self.mosfet_name,
            'a_stepper': self.a_stepper_name,
            'force_sync': self._force_sync_report(run),
            'result': run.result}
        try:
            with open(run.base_name + "_meta.json", 'w') as f:
                json.dump(meta, f, indent=1, sort_keys=True)
        except (IOError, OSError, TypeError, ValueError):
            logging.exception("filament_encoder: unable to write meta file")
    def _write_log(self, run, line):
        try:
            run.log_file.write(line)
        except IOError as e:
            logging.exception("filament_encoder: unable to write log")
            self.gcode.respond_info("FLOW log error: %s" % (str(e),))
            self._close_log(run)
    def _flush_log(self, run, force=False):
        # Write the waiting rows once the load cell, stall and heater
        # samples cover them
        self._flush_force_samples(run)
        pending = run.pending
        while pending and run.log_file is not None:
            print_time, row, motion = pending[0]
            if not force and not self._data_ready(print_time):
                break
            pending.popleft()
            if self.load_cell is not None:
                grams, counts = self.force_at(print_time)
                if counts is not None:
                    counts = "%.0f" % (counts,)
                row += ",%s,%s" % (self._fmt_csv(grams), counts or "")
            for index in range(len(self.stall_dumps)):
                sg, cs = self.stall_at(index, print_time)
                row += ",%s,%s" % ("" if sg is None else "%d" % (sg,),
                                   "" if cs is None else "%d" % (cs,))
            row += self._extra_columns(run, print_time, motion)
            self._write_log(run, row + "\n")
        if run.log_file is None:
            pending.clear()
    def _flush_force_samples(self, run):
        # Copy every native load cell sample of the run to the side file
        if run.force_file is None or run.start_time is None:
            return
        cursor = run.force_cursor
        new = []
        for sample in reversed(self.force_history):
            if cursor is not None and sample[0] <= cursor:
                break
            if sample[0] < run.start_time:
                break
            if run.stop_time is None or sample[0] <= run.stop_time:
                new.append(sample)
        for sample_time, grams, counts in reversed(new):
            self._write_side(run, 'force_file', "%.6f,%.0f,%s\n" % (
                sample_time, counts, self._fmt_csv(grams)))
            run.force_cursor = sample_time
    def _write_side(self, run, attr, line):
        fobj = getattr(run, attr)
        try:
            fobj.write(line)
        except IOError as e:
            logging.exception("filament_encoder: unable to write side file")
            self.gcode.respond_info("FLOW log error: %s" % (str(e),))
            try:
                fobj.close()
            except IOError:
                pass
            setattr(run, attr, None)
    def _end_log(self, run):
        # Close the files and complete the run description
        if run.base_name is None:
            return
        self._close_log(run)
        self._add_anchor(run, 'close')
        self._write_meta(run)
        run.base_name = None
    def _close_log(self, run):
        for attr in ('log_file', 'edges_file', 'force_file', 'angle_file'):
            fobj = getattr(run, attr)
            if fobj is None:
                continue
            try:
                fobj.close()
            except IOError:
                logging.exception("filament_encoder: unable to close log")
            setattr(run, attr, None)
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
    def _fmt_val(self, val, fmt):
        if val is None:
            return ""
        return fmt % (val,)
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
                self.get_speed(), self.get_flow())
            + self._fmt_angle_info() + self._fmt_force_info())
    def _fmt_angle_info(self):
        if self.track is None:
            return ""
        noise = self.track.noise
        msg = " revs=%.3f noise=%s angle_errors=%d" % (
            self._revolutions(), "-" if noise is None else "%.3fdeg" % (noise,),
            self.angle_errors)
        if self._has_ref():
            sfs = self.ref_at(self.history[-1][0]) if self.history else None
            msg += " sfs_edges=%s" % ("-" if sfs is None else "%d" % (sfs,),)
        return msg
    def _revolutions(self):
        # Wheel turns since the start of Klipper or ENCODER_RESET ALL=1
        edges = self.get_edges() - self.abs_base
        return edges * self.mm_per_edge / self.mm_per_rev
    def _fmt_force_info(self):
        if self.load_cell is None:
            return ""
        return " force=%sg" % (self._fmt_num(self.get_force()),)
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
            if run.log_file is not None and self.stall_dumps:
                self._start_stall()
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
        timeout = WAIT_TIMEOUT + self.force_delay
        if self.stall_dumps:
            timeout = max(timeout, WAIT_TIMEOUT + STALL_DELAY)
        deadline = eventtime + timeout
        while not run.done and eventtime < deadline:
            eventtime = self.reactor.pause(eventtime + self.sample_time)
        if not run.done:
            raise gcmd.error("FLOW %sno result within %.0fs, encoder"
                             " samples missing"
                             % (self._fmt_label(run), timeout))
    def get_status(self, eventtime):
        edges = self.get_edges()
        monitor = {'active': False, 'label': None, 'elapsed': 0.,
                   'length': 0., 'flow_avg': None, 'commanded': None,
                   'ratio': None, 'force_avg': None, 'force_min': None,
                   'force_max': None}
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
                monitor.update(self._run_force(run, self.history[-1][0]))
        status = {'absolute': (edges - self.abs_base) * self.mm_per_edge,
                  'relative': (edges - self.rel_base) * self.mm_per_edge,
                  'edges': int(round(edges - self.abs_base)),
                  'relative_edges': int(round(edges - self.rel_base)),
                  'speed': self.get_speed(), 'flow': self.get_flow(),
                  'mm_per_edge': self.mm_per_edge,
                  'force': self.get_force(),
                  'length_source': ('angle' if self.track is not None
                                    else 'pulse'),
                  'monitor': monitor,
                  'last_result': self.last_result}
        if self.track is not None:
            sfs = None
            if self._has_ref() and self.history:
                sfs = self.ref_at(self.history[-1][0])
            status.update({'mm_per_rev': self.mm_per_rev,
                           'revolutions': self._revolutions(),
                           'angle_noise': self.track.noise,
                           'angle_errors': self.angle_errors,
                           'angle_samples': self.track.samples,
                           'sfs_edges': sfs})
        return status

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
