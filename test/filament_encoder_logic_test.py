#!/usr/bin/env python3
# Logic test for klippy/extras/filament_encoder.py with simulated counts
#
# Copyright (C) 2026  Toucan3D
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Run with: python3 test/filament_encoder_logic_test.py
import json, math, os, sys, tempfile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'klippy'))
from extras import filament_encoder, pulse_counter

MM_PER_EDGE = 2.88
SPEED = 83.3
AREA = math.pi * 1.75 * 1.75 / 4.


######################################################################
# Stubs
######################################################################

class StubError(Exception):
    pass

class StubReactor:
    NOW = 0.
    NEVER = 9999999999999999.
    def __init__(self):
        self.timers = []
    def register_async_callback(self, cb, waketime=0.):
        cb(0.)
    def register_timer(self, cb, waketime=NEVER):
        self.timers.append(cb)
        return cb
    def unregister_timer(self, timer):
        self.timers.remove(timer)
    def monotonic(self):
        return 0.
    def pause(self, waketime):
        return waketime

class StubGCode:
    def __init__(self):
        self.mux_commands = {}
        self.messages = []
    def register_mux_command(self, cmd, key, value, func, desc=None):
        self.mux_commands.setdefault(cmd, (key, {}))[1][value] = func
    def respond_info(self, msg):
        self.messages.append(msg)
    def run(self, cmd, **params):
        key, values = self.mux_commands[cmd]
        values[params.get(key)](StubGCmd(self, params))

class StubGCmd:
    def __init__(self, gcode, params):
        self.gcode = gcode
        self.params = params
        self.error = StubError
    def get(self, name, default=None):
        return self.params.get(name, default)
    def get_int(self, name, default=None, minval=None, maxval=None):
        return int(self.params.get(name, default))
    def get_float(self, name, default=None, minval=None, maxval=None):
        return float(self.params.get(name, default))
    def respond_info(self, msg):
        self.gcode.respond_info(msg)

class StubToolhead:
    # Lookahead callbacks run when flush() is called with a print time
    def __init__(self):
        self.pending = []
        self.can_pause = False
    def register_lookahead_callback(self, cb):
        self.pending.append(cb)
    def flush(self, print_time):
        pending, self.pending = self.pending, []
        for cb in pending:
            cb(print_time)
    def wait_moves(self):
        pass

class StubExtruder:
    # Commanded position follows the nominal speed
    def __init__(self):
        self.speed = SPEED
    def find_past_position(self, print_time):
        return self.speed * print_time

class StubPrinter:
    def __init__(self, log_dir):
        self.reactor = StubReactor()
        self.gcode = StubGCode()
        self.toolhead = StubToolhead()
        self.objects = {'gcode': self.gcode, 'toolhead': self.toolhead,
                        'extruder': StubExtruder()}
        self.handlers = {}
        self.log_dir = log_dir
    def get_reactor(self):
        return self.reactor
    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)
    def send_event(self, event):
        for cb in self.handlers.get(event, []):
            cb()
    def get_start_args(self):
        return {'log_file': os.path.join(self.log_dir, 'klippy.log')}
    def config_error(self, msg):
        return StubError(msg)
    command_error = StubError

class StubConfig:
    def __init__(self, printer, name, opts):
        self.printer = printer
        self.name = name
        self.opts = opts
    def get_printer(self):
        return self.printer
    def get_name(self):
        return self.name
    def get(self, name, default=None):
        return self.opts.get(name, default)
    def getfloat(self, name, default=None, above=None, minval=None):
        return float(self.opts.get(name, default))
    def getlist(self, name, default=None, sep=','):
        val = self.opts.get(name)
        if val is None:
            return default
        return [p.strip() for p in val.split(sep) if p.strip()]

class StubCounter:
    def __init__(self, printer, pin, sample_time, poll_time):
        self.callback = None
    def setup_callback(self, cb):
        self.callback = cb

class StubLoadCellSensor:
    def get_samples_per_second(self):
        return 10

class StubLoadCell:
    def __init__(self):
        self.clients = []
    def add_client(self, cb):
        self.clients.append(cb)
    def get_sensor(self):
        return StubLoadCellSensor()
    def send(self, rows):
        for cb in self.clients:
            cb({'data': rows, 'errors': 0, 'overflows': 0})

class StubStallDump:
    # Stands in for tmc.TMCStallguardDump
    def __init__(self):
        self.clients = []
        self.min_interval = None
    def add_client(self, cb, min_interval=0.):
        self.clients.append(cb)
        self.min_interval = min_interval
    def has_client(self, cb):
        return cb in self.clients
    def send(self, rows):
        for cb in list(self.clients):
            if not cb({'data': rows}):
                self.clients.remove(cb)

def sg_of(t):
    return int(400 - 20 * t)

def feed_stall(dump, t0, t1, dt=0.02):
    rows = []
    t = t0
    while t <= t1 + 1e-9:
        rows.append((t, sg_of(t), 16))
        t += dt
    dump.send(rows)

def force_of(t):
    return 100. + 10. * t

def feed_force(lc, t0, t1, block=12, calibrated=True):
    # Send 10 SPS samples in blocks like the hx711
    rows = []
    t = t0
    while t <= t1 + 1e-9:
        grams = force_of(t) if calibrated else None
        rows.append([t, grams, int(1000 * t), 0])
        if len(rows) >= block:
            lc.send(rows)
            rows = []
        t += 0.1
    if rows:
        lc.send(rows)

class StubTrapq:
    # Commanded (position, velocity) from a function of the print time
    def __init__(self, func):
        self.func = func
    def state(self, print_time):
        return self.func(print_time)

# The real trapq_state needs the compiled chelper
filament_encoder.trapq_state = lambda trapq, t: trapq.state(t)

def e_state(t):
    return SPEED * t, SPEED

class StubHeater:
    # Readings every 0.1 s, temperature 200 + t
    def __init__(self):
        self.last_temp = self.last_temp_time = 0.
        self.target_temp = 210.
        self.last_pwm_value = 0.25
    def read(self, t):
        self.last_temp_time = t
        self.last_temp = 200. + t

class StubHeatedExtruder(StubExtruder):
    def __init__(self):
        StubExtruder.__init__(self)
        self.heater = StubHeater()
        self.trapq = StubTrapq(e_state)
    def get_heater(self):
        return self.heater
    def get_trapq(self):
        return self.trapq

class StubSensor:
    def __init__(self, temp):
        self.temp = temp
    def get_status(self, eventtime):
        return {'temperature': self.temp}

class StubManualStepper:
    # A axis turning at 90 deg/s, registered as G-code axis when gcode_id
    def __init__(self):
        self.gcode_id = None
        self.trapq = StubTrapq(lambda t: (90. * t, 90.))
    def get_trapq(self):
        return self.trapq
    def get_axis_gcode_id(self):
        return self.gcode_id

class StubMCU:
    def estimated_print_time(self, eventtime):
        return eventtime + 100.

def make_encoder(extruder=True, load_cell=False, log_path=None,
                 stall=False, extras=False, force_name=None):
    # Log folder inside a private folder, like printer_data/logs
    log_dir = os.path.join(tempfile.mkdtemp(), 'logs')
    os.mkdir(log_dir)
    printer = StubPrinter(log_dir)
    opts = {'pin': 'PG13', 'mm_per_edge': MM_PER_EDGE}
    if log_path is not None:
        opts['log_path'] = log_path
    if extruder:
        opts['extruder'] = 'extruder'
    if extras:
        printer.objects['extruder'] = StubHeatedExtruder()
        printer.objects['temperature_sensor Transistors'] = StubSensor(31.5)
        printer.objects['manual_stepper stepper_a'] = StubManualStepper()
        printer.objects['mcu'] = StubMCU()
        opts['mosfet_sensor'] = 'temperature_sensor Transistors'
        opts['a_stepper'] = 'manual_stepper stepper_a'
    if load_cell:
        opts['load_cell'] = force_name or 'lc'
        obj_name = force_name or 'load_cell lc'
        printer.objects[obj_name] = StubLoadCell()
    if stall:
        opts['stall_steppers'] = 'extruder, extruder_stepper e1'
        printer.objects['tmc_stallguard extruder'] = StubStallDump()
        printer.objects['tmc_stallguard extruder_stepper e1'] = (
            StubStallDump())
    orig = pulse_counter.MCU_counter
    pulse_counter.MCU_counter = StubCounter
    try:
        enc = filament_encoder.FilamentEncoder(
            StubConfig(printer, 'filament_encoder sfs', opts))
    finally:
        pulse_counter.MCU_counter = orig
    printer.send_event("klippy:ready")
    return printer, enc

def feed(enc, t0, t1, speed, count0=0, dt=0.010):
    # Feed samples of a constant filament speed, returns the last count
    count = count0
    t = t0
    edge_rate = speed / MM_PER_EDGE
    while t <= t1 + 1e-9:
        count = count0 + int((t - t0) * edge_rate)
        enc.counter.callback(t, count, t)
        t += dt
    return count

def check(cond, msg):
    if not cond:
        raise AssertionError(msg)
    print("ok   %s" % (msg,))


######################################################################
# Tests
######################################################################

def test_constant_speed():
    printer, enc = make_encoder()
    feed(enc, 1., 4., SPEED)
    speed = enc.get_speed()
    flow = enc.get_flow()
    check(abs(speed - SPEED) <= 2.9, "speed %.2f is 83.3 +- 2.9" % speed)
    check(abs(flow - 200.) <= 7., "flow %.2f is 200 +- 7" % flow)

def test_first_report_baseline():
    printer, enc = make_encoder()
    enc.counter.callback(1., 1, 1.)
    enc.counter.callback(1.01, 1, 1.01)
    check(enc.get_edges() == 0, "first report only sets the baseline")
    enc.counter.callback(1.02, 2, 1.02)
    check(enc.get_edges() == 1, "edges counted after the baseline")

class FakeMCU:
    def clock32_to_clock64(self, clock):
        return clock
    def clock_to_print_time(self, clock):
        return clock / 1000000.

def test_32bit_wrap():
    printer, enc = make_encoder()
    counter = pulse_counter.MCU_counter.__new__(pulse_counter.MCU_counter)
    counter._mcu = FakeMCU()
    counter._poll_ticks = 0
    counter._last_count = 0
    counter._callback = enc._counter_callback
    raw = 0xffffffff - 5
    clock = 1000000
    for i in range(20):
        counter._handle_counter_state({
            'next_clock': clock, 'count': raw & 0xffffffff,
            'count_clock': clock})
        raw += 1
        clock += 10000
    check(enc.get_edges() == 19, "32 bit wrap keeps counting (%d)"
          % enc.get_edges())

def test_standstill():
    printer, enc = make_encoder()
    count = feed(enc, 1., 3., SPEED)
    feed(enc, 3.01, 4.02, 0., count0=count)
    check(enc.get_speed() == 0., "speed falls to 0 within speed_window")

def test_monitor_between_samples():
    printer, enc = make_encoder()
    gcode, toolhead = printer.gcode, printer.toolhead
    gcode.run('FLOW_MONITOR_START', ENCODER='sfs', LABEL='pla', LOG='1')
    toolhead.flush(2.005)
    gcode.run('FLOW_MONITOR_STOP', ENCODER='sfs')
    toolhead.flush(5.005)
    run = enc.finishing[0]
    feed(enc, 1., 6., SPEED)
    res = enc.last_result
    check(res is not None and run.done, "monitor finished after stop time")
    check(abs(res['duration'] - 3.) < 1e-6, "duration %.4f is 3.0"
          % res['duration'])
    expect = 3. * SPEED
    check(abs(res['length'] - expect) <= MM_PER_EDGE,
          "length %.2f within one edge of %.2f" % (res['length'], expect))
    check(abs(res['commanded'] - expect) < 0.01,
          "commanded %.2f is %.2f" % (res['commanded'], expect))
    check(abs(res['ratio'] - 100.) <= 1.5, "ratio %.2f near 100"
          % res['ratio'])
    check(abs(res['flow_avg'] - 200.) <= 7., "flow_avg %.1f" % res['flow_avg'])
    check(res['flow_min'] <= res['flow_avg'] <= res['flow_max'],
          "flow_min <= flow_avg <= flow_max")
    check(any(m.startswith("FLOW pla DONE") for m in gcode.messages),
          "summary line printed")
    rows = open(run.log_name).read().splitlines()
    check(rows[0].startswith("print_time,edges,length_mm"), "csv header")
    check(290 <= len(rows) - 1 <= 301, "csv has %d rows for 3 s"
          % (len(rows) - 1))
    check(enc.get_status(0.)['last_result'] is res, "status last_result")

def test_restart_and_stop_without_run():
    printer, enc = make_encoder(extruder=False)
    gcode, toolhead = printer.gcode, printer.toolhead
    gcode.run('FLOW_MONITOR_STOP')
    check("no active monitor" in gcode.messages[-1], "stop without run")
    gcode.run('FLOW_MONITOR_START', LABEL='one', INTERVAL='0')
    toolhead.flush(1.5)
    gcode.run('FLOW_MONITOR_START', LABEL='two', INTERVAL='0')
    toolhead.flush(2.5)
    feed(enc, 1., 3., SPEED)
    check(enc.last_result['label'] == 'one', "second start ends first run")
    check(enc.last_result['commanded'] is None, "no cmd without extruder")
    check(enc.run is not None and enc.run.label == 'two', "second run active")
    st = enc.get_status(0.)
    check(st['monitor']['active'] and st['monitor']['length'] > 0.,
          "status monitor of the running run")

def test_reset_and_query():
    printer, enc = make_encoder()
    feed(enc, 1., 2., SPEED)
    printer.gcode.run('ENCODER_RESET')
    st = enc.get_status(0.)
    check(st['relative'] == 0. and st['absolute'] > 0., "ENCODER_RESET")
    printer.gcode.run('ENCODER_RESET', ALL='1')
    check(enc.get_status(0.)['absolute'] == 0., "ENCODER_RESET ALL=1")
    printer.gcode.run('ENCODER_QUERY')
    check(printer.gcode.messages[-1].startswith("ENCODER sfs absolute="),
          "ENCODER_QUERY")

def test_monitor_with_load_cell():
    printer, enc = make_encoder(load_cell=True)
    lc = printer.objects['load_cell lc']
    gcode, toolhead = printer.gcode, printer.toolhead
    gcode.run('FLOW_MONITOR_START', LABEL='force', LOG='1')
    toolhead.flush(2.005)
    gcode.run('FLOW_MONITOR_STOP')
    toolhead.flush(5.005)
    run = enc.finishing[0]
    feed(enc, 1., 6., SPEED)
    check(not run.done, "run waits for the delayed load cell samples")
    feed_force(lc, 1., 6.)
    res = enc.last_result
    check(run.done and res is not None, "run done once force covers stop")
    check(abs(res['force_avg'] - force_of(3.5)) < 1.,
          "force_avg %.2f is %.2f" % (res['force_avg'], force_of(3.5)))
    check(abs(res['force_min'] - force_of(2.1)) < 0.01
          and abs(res['force_max'] - force_of(5.0)) < 0.01,
          "force_min %.2f and force_max %.2f inside the run"
          % (res['force_min'], res['force_max']))
    check(abs(res['length'] - 3. * SPEED) <= MM_PER_EDGE,
          "length unchanged with load cell")
    check("force_avg=" in gcode.messages[-1], "summary has force")
    rows = open(run.log_name).read().splitlines()
    check(",force_g,force_counts" in rows[0], "csv force columns")
    check(290 <= len(rows) - 1 <= 301, "csv has %d rows" % (len(rows) - 1))
    worst = 0.
    for row in rows[1:]:
        cols = row.split(',')
        t, grams = float(cols[0]), float(cols[7])
        worst = max(worst, abs(grams - force_of(t)))
    check(worst < 0.01, "csv force synced to print time (err %.4f)"
          % (worst,))
    st = enc.get_status(0.)
    check(abs(st['force'] - force_of(6.)) < 0.01, "status force")

def test_load_cell_overdue_and_uncalibrated():
    printer, enc = make_encoder(load_cell=True)
    lc = printer.objects['load_cell lc']
    gcode, toolhead = printer.gcode, printer.toolhead
    gcode.run('FLOW_MONITOR_START', LABEL='late', LOG='1', INTERVAL='0')
    toolhead.flush(1.505)
    gcode.run('FLOW_MONITOR_STOP')
    toolhead.flush(2.505)
    run = enc.finishing[0]
    feed_force(lc, 1., 1.95, calibrated=False)
    feed(enc, 1., 2.505 + enc.force_delay - 0.05, SPEED)
    check(not run.done, "run waits up to force_delay %.1fs"
          % (enc.force_delay,))
    feed(enc, 2.505 + enc.force_delay, 2.505 + enc.force_delay + 0.2,
         SPEED, count0=enc.get_edges())
    res = enc.last_result
    check(run.done and res['force_avg'] is None,
          "overdue load cell finishes without force")
    rows = open(run.log_name).read().splitlines()[1:]
    early = [r for r in rows if float(r.split(',')[0]) <= 1.9]
    late = [r for r in rows if float(r.split(',')[0]) > 1.9]
    check(early and all(r.split(',')[7] == '' and r.split(',')[8] != ''
                        for r in early),
          "uncalibrated rows keep raw counts")
    check(late and all(r.endswith(',,') for r in late),
          "rows without load cell samples have empty force")

def feed_edges(enc, t0, t1, period, jitter=0.):
    # One edge every period seconds with exact edge times, samples at 10 ms
    count = 0
    next_edge = t0 + period
    last_edge = None
    t = t0
    k = 0
    while t <= t1 + 1e-9:
        while next_edge <= t:
            count += 1
            last_edge = next_edge
            k += 1
            # Asymmetric halves like the SFS, 60/40
            half = period * (1.2 if k % 2 else 0.8)
            next_edge += half + jitter * (1 if k % 4 == 0 else 0)
        enc.handle_sample(t, count, last_edge)
        t += 0.010
    return count

def test_stall_and_edge_timing():
    printer, enc = make_encoder(stall=True)
    d1 = printer.objects['tmc_stallguard extruder']
    d2 = printer.objects['tmc_stallguard extruder_stepper e1']
    gcode, toolhead = printer.gcode, printer.toolhead
    gcode.run('FLOW_MONITOR_START', LABEL='stall', LOG='1', INTERVAL='0')
    check(d1.clients and d2.clients and d1.min_interval == 0.02,
          "logged run starts both stall streams at 20 ms")
    toolhead.flush(2.005)
    gcode.run('FLOW_MONITOR_STOP')
    toolhead.flush(5.005)
    run = enc.finishing[0]
    feed_edges(enc, 1., 6., 0.05)
    check(not run.done, "run waits for the stall samples")
    feed_stall(d1, 1., 6.)
    check(not run.done, "run waits for the second motor")
    feed_stall(d2, 1., 6.)
    res = enc.last_result
    check(run.done and res is not None, "run done once stall covers stop")
    s1 = res['stall']['extruder']
    check(s1['samples'] > 140 and s1['sg_min'] == sg_of(5.0)
          and abs(s1['sg_avg'] - sg_of(3.5)) < 1.5,
          "stall stats inside the run (avg %.1f, min %d)"
          % (s1['sg_avg'], s1['sg_min']))
    check(res['pulse_cv'] is not None and res['pulse_cv'] < 0.5,
          "even full pulses give a low pulse_cv (%.3f %%) despite"
          " asymmetric edges" % (res['pulse_cv'],))
    check("pulse_cv=" in gcode.messages[-1]
          and "sg[extruder]=" in gcode.messages[-1], "summary has stall")
    rows = open(run.log_name).read().splitlines()
    check((",edge_time,sg_extruder,cs_extruder,"
           "sg_extruder_stepper_e1,cs_extruder_stepper_e1") in rows[0],
          "csv header has edge time and stall columns")
    cols = rows[-1].split(',')
    t = float(cols[0])
    check(float(cols[6]) <= t and int(cols[7]) == sg_of(
        math.floor(t / 0.02 + 1e-6) * 0.02) and cols[8] == '16',
          "csv row holds the last edge time and stall sample")
    feed_stall(d1, 6.02, 6.5)
    feed_stall(d2, 6.02, 6.5)
    check(not d1.clients and not d2.clients,
          "stall streams stop after the logged run")
    # Uneven pulses give a high scatter
    printer, enc = make_encoder(stall=True)
    gcode, toolhead = printer.gcode, printer.toolhead
    gcode.run('FLOW_MONITOR_START', LABEL='slip', INTERVAL='0')
    check(not printer.objects['tmc_stallguard extruder'].clients,
          "run without LOG does not stream stall data")
    toolhead.flush(1.005)
    gcode.run('FLOW_MONITOR_STOP')
    toolhead.flush(4.005)
    feed_edges(enc, 1., 5., 0.05, jitter=0.03)
    res = enc.last_result
    check(res['pulse_cv'] > 10., "slipping pulses raise pulse_cv (%.1f %%)"
          % (res['pulse_cv'],))

def feed_heated(enc, t0, t1, speed):
    # Encoder samples at 10 ms and heater readings at 100 ms
    heater = enc.heater
    edge_rate = speed / MM_PER_EDGE
    t = t0
    k = 0
    while t <= t1 + 1e-9:
        if k % 10 == 0:
            heater.read(t)
        enc.counter.callback(t, int((t - t0) * edge_rate), t)
        t += 0.010
        k += 1

def test_extra_columns():
    printer, enc = make_encoder(extras=True)
    gcode, toolhead = printer.gcode, printer.toolhead
    gcode.run('FLOW_MONITOR_START', LABEL='extra', LOG='1', INTERVAL='0')
    run = enc.run
    toolhead.flush(1.005)
    feed_heated(enc, 1., 2., SPEED)
    enc.heater.target_temp = 230.
    enc.heater.last_pwm_value = 0.5
    printer.objects['temperature_sensor Transistors'].temp = 33.
    printer.objects['manual_stepper stepper_a'].gcode_id = 'A'
    feed_heated(enc, 2.01, 2.9, SPEED)
    gcode.run('FLOW_MONITOR_STOP')
    toolhead.flush(3.505)
    feed_heated(enc, 2.91, 4.2, SPEED)
    check(run.done, "run with heater rows finished")
    rows = [r.split(',') for r in open(run.log_name).read().splitlines()]
    head = rows[0]
    check(head[:7] == ["print_time", "edges", "length_mm", "speed_mm_s",
                       "flow_mm3_s", "commanded_mm", "edge_time"]
          and head[7:] == ["e_cmd_vel_mm_s", "nozzle_c", "target_c",
                           "heater_power", "mosfet_c", "a_pos_deg",
                           "a_vel_deg_s"],
          "new columns appended after the old ones")
    col = dict((name, i) for i, name in enumerate(head))
    worst = 0.
    for r in rows[1:]:
        t = float(r[0])
        worst = max(worst, abs(float(r[col['nozzle_c']]) - (200. + t)))
    check(worst < 0.005, "nozzle_c interpolated between the 0.1 s"
          " readings (err %.4f)" % (worst,))
    first = [r for r in rows[1:] if float(r[0]) < 1.995][-1]
    last = [r for r in rows[1:] if float(r[0]) > 2.5][-1]
    check(first[col['target_c']] == "210.0"
          and last[col['target_c']] == "230.0"
          and last[col['heater_power']] == "0.5000",
          "target and power held as steps")
    check(first[col['mosfet_c']] == "31.50"
          and last[col['mosfet_c']] == "33.00", "mosfet_c")
    check(first[col['a_pos_deg']] == "" and first[col['a_vel_deg_s']] == "",
          "A axis empty while it is no G-code axis")
    t = float(last[0])
    check(abs(float(last[col['a_pos_deg']]) - 90. * t) < 1e-3
          and last[col['a_vel_deg_s']] == "90.000",
          "A axis position and velocity at the row time")
    check(abs(float(last[col['e_cmd_vel_mm_s']]) - SPEED) < 1e-3,
          "commanded extruder velocity")

def test_side_files_and_anchors():
    printer, enc = make_encoder(load_cell=True, extras=True,
                                force_name='eddy_force nozzle')
    lc = printer.objects['eddy_force nozzle']
    check(enc.load_cell is lc, "force source given by its full object name")
    gcode, toolhead = printer.gcode, printer.toolhead
    gcode.run('FLOW_MONITOR_START', LABEL='side', LOG='1', INTERVAL='0')
    run = enc.run
    base = run.log_name[:-4]
    toolhead.flush(2.005)
    gcode.run('FLOW_MONITOR_STOP')
    toolhead.flush(5.005)
    feed_edges(enc, 1., 6., 0.05)
    feed_force(lc, 1., 6.)
    res = enc.last_result
    check(run.done and res is not None, "run finished")
    force = open(base + "_force.csv").read().splitlines()
    check(force[0] == "print_time,force_counts,force_g", "force file header")
    times = [float(r.split(',')[0]) for r in force[1:]]
    check(len(times) == 30 and abs(times[0] - 2.1) < 1e-6
          and abs(times[-1] - 5.0) < 1e-6
          and times == sorted(set(times)),
          "every native force sample of the run, once (%d)" % (len(times),))
    edges = [r.split(',') for r in
             open(base + "_edges.csv").read().splitlines()[1:]]
    check(sum(int(n) for t, n in edges) == len(run.edge_times)
          and all(2.005 <= float(t) <= 5.005 for t, n in edges),
          "edge file holds the edge times of the run")
    meta = json.load(open(base + "_meta.json"))
    events = [a['event'] for a in meta['anchors']]
    check(events == ['open', 'start', 'close']
          and all(a['print_time'] == a['monotonic'] + 100.
                  for a in meta['anchors']),
          "time anchors with monotonic, unix and print time")
    check(meta['files']['force'].endswith("_force.csv")
          and meta['start_print_time'] == 2.005
          and meta['stop_print_time'] == 5.005
          and meta['result']['edges'] == res['edges']
          and meta['columns'][-1] == "a_vel_deg_s",
          "meta file describes the run")

def run_logged(printer, enc):
    printer.gcode.run('FLOW_MONITOR_START', LABEL='dir', LOG='1',
                      INTERVAL='0')
    return enc.run.log_name

def test_log_dir():
    printer, enc = make_encoder()
    path = run_logged(printer, enc)
    check(os.path.dirname(path) == printer.log_dir,
          "log next to klippy.log without testdata")
    testdata = os.path.join(os.path.dirname(printer.log_dir), 'testdata')
    os.mkdir(testdata)
    path = run_logged(printer, enc)
    check(os.path.dirname(path) == testdata,
          "log in the testdata folder next to the log folder")
    custom = os.path.join(tempfile.mkdtemp(), 'bench', 'flow')
    printer, enc = make_encoder(log_path=custom)
    path = run_logged(printer, enc)
    check(os.path.dirname(path) == custom and os.path.isfile(path),
          "log_path option creates the folder")

def main():
    tests = [test_constant_speed, test_first_report_baseline,
             test_32bit_wrap, test_standstill, test_monitor_between_samples,
             test_restart_and_stop_without_run, test_reset_and_query,
             test_monitor_with_load_cell,
             test_load_cell_overdue_and_uncalibrated, test_log_dir,
             test_stall_and_edge_timing, test_extra_columns,
             test_side_files_and_anchors]
    for test in tests:
        print("== %s" % (test.__name__,))
        test()
    print("All filament_encoder logic tests passed")

if __name__ == '__main__':
    main()
