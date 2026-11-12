#!/usr/bin/env python3
# Logic test for klippy/extras/json_sensor.py with temporary files
#
# Copyright (C) 2026  Toucan3D
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Run with: python3 test/json_sensor_logic_test.py
import json, os, sys, tempfile, time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'klippy'))
from extras import json_sensor


######################################################################
# Stubs
######################################################################

class StubError(Exception):
    pass

class StubReactor:
    NEVER = 9999999999999999.
    def __init__(self):
        self.timers = []
    def register_timer(self, cb, waketime=NEVER):
        self.timers.append(cb)
        return cb
    def monotonic(self):
        return 100.

class StubGCode:
    def __init__(self):
        self.mux_commands = {}
        self.messages = []
    def register_mux_command(self, cmd, key, value, func, desc=None):
        self.mux_commands.setdefault(cmd, (key, {}))[1][value] = func
    def run(self, cmd, **params):
        key, values = self.mux_commands[cmd]
        values[params.get(key)](StubGCmd(self, params))

class StubGCmd:
    error = StubError
    def __init__(self, gcode, params):
        self.gcode = gcode
        self.params = params
    def respond_info(self, msg):
        self.gcode.messages.append(msg)

class StubMCU:
    def estimated_print_time(self, eventtime):
        return eventtime + 1000.

class StubHeaters:
    def __init__(self):
        self.factories = {}
    def add_sensor_factory(self, name, factory):
        self.factories[name] = factory

class StubPrinter:
    def __init__(self, cfg_dir, debugoutput=False):
        self.reactor = StubReactor()
        self.gcode = StubGCode()
        self.objects = {'gcode': self.gcode, 'mcu': StubMCU(),
                        'heaters': StubHeaters()}
        self.handlers = {}
        self.start_args = {'config_file': os.path.join(cfg_dir,
                                                       'printer.cfg')}
        if debugoutput:
            self.start_args['debugoutput'] = 'out'
        self.configs = {}
    def get_reactor(self):
        return self.reactor
    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)
    def load_object(self, config, name):
        if name not in self.objects:
            sub = StubConfig(self, name, self.configs[name])
            if name.startswith('json_sensor '):
                self.objects[name] = json_sensor.load_config_prefix(sub)
            else:
                json_sensor.load_config(sub)
        return self.objects[name]
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)
    def send_event(self, event):
        for cb in self.handlers.get(event, []):
            cb()
    def get_start_args(self):
        return self.start_args

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
        val = self.opts.get(name, default)
        if val is None and default is None and name not in self.opts:
            if name in ('path', 'sensor'):
                raise StubError("missing option %s" % (name,))
        return val
    def getfloat(self, name, default=None, minval=None, above=None):
        return float(self.opts.get(name, default))

def make_sensor(opts, debugoutput=False):
    cfg_dir = tempfile.mkdtemp()
    printer = StubPrinter(cfg_dir, debugoutput)
    printer.configs['json_sensor test'] = opts
    sensor = printer.load_object(None, 'json_sensor test')
    return printer, sensor, cfg_dir

def write(path, data, raw=None):
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        f.write(raw if raw is not None else json.dumps(data))
    os.replace(tmp, path)

def check(cond, msg):
    if not cond:
        raise AssertionError(msg)
    print("ok   %s" % (msg,))


######################################################################
# Tests
######################################################################

def test_states_and_values():
    printer, s, cfg_dir = make_sensor({'path': 'live.json',
                                       'field': 'freq_hz'})
    path = os.path.join(cfg_dir, 'live.json')
    check(s.path == path, "relative path next to the printer config")
    printer.send_event("klippy:ready")
    st = s.get_status(0.)
    check(st['state'] == 'missing' and st['value'] is None
          and st['delta'] is None and st['recording'] is False,
          "missing file gives state missing")
    check(len(printer.reactor.timers) == 1, "update timer registered")
    now = time.time()
    write(path, {'unix_time': now, 'state': 'streaming',
                 'freq_hz': 1000000.5, 'sps': 300.2,
                 'recording': '/tmp/run.csv'})
    s.update(1.)
    st = s.get_status(0.)
    check(st['state'] == 'streaming' and st['value'] == 1000000.5
          and st['zero'] == 1000000.5 and st['delta'] == 0.
          and st['sps'] == 300.2 and st['recording'] is True
          and 0. <= st['age_s'] < 1., "valid file, first value is zero")
    write(path, {'unix_time': now, 'state': 'streaming',
                 'freq_hz': 1000012.5, 'recording': None})
    s.update(2.)
    st = s.get_status(0.)
    check(st['delta'] == 12. and st['zero'] == 1000000.5
          and st['recording'] is False and st['sps'] is None,
          "delta from the first value, missing fields are None/False")
    write(path, {'unix_time': now - 10., 'state': 'streaming',
                 'freq_hz': 1000020.})
    s.update(3.)
    st = s.get_status(0.)
    check(st['state'] == 'stale' and st['value'] is None
          and st['delta'] is None and st['age_s'] >= 10.,
          "old unix_time gives state stale without value")
    for raw, what in (('{"freq_hz": 1', "truncated json"),
                      ('[1, 2]', "json list"),
                      ('x' * 70000, "oversized file")):
        write(path, None, raw)
        s.update(4.)
        check(s.get_status(0.)['state'] == 'invalid'
              and s.get_status(0.)['value'] is None,
              "%s gives state invalid" % (what,))
    for val in (None, "123", True, float('nan')):
        write(path, {'unix_time': time.time(), 'state': 'no_device',
                     'freq_hz': val})
        s.update(5.)
        st = s.get_status(0.)
        check(st['state'] == 'no_device' and st['value'] is None,
              "value %r is no number" % (val,))
    write(path, {'freq_hz': 5.})
    s.update(6.)
    st = s.get_status(0.)
    check(st['state'] == 'ok' and st['value'] == 5.
          and st['age_s'] is not None, "no state and time fields: ok,"
          " age from the file time")

def test_tare_and_no_stale_check():
    printer, s, cfg_dir = make_sensor({'path': os.path.join(
        tempfile.mkdtemp(), 'v.json'), 'max_age': 0.})
    printer.send_event("klippy:ready")
    try:
        printer.gcode.run('JSON_SENSOR_TARE', SENSOR='test')
        tared = True
    except StubError:
        tared = False
    check(not tared, "tare without a value is a command error")
    write(s.path, {'unix_time': 1., 'value': 7.})
    s.update(1.)
    check(s.get_status(0.)['state'] == 'ok'
          and s.get_status(0.)['value'] == 7.,
          "max_age 0 disables the stale check")
    write(s.path, {'unix_time': 1., 'value': 9.})
    s.update(2.)
    printer.gcode.run('JSON_SENSOR_TARE', SENSOR='test')
    write(s.path, {'unix_time': 1., 'value': 10.5})
    s.update(3.)
    st = s.get_status(0.)
    check(st['zero'] == 9. and st['delta'] == 1.5,
          "JSON_SENSOR_TARE sets the zero to the current value")

def test_timer_never_raises():
    printer, s, cfg_dir = make_sensor({'path': '/nonexistent/x.json'})
    printer.send_event("klippy:ready")
    def boom():
        raise RuntimeError("read failed")
    s._read_file = boom
    nxt = s._update_event(50.)
    check(nxt == 50. + s.interval, "update errors are caught in the timer")
    printer2, s2, d2 = make_sensor({'path': 'x.json'}, debugoutput=True)
    printer2.send_event("klippy:ready")
    check(not printer2.reactor.timers, "no timer in batch mode")

def test_temperature_sensor():
    printer, s, cfg_dir = make_sensor({'path': 'live.json',
                                       'field': 'freq_hz'})
    json_sensor.load_config(StubConfig(printer, 'json_sensor', {}))
    factory = printer.objects['heaters'].factories['json_sensor']
    temp = factory(StubConfig(printer, 'temperature_sensor test',
                              {'sensor': 'test', 'scale': '0.5'}))
    temp.setup_minmax(-100., 100.)
    got = []
    temp.setup_callback(lambda t, v: got.append((t, v)))
    check(temp.get_report_time_delta() == s.interval,
          "report time is the file read interval")
    printer.send_event("klippy:ready")
    check(got and got[-1] == (1100., 0.),
          "0 while there is no value, print time from the mcu")
    path = s.path
    write(path, {'unix_time': time.time(), 'freq_hz': 1000.})
    s.update(1.)
    write(path, {'unix_time': time.time(), 'freq_hz': 1600.})
    s.update(2.)
    check(got[-1] == (1002., 300.), "delta x scale reported")
    write(path, {'unix_time': time.time(), 'freq_hz': 1e9})
    s.update(3.)
    check(got[-1][1] > 100., "out of range values are reported, no"
          " shutdown")

def main():
    tests = [test_states_and_values, test_tare_and_no_stale_check,
             test_timer_never_raises, test_temperature_sensor]
    for test in tests:
        print("== %s" % (test.__name__,))
        test()
    print("All json_sensor logic tests passed")

if __name__ == '__main__':
    main()
