# Show a numeric value from a JSON file written by another process
#
# Copyright (C) 2026  Toucan3D
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# The file is read on a timer and must be small and written atomically
# (write a temporary file, then rename it). Missing, stale or invalid
# data only changes the reported state, this module never shuts down
# the printer.
import json, logging, math, os, time

MAX_FILE_SIZE = 65536

def as_number(val):
    # Finite int or float, None for anything else (bool is no number)
    if isinstance(val, bool) or not isinstance(val, (int, float)):
        return None
    val = float(val)
    if math.isnan(val) or math.isinf(val):
        return None
    return val


######################################################################
# JSON file sensor
######################################################################

class JsonSensor:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name().split()[-1]
        path = os.path.expanduser(config.get('path'))
        if not os.path.isabs(path):
            # Relative to the folder of the printer config file
            cfg_file = self.printer.get_start_args().get('config_file', '')
            path = os.path.join(
                os.path.dirname(os.path.abspath(cfg_file)), path)
        self.path = os.path.normpath(path)
        self.field = config.get('field', 'value')
        self.state_field = config.get('state_field', 'state')
        self.time_field = config.get('time_field', 'unix_time')
        self.rate_field = config.get('rate_field', 'sps')
        self.recording_field = config.get('recording_field', 'recording')
        self.max_age = config.getfloat('max_age', 2., minval=0.)
        self.interval = config.getfloat('interval', 0.25, minval=0.05)
        # Reported values
        self.state = 'missing'
        self.value = self.zero = None
        self.is_tared = False
        self.sps = None
        self.age = None
        self.recording = False
        self.callbacks = []
        self.timer = None
        # Keeps reading after a shutdown, it is display data only
        self.printer.register_event_handler("klippy:ready",
                                            self._handle_ready)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command("JSON_SENSOR_TARE", "SENSOR", self.name,
                                   self.cmd_JSON_SENSOR_TARE,
                                   desc=self.cmd_JSON_SENSOR_TARE_help)
    def _handle_ready(self):
        self.update(self.reactor.monotonic())
        if self.printer.get_start_args().get('debugoutput') is not None:
            return
        self.timer = self.reactor.register_timer(
            self._update_event, self.reactor.monotonic() + self.interval)
    def add_callback(self, cb):
        # cb(eventtime) runs after every update
        self.callbacks.append(cb)
    def get_interval(self):
        return self.interval
    def _update_event(self, eventtime):
        try:
            self.update(eventtime)
        except Exception:
            logging.exception("json_sensor %s: update failed", self.name)
        return eventtime + self.interval
    def _read_file(self):
        # Returns (state, data, file modification time)
        try:
            with open(self.path, 'rb') as f:
                raw = f.read(MAX_FILE_SIZE + 1)
            mtime = os.path.getmtime(self.path)
        except (IOError, OSError):
            return 'missing', None, None
        if len(raw) > MAX_FILE_SIZE:
            return 'invalid', None, None
        try:
            data = json.loads(raw.decode('utf-8'))
        except (ValueError, UnicodeDecodeError):
            return 'invalid', None, None
        if not isinstance(data, dict):
            return 'invalid', None, None
        return 'ok', data, mtime
    def update(self, eventtime):
        old_state = self.state
        state, data, mtime = self._read_file()
        value = sps = age = None
        recording = False
        if data is not None:
            now = time.time()
            stamp = as_number(data.get(self.time_field))
            if stamp is None:
                stamp = mtime
            if stamp is not None:
                age = max(0., now - stamp)
            file_state = data.get(self.state_field)
            if file_state is not None:
                state = str(file_state)
            if self.max_age and age is not None and age > self.max_age:
                state = 'stale'
            else:
                value = as_number(data.get(self.field))
                sps = as_number(data.get(self.rate_field))
            recording = bool(data.get(self.recording_field))
        self.state = state
        self.value = value
        self.sps = sps
        self.age = age
        self.recording = recording
        if value is not None and not self.is_tared and self.zero is None:
            # Until the first tare the first valid value is the zero
            self.zero = value
        if state != old_state:
            logging.info("json_sensor %s: state %s", self.name, state)
        for cb in self.callbacks:
            try:
                cb(eventtime)
            except Exception:
                logging.exception("json_sensor %s: callback failed",
                                  self.name)
    def get_delta(self):
        if self.value is None or self.zero is None:
            return None
        return self.value - self.zero
    cmd_JSON_SENSOR_TARE_help = "Set the zero of a JSON sensor to its value"
    def cmd_JSON_SENSOR_TARE(self, gcmd):
        if self.value is None:
            raise gcmd.error("JSON sensor %s has no valid value (state %s)"
                             % (self.name, self.state))
        self.zero = self.value
        self.is_tared = True
        gcmd.respond_info("JSON sensor %s zero set to %.6g"
                          % (self.name, self.zero))
    def get_status(self, eventtime):
        return {'state': self.state, 'value': self.value,
                'delta': self.get_delta(), 'zero': self.zero,
                'sps': self.sps, 'age_s': self.age,
                'recording': self.recording}


######################################################################
# Temperature sensor type for graphs
######################################################################

class JsonTemperatureSensor:
    # Reports delta x scale of a [json_sensor] like a temperature, 0 while
    # it has no valid value
    def __init__(self, config):
        self.printer = config.get_printer()
        name = config.get('sensor')
        self.scale = config.getfloat('scale', 1.)
        self.source = self.printer.load_object(config, 'json_sensor ' + name)
        self.min_temp = self.max_temp = 0.
        self._callback = None
        self.source.add_callback(self._source_update)
    def setup_minmax(self, min_temp, max_temp):
        # Out of range values are reported, never a shutdown
        self.min_temp = min_temp
        self.max_temp = max_temp
    def setup_callback(self, cb):
        self._callback = cb
    def get_report_time_delta(self):
        return self.source.get_interval()
    def _source_update(self, eventtime):
        if self._callback is None:
            return
        delta = self.source.get_delta()
        value = 0. if delta is None else delta * self.scale
        print_time = 0.
        mcu = self.printer.lookup_object('mcu', None)
        if mcu is not None:
            print_time = mcu.estimated_print_time(eventtime)
        self._callback(print_time, value)

def load_config(config):
    pheaters = config.get_printer().load_object(config, "heaters")
    pheaters.add_sensor_factory("json_sensor", JsonTemperatureSensor)

def load_config_prefix(config):
    return JsonSensor(config)
