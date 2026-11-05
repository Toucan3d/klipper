# Show the force of a load cell like a temperature sensor
#
# Copyright (C) 2026  Toucan3D
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import collections

class LoadCellForceSensor:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.load_cell_name = config.get('load_cell')
        self.smooth_time = config.getfloat('smooth_time', 0.5, minval=0.)
        self.load_cell = None
        self.samples = collections.deque()
        self.min_temp = self.max_temp = 0.
        self._callback = None
        self.printer.register_event_handler("klippy:ready",
                                            self._handle_ready)
    def _handle_ready(self):
        load_cell = self.printer.lookup_object(
            'load_cell ' + self.load_cell_name, None)
        if load_cell is None or not hasattr(load_cell, 'add_client'):
            raise self.printer.config_error(
                "load_cell_force: unknown load_cell '%s'"
                % (self.load_cell_name,))
        self.load_cell = load_cell
        load_cell.add_client(self._load_cell_callback)
    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp
    def setup_callback(self, cb):
        self._callback = cb
    def get_report_time_delta(self):
        return self.smooth_time
    def _load_cell_callback(self, msg):
        # Rows of [print_time, grams, counts, tare_counts], grams is None
        # until the load cell is calibrated and tared
        data = msg.get('data', [])
        if not data:
            return True
        samples = self.samples
        for row in data:
            samples.append((row[0], row[1]))
        last_time = samples[-1][0]
        min_time = last_time - self.smooth_time
        while len(samples) > 1 and samples[0][0] < min_time:
            samples.popleft()
        forces = [grams for t, grams in samples if grams is not None]
        value = 0.
        if forces:
            value = sum(forces) / len(forces)
        if self._callback is not None:
            self._callback(last_time, value)
        return True

def load_config(config):
    pheaters = config.get_printer().load_object(config, "heaters")
    pheaters.add_sensor_factory("load_cell_force", LoadCellForceSensor)
