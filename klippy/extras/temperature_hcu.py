# Support for HCU-backed temperature sensors
#
# Copyright (C) 2020  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import mcu

SAMPLE_TIME = 0.1
REPORT_TIME = 0.1

class PrinterTemperatureHCU:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.temperature_callback = None
        self.pwm_callback = None
        mcu_name = config.get('sensor_mcu', 'hcu')
        hcu_mcu = mcu.get_printer_mcu(self.printer, mcu_name)
        self._mcu_temp_register = hcu_mcu.setup_register(0x4014)
        self._mcu_temp_register.setup_register_read_callback(
            REPORT_TIME, self.hcu_temp_callback)
        self._mcu_pwm_register = hcu_mcu.setup_register(0x4028)
        self._mcu_pwm_register.setup_register_read_callback(
            REPORT_TIME, self.hcu_pwm_callback)
    def setup_callback(self, temperature_callback):
        self.temperature_callback = temperature_callback
    def setup_pwm_callback(self, pwm_callback):
        self.pwm_callback = pwm_callback
    def get_report_time_delta(self):
        return REPORT_TIME
    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp
    def hcu_temp_callback(self, read_time, read_value):
        temp = read_value / 10.0
        self.temperature_callback(read_time + SAMPLE_TIME, temp)
    def hcu_pwm_callback(self, read_time, read_value):
        if self.pwm_callback is None:
            return
        duty_cycle = read_value / 65535.
        self.pwm_callback(read_time + SAMPLE_TIME, duty_cycle)

def load_config(config):
    pheaters = config.get_printer().load_object(config, "heaters")
    pheaters.add_sensor_factory("temperature_hcu", PrinterTemperatureHCU)
