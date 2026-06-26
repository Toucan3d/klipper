# Support for MLX90614 I2C infrared temperature sensors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import bus

MLX90614_I2C_ADDR = 0x5a
MLX90614_I2C_SPEED = 100000
MLX90614_REPORT_TIME = 0.3
MLX90614_MIN_REPORT_TIME = 0.1

MLX90614_REGS = {
    'ambient': 0x06,
    'object1': 0x07,
    'object2': 0x08,
    'emissivity': 0x24,
    'gradient': 0x2f,
}

MLX90614_EEPROM_UNLOCK = 0x60
MLX90614_EEPROM_DELAY = 0.002


class MLX90614:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        self.i2c = bus.MCU_I2C_from_config(
            config, default_addr=MLX90614_I2C_ADDR,
            default_speed=MLX90614_I2C_SPEED)
        self.i2c_addr = self.i2c.get_i2c_address()
        self.report_time = config.getfloat(
            'mlx90614_report_time', MLX90614_REPORT_TIME,
            minval=MLX90614_MIN_REPORT_TIME)
        self.temperature_source = config.getchoice(
            'mlx90614_temperature_source', {
                'ambient': 'ambient', 'object1': 'object1', 'object2': 'object2'
            }, 'object1')
        self.emissivity = config.getfloat(
            'emissivity', 1.0, above=0., maxval=1.)
        self.emissivity_reg = self._emissivity_to_reg(self.emissivity)
        self.temp = self.min_temp = self.max_temp = 0.0
        self.sample_timer = self.reactor.register_timer(self._sample_mlx90614)
        self.printer.add_object('mlx90614 ' + self.name, self)
        self.printer.register_event_handler('klippy:connect',
                                            self.handle_connect)

    def handle_connect(self):
        self._init_mlx90614()
        self.reactor.update_timer(self.sample_timer, self.reactor.NOW)

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp

    def setup_callback(self, cb):
        self._callback = cb

    def get_report_time_delta(self):
        return self.report_time

    def _init_mlx90614(self):
        current_emissivity = self._read_word('emissivity')
        if current_emissivity != self.emissivity_reg:
            self._write_emissivity(current_emissivity)
        logging.info(
            'mlx90614 %s: source=%s address=%#x emissivity=%.4f'
            % (self.name, self.temperature_source, self.i2c_addr,
               self.emissivity))

    def _sample_mlx90614(self, eventtime):
        try:
            raw = self._read_word(self.temperature_source)
            self.temp = raw * 0.02 - 273.15
        except Exception:
            logging.exception('mlx90614: Error reading data')
            self.temp = 0.0
            return self.reactor.NEVER

        if self.temp < self.min_temp or self.temp > self.max_temp:
            self.printer.invoke_shutdown(
                'MLX90614 temperature %.1f outside range of %.1f:%.1f'
                % (self.temp, self.min_temp, self.max_temp))

        measured_time = self.reactor.monotonic()
        print_time = self.i2c.get_mcu().estimated_print_time(measured_time)
        self._callback(print_time, self.temp)
        return measured_time + self.report_time

    def _read_word(self, reg_name):
        reg = MLX90614_REGS[reg_name]
        params = self.i2c.i2c_read([reg], 3)
        response = bytearray(params['response'])
        if len(response) != 3:
            raise self.printer.command_error(
                'MLX90614 read from %#x returned %d bytes'
                % (reg, len(response)))
        expected = self._pec([self.i2c_addr << 1, reg,
                              (self.i2c_addr << 1) | 1,
                              response[0], response[1]])
        if response[2] != expected:
            raise self.printer.command_error(
                'MLX90614 PEC mismatch on register %#x: got %#x expected %#x'
                % (reg, response[2], expected))
        return response[0] | (response[1] << 8)

    def _write_word(self, reg_name, value):
        reg = MLX90614_REGS[reg_name]
        data = [reg, value & 0xff, (value >> 8) & 0xff]
        data.append(self._pec([self.i2c_addr << 1] + data))
        self.i2c.i2c_write(data)

    def _unlock_eeprom(self):
        cmd = MLX90614_EEPROM_UNLOCK
        self.i2c.i2c_write([cmd, self._pec([self.i2c_addr << 1, cmd])])

    def _write_emissivity(self, old_emissivity_reg):
        if old_emissivity_reg <= 0:
            raise self.printer.config_error(
                'MLX90614 current emissivity register is invalid')
        old_gradient_reg = self._read_word('gradient')
        new_gradient_reg = (old_emissivity_reg * old_gradient_reg
                            // self.emissivity_reg)
        if new_gradient_reg > 0xffff:
            raise self.printer.config_error(
                'MLX90614 emissivity %.4f would overflow gradient register'
                % (self.emissivity,))

        self._unlock_eeprom()
        self.reactor.pause(self.reactor.monotonic() + MLX90614_EEPROM_DELAY)
        self._write_word('emissivity', 0)
        self.reactor.pause(self.reactor.monotonic() + MLX90614_EEPROM_DELAY)
        self._write_word('emissivity', self.emissivity_reg)
        self.reactor.pause(self.reactor.monotonic() + MLX90614_EEPROM_DELAY)
        self._write_word('gradient', 0)
        self.reactor.pause(self.reactor.monotonic() + MLX90614_EEPROM_DELAY)
        self._write_word('gradient', new_gradient_reg)
        self.reactor.pause(self.reactor.monotonic() + MLX90614_EEPROM_DELAY)

        received = self._read_word('emissivity')
        if received != self.emissivity_reg:
            raise self.printer.config_error(
                'MLX90614 emissivity write failed: got %#x expected %#x'
                % (received, self.emissivity_reg))
        logging.info(
            'mlx90614 %s: emissivity register %#x, gradient register %#x'
            % (self.name, self.emissivity_reg, new_gradient_reg))

    def _emissivity_to_reg(self, emissivity):
        value = int(emissivity * 0xffff)
        if value <= 0:
            raise self.printer.config_error(
                'MLX90614 emissivity register must be non-zero')
        return value

    def _pec(self, data):
        crc = 0
        for value in data:
            crc ^= value
            for _ in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) ^ 0x07) & 0xff
                else:
                    crc = (crc << 1) & 0xff
        return crc

    def get_status(self, eventtime):
        return {
            'temperature': round(self.temp, 2),
            'emissivity': round(self.emissivity, 4),
        }


def load_config(config):
    pheater = config.get_printer().lookup_object('heaters')
    pheater.add_sensor_factory('MLX90614', MLX90614)
