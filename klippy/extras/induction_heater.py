# Optional induction heater support commands.
#
# This module intentionally stays outside the normal heater PID path. Use
# standard Klipper heaters with heater_pin set to INDUCTION0/INDUCTION1, and
# load this extra only for board-specific current-limit and resonance commands.

import mcu


class InductionHeater:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        mcu_name = config.get('mcu', 'mcu')
        self.mcu = mcu.get_printer_mcu(self.printer, mcu_name)
        default_channels = 1 if mcu_name == 'hcu' else 2
        self.channel_count = config.getint(
            'channels', default_channels, minval=1, maxval=2)
        self.cmd_queue = self.mcu.alloc_command_queue()
        self.set_current_limit_cmd = None
        self.set_frequency_cmd = None
        self.measure_resonance_cmd = None
        self.pending_results = {}
        self.resonance_responses = []
        self.mcu.register_config_callback(self._build_config)

        gcode = self.printer.lookup_object('gcode')
        gcode.register_command(
            'SET_INDUCTION_CURRENT_LIMIT',
            self.cmd_SET_INDUCTION_CURRENT_LIMIT,
            desc=self.cmd_SET_INDUCTION_CURRENT_LIMIT_help)
        gcode.register_command(
            'SET_INDUCTION_RESONANCE_FREQUENCY',
            self.cmd_SET_INDUCTION_RESONANCE_FREQUENCY,
            desc=self.cmd_SET_INDUCTION_RESONANCE_FREQUENCY_help)
        gcode.register_command(
            'MEASURE_INDUCTION_RESONANCE',
            self.cmd_MEASURE_INDUCTION_RESONANCE,
            desc=self.cmd_MEASURE_INDUCTION_RESONANCE_help)

    def _build_config(self):
        self.set_current_limit_cmd = self.mcu.lookup_command(
            'induction_set_current_limit value=%u', cq=self.cmd_queue)
        self.set_frequency_cmd = self.mcu.lookup_command(
            'induction_set_resonance_frequency oid=%c value=%u',
            cq=self.cmd_queue)
        self.measure_resonance_cmd = self.mcu.lookup_command(
            'induction_measure_resonance oid=%c', cq=self.cmd_queue)
        for oid in range(self.channel_count):
            self.resonance_responses.append(
                self.mcu.register_serial_response(
                    self._handle_resonance_result,
                    'induction_resonance_result oid=%c value=%u', oid))

    def _check_ready(self):
        if self.set_current_limit_cmd is None:
            raise self.printer.command_error(
                "Induction MCU commands are not available yet")

    def _channel_names(self):
        return '/'.join('INDUCTION%d' % (i,) for i in range(self.channel_count))

    def _handle_resonance_result(self, params):
        oid = params['oid']
        completion = self.pending_results.pop(oid, None)
        if completion is not None:
            self.reactor.async_complete(completion, params)

    def _channel_from_gcmd(self, gcmd):
        channel = gcmd.get_int(
            'CHANNEL', None, minval=0, maxval=self.channel_count - 1)
        if channel is not None:
            return channel

        heater = gcmd.get('HEATER')
        settings = self.printer.lookup_object(
            'configfile').get_status(None)['settings']
        heater_l = heater.lower()
        for section, values in settings.items():
            section_l = section.lower()
            if section_l != heater_l and not section_l.endswith(' ' + heater_l):
                continue
            heater_pin = values.get('heater_pin', '').upper()
            for channel in range(self.channel_count):
                if heater_pin.endswith('INDUCTION%d' % (channel,)):
                    return channel
        raise gcmd.error("Unable to map heater '%s' to %s"
                         % (heater, self._channel_names()))

    cmd_SET_INDUCTION_CURRENT_LIMIT_help = (
        "Set the induction input current limit in amps")
    def cmd_SET_INDUCTION_CURRENT_LIMIT(self, gcmd):
        self._check_ready()
        current = gcmd.get_float('CURRENT', minval=0.)
        self.set_current_limit_cmd.send_wait_ack([int(current * 1000. + .5)])
        gcmd.respond_info("Induction current limit set to %.3f A" % (current,))

    cmd_SET_INDUCTION_RESONANCE_FREQUENCY_help = (
        "Set an induction channel resonance frequency")
    def cmd_SET_INDUCTION_RESONANCE_FREQUENCY(self, gcmd):
        self._check_ready()
        channel = self._channel_from_gcmd(gcmd)
        frequency = gcmd.get_int('FREQUENCY', minval=1)
        self.set_frequency_cmd.send_wait_ack([channel, frequency])
        gcmd.respond_info(
            "Induction channel %d resonance frequency set to %d Hz"
            % (channel, frequency))

    cmd_MEASURE_INDUCTION_RESONANCE_help = (
        "Run an induction resonance sweep and report the best frequency")
    def cmd_MEASURE_INDUCTION_RESONANCE(self, gcmd):
        self._check_ready()
        channel = self._channel_from_gcmd(gcmd)
        timeout = gcmd.get_float('TIMEOUT', 30., above=0.)
        completion = self.reactor.completion()
        self.pending_results[channel] = completion
        self.measure_resonance_cmd.send_wait_ack([channel])
        result = completion.wait(self.reactor.monotonic() + timeout)
        if result is None:
            self.pending_results.pop(channel, None)
            raise gcmd.error("Timed out waiting for induction resonance result")
        gcmd.respond_info(
            "Induction channel %d resonance frequency: %d Hz"
            % (channel, result['value']))


def load_config(config):
    return InductionHeater(config)
