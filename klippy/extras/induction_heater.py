# Optional induction heater support commands.
#
# This module intentionally stays outside the normal heater PID path. Use
# standard Klipper heaters with heater_pin set to INDUCTION0/INDUCTION1, and
# load this extra only for board-specific current-limit and resonance commands.

import mcu


RESONANCE_START = 100000
RESONANCE_STOP = 250000
RESONANCE_STEP = 1000
RESONANCE_TOLERANCE = 5
RESONANCE_STABLE_COUNT = 3
RESONANCE_REPORT_TIME = 0.100
RESONANCE_RESULT_FILE = "/tmp/resonance_result.txt"


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
        self.sweep_states = {}
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
            'induction_measure_resonance oid=%c enable=%c',
            cq=self.cmd_queue)
        for oid in range(self.channel_count):
            self.resonance_responses.append(
                self.mcu.register_serial_response(
                    self._handle_resonance_power,
                    'induction_resonance_power oid=%c frequency=%u power=%u',
                    oid))

    def _check_ready(self):
        if self.set_current_limit_cmd is None:
            raise self.printer.command_error(
                "Induction MCU commands are not available yet")

    def _channel_names(self):
        return '/'.join('INDUCTION%d' % (i,) for i in range(self.channel_count))

    def _handle_resonance_power(self, params):
        oid = params['oid']
        state = self.sweep_states.get(oid)
        if state is None or params['frequency'] != state['frequency']:
            return
        state['samples'].append(params['power'])
        stable_count = state['stable_count']
        if len(state['samples']) < stable_count:
            return
        samples = state['samples'][-stable_count:]
        if max(samples) - min(samples) > state['tolerance']:
            return
        completion = state.get('completion')
        if completion is not None:
            state['completion'] = None
            self.reactor.async_complete(
                completion, (sum(samples) / float(stable_count), samples))

    def _get_sweep_frequencies(self, start, stop, step):
        frequencies = list(range(start, stop + 1, step))
        if not frequencies or frequencies[-1] != stop:
            frequencies.append(stop)
        return frequencies

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
        if channel in self.sweep_states:
            raise gcmd.error(
                "Induction channel %d resonance measurement already in progress"
                % (channel,))
        start = gcmd.get_int('START', RESONANCE_START, minval=1)
        stop = gcmd.get_int('STOP', RESONANCE_STOP, minval=1)
        step = gcmd.get_int('STEP', RESONANCE_STEP, minval=1)
        tolerance = gcmd.get_int('TOLERANCE', RESONANCE_TOLERANCE, minval=0)
        stable_count = gcmd.get_int(
            'STABLE_COUNT', RESONANCE_STABLE_COUNT, minval=1)
        if stop < start:
            raise gcmd.error("STOP must be greater than or equal to START")
        frequencies = self._get_sweep_frequencies(start, stop, step)
        timeout = gcmd.get_float('TIMEOUT', None, above=0.)
        if timeout is None:
            timeout = max(30., (len(frequencies) * stable_count
                               * RESONANCE_REPORT_TIME * 2.))
        deadline = self.reactor.monotonic() + timeout
        log_file = open(RESONANCE_RESULT_FILE, 'w')
        log_file.write(
            "channel=%d start=%d stop=%d step=%d tolerance=%d"
            " stable_count=%d timeout=%.3f\n"
            % (channel, start, stop, step, tolerance, stable_count, timeout))
        log_file.flush()
        state = {
            'frequency': None, 'samples': [], 'completion': None,
            'stable_count': stable_count, 'tolerance': tolerance,
        }
        self.sweep_states[channel] = state
        best_frequency = None
        best_power = None
        calibration_started = False
        try:
            for frequency in frequencies:
                state['frequency'] = frequency
                state['samples'] = []
                state['completion'] = self.reactor.completion()
                self.set_frequency_cmd.send_wait_ack([channel, frequency])
                if not calibration_started:
                    self.measure_resonance_cmd.send_wait_ack([channel, 1])
                    calibration_started = True
                result = state['completion'].wait(deadline)
                if result is None:
                    log_file.write("frequency=%d timeout\n" % (frequency,))
                    log_file.flush()
                    raise gcmd.error(
                        "Timed out waiting for stable induction resonance"
                        " power at %d Hz" % (frequency,))
                power, samples = result
                log_file.write(
                    "frequency=%d power=%.3f samples=%s\n"
                    % (frequency, power,
                       ','.join(["%d" % (sample,) for sample in samples])))
                log_file.flush()
                if best_power is None or power > best_power:
                    best_power = power
                    best_frequency = frequency
            self.set_frequency_cmd.send_wait_ack([channel, best_frequency])
            log_file.write("best_frequency=%d best_power=%.3f\n"
                           % (best_frequency, best_power))
            log_file.flush()
        finally:
            state['completion'] = None
            self.sweep_states.pop(channel, None)
            if calibration_started:
                self.measure_resonance_cmd.send_wait_ack([channel, 0])
            log_file.close()
        gcmd.respond_info(
            "Induction channel %d resonance frequency: %d Hz"
            " (power %.3f W)" % (channel, best_frequency, best_power))


def load_config(config):
    return InductionHeater(config)
