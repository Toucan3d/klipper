# Step loss detection for a rotating axis that passes its endstop every turn
#
# The endstop of the A axis is a pin that the axis passes once per revolution.
# This module watches that pin while the axis is moving and compares what it
# sees with the commanded stepper position:
#
#  - every edge of the endstop signal must occur at the angle where the
#    endstop was found during homing, and
#  - whenever the commanded position passes over the pin the endstop must
#    trigger, and it must release again when the position leaves the pin.
#
# The edges are timestamped by the micro-controller (pulse_counter.c) on the
# same clock that schedules the steps, so no motion has to stop for the check.
#
# Copyright (C) 2026  Toucan3D
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import math
from . import pulse_counter

# Time span used to measure the commanded speed at a given time
SPEED_WINDOW = 0.002

class AAxisCheck:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.name = config.get_name()
        # The monitored stepper. Loading it here also makes sure its endstop
        # registers the (shared) pin before the counter does.
        self.stepper_name = config.get('stepper', 'manual_stepper stepper_a')
        if not config.has_section(self.stepper_name):
            raise config.error("%s: unknown stepper section '%s'"
                               % (self.name, self.stepper_name))
        mstepper = self.printer.load_object(config, self.stepper_name)
        if not hasattr(mstepper, 'get_steppers'):
            raise config.error("%s: '%s' is not a stepper section"
                               % (self.name, self.stepper_name))
        self.stepper = mstepper.get_steppers()[0]
        self.step_dist = self.stepper.get_step_dist()
        # Endstop geometry, in units of the stepper position (degrees)
        self.period = config.getfloat('period', 360., above=0.)
        self.center = config.getfloat('center', 0.)
        self.span = config.getfloat('span', None, above=0.,
                                    below=self.period)
        self.hysteresis = config.getfloat('hysteresis', .5, minval=0.)
        self.tolerance = config.getfloat('tolerance', 1., minval=0.)
        self.timing_tolerance = config.getfloat('timing_tolerance', 0.0005,
                                                minval=0.)
        self.log = config.getboolean('log', False)
        # Edge counter on the endstop pin
        self.poll_time = config.getfloat('poll_time', 0.0001, above=0.)
        sample_time = config.getfloat('sample_time', 0.005,
                                      above=self.poll_time)
        ppins = self.printer.lookup_object('pins')
        pin = config.get('pin', None)
        share_pin = pin is None
        if share_pin:
            pin = config.getsection(self.stepper_name).get('endstop_pin')
        pin_params = ppins.parse_pin(pin, can_invert=True, can_pullup=True)
        self.invert = bool(pin_params['invert'])
        pin_name = "%s:%s" % (pin_params['chip_name'], pin_params['pin'])
        if share_pin:
            ppins.allow_multi_use_pin(pin_name)
        pullup = {1: '^', -1: '~'}.get(pin_params['pullup'], '')
        self.counter = pulse_counter.MCU_counter(
            self.printer, pullup + pin_name, sample_time, self.poll_time)
        self.counter.setup_callback(self._counter_callback)
        # Fault reaction
        self.pause_on_fault = config.getboolean('pause_on_fault', True)
        if self.pause_on_fault:
            self.printer.load_object(config, 'pause_resume')
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.fault_gcode = gcode_macro.load_template(config, 'fault_gcode', '')
        # State
        self.armed = False
        self.arm_time = 0.
        self.pos_offset = 0.
        self.prev_report = None
        self.last_count = None
        self.state_info = "not enabled yet"
        self.fault = ""
        self._reset_stats()
        # Motor disable and homing invalidate the reference
        stepper_enable = self.printer.load_object(config, 'stepper_enable')
        self.enable_line = stepper_enable.lookup_enable(
            self.stepper.get_name())
        self.enable_line.register_state_callback(self._handle_motor_state)
        self.printer.register_event_handler("homing:homing_move_begin",
                                            self._handle_homing_move_begin)
        # Commands
        self.gcode.register_command('SET_A_AXIS_CHECK',
                                    self.cmd_SET_A_AXIS_CHECK,
                                    desc=self.cmd_SET_A_AXIS_CHECK_help)
        self.gcode.register_command('QUERY_A_AXIS_CHECK',
                                    self.cmd_QUERY_A_AXIS_CHECK,
                                    desc=self.cmd_QUERY_A_AXIS_CHECK_help)

    def _reset_stats(self):
        # [edge count, smallest deviation, largest deviation]
        self.edge_stats = {'trigger': [0, 0., 0.], 'release': [0, 0., 0.]}
        self.max_speed = 0.

    # Arming
    def _disarm(self, reason):
        if self.armed:
            logging.info("%s: disabled (%s)", self.name, reason)
        self.armed = False
        self.state_info = reason

    def _handle_motor_state(self, print_time, is_enable):
        if not is_enable:
            self._disarm("motor disabled")

    def _handle_homing_move_begin(self, hmove):
        for mcu_endstop in hmove.get_mcu_endstops():
            if self.stepper in mcu_endstop.get_steppers():
                self._disarm("homing")
                return

    def _arm(self, gcmd):
        if self.span is None:
            raise gcmd.error("%s: SPAN must be given (no span configured)"
                             % (self.name,))
        if not self.enable_line.is_motor_enabled():
            raise gcmd.error("%s: stepper '%s' is not enabled, home it first"
                             % (self.name, self.stepper_name))
        toolhead = self.printer.lookup_object('toolhead')
        # Reports older than the moves queued so far are not checked
        self.arm_time = toolhead.get_last_move_time()
        # Positions are tracked in micro-controller steps, so later position
        # resets of the stepper do not move the reference.
        self.pos_offset = self.stepper.mcu_to_commanded_position(0)
        self.prev_report = None
        self.fault = ""
        self.state_info = "enabled"
        self._reset_stats()
        self.armed = True

    # Geometry helpers
    def _position(self, print_time):
        mcu_pos = self.stepper.get_past_mcu_position(print_time)
        return mcu_pos * self.step_dist + self.pos_offset

    def _speed(self, print_time, pos):
        prev_pos = self._position(print_time - SPEED_WINDOW)
        return abs(pos - prev_pos) / SPEED_WINDOW

    def _from_center(self, pos):
        # Signed distance to the nearest endstop center
        half_period = .5 * self.period
        return (pos - self.center + half_period) % self.period - half_period

    def _margin(self, speed):
        return self.tolerance + speed * (self.timing_tolerance
                                         + self.poll_time)

    # Report handling
    def _counter_callback(self, time, count, count_time):
        # Invoked from the serial thread
        self.last_count = count
        if self.armed:
            self.reactor.register_async_callback(
                (lambda e: self._process_report(time, count, count_time)))

    def _process_report(self, time, count, count_time):
        if not self.armed or time < self.arm_time:
            return
        prev = self.prev_report
        self.prev_report = (time, count)
        if prev is None or time <= prev[0]:
            return
        prev_time, prev_count = prev
        triggered = bool(count & 1) != self.invert
        # The endstop level is constant since the last edge
        start_time = prev_time
        if count != prev_count:
            start_time = min(time, max(prev_time, count_time))
            was_triggered = bool(prev_count & 1) != self.invert
            msg = self._check_edge(start_time, triggered and not was_triggered)
            if msg is not None:
                return self._handle_fault(msg)
        msg = self._check_level(start_time, time, triggered)
        if msg is not None:
            self._handle_fault(msg)

    def _check_edge(self, edge_time, is_trigger):
        # The endstop changed state at edge_time: that must have happened
        # at one of the two flanks of the pin. Only a change from open to
        # triggered is known to be the trigger point found by homing. Any
        # other sequence ends with a release (or with contact chatter around
        # one), which may be up to "hysteresis" further out.
        pos = self._position(edge_time)
        speed = self._speed(edge_time, pos)
        margin = self._margin(speed)
        offset = self._from_center(pos)
        deviation = abs(offset) - .5 * self.span
        kind = 'trigger'
        max_deviation = margin
        if not is_trigger:
            kind = 'release'
            max_deviation += self.hysteresis
        stats = self.edge_stats[kind]
        if not stats[0]:
            stats[1] = stats[2] = deviation
        stats[0] += 1
        stats[1] = min(stats[1], deviation)
        stats[2] = max(stats[2], deviation)
        self.max_speed = max(self.max_speed, speed)
        if self.log:
            logging.info("%s: %s at %.3f from center, deviation %.3f"
                         " (allowed %.3f..%.3f), speed %.1f", self.name, kind,
                         offset, deviation, -margin, max_deviation, speed)
        if -margin <= deviation <= max_deviation:
            return None
        return ("endstop %s at %.2f deg from its center, expected at"
                " %.2f deg (allowed deviation %.2f..%.2f deg at %.0f deg/s)"
                % (kind, offset, math.copysign(.5 * self.span, offset),
                   -margin, max_deviation, speed))

    def _check_level(self, start_time, end_time, triggered):
        # The endstop kept its state between start_time and end_time: the
        # commanded position must not have been where the other state is
        # certain.
        start_pos = self._position(start_time)
        end_pos = self._position(end_time)
        speed = max(abs(end_pos - start_pos) / max(end_time - start_time,
                                                   self.poll_time),
                    self._speed(end_time, end_pos))
        margin = self._margin(speed)
        low = self._from_center(min(start_pos, end_pos))
        high = low + abs(end_pos - start_pos)
        half_span = .5 * self.span
        if triggered:
            limit = half_span + self.hysteresis + margin
            if low >= -limit and high <= limit:
                return None
            far = high if high > limit else low
            return ("endstop still triggered with the axis commanded to"
                    " %.2f deg from the endstop center (it must release"
                    " within %.2f deg at %.0f deg/s)" % (far, limit, speed))
        # Not triggered: the position must not have covered the part of the
        # pin where the endstop is certain to trigger.
        sure = half_span - margin
        turn = math.floor((low - sure) / self.period) + 1.
        if high <= turn * self.period - sure:
            return None
        return ("endstop did not trigger while the axis was commanded from"
                " %.2f to %.2f deg relative to the endstop center (it must"
                " trigger within %.2f deg at %.0f deg/s)"
                % (low, high, max(sure, 0.), speed))

    # Fault reaction
    def _handle_fault(self, msg):
        msg = ("%s: %s. Stepper '%s' has lost steps."
               % (self.name, msg, self.stepper_name))
        logging.error(msg)
        self._disarm("fault")
        self.fault = msg
        self.reactor.register_callback(self._fault_event)

    def _is_printing(self, eventtime):
        print_stats = self.printer.lookup_object('print_stats', None)
        if print_stats is not None:
            return print_stats.get_status(eventtime)['state'] == "printing"
        idle_timeout = self.printer.lookup_object('idle_timeout')
        return idle_timeout.get_status(eventtime)['state'] == "Printing"

    def _fault_event(self, eventtime):
        self.gcode.respond_raw("!! %s" % (self.fault,))
        script = ""
        if self.pause_on_fault and self._is_printing(eventtime):
            # Pausing from inside an event requires that the pause portion
            # of pause_resume execute immediately.
            pause_resume = self.printer.lookup_object('pause_resume')
            pause_resume.send_pause_command()
            script = "PAUSE\n"
        try:
            script += self.fault_gcode.render()
            if script.strip():
                self.gcode.run_script(script + "\nM400")
        except Exception:
            logging.exception("%s: fault script running error", self.name)

    # Status and commands
    def _is_triggered(self):
        if self.last_count is None:
            return None
        return bool(self.last_count & 1) != self.invert

    def get_status(self, eventtime=None):
        trigger = self.edge_stats['trigger']
        release = self.edge_stats['release']
        return {'enabled': self.armed, 'state': self.state_info,
                'fault': self.fault, 'triggered': self._is_triggered(),
                'center': self.center, 'span': self.span,
                'hysteresis': self.hysteresis, 'tolerance': self.tolerance,
                'timing_tolerance': self.timing_tolerance,
                'trigger_count': trigger[0],
                'trigger_deviation_min': trigger[1],
                'trigger_deviation_max': trigger[2],
                'release_count': release[0],
                'release_deviation_min': release[1],
                'release_deviation_max': release[2],
                'max_speed': self.max_speed}

    cmd_SET_A_AXIS_CHECK_help = ("Enable or disable the A axis step loss"
        " check and set its parameters: ENABLE CENTER SPAN HYSTERESIS"
        " TOLERANCE TIMING_TOLERANCE LOG")
    def cmd_SET_A_AXIS_CHECK(self, gcmd):
        enable = gcmd.get_int('ENABLE', None, minval=0, maxval=1)
        self.center = gcmd.get_float('CENTER', self.center)
        self.span = gcmd.get_float('SPAN', self.span, above=0.,
                                   below=self.period)
        # A homing macro may pass a slightly negative measured hysteresis
        self.hysteresis = max(0., gcmd.get_float('HYSTERESIS',
                                                 self.hysteresis))
        self.tolerance = gcmd.get_float('TOLERANCE', self.tolerance,
                                        minval=0.)
        self.timing_tolerance = gcmd.get_float(
            'TIMING_TOLERANCE', self.timing_tolerance, minval=0.)
        self.log = bool(gcmd.get_int('LOG', int(self.log)))
        if enable:
            self._arm(gcmd)
        elif enable is not None:
            self._disarm("disabled by command")
        gcmd.respond_info(self._describe())

    cmd_QUERY_A_AXIS_CHECK_help = "Report the A axis step loss check state"
    def cmd_QUERY_A_AXIS_CHECK(self, gcmd):
        gcmd.respond_info(self._describe())

    def _describe(self):
        triggered = self._is_triggered()
        endstop = "unknown"
        if triggered is not None:
            endstop = "triggered" if triggered else "open"
        span = "not set" if self.span is None else "%.3f" % (self.span,)
        text = ("%s: %s, endstop %s\ncenter %.3f, span %s, hysteresis %.3f,"
                " tolerance %.3f + %.4f s * speed"
                % (self.name, self.state_info, endstop, self.center, span,
                   self.hysteresis, self.tolerance,
                   self.timing_tolerance + self.poll_time))
        for kind in ('trigger', 'release'):
            count, dev_min, dev_max = self.edge_stats[kind]
            if count:
                text += ("\n%s edges: %d, deviation %.3f..%.3f"
                         % (kind, count, dev_min, dev_max))
            else:
                text += "\n%s edges: none" % (kind,)
        text += "\nhighest speed at an edge: %.1f" % (self.max_speed,)
        if self.fault:
            text += "\nlast fault: %s" % (self.fault,)
        return text

def load_config(config):
    return AAxisCheck(config)
