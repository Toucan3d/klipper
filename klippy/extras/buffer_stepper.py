# Support for the Sovol filament buffer stepper
#
# Copyright (C) 2025  Sovol3d <info@sovol3d.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import stepper
from . import force_move

MIN_KIN_TIME = 0.050
SDS_CHECK_TIME = 0.001


class BufferStepper:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        stepper_name = self.name.split()[1]
        self.gcode = self.printer.lookup_object('gcode')
        self.mcu = None
        for name, mcu in self.printer.lookup_objects(module='mcu'):
            if mcu.get_name() == 'buffer_mcu':
                self.mcu = mcu
                break
        if self.mcu is None:
            raise self.printer.config_error(
                "Must set separate MCU for buffer_stepper")

        self.reactor = self.printer.get_reactor()
        self.debug = config.getboolean('debug', False)
        self.buffer_time_start = config.getfloat(
            'buffer_time_start', 0.050, above=0.)
        self.last_kin_flush_time = 0.
        self.kin_flush_delay = SDS_CHECK_TIME

        buttons = self.printer.load_object(config, 'buttons')
        self.push = config.get('push_pin')
        buttons.register_buttons([self.push], self._push_handler)
        self.push_triggered = False
        self.min_event_systime = self.reactor.NEVER
        self.event_delay = config.getfloat('event_delay', 2.0, above=0.)
        self.current_check_id = 0

        self.rail = stepper.PrinterStepper(config)
        self.steppers = [self.rail]
        self.velocity = config.getfloat('velocity', 150., above=0.)
        self.accel = self.homing_accel = config.getfloat(
            'accel', 5000., minval=0.)
        self.push_length = config.getfloat('push_length', 25., minval=1.)
        self.next_cmd_time = 0.
        self.commanded_pos = 0.

        self.motion_queuing = self.printer.load_object(
            config, 'motion_queuing')
        self.trapq = self.motion_queuing.allocate_trapq()
        self.trapq_append = self.motion_queuing.lookup_trapq_append()
        self.rail.setup_itersolve('cartesian_stepper_alloc', b'x')
        self.rail.set_trapq(self.trapq)

        self.printer.register_event_handler("klippy:ready",
                                            self._handle_ready)
        self.print_stats = self.printer.load_object(config, 'print_stats')
        self.gcode.register_mux_command('BUFFER_STEPPER', "STEPPER",
                                        stepper_name,
                                        self.cmd_BUFFER_STEPPER,
                                        desc=self.cmd_BUFFER_STEPPER_help)

    def _push_handler(self, eventtime, state):
        if state == self.push_triggered:
            return
        self.push_triggered = state
        eventtime = self.reactor.monotonic()
        if eventtime < self.min_event_systime:
            return
        self.current_check_id += 1
        if not self.push_triggered:
            return
        global_var = self.printer.lookup_object('gcode_macro _global_var')
        status = global_var.get_status(eventtime)
        is_push_buffer = status.get('is_push_buffer', True)
        if not is_push_buffer:
            return
        self.do_move(self.push_length, self.velocity, self.accel)
        if self.print_stats.state == 'printing':
            self.gcode.run_script_from_command("NOZZLE_CLOG_CHECK")
        initial_push_state = self.push_triggered
        check_id = self.current_check_id
        delay = self.get_move_duration(self.push_length, self.velocity,
                                       self.accel) + self.event_delay
        self.reactor.register_async_callback(
            lambda evt: self._check_filament_jam(evt, initial_push_state,
                                                 check_id),
            eventtime + delay)

    def _handle_ready(self):
        self.min_event_systime = self.reactor.monotonic() + 0.2

    def sync_print_time(self):
        if hasattr(self.motion_queuing, 'get_kin_flush_delay'):
            self.kin_flush_delay = self.motion_queuing.get_kin_flush_delay()
        curtime = self.reactor.monotonic()
        est_print_time = self.mcu.estimated_print_time(curtime)
        kin_time = max(est_print_time + MIN_KIN_TIME, self.last_kin_flush_time)
        kin_time += self.kin_flush_delay
        min_print_time = max(est_print_time + self.buffer_time_start, kin_time)
        if min_print_time > self.next_cmd_time:
            self.next_cmd_time = min_print_time

    def do_enable(self, enable):
        self.sync_print_time()
        stepper_enable = self.printer.lookup_object('stepper_enable')
        for s in self.steppers:
            se = stepper_enable.lookup_enable(s.get_name())
            if enable:
                se.motor_enable(self.next_cmd_time)
            else:
                se.motor_disable(self.next_cmd_time)
        self.sync_print_time()

    def do_set_position(self, setpos):
        self.commanded_pos = setpos
        self.rail.set_position([self.commanded_pos, 0., 0.])

    def _submit_move(self, movetime, move_dist, speed, accel):
        cp = self.commanded_pos
        movepos = cp + move_dist
        axis_r, accel_t, cruise_t, cruise_v = force_move.calc_move_time(
            move_dist, speed, accel)
        self.trapq_append(self.trapq, movetime,
                          accel_t, cruise_t, accel_t,
                          cp, 0., 0., axis_r, 0., 0.,
                          0., cruise_v, accel)
        self.commanded_pos = movepos
        return movetime + accel_t + cruise_t + accel_t

    def do_move(self, movepos, speed, accel, sync=True):
        self.sync_print_time()
        self.next_cmd_time = self._submit_move(self.next_cmd_time, movepos,
                                               speed, accel)
        self.last_kin_flush_time = self.next_cmd_time
        self.motion_queuing.note_mcu_movequeue_activity(self.next_cmd_time)

    def get_move_duration(self, movepos, speed, accel):
        axis_r, accel_t, cruise_t, cruise_v = force_move.calc_move_time(
            movepos, speed, accel)
        return accel_t + cruise_t + accel_t

    def _check_filament_jam(self, eventtime, initial_push_state, check_id):
        if (self.current_check_id != check_id
            or self.push_triggered != initial_push_state
            or self.print_stats.state == 'paused'
            or getattr(self, '_in_pause', False)):
            return
        filament_sensor = self.printer.lookup_object(
            'filament_switch_sensor filament_sensor')
        filament_detected = filament_sensor.runout_helper.filament_present
        if filament_detected and self.push_triggered:
            self.gcode.respond_info(
                "Filament jam detected! Push pin is still triggered after move.")
            if self.print_stats is not None and self.print_stats.state == 'printing':
                try:
                    self._in_pause = True
                    self.gcode.run_script_from_command(
                        "SET_PIN PIN=green_led VALUE=0.00")
                    self.gcode.run_script_from_command(
                        "SET_PIN PIN=blue_led VALUE=1.00")
                    self.gcode.run_script_from_command("PAUSE")
                    self.gcode.run_script_from_command(
                        "SET_GCODE_VARIABLE MACRO=variables"
                        " VARIABLE=winding_status VALUE=True")
                    self.gcode.respond_info("Filament winding")
                finally:
                    self._in_pause = False
        elif filament_detected:
            self.gcode.run_script_from_command(
                "SET_GCODE_VARIABLE MACRO=variables"
                " VARIABLE=winding_status VALUE=False")

    cmd_BUFFER_STEPPER_help = "Command a manually configured stepper"

    def cmd_BUFFER_STEPPER(self, gcmd):
        enable = gcmd.get_int('ENABLE', None)
        if enable is not None:
            self.do_enable(enable)
        setpos = gcmd.get_float('SET_POSITION', None)
        if setpos is not None:
            self.do_set_position(setpos)
        speed = gcmd.get_float('SPEED', self.velocity, above=0.)
        accel = gcmd.get_float('ACCEL', self.accel, minval=0.)
        if gcmd.get_float('MOVE', None) is not None:
            movepos = gcmd.get_float('MOVE')
            sync = gcmd.get_int('SYNC', 1)
            self.do_move(movepos, speed, accel, sync)

    def flush_step_generation(self):
        self.motion_queuing.note_mcu_movequeue_activity(self.next_cmd_time)

    def get_position(self):
        return [self.commanded_pos, 0., 0., 0.]

    def set_position(self, newpos, homing_axes=""):
        self.do_set_position(newpos[0])

    def get_last_move_time(self):
        self.sync_print_time()
        return self.next_cmd_time

    def dwell(self, delay):
        self.next_cmd_time += max(0., delay)

    def drip_move(self, newpos, speed, drip_completion):
        self.sync_print_time()
        start_time = self.next_cmd_time
        end_time = self._submit_move(self.next_cmd_time,
                                     newpos[0] - self.commanded_pos,
                                     speed, self.homing_accel)
        self.motion_queuing.drip_update_time(start_time, end_time,
                                             drip_completion)
        self.motion_queuing.wipe_trapq(self.trapq)

    def get_kinematics(self):
        return self

    def get_steppers(self):
        return self.steppers

    def calc_position(self, stepper_positions):
        return [stepper_positions[self.rail.get_name()], 0., 0.]

    def debug_logging(self, message):
        self.gcode.respond_info(message)

    def get_status(self, eventtime):
        return {
            'push_triggered': self.push_triggered,
        }


def load_config_prefix(config):
    return BufferStepper(config)
