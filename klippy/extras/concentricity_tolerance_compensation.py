# XY compensation for concentricity error on a synchronized extra axis
#
# Copyright (C) 2026
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math


# PEP 485 isclose()
def isclose(a, b, rel_tol=1e-09, abs_tol=0.0):
    return abs(a - b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)


def lerp(t, v0, v1):
    return (1. - t) * v0 + t * v1


class MoveSplitter:
    def __init__(self, config, deflection_angle, deflection_radius,
                 move_check_distance_axis):
        self.deflection_angle = deflection_angle
        self.deflection_radius = deflection_radius
        self.split_delta_xy = config.getfloat('split_delta_xy', .025,
                                              minval=0.01)
        self.move_check_distance_axis = move_check_distance_axis

    def calc_xy_adjust(self, axis_pos):
        calc_deflection_angle = math.radians(axis_pos + self.deflection_angle)
        x_adj = self.deflection_radius * math.cos(calc_deflection_angle)
        y_adj = self.deflection_radius * math.sin(calc_deflection_angle)
        return x_adj, y_adj

    def _apply_xy_adjust(self, pos, axis_index):
        transformed = list(pos)
        x_adj, y_adj = self.calc_xy_adjust(pos[axis_index])
        transformed[0] += x_adj
        transformed[1] += y_adj
        return transformed

    def generate_moves(self, prev_pos, next_pos, axis_index):
        prev_pos = list(prev_pos)
        next_pos = list(next_pos)
        axis_d = next_pos[axis_index] - prev_pos[axis_index]
        if isclose(axis_d, 0., abs_tol=1e-10):
            yield self._apply_xy_adjust(next_pos, axis_index)
            return
        total_move_length = abs(axis_d)
        distance_checked = 0.
        current_pos = list(prev_pos)
        last_offset = self.calc_xy_adjust(prev_pos[axis_index])
        axes_d = [next_pos[i] - prev_pos[i] for i in range(len(next_pos))]
        axis_move = [not isclose(d, 0., abs_tol=1e-10) for d in axes_d]
        while (distance_checked + self.move_check_distance_axis
               < total_move_length):
            distance_checked += self.move_check_distance_axis
            t = distance_checked / total_move_length
            for i in range(len(next_pos)):
                if axis_move[i]:
                    current_pos[i] = lerp(t, prev_pos[i], next_pos[i])
            next_offset = self.calc_xy_adjust(current_pos[axis_index])
            if (abs(next_offset[0] - last_offset[0]) >= self.split_delta_xy
                or abs(next_offset[1] - last_offset[1]) >= self.split_delta_xy):
                last_offset = next_offset
                yield self._apply_xy_adjust(current_pos, axis_index)
        yield self._apply_xy_adjust(next_pos, axis_index)


class ConcentricityToleranceCompensation:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode_move = self.printer.load_object(config, 'gcode_move')
        self.axis = config.get('axis', 'A').upper()
        if (len(self.axis) != 1 or not self.axis.isupper()
            or self.axis in "XYZEFN"):
            raise config.error(
                "Option 'axis' in section '%s' must be a single extra"
                " G-Code axis letter" % (config.get_name(),))
        move_check_distance_axis = config.getfloat(
            'move_check_distance_axis', None, minval=0.01)
        legacy_move_check_distance = config.getfloat(
            'move_check_distance_a', None, minval=0.01)
        if (move_check_distance_axis is not None
            and legacy_move_check_distance is not None):
            raise config.error(
                "Options 'move_check_distance_axis' and"
                " 'move_check_distance_a' may not both be specified")
        if move_check_distance_axis is None:
            move_check_distance_axis = legacy_move_check_distance
        if move_check_distance_axis is None:
            move_check_distance_axis = 5.
        self.deflection_angle = config.getfloat('deflection_angle', 0.)
        self.deflection_radius = config.getfloat('deflection_radius', 0.)
        self.splitter = MoveSplitter(config, self.deflection_angle,
                                     self.deflection_radius,
                                     move_check_distance_axis)
        self.next_transform = None
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)

    def _handle_connect(self):
        self.next_transform = self.gcode_move.set_move_transform(
            self, force=True)

    def _get_axis_index(self, pos):
        axis_index = self.gcode_move.axis_map.get(self.axis)
        if axis_index is None or axis_index >= len(pos):
            return None
        return axis_index

    def get_position(self):
        pos = list(self.next_transform.get_position())
        if not self.deflection_radius:
            return pos
        axis_index = self._get_axis_index(pos)
        if axis_index is None:
            return pos
        x_adj, y_adj = self.splitter.calc_xy_adjust(pos[axis_index])
        pos[0] -= x_adj
        pos[1] -= y_adj
        return pos

    def move(self, newpos, speed):
        newpos = list(newpos)
        if not self.deflection_radius:
            self.next_transform.move(newpos, speed)
            return
        axis_index = self._get_axis_index(newpos)
        if axis_index is None:
            self.next_transform.move(newpos, speed)
            return
        prev_pos = self.get_position()
        for move_pos in self.splitter.generate_moves(prev_pos, newpos,
                                                     axis_index):
            self.next_transform.move(move_pos, speed)


def load_config(config):
    return ConcentricityToleranceCompensation(config)
