# Helper script to rotate a synchronized extra axis at a Z height
#
# Copyright (C) 2026
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math


class CTCTower:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.normal_transform = None
        self.axis = config.get('axis', 'A').upper()
        if (len(self.axis) != 1 or not self.axis.isupper()
            or self.axis in "XYZEFN"):
            raise config.error(
                "Option 'axis' in section '%s' must be a single extra"
                " G-Code axis letter" % (config.get_name(),))
        self.angle_delta = config.getfloat('angle_delta', 180.)
        self.axis_index = None
        self.axis_offset = 0.
        self.band_index = 0
        self.last_position = [0., 0., 0., 0.]
        self.start = self.height_delta = 0.
        self.gcode_move = self.printer.load_object(config, "gcode_move")
        self.gcode = self.printer.lookup_object("gcode")
        self.gcode.register_command("CTC_TOWER", self.cmd_CTC_TOWER,
                                    desc=self.cmd_CTC_TOWER_help)
        self.gcode.register_command("ABORT_CTC_TOWER",
                                    self.cmd_ABORT_CTC_TOWER,
                                    desc=self.cmd_ABORT_CTC_TOWER_help)

    cmd_CTC_TOWER_help = "Rotate a synchronized extra axis at Z intervals"
    def cmd_CTC_TOWER(self, gcmd):
        if self.normal_transform is not None:
            self.end_test()
        self.axis_index = self.gcode_move.axis_map.get(self.axis)
        if self.axis_index is None:
            raise gcmd.error("CTC_TOWER axis '%s' is not registered"
                             % (self.axis,))
        self.start = gcmd.get_float('START', 0.)
        self.height_delta = gcmd.get_float('LAYER_DELTA', above=0.)
        nt = self.gcode_move.set_move_transform(self, force=True)
        self.normal_transform = nt
        self.axis_offset = 0.
        self.band_index = 0
        self.get_position()
        message_parts = [
            "axis=%s" % (self.axis,),
            "start=%.6f" % (self.start,),
            "layer_delta=%.6f" % (self.height_delta,),
            "angle_delta=%.6f" % (self.angle_delta,),
        ]
        gcmd.respond_info(
            "Starting ctc tower test (" + " ".join(message_parts) + ")")

    cmd_ABORT_CTC_TOWER_help = "Stop the active ctc tower test"
    def cmd_ABORT_CTC_TOWER(self, gcmd):
        if self.normal_transform is None:
            gcmd.respond_info("CTC_TOWER is not active")
            return
        self.end_test()

    def get_position(self):
        pos = list(self.normal_transform.get_position())
        if self.axis_index is not None and self.axis_index < len(pos):
            pos[self.axis_index] -= self.axis_offset
        self.last_position = list(pos)
        return pos

    def _apply_axis_offset(self, pos):
        pos = list(pos)
        if self.axis_index is not None and self.axis_index < len(pos):
            pos[self.axis_index] += self.axis_offset
        return pos

    def _update_axis_offset(self, z):
        if z < self.start:
            return
        band_index = int(math.floor((z - self.start) / self.height_delta
                                    + 1e-9)) + 1
        if band_index > self.band_index:
            self.band_index = band_index
            if band_index & 1:
                self.axis_offset = self.angle_delta
            else:
                self.axis_offset = 0.

    def move(self, newpos, speed):
        normal_transform = self.normal_transform
        axis_index = self.axis_index
        if axis_index is None or axis_index >= len(newpos):
            normal_transform.move(newpos, speed)
            return
        self._update_axis_offset(newpos[2])
        self.last_position[:] = newpos
        normal_transform.move(self._apply_axis_offset(newpos), speed)

    def end_test(self):
        self.gcode.respond_info("Ending ctc tower test mode")
        self.gcode_move.set_move_transform(self.normal_transform, force=True)
        self.normal_transform = None
        self.axis_index = None
        self.axis_offset = 0.
        self.band_index = 0

    def is_active(self):
        return self.normal_transform is not None


def load_config(config):
    return CTCTower(config)
