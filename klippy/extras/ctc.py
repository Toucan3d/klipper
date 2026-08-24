# XY compensation for concentricity error on a synchronized extra axis
#
# Copyright (C) 2026
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import bisect


# PEP 485 isclose()
def isclose(a, b, rel_tol=1e-09, abs_tol=0.0):
    return abs(a - b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)


def lerp(t, v0, v1):
    return (1. - t) * v0 + t * v1


class LookupTable:
    # Lookup table of the measured runout (dx, dy) of the rotating part at
    # given A-axis angles. The support points come from 'lookup_a' and may be
    # placed at arbitrary, non-uniform angles. All angles are reduced modulo
    # 360, so the table describes a single revolution and the same
    # compensation repeats every turn (720 == 360 == 0, 450 == 90, ...). The
    # returned XY adjustment is the opposite of the runout, -(dx, dy).
    # With interpolate=True the value is linearly interpolated between the two
    # neighboring support points, wrapping across the 360 -> 0 boundary. With
    # interpolate=False the angle is snapped to the nearest support point and
    # exactly its value is used (no interpolation or smoothing across angles).
    def __init__(self, angles, dx, dy, interpolate=True):
        # Sort the support points by angle so neighbor lookup is well defined.
        order = sorted(range(len(angles)), key=lambda i: angles[i])
        self.angles = tuple(angles[i] for i in order)
        self.dx = tuple(dx[i] for i in order)
        self.dy = tuple(dy[i] for i in order)
        self.n = len(self.angles)
        self.interpolate = interpolate
        # Smallest gap between neighboring support points (wrapping across
        # 360 -> 0), used as the default move check distance so that no
        # support point is skipped while the axis sweeps.
        if self.n >= 2:
            gaps = [self.angles[i + 1] - self.angles[i]
                    for i in range(self.n - 1)]
            gaps.append(self.angles[0] + 360. - self.angles[-1])
            self.min_step = min(gaps)
        else:
            self.min_step = 360. if self.n else 0.

    def has_compensation(self):
        return any(v != 0. for v in self.dx) or any(v != 0. for v in self.dy)

    def calc_xy_adjust(self, axis_pos):
        if not self.n:
            return (0., 0.)
        # Reduce to one revolution: every full turn repeats the same runout.
        q = axis_pos % 360.
        if not self.interpolate:
            idx = self._nearest_index(q)
            return (-self.dx[idx], -self.dy[idx])
        i0, i1, frac = self._segment(q)
        return (-lerp(frac, self.dx[i0], self.dx[i1]),
                -lerp(frac, self.dy[i0], self.dy[i1]))

    def _segment(self, q):
        # Return (i0, i1, frac) for linear interpolation between the two
        # support points that bracket q, wrapping across 360 -> 0.
        n = self.n
        angles = self.angles
        i = bisect.bisect_right(angles, q)
        if i == 0 or i == n:
            # q is in the wrap segment between the last and the first point.
            lower = angles[n - 1]
            upper = angles[0] + 360.
            qq = q if q >= lower else q + 360.
            span = upper - lower
            frac = (qq - lower) / span if span else 0.
            return (n - 1, 0, frac)
        i0 = i - 1
        span = angles[i] - angles[i0]
        frac = (q - angles[i0]) / span if span else 0.
        return (i0, i, frac)

    def _nearest_index(self, q):
        # Index of the angularly closest support point (circular distance).
        # Exact ties resolve to the higher angle.
        best_i, best_d = 0, None
        for i, a in enumerate(self.angles):
            d = abs(q - a)
            d = min(d, 360. - d)
            if best_d is None or d <= best_d:
                best_d, best_i = d, i
        return best_i


class MoveSplitter:
    def __init__(self, config, table, move_check_distance_axis):
        self.table = table
        self.split_delta_xy = config.getfloat('split_delta_xy', .025,
                                              minval=0.01)
        self.move_check_distance_axis = move_check_distance_axis

    def calc_xy_adjust(self, axis_pos):
        return self.table.calc_xy_adjust(axis_pos)

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


class CTC:
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
        self.table = self._load_table(config)
        # Remember whether the check distance was left to the default so a
        # runtime table change (different point count -> different step) can
        # keep tracking one grid cell automatically.
        self.auto_move_check_distance = move_check_distance_axis is None
        if move_check_distance_axis is None:
            # Default to the smallest gap between support points so every
            # support point along a sweeping move is sampled; fall back to
            # 5 units when no table is set.
            move_check_distance_axis = self.table.min_step or 5.
        self.is_active = self.table.has_compensation()
        self.splitter = MoveSplitter(config, self.table,
                                     move_check_distance_axis)
        self.next_transform = None
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('SET_CTC', self.cmd_SET_CTC,
                                    desc=self.cmd_SET_CTC_help)
        self.gcode.register_command('QUERY_CTC', self.cmd_QUERY_CTC,
                                    desc=self.cmd_QUERY_CTC_help)

    def _load_table(self, config):
        dx = list(config.getfloatlist('lookup_dx', ()))
        dy = list(config.getfloatlist('lookup_dy', ()))
        if len(dx) != len(dy):
            raise config.error(
                "Options 'lookup_dx' and 'lookup_dy' in section '%s' must"
                " have the same number of entries" % (config.get_name(),))
        cfg_angles = config.getfloatlist('lookup_a', None)
        if cfg_angles is not None:
            if len(cfg_angles) != len(dx):
                raise config.error(
                    "Option 'lookup_a' in section '%s' must have the same"
                    " number of entries as 'lookup_dx'/'lookup_dy'"
                    % (config.get_name(),))
            # The angle column defines where each support point sits; the
            # angles may be non-uniform but must be distinct (modulo 360).
            angles = [a % 360. for a in cfg_angles]
            self._reject_duplicate_angles(angles, config.error)
        elif dx:
            # No explicit angles: fall back to a uniform 0..360 grid so a
            # plain dx/dy list keeps working.
            step = 360. / len(dx)
            angles = [i * step for i in range(len(dx))]
        else:
            angles = []
        interpolate = config.getboolean('interpolate', True)
        return LookupTable(angles, dx, dy, interpolate)

    def _reject_duplicate_angles(self, angles, error):
        ordered = sorted(angles)
        for i in range(len(ordered) - 1):
            if isclose(ordered[i], ordered[i + 1], abs_tol=1e-6):
                raise error("ctc 'lookup_a' has duplicate angles (mod 360):"
                            " each support point needs a distinct A position")

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
        if not self.is_active:
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
        if not self.is_active:
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

    def get_status(self, eventtime):
        return {
            'axis': self.axis,
            'active': self.is_active,
            'interpolate': self.table.interpolate,
            'points': self.table.n,
            'min_step': self.table.min_step,
            'split_delta_xy': self.splitter.split_delta_xy,
            'move_check_distance_axis': self.splitter.move_check_distance_axis,
            'lookup_a': list(self.table.angles),
            'lookup_dx': list(self.table.dx),
            'lookup_dy': list(self.table.dy),
        }

    def set_table(self, angles, dx, dy):
        # Public runtime table update (used by the ctc_ilc calibration
        # launcher). Angles are reduced modulo 360; the interpolation
        # mode is kept.
        if not (len(angles) == len(dx) == len(dy)):
            raise self.printer.command_error(
                "ctc table update requires equally long angle/dx/dy"
                " lists (a=%d dx=%d dy=%d)"
                % (len(angles), len(dx), len(dy)))
        angles = [a % 360. for a in angles]
        self._reject_duplicate_angles(angles, self.printer.command_error)
        self._set_table(angles, list(dx), list(dy),
                        self.table.interpolate)
        # The transform output changed for a fixed toolhead position;
        # resync gcode_move so subsequent moves stay continuous.
        self.gcode_move.reset_last_position()

    def clear_table(self):
        # Disable compensation until a new table is set (or restored
        # from the config on restart).
        self.set_table([], [], [])

    def _set_table(self, angles, dx, dy, interpolate):
        table = LookupTable(angles, dx, dy, interpolate)
        self.table = table
        self.splitter.table = table
        self.is_active = table.has_compensation()
        if self.auto_move_check_distance:
            self.splitter.move_check_distance_axis = table.min_step or 5.

    def _describe(self):
        fmt = lambda vals: ", ".join("%.4f" % (v,) for v in vals)
        return ("ctc: axis=%s active=%d interpolate=%s points=%d"
                " min_step=%.4f deg\n"
                "split_delta_xy=%.4f mm  move_check_distance_axis=%.4f%s\n"
                "lookup_a:  %s\n"
                "lookup_dx: %s\n"
                "lookup_dy: %s"
                % (self.axis, self.is_active, self.table.interpolate,
                   self.table.n, self.table.min_step,
                   self.splitter.split_delta_xy,
                   self.splitter.move_check_distance_axis,
                   " (auto)" if self.auto_move_check_distance else "",
                   fmt(self.table.angles), fmt(self.table.dx),
                   fmt(self.table.dy)))

    def _parse_list(self, gcmd, name):
        raw = gcmd.get(name, None)
        if raw is None:
            return None
        # G-Code splits arguments on whitespace, so a list must be passed as a
        # single comma-separated token (no spaces). An empty value clears the
        # table (disables compensation).
        raw = raw.strip().strip('"').strip("'").strip()
        if not raw:
            return []
        try:
            return [float(p) for p in raw.replace(',', ' ').split()]
        except ValueError:
            raise gcmd.error(
                "SET_CTC: %s must be a comma-separated list of numbers"
                " with no spaces (e.g. %s=0,90,180,270)" % (name, name))

    cmd_SET_CTC_help = ("Update the ctc lookup table / parameters at runtime"
                        " (not persisted to the config file)")
    def cmd_SET_CTC(self, gcmd):
        angles = self._parse_list(gcmd, 'LOOKUP_A')
        dx = self._parse_list(gcmd, 'LOOKUP_DX')
        dy = self._parse_list(gcmd, 'LOOKUP_DY')
        interp = gcmd.get_int('INTERPOLATE', None, minval=0, maxval=1)
        index = gcmd.get_int('INDEX', None, minval=0)
        point_a = gcmd.get_float('A', None)
        point_dx = gcmd.get_float('DX', None)
        point_dy = gcmd.get_float('DY', None)
        split_delta = gcmd.get_float('SPLIT_DELTA_XY', None, minval=0.01)
        move_check = gcmd.get_float('MOVE_CHECK_DISTANCE_AXIS', None,
                                    minval=0.01)
        # Start from the current table and apply only what was requested.
        new_angles = list(self.table.angles)
        new_dx = list(self.table.dx)
        new_dy = list(self.table.dy)
        new_interp = self.table.interpolate
        if interp is not None:
            new_interp = bool(interp)
        table_changed = new_interp != self.table.interpolate
        if angles is not None:
            new_angles = angles
            table_changed = True
        if dx is not None:
            new_dx = dx
            table_changed = True
        if dy is not None:
            new_dy = dy
            table_changed = True
        if (index is not None or point_a is not None
                or point_dx is not None or point_dy is not None):
            if index is None:
                raise gcmd.error("SET_CTC: A/DX/DY require INDEX")
            if index >= len(new_dx):
                raise gcmd.error(
                    "SET_CTC: INDEX=%d out of range (table has %d points)"
                    % (index, len(new_dx)))
            if point_a is not None:
                new_angles[index] = point_a
            if point_dx is not None:
                new_dx[index] = point_dx
            if point_dy is not None:
                new_dy[index] = point_dy
            table_changed = True
        if table_changed:
            # Clearing dx and dy together disables compensation; drop the
            # now-meaningless angle column as well.
            if not new_dx and not new_dy:
                new_angles = []
            if not (len(new_angles) == len(new_dx) == len(new_dy)):
                raise gcmd.error(
                    "SET_CTC: lookup_a/lookup_dx/lookup_dy must have the same"
                    " number of entries (a=%d dx=%d dy=%d). Pass LOOKUP_A too"
                    " when changing the point count."
                    % (len(new_angles), len(new_dx), len(new_dy)))
            new_angles = [a % 360. for a in new_angles]
            self._reject_duplicate_angles(new_angles, gcmd.error)
            self._set_table(new_angles, new_dx, new_dy, new_interp)
        if split_delta is not None:
            self.splitter.split_delta_xy = split_delta
        if move_check is not None:
            self.splitter.move_check_distance_axis = move_check
            self.auto_move_check_distance = False
        if table_changed:
            # The transform output changed for a fixed toolhead position;
            # resync gcode_move so subsequent moves stay continuous.
            self.gcode_move.reset_last_position()
        gcmd.respond_info(self._describe())

    cmd_QUERY_CTC_help = "Report the current ctc lookup table and parameters"
    def cmd_QUERY_CTC(self, gcmd):
        gcmd.respond_info(self._describe())


def load_config(config):
    return CTC(config)
