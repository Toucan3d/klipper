# Raster scan of the nozzle tip with an upside down eddy current sensor
#
# The sensor (e.g. BTT Eddy) is placed on the print bed facing upwards so
# it measures the nozzle above it.  The EDDY_NOZZLE_SCAN command then moves
# the toolhead in a scanner like raster pattern over the sensor at a
# constant Z height, records the sensor frequency together with the X/Y
# toolhead position of every sample and writes the result to a .csv file
# for offline analysis.
#
# Example config:
#
#   [eddy_nozzle_scan]
#   sensor: probe_eddy_current btt_eddy
#   speed: 10.0          # scan speed in mm/s
#   x_length: 20.0       # scan area size in X (mm)
#   y_length: 20.0       # scan area size in Y (mm)
#   resolution: 0.5      # Y distance between scan lines (mm)
#   #bidirectional: True # scan each line forth and back before stepping Y
#   #output_dir: /tmp    # directory the .csv files are written to
#
# The scan starts at the current toolhead position (position the nozzle
# at the desired Z height over one corner of the scan area first) and
# covers the area towards +X/+Y.  Z is never commanded during the scan.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, os, time

# Wait this long (in wall time) for the trailing sensor samples to arrive
SAMPLE_DRAIN_TIMEOUT = 5.


class EddyNozzleScan:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.sensor_name = config.get('sensor')
        self.speed = config.getfloat('speed', 10., above=0.)
        self.x_length = config.getfloat('x_length', 20., above=0.)
        self.y_length = config.getfloat('y_length', 20., above=0.)
        self.resolution = config.getfloat('resolution', 0.5, above=0.)
        self.bidirectional = config.getboolean('bidirectional', True)
        self.output_dir = config.get('output_dir', '/tmp')
        # Sample collection state
        self._samples = []
        self._recording = False
        self._client_active = False
        self._rec_start = 0.
        self._rec_end = 0.
        self._last_sample_time = 0.
        self._toolhead = None
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('EDDY_NOZZLE_SCAN', self.cmd_EDDY_NOZZLE_SCAN,
                               desc=self.cmd_EDDY_NOZZLE_SCAN_help)

    def _lookup_sensor(self):
        sensor = self.printer.lookup_object(self.sensor_name, None)
        if sensor is None:
            # Convenience: allow "btt_eddy" for "probe_eddy_current btt_eddy"
            sensor = self.printer.lookup_object(
                "probe_eddy_current " + self.sensor_name, None)
        if sensor is None or not hasattr(sensor, 'add_client'):
            raise self.printer.command_error(
                "eddy_nozzle_scan: unknown sensor '%s'" % (self.sensor_name,))
        return sensor

    def _check_homed(self):
        curtime = self.printer.get_reactor().monotonic()
        kin = self._toolhead.get_kinematics()
        homed = kin.get_status(curtime)['homed_axes']
        if any(axis not in homed for axis in "xyz"):
            raise self.printer.command_error(
                "Must home X, Y and Z axes before EDDY_NOZZLE_SCAN")

    # Reconstruct the commanded toolhead position at a given print time
    # from the recent stepper step history
    def _lookup_toolhead_pos(self, pos_time):
        kin = self._toolhead.get_kinematics()
        kin_spos = {s.get_name(): s.mcu_to_commanded_position(
                                      s.get_past_mcu_position(pos_time))
                    for s in kin.get_steppers()}
        return kin.calc_position(kin_spos)

    # Sensor batch callback - runs from the reactor while the scan moves
    # are executing.  Positions must be looked up here (not after the
    # scan) because the stepper step history only covers ~30 seconds.
    def _handle_batch(self, msg):
        if not self._client_active:
            return False
        data = msg['data']
        if not data:
            return True
        self._last_sample_time = data[-1][0]
        if not self._recording:
            return True
        try:
            rec_start = self._rec_start
            rec_end = self._rec_end
            for sample_time, freq, z in data:
                if sample_time < rec_start or sample_time > rec_end:
                    continue
                pos = self._lookup_toolhead_pos(sample_time)
                self._samples.append((sample_time, pos[0], pos[1], freq, z))
        except Exception:
            logging.exception("eddy_nozzle_scan: error processing samples")
        return True

    def _run_raster(self, start_pos, speed, x_length, y_length,
                    resolution, bidirectional):
        move = self._toolhead.manual_move
        x0, y0 = start_pos[0], start_pos[1]
        x1 = x0 + x_length
        num_lines = int(y_length / resolution + .000001) + 1
        for i in range(num_lines):
            # Step to the next scan line (no-op on the first line)
            move([None, y0 + i * resolution, None], speed)
            if bidirectional:
                # Measure the line in both directions, then step Y
                move([x1, None, None], speed)
                move([x0, None, None], speed)
            elif i % 2 == 0:
                move([x1, None, None], speed)
            else:
                move([x0, None, None], speed)
        return num_lines

    def _wait_for_samples(self):
        # Wait until the sensor stream has caught up with the end of the
        # scan so the trailing samples are captured as well
        reactor = self.printer.get_reactor()
        deadline = reactor.monotonic() + SAMPLE_DRAIN_TIMEOUT
        while self._last_sample_time < self._rec_end:
            systime = reactor.monotonic()
            if systime > deadline:
                logging.warning("eddy_nozzle_scan: timeout waiting for"
                                " trailing sensor samples")
                break
            reactor.pause(systime + 0.100)

    def _write_csv(self, filename, params):
        if not os.path.isabs(filename):
            filename = os.path.join(self.output_dir, filename)
        try:
            dirname = os.path.dirname(filename)
            if dirname and not os.path.isdir(dirname):
                os.makedirs(dirname)
            with open(filename, "w") as f:
                f.write("# eddy_nozzle_scan sensor='%s'\n"
                        % (self.sensor_name,))
                f.write("# %s\n" % (" ".join(
                    ["%s=%s" % (k, v) for k, v in params])))
                f.write("print_time,x,y,frequency,z_calibrated\n")
                for sample_time, x, y, freq, z in self._samples:
                    f.write("%.6f,%.5f,%.5f,%.3f,%.6f\n"
                            % (sample_time, x, y, freq, z))
        except (IOError, OSError) as e:
            raise self.printer.command_error(
                "eddy_nozzle_scan: error writing '%s': %s" % (filename, e))
        return filename

    cmd_EDDY_NOZZLE_SCAN_help = (
        "Raster scan the nozzle over an upside down eddy current sensor"
        " and write samples with X/Y positions to a .csv file")
    def cmd_EDDY_NOZZLE_SCAN(self, gcmd):
        speed = gcmd.get_float('SPEED', self.speed, above=0.)
        x_length = gcmd.get_float('X_LENGTH', self.x_length, above=0.)
        y_length = gcmd.get_float('Y_LENGTH', self.y_length, above=0.)
        resolution = gcmd.get_float('RESOLUTION', self.resolution, above=0.)
        bidirectional = bool(gcmd.get_int(
            'BIDIRECTIONAL', int(self.bidirectional), minval=0, maxval=1))
        filename = gcmd.get('FILENAME', time.strftime(
            "eddy_nozzle_scan_%Y%m%d_%H%M%S.csv"))
        sensor = self._lookup_sensor()
        self._toolhead = self.printer.lookup_object('toolhead')
        self._check_homed()
        self._toolhead.wait_moves()
        start_pos = self._toolhead.get_position()
        # Start sensor data collection
        self._samples = []
        self._recording = False
        self._last_sample_time = 0.
        self._client_active = True
        sensor.add_client(self._handle_batch)
        try:
            # Give the sensor time to start streaming
            self._toolhead.dwell(0.500)
            self._rec_start = self._toolhead.get_last_move_time()
            self._rec_end = 9999999999.
            self._recording = True
            num_lines = self._run_raster(start_pos, speed, x_length,
                                         y_length, resolution, bidirectional)
            self._toolhead.wait_moves()
            self._rec_end = self._toolhead.get_last_move_time()
            self._wait_for_samples()
        finally:
            self._recording = False
            self._client_active = False
        # Return to the scan start position
        self._toolhead.manual_move(
            [start_pos[0], start_pos[1], None], speed)
        if not self._samples:
            raise self.printer.command_error(
                "eddy_nozzle_scan: no sensor samples received")
        params = [('speed', "%.3f" % (speed,)),
                  ('x_length', "%.3f" % (x_length,)),
                  ('y_length', "%.3f" % (y_length,)),
                  ('resolution', "%.3f" % (resolution,)),
                  ('bidirectional', int(bidirectional)),
                  ('start_x', "%.5f" % (start_pos[0],)),
                  ('start_y', "%.5f" % (start_pos[1],)),
                  ('z_height', "%.5f" % (start_pos[2],))]
        outname = self._write_csv(filename, params)
        gcmd.respond_info(
            "eddy_nozzle_scan: %d lines, %d samples, z=%.3f\n"
            "Results written to %s"
            % (num_lines, len(self._samples), start_pos[2], outname))


def load_config(config):
    return EddyNozzleScan(config)
