# Raster scan of the nozzle tip with an upside down eddy current sensor
#
# The sensor (e.g. BTT Eddy) is placed on the print bed facing upwards so
# it measures the nozzle above it.  The EDDY_NOZZLE_SCAN command then moves
# the toolhead in a scanner like raster pattern over the sensor at a
# constant Z height, records the sensor frequency together with the X/Y
# toolhead position of every sample and writes the result to a .csv file
# for offline analysis.
#
# EDDY_NOZZLE_SCAN_AUTO combines a fast coarse scan of the full area with
# a slow high resolution scan of a small window centered on the detected
# signal peak, drastically reducing the total scan time.
#
# Example config (standalone, no [probe_eddy_current] section needed -
# use this when the printer already has another probe, as Klipper only
# supports a single probe):
#
#   [eddy_nozzle_scan]
#   i2c_mcu: eddy          # mcu of the LDC1612 (BTT Eddy)
#   i2c_bus: i2c0f
#   speed: 10.0            # scan speed in mm/s
#   x_length: 20.0         # scan area size in X (mm)
#   y_length: 20.0         # scan area size in Y (mm)
#   resolution: 0.5        # Y distance between scan lines (mm)
#   #bidirectional: True   # scan each line forth and back before stepping Y
#   #output_dir: /tmp      # directory the .csv files are written to
#   #coarse_speed: 40.0    # AUTO: speed of the coarse locating scan
#   #coarse_resolution: 1.0 # AUTO: Y line distance of the coarse scan
#   #fine_speed: 3.0       # AUTO: speed of the fine scan
#   #fine_size: 4.0        # AUTO: size of the fine scan window (mm)
#   #fine_resolution: 0.1  # AUTO: Y line distance of the fine scan
#   #min_signal: 50.0      # AUTO: minimum peak height over baseline (Hz)
#
# Alternatively, if the eddy sensor is already configured as the probe
# of this printer, reference that section instead of the i2c options:
#
#   [eddy_nozzle_scan]
#   sensor: probe_eddy_current btt_eddy
#
# The scan starts at the current toolhead position (position the nozzle
# at the desired Z height over one corner of the scan area first) and
# covers the area towards +X/+Y.  Z is never commanded during the scan.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, os, time
from . import ldc1612

# Wait this long (in wall time) for the trailing sensor samples to arrive
SAMPLE_DRAIN_TIMEOUT = 5.


class EddyNozzleScan:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.sensor_name = config.get('sensor', None)
        self.sensor_helper = None
        if self.sensor_name is None:
            # No probe section referenced - drive the LDC1612 directly
            # from this section's i2c_* options.  This avoids conflicts
            # with an existing probe ([probe_eddy_current] registers as
            # the printer's probe and Klipper only supports one).
            self.sensor_helper = ldc1612.LDC1612(config)
            self.sensor_name = "ldc1612 " + self.name
        self.speed = config.getfloat('speed', 10., above=0.)
        self.x_length = config.getfloat('x_length', 20., above=0.)
        self.y_length = config.getfloat('y_length', 20., above=0.)
        self.resolution = config.getfloat('resolution', 0.5, above=0.)
        self.bidirectional = config.getboolean('bidirectional', True)
        self.output_dir = config.get('output_dir', '/tmp')
        # Coarse/fine two phase scan (EDDY_NOZZLE_SCAN_AUTO)
        self.coarse_speed = config.getfloat('coarse_speed', 40., above=0.)
        self.coarse_resolution = config.getfloat('coarse_resolution', 1.,
                                                 above=0.)
        self.fine_speed = config.getfloat('fine_speed', 3., above=0.)
        self.fine_size = config.getfloat('fine_size', 4., above=0.)
        self.fine_resolution = config.getfloat('fine_resolution', 0.1,
                                               above=0.)
        self.min_signal = config.getfloat('min_signal', 50., above=0.)
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
        gcode.register_command('EDDY_NOZZLE_SCAN_AUTO',
                               self.cmd_EDDY_NOZZLE_SCAN_AUTO,
                               desc=self.cmd_EDDY_NOZZLE_SCAN_AUTO_help)

    def _lookup_sensor(self):
        if self.sensor_helper is not None:
            return self.sensor_helper
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

    # Scan one rectangular region starting at the current toolhead
    # position and return the collected samples
    def _scan_region(self, sensor, speed, x_length, y_length,
                     resolution, bidirectional):
        self._toolhead.wait_moves()
        start_pos = self._toolhead.get_position()
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
        samples = self._samples
        self._samples = []
        if not samples:
            raise self.printer.command_error(
                "eddy_nozzle_scan: no sensor samples received")
        return samples, num_lines, start_pos

    # Locate the nozzle: weighted centroid of all samples well above the
    # baseline frequency (metal close to the coil raises the frequency)
    def _find_peak(self, samples, min_signal):
        freqs = sorted([s[3] for s in samples])
        baseline = freqs[len(freqs) // 2]
        peak = freqs[-1]
        signal = peak - baseline
        if signal < min_signal:
            raise self.printer.command_error(
                "eddy_nozzle_scan: no clear signal peak found (baseline"
                " %.1f Hz, peak %.1f Hz, height %.1f Hz < MIN_SIGNAL %.1f"
                " Hz). Is the nozzle within the scan area?"
                % (baseline, peak, signal, min_signal))
        threshold = baseline + .5 * signal
        wsum = xsum = ysum = 0.
        for sample_time, x, y, freq, z in samples:
            weight = freq - threshold
            if weight <= 0.:
                continue
            wsum += weight
            xsum += weight * x
            ysum += weight * y
        return xsum / wsum, ysum / wsum, baseline, peak

    def _write_csv(self, filename, samples, params):
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
                for sample_time, x, y, freq, z in samples:
                    f.write("%.6f,%.5f,%.5f,%.3f,%.6f\n"
                            % (sample_time, x, y, freq, z))
        except (IOError, OSError) as e:
            raise self.printer.command_error(
                "eddy_nozzle_scan: error writing '%s': %s" % (filename, e))
        return filename

    def _scan_params(self, speed, x_length, y_length, resolution,
                     bidirectional, start_pos):
        return [('speed', "%.3f" % (speed,)),
                ('x_length', "%.3f" % (x_length,)),
                ('y_length', "%.3f" % (y_length,)),
                ('resolution', "%.3f" % (resolution,)),
                ('bidirectional', int(bidirectional)),
                ('start_x', "%.5f" % (start_pos[0],)),
                ('start_y', "%.5f" % (start_pos[1],)),
                ('z_height', "%.5f" % (start_pos[2],))]

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
        samples, num_lines, start_pos = self._scan_region(
            sensor, speed, x_length, y_length, resolution, bidirectional)
        # Return to the scan start position
        self._toolhead.manual_move(
            [start_pos[0], start_pos[1], None], speed)
        params = self._scan_params(speed, x_length, y_length, resolution,
                                   bidirectional, start_pos)
        outname = self._write_csv(filename, samples, params)
        gcmd.respond_info(
            "eddy_nozzle_scan: %d lines, %d samples, z=%.3f\n"
            "Results written to %s"
            % (num_lines, len(samples), start_pos[2], outname))

    cmd_EDDY_NOZZLE_SCAN_AUTO_help = (
        "Locate the nozzle with a fast coarse raster scan, then raster a"
        " small high resolution window centered on the signal peak")
    def cmd_EDDY_NOZZLE_SCAN_AUTO(self, gcmd):
        x_length = gcmd.get_float('X_LENGTH', self.x_length, above=0.)
        y_length = gcmd.get_float('Y_LENGTH', self.y_length, above=0.)
        coarse_speed = gcmd.get_float('COARSE_SPEED', self.coarse_speed,
                                      above=0.)
        coarse_res = gcmd.get_float('COARSE_RESOLUTION',
                                    self.coarse_resolution, above=0.)
        fine_speed = gcmd.get_float('FINE_SPEED', self.fine_speed, above=0.)
        fine_size = gcmd.get_float('FINE_SIZE', self.fine_size, above=0.)
        fine_res = gcmd.get_float('FINE_RESOLUTION', self.fine_resolution,
                                  above=0.)
        bidirectional = bool(gcmd.get_int('BIDIRECTIONAL', 0,
                                          minval=0, maxval=1))
        min_signal = gcmd.get_float('MIN_SIGNAL', self.min_signal, above=0.)
        basename = gcmd.get('FILENAME', time.strftime(
            "eddy_nozzle_scan_%Y%m%d_%H%M%S"))
        if basename.endswith('.csv'):
            basename = basename[:-4]
        sensor = self._lookup_sensor()
        self._toolhead = self.printer.lookup_object('toolhead')
        self._check_homed()
        # Phase 1: fast coarse scan of the full area to locate the nozzle
        gcmd.respond_info("eddy_nozzle_scan: coarse scan %.1fx%.1fmm ..."
                          % (x_length, y_length))
        coarse_samples, coarse_lines, start_pos = self._scan_region(
            sensor, coarse_speed, x_length, y_length, coarse_res, False)
        peak_x, peak_y, baseline, peak = self._find_peak(coarse_samples,
                                                         min_signal)
        gcmd.respond_info(
            "eddy_nozzle_scan: peak at X=%.3f Y=%.3f (baseline %.1f Hz,"
            " peak %.1f Hz, height %.1f Hz)"
            % (peak_x, peak_y, baseline, peak, peak - baseline))
        # Phase 2: slow fine scan of a small window centered on the peak,
        # clamped so it stays inside the (known reachable) coarse area
        fine_x = min(fine_size, x_length)
        fine_y = min(fine_size, y_length)
        fx0 = min(max(peak_x - fine_x * .5, start_pos[0]),
                  start_pos[0] + x_length - fine_x)
        fy0 = min(max(peak_y - fine_y * .5, start_pos[1]),
                  start_pos[1] + y_length - fine_y)
        self._toolhead.manual_move([fx0, fy0, None], coarse_speed)
        fine_samples, fine_lines, fine_start = self._scan_region(
            sensor, fine_speed, fine_x, fine_y, fine_res, bidirectional)
        # Park the nozzle centered over the detected peak
        self._toolhead.manual_move([peak_x, peak_y, None], coarse_speed)
        # Write results
        peak_params = [('peak_x', "%.5f" % (peak_x,)),
                       ('peak_y', "%.5f" % (peak_y,)),
                       ('baseline_hz', "%.3f" % (baseline,)),
                       ('peak_hz', "%.3f" % (peak,))]
        coarse_params = ([('phase', 'coarse')]
                         + self._scan_params(coarse_speed, x_length, y_length,
                                             coarse_res, False, start_pos)
                         + peak_params)
        fine_params = ([('phase', 'fine')]
                       + self._scan_params(fine_speed, fine_x, fine_y,
                                           fine_res, bidirectional,
                                           fine_start)
                       + peak_params)
        coarse_name = self._write_csv(basename + "_coarse.csv",
                                      coarse_samples, coarse_params)
        fine_name = self._write_csv(basename + "_fine.csv",
                                    fine_samples, fine_params)
        gcmd.respond_info(
            "eddy_nozzle_scan: coarse %d lines / %d samples, fine %d lines"
            " / %d samples, z=%.3f\n"
            "Nozzle parked over peak at X=%.3f Y=%.3f\n"
            "Results written to %s and %s"
            % (coarse_lines, len(coarse_samples), fine_lines,
               len(fine_samples), start_pos[2], peak_x, peak_y,
               coarse_name, fine_name))


def load_config(config):
    return EddyNozzleScan(config)
