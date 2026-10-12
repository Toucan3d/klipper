# Launch the external Varoline slot-angle offset calibration
#
# Finds the machine A position at which the slot of the rotating part is
# parallel to gantry X. The calibration needs the concentricity lookup
# measured by VAROLINE_CTC_CALIBRATE: the active [ctc] table is exported in
# direct form for the program, compensation is switched off while the
# program applies the lookup itself, and the table is restored when the
# run ends. The result is reported on the console and kept in the
# session's angle_calibration.json; nothing is written to the config.
#
# Copyright (C) 2026
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os, json, logging
from . import ctc_launcher

class VarolineOffsetLauncher(ctc_launcher.CalibrationLauncher):
    command_prefix = 'VAROLINE_OFFSET'
    program_stem = 'varoline_offset'
    default_output_dir = '~/varoline_offset_sessions'
    label = 'Varoline offset'
    console_tag = 'Offset'
    def __init__(self, config):
        self.default_initial_a = config.getfloat('initial_a', 0.)
        self.initial_a = self.default_initial_a
        self.lookup_file = None
        self.saved_table = None
        self.recommended_a = None
        ctc_launcher.CalibrationLauncher.__init__(self, config)
    def get_status(self, eventtime):
        status = ctc_launcher.CalibrationLauncher.get_status(
            self, eventtime)
        status['recommended_a'] = self.recommended_a
        return status
    def prepare(self, gcmd):
        ctc = self.printer.lookup_object('ctc', None)
        if ctc is None or not ctc.is_active:
            raise gcmd.error(
                "No active [ctc] lookup table. Run VAROLINE_CTC_CALIBRATE"
                " (and SAVE_CONFIG) before the offset calibration")
        self.initial_a = gcmd.get_float('INITIAL_A',
                                        self.default_initial_a)
        table = ctc.get_status(self.reactor.monotonic())
        self.lookup_file = os.path.join(self.output_dir,
                                        'ctc_lookup_direct.cfg')
        try:
            ctc_launcher.write_direct_lookup_file(table, self.lookup_file)
        except OSError as e:
            raise gcmd.error("Unable to write the lookup file %s: %s"
                             % (self.lookup_file, e))
        self.saved_table = (list(table['lookup_a']),
                            list(table['lookup_dx']),
                            list(table['lookup_dy']))
        self.recommended_a = None
    def program_args(self):
        return ['--lookup', self.lookup_file,
                '--initial-a', '%.4f' % (self.initial_a,)]
    def before_start(self):
        # The program applies the lookup itself; [ctc] must not add its
        # own compensation on top. Restored in cleanup().
        ctc = self.printer.lookup_object('ctc', None)
        if ctc is not None:
            ctc.clear_table()
            self._respond("ctc lookup compensation disabled for the"
                          " duration of the calibration")
    def cleanup(self):
        if self.saved_table is None:
            return
        angles, dx, dy = self.saved_table
        self.saved_table = None
        ctc = self.printer.lookup_object('ctc', None)
        if ctc is not None:
            ctc.set_table(angles, dx, dy)
            self._respond("ctc lookup compensation restored")
    def stage_result(self):
        result_path = os.path.join(self.session_dir,
                                   'angle_calibration.json')
        with open(result_path, 'r') as f:
            result = json.load(f)
        logging.info("varoline_offset: result %s", json.dumps(result))
        recommended = result.get(
            'recommended_machine_a_for_slot_parallel_x_deg')
        if not result.get('pass') or recommended is None:
            raise ValueError("the calibration did not pass (%s)"
                             % ("; ".join(result.get('problems', []))
                                or "no result"))
        self.recommended_a = float(recommended)
        self._respond(
            "Varoline offset: the slot is parallel to X at machine"
            " A=%+.4f deg. Result stored in %s"
            % (self.recommended_a, result_path))

def load_config(config):
    return VarolineOffsetLauncher(config)
