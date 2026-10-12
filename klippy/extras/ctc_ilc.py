# Launch the external CTC ILC concentricity calibration
#
# Process supervision and the operator prompts live in ctc_launcher;
# this module adds the ILC-specific checks and stages the resulting
# lookup table into the [ctc] config section.
#
# Copyright (C) 2026
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os, json, logging
from . import ctc_launcher

LOOKUP_OPTIONS = ctc_launcher.LOOKUP_OPTIONS

class CTCILCLauncher(ctc_launcher.CalibrationLauncher):
    command_prefix = 'VAROLINE_CTC'
    program_stem = 'ctc_ilc_auto'
    default_output_dir = '~/ctc_ilc_sessions'
    label = 'CTC'
    console_tag = 'CTC'
    def __init__(self, config):
        self.result_variant = config.getchoice(
            'result_variant', ['sign_inverted', 'direct'],
            'sign_inverted')
        self.initial_lookup_file = None
        self.saved_table = None
        ctc_launcher.CalibrationLauncher.__init__(self, config)
    def prepare(self, gcmd):
        self.initial_lookup_file = None
        self.saved_table = None
        ctc = self.printer.lookup_object('ctc', None)
        if ctc is None or not ctc.is_active:
            return
        update = gcmd.get_int('UPDATE_LOOKUP', 0)
        disable = gcmd.get_int('DISABLE_LOOKUP', 0)
        if update and disable:
            raise gcmd.error("UPDATE_LOOKUP and DISABLE_LOOKUP exclude"
                             " each other")
        if not (update or disable):
            raise gcmd.error(
                "ctc lookup compensation is active. Rerun with"
                " UPDATE_LOOKUP=1 to refine the active table (small"
                " concentricity changes since the last calibration) or"
                " with DISABLE_LOOKUP=1 to measure a new table from"
                " scratch")
        table = ctc.get_status(self.reactor.monotonic())
        self.saved_table = (list(table['lookup_a']),
                            list(table['lookup_dx']),
                            list(table['lookup_dy']))
        if not update:
            gcmd.respond_info(
                "The active ctc lookup table is ignored for this run;"
                " the new table is measured from scratch")
            return
        path = os.path.join(self.output_dir, 'ctc_lookup_direct.cfg')
        try:
            ctc_launcher.write_direct_lookup_file(table, path)
        except OSError as e:
            raise gcmd.error("Unable to write the lookup file %s: %s"
                             % (path, e))
        self.initial_lookup_file = path
        gcmd.respond_info(
            "The active ctc lookup table (%d points) is the starting"
            " point of this run" % (len(table['lookup_a']),))
    def program_args(self):
        if self.initial_lookup_file is None:
            return []
        return ['--initial-lookup', self.initial_lookup_file]
    def before_start(self):
        # The program applies the lookup itself; [ctc] must not add its
        # own compensation on top. Cleared here rather than in prepare()
        # so the head is positioned with compensation active; a refining
        # run subtracts the table entry at the captured angle. The
        # previous table is restored by cleanup() unless a new one was
        # staged.
        ctc = self.printer.lookup_object('ctc', None)
        if ctc is not None and ctc.is_active:
            ctc.clear_table()
            self._respond("ctc lookup compensation disabled for the"
                          " duration of the calibration")
    def cleanup(self):
        saved = self.saved_table
        self.saved_table = None
        self.initial_lookup_file = None
        if saved is None or self.state == 'complete':
            return
        ctc = self.printer.lookup_object('ctc', None)
        if ctc is not None:
            ctc.set_table(*saved)
            self._respond("previous ctc lookup table restored")
    def _parse_lookup_file(self, path):
        options = {}
        with open(path, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or ':' not in line:
                    continue
                name, value = line.split(':', 1)
                options[name.strip()] = value.strip()
        missing = [n for n in LOOKUP_OPTIONS if n not in options]
        if missing:
            raise ValueError("%s is missing options: %s"
                             % (path, ", ".join(missing)))
        parsed = {n: [float(p) for p in options[n].split(',')]
                  for n in LOOKUP_OPTIONS}
        counts = {len(v) for v in parsed.values()}
        if len(counts) != 1 or not min(counts):
            raise ValueError("%s has inconsistent or empty lookup"
                             " columns" % (path,))
        return options, parsed
    def stage_result(self):
        summary_path = os.path.join(self.session_dir,
                                    'session_summary.json')
        try:
            with open(summary_path, 'r') as f:
                summary = json.load(f)
        except (IOError, OSError, ValueError):
            logging.exception("ctc_ilc: unable to read %s", summary_path)
            summary = {}
        logging.info("ctc_ilc: session summary %s", json.dumps(summary))
        lookup_path = os.path.join(
            self.session_dir,
            'final_lookup_%s.cfg' % (self.result_variant,))
        raw_options, parsed = self._parse_lookup_file(lookup_path)
        ctc = self.printer.lookup_object('ctc', None)
        if ctc is not None:
            ctc.set_table(parsed['lookup_a'], parsed['lookup_dx'],
                          parsed['lookup_dy'])
            self._respond("New lookup table (%d points) applied to"
                          " [ctc]" % (len(parsed['lookup_a']),))
        else:
            self._respond("Warning: no [ctc] section is configured;"
                          " the lookup takes effect after SAVE_CONFIG"
                          " and restart")
        configfile = self.printer.lookup_object('configfile')
        for name in LOOKUP_OPTIONS:
            configfile.set('ctc', name, raw_options[name])
        self._respond(
            "The SAVE_CONFIG command will update the printer config"
            " file with the new lookup table and restart the printer."
            " If SAVE_CONFIG reports a conflict, remove the lookup_a/"
            "lookup_dx/lookup_dy options from the [ctc] section of"
            " printer.cfg first.")

def load_config(config):
    return CTCILCLauncher(config)
