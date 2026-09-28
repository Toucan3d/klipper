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

LOOKUP_OPTIONS = ('lookup_a', 'lookup_dx', 'lookup_dy')

class CTCILCLauncher(ctc_launcher.CalibrationLauncher):
    command_prefix = 'CTC_ILC'
    program_stem = 'ctc_ilc_auto'
    default_output_dir = '~/ctc_ilc_sessions'
    label = 'ILC'
    console_tag = 'ILC'
    def __init__(self, config):
        self.result_variant = config.getchoice(
            'result_variant', ['sign_inverted', 'direct'],
            'sign_inverted')
        ctc_launcher.CalibrationLauncher.__init__(self, config)
    def prepare(self, gcmd):
        ctc = self.printer.lookup_object('ctc', None)
        if ctc is not None and ctc.is_active:
            if not gcmd.get_int('DISABLE_LOOKUP', 0):
                raise gcmd.error(
                    "ctc lookup compensation is active. The calibration"
                    " must run without compensation - rerun with"
                    " DISABLE_LOOKUP=1 to disable it for this session")
            ctc.clear_table()
            gcmd.respond_info(
                "ctc lookup compensation disabled for this run")
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
