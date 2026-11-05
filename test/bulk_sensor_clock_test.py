#!/usr/bin/env python3
# Clock sync test for klippy/extras/bulk_sensor.py with slow sensors
#
# Copyright (C) 2026  Toucan3D
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Run with: python3 test/bulk_sensor_clock_test.py
import os, random, sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'klippy'))
from extras import bulk_sensor

MCU_FREQ = 180000000.
UPDATE_INTERVAL = 0.10

class FakeMCU:
    def clock_to_print_time(self, clock):
        return clock / MCU_FREQ

def run_sync(sps, duration=300.):
    # Simulate the status queries of FixedFreqReader for a sensor that
    # reports full blocks of samples, return the timestamp errors
    chip_smooth = sps * UPDATE_INTERVAL * 2
    reader = bulk_sensor.FixedFreqReader(FakeMCU(), chip_smooth, "<i")
    sync = reader.clock_sync
    block = reader.samples_per_block
    period = 1. / sps
    rnd = random.Random(1)
    errors = []
    reported = 0
    t = 0.
    first = True
    while t < duration:
        t += UPDATE_INTERVAL + rnd.uniform(0., 0.005)
        done = int(t / period)
        mcu_clock = t * MCU_FREQ + rnd.uniform(0., 2000.)
        if first:
            sync.reset(mcu_clock, done + 1)
            first = False
            continue
        sync.update(mcu_clock, done + 1)
        full = (done // block) * block
        if full <= reported:
            continue
        base_time, base_chip, inv_freq = sync.get_time_translation()
        for sample in range(reported, full):
            ptime = base_time + (sample - base_chip) * inv_freq
            if t > 10.:
                errors.append(ptime - sample * period)
        sync.set_last_chip_clock(full - 1)
        reported = full
    return errors

def main():
    for sps in (10, 20, 80, 320):
        errors = run_sync(sps)
        mean = sum(errors) / len(errors)
        spread = max(errors) - min(errors)
        print("sps=%d mean=%.4fs spread=%.4fs" % (sps, mean, spread))
        if abs(mean) > 2. / sps or spread > 0.5 / sps + 0.01:
            raise AssertionError("clock sync unstable at %d sps" % (sps,))
    print("All bulk_sensor clock tests passed")

if __name__ == '__main__':
    main()
