#!/usr/bin/env python

from __future__ import print_function

import argparse
import dronekit
import sys
import time
import logging

LOG = logging.getLogger(__name__)


class SafetyNet(object):
    def __init__(self, vehicle, safe_alt,
                 safe_mode='BRAKE'):
        self.vehicle = vehicle
        self.safe_alt = safe_alt
        self.safe_mode = safe_mode
        self.active = False
        self.enabled = False
        self.last_alt = 0
        self.last_alt_time = 0

        self.init_callbacks()

        if self.vehicle.armed:
            self.enable_safety_net()

        if self.vehicle.location.global_relative_frame.alt < self.safe_alt:
            self.active = True

    def init_callbacks(self):
        self.vehicle.add_attribute_listener('armed', self.armed_callback)

    def enable_safety_net(self):
        self.vehicle.location.add_attribute_listener(
            'global_relative_frame',
            self.location_callback)
        self.vehicle.add_attribute_listener(
            'mode',
            self.mode_callback)
        self.enabled = True

    def disable_safety_net(self):
        self.vehicle.location.remove_attribute_listener(
            'global_relative_frame',
            self.location_callback)
        self.vehicle.remove_attribute_listener(
            'mode',
            self.mode_callback)
        self.enabled = False

    def armed_callback(self, parent, aname, aval):
        if aval:
            LOG.info('vehicle is armed')
            self.enable_safety_net()
        else:
            LOG.info('vehicle is disarmed')
            self.disable_safety_net()

    def mode_callback(self, parent, aname, aval):
        if aval.name in ['LAND', 'RTL']:
            LOG.warning('disabling safety net for mode %s', aval.name)
            self.enabled = False
        else:
            LOG.warning('enabling safety net for mode %s', aval.name)
            self.enabled = True

    def location_callback(self, parent, aname, aval):
        if not self.enabled:
            return

        now = time.time()

        if self.last_alt != 0:
            c_pos = aval.alt - self.last_alt
            c_time = now - self.last_alt_time
            v = c_pos/c_time
            LOG.info('v: %f', v)

        self.last_alt = aval.alt
        self.last_alt_time = now

        if self.active:
            if aval.alt > self.safe_alt + 1:
                self.active = False
                LOG.warning('deactivated safety net @ %d meters', aval.alt)
        else:
            if aval.alt <= self.safe_alt:
                self.vehicle.mode = self.safe_mode
                self.active = True
                LOG.warning('activated safety net @ %d meters', aval.alt)

def parse_args():
    p = argparse.ArgumentParser()

    p.add_argument('--system-id', '-s',
                   default=255,
                   type=int,
                   help='Our mavlink system id')

    p.add_argument('--safe-mode', '-m',
                   default='BRAKE',
                   help='Mode to use for safety net')

    p.add_argument('port')

    p.add_argument('altitude',
                   nargs='?',
                   default='10',
                   type=int,
                   help='Safetynet activation altitude, in meters')

    return p.parse_args()

def main():
    args = parse_args()
    logging.basicConfig(level='INFO')
    vehicle = dronekit.connect(args.port,
                               source_system=args.system_id)

    net = SafetyNet(vehicle, args.altitude,
                    safe_mode=args.safe_mode)

    while True:
        time.sleep(2)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(1)
