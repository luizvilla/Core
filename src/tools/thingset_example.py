#
# Copyright (c) 2021-present LAAS-CNRS
#
#   This program is free software: you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation, either version 2 of the License, or
#   (at your option) any later version.
#
# SPDX-License-Identifier: GPL-2.0-or-later
#

"""Safe-by-default command-line example for the ThingSet power test bench."""

import argparse
import json
import time

from power_test_bench import PowerTestBench
from thingset_tools import ThingSetTools


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="serial port, e.g. /dev/ttyACM1")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--leg", type=int, choices=(1, 2), default=1)
    parser.add_argument("--duty", type=float, default=0.1)
    parser.add_argument(
        "--duration",
        type=float,
        default=1.0,
        help="powered duration in seconds (only with --enable-power)",
    )
    parser.add_argument(
        "--enable-power",
        action="store_true",
        help="explicitly allow the selected leg to enter POWER_ON",
    )
    parser.add_argument(
        "--connect-driver",
        action="store_true",
        help="connect the selected gate driver during powered operation",
    )
    parser.add_argument(
        "--connect-capacitor",
        action="store_true",
        help="connect the selected capacitor during powered operation",
    )
    parser.add_argument(
        "--json",
        default="thingset_objects.json",
        help="discovery output file; pass an empty string to disable",
    )
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.duration < 0:
        parser.error("--duration must be non-negative")
    if not args.enable_power and (
        args.connect_driver or args.connect_capacitor
    ):
        parser.error(
            "--connect-driver and --connect-capacitor require --enable-power"
        )

    with ThingSetTools(args.port, baudrate=args.baud) as client:
        client.discover(json_path=args.json or None)
        bench = PowerTestBench(client)

        # Establish the safe baseline before applying any requested duty.
        bench.shutdown()
        bench.configure_leg(args.leg, duty_cycle=args.duty)

        print("Converter metadata:")
        print(json.dumps(bench.read_metadata(), indent=2))
        print("Measurements:")
        print(json.dumps(bench.read_measurements(), indent=2))

        if not args.enable_power:
            print("Power remains OFF; pass --enable-power to energize one leg.")
            return

        disconnect_afterward = args.connect_driver or args.connect_capacitor
        try:
            bench.power_on(
                args.leg,
                connect_driver=args.connect_driver,
                connect_capacitor=args.connect_capacitor,
                duty_cycle=args.duty,
            )
            print(
                f"Leg {args.leg} is powered for {args.duration:.3f} seconds."
            )
            time.sleep(args.duration)
            print(json.dumps(bench.read_measurements(), indent=2))
        finally:
            bench.shutdown(disconnect_hardware=disconnect_afterward)


if __name__ == "__main__":
    main()
