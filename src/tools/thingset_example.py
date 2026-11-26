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
from contextlib import nullcontext
import csv
import json
import time

from power_test_bench import PowerTestBench
from scope_serial import ScopeSerial
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
    parser.add_argument(
        "--scope-port",
        help="explicit scope data port (the board's non-ThingSet if00)",
    )
    parser.add_argument(
        "--capture",
        action="store_true",
        help="capture scope data while power remains off unless separately enabled",
    )
    parser.add_argument(
        "--pretrigger",
        type=float,
        default=0.2,
        help="scope pre-trigger ratio from 0.0 to 0.9",
    )
    parser.add_argument(
        "--decimation",
        type=int,
        default=1,
        help="scope decimation from 1 to 100",
    )
    parser.add_argument(
        "--scope-csv",
        help="optional output CSV path for the decoded scope capture",
    )
    return parser


def write_scope_csv(path, capture):
    with open(path, "w", newline="", encoding="utf-8") as output:
        writer = csv.writer(output)
        writer.writerow(("time_s",) + capture.channel_names)
        for timestamp, sample in zip(capture.time_axis_s, capture.samples):
            writer.writerow((timestamp,) + sample)


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
    if args.capture and not args.scope_port:
        parser.error("--capture requires an explicit --scope-port")
    if args.scope_csv and not args.capture:
        parser.error("--scope-csv requires --capture")
    if args.pretrigger < 0.0 or args.pretrigger > 0.9:
        parser.error("--pretrigger must be between 0.0 and 0.9")
    if args.decimation < 1 or args.decimation > 100:
        parser.error("--decimation must be between 1 and 100")

    with ThingSetTools(args.port, baudrate=args.baud) as client:
        scope_context = (
            ScopeSerial(args.scope_port) if args.capture else nullcontext(None)
        )
        with scope_context as scope_client:
            client.discover(json_path=args.json or None)
            bench = PowerTestBench(client, scope_client)

            # Establish the safe baseline before applying any requested duty.
            bench.shutdown()
            bench.configure_leg(args.leg, duty_cycle=args.duty)

            print("Converter metadata:")
            print(json.dumps(bench.read_metadata(), indent=2))
            print("Measurements:")
            print(json.dumps(bench.read_measurements(), indent=2))

            if args.capture:
                bench.arm_scope(
                    pretrigger_ratio=args.pretrigger,
                    decimation=args.decimation,
                )
                bench.trigger_scope()
                bench.wait_scope_ready()
                capture = bench.download_scope()
                print(
                    "Scope capture: "
                    f"{len(capture.samples)} samples, "
                    f"{capture.sample_period_us} us period, "
                    f"{capture.duration_ms:g} ms window"
                )
                if args.scope_csv:
                    write_scope_csv(args.scope_csv, capture)
                    print(f"Scope CSV written to {args.scope_csv}")

            if not args.enable_power:
                print(
                    "Power remains OFF; pass --enable-power to energize one leg."
                )
                return

            disconnect_afterward = (
                args.connect_driver or args.connect_capacitor
            )
            try:
                bench.power_on(
                    args.leg,
                    connect_driver=args.connect_driver,
                    connect_capacitor=args.connect_capacitor,
                    duty_cycle=args.duty,
                )
                print(
                    f"Leg {args.leg} is powered for "
                    f"{args.duration:.3f} seconds."
                )
                time.sleep(args.duration)
                print(json.dumps(bench.read_measurements(), indent=2))
            finally:
                bench.shutdown(disconnect_hardware=disconnect_afterward)


if __name__ == "__main__":
    main()
