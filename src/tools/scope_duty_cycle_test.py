#
# Copyright (c) 2021-present LAAS-CNRS
#
#   This program is free software: you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation, either version 2 of the License, or
#   (at your option) any later version.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# @author Luiz Villa <luiz.villa@laas.fr>
#

"""
scope_duty_cycle_test.py

Live-hardware acceptance script for the software-scope feature. Runs three
fixed-duty-cycle (open-loop, no PID) power phases in sequence - Leg 1 only,
Leg 2 only, then both legs together - and captures a decimated scope buffer
during each one. Every phase writes a CSV, a plain-text report, and a PNG
plot so a capture can be inspected without touching the hardware again.

This script does not control the external supply. It assumes, per this
project's established live-test conditions (see automated_test.md):

    - The high side is already energized at a known, operator-controlled
      voltage (nominally 30 V) on an isolated, current-limited bench.
    - No external load is connected.
    - Both --shell-port and --scope-port are explicit stable
      /dev/serial/by-id/... paths for the target board's if02 (ThingSet
      shell) and if00 (scope data) interfaces - no auto-detection, so a
      protected or unrelated board can never be opened by accident.
    - Global switching frequency is left at its compiled 200 kHz default;
      this script never writes Config/Frequency_Hz (known firmware issue,
      waived in prior acceptance runs).

Because the applied duty cycle is open-loop (wBuck/wBoost left false), there
is no feedback to stop a runaway output. A software interlock polls
Measurements during the settle period and after capture, and immediately
requests a shutdown if any voltage or current magnitude exceeds
--max-voltage / --max-current.

Example:

    python3 scope_duty_cycle_test.py \\
        --shell-port /dev/serial/by-id/usb-OwnTech...-if02 \\
        --scope-port /dev/serial/by-id/usb-OwnTech...-if00 \\
        --enable-power
"""

import argparse
import csv
import datetime
import json
import statistics
import sys
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from power_test_bench import PowerTestBench, TesterMode
from scope_serial import ScopeSerial
from thingset_tools import ThingSetTools


DEFAULT_DUTY = 0.3
DEFAULT_DECIMATION = 1
DEFAULT_PRETRIGGER = 0.2
DEFAULT_SETTLE_S = 0.5
DEFAULT_POLL_INTERVAL_S = 0.05
DEFAULT_MAX_VOLTAGE_V = 35.0
DEFAULT_MAX_CURRENT_A = 3.0

VOLTAGE_FIELDS = ("rV1Low_V", "rV2Low_V", "rVHigh_V")
CURRENT_FIELDS = ("rI1Low_A", "rI2Low_A", "rIHigh_A")

PHASES = (
    ("leg1_only", (1,)),
    ("leg2_only", (2,)),
    ("both_legs", (1, 2)),
)


class SafetyInterlockError(RuntimeError):
    """Raised when a live measurement exceeds a configured safety limit."""


def log(message):
    timestamp = datetime.datetime.now().strftime("%H:%M:%S")
    print(f"[{timestamp}] {message}", flush=True)


def check_interlock(measurements, max_voltage, max_current):
    for field in VOLTAGE_FIELDS:
        value = measurements.get(field)
        if isinstance(value, (int, float)) and abs(value) > max_voltage:
            raise SafetyInterlockError(
                f"{field}={value:g} V exceeds interlock limit "
                f"{max_voltage:g} V"
            )
    for field in CURRENT_FIELDS:
        value = measurements.get(field)
        if isinstance(value, (int, float)) and abs(value) > max_current:
            raise SafetyInterlockError(
                f"{field}={value:g} A exceeds interlock limit "
                f"{max_current:g} A"
            )


def settle_with_interlock(bench, seconds, poll_interval, max_voltage, max_current):
    """Poll measurements for `seconds`, aborting immediately on an
    interlock trip. Returns the last measurement set."""

    deadline = time.monotonic() + seconds
    measurements = bench.read_measurements()
    check_interlock(measurements, max_voltage, max_current)
    while time.monotonic() < deadline:
        time.sleep(min(poll_interval, max(0.0, deadline - time.monotonic())))
        measurements = bench.read_measurements()
        check_interlock(measurements, max_voltage, max_current)
    return measurements


def configure_phase_legs(bench, active_legs, duty):
    for leg in (1, 2):
        if leg in active_legs:
            bench.configure_leg(
                leg,
                enable=True,
                driver=True,
                capacitor=True,
                buck=False,
                boost=False,
                duty_cycle=duty,
            )
        else:
            bench.configure_leg(leg, enable=False, driver=False, capacitor=False)


def run_phase(bench, phase_name, active_legs, args, output_dir):
    log(f"--- Phase: {phase_name} (active legs: {list(active_legs)}) ---")
    report = {
        "phase": phase_name,
        "active_legs": list(active_legs),
        "duty_cycle": args.duty,
        "ok": False,
    }
    try:
        bench.shutdown(disconnect_hardware=True)
        configure_phase_legs(bench, active_legs, args.duty)
        bench.set_mode(TesterMode.POWER_ON)

        log(
            f"Powered on; settling {args.settle_seconds:g} s with interlock "
            f"|V|<={args.max_voltage:g} V, |I|<={args.max_current:g} A"
        )
        report["pre_capture_measurements"] = settle_with_interlock(
            bench,
            args.settle_seconds,
            args.poll_interval,
            args.max_voltage,
            args.max_current,
        )

        bench.arm_scope(pretrigger_ratio=args.pretrigger, decimation=args.decimation)
        bench.trigger_scope()
        bench.wait_scope_ready()
        capture = bench.download_scope()
        report["capture"] = capture

        post_measurements = bench.read_measurements()
        check_interlock(post_measurements, args.max_voltage, args.max_current)
        report["post_capture_measurements"] = post_measurements
        report["scope_status"] = bench.read_scope_status()
        report["ok"] = True
    except Exception as exc:
        report["error"] = str(exc)
        raise
    finally:
        try:
            bench.shutdown(disconnect_hardware=True)
        except Exception as exc:
            report["shutdown_error"] = str(exc)
        write_phase_outputs(output_dir, phase_name, report)
    return report


def write_scope_csv(path, capture):
    with open(path, "w", newline="", encoding="utf-8") as output:
        writer = csv.writer(output)
        writer.writerow(("time_s",) + capture.channel_names)
        for timestamp, sample in zip(capture.time_axis_s, capture.samples):
            writer.writerow((timestamp,) + sample)


def plot_capture(capture, phase_name, png_path):
    time_ms = [t * 1000.0 for t in capture.time_axis_s]
    columns = {
        name: [row[index] for row in capture.samples]
        for index, name in enumerate(capture.channel_names)
    }

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle(f"Scope capture - {phase_name}")

    axes[0].plot(time_ms, columns["V1Low_V"], label="V1Low_V")
    axes[0].plot(time_ms, columns["V2Low_V"], label="V2Low_V")
    axes[0].plot(time_ms, columns["VHigh_V"], label="VHigh_V")
    axes[0].set_ylabel("Voltage (V)")
    axes[0].legend(loc="upper right", fontsize="small")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(time_ms, columns["I1Low_A"], label="I1Low_A")
    axes[1].plot(time_ms, columns["I2Low_A"], label="I2Low_A")
    axes[1].plot(time_ms, columns["IHigh_A"], label="IHigh_A")
    axes[1].set_ylabel("Current (A)")
    axes[1].legend(loc="upper right", fontsize="small")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(time_ms, columns["Duty1"], label="Duty1")
    axes[2].plot(time_ms, columns["Duty2"], label="Duty2")
    axes[2].set_ylabel("Duty")
    axes[2].set_xlabel("Time relative to trigger (ms)")
    axes[2].legend(loc="upper right", fontsize="small")
    axes[2].grid(True, alpha=0.3)

    for axis in axes:
        axis.axvline(0.0, color="black", linewidth=0.8, linestyle="--")

    fig.tight_layout()
    fig.savefig(png_path, dpi=150)
    plt.close(fig)


def write_phase_outputs(output_dir, phase_name, report):
    lines = [
        f"Phase: {phase_name}",
        f"Active legs: {report.get('active_legs')}",
        f"Requested duty cycle: {report.get('duty_cycle')}",
        f"Result: {'OK' if report.get('ok') else 'FAILED'}",
    ]
    if report.get("error"):
        lines.append(f"Error: {report['error']}")
    if report.get("shutdown_error"):
        lines.append(f"Shutdown error: {report['shutdown_error']}")
    lines.append("")

    if "pre_capture_measurements" in report:
        lines.append("Pre-capture measurements (after settle):")
        lines.append(json.dumps(report["pre_capture_measurements"], indent=2))
        lines.append("")
    if "post_capture_measurements" in report:
        lines.append("Post-capture measurements:")
        lines.append(json.dumps(report["post_capture_measurements"], indent=2))
        lines.append("")
    if "scope_status" in report:
        lines.append("Scope status at download:")
        lines.append(json.dumps(report["scope_status"], indent=2))
        lines.append("")

    capture = report.get("capture")
    if capture is not None:
        lines.append(
            f"Capture: {len(capture.samples)} samples, "
            f"{capture.sample_period_us} us period, "
            f"{capture.duration_ms:g} ms window, "
            f"final_index={capture.final_index}, "
            f"pretrigger_ratio={capture.pretrigger_ratio:g}"
        )
        lines.append("")
        lines.append("Per-channel statistics over the full capture window:")
        columns = list(zip(*capture.samples))
        for name, column in zip(capture.channel_names, columns):
            lines.append(
                f"  {name:10s} min={min(column): .6g}  "
                f"mean={statistics.fmean(column): .6g}  "
                f"max={max(column): .6g}  "
                f"pk-pk={max(column) - min(column): .6g}"
            )

    (output_dir / f"{phase_name}.txt").write_text(
        "\n".join(lines) + "\n", encoding="utf-8"
    )

    if capture is not None:
        write_scope_csv(output_dir / f"{phase_name}.csv", capture)
        plot_capture(capture, phase_name, output_dir / f"{phase_name}.png")


def write_summary(output_dir, args, reports, metadata):
    lines = [
        "Scope duty-cycle test summary",
        f"Run directory: {output_dir}",
        f"Converter metadata: {json.dumps(metadata)}",
        f"Requested duty cycle: {args.duty:g}",
        f"Decimation: {args.decimation}, pretrigger ratio: {args.pretrigger:g}",
        f"Settle time: {args.settle_seconds:g} s",
        f"Interlock limits: |V| <= {args.max_voltage:g} V, "
        f"|I| <= {args.max_current:g} A",
        "",
    ]
    for report in reports:
        status = "OK" if report.get("ok") else "FAILED"
        lines.append(f"- {report['phase']}: {status}")
        if report.get("error"):
            lines.append(f"    error: {report['error']}")
        if report.get("shutdown_error"):
            lines.append(f"    shutdown_error: {report['shutdown_error']}")
    (output_dir / "summary.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")


def build_parser():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--shell-port",
        required=True,
        help="explicit ThingSet shell serial path (if02), e.g. "
        "/dev/serial/by-id/usb-OwnTech...-if02",
    )
    parser.add_argument(
        "--scope-port",
        required=True,
        help="explicit scope data serial path (if00), e.g. "
        "/dev/serial/by-id/usb-OwnTech...-if00",
    )
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--duty",
        type=float,
        default=DEFAULT_DUTY,
        help="fixed open-loop duty cycle applied to every active leg, 0-1 "
        f"(default {DEFAULT_DUTY:g})",
    )
    parser.add_argument(
        "--decimation",
        type=int,
        default=DEFAULT_DECIMATION,
        help=f"scope decimation, 1-100 (default {DEFAULT_DECIMATION})",
    )
    parser.add_argument(
        "--pretrigger",
        type=float,
        default=DEFAULT_PRETRIGGER,
        help=f"scope pre-trigger ratio, 0.0-0.9 (default {DEFAULT_PRETRIGGER:g})",
    )
    parser.add_argument(
        "--settle-seconds",
        type=float,
        default=DEFAULT_SETTLE_S,
        help="settle time after POWER_ON before arming the scope "
        f"(default {DEFAULT_SETTLE_S:g})",
    )
    parser.add_argument(
        "--poll-interval",
        type=float,
        default=DEFAULT_POLL_INTERVAL_S,
        help="interlock polling interval during settle, seconds "
        f"(default {DEFAULT_POLL_INTERVAL_S:g})",
    )
    parser.add_argument(
        "--max-voltage",
        type=float,
        default=DEFAULT_MAX_VOLTAGE_V,
        help="abort and shut down if any voltage measurement exceeds this "
        f"magnitude, volts (default {DEFAULT_MAX_VOLTAGE_V:g})",
    )
    parser.add_argument(
        "--max-current",
        type=float,
        default=DEFAULT_MAX_CURRENT_A,
        help="abort and shut down if any current measurement exceeds this "
        f"magnitude, amps (default {DEFAULT_MAX_CURRENT_A:g})",
    )
    parser.add_argument(
        "--output-dir",
        default="scope_test_results",
        help="parent directory for a new timestamped run directory "
        "(default scope_test_results)",
    )
    parser.add_argument(
        "--json",
        default="thingset_objects.json",
        help="discovery output path; pass an empty string to disable",
    )
    parser.add_argument(
        "--enable-power",
        action="store_true",
        help="required: explicitly confirms the bench is isolated, "
        "current-limited, and intentionally energized before any leg is "
        "enabled",
    )
    parser.add_argument(
        "--continue-on-error",
        action="store_true",
        help="attempt remaining phases after a phase failure instead of "
        "stopping (every phase always ends with a shutdown request)",
    )
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)

    if not args.enable_power:
        parser.error(
            "--enable-power is required: this script energizes Twist legs "
            "at a fixed, open-loop duty cycle with no closed-loop "
            "regulation. Pass --enable-power only after confirming the "
            "bench is isolated, current-limited, and intentionally "
            "energized (see automated_test.md for the expected conditions)."
        )
    if args.duty < 0.0 or args.duty > 1.0:
        parser.error("--duty must be between 0.0 and 1.0")
    if not 1 <= args.decimation <= 100:
        parser.error("--decimation must be between 1 and 100")
    if not 0.0 <= args.pretrigger <= 0.9:
        parser.error("--pretrigger must be between 0.0 and 0.9")
    if args.settle_seconds < 0:
        parser.error("--settle-seconds must be non-negative")
    if args.poll_interval <= 0:
        parser.error("--poll-interval must be positive")
    if args.max_voltage <= 0 or args.max_current <= 0:
        parser.error("--max-voltage and --max-current must be positive")

    run_dir = Path(args.output_dir) / datetime.datetime.now(
        datetime.timezone.utc
    ).strftime("%Y%m%dT%H%M%SZ")
    run_dir.mkdir(parents=True, exist_ok=True)
    log(f"Writing results to {run_dir}")

    reports = []
    metadata = {}
    exit_code = 0

    with ThingSetTools(args.shell_port, baudrate=args.baud) as client, ScopeSerial(
        args.scope_port
    ) as scope_client:
        client.discover(json_path=args.json or None)
        bench = PowerTestBench(client, scope_client)

        metadata = bench.read_metadata()
        log(f"Converter metadata: {json.dumps(metadata)}")

        bench.shutdown(disconnect_hardware=True)

        try:
            for phase_name, active_legs in PHASES:
                try:
                    report = run_phase(bench, phase_name, active_legs, args, run_dir)
                    reports.append(report)
                    log(f"Phase {phase_name}: OK")
                except Exception as exc:
                    reports.append(
                        {
                            "phase": phase_name,
                            "ok": False,
                            "error": str(exc),
                        }
                    )
                    log(f"Phase {phase_name}: FAILED - {exc}")
                    exit_code = 1
                    if not args.continue_on_error:
                        break
        finally:
            try:
                bench.shutdown(disconnect_hardware=True)
                log("Final shutdown OK: POWER_OFF, both legs disabled")
            except Exception as exc:
                log(f"Final shutdown reported errors: {exc}")
                exit_code = 1

    write_summary(run_dir, args, reports, metadata)
    log(f"Done. Results in {run_dir}")
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
