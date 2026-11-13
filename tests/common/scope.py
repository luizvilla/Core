#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## @file scope.py
## @brief Captures the Tektronix MSO54: screenshot, measurements, setup, optionally waveforms.
## @par Used by
##   tests/l2/bench.py (bench sessions, level L2) through capture(); standalone for ad-hoc captures.
## @note pyvisa-py over USB returns 20 KiB per read and has no device clear: screenshot() loops
##   to the PNG trailer, and a failed query is recovered with drain(). MEASUrement:MEAS<x>:SOUrce2?
##   is a suffix error on single-source measurements (firmware 2.8.1): it is asked only for TWO_SOURCE.
"""Capture the Tektronix MSO54 screen, measurements and (optionally) waveforms.

Each call writes one folder under tests/out/scope/<timestamp>[_label]/:
    screen.png        screenshot of the scope display
    summary.md        instrument, timebase, trigger, channels, measurement table
    capture.json      the same data, machine-readable
    waveforms.npz     (--waveforms) time + value (channel units) per displayed channel
and points tests/out/scope/latest at it, so "look at the latest scope capture"
is enough to hand it to an agent.

Usage (from the repo root):
    .venv/bin/python tests/common/scope.py                    # screen + measurements
    .venv/bin/python tests/common/scope.py -l boost_step      # label the folder
    .venv/bin/python tests/common/scope.py --waveforms        # also pull CH1..CH4 data
    .venv/bin/python tests/common/scope.py --resource TCPIP::192.168.1.50::INSTR
    .venv/bin/python tests/common/scope.py --list             # show VISA resources

Needs: pyvisa, pyvisa-py, pyusb (in .venv) and the udev rule in
tests/common/udev/99-tektronix-mso54.rules for USB access without root.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import sys
from pathlib import Path

import pyvisa
from pyvisa.errors import VisaIOError

REPO = Path(__file__).resolve().parents[2]
OUT_ROOT = REPO / "tests" / "out" / "scope"
TEK_VIDS = ("0X0699", "1689")       # pyvisa-py prints the VID in decimal
SCOPE_FILE = "C:/claude_capture.png"   # temporary file on the scope's disk
PNG_END = b"IEND\xaeB`\x82"
N_CHANNELS = 4                          # MSO54
STATS = ["MEAN", "MINimum", "MAXimum", "STDDev", "PK2PK", "POPUlation"]
TWO_SOURCE = {"PHASE", "DELAY", "SKEW"}  # SOUrce2? answers only on these (suffix error otherwise)


class Scope:
    def __init__(self, resource: str | None, timeout_s: float = 10.0):
        self.rm = pyvisa.ResourceManager("@py")
        if resource is None:
            resource = self._find_usb()
        self.resource = resource
        self.inst = self.rm.open_resource(resource)
        self.inst.timeout = int(timeout_s * 1000)
        self.inst.read_termination = "\n"
        self.inst.write_termination = "\n"
        self.drain()
        self.inst.write("HEADer OFF")
        self.inst.write("VERBose ON")

    def drain(self) -> None:
        """Discard stale output (pyvisa-py USB has no device clear)."""
        old = self.inst.timeout
        self.inst.timeout = 300
        try:
            while True:
                self.inst.read_raw()
        except VisaIOError:
            pass
        finally:
            self.inst.timeout = old

    def _find_usb(self) -> str:
        found = [r for r in self.rm.list_resources("USB?*")
                 if any(f"::{v}::" in r.upper() for v in TEK_VIDS)]
        if not found:
            sys.exit("No Tektronix USB instrument found. Check the cable, the udev "
                     "rule, or pass --resource TCPIP::<ip>::INSTR.")
        return found[0]

    def q(self, cmd: str, timeout_s: float | None = None) -> str | None:
        """Query; return None (and recover) if the scope does not answer."""
        old = self.inst.timeout
        if timeout_s is not None:
            self.inst.timeout = int(timeout_s * 1000)
        try:
            return self.inst.query(cmd).strip().strip('"')
        except VisaIOError:
            self.drain()
            return None
        finally:
            self.inst.timeout = old

    def num(self, cmd: str) -> float | None:
        v = self.q(cmd, timeout_s=3)
        try:
            f = float(v)
        except (TypeError, ValueError):
            return None
        return None if abs(f) >= 9.9e37 else f   # 9.91E37 = "no value"

    def running(self) -> bool:
        return self.q("ACQuire:STATE?") in ("1", "ON", "RUN")

    def settle(self, seconds: float) -> str:
        """@brief Restart the measurement statistics and collect for `seconds` if the scope runs.
        A stopped scope (single shot) is left untouched: its last acquisition is the evidence.
        @return how the statistics were gathered, for the record."""
        import time
        if not self.running():
            return "scope stopped: statistics of the acquisitions held on screen"
        self.inst.write("CLEAR")
        self.q("*OPC?", timeout_s=10)
        time.sleep(seconds)
        n = self.num("ACQuire:NUMACq?")
        return f"statistics restarted, {seconds:g} s collected ({n or 0:.0f} acquisitions)"

    # ---- captures -------------------------------------------------------
    def screenshot(self) -> bytes:
        self.inst.write("SAVE:IMAGe:COMPosition NORMal")
        self.inst.write(f'SAVE:IMAGe "{SCOPE_FILE}"')
        self.q("*OPC?", timeout_s=20)
        old = self.inst.timeout
        self.inst.timeout = 30000
        try:
            self.inst.write(f'FILESystem:READFile "{SCOPE_FILE}"')
            data = b""
            # pyvisa-py returns 20 KiB per read: loop until the PNG trailer.
            while not data.rstrip(b"\n").endswith(PNG_END):
                data += self.inst.read_raw()
        finally:
            self.inst.timeout = old
        self.inst.write(f'FILESystem:DELEte "{SCOPE_FILE}"')
        return data.rstrip(b"\n")

    def setup(self) -> dict:
        s: dict = {
            "idn": self.q("*IDN?"),
            "resource": self.resource,
            "acquire": {
                "state": self.q("ACQuire:STATE?"),
                "mode": self.q("ACQuire:MODe?"),
                "stopafter": self.q("ACQuire:STOPAfter?"),
                "num_acq": self.num("ACQuire:NUMACq?"),
            },
            "horizontal": {
                "scale_s_div": self.num("HORizontal:SCAle?"),
                "position_pct": self.num("HORizontal:POSition?"),
                "record_length": self.num("HORizontal:RECOrdlength?"),
                "sample_rate": self.num("HORizontal:SAMPLERate?"),
            },
        }
        src = self.q("TRIGger:A:EDGE:SOUrce?")
        s["trigger"] = {
            "type": self.q("TRIGger:A:TYPe?"),
            "source": src,
            "slope": self.q("TRIGger:A:EDGE:SLOpe?"),
            "coupling": self.q("TRIGger:A:EDGE:COUPling?"),
            "level": self.num(f"TRIGger:A:LEVel:{src}?") if src else None,
            "mode": self.q("TRIGger:A:MODe?"),
        }
        chans = {}
        for n in range(1, N_CHANNELS + 1):
            ch = f"CH{n}"
            on = self.q(f"DISplay:GLObal:{ch}:STATE?")
            if on not in ("1", "ON"):
                continue
            chans[ch] = {
                "label": self.q(f"{ch}:LABel:NAMe?"),
                "scale_div": self.num(f"{ch}:SCAle?"),
                "offset": self.num(f"{ch}:OFFSet?"),
                "position_div": self.num(f"{ch}:POSition?"),
                "coupling": self.q(f"{ch}:COUPling?"),
                "bandwidth": self.num(f"{ch}:BANdwidth?"),
                "termination": self.num(f"{ch}:TERmination?"),
                "probe_units": self.q(f"{ch}:PRObe:UNIts?", timeout_s=2),
            }
        s["channels"] = chans
        return s

    def measurements(self) -> list[dict]:
        lst = self.q("MEASUrement:LIST?")
        if not lst or lst.upper() == "NONE":
            return []
        out = []
        for m in [x.strip() for x in lst.split(",") if x.strip()]:
            base = f"MEASUrement:{m}"
            typ = self.q(f"{base}:TYPe?")
            rec = {
                "id": m,
                "type": typ,
                "source1": self.q(f"{base}:SOUrce1?"),
                "source2": self.q(f"{base}:SOUrce2?", timeout_s=2) if (typ or "").upper() in TWO_SOURCE else None,
                "edges": self.edges(m, typ),
                "unit": self.q(f"{base}:YUNIT?", timeout_s=2),
                "current": {s.lower()[:6]: self.num(f"{base}:RESUlts:CURRentacq:{s}?")
                            for s in STATS},
                "all": {s.lower()[:6]: self.num(f"{base}:RESUlts:ALLAcqs:{s}?")
                        for s in STATS},
            }
            out.append(rec)
        return out

    def measurement_list(self) -> list[dict]:
        """@brief Measurements on the scope: [{id, type, source1, source2}] (no results)."""
        lst = self.q("MEASUrement:LIST?")
        out = []
        for m in [x.strip() for x in (lst or "").split(",") if x.strip() and x.strip().upper() != "NONE"]:
            typ = self.q(f"MEASUrement:{m}:TYPe?")
            out.append({"id": m, "type": typ, "source1": self.q(f"MEASUrement:{m}:SOUrce1?"),
                        "source2": self.q(f"MEASUrement:{m}:SOUrce2?", timeout_s=2)
                        if (typ or "").upper() in TWO_SOURCE else None,
                        "edges": self.edges(m, typ)})
        return out

    def edges(self, m: str, typ: str | None) -> list | None:
        """@brief [from edge, to edge] ('RISE'/'FALL'/'BOTH') of a DELAY measurement, else None."""
        if (typ or "").upper() != "DELAY":
            return None
        e = [self.q(f"MEASUrement:{m}:FROMedge?", timeout_s=2), self.q(f"MEASUrement:{m}:TOEdge?", timeout_s=2)]
        return [(x or "").upper()[:4] for x in e]

    def add_measurement(self, typ: str, src1: str, src2: str | None = None, edges=None) -> str | None:
        """@brief Add a measurement and set its sources (and, for DELAY, its from/to edges RISE|FALL).
        @return its id (MEASn), or None."""
        before = set((self.q("MEASUrement:LIST?") or "").split(","))
        self.inst.write(f"MEASUrement:ADDMEAS {typ}")
        self.q("*OPC?", timeout_s=10)
        new = [m for m in (self.q("MEASUrement:LIST?") or "").split(",") if m and m not in before]
        if not new:
            return None
        m = new[0]
        self.inst.write(f"MEASUrement:{m}:SOUrce1 {src1}")
        if src2:
            self.inst.write(f"MEASUrement:{m}:SOUrce2 {src2}")
        if edges:
            self.inst.write(f"MEASUrement:{m}:FROMedge {edges[0]}")
            self.inst.write(f"MEASUrement:{m}:TOEdge {edges[1]}")
            self.inst.write(f"MEASUrement:{m}:TOEDGESEARCHDIRect FORWard")
        self.q("*OPC?", timeout_s=10)
        return m

    def delete_measurement(self, m: str) -> None:
        self.inst.write(f'MEASUrement:DELete "{m}"')
        self.q("*OPC?", timeout_s=10)

    def arm_single(self) -> None:
        """@brief Single sequence: the scope stops after the next trigger."""
        self.inst.write("ACQuire:STOPAfter SEQuence")
        self.inst.write("ACQuire:STATE RUN")  # no *OPC? here: on a single sequence it waits for the trigger

    def run_continuous(self) -> None:
        self.inst.write("ACQuire:STOPAfter RUNSTop")
        self.inst.write("ACQuire:STATE RUN")

    def waveform(self, ch: str):
        import numpy as np
        reclen = int(self.num("HORizontal:RECOrdlength?") or 10000)
        self.inst.write(f"DATa:SOUrce {ch}")
        self.inst.write("DATa:ENCdg SRIbinary")      # signed int, LSB first
        self.inst.write("WFMOutpre:BYT_Nr 2")
        self.inst.write("DATa:STARt 1")
        self.inst.write(f"DATa:STOP {reclen}")
        p = {k: self.num(f"WFMOutpre:{k}?")
             for k in ("YMUlt", "YOFf", "YZEro", "XINcr", "XZEro", "PT_Off")}
        old = self.inst.timeout
        self.inst.timeout = 60000
        try:
            raw = self.inst.query_binary_values("CURVe?", datatype="h",
                                                is_big_endian=False,
                                                container=np.array)
        finally:
            self.inst.timeout = old
        v = (raw - (p["YOFf"] or 0)) * (p["YMUlt"] or 1) + (p["YZEro"] or 0)
        t = (np.arange(len(v)) - (p["PT_Off"] or 0)) * (p["XINcr"] or 1) + (p["XZEro"] or 0)
        return t, v


# ---- report ---------------------------------------------------------------
def eng(x: float | None, unit: str = "") -> str:
    if x is None:
        return "—"
    if x == 0:
        return f"0 {unit}".strip()
    prefixes = [(1e9, "G"), (1e6, "M"), (1e3, "k"), (1, ""), (1e-3, "m"),
                (1e-6, "µ"), (1e-9, "n"), (1e-12, "p")]
    for f, p in prefixes:
        if abs(x) >= f:
            return f"{x / f:.4g} {p}{unit}".strip()
    return f"{x:.4g} {unit}".strip()


def summary_md(cap: dict) -> str:
    s, ms = cap["setup"], cap["measurements"]
    h, t, a = s["horizontal"], s["trigger"], s["acquire"]
    lines = [
        f"# Scope capture {cap['timestamp']}" + (f" — {cap['label']}" if cap["label"] else ""),
        "",
        f"- Instrument: {s['idn']}",
        *([f"- Statistics: {cap['stats']}"] if cap.get("stats") else []),
        f"- Acquisition: {a['state']} / {a['mode']} / stop after {a['stopafter']}, "
        f"{a['num_acq']:.0f} acqs" if a["num_acq"] is not None else
        f"- Acquisition: {a['state']} / {a['mode']}",
        f"- Timebase: {eng(h['scale_s_div'], 's')}/div, record {h['record_length']:.0f} pts, "
        f"{eng(h['sample_rate'], 'S/s')}" if h["record_length"] else
        f"- Timebase: {eng(h['scale_s_div'], 's')}/div",
        f"- Trigger: {t['type']} {t['source']} {t['slope']} @ {eng(t['level'], 'V')}, "
        f"mode {t['mode']}",
        "",
        "## Channels",
        "",
        "| Ch | Label | Scale/div | Offset | Pos (div) | Coupling | BW | Term |",
        "|---|---|---|---|---|---|---|---|",
    ]
    for ch, c in s["channels"].items():
        u = c["probe_units"] or "V"
        lines.append(f"| {ch} | {c['label'] or ''} | {eng(c['scale_div'], u)} | "
                     f"{eng(c['offset'], u)} | {c['position_div']} | {c['coupling']} | "
                     f"{eng(c['bandwidth'], 'Hz')} | {eng(c['termination'], 'Ω')} |")
    lines += ["", "## Measurements", ""]
    if not ms:
        lines.append("No measurements configured on the scope.")
    else:
        lines += ["| Meas | Type | Source | Current mean | All mean | All min | All max "
                  "| All σ | N |",
                  "|---|---|---|---|---|---|---|---|---|"]
        for m in ms:
            u = m["unit"] or ""
            src = m["source1"] or ""
            c, al = m["current"], m["all"]
            n = al.get("popula")
            lines.append(f"| {m['id']} | {m['type']} | {src} | {eng(c['mean'], u)} | "
                         f"{eng(al['mean'], u)} | {eng(al['minimu'], u)} | "
                         f"{eng(al['maximu'], u)} | {eng(al['stddev'], u)} | "
                         f"{'' if n is None else f'{n:.0f}'} |")
    if cap.get("waveforms"):
        lines += ["", "## Waveforms (waveforms.npz)", "",
                  "| Ch | Points | Min | Max | Mean | RMS |", "|---|---|---|---|---|---|"]
        for ch, w in cap["waveforms"].items():
            u = s["channels"][ch]["probe_units"] or "V"
            lines.append(f"| {ch} | {w['points']} | {eng(w['min'], u)} | "
                         f"{eng(w['max'], u)} | {eng(w['mean'], u)} | "
                         f"{eng(w['rms'], u)} |")
    lines += ["", "![screen](screen.png)", ""]
    return "\n".join(lines)


def capture(scope: Scope, out: Path, label: str = "", waveforms: bool = False,
            screen: bool = True, stats_note: str = "") -> dict:
    """@brief Write screen.png, capture.json, summary.md (and waveforms.npz) into `out`.
    A running scope is stopped first and restarted at the end, so the measurements, the screen and every channel's
    waveform come from the same acquisition (each channel takes seconds to read; read from a running scope they came
    from successive sweeps, after the bench had moved on: L2-INV-09, 2026-11-13).
    @return the capture dict (as in capture.json)."""
    out.mkdir(parents=True, exist_ok=True)
    was_running = scope.running()
    if was_running:
        scope.inst.write("ACQuire:STATE STOP")
        scope.q("*OPC?", timeout_s=10)
    try:
        return _capture_held(scope, out, label, waveforms, screen, stats_note, was_running)
    finally:
        if was_running:
            scope.inst.write("ACQuire:STATE RUN")


def _capture_held(scope: Scope, out: Path, label: str, waveforms: bool, screen: bool, stats_note: str,
                  was_running: bool) -> dict:
    """@brief capture() on a held acquisition."""
    cap = {"timestamp": dt.datetime.now().isoformat(timespec="seconds"), "label": label,
           "stats": stats_note, "held": "stopped for the capture, restarted after" if was_running else "already stopped",
           "setup": scope.setup(), "measurements": scope.measurements()}
    if waveforms:
        import numpy as np
        arrays, stats = {}, {}
        for ch in cap["setup"]["channels"]:
            t, v = scope.waveform(ch)
            arrays[f"{ch}_t"], arrays[f"{ch}_v"] = t, v
            stats[ch] = {"points": int(len(v)), "min": float(v.min()),
                         "max": float(v.max()), "mean": float(v.mean()),
                         "rms": float(np.sqrt(np.mean(v ** 2)))}
        np.savez_compressed(out / "waveforms.npz", **arrays)
        cap["waveforms"] = stats
    if screen:
        (out / "screen.png").write_bytes(scope.screenshot())
    (out / "capture.json").write_text(json.dumps(cap, indent=2))
    (out / "summary.md").write_text(summary_md(cap))
    return cap


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-r", "--resource", help="VISA resource (default: first Tek USB)")
    ap.add_argument("-l", "--label", default="", help="suffix for the capture folder")
    ap.add_argument("-o", "--out", type=Path, help="output folder (overrides default)")
    ap.add_argument("--waveforms", action="store_true",
                    help="also download displayed channels to waveforms.npz")
    ap.add_argument("--no-screen", action="store_true", help="skip the screenshot")
    ap.add_argument("--list", action="store_true", help="list VISA resources and exit")
    args = ap.parse_args()

    if args.list:
        rm = pyvisa.ResourceManager("@py")
        for r in rm.list_resources():
            print(r)
        return

    scope = Scope(args.resource)
    stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    out = args.out or OUT_ROOT / (stamp + (f"_{args.label}" if args.label else ""))
    capture(scope, out, label=args.label, waveforms=args.waveforms, screen=not args.no_screen)
    if args.out is None:
        latest = OUT_ROOT / "latest"
        if latest.is_symlink() or latest.exists():
            latest.unlink()
        latest.symlink_to(out.name)
    print(out)


if __name__ == "__main__":
    main()
