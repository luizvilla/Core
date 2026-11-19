"""Independent injector on the SPOC's MUX Spin (ThingSet /Mux/Inj).

Firmware: owntech-foundation/SPOC_Software, branch mux-injector, commit
b500a4e5d37df710018dac232c65e7b8a362d959 (pinned in docs/safety/spoc_bench.md).
Wiring: MUX Spin pin 2 (PB12, OPAMP4 <- DAC4_CH1) through 220-470 ohm and
pin 4 (PB13, GPIO) through ~100 ohm, both to TP5 (CH3's signal test point).

Rules kept by the firmware: one source at a time, the idle one high
impedance; a change of source disables CH3's muxes and leaves them disabled,
so route CH3 to the target pin *after* choosing the source:

    inj = Injector(mux)          # mux: a connected MuxController
    inj.level(1200)              # DAC source at 1.2 V (full scale rVref_mV, 2048 here)
    mux.route('ch3', 'PA1')      # then route TP5 to the CUT pin
    inj.level(1500)              # same source: level changes, CH3 stays routed
    inj.off()
"""
from __future__ import annotations

import json

OFF, DAC, GPIO = 0, 1, 2
BASE = "Mux/Inj"


class Injector:
    def __init__(self, mux) -> None:
        self.mux = mux

    def _shell(self):
        return self.mux._require_shell()

    def _set(self, name: str, value: str) -> None:
        ok, response = self._shell().set_value(f"{BASE}/{name}", value)
        if not ok:
            raise RuntimeError(f"ThingSet write failed for {BASE}/{name}: {response}")

    def _apply(self, source: int, *, level_mV: int | None = None, high: bool | None = None) -> dict:
        if level_mV is not None:
            self._set("wLevel_mV", str(int(level_mV)))
        if high is not None:
            self._set("wGpio", "true" if high else "false")
        self._set("wSource", str(source))
        self._set("xApply", "true")
        st = self.status()
        if st["rSource"] != source:
            raise RuntimeError(f"injector source {source} not applied: {st}")
        return st

    def status(self) -> dict:
        raw = self._shell().send_cmd(f"?{BASE}", read_timeout=1.0).decode(errors="replace")
        start, end = raw.find("{"), raw.rfind("}")
        if start < 0 or end < start:
            raise RuntimeError(f"no injector status in reply: {raw!r}")
        return json.loads(raw[start:end + 1])

    def off(self) -> dict:
        return self._apply(OFF)

    def level(self, level_mV: int) -> dict:
        """DAC source (PB12). Returns the status; rLevel_mV is the level the DAC code gives."""
        return self._apply(DAC, level_mV=level_mV)

    def step(self, high: bool) -> dict:
        """GPIO source (PB13), 0 V or 3.3 V; edge under ~6 ns at the pin."""
        return self._apply(GPIO, high=high)
