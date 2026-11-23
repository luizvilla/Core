# L0-EDG · edge cases of the hardware trips

Arming, clearing and reporting at the edges, on the SPOC bench (ADR 0009, issue luizvilla/Core#23, part of C14). `SAFETY_L0_TWIST` on the CUT, wiring as L0-TRP: the CUT's DAC drives I1 (PA1) through CH1 → TP1–TP3 → CH2, the MUX injector holds or drives I2 (PC1) through CH3.

```bash
bash tests/l0/spoc_edge/run.sh --run
```

| ID | Case | Expected |
|---|---|---|
| L0-EDG-01 | first PWM start of a fresh boot with I1 50 mV above its high limit | arming refused, outputs never on |
| L0-EDG-02 | after a trip, PWM off: clear while still above the limit, then at 0 A | refused (-1), fault kept; then accepted (0) |
| L0-EDG-03 | a second fault (from the MUX injector, on I2) about 70 ms after a clear, then a clear about 0.6 s later, then a reboot and a clear | second clear refused (-2), `/Safety rRebootRequired` true, "reboot required" in the text; after the reboot: flag false, fault still latched, clear accepted |
| L0-EDG-04 | I1, then I2, above their high limits | both comparator fault inputs recorded, outputs off |
| L0-EDG-05 | I1 20 mV inside each threshold for 10 s, PWM on | no trip |
| L0-EDG-06 | pulses on I2 (MUX injector pulse mode: 1.0 V hold to 2.0 V and back), widths 60-420 ns requested, each captured on SPOC CH3 | no comparator trip up to 242 ns above the 1.7 V threshold, a trip from 322 ns (filter 282 ns); boundary recorded |
| L0-EDG-07 | PWM started before data acquisition (the Safety API starts it unattended), then the control task; raw reading of I1 (`/Adc`) at two CUT DAC levels 200 mV apart | a value at each level, step within 10 % of 400 counts; no fault after 1 s of PWM (the control task took the dispatch over) |
| L0-EDG-08 | `/Fault` case 17 (test hook: no data dispatch): .a while the PWM runs; .b from boot, before the first PWM start | .a "frozen current sensor" (the last value stays), .b "no current reading" (never a value); within 1 s, outputs off, reboot required |

Fresh boots: 01 alone (the arming self-tests run once per boot); 02–06 and 03 after a first arming with both current pins at 0 A; a reboot that must lift "reboot required" (03); 07–08 after a first arming; a reset at the end (08 leaves a self-test fault).

**Repeat-clear rule (decided 2026-11-20, Luiz Villa: reading B).** The time runs from a clear to the next fault: a fault within `CONFIG_OWNTECH_SAFETY_REPEAT_CLEAR_MS` of a clear cannot be cleared, however long one waits, until a reboot (software reset or reset pin; a watchdog reset keeps it). The state is shown to the operator: `SafetyAPI::rebootRequired()`, the fault text ("…; repeat fault after a clear: reboot required"), and `/Safety rRebootRequired` in the test firmware. The first run (before the decision) found the code counting clear-to-clear time: a fault 70 ms after a clear, cleared 0.6 s later, was accepted.

The pulse mode is in Core-MUX (`/Mux/Inj` `wPulseHigh_mV`, `wPulse_ns`, `xPulse`; `mux-injector` `fe04fe3`): the DAC source steps to the pulse level and back, interrupts off, timed on the cycle counter. Its own timing runs long and in steps of about 41 ns, so the time above the threshold is read on the scope. The ADC watchdog on the same pin has no filter and samples every PWM period: it can catch a pulse, which the verdict tells apart by fault input.

Not covered here: a fault during dead time, trip latency (direct probes, C13): `tests/l0/spoc_latency` (L0-LAT).
