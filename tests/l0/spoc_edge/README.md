# L0-EDG · edge cases of the hardware trips

Arming, clearing and reporting at the edges, on the SPOC bench (ADR 0009, issue luizvilla/Core#23, part of C14). `SAFETY_L0_TWIST` on the CUT, wiring as L0-TRP: the CUT's DAC drives I1 (PA1) through CH1 → TP1–TP3 → CH2, the MUX injector holds or drives I2 (PC1) through CH3.

```bash
bash tests/l0/spoc_edge/run.sh --run
```

| ID | Case | Expected |
|---|---|---|
| L0-EDG-01 | first PWM start of a fresh boot with I1 50 mV above its high limit | arming refused, outputs never on |
| L0-EDG-02 | after a trip, PWM off: clear while still above the limit, then at 0 A | refused (-1), fault kept; then accepted (0) |
| L0-EDG-03 | a second fault (from the MUX injector, on I2) about 70 ms after a clear, then a clear about 0.6 s later | INFO: the rule is open (below) |
| L0-EDG-04 | I1, then I2, above their high limits | both comparator fault inputs recorded, outputs off |
| L0-EDG-05 | I1 20 mV inside each threshold for 10 s, PWM on | no trip |
| L0-EDG-06 | pulses on I2 (MUX injector pulse mode: 1.0 V hold to 2.0 V and back), widths 60-420 ns requested, each captured on SPOC CH3 | no comparator trip up to 242 ns above the 1.7 V threshold, a trip from 322 ns (filter 282 ns); boundary recorded |

Two fresh boots: 01 alone (the arming self-tests run once per boot), 02–05 after a first arming with both current pins at 0 A. A reset at the end.

**Repeat-clear rule, open (2026-11-20).** The ADR states it two ways: the decision text says "a repeated clear within N s needs a reboot", which is what `clearFault()` does (it compares the new clear with the previous clear); the implementation note and the Kconfig prompt say "a second fault within N s of a clear". With the code as it is, a fault 70 ms after a clear, cleared 0.6 s later, is accepted. With the 100 ms window of the test build the bench cannot clear twice within the window over ThingSet, so neither reading is judged yet.

The pulse mode is in Core-MUX (`/Mux/Inj` `wPulseHigh_mV`, `wPulse_ns`, `xPulse`; local commit on `mux-injector` after `b500a4e`): the DAC source steps to the pulse level and back, interrupts off, timed on the cycle counter. Its own timing runs long and in steps of about 41 ns, so the time above the threshold is read on the scope. The ADC watchdog on the same pin has no filter and samples every PWM period: it can catch a pulse, which the verdict tells apart by fault input.

Not covered here: a fault during dead time, trip latency (direct probes, C13).
