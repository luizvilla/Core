# L0-CPU · the protection with the CPU dead

IEC 62109-1 Annex B route (a), ADR 0009: each protection must work with the CPU dead or the software disabled. Two parts on the SPOC bench (issue luizvilla/Core#23, C12).

```bash
bash tests/l0/spoc_dead/run.sh --run                              # both parts, a campaign of its own
.venv/bin/python tests/l0/spoc_dead/run.py --out /tmp/x --part 1  # development run of one part
```

The CUT ends on `SAFETY_L0_TWIST`, the safety test firmware it keeps between campaigns.

## L0-CPU-01 · lockup, per PWM pin (also L0-SUP-08.c)

`SAFETY_SUP` (`tests/firmware/safety_supervision`) on the CUT: a bare Spin with no trips, PWM on units A, C, D, E, F at 200 kHz, 50 %. Its lockup case sets FAULTMASK and faults; PA6 (Spin 34) rises a few cycles before, as a marker. For each of the ten PWM pins:
- PA6 is routed to SPOC CH1 and the pin to CH3 (the injector off). This works with or without TP5-TP7;
- the scope is armed on one single capture (CH1 rising at 2.5 V, NORMal sweep: PWM crosstalk on CH1 reaches about 1.9 V at a lower level), 2 µs/div;
- verdict: the marker reaches its high level, the pin switches before it, and the pin is low and stays low within 1 µs of it.

The supervision firmware is driven over SWD (`g_safety_sup_cmd`), as in L0-SUP; each lockup ends in a watchdog reset.

## L0-CPU-02 · trips with the CPU halted

`SAFETY_L0_TWIST` on the CUT, set up as L0-TRP: arming with the current pins held at 0 A, routes and start level per case. Then, in one SWD session:
1. halt the CPU; the watchdog is frozen in debug (DBGMCU), the HRTIM keeps running;
2. read the PC, the output enables (OENR) and the fault flags (HRTIM ISR);
3. write DAC2 DHR12R1 to 50 mV past the threshold: the DAC changes with no instruction running;
4. read the PC again, OENR, the fault flags and the PWM pins (GPIO IDR); resume.

Verdict: the PC is unchanged, the outputs were on before, the path's own FLT flag is set, OENR is cleared and every PWM pin is low. Software paths are not tested here: they need the CPU. Cases that need two held pins wait for TP5-TP7 (L0-TRP README).

## Findings

- 2026-11-20: without a hardware trip, no timer listened to a fault channel, and the HRTIM system fault did nothing. Every PWM pin kept switching after a lockup until the watchdog reset (about 1 ms). The Safety API now enables a sentinel fault channel in that case (FLT1 from its pin, active high; it never trips). With it, all ten pins stop within 25 ns of the marker.
- The halting-debug state left by the ST-Link (DHCSR.C_DEBUGEN) was ruled out: a lockup on a power-cycled CUT behaved the same.

History: [docs/safety/spoc_bench.md](../../../docs/safety/spoc_bench.md).
