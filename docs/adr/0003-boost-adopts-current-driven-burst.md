---
status: accepted
date: 2026-11-06
---

# The boost object adopts Andres Vazquez Sieber's control: current-driven burst, no reset at burst, his PI gains

Andres Vazquez Sieber's boost controller (`core-micro-verter-boost`, commit `43e249e`) was tested on the board on 2026-11-06 ([bench evidence](../testing/bench/2026-11-06_andres-boost/README.md)) and is the one to ship. Luiz Villa decided to amend Core's `BoostConverter` to take its control approach, keeping Core's structure: the modes and the leg split. The decision was taken after finding #3, where Core's boost restarts its current loop from duty 0.05 at every mode change and every burst pause, and needs over 100 ms to reach the duty that delivers power.

## What changes

| | Before (Core today) | After |
|---|---|---|
| Light load | Voltage hysteresis: pause both legs at ≥ 405 V, resume at ≤ 395 V | Current-driven pulse skipping (Andres): when the current loop sits at minimum duty with the current above its reference, the burst duty (skipped PWM periods out of a burst period) goes up one step per tick; when the current is below its reference, it goes down. The boost object computes the burst command; the application applies it with the HRTIM burst mode |
| Controller state at burst | All PIDs reset on every pause | Kept across burst ↔ continuous transitions |
| PI gains | Voltage Kp 0.05 A/V, Ti 100 ms; current Kp 0.000215, Ti 75.2 µs | Andres's: current Kp 0.002712, Ti 341.3 µs; voltage Kp = 0.433·Vbus/Vin (scheduled with the measured Vin, clamped 24–48 V), Ti 4.57 ms |
| STARTUP | Bus reference ramp at 200 V/s, current ceiling 1 A, leg 1 only | BUSREG toward 400 V with a small current clamp (value to confirm, Andres's bench used 0.25 A); switching to BUSREG resets nothing |
| BUSREG / MPPT current | Total reference bounded 0–2 A, leg 1 up to 1 A | Bounded at the rated input current (value to confirm, about 11 A for 400 W at 36 V); leg 1 first, leg 2 takes the excess |
| Burst and legs | Both legs paused | Burst acts on leg 1 independently of leg 2 demand (amended 2026-11-09) |
| Over-voltage / over-current | Burst pause at 405 V | Andres's thresholds: full burst above 440 V; stop above 460 V or 8 A per leg |

Kept: the modes IDLE / STARTUP / BUSREG / MPPT as the application's interface, and the leg split. Not adopted initially: the 100 Hz PR on the current loop (in Andres's code it was always on, which looked unintended; to revisit for grid-connected operation).

Update, 2026-11-09: adopt the 10 Hz voltage-loop tuning from Andres's `b37b3d4`
(`Kp = (0.24773/6) · VBusRef/VLow`, `Ti = 34.13 ms`), and the optional 100 Hz
notch/resonant paths from `5bcfdf6`. The notch runs at 10 kHz with a 10 Hz
bandwidth, on voltage-loop feedback only; burst and trip decisions keep the raw
bus measurement. Both boost legs receive a parallel pure resonator (`Kr = 0.88`
at nominal 400 V, phase `0.09425` rad), with the summed duty clamped to the
existing limits. Resonator state is held with the PI during burst and cleared
when its leg is disabled or control is reset. Both optional paths default OFF;
serial `n` toggles the notch and `k` toggles the resonators (`m` is already MPPT).
The existing voltage-only gain schedule and operating modes are retained; this
does not import PBALANCE or add grid injection. Signal-response and lifecycle
checks are L0-BST-10 and L0-APP-06; hardware validation remains outstanding.

## Considered options

- **Keep Core's boost and retune it** (L1-BST-09 compares tunings): rejected as the shipping path; Andres's controller is already validated on the bench, and the restart behaviour of #3 is structural, not a tuning problem.
- **Ship Andres's `main.cpp` as is**: rejected; it is a bench program (serial menu, one leg at a time, gains changed only in IDLE, a modified control library with `setKp`), and it would lose the tests and the application structure built around `BoostConverter`.
- **Burst by software pulse skipping** (enable/disable the legs per 100 µs tick): rejected; 20 PWM periods of granularity, where the HRTIM burst mode skips single periods.

## 2026-11-09: restore signed voltage demand for burst regulation

Andres's `43e249e` and `5bcfdf6` keep symmetric voltage-PI output bounds.
Clamping that internal demand to zero during the port removed the signal that
increases skipping when bus voltage is above target and measured current is zero.
The voltage PI now retains signed demand internally for leg 1's current PI and
burst comparisons, while total and per-leg delivered current references stay
nonnegative. MPPT and direct current commands do not inherit negative voltage
demand. Active burst starts at minimum pulse duty, as in Andres's code; IDLE and
trip still clear the outputs. The application initializes burst on entry and
stops and disables it on exit, rather than only stopping the burst operation.

Host regressions cover zero-current suppression, recovery below target, entry
from elevated duty, mode isolation and startup duty. Physical regulation still
requires a hardware check.

## Consequences

### 2026-11-09 amendment: independent burst and register update order

Remove both leg-2 reference checks from leg 1 burst control. A positive residual
reference, even one float step, must neither cancel skipping nor block entry.
Leg 1 now follows its own current error at minimum duty regardless of leg 2.
This supersedes the earlier leg-2 guard; leg-2 enable-threshold finding #5 remains
separate.

With preload enabled, `hrtim_burst_set()` must write BMPER before BMCMP.
[RM0440 section 28.3.15, page 960](https://www.st.com/resource/en/reference_manual/rm0440-stm32g4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
states that writing BMPER blocks update transfer until BMCMP is written. The old
compare-first/period-last order could leave the hardware pattern unchanged while
the controller updated the software skip count. L0-BST-11.a executes the actual
driver function against this preload handshake; L0-BST-06.d covers independent
burst with one-step, 1 mA and 2.5 A leg-2 references in BUSREG and MPPT.

- Production change under the procedure's rules: L0 tests written first and failing (new burst, no reset, gains, protection), then the change with a worklog entry, then L1, then a hardware check against the bench captures above.
- The contract becomes **app-v2**: a burst command (skipped periods, burst period) joins the outputs, and the L1 PWM adapter models the HRTIM burst mode. The ADC sampling instant fix from #3 (L1-MOD-08.b) goes into the same version.
- Tests tied to the old behaviour are retired or rewritten: L0-BST-05 (ramp), L0-BST-06 (1 A ceiling), L0-BST-07 (voltage burst), L1-APP-02 and L1-APP-04 and their L1-BST counterparts. ADR 0002's argument for the 200 V/s ramp (a 2 s start-up keeps L1 runs affordable) is replaced by the current clamp, which sets the start-up time instead.
- The current-clamp values (STARTUP, rated) are to be confirmed with Andres before the change is merged.
