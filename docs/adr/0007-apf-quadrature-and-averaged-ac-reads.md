---
status: accepted
date: 2026-11-13
---

# APF quadrature for the PLL, the feedforward and the current; averaged AC reads

Decided by Luiz Villa on 2026-11-13, in a design review with Claude (branch `apf_quadrature`, from `20khz_test`). This record keeps the decisions and the reasons given in that review, so the code changes can be traced back to them.

## Context

The grid-following inverter (`singlePhaseInverterFollowing`, fork `luizvilla/single_phase_inverter`) built every quadrature signal with a SOGI:

- PLL: Vα and Vβ from the voltage SOGI (Kr 500 rad/s), Vq to the PLL PI.
- Feedforward: Vdq of that same SOGI.
- Current: Iα and Iβ from a current SOGI (Kr 2000 rad/s). In dq it acts as a low-pass at Kr/2, which limited the current PI (candidate C: Kp 1 V/A, Ti 5 ms).

Each control period read only the newest ADC sample of VAC and IAC (`getLatestValueFast`), although the ADC samples every 20 µs and the control period is 50 µs (about 2.5 samples per period).

Step 2b (key `G`) used an extrapolated copy of VAC, `vac + 1.3·(vac − prev) − vac_dc`, as external feedforward until the relay contact. Step 1d (key `a`) copied the same signal to the bridge with the relay open, and step 1c (keys `+`/`-`) set an offset on the feedforward gain.

## Decision

All-pass filter (APF): first order, `H(s) = (ω₀ − s)/(ω₀ + s)`, discretized with Tustin and prewarped at ω₀. Gain 1 at every frequency, −90° at ω₀. New class `Apf` in the inverter fork, next to `Sogi`. Its ω₀ is the PLL estimate **low-passed with τ = 10 ms** and clamped to ±5 % of ω_ref, the same value the voltage SOGI receives (`w_sogi`), updated every period (see "Found in implementation").

| Signal path | Before | After |
|---|---|---|
| PLL | Vq of Park(SOGI Vα, SOGI Vβ) | Vq of Park(SOGI Vα, **APF(SOGI Vα)**) |
| Voltage feedforward | Vdq of the SOGI | Park(**averaged VAC**, **APF(averaged VAC)**) |
| Current feedback | Park(current SOGI α, β) | Park(**averaged IAC**, **APF(averaged IAC)**) |
| Current PI | Kp 1 V/A, Ti 5 ms, limit 30 V | **Kp 3.5 V/A, Ti 20 ms**, limit 30 V |
| VAC, IAC reads | newest sample | **average of the samples that arrived since the previous control period** (2 or 3) |

Three APF instances run each period: on SOGI Vα (PLL), on the averaged VAC (feedforward) and on the averaged IAC (current). The current SOGI is removed (`setCurrentSogiGain`, `app::inverter_current_sogi_gain`).

The averaged VAC feeds everything that used VAC: the SOGI input, the feedforward, the grid observer (through the SOGI) and the stale-VAC check. The averaged IAC feeds the current loop **and the bench current trip** (|IAC| > 3 A).

The averaging read is a new Spin/shield API call, `SensorsAPI::getAverageValueFast(sensor, dataValid)`, next to `getLatestValueFast`. It averages the DMA samples of the sensor written since its previous call. When none arrived it returns the previous average with `DATA_IS_OLD`. Overrun and empty-slot handling are those of `getLatestValueFast`.

Removed from the application:

- the extrapolated VAC copy as the feedforward of step 2b. Step 2b keeps its PLL freeze, feedforward hold, blanking and connect window, on the new feedforward;
- step 1d (stage 5, key `a`), `vac_follow_advance`, `vac_follow_dc_tau` and the VAC copy state (`vac_dc`, `vac_prev`);
- step 1c (keys `+`/`-`, `vac_offset`, `inverter_vac_offset_*`). The feedforward gain stays at the library default, 1.

Stages 6 (`INV_FOLLOW_CONNECT`) and 7 (`INV_BUS_LOOP`) keep their numbers, so the L1 scenarios and L2 sessions that send them still work. Value 5 is no longer a stage.

## Reasons given

- **Vβ = APF(SOGI Vα), not APF(raw VAC).** An APF passes harmonics and DC with gain 1 and only shifts their phase, so APF(raw VAC) would carry every harmonic into Vβ (3rd harmonic: 1.00, against 0.17 for SOGI Vβ at Kr 500). On SOGI Vα, which is band-pass, the team's simulation showed a clean Vβ, and the PLL angle resolves more easily.
- **APF tuned to the PLL ω.** The APF is −90° only at ω₀. A fixed 50 Hz gives 87.1° at 47.5 Hz: a steady Vq error and a 2ω ripple that the PLL would chase. The PLL estimate is already there, so this costs one coefficient update per period, not an extra filter.
- **Feedforward on the averaged VAC.** With no SOGI in the path, the grid harmonics reach the output, which helps the current loop reject them. Note: inverse-Park(Park(Vα, Vβ, θ), θ + δ) = Vα·cos δ − Vβ·sin δ, so Vβ matters only through the output phase advance δ (0.5 period).
- **Current PI Kp 3.5 V/A, Ti 20 ms.** The team's choice for the best phase margin with a 450 Hz bandwidth. Note from the review: on a pure 500 µH plant the crossover is Kp/L ≈ 1.1 kHz, so the margin is to be confirmed with the full feedback path (averaging, actuation delay, APF on the β channel).
- **APF in the current feedback, accepted.** The α half of Idq responds at once; the β half lags through the APF (group delay 1/ω₀ ≈ 3.2 ms at 50 Hz). Transients therefore show a 2ω coupling between d and q. Accepted and to be checked on L1.
- **Averaging window: samples since the previous period.** Averaging the whole DMA ring (14 VAC samples over 280 µs, 21 IAC samples over 420 µs) would add 140 µs and 210 µs of delay, 23° and 34° at 450 Hz, and different delays for V and I. The samples since the previous period (2 or 3, about 25 µs of delay) keep V and I aligned. The window alternates between 40 µs and 60 µs; the small 10 kHz ripple this causes is accepted and checked on L1.
- **Bench trip on the averaged IAC.** Chosen by the team. It acts up to about 25 µs later than on the newest sample and does not see a single-sample spike; the shield's hardware overcurrent thresholds are unchanged.

## Found in implementation (2026-11-13)

- **Tuning the filters to the instantaneous PLL frequency makes the PLL unstable.** With the APF added, the PLL never locked on a clean 50 Hz grid in L0-GFL: the frequency swung between the ±5 % clamps. When the PLL frequency is above the grid's, the SOGI α and the APF both lead a little, which raises Vq and the frequency further. Loop gain of this path: Kp·V·2/Kr ≈ 0.84 through the SOGI (already there before, stable because below 1) plus Kp·V/(2ω₀) ≈ 0.33 through the APF, 1.17 in total: the PLL's s² coefficient turns negative. A first-order low-pass on the tuning frequency is stable from τ ≈ 0.8 ms (Routh). Measured on the L0-GFL suite: τ = 10 ms passes everything, lock 0.20 s; τ = 3 ms passes, lock 0.32 s; tuning the APF to the fixed ω_ref instead fails L0-GFL-03 at 51 Hz (3.4 V of Vq ripple). **Chosen: τ = 10 ms**, for the SOGI and the three APFs alike (one tuning frequency, two operations per period). This answers review question 6 ("why extra filtering?"): the low-pass is there for stability, not to clean the signal.
- **The PLL angle leads VAC by one sample.** The SOGI α leads its input by about ω·Ts (a known property, [design doc §2](../testing/inverter-current-loop-design.md)), and the PLL locks on it. The feedforward dq of the raw VAC is therefore rotated by −ω·Ts (Vq ≈ −5 V at 50 µs). The bridge still gets VAC back exactly: the inverse Park undoes the Park.
- **L0 results (control period 100 µs in L0-GFL), before → after:** peak instantaneous |Vq| with the bench harmonics 59.3 → 13.5 V, with the EN 50160 limit harmonics 130.6 → 23.9 V, with the −5.7 V offset 0.34 → 0.28 V; |Vq| ripple at 51 Hz 0.039 → 0.003 V; lock from a 1 rad offset 0.18 → 0.20 s.

## Considered options

- **Vβ from APF(raw VAC).** Rejected: harmonics and DC pass unattenuated.
- **Keep the SOGI Vβ for the PLL.** Rejected: the team's simulation favoured APF(SOGI Vα).
- **A fixed-frequency APF.** Rejected: off-nominal phase error (above).
- **Averaging a fixed 3 newest samples, or the whole DMA ring.** Fixed 3 is the fallback if the alternating window shows on L1. The whole ring is rejected (delay).
- **Averaging in the application over successive `getLatestValueFast` reads.** Rejected: it averages across control periods, not across ADC samples.
- **Keep step 1d and step 1c.** Rejected: key `a` is no longer used on the bench.

## Tests

- **L0 (run on this machine):** the APF (gain 1 and −90° at ω₀, follows ω₀, DC gain 1); the averaging read (2 and 3 samples, nothing new, ring wrap, overrun); the existing L0 tests updated for the removed stages and the removed current SOGI.
- **L0-TIM (run on this machine):** the critical task stays within its budget with the averaging reads and three APFs, without the current SOGI.
- **L1 (documented, to be run on another machine):**
  - current step on d and on q with Kp 3.5 V/A, Ti 20 ms, against candidate C: overshoot, settling, 2ω ripple, and the 10 kHz ripple of the alternating averaging window;
  - step 2b connection (stage 6) on the new feedforward.
- **L1, catalogued only (not run, time):** PLL lock time and θ ripple on a distorted grid (3rd and 5th harmonics) with a DC offset, at 47.5, 50 and 52.5 Hz, before against after. The PLL PI gains stay as they are unless this check fails.

## Consequences

- The critical task gains three APF calls, one Park transform and two averaging reads, and loses one SOGI call. L0-TIM must confirm the budget (40 µs, ADR 0005/0006) before a bench session.
- After L0-TIM the board holds the TIMING firmware: flash env USB before the bench.
- The bench procedure loses steps 1c and 1d ([inverter-session.md](../testing/bench/inverter-session.md)).
- `getAverageValueFast` is a new Spin API call, like `getLatestValueFast`: to be reviewed with Jean Alinei before it goes upstream.
