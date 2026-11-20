# L0-ADC: offset of the CUT ADC at the low end (SPOC bench)

Issue luizvilla/Core#23. On 2026-11-20 the CUT's ADC read about 14 mV low near 300 mV (286 mV with 302 mV on the pin, `docs/safety/spoc_bench.md`). That makes the ADC-watchdog low bounds trip early and the software low bounds trip late (L0-TRP-03.ownverter.V_HIGH.low, +29 mV against ±25 mV). This test finds out which of two causes is behind it:
- the bench's source impedance (SPOC mux and node) against the production sampling time, 12.5 cycles at 42.5 MHz (294 ns, `adc_core.c`);
- an offset or gain error of the ADC itself.

## Method

The CUT's DAC (DAC2_CH1 on PA6) and its ADCs share the VREFBUF. An ideal pair reads back its own code: the DAC outputs VREF × code / 4096, and the ADC returns Vin / VREF × 4096. So the error is **ADC counts − DAC code**, whatever VREF's true value. The scope's CH1 and CH2 readings are recorded as information only.

| Case | Point |
|---|---|
| 00 | VREF+ from VREFINT, against the 2048 mV VREFBUF |
| 01 | PA6 itself on ADC2 IN3: the DAC pin, no SPOC path |
| 02 | Twist I1 (PA1) and V_HIGH (PC3) through CH1 → TP1–TP3 → CH2, on each ADC that reads them |
| 03 | the same pins on ADC1, converted right after VREFINT: memory of the previous channel |
| 04 | the production reading of V_HIGH (PC3; regular group, DMA, `peekAdcRaw()`), armed, 100–1500 mV, PWM outputs on, then off: statistics of the newest sample each task period over 1 s (`/Adc xStat`) |
| 05 | the same with the outputs on, ADC1's trigger moved to the first leg's CMP3 (`/Pwm wTrig`) and swept over the period, at 100 and 1000 mV; then the pin node on CH2 over two periods |
| 06 | armed, injected conversions with SMPR kept (`/AdcJ wSmp 255`) at instants unrelated to the PWM: the V_HIGH pin and the DAC pin, outputs on, then off |

01–03 cover levels from 100 to 1900 mV and each sampling time, SMPR codes 0–7 (2.5 to 640.5 cycles). They use `/AdcJ` in `tests/firmware/safety_l0`: software-triggered injected conversions, 256 per point.
- **Why a fresh boot:** SMPR can only change while the regular group is stopped, so the probe runs before data acquisition starts.
- **What `/AdcJ` does to the ADC:** it wakes, calibrates and enables the ADC as the driver does, then leaves it disabled, with SMPR and JSQR restored.

How to read the result:
- An error that shrinks as the sampling time grows points to the source impedance.
- An error that stays flat, and also shows on PA6, belongs to the ADC itself.

This is a characterisation: the results are INFO lines, with no verdict.

## Result (2026-11-20)

The ADC reads 10 counts (5 mV) below the DAC code, flat against level, sampling time and path. The large low-end error came from the sample instant: as found, the ADCs are triggered by the HRTIM master period, which is a switching edge, and each sample lands in the transient the edge puts on the pin. Details: `docs/safety/spoc_bench.md`, 2026-11-20.

## Run

```
bash tests/l0/spoc_adc/run.sh --run
```

It flashes `SAFETY_L0_TWIST`, which stays on the CUT afterwards. The wiring is the same as for L0-TRP.
