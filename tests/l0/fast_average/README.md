# L0-FAV: averaging read of the DMA buffer

Host tests of the arithmetic behind `SensorsAPI::getAverageValueFast` ([ADR 0007](../../../docs/adr/0007-apf-quadrature-and-averaged-ac-reads.md)): the average of a sensor's samples written in the ADC's circular DMA buffer since the previous read. The code under test is `data_fast_sum_new_samples()` in [data_fast_average.h](../../../zephyr/modules/owntech_spin_api/zephyr/src/data/data_fast_average.h). It has no hardware access, so the test plays the DMA. The CNDTR read, the overrun handling and the conversion `raw × gain + offset` are in `Sensors.cpp` and are exercised on the target by L0-TIM.

## Run

```bash
bash tests/l0/fast_average/run.sh
tests/run_all.sh --l0
```

## What is compiled

| Source | Flags |
|---|---|
| `test_fast_average.cpp` (includes `data_fast_average.h`) | strict (`-Werror`) |

## How the test plays the DMA

A buffer of size S holds a sequence of n channels: slot i always holds rank (i mod n) + 1. Conversions are written one by one. The newest slot of the rank comes from CNDTR through the same table as `DataAPI::prepareFastChannel`. Reads come every 2.5 sequences, as the Core does: one ADC sequence every 20 µs and one control period every 50 µs. They are started at every phase within a sequence. Each value converted is distinct, so a wrong slot shows in the average.

## Criteria

| Test case | Criteria | Checks |
|---|---|---|
| L0-FAV-01 | .a, .b | ADC1 (S 42, 3 channels, VAC rank 2) and ADC2 (S 42, 2 channels, IAC rank 2): over 1000 reads at every phase, the count equals the samples the rank received and the average equals their mean; reads of 2 and of 3 samples both occur, no other count |
| L0-FAV-02 | .a, .b, .c | Nothing new (also after a conversion of another rank): no sample; never-written slots skipped, only empty slots give none; the largest gap (one sequence short of a lap) sums S/n − 1 samples |
