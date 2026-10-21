# HRTIM host tests

Register-level tests of `hrtim.c` on a Linux x86_64 host, without a board.

The real STM32G4 LL headers are used. HRTIM1 and RCC are RAM pages mapped
at their real addresses (`mmap` at 0x40016000 and 0x40021000), so the LL
functions read and write them unchanged, and the tests check the register
values (PERxR, RSTxR, CMPxR, CR2…) after each driver call.

```
tests/hrtim_host/run.sh
```

`HAL` can point to another Zephyr `modules/hal` directory (default: the
PlatformIO `framework-zephyr` package).

What this does not cover: anything that depends on the hardware running
(counters, DLL, outputs, ADC triggers firing), and the Spin / shield C++
layers (checked by building the firmware).

| Test | Checks |
|---|---|
| test_c1 | master period rounding in `hrtim_change_frequency()` |
| test_c2 | per-unit frequency at init, master paths leave the unit alone, second init |
| test_c3 | units with their own frequency are not reset by the master, phase shift / current mode refused |
| test_c4 | runtime frequency change of one unit, CMP1 / CMP3 rescaled |
| test_c5 | synchronous counter reset (one CR2 write) |
| test_c6 | inverter legs in up-down mode: buck (timer D) and boost (timer E) only differ by the swap bit, dead time on both, re-init clears the swap |
| test_seq | the `PowerAPI::initMode()` call order for the micro-verter legs |
