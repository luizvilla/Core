# L0-CAL · SPOC bench calibration

Checks the SPOC bench before any L0 safety case is judged on it (ADR 0009, issue luizvilla/Core#26), and records the four channels' gain and skew for the campaign (`calibration.json` in the test-case folder).

```bash
bash tests/common/spoc_preflight.sh                       # CUT, MUX and scope answer
.venv/bin/python tests/l0/spoc_cal/trim.py                # compensation of each channel, routed alone
.venv/bin/python tests/l0/spoc_cal/trim.py --live --ch 3  # live reading while turning CH3's trimmer
bash tests/l0/spoc_cal/run.sh --run                       # the calibration, as a campaign of its own
```

## Bench

| Item | Setting |
|---|---|
| CUT Spin | Core-CUT (SPOC_Software `Core-CUT`), USB 2fe3:01ff; drives PA8 (Spin pin 12) at 1 kHz, 0.5 |
| MUX Spin | Core-MUX with the injector (SPOC_Software `mux-injector`, `b500a4e`), USB 2fe3:0100 |
| Scope | Rigol MHO984 on USB, ×10 probe setting on all four channels (9 MΩ divider into 1 MΩ) |
| Cables | **30 cm BNC.** With 1 m cables the scope's own FREQuency and PDUTy missed edges |
| Injection wiring | TP1–TP3 (self-injection); MUX Spin pins 2 and 4 to TP5 through their resistors |

While TP1 and TP3 are wired, SPOC CH1 and CH2 share one node: routing either shows the signal on both scope channels. **Never route two different CUT outputs to CH1 and CH2 while the wire is in** (it would short them), except the intended self-injection source and target.

`MuxController.route()` disables every other route first; use `route_pair()` for two channels at once.

## Criteria

| ID | What | Limit |
|---|---|---|
| L0-CAL-01 | CUT, MUX and scope answer | — |
| L0-CAL-02.a–d | divider compensation per channel, routed alone (Core4 `compensation_pct`, from the waveform) | 95–105 % |
| L0-CAL-03.a–d | scope FREQuency, median and spread of 5 | 1000 Hz ±0.1 %, spread ≤ 0.1 % |
| L0-CAL-04.a–d | scope PDUTy, median and spread of 5 | 0.5 ±0.005, spread ≤ 0.005 |
| L0-CAL-05.a–d | VTOP − VBASe, level spreads | 3.3 V ±5 %, ≤ 50 mV |
| L0-CAL-06.a–d | gain against CH1 | INFO, recorded |
| L0-CAL-07/08.b–d | skew CH1 → CHn, rising / falling: 50 % crossings from 5 single captures at 20 ns/div | INFO, recorded |

The skew comes from the waveforms because the scope's RRDelay read invalid and FFDelay a constant −0.2 ns on this bench. Samples are 1 ns apart (1 GSa/s): sub-nanosecond skews are interpolated. Absolute gain against a reference instrument is not covered.

History and findings: [docs/safety/spoc_bench.md](../../../docs/safety/spoc_bench.md).
