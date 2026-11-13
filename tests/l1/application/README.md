# Switched application tests

`test_switched.py` evaluates formal L1-APP criteria using the original switching plant and unchanged Core application. Run it through `tests/run_all.sh --l1`; it reuses that campaign's nominal-grid and lost-VAC raw runs, then executes the declared DCDC scenario files. It does not create an averaged plant or manufacture a sensor trajectory.

Initial conditions, separate simulation loads and the regulation observation window are declared before execution in `../scenarios/dcdc_*.json`. `../plant/fixtures.py` derives explicitly wired load branches and capacitor precharge from the accepted model. The translator initializes floating transformer/rectifier nodes without a fictitious initial winding-voltage mismatch. Capacitor consistency, finite states, actual Core calls once per control period and per-electrical-step physical current extrema are recorded. Stored waveform decimation does not set the startup peak-current criterion.

Startup checks cover the first 50 ms only. The 2 s full startup (`dcdc_startup_full.json`) remains an explicit BLOCKED completion criterion, not a shortened test reported as full-ramp success. Regulation is assessed over the predeclared 80–100 ms window after the 50 ms load step; failure is preserved. Burst tests start above 405 V and check disable/release during real bus discharge. If the bus does not later rise to 405 V, repeated cycling remains unexercised rather than silently passing.

No production gains, current limits, mode sequencing or sensor set changes. These fixtures assess software coupled to the supplied schematic and its assumed actuator parameters; they are not hardware calibration evidence. The grid-frequency finding #1 remains separate.
