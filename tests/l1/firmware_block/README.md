# Native application firmware block

`wrapper.cpp` implements Pulsim's C-Block ABI and calls `controlInit` once per independent context and `controlStep` exactly once per 100 us. Its input/output index enums are in `contract.h` and documented by `../contract.json`.

Compile from Core with `tests/l1/firmware_block/build.sh`. Core sources and the wrapper use C++20 (matching the firmware and control_library), `-O1 -g -fPIC -Wall -Wextra -Werror` and UBSan without recovery. Library sources use the same flags without `-Werror`. No production source is copied or changed. The resolved inverter override follows L0's `SINGLE_PHASE_INVERTER_DIR` convention. Source/dependency hashes, revisions and the binary hash are in `build/manifest.json`; build artifacts are ignored.

The wrapper only translates buffers, holds the last delivered VAC at the sensor boundary, owns the native context and logs state. It contains no alternate application sequence, tuning or hardware PWM generation. Bridge and relay requests are read from the real application/library for assertions; both remain off in this application.

Input dimensions, output dimensions, enums, Boolean commands and control cadence are checked before application execution. A bad call makes the context inert and reports `block_error`. Direct sensor behavior remains Core's behavior. Teardown is explicit and idempotent through Pulsim's handle; new runs allocate new state.

`../integration/test_wrapper.cpp` compares all eight returned actuator/action fields with direct calls to unchanged `controlStep`, checks stale-VAC state/cold-loop phase, independent contexts and invalid ABI/timing. `../integration/test_rc.py` proves the exported symbol executes through Pulsim itself, closes an electrical RC loop and resets independently. See the procedure's L1-INT cases; these are not application performance claims.
