#!/bin/sh
# Builds src/main.cpp for the host with stubbed OwnTech APIs and the real
# single_phase_inverter / control_library, and runs the sequencing tests.
# usage: tests/host_firmware/run.sh [single_phase_inverter] [control_library]
set -e
D=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$D/../.." && pwd)
SPI=$(cd "${1:-$ROOT/owntech/lib/USB/single_phase_inverter}" && pwd)
CL=$(cd "${2:-$ROOT/owntech/lib/USB/control_library}" && pwd)
OUT=$(mktemp -d)
g++ -std=c++2a -O0 -g -Wall -Wno-unused-variable -Wno-unused-function \
    -I"$D/shim" -I"$ROOT/src" -I"$SPI/src" -I"$CL/src" \
    "$CL/src/trigo.cpp" "$CL/src/transform.cpp" "$CL/src/pid.cpp" \
    "$SPI/src/sogi.cpp" "$SPI/src/singlePhaseInverter.cpp" \
    "$D/test_firmware.cpp" -lm -o "$OUT/test_firmware"
"$OUT/test_firmware"; R=$?
rm -rf "$OUT"
exit $R
