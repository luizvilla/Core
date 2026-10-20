#!/bin/sh
# Builds hrtim.c for the host with the real STM32 LL headers, HRTIM1 and
# RCC mapped in RAM at their real addresses, and runs the tests.
# usage: tests/hrtim_host/run.sh   (from the repository root, Linux x86_64)
set -e
D=$(cd "$(dirname "$0")" && pwd)
CORE=$(cd "$D/../.." && pwd)
HAL=${HAL:-$HOME/.platformio/packages/framework-zephyr/_pio/modules/hal}
OUT=$(mktemp -d)
FAIL=0
for T in "$D"/test_*.c; do
    N=$(basename "$T" .c)
    gcc -std=gnu11 -O0 -g -w -include "$D/shim/host_cmsis.h" \
        -DSTM32G474xx -DUSE_FULL_LL_DRIVER -DCONFIG_SOC_SERIES_STM32G4X \
        -DCONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC=170000000 \
        -I"$D/shim" -I"$D" -I"$CORE/zephyr/modules/owntech_hrtim_driver/zephyr/public_api" \
        -I"$HAL/stm32/stm32cube/common_ll/include" \
        -I"$HAL/stm32/stm32cube/stm32g4xx/drivers/include" \
        -I"$HAL/stm32/stm32cube/stm32g4xx/soc" \
        -I"$HAL/cmsis/CMSIS/Core/Include" \
        "$CORE/zephyr/modules/owntech_hrtim_driver/zephyr/src/hrtim.c" \
        "$D/shim/ll_gpio_stub.c" "$T" -o "$OUT/$N"
    echo "== $N"
    "$OUT/$N" || FAIL=1
done
rm -rf "$OUT"
exit $FAIL
