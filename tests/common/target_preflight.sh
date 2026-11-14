#!/usr/bin/env bash
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file target_preflight.sh
## @brief Checks everything the on-target tests (bare Spin + ST-Link) need, and says how to fix what is missing.
##
## One line per item: "OK <item>" or "MISSING <item>: <why>. Fix: <command>". Written to be read by a person
## or an AI assistant; --install runs the fixes that need no administrator rights (PlatformIO packages and the
## MCUboot bootloader). Fixes that need sudo (udev rules) are printed, never run.
## @return 0 when every item is OK.
##
## @par Usage
##   bash tests/common/target_preflight.sh [--install] [--env=SAFETY_SUP]
repo=$(cd "$(dirname "$0")/../.." && pwd)
install=0
env=SAFETY_SUP
for arg in "$@"; do
    case "$arg" in
        --install) install=1 ;;
        --env=*) env=${arg#--env=} ;;
    esac
done
PIO=${PIO:-$HOME/.platformio/penv/bin/pio}
PKG=${PLATFORMIO_PACKAGES_DIR:-$HOME/.platformio/packages}
OPENOCD="$PKG/tool-openocd/bin/openocd"
OCD_ARGS=(-d0 -s "$PKG/tool-openocd/openocd/scripts" -f interface/stlink.cfg -f target/stm32g4x.cfg)
NM="$PKG/toolchain-gccarmnoneeabi/bin/arm-none-eabi-nm"
missing=0
ok() { echo "OK $1"; }
miss() { echo "MISSING $1: $2. Fix: $3"; missing=$((missing + 1)); }

# 1. Python 3 (reader and evaluator, standard library only).
if python3 -c 'import sys; sys.exit(sys.version_info < (3, 8))' 2>/dev/null; then ok "python3 >= 3.8"
else miss "python3 >= 3.8" "not found" "install python3 with the system package manager"; fi

# 2. PlatformIO Core, the user install the project uses (not a distribution package).
if [ -x "$PIO" ]; then ok "PlatformIO Core ($PIO)"
else
    miss "PlatformIO Core" "$PIO not found" \
        "curl -fsSL -o /tmp/get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py && python3 /tmp/get-platformio.py"
    echo "STOP: the remaining items need PlatformIO"; exit 1
fi

# 3. OpenOCD and the ARM toolchain: PlatformIO packages of the test env (owntech/pio_extra.ini platform_packages).
if [ ! -x "$OPENOCD" ] || [ ! -x "$NM" ]; then
    if [ $install -eq 1 ]; then
        echo "INSTALL PlatformIO packages for env $env"
        (cd "$repo" && "$PIO" pkg install -e $env) >/dev/null 2>&1
    fi
fi
if [ -x "$OPENOCD" ]; then ok "OpenOCD ($("$OPENOCD" --version 2>&1 | head -1))"
else miss "OpenOCD (PlatformIO tool-openocd)" "$OPENOCD not found" "cd $repo && $PIO pkg install -e $env   (or run this script with --install)"; fi
if [ -x "$NM" ]; then ok "arm-none-eabi-nm"
else miss "ARM toolchain (toolchain-gccarmnoneeabi)" "$NM not found" "cd $repo && $PIO pkg install -e $env   (or --install)"; fi

# 4. ST-Link on USB (V2, V2-1, V3 product ids).
if command -v lsusb >/dev/null; then
    if lsusb | grep -qiE '0483:(3748|374b|374d|374e|374f|3752|3753|3754)'; then ok "ST-Link on USB ($(lsusb | grep -ioE 'ST-?LINK[-A-Z0-9]*' | head -1))"
    else miss "ST-Link on USB" "no ST-Link probe (vendor 0483) seen by lsusb" "plug the ST-Link into this computer and its SWD cable into the Spin's SWD header"; fi
else echo "SKIP ST-Link on USB: lsusb not installed (usbutils); OpenOCD check below decides"; fi

# 5. OpenOCD reaches the target: USB permissions (udev), probe firmware, Spin powered and wired.
if [ -x "$OPENOCD" ]; then
    out=$("$OPENOCD" "${OCD_ARGS[@]}" -c init -c "mdw 0x08000000" -c shutdown 2>&1)
    if echo "$out" | grep -q 'LIBUSB_ERROR_ACCESS'; then
        miss "USB access to the ST-Link" "permission denied (udev rules missing)" \
            "curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules && sudo udevadm control --reload-rules && sudo udevadm trigger   (then unplug and replug the ST-Link; needs administrator rights)"
    elif echo "$out" | grep -qiE 'open failed|no device found|unable to open'; then
        miss "ST-Link opened by OpenOCD" "OpenOCD found no usable probe" "check the USB cable; update the probe firmware with STM32CubeProgrammer if OpenOCD reports an old version"
    elif ! echo "$out" | grep -q '^0x08000000:'; then
        miss "Spin reached over SWD" "probe opens but the STM32G4 does not answer" "power the Spin (its USB-C, or the shield), check the SWD cable (SWDIO, SWCLK, GND, 3V3 reference)"
    else
        ok "Spin reached over SWD"
        word=$(echo "$out" | awk '/^0x08000000:/{print $2}')
        if [ "$word" = ffffffff ]; then
            if [ $install -eq 1 ]; then
                echo "INSTALL MCUboot bootloader v1.1.0 at 0x08000000"
                mkdir -p "$repo/owntech/bootloader"
                curl -fsSL -o "$repo/owntech/bootloader/zephyr.bin" https://github.com/owntech-foundation/bootloader/releases/download/v1.1.0/zephyr.bin &&
                    "$OPENOCD" "${OCD_ARGS[@]}" -c "program $repo/owntech/bootloader/zephyr.bin 0x8000000 verify reset exit" >/dev/null 2>&1 &&
                    ok "MCUboot bootloader (just installed)" ||
                    miss "MCUboot bootloader" "install failed" "see the OpenOCD command in this script"
            else
                miss "MCUboot bootloader" "flash at 0x08000000 is empty; the test image needs MCUboot" "bash tests/common/target_preflight.sh --install   (downloads owntech-foundation/bootloader v1.1.0 and flashes it over the ST-Link)"
            fi
        else ok "MCUboot bootloader present (0x08000000 = 0x$word)"; fi
    fi
fi

[ $missing -eq 0 ] && echo "PREFLIGHT OK" || echo "PREFLIGHT: $missing item(s) missing"
exit $((missing > 0))
