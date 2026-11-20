#!/usr/bin/env bash
# What the SPOC bench (docs/safety/spoc_bench.md) needs, one OK/MISSING line each.
# SPOC_ROLES="mux" skips the CUT, for a test that flashes it itself (its ST-Link: target_preflight.sh).
cd "$(dirname "$0")"
py=python3
ok() { echo "OK       $1"; }
miss() { echo "MISSING  $1  -> $2"; fail=1; }
$py -c "import serial" 2>/dev/null && ok "pyserial" || miss "pyserial" "pip install pyserial"
for role in ${SPOC_ROLES:-mux cut}; do
  p=$($py -c "from spoc.boards import find; print(find('$role') or '')")
  [ -n "$p" ] && ok "$role Spin shell on $p" || miss "$role Spin (2fe3:$([ $role = mux ] && echo 0100 or 0102 || echo 01ff))" "plug its USB, check it runs Core-${role^^}$([ $role = cut ] && echo ' or the safety test firmware')"
done
tmc=$($py -c "import rigol_mho as r; print(r.scope_path())")
if [ ! -e "$tmc" ]; then miss "scope $tmc" "plug the MHO984 USB"
elif [ ! -w "$tmc" ]; then miss "scope write access" "sudo cp tests/common/udev/99-rigol-usbtmc.rules /etc/udev/rules.d/ && sudo udevadm control --reload && sudo udevadm trigger, then replug"
else
  idn=$(timeout 40 $py -c "import rigol_mho as r; s=r.RigolScope(r.open_transport()); print(s.idn())" 2>/dev/null)
  [ -n "$idn" ] && ok "scope on $tmc: $idn" || miss "scope answers *IDN?" "replug the scope's USB cable (usbtmc out of step)"
fi
exit ${fail:-0}
