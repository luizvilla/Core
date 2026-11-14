#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file protection_manifest.py
## @brief Build step of ADR 0009 (issue luizvilla/Core#17): reads Zephyr's parsed devicetree (edt.pickle), runs
##        the protection allocation and writes the protection manifest, the board's protection table and a
##        generated header.
##
## Exit status 1 when a limit has no hardware path and no exception: the build stops. Warnings are printed one
## per line, prefixed "WARNING: ", for CMake to show. Called from the safety module's CMakeLists.txt.
##
## @par Usage
##   protection_manifest.py --edt build/zephyr/edt.pickle --zephyr-base $ZEPHYR_BASE --out-dir build/zephyr/safety
##                          [--header include/generated/safety_protection_plan.h] [--report-only]
import argparse
import json
import os
import pickle
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from safety_allocation import allocate, markdown_table, path_text  # noqa: E402


def f32(bits):
    """IEEE 754 float stored as an int devicetree property."""
    return struct.unpack('<f', struct.pack('<I', bits & 0xFFFFFFFF))[0]


def prop(node, name, default=None):
    p = node.props.get(name)
    return default if p is None else p.val


def spin_pin_map(edt):
    """Spin header pin -> MCU pin ('PA1'), from the spin-header connector's gpio-map."""
    out = {}
    for node in edt.compat2nodes.get('spin-header', []):
        raw = node._node.props.get('gpio-map')
        if raw is None:
            continue
        # Raw big-endian cells; dtlib has already resolved the &gpioX references to phandle numbers.
        cells = struct.unpack('>%dI' % (len(raw.value) // 4), raw.value)
        for i in range(0, len(cells) - 4, 5):
            pin, _, phandle, gpio_pin, _ = cells[i:i + 5]
            ctrl = node._node.dt.phandle2node.get(phandle)
            label = (ctrl.labels[0] if ctrl is not None and ctrl.labels else '')
            if label.startswith('gpio') and len(label) == 5:
                out[pin] = f'P{label[4].upper()}{gpio_pin}'
    return out


def build_model(edt):
    pins = spin_pin_map(edt)
    sensors = {}
    for node in edt.compat2okay.get('shield-sensors', []):
        name = prop(node, 'sensor-name')
        chans = []
        for child in node.children.values():
            ioc = prop(child, 'io-channels') or []
            sp = prop(child, 'spin-pin')
            for c in ioc:
                label = c.controller.labels[0] if c.controller.labels else c.controller.name
                chans.append({'adc': label, 'channel': c.data.get('input'), 'spin_pin': sp,
                              'mcu_pin': pins.get(sp)})
        gain, offset = prop(node, 'default-gain'), prop(node, 'default-offset')
        dadc = prop(node, 'default-adc', 'UNKNOWN_ADC')
        sensors[name] = {'default_adc': None if dadc in (None, 'UNKNOWN_ADC') else 'adc' + dadc[-1],
                         'gain': f32(gain) if gain is not None else None,
                         'offset': f32(offset) if offset is not None else 0.0,
                         'conv': prop(node, 'sensor-conv-type', 'LINEAR'), 'channels': chans}
    limits = []
    for node in edt.compat2okay.get('safety-thresholds', []):
        limits.append({'sensor': prop(node, 'sensor-name'), 'name': prop(node, 'threshold-name'),
                       'high': f32(prop(node, 'threshold-high')), 'low': f32(prop(node, 'threshold-low')),
                       'protection': prop(node, 'protection', 'hardware'), 'exception': prop(node, 'exception'),
                       'comparator_bound': prop(node, 'comparator-bound', 'high'),
                       'priority': prop(node, 'priority'),
                       'hardware_bounds': prop(node, 'hardware-bounds'),
                       'idle_band': f32(prop(node, 'idle-band')) if prop(node, 'idle-band') is not None else None})
    by_pin = {ch['spin_pin']: n for n, s in sensors.items() for ch in s['channels']}
    legs = []
    for node in edt.compat2okay.get('power-leg', []):
        for child in node.children.values():
            pw = prop(child, 'pwms') or []
            timer = pw[0].controller.labels[0] if pw and pw[0].controller.labels else None
            cur = prop(child, 'current-pin-num')
            legs.append({'name': prop(child, 'leg-name'), 'timer': timer,
                         'pwm_x1_high': prop(child, 'pwm-x1-high', 1),
                         'safe_state': prop(child, 'safe-state', 'open'),
                         'current_sensor': by_pin.get(cur), 'short_risk_accepted': prop(child, 'short-risk-accepted')})
    outputs = []
    for node in edt.compat2okay.get('safety-outputs', []):
        for child in node.children.values():
            g = (prop(child, 'gpios') or [None])[0]
            # edtlib resolves the Spin header's gpio-map: the controller is already the MCU port (gpiob) and the
            # cell its pin. A spin-header controller (unresolved) would carry the Spin pin instead.
            label = (g.controller.labels[0] if g is not None and g.controller.labels else '')
            cell = g.data.get('pin') if g is not None else None
            if label.startswith('gpio') and len(label) == 5 and cell is not None:
                mcu = f'P{label[4].upper()}{cell}'
                sp = next((k for k, v in pins.items() if v == mcu), None)
            else:
                sp, mcu = cell, pins.get(cell)
            outputs.append({'name': prop(child, 'output-name', child.name), 'spin_pin': sp, 'mcu_pin': mcu,
                            'safe_level': prop(child, 'safe-level', 0),
                            'reset_state_safe': bool(prop(child, 'reset-state-safe', False))})
    shield = None
    for node in edt.compat2okay.get('shield-pcb', []):
        shield = f'{prop(node, "shield-name")} {prop(node, "shield-version")}'
    return shield or 'no shield', {'sensors': sensors, 'limits': limits, 'legs': legs, 'safe_outputs': outputs}


def comparator_table(alloc):
    """C initialisers of the comparator trips, for safety_trips.c (one entry per comparator path)."""
    rows = []
    for l in alloc['limits']:
        for bound in ('high', 'low'):
            p = l['paths'][bound]
            if not p or p['kind'] != 'comparator':
                continue
            port, pin = p['mcu_pin'][1], int(p['mcu_pin'][2:])
            dac, ch = p['dac'].split('_CH')
            inmsel = 4 + p['dac_options'].index(p['dac'])   # RM0440 Table 197: 100 or 101
            rows.append(f'    {{ "{l["sensor"]}", {"1" if bound == "high" else "0"}, {p["comp"]}, {p["inpsel"]}, '
                        f'{p["flt"]}, {1 if p["polarity"] == "inverted" else 0}, {dac[3:]}, {ch}, '
                        f'{p["dac_code"] if p["dac_code"] is not None else 0}, \'{port}\', {pin}, {inmsel}, '
                        f'{float(l.get("gain") or 0.0)!r}f, {float(l.get("offset") or 0.0)!r}f }}, \\')
    return ('/* sensor, high bound?, comparator, INPSEL, FLT, inverted, DAC n, DAC channel, DAC code, port, pin,\n'
            ' * INMSEL, sensor gain, sensor offset (value = counts * gain + offset) */\n'
            f'#define SAFETY_COMPARATOR_TRIPS {len(rows)}\n'
            '#define SAFETY_COMPARATOR_TABLE \\\n' + '\n'.join(rows) + '\n\n')


def watchdog_table(alloc):
    """C initialisers of the ADC-watchdog trips, for safety_trips.c (one entry per watchdog; a window may cover
    both bounds of a limit)."""
    rows, seen = [], set()
    for l in alloc['limits']:
        for bound in ('low', 'high'):
            p = l['paths'][bound]
            if not p or p['kind'] != 'adc_watchdog' or (p['adc'], p['awd']) in seen:
                continue
            seen.add((p['adc'], p['awd']))
            current = l['sensor'].upper().startswith('I')
            band = l.get('idle_band') if l.get('idle_band') is not None else \
                0.25 * max(abs(l['high'] or 0.0), abs(l['low'] or 0.0))
            rows.append(f'    {{ "{l["sensor"]}", {p["adc"][-1]}, {p["awd"]}, {p["channel"]}, {p["flt"]}, '
                        f'{p["counts_low"]}, {p["counts_high"]}, {float(l.get("gain") or 0.0)!r}f, '
                        f'{float(l.get("offset") or 0.0)!r}f, {float(band)!r}f, {1 if current else 0} }}, \\')
    return ('/* sensor, ADC n, watchdog k, channel, FLT (= EEV), window low, window high (12 bits for AWD1, 8 for\n'
            ' * AWD2/AWD3; a side without hardware protection is open: 0 or full scale), sensor gain, offset, idle\n'
            ' * band (current sensors: largest reading allowed at idle), current? */\n'
            f'#define SAFETY_WATCHDOG_TRIPS {len(rows)}\n'
            '#define SAFETY_WATCHDOG_TABLE \\\n' + '\n'.join(rows) + '\n\n')


FAULT_INACTIVE, FAULT_ACTIVE = 2, 1   # HRTIM OUTxR FAULTx: 10 inactive, 01 active


def leg_table(alloc):
    """C initialisers of each leg's safe state: HRTIM timer index (pwma = 0 ... pwmf = 5) and the fault level of
    its two outputs. short-low turns the low-side switch on: OUT2 if PWMx1 drives the high side, else OUT1."""
    rows = []
    for g in alloc['legs']:
        t = g.get('timer') or ''
        if not (t.startswith('pwm') and len(t) == 4):
            continue
        idx = ord(t[3]) - ord('a')
        state = g.get('safe_state', 'open')
        f1 = f2 = FAULT_INACTIVE
        x1_high = bool(g.get('pwm_x1_high', 1))
        if state == 'short-low':
            f1, f2 = (FAULT_INACTIVE, FAULT_ACTIVE) if x1_high else (FAULT_ACTIVE, FAULT_INACTIVE)
        elif state == 'short-high':
            f1, f2 = (FAULT_ACTIVE, FAULT_INACTIVE) if x1_high else (FAULT_INACTIVE, FAULT_ACTIVE)
        rows.append(f'    {{ "{g["name"]}", {idx}, {f1}, {f2}, {2 if x1_high else 1} }}, \\')
    return ('/* leg, HRTIM timer index, fault level OUT1, fault level OUT2 (2 inactive, 1 active), low-side output */\n'
            f'#define SAFETY_LEGS {len(rows)}\n#define SAFETY_LEG_TABLE \\\n' + '\n'.join(rows) + '\n\n')


def safe_output_table(alloc):
    """C initialisers of the safe outputs: port, pin, safe level."""
    rows = []
    for o in alloc['safe_outputs']:
        mp = o.get('mcu_pin')
        if not mp:
            continue
        rows.append(f'    {{ "{o["name"]}", \'{mp[1]}\', {int(mp[2:])}, {o.get("safe_level", 0)} }}, \\')
    return ('/* safe output, port, pin, safe level */\n'
            f'#define SAFETY_SAFE_OUTPUTS {len(rows)}\n#define SAFETY_SAFE_OUTPUT_TABLE \\\n'
            + '\n'.join(rows) + '\n\n')


def safety_mode(edt):
    """safety-mode of the app's safety-config node: "production" when absent (ADR 0009, #22)."""
    nodes = edt.compat2okay.get('safety-config', [])
    if not nodes:
        return 'production'
    p = nodes[0].props.get('safety-mode')
    return p.val if p is not None else 'production'


def plan_text(alloc):
    """One-line plan for the boot line: LIMIT low/high path kinds."""
    short = {'comparator': 'comp', 'adc_watchdog': 'awd', 'software': 'sw'}
    parts = []
    for l in alloc['limits']:
        lo, hi = l['paths']['low'], l['paths']['high']
        parts.append(f'{l["sensor"]} {short.get(lo["kind"], "?") if lo else "none"}-/'
                     f'{short.get(hi["kind"], "?") if hi else "none"}+')
    return ', '.join(parts) or 'no limits declared'


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--edt', required=True)
    ap.add_argument('--zephyr-base', required=True)
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--header')
    ap.add_argument('--report-only', action='store_true', help='never fail (design-time check)')
    a = ap.parse_args()
    sys.path.insert(0, os.path.join(a.zephyr_base, 'scripts', 'dts', 'python-devicetree', 'src'))
    with open(a.edt, 'rb') as f:
        edt = pickle.load(f)
    shield, model = build_model(edt)
    alloc = allocate(model)
    by_sensor = {m['sensor']: m for m in model['limits']}
    for l in alloc['limits']:   # sensor scaling and idle band, for the firmware's checks
        sn = model['sensors'].get(l['sensor'], {})
        l['gain'], l['offset'] = sn.get('gain'), sn.get('offset')
        l['idle_band'] = by_sensor.get(l['sensor'], {}).get('idle_band')
    mode = safety_mode(edt)
    os.makedirs(a.out_dir, exist_ok=True)
    manifest = {'layout': 1, 'shield': shield, 'safety_mode': mode, 'model': model, **alloc}
    with open(os.path.join(a.out_dir, 'protection-manifest.json'), 'w') as f:
        json.dump(manifest, f, indent=1)
    with open(os.path.join(a.out_dir, 'protection-table.md'), 'w') as f:
        f.write(markdown_table(shield, alloc))
        if mode == 'development':
            f.write('\n**Development mode** (`safety-mode = "development"`): not for a deployed converter.\n')
    if a.header:
        os.makedirs(os.path.dirname(a.header), exist_ok=True)
        text = plan_text(alloc).replace('"', "'")
        with open(a.header, 'w') as f:
            f.write('/* Generated by protection_manifest.py (ADR 0009). Do not edit. */\n'
                    '#ifndef SAFETY_PROTECTION_PLAN_H_\n#define SAFETY_PROTECTION_PLAN_H_\n'
                    f'#define SAFETY_PROTECTION_SHIELD "{shield}"\n'
                    f'#define SAFETY_PROTECTION_PLAN "{text}"\n'
                    f'#define SAFETY_PROTECTION_LIMITS {len(alloc["limits"])}\n'
                    f'#define SAFETY_PROTECTION_MODE "{mode}"\n'
                    + comparator_table(alloc) + watchdog_table(alloc) + leg_table(alloc)
                    + safe_output_table(alloc) + '#endif\n')
    print(f'safety: protection plan for {shield} ({mode}): {plan_text(alloc)}')
    if mode == 'development':
        print('WARNING: safety-mode = "development": disarm() and loosened software limits are possible. '
              'Never flash this build on a deployed converter.')
    for l in alloc['limits']:
        print(f'safety:   {l["sensor"]:12s} low {path_text(l["paths"]["low"]):24s} high {path_text(l["paths"]["high"])}')
    for w in alloc['warnings']:
        print('WARNING: ' + w)
    for e in alloc['errors']:
        print('ERROR: ' + e)
    return 1 if alloc['errors'] and not a.report_only else 0


if __name__ == '__main__':
    sys.exit(main())
