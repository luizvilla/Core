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
        sensors[name] = {'gain': f32(gain) if gain is not None else None,
                         'offset': f32(offset) if offset is not None else 0.0,
                         'conv': prop(node, 'sensor-conv-type', 'LINEAR'), 'channels': chans}
    limits = []
    for node in edt.compat2okay.get('safety-thresholds', []):
        limits.append({'sensor': prop(node, 'sensor-name'), 'name': prop(node, 'threshold-name'),
                       'high': f32(prop(node, 'threshold-high')), 'low': f32(prop(node, 'threshold-low')),
                       'protection': prop(node, 'protection', 'hardware'), 'exception': prop(node, 'exception'),
                       'comparator_bound': prop(node, 'comparator-bound', 'high'),
                       'priority': prop(node, 'priority')})
    by_pin = {ch['spin_pin']: n for n, s in sensors.items() for ch in s['channels']}
    legs = []
    for node in edt.compat2okay.get('power-leg', []):
        for child in node.children.values():
            pw = prop(child, 'pwms') or []
            timer = pw[0].controller.labels[0] if pw and pw[0].controller.labels else None
            cur = prop(child, 'current-pin-num')
            legs.append({'name': prop(child, 'leg-name'), 'timer': timer,
                         'safe_state': prop(child, 'safe-state', 'open'),
                         'current_sensor': by_pin.get(cur), 'short_risk_accepted': prop(child, 'short-risk-accepted')})
    outputs = []
    for node in edt.compat2okay.get('safety-outputs', []):
        for child in node.children.values():
            g = (prop(child, 'gpios') or [None])[0]
            sp = g.data.get('pin') if g is not None else None
            outputs.append({'name': prop(child, 'output-name', child.name), 'spin_pin': sp, 'mcu_pin': pins.get(sp),
                            'safe_level': prop(child, 'safe-level', 0),
                            'reset_state_safe': bool(prop(child, 'reset-state-safe', False))})
    shield = None
    for node in edt.compat2okay.get('shield-pcb', []):
        shield = f'{prop(node, "shield-name")} {prop(node, "shield-version")}'
    return shield or 'no shield', {'sensors': sensors, 'limits': limits, 'legs': legs, 'safe_outputs': outputs}


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
    os.makedirs(a.out_dir, exist_ok=True)
    manifest = {'layout': 1, 'shield': shield, 'model': model, **alloc}
    with open(os.path.join(a.out_dir, 'protection-manifest.json'), 'w') as f:
        json.dump(manifest, f, indent=1)
    with open(os.path.join(a.out_dir, 'protection-table.md'), 'w') as f:
        f.write(markdown_table(shield, alloc))
    if a.header:
        os.makedirs(os.path.dirname(a.header), exist_ok=True)
        text = plan_text(alloc).replace('"', "'")
        with open(a.header, 'w') as f:
            f.write('/* Generated by protection_manifest.py (ADR 0009). Do not edit. */\n'
                    '#ifndef SAFETY_PROTECTION_PLAN_H_\n#define SAFETY_PROTECTION_PLAN_H_\n'
                    f'#define SAFETY_PROTECTION_SHIELD "{shield}"\n'
                    f'#define SAFETY_PROTECTION_PLAN "{text}"\n'
                    f'#define SAFETY_PROTECTION_LIMITS {len(alloc["limits"])}\n#endif\n')
    print(f'safety: protection plan for {shield}: {plan_text(alloc)}')
    for l in alloc['limits']:
        print(f'safety:   {l["sensor"]:12s} low {path_text(l["paths"]["low"]):24s} high {path_text(l["paths"]["high"])}')
    for w in alloc['warnings']:
        print('WARNING: ' + w)
    for e in alloc['errors']:
        print('ERROR: ' + e)
    return 1 if alloc['errors'] and not a.report_only else 0


if __name__ == '__main__':
    sys.exit(main())
